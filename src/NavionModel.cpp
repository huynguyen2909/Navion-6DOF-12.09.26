#include "navion/model/NavionModel.hpp"

#include "navion/dynamics/LoadAccumulator.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace navion
{
namespace
{

constexpr double MINIMUM_AIRSPEED_MPS = 1.0e-10;

void validateFlightEnvironment(const Environment& environment)
{
    if (!environment.isFinite() ||
        environment.airDensityKgM3 < 0.0 ||
        environment.dynamicViscosityPaS < 0.0 ||
        environment.speedOfSoundMps <= 0.0 ||
        !environment.gravityNedMps2.isFinite())
    {
        throw std::invalid_argument("NavionModel received an invalid environment.");
    }
}

} // namespace

struct NavionModel::CoupledEvaluation
{
    ModelEvaluation model{};
    std::optional<GroundContactEvaluation> ground{};
    std::optional<FrictionSolveResult> friction{};
};

NavionModel::NavionModel(const MassProperties& massProperties)
    : rigidBody_(massProperties)
{
}

void NavionModel::addLoadComponent(
    std::unique_ptr<ILoadComponent> component
)
{
    if (!component)
    {
        throw std::invalid_argument("Cannot add a null load component.");
    }
    loadComponents_.push_back(std::move(component));
}

void NavionModel::setGroundContactComponent(
    std::unique_ptr<IGroundContactComponent> component
)
{
    if (!component)
    {
        throw std::invalid_argument("Cannot set a null ground-contact component.");
    }
    if (groundContactComponent_)
    {
        throw std::logic_error("NavionModel already has a ground-contact component.");
    }
    groundContactComponent_ = std::move(component);
}

ModelEvaluation NavionModel::evaluate(
    double timeS,
    const RigidBodyState& state,
    const ControlInputs& controls,
    const Environment& environment
) const
{
    if (groundContactComponent_)
    {
        throw std::logic_error(
            "Ground contact requires evaluate(time, dt, state, controls, environment)."
        );
    }
    return evaluateCoupled(timeS, 0.0, state, controls, environment).model;
}

ModelEvaluation NavionModel::evaluate(
    double timeS,
    double timeStepS,
    const RigidBodyState& state,
    const ControlInputs& controls,
    const Environment& environment
) const
{
    if (!std::isfinite(timeStepS) || timeStepS <= 0.0)
    {
        throw std::invalid_argument("Coupled evaluation requires finite dt > 0.");
    }
    return evaluateCoupled(timeS, timeStepS, state, controls, environment).model;
}

StateDerivative NavionModel::evaluateDerivative(
    double timeS,
    const RigidBodyState& state,
    const ControlInputs& controls,
    const Environment& environment
) const
{
    return evaluate(timeS, state, controls, environment).stateDerivative;
}

StateDerivative NavionModel::evaluateDerivative(
    double timeS,
    double timeStepS,
    const RigidBodyState& state,
    const ControlInputs& controls,
    const Environment& environment
) const
{
    return evaluate(
        timeS,
        timeStepS,
        state,
        controls,
        environment
    ).stateDerivative;
}

void NavionModel::commitAcceptedStep(
    double timeS,
    double timeStepS,
    const RigidBodyState& acceptedState,
    const ControlInputs& controls,
    const Environment& environment
)
{
    if (!groundContactComponent_)
    {
        return;
    }
    if (!std::isfinite(timeStepS) || timeStepS <= 0.0)
    {
        throw std::invalid_argument("Ground-contact commit requires finite dt > 0.");
    }

    CoupledEvaluation accepted = evaluateCoupled(
        timeS,
        timeStepS,
        acceptedState,
        controls,
        environment
    );
    groundContactComponent_->commitAcceptedStep(
        *accepted.ground,
        *accepted.friction
    );
}

void NavionModel::resetGroundContactHistory() noexcept
{
    if (groundContactComponent_)
    {
        groundContactComponent_->resetHistory();
    }
}

std::size_t NavionModel::componentCount() const noexcept
{
    return loadComponents_.size() + (groundContactComponent_ ? 1U : 0U);
}

bool NavionModel::hasGroundContactComponent() const noexcept
{
    return static_cast<bool>(groundContactComponent_);
}

const MassProperties& NavionModel::massProperties() const noexcept
{
    return rigidBody_.massProperties();
}

NavionModel::CoupledEvaluation NavionModel::evaluateCoupled(
    double timeS,
    double timeStepS,
    const RigidBodyState& state,
    const ControlInputs& controls,
    const Environment& environment
) const
{
    if (!std::isfinite(timeS))
    {
        throw std::invalid_argument("Evaluation time must be finite.");
    }
    if (!state.isFinite())
    {
        throw std::invalid_argument("NavionModel received a non-finite state.");
    }
    if (!controls.isFinite())
    {
        throw std::invalid_argument("NavionModel received non-finite controls.");
    }

    const FlightCondition flightCondition =
        calculateFlightCondition(state, environment);
    const EvaluationContext context{
        timeS,
        state,
        controls,
        environment,
        flightCondition,
        rigidBody_.massProperties(),
        &rigidBody_.inverseInertiaBodyKgM2()
    };

    CoupledEvaluation output;
    output.model.flightCondition = flightCondition;
    output.model.componentLoads.reserve(
        loadComponents_.size() + (groundContactComponent_ ? 1U : 0U)
    );

    // A fresh accumulator is deliberately created at every derivative call,
    // including k1-k4. Loads therefore always correspond to that stage state.
    // The diagnostic report stores the exact same load that is added; it does
    // not re-evaluate a component or contribute to the sum a second time.
    LoadAccumulator accumulator;
    for (const auto& component : loadComponents_)
    {
        const BodyLoad load = component->computeLoad(context);
        accumulator.add(load);
        output.model.componentLoads.push_back({
            std::string(component->name()),
            load
        });
    }

    if (groundContactComponent_)
    {
        output.ground = groundContactComponent_->evaluateContacts(
            context,
            timeStepS
        );

        // PGS needs every pre-friction load. Gravity is temporarily included
        // here for the constraint RHS only; it remains excluded from BodyLoad
        // and is still added exactly once inside RigidBody6DOF.
        BodyLoad preFrictionLoad =
            accumulator.total() + output.ground->normalLoad;
        const Quaternion attitude = state.attitudeBodyToNed.normalized();
        const Vec3 gravityBodyMps2 =
            attitude.conjugate().rotate(environment.gravityNedMps2);
        preFrictionLoad.forceBodyN +=
            rigidBody_.massProperties().massKg * gravityBodyMps2;

        output.friction = groundContactComponent_->solveFriction(
            *output.ground,
            preFrictionLoad,
            context,
            timeStepS
        );
        groundContactComponent_->applyFrictionResult(
            *output.ground,
            *output.friction
        );
        accumulator.add(output.ground->totalLoad);
        output.model.componentLoads.push_back({
            std::string(groundContactComponent_->name()),
            output.ground->totalLoad
        });

        output.model.groundNormalLoad = output.ground->normalLoad;
        output.model.groundFrictionLoad = output.friction->frictionLoad;
        output.model.frictionSolverIterations = output.friction->iterations;
        output.model.frictionSolverConverged = output.friction->converged;
        output.model.groundContacts.reserve(output.ground->contacts.size());
        for (const GroundContactPoint& contact : output.ground->contacts)
        {
            if (contact.weightOnWheels)
            {
                ++output.model.groundContactCount;
            }
            output.model.groundContacts.push_back({
                contact.name,
                contact.weightOnWheels,
                contact.compressionM,
                contact.normalForceN,
                contact.rollingFrictionN,
                contact.lateralFrictionN,
                contact.wheelSlipDeg,
                contact.steeringAngleRad
            });
        }
    }

    output.model.totalComponentLoad = accumulator.total();
    output.model.contributingComponentCount =
        accumulator.contributingLoadCount();
    output.model.stateDerivative = rigidBody_.evaluate(
        state,
        output.model.totalComponentLoad,
        environment.gravityNedMps2
    );
    return output;
}

FlightCondition NavionModel::calculateFlightCondition(
    const RigidBodyState& state,
    const Environment& environment
)
{
    validateFlightEnvironment(environment);

    const Quaternion attitude = state.attitudeBodyToNed.normalized();
    const Vec3 windVelocityBodyMps =
        attitude.conjugate().rotate(environment.windVelocityNedMps);

    FlightCondition condition;
    condition.airRelativeVelocityBodyMps =
        state.velocityBodyMps - windVelocityBodyMps;
    condition.airspeedMps = condition.airRelativeVelocityBodyMps.norm();

    if (condition.airspeedMps > MINIMUM_AIRSPEED_MPS)
    {
        const Vec3& velocity = condition.airRelativeVelocityBodyMps;
        const double longitudinalSpeed = std::hypot(velocity.x, velocity.z);
        condition.angleOfAttackRad = std::atan2(velocity.z, velocity.x);
        condition.sideSlipRad = std::atan2(velocity.y, longitudinalSpeed);
    }

    condition.mach = condition.airspeedMps / environment.speedOfSoundMps;
    condition.dynamicPressurePa =
        0.5 * environment.airDensityKgM3 *
        condition.airspeedMps * condition.airspeedMps;

    if (!condition.isFinite())
    {
        throw std::runtime_error("NavionModel produced a non-finite flight condition.");
    }

    return condition;
}

} // namespace navion
