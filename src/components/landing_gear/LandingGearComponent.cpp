#include "navion/components/landing_gear/LandingGearComponent.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace navion::landing_gear
{
namespace
{

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double MINIMUM_DIRECTION_NORM = 1.0e-9;
constexpr double MINIMUM_STRUT_PROJECTION = 1.0e-6;

[[nodiscard]] bool finiteAndNonnegative(double value) noexcept
{
    return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] Vec3 normalizedOr(const Vec3& value, const Vec3& fallback)
{
    const double magnitude = value.norm();
    return magnitude > MINIMUM_DIRECTION_NORM ? value / magnitude : fallback;
}

[[nodiscard]] double steeringAngle(
    const GearParameters& gear,
    double requestedAngleRad
) noexcept
{
    if (gear.steeringMode == SteeringMode::Fixed)
    {
        return 0.0;
    }
    return std::clamp(
        requestedAngleRad,
        -gear.maximumSteeringAngleRad,
        gear.maximumSteeringAngleRad
    );
}

[[nodiscard]] double sideFrictionCoefficient(
    const GearParameters& gear,
    double wheelSlipDeg,
    const LandingGearParameters& parameters
) noexcept
{
    const double x = gear.pacejkaStiffnessB * wheelSlipDeg;
    const double coefficient = gear.staticFrictionCoefficient * std::sin(
        gear.pacejkaShapeC * std::atan(
            x - gear.pacejkaCurvatureE * (x - std::atan(x))
        )
    );
    return coefficient * parameters.staticFrictionFactor;
}

void validateContext(const EvaluationContext& context)
{
    if (!context.state.isFinite() || !context.controls.isFinite() ||
        !context.massProperties.inertiaBodyKgM2.isFinite() ||
        !std::isfinite(context.massProperties.massKg))
    {
        throw std::invalid_argument(
            "LandingGearComponent received an invalid EvaluationContext."
        );
    }
}

} // namespace

void GearParameters::validate() const
{
    if (name.empty() || !locationFromCgBodyM.isFinite() ||
        !std::isfinite(springStiffnessNpm) || springStiffnessNpm <= 0.0 ||
        !finiteAndNonnegative(dampingCoefficientNsPm) ||
        !finiteAndNonnegative(staticFrictionCoefficient) ||
        !finiteAndNonnegative(rollingFrictionCoefficient) ||
        !finiteAndNonnegative(maximumSteeringAngleRad) ||
        !std::isfinite(pacejkaStiffnessB) ||
        !std::isfinite(pacejkaShapeC) ||
        !std::isfinite(pacejkaCurvatureE))
    {
        throw std::invalid_argument("Landing-gear parameters are invalid.");
    }
}

void LandingGearParameters::validate() const
{
    if (solverMaximumIterations <= 0 ||
        !std::isfinite(solverConvergenceTolerance) ||
        solverConvergenceTolerance <= 0.0 ||
        !finiteAndNonnegative(rollingFrictionFactor) ||
        !finiteAndNonnegative(staticFrictionFactor) ||
        !finiteAndNonnegative(maximumStrutForceN) ||
        !std::isfinite(groundDownPositionNedM) || gears.empty())
    {
        throw std::invalid_argument("Landing-gear global parameters are invalid.");
    }

    std::unordered_set<std::string> names;
    for (const GearParameters& gear : gears)
    {
        gear.validate();
        if (!names.insert(gear.name).second)
        {
            throw std::invalid_argument("Landing-gear names must be unique.");
        }
    }
}

PGSFrictionSolver::PGSFrictionSolver(
    int maximumIterations,
    double convergenceTolerance
)
    : maximumIterations_(maximumIterations),
      convergenceTolerance_(convergenceTolerance)
{
    if (maximumIterations_ <= 0 ||
        !std::isfinite(convergenceTolerance_) ||
        convergenceTolerance_ <= 0.0)
    {
        throw std::invalid_argument("PGS solver settings are invalid.");
    }
}

FrictionSolveResult PGSFrictionSolver::solve(
    const std::vector<FrictionConstraint>& constraints,
    const BodyLoad& preFrictionLoadIncludingGravity,
    const RigidBodyState& state,
    const MassProperties& massProperties,
    double timeStepS,
    const Matrix3* inverseInertiaBodyKgM2
) const
{
    if (!preFrictionLoadIncludingGravity.isFinite() || !state.isFinite() ||
        !std::isfinite(timeStepS) || timeStepS < 0.0 ||
        !std::isfinite(massProperties.massKg) || massProperties.massKg <= 0.0 ||
        !massProperties.inertiaBodyKgM2.isFinite())
    {
        throw std::invalid_argument("PGS solver received invalid input.");
    }

    FrictionSolveResult result;
    const std::size_t count = constraints.size();
    if (count == 0U)
    {
        return result;
    }

    if (inverseInertiaBodyKgM2 != nullptr &&
        !inverseInertiaBodyKgM2->isFinite())
    {
        throw std::invalid_argument("PGS received a non-finite inverse inertia.");
    }
    const Matrix3 inertiaInverse = inverseInertiaBodyKgM2 != nullptr
        ? *inverseInertiaBodyKgM2
        : massProperties.inertiaBodyKgM2.inverse();
    const Vec3& velocity = state.velocityBodyMps;
    const Vec3& omega = state.angularRateBodyRadps;
    const Vec3 velocityDerivativePre =
        preFrictionLoadIncludingGravity.forceBodyN / massProperties.massKg -
        cross(omega, velocity);
    const Vec3 angularDerivativePre = inertiaInverse * (
        preFrictionLoadIncludingGravity.momentAboutCgBodyNm -
        cross(omega, massProperties.inertiaBodyKgM2 * omega)
    );
    const Vec3 rhsVelocity = timeStepS > 1.0e-9
        ? velocityDerivativePre + velocity / timeStepS
        : velocityDerivativePre;
    const Vec3 rhsAngular = timeStepS > 1.0e-9
        ? angularDerivativePre + omega / timeStepS
        : angularDerivativePre;

    std::vector<std::vector<double>> matrix(
        count,
        std::vector<double>(count, 0.0)
    );
    for (std::size_t i = 0U; i < count; ++i)
    {
        const Vec3 translationalResponse =
            constraints[i].directionBody / massProperties.massKg;
        const Vec3 rotationalResponse = inertiaInverse * cross(
            constraints[i].leverArmFromCgBodyM,
            constraints[i].directionBody
        );
        for (std::size_t j = 0U; j < count; ++j)
        {
            matrix[i][j] = dot(
                constraints[j].directionBody,
                translationalResponse + cross(
                    rotationalResponse,
                    constraints[j].leverArmFromCgBodyM
                )
            );
        }
    }

    std::vector<double> rhs(count, 0.0);
    for (std::size_t i = 0U; i < count; ++i)
    {
        const double diagonal = matrix[i][i];
        if (std::abs(diagonal) < 1.0e-12)
        {
            continue;
        }
        rhs[i] = -dot(
            constraints[i].directionBody,
            rhsVelocity + cross(
                rhsAngular,
                constraints[i].leverArmFromCgBodyM
            )
        ) / diagonal;
        for (std::size_t j = 0U; j < count; ++j)
        {
            matrix[i][j] /= diagonal;
        }
    }

    std::vector<double> multipliers(count, 0.0);
    for (std::size_t i = 0U; i < count; ++i)
    {
        multipliers[i] = std::clamp(
            constraints[i].warmStartForceN,
            constraints[i].minimumForceN,
            constraints[i].maximumForceN
        );
    }

    result.converged = false;
    for (int iteration = 0; iteration < maximumIterations_; ++iteration)
    {
        double changeNorm = 0.0;
        for (std::size_t i = 0U; i < count; ++i)
        {
            double sum = 0.0;
            for (std::size_t j = 0U; j < count; ++j)
            {
                sum += matrix[i][j] * multipliers[j];
            }
            const double previous = multipliers[i];
            multipliers[i] = std::clamp(
                previous + rhs[i] - sum,
                constraints[i].minimumForceN,
                constraints[i].maximumForceN
            );
            changeNorm += std::abs(multipliers[i] - previous);
        }
        result.iterations = static_cast<std::size_t>(iteration + 1);
        if (changeNorm < convergenceTolerance_)
        {
            result.converged = true;
            break;
        }
    }

    for (std::size_t i = 0U; i < count; ++i)
    {
        const Vec3 force = multipliers[i] * constraints[i].directionBody;
        result.frictionLoad.forceBodyN += force;
        result.frictionLoad.momentAboutCgBodyNm += cross(
            constraints[i].leverArmFromCgBodyM,
            force
        );
    }
    result.multipliersN = std::move(multipliers);
    return result;
}

LandingGearComponent::LandingGearComponent(
    LandingGearParameters parameters
)
    : parameters_(std::move(parameters)),
      solver_(
          parameters_.solverMaximumIterations,
          parameters_.solverConvergenceTolerance
      ),
      committedHistory_(parameters_.gears.size())
{
    parameters_.validate();
}

GroundContactEvaluation LandingGearComponent::evaluateContacts(
    const EvaluationContext& context,
    double timeStepS
) const
{
    validateContext(context);
    if (!std::isfinite(timeStepS) || timeStepS < 0.0)
    {
        throw std::invalid_argument("Landing Gear requires finite dt >= 0.");
    }

    GroundContactEvaluation output;
    output.contacts.reserve(parameters_.gears.size());
    output.frictionConstraints.reserve(parameters_.gears.size() * 2U);

    for (std::size_t index = 0U; index < parameters_.gears.size(); ++index)
    {
        GroundContactPoint contact = evaluateGear(
            parameters_.gears[index],
            committedHistory_[index],
            context,
            timeStepS
        );
        if (contact.weightOnWheels)
        {
            const Vec3 normalForce =
                contact.normalForceN * contact.normalDirectionBody;
            output.normalLoad.forceBodyN += normalForce;
            output.normalLoad.momentAboutCgBodyNm += cross(
                contact.contactPositionFromCgBodyM,
                normalForce
            );

            const GearParameters& gear = parameters_.gears[index];
            double rollingCoefficient =
                parameters_.rollingFrictionFactor *
                gear.rollingFrictionCoefficient;
            if (gear.brakeGroup != BrakeGroup::None)
            {
                rollingCoefficient +=
                    brakeCommand(gear.brakeGroup, context.controls) *
                    parameters_.staticFrictionFactor *
                    (gear.staticFrictionCoefficient -
                     gear.rollingFrictionCoefficient);
            }
            const double lateralCoefficient = sideFrictionCoefficient(
                gear,
                contact.wheelSlipDeg,
                parameters_
            );

            const GearHistory& history = committedHistory_[index];
            const double rollingLimit = std::abs(
                rollingCoefficient * contact.normalForceN
            );
            const double lateralLimit = std::abs(
                lateralCoefficient * contact.normalForceN
            );
            output.frictionConstraints.push_back({
                contact.rollingDirectionBody,
                contact.contactPositionFromCgBodyM,
                -rollingLimit,
                rollingLimit,
                std::clamp(
                    history.previousRollingMultiplierN,
                    -rollingLimit,
                    rollingLimit
                ),
                index,
                FrictionAxis::Rolling
            });
            output.frictionConstraints.push_back({
                contact.lateralDirectionBody,
                contact.contactPositionFromCgBodyM,
                -lateralLimit,
                lateralLimit,
                std::clamp(
                    history.previousLateralMultiplierN,
                    -lateralLimit,
                    lateralLimit
                ),
                index,
                FrictionAxis::Lateral
            });
        }
        output.contacts.push_back(std::move(contact));
    }
    output.totalLoad = output.normalLoad;
    return output;
}

FrictionSolveResult LandingGearComponent::solveFriction(
    const GroundContactEvaluation& contacts,
    const BodyLoad& preFrictionLoadIncludingGravity,
    const EvaluationContext& context,
    double timeStepS
) const
{
    return solver_.solve(
        contacts.frictionConstraints,
        preFrictionLoadIncludingGravity,
        context.state,
        context.massProperties,
        timeStepS,
        context.inverseInertiaBodyKgM2
    );
}

void LandingGearComponent::applyFrictionResult(
    GroundContactEvaluation& contacts,
    const FrictionSolveResult& friction
) const
{
    if (friction.multipliersN.size() != contacts.frictionConstraints.size())
    {
        throw std::invalid_argument(
            "Friction result does not match the contact constraint set."
        );
    }
    contacts.totalLoad = contacts.normalLoad + friction.frictionLoad;
    for (GroundContactPoint& contact : contacts.contacts)
    {
        contact.rollingFrictionN = 0.0;
        contact.lateralFrictionN = 0.0;
    }
    for (std::size_t index = 0U;
         index < contacts.frictionConstraints.size();
         ++index)
    {
        const FrictionConstraint& constraint =
            contacts.frictionConstraints[index];
        GroundContactPoint& contact =
            contacts.contacts.at(constraint.owningContactIndex);
        if (constraint.axis == FrictionAxis::Rolling)
        {
            contact.rollingFrictionN = friction.multipliersN[index];
        }
        else
        {
            contact.lateralFrictionN = friction.multipliersN[index];
        }
    }
}

void LandingGearComponent::commitAcceptedStep(
    const GroundContactEvaluation& contacts,
    const FrictionSolveResult& friction
)
{
    if (contacts.contacts.size() != committedHistory_.size() ||
        friction.multipliersN.size() != contacts.frictionConstraints.size())
    {
        throw std::invalid_argument("Cannot commit mismatched ground-contact data.");
    }

    for (std::size_t index = 0U; index < contacts.contacts.size(); ++index)
    {
        const GroundContactPoint& contact = contacts.contacts[index];
        GearHistory& history = committedHistory_[index];
        history.weightOnWheels = contact.weightOnWheels;
        history.compressionM = contact.compressionM;
        history.normalForceN = contact.normalForceN;
        history.steeringAngleRad = contact.steeringAngleRad;
        history.wheelSlipDeg = contact.wheelSlipDeg;
        history.rollingDirectionBody = contact.rollingDirectionBody;
        history.lateralDirectionBody = contact.lateralDirectionBody;
        history.normalDirectionBody = contact.normalDirectionBody;
        history.contactPositionFromCgBodyM =
            contact.contactPositionFromCgBodyM;
    }
    for (std::size_t index = 0U;
         index < contacts.frictionConstraints.size();
         ++index)
    {
        const FrictionConstraint& constraint =
            contacts.frictionConstraints[index];
        GearHistory& history =
            committedHistory_.at(constraint.owningContactIndex);
        if (constraint.axis == FrictionAxis::Rolling)
        {
            history.previousRollingMultiplierN = friction.multipliersN[index];
        }
        else
        {
            history.previousLateralMultiplierN = friction.multipliersN[index];
        }
    }
}

void LandingGearComponent::resetHistory() noexcept
{
    for (GearHistory& history : committedHistory_)
    {
        history = {};
    }
}

std::string_view LandingGearComponent::name() const noexcept
{
    return "LandingGearPGS";
}

const LandingGearParameters& LandingGearComponent::parameters() const noexcept
{
    return parameters_;
}

GroundContactPoint LandingGearComponent::evaluateGear(
    const GearParameters& gear,
    const GearHistory& history,
    const EvaluationContext& context,
    double timeStepS
) const
{
    GroundContactPoint result;
    result.name = gear.name;
    result.wheelSlipDeg = history.wheelSlipDeg;
    result.rollingDirectionBody = history.rollingDirectionBody;
    result.lateralDirectionBody = history.lateralDirectionBody;
    result.normalDirectionBody = history.normalDirectionBody;
    result.contactPositionFromCgBodyM = gear.locationFromCgBodyM;
    result.steeringAngleRad = steeringAngle(
        gear,
        context.controls.noseWheelSteeringRad
    );

    const Quaternion attitude = context.state.attitudeBodyToNed.normalized();
    const Vec3 groundNormalBody =
        attitude.conjugate().rotate({0.0, 0.0, -1.0});
    result.normalDirectionBody = groundNormalBody;

    if (!context.controls.landingGearExtended)
    {
        return result;
    }

    const Vec3 wheelPositionNed =
        context.state.positionNedM +
        attitude.rotate(gear.locationFromCgBodyM);
    const double heightAboveGroundM =
        parameters_.groundDownPositionNedM - wheelPositionNed.z;
    result.weightOnWheels = heightAboveGroundM < 0.0;
    if (!result.weightOnWheels)
    {
        return result;
    }

    const double strutProjection = std::max(
        MINIMUM_STRUT_PROJECTION,
        -groundNormalBody.z
    );
    double compressionM = std::max(
        0.0,
        -heightAboveGroundM / strutProjection
    );
    const Vec3 contactPosition =
        gear.locationFromCgBodyM + Vec3{0.0, 0.0, -compressionM};
    const Vec3 contactVelocity =
        context.state.velocityBodyMps +
        cross(context.state.angularRateBodyRadps, contactPosition);

    Vec3 rollingDirection{
        std::cos(result.steeringAngleRad),
        std::sin(result.steeringAngleRad),
        0.0
    };
    rollingDirection -= dot(rollingDirection, groundNormalBody) *
        groundNormalBody;
    rollingDirection = normalizedOr(
        rollingDirection,
        {1.0, 0.0, 0.0}
    );
    const Vec3 lateralDirection = normalizedOr(
        cross(groundNormalBody, rollingDirection),
        {0.0, 1.0, 0.0}
    );

    const double normalVelocity = dot(contactVelocity, groundNormalBody);
    double compressionRateMps = 0.0;
    if (timeStepS > 1.0e-9)
    {
        compressionRateMps = -normalVelocity / strutProjection;
        const double maximumCompressionSpeed = compressionM / timeStepS;
        if (std::abs(compressionRateMps) > maximumCompressionSpeed)
        {
            compressionRateMps = std::copysign(
                maximumCompressionSpeed,
                compressionRateMps
            );
        }
    }

    const double springForceN = -compressionM * gear.springStiffnessNpm;
    const double dampingForceN =
        -compressionRateMps * gear.dampingCoefficientNsPm;
    double strutForceN = std::min(springForceN + dampingForceN, 0.0);
    if (parameters_.maximumStrutForceN > 0.0 &&
        -strutForceN > parameters_.maximumStrutForceN)
    {
        strutForceN = -parameters_.maximumStrutForceN;
        compressionM = parameters_.maximumStrutForceN /
            gear.springStiffnessNpm;
    }

    result.compressionM = compressionM;
    result.normalForceN = std::max(0.0, -strutForceN / strutProjection);
    result.contactPositionFromCgBodyM = contactPosition;
    result.rollingDirectionBody = rollingDirection;
    result.lateralDirectionBody = lateralDirection;

    const double rollingVelocity = dot(contactVelocity, rollingDirection);
    const double lateralVelocity = dot(contactVelocity, lateralDirection);
    const double planarSpeed = std::hypot(rollingVelocity, lateralVelocity);
    if (planarSpeed > 3.0e-4)
    {
        result.wheelSlipDeg = -std::atan2(
            lateralVelocity,
            std::abs(rollingVelocity)
        ) * 180.0 / PI;
    }
    return result;
}

double LandingGearComponent::brakeCommand(
    BrakeGroup group,
    const ControlInputs& controls
) const noexcept
{
    switch (group)
    {
    case BrakeGroup::Left:
        return std::clamp(controls.brakeLeft, 0.0, 1.0);
    case BrakeGroup::Right:
        return std::clamp(controls.brakeRight, 0.0, 1.0);
    case BrakeGroup::None:
    default:
        return 0.0;
    }
}

LandingGearParameters makeT6cReferenceLandingGearParameters()
{
    LandingGearParameters parameters;
    parameters.solverMaximumIterations = 50;
    parameters.solverConvergenceTolerance = 1.0e-5;
    parameters.rollingFrictionFactor = 1.0;
    parameters.staticFrictionFactor = 1.0;
    parameters.maximumStrutForceN = 0.0;

    GearParameters nose;
    nose.name = "NOSE_LG";
    nose.locationFromCgBodyM = {2.844799, 0.0, 2.032001};
    nose.springStiffnessNpm = 58375.611749;
    nose.dampingCoefficientNsPm = 43781.708812;
    nose.staticFrictionCoefficient = 0.5;
    nose.rollingFrictionCoefficient = 0.02;
    nose.maximumSteeringAngleRad = 0.1221730476;
    nose.steeringMode = SteeringMode::Steered;
    nose.brakeGroup = BrakeGroup::None;

    GearParameters left;
    left.name = "LEFT_MLG";
    left.locationFromCgBodyM = {-0.731520, -2.067559, 2.032001};
    left.springStiffnessNpm = 72969.514687;
    left.dampingCoefficientNsPm = 116751.223499;
    left.staticFrictionCoefficient = 0.5;
    left.rollingFrictionCoefficient = 0.02;
    left.steeringMode = SteeringMode::Fixed;
    left.brakeGroup = BrakeGroup::Left;

    GearParameters right = left;
    right.name = "RIGHT_MLG";
    right.locationFromCgBodyM.y = 2.067559;
    right.brakeGroup = BrakeGroup::Right;

    parameters.gears = {nose, left, right};
    parameters.validate();
    return parameters;
}

MassProperties makeT6cReferenceMassProperties()
{
    // The uploaded YAML uses JSBSim's negated-Ixz convention. Its ixz value
    // is -1355.817948, therefore the symmetric tensor stores -ixz here.
    return {
        2267.965273,
        Matrix3({
            5716.535216, 0.0, 1355.817948,
            0.0, 10315.062951, 0.0,
            1355.817948, 0.0, 14039.494855
        })
    };
}

} // namespace navion::landing_gear
