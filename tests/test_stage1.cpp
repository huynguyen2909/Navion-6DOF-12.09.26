#include "navion/components/ILoadComponent.hpp"
#include "navion/core/FlightTypes.hpp"
#include "navion/core/MathTypes.hpp"
#include "navion/dynamics/LoadAccumulator.hpp"
#include "navion/dynamics/RigidBody6DOF.hpp"
#include "navion/integration/RK4Integrator.hpp"
#include "navion/model/NavionModel.hpp"

#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
)
{
    if (!std::isfinite(actual) ||
        std::abs(actual - expected) > tolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected)
        );
    }
}

void requireVecNear(
    const navion::Vec3& actual,
    const navion::Vec3& expected,
    double tolerance,
    const std::string& message
)
{
    requireNear(actual.x, expected.x, tolerance, message + " [x]");
    requireNear(actual.y, expected.y, tolerance, message + " [y]");
    requireNear(actual.z, expected.z, tolerance, message + " [z]");
}

template<typename Function>
void requireThrows(Function&& function, const std::string& message)
{
    bool threw = false;
    try
    {
        function();
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    require(threw, message);
}

navion::MassProperties simpleMassProperties(double massKg = 2.0)
{
    return {
        massKg,
        navion::Matrix3::diagonal(2.0, 3.0, 4.0)
    };
}

class ConstantLoadComponent final : public navion::ILoadComponent
{
public:
    explicit ConstantLoadComponent(navion::BodyLoad load)
        : load_(load)
    {
    }

    navion::BodyLoad computeLoad(
        const navion::EvaluationContext& context
    ) const override
    {
        (void)context;
        ++callCount_;
        return load_;
    }

    std::string_view name() const noexcept override
    {
        return "ConstantLoad";
    }

    [[nodiscard]] std::size_t callCount() const noexcept
    {
        return callCount_;
    }

private:
    navion::BodyLoad load_{};
    mutable std::size_t callCount_{0U};
};

void testVectorAndMatrixMath()
{
    using navion::Matrix3;
    using navion::Vec3;

    requireVecNear(
        navion::cross({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}),
        {0.0, 0.0, 1.0},
        1.0e-14,
        "Right-handed cross product"
    );

    const Matrix3 matrix({
        4.0, 1.0, 0.0,
        1.0, 3.0, 0.0,
        0.0, 0.0, 2.0
    });
    const Vec3 value{2.0, -1.0, 3.0};
    const Vec3 recovered = matrix.inverse() * (matrix * value);
    requireVecNear(recovered, value, 1.0e-12, "Matrix inverse");
}

void testQuaternionConvention()
{
    const double halfYaw = 0.25 * PI;
    const navion::Quaternion yaw90{
        std::cos(halfYaw),
        0.0,
        0.0,
        std::sin(halfYaw)
    };

    requireVecNear(
        yaw90.rotate({1.0, 0.0, 0.0}),
        {0.0, 1.0, 0.0},
        1.0e-12,
        "q_nb positive yaw must rotate BODY +X toward NED +East"
    );
}

void testLoadAccumulator()
{
    navion::LoadAccumulator accumulator;
    accumulator.add({{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
    accumulator.add({{-2.0, 1.0, 0.5}, {2.0, -5.0, 4.0}});

    requireVecNear(
        accumulator.total().forceBodyN,
        {-1.0, 3.0, 3.5},
        1.0e-14,
        "Accumulated force"
    );
    requireVecNear(
        accumulator.total().momentAboutCgBodyNm,
        {6.0, 0.0, 10.0},
        1.0e-14,
        "Accumulated moment"
    );
    require(
        accumulator.contributingLoadCount() == 2U,
        "Load count must be tracked."
    );

    accumulator.reset();
    requireVecNear(accumulator.total().forceBodyN, {}, 0.0, "Reset force");
    requireVecNear(accumulator.total().momentAboutCgBodyNm, {}, 0.0, "Reset moment");
    require(
        accumulator.contributingLoadCount() == 0U,
        "Reset must clear load count."
    );

    const double nan = std::numeric_limits<double>::quiet_NaN();
    requireThrows(
        [&accumulator, nan]() {
            accumulator.add({{nan, 0.0, 0.0}, {}});
        },
        "Accumulator must reject non-finite loads."
    );

    const navion::BodyLoad pointLoad = navion::makeBodyLoadAtPoint(
        {0.0, 10.0, 0.0},
        {0.0, 0.0, 2.0},
        {-3.0, 0.0, 0.0}
    );
    requireVecNear(
        pointLoad.momentAboutCgBodyNm,
        {0.0, 0.0, -28.0},
        1.0e-14,
        "Moment transfer from load point to CG"
    );
}

void testRigidBodyTranslationAndGravity()
{
    const navion::RigidBody6DOF rigidBody(simpleMassProperties());

    navion::RigidBodyState state;
    navion::Environment environment;
    const navion::BodyLoad load{{4.0, 0.0, 0.0}, {}};

    const navion::StateDerivative derivative =
        rigidBody.evaluate(state, load, environment.gravityNedMps2);

    requireVecNear(
        derivative.velocityRateBodyMps2,
        {2.0, 0.0, 9.80665},
        1.0e-12,
        "Newton translation plus NED gravity"
    );
}

void testRigidBodyRotatingFrameTerm()
{
    const navion::RigidBody6DOF rigidBody(simpleMassProperties());
    navion::RigidBodyState state;
    state.velocityBodyMps = {10.0, 0.0, 0.0};
    state.angularRateBodyRadps = {0.0, 0.0, 1.0};

    navion::Environment environment;
    environment.gravityNedMps2 = {};

    const navion::StateDerivative derivative =
        rigidBody.evaluate(state, {}, environment.gravityNedMps2);

    requireVecNear(
        derivative.velocityRateBodyMps2,
        {0.0, -10.0, 0.0},
        1.0e-12,
        "BODY rotating-frame term -omega cross velocity"
    );
}

void testEulerAngularCoupling()
{
    const navion::RigidBody6DOF rigidBody(simpleMassProperties());
    navion::RigidBodyState state;
    state.angularRateBodyRadps = {1.0, 2.0, 3.0};

    navion::Environment environment;
    environment.gravityNedMps2 = {};

    const navion::StateDerivative derivative =
        rigidBody.evaluate(state, {}, environment.gravityNedMps2);

    requireVecNear(
        derivative.angularAccelerationBodyRadps2,
        {-3.0, 2.0, -0.5},
        1.0e-12,
        "Euler angular coupling"
    );
}

void testFlightConditionAndNavionModel()
{
    navion::NavionModel model(simpleMassProperties());

    auto component = std::make_unique<ConstantLoadComponent>(
        navion::BodyLoad{{4.0, 0.0, 0.0}, {}}
    );
    ConstantLoadComponent* componentObserver = component.get();
    model.addLoadComponent(std::move(component));

    navion::RigidBodyState state;
    state.velocityBodyMps = {10.0, 0.0, 0.0};

    navion::Environment environment;
    environment.windVelocityNedMps = {2.0, 0.0, 0.0};
    environment.gravityNedMps2 = {};
    environment.speedOfSoundMps = 320.0;

    const navion::ModelEvaluation evaluation =
        model.evaluate(0.0, state, {}, environment);

    requireVecNear(
        evaluation.flightCondition.airRelativeVelocityBodyMps,
        {8.0, 0.0, 0.0},
        1.0e-12,
        "Air-relative velocity"
    );
    requireNear(
        evaluation.flightCondition.airspeedMps,
        8.0,
        1.0e-12,
        "Airspeed"
    );
    requireNear(
        evaluation.flightCondition.mach,
        0.025,
        1.0e-12,
        "Mach"
    );
    requireVecNear(
        evaluation.stateDerivative.velocityRateBodyMps2,
        {2.0, 0.0, 0.0},
        1.0e-12,
        "NavionModel component force to rigid body"
    );
    require(evaluation.contributingComponentCount == 1U, "Component count");
    require(componentObserver->callCount() == 1U, "Component evaluation count");
}

void testRk4CallsEveryStageAndIntegratesConstantForce()
{
    navion::NavionModel model(simpleMassProperties());
    auto component = std::make_unique<ConstantLoadComponent>(
        navion::BodyLoad{{4.0, 0.0, 0.0}, {}}
    );
    ConstantLoadComponent* componentObserver = component.get();
    model.addLoadComponent(std::move(component));

    navion::Environment environment;
    environment.gravityNedMps2 = {};
    navion::ControlInputs controls;

    navion::RigidBodyState state;
    navion::RK4Integrator integrator;

    constexpr double dt = 0.01;
    constexpr std::size_t stepCount = 100U;
    double time = 0.0;

    for (std::size_t step = 0U; step < stepCount; ++step)
    {
        state = integrator.step(
            time,
            dt,
            state,
            [&model, &controls, &environment](
                double stageTime,
                const navion::RigidBodyState& stageState
            ) {
                return model.evaluateDerivative(
                    stageTime,
                    stageState,
                    controls,
                    environment
                );
            }
        );
        time += dt;
    }

    requireNear(state.velocityBodyMps.x, 2.0, 1.0e-11, "RK4 velocity");
    requireNear(state.positionNedM.x, 1.0, 1.0e-11, "RK4 position");
    require(
        componentObserver->callCount() == 4U * stepCount,
        "Every RK4 stage must recompute component loads."
    );
    requireNear(
        state.attitudeBodyToNed.normSquared(),
        1.0,
        1.0e-13,
        "RK4 quaternion normalization"
    );
}

void testRk4QuaternionKinematics()
{
    navion::RigidBodyState state;
    state.angularRateBodyRadps = {0.0, 0.0, 0.5 * PI};

    navion::RK4Integrator integrator;
    constexpr double dt = 0.01;
    constexpr std::size_t stepCount = 100U;
    double time = 0.0;

    for (std::size_t step = 0U; step < stepCount; ++step)
    {
        state = integrator.step(
            time,
            dt,
            state,
            [](double, const navion::RigidBodyState& stageState) {
                navion::StateDerivative derivative;
                derivative.attitudeRate =
                    navion::quaternionDerivativeBodyToNed(
                        stageState.attitudeBodyToNed,
                        stageState.angularRateBodyRadps
                    );
                return derivative;
            }
        );
        time += dt;
    }

    requireVecNear(
        state.attitudeBodyToNed.rotate({1.0, 0.0, 0.0}),
        {0.0, 1.0, 0.0},
        2.0e-9,
        "RK4 positive yaw quaternion kinematics"
    );
}

void testMassPropertyValidation()
{
    requireThrows(
        []() {
            const navion::RigidBody6DOF invalid({
                -1.0,
                navion::Matrix3::identity()
            });
            (void)invalid;
        },
        "Negative mass must be rejected."
    );

    requireThrows(
        []() {
            const navion::RigidBody6DOF invalid({
                1.0,
                navion::Matrix3::diagonal(1.0, -1.0, 1.0)
            });
            (void)invalid;
        },
        "Non-positive-definite inertia must be rejected."
    );
}

} // namespace

int main()
{
    try
    {
        testVectorAndMatrixMath();
        testQuaternionConvention();
        testLoadAccumulator();
        testRigidBodyTranslationAndGravity();
        testRigidBodyRotatingFrameTerm();
        testEulerAngularCoupling();
        testFlightConditionAndNavionModel();
        testRk4CallsEveryStageAndIntegratesConstantForce();
        testRk4QuaternionKinematics();
        testMassPropertyValidation();
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }

    std::cout << "All Navion Stage 1 tests passed.\n";
    return 0;
}
