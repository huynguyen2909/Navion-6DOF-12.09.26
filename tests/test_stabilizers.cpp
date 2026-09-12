#include "navion/components/ILocalFlowField.hpp"
#include "navion/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "navion/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "navion/integration/RK4Integrator.hpp"
#include "navion/model/NavionModel.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

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
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
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

navion::HorizontalStabilizerConfig horizontalConfig()
{
    navion::HorizontalStabilizerConfig config;
    config.Area = 2.0;
    config.TailSpan = 4.0;
    config.TailMAC = 1.0;
    config.PositionWrtCG[0] = -4.0;
    config.ElevatorArea = 0.5;
    config.ElevatorMechanicalLimit[0] = -0.2;
    config.ElevatorMechanicalLimit[1] = 0.2;
    config.LiftCurveSlope = 1.0;
    config.ElevatorEffectiveness = 1.0;
    return config;
}

navion::VerticalStabilizerConfig verticalConfig()
{
    navion::VerticalStabilizerConfig config;
    config.Area = 2.0;
    config.RudderArea = 0.5;
    config.TailSpan = 2.0;
    config.TailMAC = 1.0;
    config.PositionWrtCG[0] = -4.0;
    config.PositionWrtCG[2] = -1.0;
    config.RudderMechanicalLimit[0] = -0.2;
    config.RudderMechanicalLimit[1] = 0.2;
    config.SideForceCurveSlope = 1.0;
    config.RudderEffectiveness = 1.0;
    return config;
}

struct ContextFixture
{
    navion::RigidBodyState state{};
    navion::ControlInputs controls{};
    navion::Environment environment{};
    navion::FlightCondition flightCondition{};
    navion::MassProperties massProperties{
        1000.0,
        navion::Matrix3::diagonal(1000.0, 1200.0, 1500.0)
    };

    navion::EvaluationContext context() const
    {
        return {
            0.0,
            state,
            controls,
            environment,
            flightCondition,
            massProperties
        };
    }

    void setAirRelativeVelocity(const navion::Vec3& velocityBodyMps)
    {
        flightCondition.airRelativeVelocityBodyMps = velocityBodyMps;
        flightCondition.airspeedMps = velocityBodyMps.norm();
        if (flightCondition.airspeedMps > 0.0)
        {
            flightCondition.angleOfAttackRad =
                std::atan2(velocityBodyMps.z, velocityBodyMps.x);
            flightCondition.sideSlipRad = std::atan2(
                velocityBodyMps.y,
                std::hypot(velocityBodyMps.x, velocityBodyMps.z)
            );
        }
        flightCondition.mach =
            flightCondition.airspeedMps / environment.speedOfSoundMps;
        flightCondition.dynamicPressurePa =
            0.5 * environment.airDensityKgM3 *
            flightCondition.airspeedMps * flightCondition.airspeedMps;
    }
};

class CountingLocalFlowField final : public navion::ILocalFlowField
{
public:
    explicit CountingLocalFlowField(navion::Vec3 incrementBodyMps)
        : incrementBodyMps_(incrementBodyMps)
    {
    }

    navion::Vec3 velocityIncrementBodyMps(
        const navion::EvaluationContext& context,
        const navion::Vec3& positionFromCgBodyM
    ) const override
    {
        (void)context;
        (void)positionFromCgBodyM;
        ++callCount_;
        return incrementBodyMps_;
    }

    [[nodiscard]] std::size_t callCount() const noexcept
    {
        return callCount_;
    }

private:
    navion::Vec3 incrementBodyMps_{};
    mutable std::size_t callCount_{0U};
};

void testHorizontalLoadAndMomentContract()
{
    navion::HorizontalStabilizer stabilizer(horizontalConfig());
    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 0.0, 0.0});
    fixture.controls.elevatorRad = 0.1;

    const navion::HorizontalStabilizerEvaluation result =
        stabilizer.evaluateDetailed(fixture.context());

    constexpr double expectedDynamicPressurePa = 61.25;
    constexpr double expectedLiftN = 12.25;
    requireNear(
        result.dynamicPressurePa,
        expectedDynamicPressurePa,
        1.0e-12,
        "Horizontal-tail local dynamic pressure"
    );
    requireNear(result.liftCoefficient, 0.1, 1.0e-14, "Horizontal-tail CL");
    requireVecNear(
        result.bodyLoad.forceBodyN,
        {0.0, 0.0, -expectedLiftN},
        1.0e-12,
        "Horizontal-tail BODY force"
    );
    requireVecNear(
        result.bodyLoad.momentAboutCgBodyNm,
        {0.0, -49.0, 0.0},
        1.0e-12,
        "Horizontal-tail CG moment includes r cross F exactly once"
    );

    fixture.controls.elevatorRad = 1.0;
    const auto clamped = stabilizer.evaluateDetailed(fixture.context());
    requireNear(
        clamped.elevatorDeflectionRad,
        0.2,
        1.0e-14,
        "Elevator mechanical clamp"
    );
}

void testHorizontalAngularRateAndIntrinsicMoment()
{
    auto config = horizontalConfig();
    config.LiftCurveSlope = 0.0;
    config.PitchMomentCoefficient = 0.5;
    navion::HorizontalStabilizer stabilizer(config);

    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 0.0, 0.0});
    fixture.state.angularRateBodyRadps = {0.0, 0.1, 0.0};

    const auto result = stabilizer.evaluateDetailed(fixture.context());
    requireVecNear(
        result.localVelocityBodyMps,
        {10.0, 0.0, 0.4},
        1.0e-14,
        "Horizontal-tail local velocity includes omega cross r"
    );
    require(
        result.intrinsicMomentAtAerodynamicCenterBodyNm.y > 0.0,
        "Positive intrinsic Cm must create positive BODY-Y moment."
    );
    requireNear(
        result.bodyLoad.momentAboutCgBodyNm.y,
        result.intrinsicMomentAtAerodynamicCenterBodyNm.y,
        1.0e-12,
        "Zero-force intrinsic pitch moment transfer"
    );
}

void testVerticalRestoringSigns()
{
    navion::VerticalStabilizer stabilizer(verticalConfig());
    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 1.0, 0.0});

    const auto betaResult = stabilizer.evaluateDetailed(fixture.context());
    require(
        betaResult.bodyLoad.forceBodyN.y < 0.0,
        "Positive beta must produce negative BODY-Y fin force."
    );
    require(
        betaResult.bodyLoad.momentAboutCgBodyNm.z > 0.0,
        "An aft fin must produce restoring positive yaw moment for positive beta."
    );

    fixture.setAirRelativeVelocity({10.0, 0.0, 0.0});
    fixture.controls.rudderRad = 0.1; // trailing edge left
    const auto rudderResult = stabilizer.evaluateDetailed(fixture.context());
    require(
        rudderResult.bodyLoad.forceBodyN.y > 0.0,
        "Positive rudder must produce positive BODY-Y force."
    );
    require(
        rudderResult.bodyLoad.momentAboutCgBodyNm.z < 0.0,
        "Positive rudder on an aft fin must produce nose-left yaw moment."
    );
}

void testVerticalDragUsesFullLocalVelocity()
{
    auto config = verticalConfig();
    config.SideForceCurveSlope = 0.0;
    config.ZeroLiftDragCoefficient = 0.1;
    navion::VerticalStabilizer stabilizer(config);

    ContextFixture fixture;
    fixture.setAirRelativeVelocity({10.0, 0.0, 2.0});
    const auto result = stabilizer.evaluateDetailed(fixture.context());

    require(result.bodyLoad.forceBodyN.x < 0.0, "Fin drag must oppose +u.");
    require(
        result.bodyLoad.forceBodyN.z < 0.0,
        "Fin drag must include and oppose the local +w component."
    );
}

void testLocalFlowFieldAndRk4StageEvaluation()
{
    auto flowField =
        std::make_shared<CountingLocalFlowField>(navion::Vec3{0.0, 0.0, 0.5});

    navion::NavionModel model({
        1000.0,
        navion::Matrix3::diagonal(1000.0, 1200.0, 1500.0)
    });
    model.addLoadComponent(
        std::make_unique<navion::HorizontalStabilizer>(
            horizontalConfig(),
            flowField
        )
    );
    model.addLoadComponent(
        std::make_unique<navion::VerticalStabilizer>(
            verticalConfig(),
            flowField
        )
    );

    navion::RigidBodyState state;
    state.velocityBodyMps = {10.0, 0.0, 0.0};
    navion::ControlInputs controls;
    navion::Environment environment;
    environment.gravityNedMps2 = {};

    navion::RK4Integrator integrator;
    state = integrator.step(
        0.0,
        0.01,
        state,
        [&model, &controls, &environment](
            double stageTimeS,
            const navion::RigidBodyState& stageState
        ) {
            return model.evaluateDerivative(
                stageTimeS,
                stageState,
                controls,
                environment
            );
        }
    );

    require(state.isFinite(), "Integrated state must remain finite.");
    require(
        flowField->callCount() == 8U,
        "Two stabilizers must recompute local flow at all four RK4 stages."
    );

    const navion::ModelEvaluation evaluation =
        model.evaluate(0.01, state, controls, environment);
    require(
        evaluation.contributingComponentCount == 2U,
        "NavionModel must accumulate one load from each stabilizer."
    );
    require(evaluation.totalComponentLoad.isFinite(), "Total load must be finite.");
}

void testAutomaticAspectRatioAndLimitOrdering()
{
    auto horizontal = horizontalConfig();
    horizontal.TailAR = 0.0;
    horizontal.ElevatorMechanicalLimit[0] = 0.2;
    horizontal.ElevatorMechanicalLimit[1] = -0.2;
    const navion::HorizontalStabilizer horizontalModel(horizontal);
    requireNear(
        horizontalModel.getConfig().TailAR,
        8.0,
        1.0e-14,
        "Horizontal-tail automatic aspect ratio"
    );
    require(
        horizontalModel.getConfig().ElevatorMechanicalLimit[0] == -0.2,
        "Elevator limit ordering"
    );

    auto vertical = verticalConfig();
    vertical.TailAR = 0.0;
    vertical.RudderMechanicalLimit[0] = 0.2;
    vertical.RudderMechanicalLimit[1] = -0.2;
    const navion::VerticalStabilizer verticalModel(vertical);
    requireNear(
        verticalModel.getConfig().TailAR,
        2.0,
        1.0e-14,
        "Vertical-tail automatic aspect ratio"
    );
    require(
        verticalModel.getConfig().RudderMechanicalLimit[0] == -0.2,
        "Rudder limit ordering"
    );
}

} // namespace

int main()
{
    try
    {
        testHorizontalLoadAndMomentContract();
        testHorizontalAngularRateAndIntrinsicMoment();
        testVerticalRestoringSigns();
        testVerticalDragUsesFullLocalVelocity();
        testLocalFlowFieldAndRk4StageEvaluation();
        testAutomaticAspectRatioAndLimitOrdering();
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }

    std::cout << "All Navion Stage 2 stabilizer tests passed.\n";
    return 0;
}
