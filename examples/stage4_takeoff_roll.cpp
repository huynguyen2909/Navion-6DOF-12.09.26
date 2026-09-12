#include "navion/components/landing_gear/LandingGearComponent.hpp"
#include "navion/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "navion/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "navion/integration/RK4Integrator.hpp"
#include "navion/model/NavionModel.hpp"
#include "navion/components/propeller/PropellerModel.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace
{

navion::HorizontalStabilizerConfig horizontalTail()
{
    navion::HorizontalStabilizerConfig config;
    config.Area = 3.99483072;
    config.TailSpan = 3.99741452;
    config.TailMAC = 1.01236110;
    config.PositionWrtCG[0] = -4.35074096;
    config.PositionWrtCG[2] = 0.4;
    config.ElevatorArea = 1.1;
    config.ElevatorMechanicalLimit[0] = -0.52;
    config.ElevatorMechanicalLimit[1] = 0.35;
    config.LiftCurveSlope = 3.324;
    config.ZeroLiftDragCoefficient = 0.010;
    config.InducedDragFactor = 0.0884;
    config.ElevatorEffectiveness = 0.447;
    return config;
}

navion::VerticalStabilizerConfig verticalTail()
{
    navion::VerticalStabilizerConfig config;
    config.Area = 1.356384384;
    config.RudderArea = 0.7738823232;
    config.TailSpan = 1.50;
    config.TailMAC = 0.90;
    config.PositionWrtCG[0] = -4.649228943;
    config.PositionWrtCG[2] = -0.765145070;
    config.RudderMechanicalLimit[0] = -0.40;
    config.RudderMechanicalLimit[1] = 0.30;
    config.SideForceCurveSlope = 3.694542443;
    config.RudderEffectiveness = 0.523912357;
    config.ZeroLiftDragCoefficient = 0.012;
    config.InducedDragFactor = 0.10;
    return config;
}

navion::landing_gear::LandingGearParameters demonstrationGear()
{
    using navion::landing_gear::BrakeGroup;
    using navion::landing_gear::GearParameters;
    using navion::landing_gear::LandingGearParameters;
    using navion::landing_gear::SteeringMode;

    LandingGearParameters parameters;
    parameters.solverMaximumIterations = 50;
    parameters.solverConvergenceTolerance = 1.0e-5;
    parameters.maximumStrutForceN = 50000.0;

    GearParameters nose;
    nose.name = "NOSE_LG";
    nose.locationFromCgBodyM = {2.5, 0.0, 1.0};
    nose.springStiffnessNpm = 60000.0;
    nose.dampingCoefficientNsPm = 12000.0;
    nose.staticFrictionCoefficient = 0.5;
    nose.rollingFrictionCoefficient = 0.02;
    nose.maximumSteeringAngleRad = 0.12;
    nose.steeringMode = SteeringMode::Steered;
    nose.brakeGroup = BrakeGroup::None;

    GearParameters left = nose;
    left.name = "LEFT_MLG";
    left.locationFromCgBodyM = {-1.0, -1.3, 1.0};
    left.springStiffnessNpm = 75000.0;
    left.dampingCoefficientNsPm = 15000.0;
    left.maximumSteeringAngleRad = 0.0;
    left.steeringMode = SteeringMode::Fixed;
    left.brakeGroup = BrakeGroup::Left;

    GearParameters right = left;
    right.name = "RIGHT_MLG";
    right.locationFromCgBodyM.y = 1.3;
    right.brakeGroup = BrakeGroup::Right;
    parameters.gears = {nose, left, right};
    return parameters;
}

navion::ControlInputs takeoffRollControls(double timeS)
{
    (void)timeS;
    navion::ControlInputs controls;
    controls.throttle = 1.0;
    controls.propellerEnabled = true;
    controls.landingGearExtended = true;
    controls.elevatorRad = 0.0;
    controls.rudderRad = 0.0;
    controls.noseWheelSteeringRad = 0.0;

    // Scenario invariant: brakes are released at t=0 and at every RK4 stage
    // until the prescribed rotation-speed threshold is reached.
    controls.brakeLeft = 0.0;
    controls.brakeRight = 0.0;
    return controls;
}

void enforceReleasedBrakes(const navion::ControlInputs& controls)
{
    if (controls.brakeLeft != 0.0 || controls.brakeRight != 0.0)
    {
        throw std::logic_error("Takeoff-roll brake invariant was violated.");
    }
}

} // namespace

int main()
{
    // Provisional integration-demo data, not a validated Navion data set.
    constexpr double massKg = 1250.0;
    constexpr double rotationSpeedMps = 30.0;
    constexpr double dtS = 0.002;
    constexpr double maximumTimeS = 15.0;

    const navion::MassProperties massProperties{
        massKg,
        navion::Matrix3::diagonal(5000.0, 10000.0, 12000.0)
    };
    navion::NavionModel model(massProperties);
    model.addLoadComponent(
        std::make_unique<navion::HorizontalStabilizer>(horizontalTail())
    );
    model.addLoadComponent(
        std::make_unique<navion::VerticalStabilizer>(verticalTail())
    );

    auto propeller = navion::propeller::makeEstimatedNavionNaca5868_9Parameters();
    propeller.radialElementCount = 12U;
    propeller.azimuthStationCount = 16U;
    model.addLoadComponent(
        std::make_unique<navion::propeller::PropellerComponent>(propeller)
    );
    model.setGroundContactComponent(
        std::make_unique<navion::landing_gear::LandingGearComponent>(
            demonstrationGear()
        )
    );

    navion::Environment environment;
    navion::RigidBodyState state;
    const double staticCompressionM =
        massKg * environment.gravityNedMps2.z / 210000.0;
    state.positionNedM.z = staticCompressionM - 1.0;

    navion::RK4Integrator integrator;
    double timeS = 0.0;
    std::size_t step = 0U;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "# t_s airspeed_mps u_mps contacts Fx_ground_N PGS_iter brakes\n";

    while (timeS < maximumTimeS &&
           state.velocityBodyMps.norm() < rotationSpeedMps)
    {
        const navion::ControlInputs acceptedControls =
            takeoffRollControls(timeS);
        enforceReleasedBrakes(acceptedControls);

        const navion::RigidBodyState next = integrator.step(
            timeS,
            dtS,
            state,
            [&model, &environment](
                double stageTimeS,
                const navion::RigidBodyState& stageState
            ) {
                const navion::ControlInputs stageControls =
                    takeoffRollControls(stageTimeS);
                enforceReleasedBrakes(stageControls);
                return model.evaluateDerivative(
                    stageTimeS,
                    dtS,
                    stageState,
                    stageControls,
                    environment
                );
            }
        );

        timeS += dtS;
        state = next;
        const navion::ControlInputs endControls = takeoffRollControls(timeS);
        enforceReleasedBrakes(endControls);
        model.commitAcceptedStep(
            timeS,
            dtS,
            state,
            endControls,
            environment
        );

        if (step % 250U == 0U)
        {
            const navion::ModelEvaluation report = model.evaluate(
                timeS,
                dtS,
                state,
                endControls,
                environment
            );
            std::cout << timeS << ' '
                      << report.flightCondition.airspeedMps << ' '
                      << state.velocityBodyMps.x << ' '
                      << report.groundContactCount << ' '
                      << report.groundFrictionLoad.forceBodyN.x << ' '
                      << report.frictionSolverIterations << " 0,0\n";
        }
        ++step;
    }

    std::cout << "# stopped_at_t_s=" << timeS
              << " speed_mps=" << state.velocityBodyMps.norm()
              << " target_V_rotation_mps=" << rotationSpeedMps << '\n';
    std::cout << "# NOTE: Wing and Fuselage are absent; this is an architecture/"
                 "PGS integration scenario, not a validated Navion takeoff.\n";
    return 0;
}
