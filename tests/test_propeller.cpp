#include "navion/components/propeller/PropellerModel.hpp"
#include "navion/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "navion/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "navion/integration/RK4Integrator.hpp"
#include "navion/model/NavionModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace navion::propeller;
using navion::Matrix3;
using navion::Vec3;
using navion::cross;

namespace {

double vectorNorm(const Vec3& value) {
    return value.norm();
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double absoluteTolerance,
    const std::string& message
) {
    if (!std::isfinite(actual) ||
        std::abs(actual - expected) > absoluteTolerance) {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected) +
            ", abs_tol=" + std::to_string(absoluteTolerance)
        );
    }
}

void requireRelativeNear(
    double actual,
    double expected,
    double relativeTolerance,
    const std::string& message
) {
    const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
    requireNear(actual, expected, relativeTolerance * scale, message);
}

RuntimeInput seaLevelInput() {
    RuntimeInput input{};
    input.airDensityKgM3 = 1.225;
    input.speedOfSoundMps = 340.294;
    return input;
}

struct ContextFixture {
    navion::RigidBodyState state{};
    navion::ControlInputs controls{};
    navion::Environment environment{};
    navion::FlightCondition flightCondition{};
    navion::MassProperties massProperties{
        1000.0,
        Matrix3::diagonal(1200.0, 1500.0, 2000.0)
    };

    void setAirRelativeVelocity(const Vec3& velocityBodyMps) {
        flightCondition.airRelativeVelocityBodyMps = velocityBodyMps;
        flightCondition.airspeedMps = velocityBodyMps.norm();
        if (flightCondition.airspeedMps > 0.0) {
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

    navion::EvaluationContext context() const {
        return {
            0.0,
            state,
            controls,
            environment,
            flightCondition,
            massProperties
        };
    }
};

PropellerParameters makeAnalyticParameters() {
    PropellerParameters p{};
    p.propellerName = "analytic constant-chord test rotor";
    p.airfoilName = "constant-coefficient test polar";
    p.bladeCount = 3;
    p.radiusM = 1.2;
    p.rootCutoutFraction = 0.2;
    p.rotationRateRadps = 100.0;
    p.rotationSign = +1;
    p.rotatingInertiaKgM2 = 0.0;
    p.hubPositionFromCgBodyM = {};
    p.bodyFromPropeller = Matrix3::identity();
    p.bladeGeometry = {
        {0.2, 0.15, 0.1},
        {1.0, 0.15, 0.1},
    };
    p.airfoilPolar = AirfoilPolar({
        {-1.0, 1.0, 0.08},
        {+1.0, 1.0, 0.08},
    });
    p.radialElementCount = 800;
    p.azimuthStationCount = 96;
    p.inflow.lambdaMaximum = 0.5;
    return p;
}

void testPolarInterpolationAndClamp() {
    const AirfoilPolar polar({
        {-1.0, -2.0, 0.2},
        { 0.0,  0.0, 0.1},
        { 1.0,  2.0, 0.2},
    });
    const PolarLookup middle = polar.lookup(0.25);
    requireNear(middle.cl, 0.5, 1.0e-15, "Polar Cl interpolation failed");
    requireNear(middle.cd, 0.125, 1.0e-15, "Polar Cd interpolation failed");
    require(!middle.clamped, "In-range polar lookup was marked clamped");
    const PolarLookup outside = polar.lookup(2.0);
    requireNear(outside.cl, 2.0, 0.0, "Polar upper clamp Cl failed");
    require(outside.clamped, "Out-of-range polar lookup was not marked clamped");
}

void testAnalyticBladeElementIntegral() {
    const PropellerParameters p = makeAnalyticParameters();
    const PropellerModel model(p);
    const RuntimeInput input = seaLevelInput();
    const DiskLoads loads = model.evaluateDiskAtInflow(input, 0.0).loads;

    const double root_m = p.rootCutoutFraction * p.radiusM;
    const double cl = 1.0;
    const double cd = 0.08;
    const double chordM = 0.15;
    const double expectedThrust_N =
        static_cast<double>(p.bladeCount) * input.airDensityKgM3 *
        p.rotationRateRadps * p.rotationRateRadps * chordM * cl *
        (std::pow(p.radiusM, 3) - std::pow(root_m, 3)) / 6.0;
    const double expectedTorque_Nm =
        static_cast<double>(p.bladeCount) * input.airDensityKgM3 *
        p.rotationRateRadps * p.rotationRateRadps * chordM * cd *
        (std::pow(p.radiusM, 4) - std::pow(root_m, 4)) / 8.0;

    requireRelativeNear(
        loads.thrustN,
        expectedThrust_N,
        2.0e-6,
        "Midpoint BEMT thrust does not match analytic integral"
    );
    requireRelativeNear(
        loads.torqueRequiredNm,
        expectedTorque_Nm,
        2.0e-6,
        "Midpoint BEMT torque does not match analytic integral"
    );
    requireRelativeNear(
        loads.aerodynamicMomentAtHubPropellerNm.x,
        -expectedTorque_Nm,
        2.0e-6,
        "Integrated aerodynamic reaction-torque sign is wrong"
    );
    require(vectorNorm(Vec3{loads.forcePropellerN.y, loads.forcePropellerN.z, 0.0}) <
                1.0e-10 * expectedThrust_N,
            "Analytic axial case lost disk symmetry");
}

void testStaticMomentumClosure() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    const PropellerOutput output = model.evaluate(seaLevelInput());
    require(output.inflow.converged, "Static inflow solution did not converge");
    require(output.disk.thrustN > 0.0, "Static thrust must be positive");
    require(output.inflow.lambdaInduced > 0.0, "Static induced inflow must be positive");
    requireNear(
        output.disk.thrustCoefficientRotor,
        2.0 * output.inflow.lambdaInduced * output.inflow.lambdaInduced,
        2.0e-9,
        "Static CT=2*lambda_i^2 momentum identity failed"
    );
    require(std::abs(output.inflow.momentumResidual) < 1.0e-9,
            "Static momentum residual exceeds tolerance");
    require(output.disk.polarClampCount == 0, "Static solution clamped polar data");
    require(output.disk.reverseTangentialFlowCount == 0,
            "Static solution entered reverse tangential flow");
    require(output.disk.compressibilityWarning,
            "Static tip-speed condition should expose the V1 compressibility warning");
    requireRelativeNear(
        output.disk.thrustN,
        4554.89,
        5.0e-5,
        "Stage 3 refactor changed the Propeller V1 static-thrust regression"
    );
    requireRelativeNear(
        output.disk.torqueRequiredNm,
        531.73,
        5.0e-5,
        "Stage 3 refactor changed the Propeller V1 static-torque regression"
    );

    const double expectedRatio = std::pow(pi, 3) / 4.0;
    requireRelativeNear(
        output.disk.thrustCoefficientPropeller /
            output.disk.thrustCoefficientRotor,
        expectedRatio,
        1.0e-12,
        "Rotor and propeller CT normalizations were mixed"
    );
}

void testInstallationTransform() {
    PropellerParameters p = makeEstimatedNavionNaca5868_9Parameters();
    p.hubPositionFromCgBodyM = {};
    p.rotatingInertiaKgM2 = 0.0;
    // Proper +90 radian rotation about body z: propeller +x maps to body +y.
    p.bodyFromPropeller = Matrix3({
        0.0, -1.0, 0.0,
        1.0,  0.0, 0.0,
        0.0,  0.0, 1.0
    });
    RuntimeInput input = seaLevelInput();
    input.velocityCgRelativeAirBodyMps = {0.0, 20.0, 0.0};
    const PropellerOutput output = PropellerModel(p).evaluate(input);
    require(output.inflow.converged, "Rotated-installation case failed to converge");
    const double scale = std::max(1.0, output.disk.thrustN);
    require(std::abs(output.forceBodyN.x) < 1.0e-12 * scale,
            "Installation transform left thrust in body x");
    requireNear(output.forceBodyN.y, output.disk.thrustN, 1.0e-12 * scale,
                "Installation transform did not map propeller x to body y");
    require(std::abs(output.forceBodyN.z) < 1.0e-12 * scale,
            "Installation transform generated body z thrust");
}

void testElementSampleSums() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    RuntimeInput input = seaLevelInput();
    input.velocityCgRelativeAirBodyMps = {24.0, 1.5, 2.5};
    const PropellerOutput solved = model.evaluate(input);
    require(solved.inflow.converged, "Element-sum case failed to converge");
    const DiskEvaluation disk = model.evaluateDiskAtInflow(
        input,
        solved.inflow.lambdaInduced,
        true
    );
    Vec3 forceSum{};
    Vec3 momentSum{};
    for (const ElementSample& sample : disk.elementSamples) {
        forceSum += sample.differentialMeanForcePropellerN;
        momentSum += sample.differentialMeanMomentPropellerNm;
    }
    require(disk.elementSamples.size() == disk.loads.elementCount,
            "Captured element count is inconsistent");
    requireNear(vectorNorm(forceSum - disk.loads.forcePropellerN), 0.0, 1.0e-9,
                "Element force samples do not sum to disk force");
    requireNear(
        vectorNorm(momentSum - disk.loads.aerodynamicMomentAtHubPropellerNm),
        0.0,
        1.0e-9,
        "Element moment samples do not sum to disk moment"
    );
}

void testAxialSymmetryAndTorqueBookkeeping() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    RuntimeInput input = seaLevelInput();
    input.velocityCgRelativeAirBodyMps = {25.0, 0.0, 0.0};
    const PropellerOutput output = model.evaluate(input);
    require(output.inflow.converged, "Axial-flow inflow did not converge");

    const double forceScale = std::max(1.0, std::abs(output.disk.thrustN));
    const double momentScale = std::max(1.0, output.disk.torqueRequiredNm);
    require(std::abs(output.disk.forcePropellerN.y) < 1.0e-12 * forceScale,
            "Axial flow generated propeller Y force");
    require(std::abs(output.disk.forcePropellerN.z) < 1.0e-12 * forceScale,
            "Axial flow generated propeller Z force");
    require(std::abs(output.hubBendingMomentBodyNm.y) < 1.0e-12 * momentScale,
            "Axial flow generated hub pitch moment");
    require(std::abs(output.hubBendingMomentBodyNm.z) < 1.0e-12 * momentScale,
            "Axial flow generated hub yaw moment");

    const Vec3 expectedReaction{
        -static_cast<double>(model.parameters().rotationSign) *
            output.disk.torqueRequiredNm,
        0.0,
        0.0,
    };
    requireNear(
        vectorNorm(output.reactionTorqueBodyNm - expectedReaction),
        0.0,
        1.0e-10 * momentScale,
        "Reaction torque projection or sign is incorrect"
    );
}

void testPFactorSignAgainstStevens() {
    RuntimeInput input = seaLevelInput();
    const double alphaRad = 0.12;
    input.velocityCgRelativeAirBodyMps = {
        25.0 * std::cos(alphaRad),
        0.0,
        25.0 * std::sin(alphaRad),
    };

    PropellerParameters clockwise = makeEstimatedNavionNaca5868_9Parameters();
    clockwise.hubPositionFromCgBodyM = {};
    const PropellerOutput positive = PropellerModel(clockwise).evaluate(input);
    require(positive.inflow.converged, "Positive-rotation P-factor case failed");
    require(positive.hubBendingMomentBodyNm.z < -1.0,
            "For +x clockwise rotation and positive W, Stevens predicts negative yaw");

    PropellerParameters counterClockwise = clockwise;
    counterClockwise.rotationSign = -1;
    const PropellerOutput negative = PropellerModel(counterClockwise).evaluate(input);
    require(negative.inflow.converged, "Negative-rotation P-factor case failed");
    require(negative.hubBendingMomentBodyNm.z > 1.0,
            "Reversing rotation did not reverse P-factor yaw");
    requireRelativeNear(
        positive.disk.thrustN,
        negative.disk.thrustN,
        1.0e-11,
        "Reversing a mirrored propeller changed mean thrust"
    );
}

void testAerodynamicPitchRateDamping() {
    PropellerParameters p = makeEstimatedNavionNaca5868_9Parameters();
    p.hubPositionFromCgBodyM = {};
    p.rotatingInertiaKgM2 = 0.0;
    const PropellerModel model(p);
    RuntimeInput input = seaLevelInput();
    input.velocityCgRelativeAirBodyMps = {20.0, 0.0, 0.0};
    input.angularRateBodyWrtInertialBodyRadps = {0.0, 0.05, 0.0};
    const PropellerOutput output = model.evaluate(input);
    require(output.inflow.converged, "Pitch-rate damping case failed to converge");
    require(output.hubBendingMomentBodyNm.y < -1.0,
            "Positive pitch rate did not produce negative aerodynamic pitch damping");
}

void testGyroscopicMomentAndTotalMomentIdentity() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    RuntimeInput input = seaLevelInput();
    input.velocityCgRelativeAirBodyMps = {20.0, 1.0, 2.0};
    input.angularRateBodyWrtInertialBodyRadps = {0.02, 0.05, -0.03};
    const PropellerOutput output = model.evaluate(input);
    require(output.inflow.converged, "Gyroscopic case failed to converge");

    const Vec3 shaftAxis =
        model.parameters().bodyFromPropeller * Vec3{1.0, 0.0, 0.0};
    const Vec3 angularMomentum =
        static_cast<double>(model.parameters().rotationSign) *
        model.parameters().rotatingInertiaKgM2 *
        model.parameters().rotationRateRadps * shaftAxis;
    const Vec3 expectedGyro = -cross(
        input.angularRateBodyWrtInertialBodyRadps,
        angularMomentum
    );
    requireNear(
        vectorNorm(output.gyroscopicMomentBodyNm - expectedGyro),
        0.0,
        1.0e-11,
        "Gyroscopic moment is inconsistent with -omega cross H"
    );

    const Vec3 reconstructed =
        output.aerodynamicMomentAtHubBodyNm +
        output.momentArmBodyNm +
        output.gyroscopicMomentBodyNm;
    requireNear(
        vectorNorm(output.totalMomentAtCgBodyNm - reconstructed),
        0.0,
        1.0e-11,
        "CG moment decomposition does not close"
    );
    requireNear(
        vectorNorm(
            output.aerodynamicMomentAtHubBodyNm -
            output.reactionTorqueBodyNm -
            output.hubBendingMomentBodyNm
        ),
        0.0,
        1.0e-11,
        "Hub aerodynamic-moment decomposition does not close"
    );
}

void testDensityScaling() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    RuntimeInput low = seaLevelInput();
    low.airDensityKgM3 = 0.9;
    low.velocityCgRelativeAirBodyMps = {18.0, 0.0, 1.0};
    RuntimeInput high = low;
    high.airDensityKgM3 = 1.2;

    const PropellerOutput lowOutput = model.evaluate(low);
    const PropellerOutput highOutput = model.evaluate(high);
    require(lowOutput.inflow.converged && highOutput.inflow.converged,
            "Density-scaling cases failed to converge");
    requireNear(
        lowOutput.inflow.lambdaInduced,
        highOutput.inflow.lambdaInduced,
        2.0e-11,
        "Alpha-only incompressible model lambda should be density-independent"
    );
    requireRelativeNear(
        highOutput.disk.thrustN / lowOutput.disk.thrustN,
        high.airDensityKgM3 / low.airDensityKgM3,
        1.0e-10,
        "Thrust did not scale linearly with density"
    );
}

void testGridConvergence() {
    PropellerParameters coarse = makeEstimatedNavionNaca5868_9Parameters();
    coarse.radialElementCount = 32;
    coarse.azimuthStationCount = 48;
    PropellerParameters fine = coarse;
    fine.radialElementCount = 128;
    fine.azimuthStationCount = 192;

    RuntimeInput input = seaLevelInput();
    input.velocityCgRelativeAirBodyMps = {26.0, 1.0, 3.0};
    input.angularRateBodyWrtInertialBodyRadps = {0.01, 0.03, -0.02};
    const PropellerOutput a = PropellerModel(coarse).evaluate(input);
    const PropellerOutput b = PropellerModel(fine).evaluate(input);
    require(a.inflow.converged && b.inflow.converged,
            "Grid-convergence cases failed to converge");
    requireRelativeNear(a.disk.thrustN, b.disk.thrustN, 2.0e-3,
                        "Thrust grid convergence failed");
    requireRelativeNear(a.disk.torqueRequiredNm, b.disk.torqueRequiredNm, 2.0e-3,
                        "Torque grid convergence failed");
    require(
        vectorNorm(a.hubBendingMomentBodyNm - b.hubBendingMomentBodyNm) /
            std::max(1.0, vectorNorm(b.hubBendingMomentBodyNm)) < 5.0e-3,
        "Hub bending-moment grid convergence failed"
    );
}

void testTakeoffSweepWithExplicitNumericalGuess() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    constexpr double rotationSpeed_m_s = 65.0 * 0.44704;
    constexpr double groundPitch_rad = 0.035;

    double previousLambda = model.parameters().inflow.initialGuess;
    for (std::size_t index = 0; index <= 100; ++index) {
        const double speed_m_s =
            rotationSpeed_m_s * static_cast<double>(index) / 100.0;
        RuntimeInput input = seaLevelInput();
        input.velocityCgRelativeAirBodyMps = {
            speed_m_s * std::cos(groundPitch_rad),
            0.0,
            speed_m_s * std::sin(groundPitch_rad),
        };
        const PropellerOutput output = model.evaluate(input, previousLambda);
        require(output.inflow.converged, "A takeoff-sweep point did not converge");
        require(std::abs(output.inflow.momentumResidual) < 1.0e-9,
                "A takeoff-sweep residual exceeds tolerance");
        require(output.disk.polarClampCount == 0,
                "A takeoff-sweep point clamped the polar");
        require(output.disk.reverseTangentialFlowCount == 0,
                "A takeoff-sweep point entered reverse tangential flow");
        require(output.forceBodyN.isFinite() &&
                output.totalMomentAtCgBodyNm.isFinite(),
                "A takeoff-sweep point returned non-finite loads");
        previousLambda = output.inflow.lambdaInduced;
    }
    require(previousLambda >= 0.0,
            "Explicit numerical inflow guess became invalid during sweep");
}

void testCommonComponentContract() {
    PropellerParameters parameters = makeEstimatedNavionNaca5868_9Parameters();
    parameters.radialElementCount = 24U;
    parameters.azimuthStationCount = 32U;
    const PropellerComponent component(parameters);

    ContextFixture fixture;
    fixture.setAirRelativeVelocity({20.0, 1.0, 2.0});
    fixture.state.angularRateBodyRadps = {0.02, 0.05, -0.03};
    fixture.controls.propellerEnabled = true;
    fixture.controls.throttle = 1.0;

    const PropellerOutput detailed = component.evaluateDetailed(fixture.context());
    const navion::BodyLoad load = component.computeLoad(fixture.context());
    require(detailed.inflow.converged,
            "Common-contract propeller inflow did not converge");
    requireNear(
        vectorNorm(load.forceBodyN - detailed.forceBodyN),
        0.0,
        1.0e-12,
        "Propeller BodyLoad force differs from detailed output"
    );
    requireNear(
        vectorNorm(
            load.momentAboutCgBodyNm - detailed.totalMomentAtCgBodyNm
        ),
        0.0,
        1.0e-12,
        "Propeller BodyLoad moment differs from detailed CG moment"
    );
    requireNear(
        vectorNorm(
            detailed.momentArmBodyNm -
            cross(parameters.hubPositionFromCgBodyM, detailed.forceBodyN)
        ),
        0.0,
        1.0e-11,
        "Propeller arm moment is not r_CG_to_hub cross F"
    );

    fixture.controls.propellerEnabled = false;
    const navion::BodyLoad disabled = component.computeLoad(fixture.context());
    requireNear(vectorNorm(disabled.forceBodyN), 0.0, 0.0,
                "Disabled PropellerComponent returned force");
    requireNear(vectorNorm(disabled.momentAboutCgBodyNm), 0.0, 0.0,
                "Disabled PropellerComponent returned moment");

    fixture.controls.propellerEnabled = true;
    fixture.controls.propellerSpeedScale = 0.0;
    const navion::BodyLoad stopped = component.computeLoad(fixture.context());
    requireNear(vectorNorm(stopped.forceBodyN), 0.0, 0.0,
                "Zero-speed PropellerComponent returned force");
    requireNear(vectorNorm(stopped.momentAboutCgBodyNm), 0.0, 0.0,
                "Zero-speed PropellerComponent returned moment");
}

navion::HorizontalStabilizerConfig zeroHorizontalTailConfig() {
    navion::HorizontalStabilizerConfig config;
    config.Area = 1.0;
    config.TailSpan = 1.0;
    config.TailMAC = 1.0;
    config.ElevatorMechanicalLimit[0] = -0.5;
    config.ElevatorMechanicalLimit[1] = 0.5;
    return config;
}

navion::VerticalStabilizerConfig zeroVerticalTailConfig() {
    navion::VerticalStabilizerConfig config;
    config.Area = 1.0;
    config.TailSpan = 1.0;
    config.TailMAC = 1.0;
    config.RudderMechanicalLimit[0] = -0.5;
    config.RudderMechanicalLimit[1] = 0.5;
    return config;
}

void testStage3NavionModelAndRk4Integration() {
    PropellerParameters parameters = makeEstimatedNavionNaca5868_9Parameters();
    parameters.radialElementCount = 16U;
    parameters.azimuthStationCount = 24U;

    navion::NavionModel model({
        1000.0,
        Matrix3::diagonal(1200.0, 1500.0, 2000.0)
    });
    model.addLoadComponent(
        std::make_unique<navion::HorizontalStabilizer>(
            zeroHorizontalTailConfig()
        )
    );
    model.addLoadComponent(
        std::make_unique<navion::VerticalStabilizer>(
            zeroVerticalTailConfig()
        )
    );
    model.addLoadComponent(
        std::make_unique<PropellerComponent>(parameters)
    );

    navion::RigidBodyState state;
    state.velocityBodyMps = {20.0, 0.0, 0.0};
    navion::ControlInputs controls;
    controls.propellerEnabled = true;
    controls.throttle = 1.0;
    navion::Environment environment;
    environment.gravityNedMps2 = {};

    const navion::ModelEvaluation evaluation =
        model.evaluate(0.0, state, controls, environment);
    require(evaluation.contributingComponentCount == 3U,
            "Stage 3 must accumulate HS, VS, and Propeller loads");
    require(evaluation.totalComponentLoad.forceBodyN.x > 0.0,
            "Integrated propeller must create positive BODY-X thrust");
    require(evaluation.totalComponentLoad.isFinite(),
            "Stage 3 total component load must be finite");

    navion::RK4Integrator integrator;
    const navion::RigidBodyState next = integrator.step(
        0.0,
        1.0e-3,
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
    require(next.isFinite(), "Stage 3 RK4 state must remain finite");
    require(next.velocityBodyMps.x > state.velocityBodyMps.x,
            "Propeller thrust must accelerate the test rigid body in +X");
}

void testDisabledModel() {
    const PropellerModel model(makeEstimatedNavionNaca5868_9Parameters());
    RuntimeInput input = seaLevelInput();
    input.enabled = false;
    const PropellerOutput output = model.evaluate(input);
    require(output.inflow.converged, "Disabled model should return a valid zero-load state");
    requireNear(vectorNorm(output.forceBodyN), 0.0, 0.0,
                "Disabled model returned nonzero force");
    requireNear(vectorNorm(output.totalMomentAtCgBodyNm), 0.0, 0.0,
                "Disabled model returned nonzero moment");
}

void testPrescribedRotationRateScale() {
    PropellerParameters parameters = makeEstimatedNavionNaca5868_9Parameters();
    parameters.radialElementCount = 12U;
    parameters.azimuthStationCount = 16U;
    const PropellerModel model(parameters);

    RuntimeInput stopped = seaLevelInput();
    stopped.rotationRateScale = 0.0;
    const PropellerOutput stoppedOutput = model.evaluate(stopped);
    require(stoppedOutput.inflow.converged,
            "Stopped prescribed-RPM input must be a valid zero-load result");
    requireNear(stoppedOutput.rpm, 0.0, 0.0,
                "Stopped prescribed-RPM input returned nonzero RPM");
    requireNear(vectorNorm(stoppedOutput.forceBodyN), 0.0, 0.0,
                "Stopped prescribed-RPM input returned force");

    RuntimeInput halfSpeed = seaLevelInput();
    halfSpeed.rotationRateScale = 0.5;
    const PropellerOutput halfOutput = model.evaluate(halfSpeed);
    require(halfOutput.inflow.converged,
            "Intermediate prescribed-RPM input did not converge");
    requireRelativeNear(
        halfOutput.rpm,
        0.5 * model.parameters().rpm(),
        1.0e-12,
        "Intermediate prescribed-RPM output is inconsistent"
    );
    require(halfOutput.disk.thrustN > 0.0,
            "Intermediate prescribed-RPM input must produce thrust");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, void (*)()>> tests{
        {"polar interpolation and clamp", testPolarInterpolationAndClamp},
        {"analytic blade-element integral", testAnalyticBladeElementIntegral},
        {"static momentum closure", testStaticMomentumClosure},
        {"axial symmetry and torque bookkeeping", testAxialSymmetryAndTorqueBookkeeping},
        {"installation-frame transform", testInstallationTransform},
        {"element-sample sums", testElementSampleSums},
        {"P-factor sign against Stevens", testPFactorSignAgainstStevens},
        {"aerodynamic pitch-rate damping", testAerodynamicPitchRateDamping},
        {"gyroscopic and total-moment identities", testGyroscopicMomentAndTotalMomentIdentity},
        {"density scaling", testDensityScaling},
        {"grid convergence", testGridConvergence},
        {"takeoff sweep with explicit numerical guess",
         testTakeoffSweepWithExplicitNumericalGuess},
        {"common ILoadComponent contract", testCommonComponentContract},
        {"Stage 3 NavionModel and RK4 integration",
         testStage3NavionModelAndRk4Integration},
        {"disabled model", testDisabledModel},
        {"prescribed rotation-rate scale", testPrescribedRotationRateScale},
    };

    try {
        for (const auto& [name, function] : tests) {
            function();
            std::cout << "[PASS] " << name << '\n';
        }
        std::cout << "All " << tests.size() << " propeller V1 tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "[FAIL] " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
