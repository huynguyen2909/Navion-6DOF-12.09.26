#include "navion/config/ProvisionalNavionConfig.hpp"
#include "navion/initialization/GroundStaticTrim.hpp"
#include "navion/integration/RK4Integrator.hpp"
#include "navion/model/NavionModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{

constexpr double targetPropellerRpm = 2300.0;
constexpr double preReleaseHoldDurationS = 2.0;
constexpr double propellerRampDurationS = 5.0;
constexpr double simulationEndTimeS = 30.0;
constexpr double dtS = 0.002;
constexpr double airbornePersistenceS = 0.25;
constexpr double airborneNormalForceToleranceN = 5.0;

enum class SimulationPhase : int
{
    GroundTrim = 0,
    PreReleaseHold = 1,
    TakeoffRpmRamp = 2,
    TakeoffFullRpm = 3
};

struct LoggedComponent
{
    std::string_view modelName;
    std::string_view csvPrefix;
};

constexpr std::array<LoggedComponent, 6> loggedComponents{{
    {"MainWing", "main_wing"},
    {"HorizontalStabilizer", "horizontal_stabilizer"},
    {"VerticalStabilizer", "vertical_stabilizer"},
    {"Fuselage", "fuselage"},
    {"Propeller", "propeller"},
    {"LandingGearPGS", "landing_gear"}
}};

struct EulerAngles
{
    double rollRad{0.0};
    double pitchRad{0.0};
    double yawRad{0.0};
};

EulerAngles bodyToNedEuler(const navion::Quaternion& attitudeBodyToNed)
{
    const navion::Quaternion q = attitudeBodyToNed.normalized();
    EulerAngles result;
    result.rollRad = std::atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    );
    result.pitchRad = std::asin(std::clamp(
        2.0 * (q.w * q.y - q.z * q.x),
        -1.0,
        1.0
    ));
    result.yawRad = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    return result;
}

double smoothStep01(double value)
{
    const double clamped = std::clamp(value, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

double propellerSpeedScale(double timelineTimeS)
{
    return smoothStep01(
        (timelineTimeS - preReleaseHoldDurationS) /
        propellerRampDurationS
    );
}

SimulationPhase takeoffPhase(double timelineTimeS)
{
    const double fullRpmTimeS =
        preReleaseHoldDurationS + propellerRampDurationS;
    return timelineTimeS < fullRpmTimeS - 1.0e-9
        ? SimulationPhase::TakeoffRpmRamp
        : SimulationPhase::TakeoffFullRpm;
}

navion::ControlInputs groundStaticTrimControls()
{
    navion::ControlInputs controls;
    controls.throttle = 0.0;
    controls.propellerEnabled = false;
    controls.propellerSpeedScale = 0.0;
    controls.landingGearExtended = true;
    controls.elevatorRad = 0.0;
    controls.aileronRad = 0.0;
    controls.rudderRad = 0.0;
    controls.flapRad = 0.0;
    controls.noseWheelSteeringRad = 0.0;
    controls.brakeLeft = 1.0;
    controls.brakeRight = 1.0;
    return controls;
}

navion::ControlInputs preReleaseHoldControls()
{
    return groundStaticTrimControls();
}

navion::ControlInputs takeoffRollControls(double timelineTimeS)
{
    navion::ControlInputs controls;
    controls.propellerSpeedScale = propellerSpeedScale(timelineTimeS);
    controls.throttle = controls.propellerSpeedScale;
    controls.propellerEnabled = true;
    controls.landingGearExtended = true;
    controls.elevatorRad = 0.0;
    controls.aileronRad = 0.0;
    controls.rudderRad = 0.0;
    controls.flapRad = 0.0;
    controls.noseWheelSteeringRad = 0.0;
    controls.brakeLeft = 0.0;
    controls.brakeRight = 0.0;
    return controls;
}

navion::RigidBodyState makeInitialGroundGuess(
    const navion::ProvisionalNavionConfig& aircraft,
    const navion::Environment& environment
)
{
    double totalStrutStiffnessNpm = 0.0;
    double stiffnessWeightedGearDownM = 0.0;
    for (const auto& gear : aircraft.landingGear.gears)
    {
        totalStrutStiffnessNpm += gear.springStiffnessNpm;
        stiffnessWeightedGearDownM +=
            gear.springStiffnessNpm * gear.locationFromCgBodyM.z;
    }
    if (totalStrutStiffnessNpm <= 0.0)
    {
        throw std::logic_error("Landing Gear has no positive strut stiffness.");
    }

    const double referenceGearDownM =
        stiffnessWeightedGearDownM / totalStrutStiffnessNpm;
    const double approximateCompressionM =
        aircraft.massProperties.massKg * environment.gravityNedMps2.z /
        totalStrutStiffnessNpm;

    navion::RigidBodyState state;
    state.positionNedM.z =
        aircraft.landingGear.groundDownPositionNedM -
        referenceGearDownM + approximateCompressionM;
    return state;
}

void enforceReleasedBrakes(const navion::ControlInputs& controls)
{
    if (controls.brakeLeft != 0.0 || controls.brakeRight != 0.0)
    {
        throw std::logic_error("Takeoff-roll brake invariant was violated.");
    }
}

const navion::BodyLoad& componentLoad(
    const navion::ModelEvaluation& evaluation,
    std::string_view name
)
{
    for (const auto& report : evaluation.componentLoads)
    {
        if (report.name == name)
        {
            return report.load;
        }
    }
    throw std::runtime_error(
        "ModelEvaluation is missing component-load report: " +
        std::string(name)
    );
}

bool isAirborneCandidate(const navion::ModelEvaluation& evaluation)
{
    return evaluation.groundContactCount == 0U &&
        evaluation.groundNormalLoad.forceBodyN.norm() <=
            airborneNormalForceToleranceN;
}

class CsvLogger
{
public:
    CsvLogger(const std::string& path, double trimmedDownPositionM)
        : stream_(path), trimmedDownPositionM_(trimmedDownPositionM)
    {
        if (!stream_)
        {
            throw std::runtime_error("Cannot open CSV output: " + path);
        }
        stream_ << std::setprecision(12);
        stream_
            << "time_s,phase_id,time_since_brake_release_s,propeller_rpm,"
            << "propeller_enabled,position_north_m,position_east_m,position_down_m,"
            << "height_above_trim_m,velocity_down_ned_mps,airborne_candidate,"
            << "u_mps,v_mps,w_mps,roll_rad,pitch_rad,yaw_rad,"
            << "p_radps,q_radps,r_radps,airspeed_mps,alpha_rad,beta_rad,"
            << "component_fx_body_n,component_fy_body_n,component_fz_body_n,"
            << "gravity_fx_body_n,gravity_fy_body_n,gravity_fz_body_n,"
            << "net_fx_body_n,net_fy_body_n,net_fz_body_n,"
            << "net_l_body_nm,net_m_body_nm,net_n_body_nm,"
            << "ground_normal_fx_body_n,ground_normal_fy_body_n,"
            << "ground_normal_fz_body_n,ground_friction_fx_body_n,"
            << "ground_friction_fy_body_n,ground_friction_fz_body_n,";
        for (const LoggedComponent& component : loggedComponents)
        {
            stream_ << component.csvPrefix << "_fx_body_n,"
                    << component.csvPrefix << "_fy_body_n,"
                    << component.csvPrefix << "_fz_body_n,"
                    << component.csvPrefix << "_l_body_nm,"
                    << component.csvPrefix << "_m_body_nm,"
                    << component.csvPrefix << "_n_body_nm,";
        }
        stream_
            << "udot_mps2,vdot_mps2,wdot_mps2,"
            << "pdot_radps2,qdot_radps2,rdot_radps2,"
            << "ground_contact_count,pgs_iterations,pgs_converged,"
            << "throttle,elevator_rad,aileron_rad,rudder_rad,flap_rad,"
            << "brake_left,brake_right,nose_wheel_steering_rad\n";
    }

    void write(
        double timeS,
        SimulationPhase phase,
        const navion::RigidBodyState& state,
        const navion::ControlInputs& controls,
        const navion::Environment& environment,
        const navion::MassProperties& massProperties,
        const navion::ModelEvaluation& evaluation
    )
    {
        const navion::Quaternion attitude =
            state.attitudeBodyToNed.normalized();
        const EulerAngles euler = bodyToNedEuler(attitude);
        const navion::Vec3 gravityAccelerationBody =
            attitude.conjugate().rotate(environment.gravityNedMps2);
        const navion::Vec3 gravityForceBody =
            massProperties.massKg * gravityAccelerationBody;
        const navion::Vec3 netForceBody =
            evaluation.totalComponentLoad.forceBodyN + gravityForceBody;
        const navion::Vec3& netMomentBody =
            evaluation.totalComponentLoad.momentAboutCgBodyNm;
        const navion::Vec3 velocityNedMps =
            attitude.rotate(state.velocityBodyMps);
        const auto& derivative = evaluation.stateDerivative;

        stream_
            << timeS << ','
            << static_cast<int>(phase) << ','
            << timeS - preReleaseHoldDurationS << ','
            << targetPropellerRpm * controls.propellerSpeedScale << ','
            << (controls.propellerEnabled ? 1 : 0) << ','
            << state.positionNedM.x << ','
            << state.positionNedM.y << ','
            << state.positionNedM.z << ','
            << trimmedDownPositionM_ - state.positionNedM.z << ','
            << velocityNedMps.z << ','
            << (isAirborneCandidate(evaluation) ? 1 : 0) << ','
            << state.velocityBodyMps.x << ','
            << state.velocityBodyMps.y << ','
            << state.velocityBodyMps.z << ','
            << euler.rollRad << ','
            << euler.pitchRad << ','
            << euler.yawRad << ','
            << state.angularRateBodyRadps.x << ','
            << state.angularRateBodyRadps.y << ','
            << state.angularRateBodyRadps.z << ','
            << evaluation.flightCondition.airspeedMps << ','
            << evaluation.flightCondition.angleOfAttackRad << ','
            << evaluation.flightCondition.sideSlipRad << ','
            << evaluation.totalComponentLoad.forceBodyN.x << ','
            << evaluation.totalComponentLoad.forceBodyN.y << ','
            << evaluation.totalComponentLoad.forceBodyN.z << ','
            << gravityForceBody.x << ','
            << gravityForceBody.y << ','
            << gravityForceBody.z << ','
            << netForceBody.x << ','
            << netForceBody.y << ','
            << netForceBody.z << ','
            << netMomentBody.x << ','
            << netMomentBody.y << ','
            << netMomentBody.z << ','
            << evaluation.groundNormalLoad.forceBodyN.x << ','
            << evaluation.groundNormalLoad.forceBodyN.y << ','
            << evaluation.groundNormalLoad.forceBodyN.z << ','
            << evaluation.groundFrictionLoad.forceBodyN.x << ','
            << evaluation.groundFrictionLoad.forceBodyN.y << ','
            << evaluation.groundFrictionLoad.forceBodyN.z << ',';

        for (const LoggedComponent& component : loggedComponents)
        {
            const navion::BodyLoad& load =
                componentLoad(evaluation, component.modelName);
            stream_ << load.forceBodyN.x << ','
                    << load.forceBodyN.y << ','
                    << load.forceBodyN.z << ','
                    << load.momentAboutCgBodyNm.x << ','
                    << load.momentAboutCgBodyNm.y << ','
                    << load.momentAboutCgBodyNm.z << ',';
        }

        stream_
            << derivative.velocityRateBodyMps2.x << ','
            << derivative.velocityRateBodyMps2.y << ','
            << derivative.velocityRateBodyMps2.z << ','
            << derivative.angularAccelerationBodyRadps2.x << ','
            << derivative.angularAccelerationBodyRadps2.y << ','
            << derivative.angularAccelerationBodyRadps2.z << ','
            << evaluation.groundContactCount << ','
            << evaluation.frictionSolverIterations << ','
            << (evaluation.frictionSolverConverged ? 1 : 0) << ','
            << controls.throttle << ','
            << controls.elevatorRad << ','
            << controls.aileronRad << ','
            << controls.rudderRad << ','
            << controls.flapRad << ','
            << controls.brakeLeft << ','
            << controls.brakeRight << ','
            << controls.noseWheelSteeringRad << '\n';

        if (!stream_)
        {
            throw std::runtime_error("Failed while writing takeoff CSV.");
        }
    }

private:
    std::ofstream stream_;
    double trimmedDownPositionM_{0.0};
};

} // namespace

int main(int argc, char** argv)
{
    const std::string csvPath =
        argc > 1 ? argv[1] : "stage5_3_takeoff_roll_30s.csv";

    const navion::ProvisionalNavionConfig aircraft =
        navion::makeProvisionalNavionConfig();
    navion::NavionModel model(aircraft.massProperties);
    navion::addProvisionalNavionComponents(model, aircraft);

    navion::Environment environment;
    const navion::RigidBodyState initialGroundGuess =
        makeInitialGroundGuess(aircraft, environment);
    navion::RigidBodyState state = initialGroundGuess;
    const navion::ControlInputs trimControls = groundStaticTrimControls();
    const navion::ModelEvaluation initialGroundEvaluation = model.evaluate(
        -dtS,
        dtS,
        initialGroundGuess,
        trimControls,
        environment
    );

    navion::GroundStaticTrimSolver trimSolver;
    const navion::GroundStaticTrimResult trim = trimSolver.solve(
        model,
        state,
        trimControls,
        environment
    );
    if (!trim.converged)
    {
        throw std::runtime_error("Ground static trim did not converge.");
    }
    state = trim.state;
    model.resetGroundContactHistory();
    model.commitAcceptedStep(
        0.0,
        dtS,
        state,
        trimControls,
        environment
    );

    CsvLogger csv(csvPath, state.positionNedM.z);
    csv.write(
        -dtS,
        SimulationPhase::GroundTrim,
        initialGroundGuess,
        trimControls,
        environment,
        aircraft.massProperties,
        initialGroundEvaluation
    );
    csv.write(
        0.0,
        SimulationPhase::GroundTrim,
        state,
        trimControls,
        environment,
        aircraft.massProperties,
        trim.evaluation
    );

    const EulerAngles trimEuler = bodyToNedEuler(state.attitudeBodyToNed);
    constexpr double radiansToDegrees =
        180.0 / 3.14159265358979323846;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "# ground_static_trim converged=1 iterations="
              << trim.iterations
              << " contacts=" << trim.evaluation.groundContactCount
              << " down_m=" << state.positionNedM.z
              << " roll_deg=" << trimEuler.rollRad * radiansToDegrees
              << " pitch_deg=" << trimEuler.pitchRad * radiansToDegrees
              << " residual_Fz_N=" << trim.residual.netForceBodyN.z
              << " residual_L_Nm="
              << trim.residual.netMomentAboutCgBodyNm.x
              << " residual_M_Nm="
              << trim.residual.netMomentAboutCgBodyNm.y << '\n';
    std::cout << "# schedule t=[0,2): rpm=0 brakes=1,1; "
                 "t=[2,7): brakes=0,0 smoothstep_rpm=0..2300; "
                 "t=[7,30]: rpm=2300; fixed_blade_pitch; "
                 "flap=elevator=aileron=rudder=0\n";

    navion::RK4Integrator integrator;
    double timeS = 0.0;
    std::size_t step = 0U;

    // Integrate and log the two-second, zero-RPM, pre-release hold. The final
    // hold state at exactly t=2 s is logged below with released controls.
    const navion::ControlInputs holdControls = preReleaseHoldControls();
    const std::size_t holdStepCount = static_cast<std::size_t>(
        std::llround(preReleaseHoldDurationS / dtS)
    );
    navion::ModelEvaluation report = trim.evaluation;
    for (std::size_t holdStep = 0U; holdStep < holdStepCount; ++holdStep)
    {
        const navion::RigidBodyState next = integrator.step(
            timeS,
            dtS,
            state,
            [&model, &environment, &holdControls](
                double stageTimeS,
                const navion::RigidBodyState& stageState
            ) {
                return model.evaluateDerivative(
                    stageTimeS,
                    dtS,
                    stageState,
                    holdControls,
                    environment
                );
            }
        );
        timeS = static_cast<double>(holdStep + 1U) * dtS;
        state = next;
        model.commitAcceptedStep(
            timeS,
            dtS,
            state,
            holdControls,
            environment
        );
        report = model.evaluate(
            timeS,
            dtS,
            state,
            holdControls,
            environment
        );
        if (holdStep + 1U < holdStepCount)
        {
            csv.write(
                timeS,
                SimulationPhase::PreReleaseHold,
                state,
                holdControls,
                environment,
                aircraft.massProperties,
                report
            );
        }
    }

    navion::ControlInputs controls = takeoffRollControls(timeS);
    enforceReleasedBrakes(controls);
    report = model.evaluate(timeS, dtS, state, controls, environment);
    csv.write(
        timeS,
        takeoffPhase(timeS),
        state,
        controls,
        environment,
        aircraft.massProperties,
        report
    );

    std::optional<double> liftoffTimeS;
    double airborneCandidateDurationS = 0.0;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "# t_s rpm airspeed_mps u_mps down_m height_from_trim_m "
                 "contacts Fz_ground_N PGS_iter\n";

    while (timeS < simulationEndTimeS - 0.5 * dtS)
    {
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
        controls = takeoffRollControls(timeS);
        enforceReleasedBrakes(controls);
        model.commitAcceptedStep(
            timeS,
            dtS,
            state,
            controls,
            environment
        );
        report = model.evaluate(timeS, dtS, state, controls, environment);
        csv.write(
            timeS,
            takeoffPhase(timeS),
            state,
            controls,
            environment,
            aircraft.massProperties,
            report
        );

        if (isAirborneCandidate(report))
        {
            airborneCandidateDurationS += dtS;
            if (!liftoffTimeS &&
                airborneCandidateDurationS >= airbornePersistenceS)
            {
                liftoffTimeS =
                    timeS - airborneCandidateDurationS + dtS;
            }
        }
        else
        {
            airborneCandidateDurationS = 0.0;
        }

        if (step % 500U == 0U)
        {
            std::cout << timeS << ' '
                      << targetPropellerRpm * controls.propellerSpeedScale << ' '
                      << report.flightCondition.airspeedMps << ' '
                      << state.velocityBodyMps.x << ' '
                      << state.positionNedM.z << ' '
                      << trim.state.positionNedM.z - state.positionNedM.z << ' '
                      << report.groundContactCount << ' '
                      << report.groundNormalLoad.forceBodyN.z << ' '
                      << report.frictionSolverIterations << '\n';
        }
        ++step;
    }

    std::cout << "# completed_at_t_s=" << timeS
              << " speed_mps=" << report.flightCondition.airspeedMps
              << " rpm=" << targetPropellerRpm * controls.propellerSpeedScale
              << " down_m=" << state.positionNedM.z
              << " height_from_trim_m="
              << trim.state.positionNedM.z - state.positionNedM.z
              << " contacts=" << report.groundContactCount << '\n';
    if (liftoffTimeS)
    {
        std::cout << "# persistent_airborne_candidate_from_t_s="
                  << *liftoffTimeS
                  << " criterion=no_contact_and_ground_normal_below_"
                  << airborneNormalForceToleranceN
                  << "_N_for_" << airbornePersistenceS << "_s\n";
    }
    else
    {
        std::cout << "# persistent_airborne_candidate=not_detected\n";
    }
    std::cout << "# csv=" << csvPath << '\n';
    std::cout << "# NOTE: z-Down decreasing indicates upward CG motion, but "
                 "liftoff also requires persistent loss of all ground contacts.\n";
    std::cout << "# NOTE: component parameters remain provisional proxies; "
                 "this is not a validated Navion takeoff-performance result.\n";
    return 0;
}
