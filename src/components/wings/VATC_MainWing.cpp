#include "navion/components/wings/VATC_MainWing.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace navion
{
namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double EPSILON_VELOCITY_MPS = 1.0e-8;
constexpr double MAX_FLAP_DEFLECTION_RAD = 40.0 * PI / 180.0;
constexpr double MAX_PROFILE_DRAG_SWEEP_RAD = 40.0 * PI / 180.0;

bool allFinite(std::initializer_list<double> values) noexcept
{
    return std::all_of(
        values.begin(),
        values.end(),
        [](double value) { return std::isfinite(value); }
    );
}

double radiansToDegrees(double radians) noexcept
{
    return radians * 180.0 / PI;
}

double degreesToRadians(double degrees) noexcept
{
    return degrees * PI / 180.0;
}

void validateLookupTable(const VATC_LookupTable& table)
{
    if (table.x.size() != table.values.size() ||
        table.x.size() < 2U)
    {
        throw std::invalid_argument(
            "MainWing lookup table has invalid dimensions: " + table.label
        );
    }

    for (std::size_t i = 0; i < table.x.size(); ++i)
    {
        if (!std::isfinite(table.x[i]) ||
            !std::isfinite(table.values[i]))
        {
            throw std::invalid_argument(
                "MainWing lookup table contains non-finite data: " + table.label
            );
        }

        if (i > 0U && table.x[i] <= table.x[i - 1U])
        {
            throw std::invalid_argument(
                "MainWing lookup-table axis must be strictly increasing: " +
                table.label
            );
        }
    }
}

double interpolate(const VATC_LookupTable& table, double query)
{
    if (!std::isfinite(query))
    {
        throw std::invalid_argument(
            "MainWing received a non-finite lookup query: " + table.label
        );
    }

    if (query < table.x.front() - 1.0e-10 ||
        query > table.x.back() + 1.0e-10)
    {
        throw std::out_of_range(
            "MainWing lookup query is outside table domain: " + table.label
        );
    }

    if (query <= table.x.front())
    {
        return table.values.front();
    }

    if (query >= table.x.back())
    {
        return table.values.back();
    }

    const std::size_t index = static_cast<std::size_t>(
        std::upper_bound(
            table.x.begin(),
            table.x.end(),
            query
        ) - table.x.begin() - 1
    );

    const double x0 = table.x[index];
    const double x1 = table.x[index + 1U];
    const double y0 = table.values[index];
    const double y1 = table.values[index + 1U];

    return y0 + (query - x0) / (x1 - x0) * (y1 - y0);
}

void validateConfig(const VATC_MainWingConfig& config)
{
    if (!allFinite({
            config.flight.rho_kg_m3,
            config.flight.speed_m_s,
            config.flight.alpha_deg,
            config.flight.beta_deg,
            config.flight.p_deg_s,
            config.flight.q_deg_s,
            config.flight.r_deg_s,
            config.controls.aileron_deg,
            config.controls.flap_deg,
            config.wing.area_m2,
            config.wing.aspect_ratio,
            config.wing.mean_chord_m,
            config.wing.taper,
            config.wing.sweep_le_deg,
            config.wing.incidence_deg,
            config.reference.aero_h,
            config.reference.aero_z_m,
            config.reference.output_h,
            config.reference.output_z_m,
            config.reference.stability_angle_deg,
            config.aero.cl_alpha_per_rad,
            config.aero.CL_alpha_per_rad,
            config.aero.CL0,
            config.aero.CD0,
            config.aero.oswald_e,
            config.aero.Cm0,
            config.aero.Cm_alpha_per_rad,
            config.flap.eta_in,
            config.flap.eta_out,
            config.flap.chord_ratio,
            config.flap.thickness_ratio,
            config.flap.section_effectiveness_per_rad,
            config.flap.effectiveness_ratio_actual,
            config.flap.effectiveness_ratio_reference,
            config.flap.induced_factor_K,
            config.flap.interference_factor,
            config.lateral.CY_beta,
            config.lateral.CY_p,
            config.lateral.CY_r,
            config.lateral.CY_da,
            config.lateral.Cl_beta,
            config.lateral.Cl_p,
            config.lateral.Cl_r,
            config.lateral.Cl_da,
            config.lateral.Cn_beta,
            config.lateral.Cn_p,
            config.lateral.Cn_r,
            config.lateral.Cn_da
        }))
    {
        throw std::invalid_argument(
            "MainWing configuration values must be finite."
        );
    }

    if (config.model.scope != "provisional_aircraft_proxy" &&
        config.model.scope != "wing_only")
    {
        throw std::invalid_argument(
            "MainWing scope must be 'provisional_aircraft_proxy' or 'wing_only'."
        );
    }

    if (config.model.derivative_axes != "body" &&
        config.model.derivative_axes != "stability")
    {
        throw std::invalid_argument(
            "MainWing derivative_axes must be 'body' or 'stability'."
        );
    }

    if (config.model.flap_moment_mode != "zero_referenced" &&
        config.model.flap_moment_mode != "literal_roskam")
    {
        throw std::invalid_argument(
            "MainWing flap_moment_mode must be 'zero_referenced' or "
            "'literal_roskam'."
        );
    }

    if (config.flight.rho_kg_m3 <= 0.0 ||
        config.flight.speed_m_s < 0.0)
    {
        throw std::invalid_argument(
            "MainWing configured flight defaults require rho > 0 and V >= 0."
        );
    }

    if (config.controls.flap_deg < 0.0 || config.controls.flap_deg > 40.0)
    {
        throw std::invalid_argument(
            "MainWing configured flap default must be in [0, 40] deg."
        );
    }

    if (config.wing.area_m2 <= 0.0 ||
        config.wing.aspect_ratio < 2.5 ||
        config.wing.mean_chord_m <= 0.0)
    {
        throw std::invalid_argument(
            "MainWing requires area > 0, aspect_ratio >= 2.5, and MAC > 0."
        );
    }

    if (config.wing.taper <= 0.0 || config.wing.taper > 1.0 ||
        config.flap.eta_in < 0.0 ||
        config.flap.eta_out <= config.flap.eta_in ||
        config.flap.eta_out > 1.0)
    {
        throw std::invalid_argument(
            "MainWing has invalid taper ratio or flap span stations."
        );
    }

    if (config.flap.chord_ratio <= 0.0 ||
        config.flap.chord_ratio >= 1.0 ||
        config.flap.thickness_ratio <= 0.0)
    {
        throw std::invalid_argument(
            "MainWing requires valid flap chord and thickness ratios."
        );
    }

    if (config.aero.cl_alpha_per_rad <= 0.0 ||
        config.aero.CL_alpha_per_rad <= 0.0 ||
        config.aero.CD0 < 0.0 ||
        config.aero.oswald_e <= 0.0 ||
        config.aero.oswald_e > 1.5 ||
        config.flap.section_effectiveness_per_rad <= 0.0 ||
        config.flap.effectiveness_ratio_actual <= 0.0 ||
        config.flap.effectiveness_ratio_reference <= 0.0 ||
        config.flap.induced_factor_K < 0.0)
    {
        throw std::invalid_argument(
            "MainWing aerodynamic coefficients/effectiveness values are invalid."
        );
    }

    validateLookupTable(config.tables.kprime);
    validateLookupTable(config.tables.profile_drag);
    validateLookupTable(config.tables.Kb_cumulative);
    validateLookupTable(config.tables.Kp_cumulative);
    validateLookupTable(config.tables.Klambda_cumulative);
    validateLookupTable(config.tables.moment_ratio);
}

void validateContext(const EvaluationContext& context)
{
    if (!context.flightCondition.isFinite() ||
        !context.state.angularRateBodyRadps.isFinite() ||
        !std::isfinite(context.controls.aileronRad) ||
        !std::isfinite(context.controls.flapRad) ||
        !std::isfinite(context.environment.airDensityKgM3) ||
        context.environment.airDensityKgM3 <= 0.0)
    {
        throw std::invalid_argument(
            "MainWing received an invalid EvaluationContext."
        );
    }
}

} // namespace

struct VATC_MainWing::WorkingData
{
    // ---------------------------------------------------------------------
    // Default runtime condition copied from navion_model_direct_plots(5).cpp
    // rho = 1.0556 kg/m^3, V = 43.9 m/s,
    // alpha = beta = p = q = r = delta_a = 0,
    // delta_f = 20 deg.
    // These values are subsequently overwritten by readRuntimeInputs(context)
    // during normal VATC component evaluation.
    // ---------------------------------------------------------------------
    double airDensityKgM3{1.0556};
    double airspeedMps{43.9};
    double angleOfAttackRad{0.0};
    double sideSlipRad{0.0};
    double rollRateRadps{0.0};
    double pitchRateRadps{0.0};
    double yawRateRadps{0.0};
    double aileronDeflectionRad{0.0};
    double flapDeflectionRad{0.3490658503988659};  // 20 deg
    double flapDeflectionDeg{20.0};

    // Geometry-derived values for the old default Navion input set.
    // b = sqrt(S*AR)
    // sweep_c/4 = atan(tan(sweep_LE) - (1-lambda)/(AR*(1+lambda)))
    // S_wf from the original trapezoidal-wing expression
    // alpha_w = alpha + i_w
    double wingSpanM{10.166438904552567};
    double quarterChordSweepAngleRad{0.0028839092258502973};
    double flappedWingAreaM2{9.482981485714285};
    double wingAngleOfAttackRad{0.03490658503988659};

    // qbar = 0.5*rho*V^2
    double dynamicPressurePa{1017.181438};

    // Flap lookup/geometry values at delta_f = 20 deg and eta = 0.13..0.65.
    double flapNonlinearityFactor{0.88};
    double profileDragTableValue{0.02911111111111111};
    double liftSpanFactor{0.6004999999999999};
    double pitchSpanFactor{0.61875};
    double sweepPitchFactor{0.020649999999999995};
    double projectedChordRatio{0.9873354503650408};
    double flapChordToProjectedChordRatio{0.2126936695348659};
    double thicknessToProjectedChordRatio{0.10128269977850758};
    double pitchMomentRatio{-0.3013992918418697};

    // Roskam reference-wing calculation.
    double sectionSlopeParameter{0.954929658551372};
    double referenceLiftCurveSlope{4.386771735382717};

    // Lift.
    double sectionFlapLiftCoefficientIncrement{1.1211995114811573};
    double flapLiftCoefficientIncrement{0.6455636006875723};
    double referenceFlapLiftCoefficientIncrement{0.9017151598932495};
    double cleanLiftCoefficient{0.3825614397586069};
    double liftCoefficient{1.0281250404461793};

    // Clean pitch and drag.
    double cleanPitchMomentCoefficient{-0.0439822971502571};
    double profileDragCoefficientIncrement{0.016132478945144947};
    double flapInducedDragCoefficientIncrement{0.026046914342705396};
    double interferenceDragCoefficientIncrement{0.0};
    double flapDragCoefficientIncrement{0.04217939328785034};
    double dragCoefficient{0.14181238828955073};

    // Roskam pitch terms.
    double pitchMomentTerm1{0.0};
    double pitchMomentTerm2{0.0002162309703057396};
    double pitchMomentTerm3{-0.16392918211822846};
    double pitchMomentTerm4{0.0007399645172175379};
    double pitchMomentTerm5{0.0006849423986899577};
    double literalFlapPitchMomentCoefficientIncrement{-0.16228804423201523};
    double zeroFlapPitchMomentBaseline{0.0};
    double flapPitchMomentCoefficientIncrement{-0.16228804423201523};
    double pitchMomentCoefficientAtAerodynamicReference{-0.20627034138227235};

    // Lateral-rate transformation.
    double derivativeFrameRollRateRadps{0.0};
    double derivativeFrameYawRateRadps{0.0};
    double normalizedRollRate{0.0};
    double normalizedPitchRate{0.0};
    double normalizedYawRate{0.0};

    // Lateral coefficients at aerodynamic reference before CG transfer.
    double sideForceCoefficient{0.0};
    double rollMomentCoefficientAtAerodynamicReference{0.0};
    double yawMomentCoefficientAtAerodynamicReference{0.0};

    // Longitudinal BODY coefficients.
    double axialForceCoefficient{-0.14181238828955073};
    double normalForceCoefficient{-1.0281250404461793};

    // Reference transfer for the old defaults: aero_h = output_h = 0.25,
    // aero_z = output_z = 0, therefore dx = dz = 0.
    double momentTransferDxM{0.0};
    double momentTransferDzM{0.0};

    // Final BODY moment coefficients about the configured CG/output reference.
    double rollMomentCoefficient{0.0};
    double pitchMomentCoefficient{-0.20627034138227235};
    double yawMomentCoefficient{0.0};

    double upwardNormalForceCoefficient{1.0281250404461793};

    // Dimensional loads for the old default condition.
    Vec3 forceBodyN{
        -2468.3876738450695,
        0.0,
        -17895.553467636
    };

    Vec3 intrinsicMomentAtAerodynamicReferenceBodyNm{
        0.0,
        -6247.197464641232,
        0.0
    };

    Vec3 momentAboutCgBodyNm{
        0.0,
        -6247.197464641232,
        0.0
    };
};

VATC_MainWing::VATC_MainWing(const VATC_MainWingConfig& config)
    : config_(config)
{
    validateConfig(config_);
}

BodyLoad VATC_MainWing::computeLoad(
    const EvaluationContext& context
) const
{
    return evaluateDetailed(context).bodyLoad;
}

std::string_view VATC_MainWing::name() const noexcept
{
    return "MainWing";
}

MainWingEvaluation VATC_MainWing::evaluateDetailed(
    const EvaluationContext& context
) const
{
    validateContext(context);

    WorkingData data;

    // Same logical order as the original calculate(const Inputs&) function,
    // split into VATC-style OOP calculation stages.
    readRuntimeInputs(context, data);
    calculateGeometry(data);
    calculateFlapLookupData(data);
    calculateLongitudinalAerodynamics(data);
    calculateLateralAerodynamics(data);
    calculateBodyAxisCoefficients(data);
    calculateDimensionalLoads(data);

    MainWingEvaluation result;

    result.bodyLoad.forceBodyN = data.forceBodyN;
    result.bodyLoad.momentAboutCgBodyNm = data.momentAboutCgBodyNm;

    result.forceBodyN = data.forceBodyN;
    result.intrinsicMomentAtAerodynamicReferenceBodyNm =
        data.intrinsicMomentAtAerodynamicReferenceBodyNm;
    result.momentAboutCgBodyNm = data.momentAboutCgBodyNm;

    result.airDensityKgM3 = data.airDensityKgM3;
    result.airspeedMps = data.airspeedMps;
    result.angleOfAttackRad = data.angleOfAttackRad;
    result.sideSlipRad = data.sideSlipRad;
    result.rollRateRadps = data.rollRateRadps;
    result.pitchRateRadps = data.pitchRateRadps;
    result.yawRateRadps = data.yawRateRadps;
    result.aileronDeflectionRad = data.aileronDeflectionRad;
    result.flapDeflectionRad = data.flapDeflectionRad;
    result.flapDeflectionDeg = data.flapDeflectionDeg;
    result.dynamicPressurePa = data.dynamicPressurePa;

    result.wingSpanM = data.wingSpanM;
    result.wingAngleOfAttackRad = data.wingAngleOfAttackRad;
    result.quarterChordSweepAngleRad = data.quarterChordSweepAngleRad;
    result.flappedWingAreaM2 = data.flappedWingAreaM2;
    result.projectedChordRatio = data.projectedChordRatio;
    result.flapChordToProjectedChordRatio =
        data.flapChordToProjectedChordRatio;
    result.thicknessToProjectedChordRatio =
        data.thicknessToProjectedChordRatio;
    result.flapNonlinearityFactor = data.flapNonlinearityFactor;
    result.profileDragTableValue = data.profileDragTableValue;
    result.liftSpanFactor = data.liftSpanFactor;
    result.pitchSpanFactor = data.pitchSpanFactor;
    result.sweepPitchFactor = data.sweepPitchFactor;
    result.pitchMomentRatio = data.pitchMomentRatio;
    result.sectionSlopeParameter = data.sectionSlopeParameter;
    result.referenceLiftCurveSlope = data.referenceLiftCurveSlope;
    result.sectionFlapLiftCoefficientIncrement =
        data.sectionFlapLiftCoefficientIncrement;
    result.flapLiftCoefficientIncrement = data.flapLiftCoefficientIncrement;
    result.referenceFlapLiftCoefficientIncrement =
        data.referenceFlapLiftCoefficientIncrement;
    result.cleanLiftCoefficient = data.cleanLiftCoefficient;
    result.liftCoefficient = data.liftCoefficient;
    result.cleanPitchMomentCoefficient = data.cleanPitchMomentCoefficient;
    result.dragCoefficient = data.dragCoefficient;
    result.profileDragCoefficientIncrement =
        data.profileDragCoefficientIncrement;
    result.flapInducedDragCoefficientIncrement =
        data.flapInducedDragCoefficientIncrement;
    result.interferenceDragCoefficientIncrement =
        data.interferenceDragCoefficientIncrement;
    result.flapDragCoefficientIncrement = data.flapDragCoefficientIncrement;
    result.pitchMomentTerm1 = data.pitchMomentTerm1;
    result.pitchMomentTerm2 = data.pitchMomentTerm2;
    result.pitchMomentTerm3 = data.pitchMomentTerm3;
    result.pitchMomentTerm4 = data.pitchMomentTerm4;
    result.pitchMomentTerm5 = data.pitchMomentTerm5;
    result.literalFlapPitchMomentCoefficientIncrement =
        data.literalFlapPitchMomentCoefficientIncrement;
    result.zeroFlapPitchMomentBaseline = data.zeroFlapPitchMomentBaseline;
    result.flapPitchMomentCoefficientIncrement =
        data.flapPitchMomentCoefficientIncrement;
    result.pitchMomentCoefficientAtAerodynamicReference =
        data.pitchMomentCoefficientAtAerodynamicReference;
    result.derivativeFrameRollRateRadps = data.derivativeFrameRollRateRadps;
    result.derivativeFrameYawRateRadps = data.derivativeFrameYawRateRadps;
    result.normalizedRollRate = data.normalizedRollRate;
    result.normalizedPitchRate = data.normalizedPitchRate;
    result.normalizedYawRate = data.normalizedYawRate;
    result.upwardNormalForceCoefficient =
        data.upwardNormalForceCoefficient;

    result.axialForceCoefficient = data.axialForceCoefficient;
    result.sideForceCoefficient = data.sideForceCoefficient;
    result.normalForceCoefficient = data.normalForceCoefficient;
    result.rollMomentCoefficientAtAerodynamicReference =
        data.rollMomentCoefficientAtAerodynamicReference;
    result.yawMomentCoefficientAtAerodynamicReference =
        data.yawMomentCoefficientAtAerodynamicReference;
    result.rollMomentCoefficient = data.rollMomentCoefficient;
    result.pitchMomentCoefficient = data.pitchMomentCoefficient;
    result.yawMomentCoefficient = data.yawMomentCoefficient;
    result.momentTransferDxM = data.momentTransferDxM;
    result.momentTransferDzM = data.momentTransferDzM;

    // Preserve the warnings from the standalone model.
    result.warnings.push_back(
        "Roskam table samples are coarse estimates for the supplied geometry; "
        "retabulate after geometry changes."
    );

    if (config_.model.scope == "provisional_aircraft_proxy")
    {
        result.warnings.push_back(
            "NASA whole-aircraft initial derivatives used as proxy: outputs "
            "are NOT validated isolated-wing coefficients."
        );
    }

    if (std::abs(data.angleOfAttackRad) > 10.0 * PI / 180.0 ||
        std::abs(data.sideSlipRad) > 5.0 * PI / 180.0)
    {
        result.warnings.push_back(
            "Outside nominal small-angle demonstration range."
        );
    }

    if (std::abs(data.sideSlipRad) > 1.0e-12)
    {
        result.warnings.push_back(
            "Longitudinal force transform neglects beta coupling, as in the "
            "supplied reduced model."
        );
    }

    if (std::abs(data.pitchRateRadps) > 1.0e-12)
    {
        result.warnings.push_back(
            "Pitch-rate q is reported but does not enter the supplied static "
            "pitch model."
        );
    }

    if (config_.model.flap_moment_mode == "literal_roskam" &&
        std::abs(config_.reference.aero_h - 0.25) > 1.0e-12)
    {
        result.warnings.push_back(
            "Literal Eq.8.74 has a nonzero zero-flap baseline away from "
            "quarter chord."
        );
    }

    if (!result.bodyLoad.isFinite() ||
        !result.forceBodyN.isFinite() ||
        !result.intrinsicMomentAtAerodynamicReferenceBodyNm.isFinite() ||
        !result.momentAboutCgBodyNm.isFinite() ||
        !allFinite({
            result.airDensityKgM3,
            result.airspeedMps,
            result.angleOfAttackRad,
            result.sideSlipRad,
            result.rollRateRadps,
            result.pitchRateRadps,
            result.yawRateRadps,
            result.aileronDeflectionRad,
            result.flapDeflectionRad,
            result.flapDeflectionDeg,
            result.dynamicPressurePa,
            result.wingSpanM,
            result.wingAngleOfAttackRad,
            result.quarterChordSweepAngleRad,
            result.flappedWingAreaM2,
            result.projectedChordRatio,
            result.flapChordToProjectedChordRatio,
            result.thicknessToProjectedChordRatio,
            result.flapNonlinearityFactor,
            result.profileDragTableValue,
            result.liftSpanFactor,
            result.pitchSpanFactor,
            result.sweepPitchFactor,
            result.pitchMomentRatio,
            result.sectionSlopeParameter,
            result.referenceLiftCurveSlope,
            result.sectionFlapLiftCoefficientIncrement,
            result.flapLiftCoefficientIncrement,
            result.referenceFlapLiftCoefficientIncrement,
            result.cleanLiftCoefficient,
            result.liftCoefficient,
            result.cleanPitchMomentCoefficient,
            result.dragCoefficient,
            result.profileDragCoefficientIncrement,
            result.flapInducedDragCoefficientIncrement,
            result.interferenceDragCoefficientIncrement,
            result.flapDragCoefficientIncrement,
            result.pitchMomentTerm1,
            result.pitchMomentTerm2,
            result.pitchMomentTerm3,
            result.pitchMomentTerm4,
            result.pitchMomentTerm5,
            result.literalFlapPitchMomentCoefficientIncrement,
            result.zeroFlapPitchMomentBaseline,
            result.flapPitchMomentCoefficientIncrement,
            result.pitchMomentCoefficientAtAerodynamicReference,
            result.derivativeFrameRollRateRadps,
            result.derivativeFrameYawRateRadps,
            result.normalizedRollRate,
            result.normalizedPitchRate,
            result.normalizedYawRate,
            result.upwardNormalForceCoefficient,
            result.axialForceCoefficient,
            result.sideForceCoefficient,
            result.normalForceCoefficient,
            result.rollMomentCoefficientAtAerodynamicReference,
            result.yawMomentCoefficientAtAerodynamicReference,
            result.rollMomentCoefficient,
            result.pitchMomentCoefficient,
            result.yawMomentCoefficient,
            result.momentTransferDxM,
            result.momentTransferDzM
        }))
    {
        throw std::runtime_error("MainWing produced a non-finite result.");
    }

    return result;
}

const VATC_MainWingConfig& VATC_MainWing::getConfig() const noexcept
{
    return config_;
}

void VATC_MainWing::readRuntimeInputs(
    const EvaluationContext& context,
    WorkingData& data
) const
{
    data.airDensityKgM3 = context.environment.airDensityKgM3;
    data.airspeedMps = context.flightCondition.airspeedMps;
    data.angleOfAttackRad = context.flightCondition.angleOfAttackRad;
    data.sideSlipRad = context.flightCondition.sideSlipRad;

    data.rollRateRadps = context.state.angularRateBodyRadps.x;
    data.pitchRateRadps = context.state.angularRateBodyRadps.y;
    data.yawRateRadps = context.state.angularRateBodyRadps.z;

    data.aileronDeflectionRad = context.controls.aileronRad;
    data.flapDeflectionRad = context.controls.flapRad;
    data.flapDeflectionDeg = radiansToDegrees(data.flapDeflectionRad);

    if (data.airspeedMps < 0.0)
    {
        throw std::invalid_argument("MainWing requires nonnegative airspeed.");
    }

    if (data.flapDeflectionRad < 0.0 ||
        data.flapDeflectionRad > MAX_FLAP_DEFLECTION_RAD)
    {
        throw std::invalid_argument(
            "MainWing plain flap requires 0 <= flap deflection <= 40 deg."
        );
    }
}

void VATC_MainWing::calculateGeometry(WorkingData& data) const
{
    // Original:
    // b = sqrt(S*AR)
    data.wingSpanM = std::sqrt(config_.wing.area_m2 * config_.wing.aspect_ratio);

    // Original:
    // sweep = atan(tan(le) - (1-taper)/(AR*(1+taper)))
    data.quarterChordSweepAngleRad = std::atan(
        std::tan(degreesToRadians(config_.wing.sweep_le_deg)) -
        (1.0 - config_.wing.taper) /
            (config_.wing.aspect_ratio * (1.0 + config_.wing.taper))
    );

    if (std::abs(data.quarterChordSweepAngleRad) >
        MAX_PROFILE_DRAG_SWEEP_RAD)
    {
        throw std::invalid_argument(
            "MainWing sweep is outside the profile-drag method range."
        );
    }

    // Original:
    // swf = S*2/(1+taper)*((eta_o-eta_i)
    //       -.5*(1-taper)*(eta_o^2-eta_i^2))
    const double etaIn = config_.flap.eta_in;
    const double etaOut = config_.flap.eta_out;

    data.flappedWingAreaM2 =
        config_.wing.area_m2 * 2.0 / (1.0 + config_.wing.taper) *
        (
            (etaOut - etaIn) -
            0.5 * (1.0 - config_.wing.taper) *
                (etaOut * etaOut - etaIn * etaIn)
        );

    // Original: aw = a + iw
    data.wingAngleOfAttackRad =
        data.angleOfAttackRad + degreesToRadians(config_.wing.incidence_deg);

    // Original: qbar = 0.5*rho*V^2
    data.dynamicPressurePa =
        0.5 * data.airDensityKgM3 *
        data.airspeedMps * data.airspeedMps;
}

void VATC_MainWing::calculateFlapLookupData(WorkingData& data) const
{
    // Original: kpnon = table(kprime, dfdeg)
    data.flapNonlinearityFactor =
        interpolate(config_.tables.kprime, data.flapDeflectionDeg);

    // Original: cdp = table(profile_drag, dfdeg)
    data.profileDragTableValue =
        interpolate(config_.tables.profile_drag, data.flapDeflectionDeg);

    // Original: Kb = B(eta_o) - B(eta_i)
    data.liftSpanFactor =
        interpolate(
            config_.tables.Kb_cumulative,
            config_.flap.eta_out
        ) -
        interpolate(
            config_.tables.Kb_cumulative,
            config_.flap.eta_in
        );

    // Original: Kp = P(eta_o) - P(eta_i)
    data.pitchSpanFactor =
        interpolate(
            config_.tables.Kp_cumulative,
            config_.flap.eta_out
        ) -
        interpolate(
            config_.tables.Kp_cumulative,
            config_.flap.eta_in
        );

    // Original: Klambda = Acurve(eta_o) - Acurve(eta_i)
    data.sweepPitchFactor =
        interpolate(
            config_.tables.Klambda_cumulative,
            config_.flap.eta_out
        ) -
        interpolate(
            config_.tables.Klambda_cumulative,
            config_.flap.eta_in
        );

    // Original: rc = 1 - fc*(1-cos(df))
    data.projectedChordRatio =
        1.0 - config_.flap.chord_ratio *
        (1.0 - std::cos(data.flapDeflectionRad));

    if (data.projectedChordRatio <= 0.0)
    {
        throw std::runtime_error(
            "MainWing produced a nonpositive projected chord ratio."
        );
    }

    // Original: ratio = fc/rc
    data.flapChordToProjectedChordRatio =
        config_.flap.chord_ratio / data.projectedChordRatio;

    // Original intermediate: t_over_cprime = tc/rc
    data.thicknessToProjectedChordRatio =
        config_.flap.thickness_ratio / data.projectedChordRatio;

    // Original: Rm = table(moment_ratio, ratio)
    data.pitchMomentRatio = interpolate(
        config_.tables.moment_ratio,
        data.flapChordToProjectedChordRatio
    );

    if (data.liftSpanFactor < 0.0 ||
        data.pitchSpanFactor < 0.0 ||
        data.flapNonlinearityFactor < 0.0 ||
        data.profileDragTableValue < 0.0)
    {
        throw std::runtime_error(
            "MainWing produced invalid interpolated flap data."
        );
    }
}

void VATC_MainWing::calculateLongitudinalAerodynamics(
    WorkingData& data
) const
{
    // Roskam 8.22 reference-wing slope, exactly as in the standalone model.
    // Original: k = clalpha/(2*pi)
    data.sectionSlopeParameter =
        config_.aero.cl_alpha_per_rad / (2.0 * PI);

    // Original:
    // cla_ref = 2*pi*6/(2 + sqrt(4 + 36/(k*k)))
    data.referenceLiftCurveSlope =
        2.0 * PI * 6.0 /
        (
            2.0 +
            std::sqrt(
                4.0 + 36.0 /
                    (data.sectionSlopeParameter * data.sectionSlopeParameter)
            )
        );

    // Original:
    // dcl = df * eff * kpnon
    data.sectionFlapLiftCoefficientIncrement =
        data.flapDeflectionRad *
        config_.flap.section_effectiveness_per_rad *
        data.flapNonlinearityFactor;

    // Original:
    // dCL = Kb*dcl*(cla/clalpha)*Ract
    data.flapLiftCoefficientIncrement =
        data.liftSpanFactor *
        data.sectionFlapLiftCoefficientIncrement *
        (config_.aero.CL_alpha_per_rad / config_.aero.cl_alpha_per_rad) *
        config_.flap.effectiveness_ratio_actual;

    // Original:
    // dCLref = dcl*(cla_ref/clalpha)*Rref
    data.referenceFlapLiftCoefficientIncrement =
        data.sectionFlapLiftCoefficientIncrement *
        (data.referenceLiftCurveSlope / config_.aero.cl_alpha_per_rad) *
        config_.flap.effectiveness_ratio_reference;

    // Original: CLup = cl0 + cla*aw
    data.cleanLiftCoefficient =
        config_.aero.CL0 +
        config_.aero.CL_alpha_per_rad * data.wingAngleOfAttackRad;

    // Original: Cmup = cm0 + cma*aw
    data.cleanPitchMomentCoefficient =
        config_.aero.Cm0 +
        config_.aero.Cm_alpha_per_rad * data.wingAngleOfAttackRad;

    // Original: CL = CLup + dCL
    data.liftCoefficient =
        data.cleanLiftCoefficient + data.flapLiftCoefficientIncrement;

    // Original:
    // dp = cdp*cos(sweep)*swf/S
    data.profileDragCoefficientIncrement =
        data.profileDragTableValue *
        std::cos(data.quarterChordSweepAngleRad) *
        data.flappedWingAreaM2 / config_.wing.area_m2;

    // Original:
    // di = K*K*dCL*dCL*cos(sweep)
    data.flapInducedDragCoefficientIncrement =
        config_.flap.induced_factor_K *
        config_.flap.induced_factor_K *
        data.flapLiftCoefficientIncrement *
        data.flapLiftCoefficientIncrement *
        std::cos(data.quarterChordSweepAngleRad);

    // Original: kint*dp
    data.interferenceDragCoefficientIncrement =
        config_.flap.interference_factor *
        data.profileDragCoefficientIncrement;

    // Original: dD = dp + di + kint*dp
    data.flapDragCoefficientIncrement =
        data.profileDragCoefficientIncrement +
        data.flapInducedDragCoefficientIncrement +
        data.interferenceDragCoefficientIncrement;

    // Original:
    // CD = cd0 + CL^2/(pi*e*AR) + dD
    data.dragCoefficient =
        config_.aero.CD0 +
        data.liftCoefficient * data.liftCoefficient /
            (PI * config_.aero.oswald_e * config_.wing.aspect_ratio) +
        data.flapDragCoefficientIncrement;

    // Roskam Eq. 8.74 terms, preserved exactly.
    // T1 = (h-.25)*CL
    data.pitchMomentTerm1 =
        (config_.reference.aero_h - 0.25) *
        data.liftCoefficient;

    // T2 = Klambda*(AR/1.5)*dCLref*tan(sweep)
    data.pitchMomentTerm2 =
        data.sweepPitchFactor *
        (config_.wing.aspect_ratio / 1.5) *
        data.referenceFlapLiftCoefficientIncrement *
        std::tan(data.quarterChordSweepAngleRad);

    // T3 = Kp*Rm*dCLref*rc^2
    data.pitchMomentTerm3 =
        data.pitchSpanFactor *
        data.pitchMomentRatio *
        data.referenceFlapLiftCoefficientIncrement *
        data.projectedChordRatio * data.projectedChordRatio;

    // T4 = -Kp*.25*CLup*(rc^2-rc)
    data.pitchMomentTerm4 =
        -data.pitchSpanFactor * 0.25 *
        data.cleanLiftCoefficient *
        (
            data.projectedChordRatio * data.projectedChordRatio -
            data.projectedChordRatio
        );

    // T5 = Kp*Cmup*(rc^2-1)
    data.pitchMomentTerm5 =
        data.pitchSpanFactor *
        data.cleanPitchMomentCoefficient *
        (
            data.projectedChordRatio * data.projectedChordRatio - 1.0
        );

    // Original: raw = T1+T2+T3+T4+T5
    data.literalFlapPitchMomentCoefficientIncrement =
        data.pitchMomentTerm1 +
        data.pitchMomentTerm2 +
        data.pitchMomentTerm3 +
        data.pitchMomentTerm4 +
        data.pitchMomentTerm5;

    // Original: baseline = (h-.25)*CLup
    data.zeroFlapPitchMomentBaseline =
        (config_.reference.aero_h - 0.25) *
        data.cleanLiftCoefficient;

    // Original:
    // dCm = raw - (mode=="zero_referenced" ? baseline : 0)
    data.flapPitchMomentCoefficientIncrement =
        data.literalFlapPitchMomentCoefficientIncrement -
        (
            config_.model.flap_moment_mode == "zero_referenced"
                ? data.zeroFlapPitchMomentBaseline
                : 0.0
        );

    // Original: Cm_at_aero_reference = Cmup + dCm
    data.pitchMomentCoefficientAtAerodynamicReference =
        data.cleanPitchMomentCoefficient +
        data.flapPitchMomentCoefficientIncrement;
}

void VATC_MainWing::calculateLateralAerodynamics(
    WorkingData& data
) const
{
    // Original defaults: p = pb, rr = rb
    data.derivativeFrameRollRateRadps = data.rollRateRadps;
    data.derivativeFrameYawRateRadps = data.yawRateRadps;

    // Original stability-axis conversion, preserved exactly.
    if (config_.model.derivative_axes == "stability")
    {
        const double cosine = std::cos(degreesToRadians(config_.reference.stability_angle_deg));
        const double sine = std::sin(degreesToRadians(config_.reference.stability_angle_deg));

        data.derivativeFrameRollRateRadps =
            cosine * data.rollRateRadps + sine * data.yawRateRadps;

        data.derivativeFrameYawRateRadps =
            -sine * data.rollRateRadps + cosine * data.yawRateRadps;
    }

    data.normalizedRollRate = 0.0;
    data.normalizedPitchRate = 0.0;
    data.normalizedYawRate = 0.0;

    if (data.airspeedMps > EPSILON_VELOCITY_MPS)
    {
        // Original: ph = p*b/(2V)
        data.normalizedRollRate =
            data.derivativeFrameRollRateRadps *
            data.wingSpanM / (2.0 * data.airspeedMps);

        // Original q_hat_unused = qb*c/(2V)
        data.normalizedPitchRate =
            data.pitchRateRadps *
            config_.wing.mean_chord_m /
            (2.0 * data.airspeedMps);

        // Original: rh = rr*b/(2V)
        data.normalizedYawRate =
            data.derivativeFrameYawRateRadps *
            data.wingSpanM / (2.0 * data.airspeedMps);
    }

    // Original CY equation.
    data.sideForceCoefficient =
        config_.lateral.CY_beta * data.sideSlipRad +
        config_.lateral.CY_p * data.normalizedRollRate +
        config_.lateral.CY_r * data.normalizedYawRate +
        config_.lateral.CY_da * data.aileronDeflectionRad;

    // Original Cl equation.
    data.rollMomentCoefficientAtAerodynamicReference =
        config_.lateral.Cl_beta * data.sideSlipRad +
        config_.lateral.Cl_p * data.normalizedRollRate +
        config_.lateral.Cl_r * data.normalizedYawRate +
        config_.lateral.Cl_da * data.aileronDeflectionRad;

    // Original Cn equation.
    data.yawMomentCoefficientAtAerodynamicReference =
        config_.lateral.Cn_beta * data.sideSlipRad +
        config_.lateral.Cn_p * data.normalizedRollRate +
        config_.lateral.Cn_r * data.normalizedYawRate +
        config_.lateral.Cn_da * data.aileronDeflectionRad;

    // Rotate Cl/Cn back into BODY axes when derivatives were supplied in
    // stability axes. This is the same two-line transformation as before.
    if (config_.model.derivative_axes == "stability")
    {
        const double rollStability =
            data.rollMomentCoefficientAtAerodynamicReference;
        const double yawStability =
            data.yawMomentCoefficientAtAerodynamicReference;

        const double cosine = std::cos(degreesToRadians(config_.reference.stability_angle_deg));
        const double sine = std::sin(degreesToRadians(config_.reference.stability_angle_deg));

        data.rollMomentCoefficientAtAerodynamicReference =
            cosine * rollStability - sine * yawStability;

        data.yawMomentCoefficientAtAerodynamicReference =
            sine * rollStability + cosine * yawStability;
    }
}

void VATC_MainWing::calculateBodyAxisCoefficients(
    WorkingData& data
) const
{
    // Reduced longitudinal transformation from the supplied standalone model.
    // beta coupling remains deliberately omitted.
    // Original: CX = -CD*cos(a) + CL*sin(a)
    data.axialForceCoefficient =
        -data.dragCoefficient * std::cos(data.angleOfAttackRad) +
        data.liftCoefficient * std::sin(data.angleOfAttackRad);

    // Original: CZ = -CL*cos(a) - CD*sin(a)
    data.normalForceCoefficient =
        -data.liftCoefficient * std::cos(data.angleOfAttackRad) -
        data.dragCoefficient * std::sin(data.angleOfAttackRad);

    // h is positive aft while BODY x is positive forward.
    // Original: dx = (hout-h)*c
    data.momentTransferDxM =
        (config_.reference.output_h -
         config_.reference.aero_h) *
        config_.wing.mean_chord_m;

    // Original: dz = z-zout
    data.momentTransferDzM =
        config_.reference.aero_z_m - config_.reference.output_z_m;

    // Preserve the original coefficient-level transfer exactly.
    // Original: CM = Cmup+dCm + dz/c*CX - dx/c*CZ
    data.pitchMomentCoefficient =
        data.pitchMomentCoefficientAtAerodynamicReference +
        data.momentTransferDzM / config_.wing.mean_chord_m *
            data.axialForceCoefficient -
        data.momentTransferDxM / config_.wing.mean_chord_m *
            data.normalForceCoefficient;

    // Original: Cl -= dz/b*CY
    data.rollMomentCoefficient =
        data.rollMomentCoefficientAtAerodynamicReference -
        data.momentTransferDzM / data.wingSpanM *
            data.sideForceCoefficient;

    // Original: Cn += dx/b*CY
    data.yawMomentCoefficient =
        data.yawMomentCoefficientAtAerodynamicReference +
        data.momentTransferDxM / data.wingSpanM *
            data.sideForceCoefficient;

    // Original intermediate: normal_up_coefficient = -CZ
    data.upwardNormalForceCoefficient = -data.normalForceCoefficient;
}

void VATC_MainWing::calculateDimensionalLoads(
    WorkingData& data
) const
{
    const double qS = data.dynamicPressurePa * config_.wing.area_m2;

    // Original dimensional forces:
    // X = qbar*S*CX; Y = qbar*S*CY; Z = qbar*S*CZ
    data.forceBodyN = {
        qS * data.axialForceCoefficient,
        qS * data.sideForceCoefficient,
        qS * data.normalForceCoefficient
    };

    // Aerodynamic-reference intrinsic moment, retained for diagnostics.
    data.intrinsicMomentAtAerodynamicReferenceBodyNm = {
        qS * data.wingSpanM *
            data.rollMomentCoefficientAtAerodynamicReference,
        qS * config_.wing.mean_chord_m *
            data.pitchMomentCoefficientAtAerodynamicReference,
        qS * data.wingSpanM *
            data.yawMomentCoefficientAtAerodynamicReference
    };

    // Original dimensional moments AFTER the coefficient-level transfer:
    // L = qbar*S*b*Cl; M = qbar*S*c*CM; N = qbar*S*b*Cn
    data.momentAboutCgBodyNm = {
        qS * data.wingSpanM * data.rollMomentCoefficient,
        qS * config_.wing.mean_chord_m * data.pitchMomentCoefficient,
        qS * data.wingSpanM * data.yawMomentCoefficient
    };
}

} // namespace navion
