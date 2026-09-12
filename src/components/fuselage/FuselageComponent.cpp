#include "navion/components/fuselage/FuselageComponent.hpp"

#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace navion::fuselage
{
namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double ZERO_AIRSPEED_TOLERANCE_MPS = 1.0e-10;

[[nodiscard]] constexpr double degreesToRadians(double degrees) noexcept
{
    return degrees * PI / 180.0;
}

[[nodiscard]] bool allFinite(std::initializer_list<double> values) noexcept
{
    for (const double value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

void validateConfig(const FuselageComponentConfig& config)
{
    const auto& g = config.geometry;
    if (!allFinite({
            g.Lf_m, g.df_m, g.Ln_m, g.Lc_m, g.Lt_m, g.db_m,
            g.theta_rad, g.psi_rad, g.Sw_m2, g.mac_m, g.b_m
        }) ||
        g.Lf_m <= 0.0 || g.df_m <= 0.0 ||
        g.Ln_m < 0.0 || g.Lc_m < 0.0 || g.Lt_m < 0.0 ||
        g.db_m < 0.0 || g.db_m > g.df_m ||
        g.Sw_m2 <= 0.0 || g.mac_m <= 0.0 || g.b_m < 0.0)
    {
        throw std::invalid_argument(
            "FuselageComponent received invalid geometry."
        );
    }

    const auto& t = config.tuning;
    const auto& p = t.pitch;
    const auto& y = t.yaw;
    if (!allFinite({
            t.windshield_drag_increment,
            p.fineness_ratio_ref,
            p.nose_fineness_ratio_ref,
            p.windshield_angle_ref_rad,
            p.tail_fineness_ratio_ref,
            p.upsweep_angle_ref_rad,
            p.cm0_fineness_ratio_anchor,
            p.cm0_fineness_ratio_sensitivity,
            p.cm0_nose_correction_anchor,
            p.cm0_nose_windshield_angle_sensitivity_per_rad,
            p.cm0_nose_fineness_ratio_sensitivity,
            p.cm0_tail_correction_anchor,
            p.cm0_tail_upsweep_angle_sensitivity_per_rad,
            p.cm0_tail_fineness_ratio_sensitivity,
            p.cmalpha_fineness_ratio_anchor,
            p.cmalpha_fineness_ratio_sensitivity,
            p.cmalpha_nose_correction_anchor,
            p.cmalpha_nose_windshield_angle_sensitivity_per_rad,
            p.cmalpha_nose_fineness_ratio_sensitivity,
            p.cmalpha_tail_correction_anchor,
            p.cmalpha_tail_upsweep_angle_sensitivity_per_rad,
            p.cmalpha_tail_fineness_ratio_sensitivity,
            y.fineness_ratio_ref,
            y.nose_fineness_ratio_ref,
            y.tail_fineness_ratio_ref,
            y.cnbeta_fineness_ratio_anchor,
            y.cnbeta_fineness_ratio_sensitivity,
            y.cnbeta_nose_correction_anchor,
            y.cnbeta_nose_fineness_ratio_sensitivity,
            y.cnbeta_tail_correction_anchor,
            y.cnbeta_tail_fineness_ratio_sensitivity
        }) || !config.aerodynamicReferencePositionFromCgBodyM.isFinite())
    {
        throw std::invalid_argument(
            "FuselageComponent received non-finite tuning or installation data."
        );
    }
}

[[nodiscard]] FuselageComponentConfig validated(
    FuselageComponentConfig config
)
{
    validateConfig(config);
    return config;
}

void validateContext(const EvaluationContext& context)
{
    if (!context.flightCondition.isFinite() ||
        !context.environment.isFinite() ||
        context.flightCondition.airspeedMps < 0.0 ||
        context.environment.airDensityKgM3 < 0.0 ||
        context.environment.dynamicViscosityPaS <= 0.0)
    {
        throw std::invalid_argument(
            "FuselageComponent received an invalid EvaluationContext."
        );
    }
}

} // namespace

FuselageComponent::FuselageComponent(FuselageComponentConfig config)
    : config_(validated(std::move(config))),
      model_(config_.geometry, config_.tuning)
{
}

BodyLoad FuselageComponent::computeLoad(
    const EvaluationContext& context
) const
{
    return evaluateDetailed(context).bodyLoad;
}

std::string_view FuselageComponent::name() const noexcept
{
    return "Fuselage";
}

FuselageEvaluation FuselageComponent::evaluateDetailed(
    const EvaluationContext& context
) const
{
    validateContext(context);

    FuselageEvaluation result;
    result.aerodynamicState.V_mps = context.flightCondition.airspeedMps;
    result.aerodynamicState.rho_kgm3 = context.environment.airDensityKgM3;
    result.aerodynamicState.mu_pas =
        context.environment.dynamicViscosityPaS;
    result.aerodynamicState.mach = context.flightCondition.mach;
    result.aerodynamicState.alpha_rad =
        context.flightCondition.angleOfAttackRad;
    result.aerodynamicState.beta_rad =
        context.flightCondition.sideSlipRad;

    // The uploaded turbulent skin-friction expression contains log10(Re)
    // and is undefined at exactly V=0. At zero dynamic pressure every
    // dimensional aerodynamic load is exactly zero, so bypassing the kernel
    // supplies the mathematical limiting load required by takeoff start.
    if (result.aerodynamicState.V_mps <= ZERO_AIRSPEED_TOLERANCE_MPS ||
        result.aerodynamicState.rho_kgm3 == 0.0)
    {
        result.zeroDynamicPressureBypass = true;
        return result;
    }

    result.aerodynamicOutput = model_.evaluate(result.aerodynamicState);

    switch (config_.dragDirectionMode)
    {
    case DragDirectionMode::OppositeAirRelativeVelocity:
        result.unitDragDirectionBody =
            -context.flightCondition.airRelativeVelocityBodyMps /
            context.flightCondition.airspeedMps;
        break;
    case DragDirectionMode::BodyNegativeX:
        result.unitDragDirectionBody = {-1.0, 0.0, 0.0};
        break;
    default:
        throw std::logic_error("Unknown fuselage drag-direction mode.");
    }

    result.dragForceBodyN =
        result.aerodynamicOutput.drag_n * result.unitDragDirectionBody;
    result.intrinsicMomentAtReferenceBodyNm = {
        0.0,
        result.aerodynamicOutput.pitch_moment_nm,
        result.aerodynamicOutput.yaw_moment_nm
    };
    result.bodyLoad = makeBodyLoadAtPoint(
        result.dragForceBodyN,
        result.intrinsicMomentAtReferenceBodyNm,
        config_.aerodynamicReferencePositionFromCgBodyM
    );

    result.warnings.push_back(
        "Fuselage CM/CN correction slopes are uncalibrated unless the "
        "configured tuning has been replaced with validated data."
    );

    if (std::abs(
            config_.geometry.Ln_m + config_.geometry.Lc_m +
            config_.geometry.Lt_m - config_.geometry.Lf_m
        ) > 1.0e-6)
    {
        result.warnings.push_back(
            "Fuselage nose/cabin/tail lengths do not sum to total length."
        );
    }

    if (!result.bodyLoad.isFinite() ||
        !result.unitDragDirectionBody.isFinite() ||
        !result.dragForceBodyN.isFinite() ||
        !result.intrinsicMomentAtReferenceBodyNm.isFinite() ||
        !allFinite({
            result.aerodynamicOutput.reynolds,
            result.aerodynamicOutput.cf,
            result.aerodynamicOutput.cd_skin_friction,
            result.aerodynamicOutput.cd_base,
            result.aerodynamicOutput.cd_upsweep,
            result.aerodynamicOutput.cd_fuselage_sfront,
            result.aerodynamicOutput.cd_fuselage_sw,
            result.aerodynamicOutput.drag_n,
            result.aerodynamicOutput.cm0_fr,
            result.aerodynamicOutput.dcm0_nose,
            result.aerodynamicOutput.dcm0_tail,
            result.aerodynamicOutput.cm0,
            result.aerodynamicOutput.cmalpha_fr,
            result.aerodynamicOutput.dcmalpha_nose,
            result.aerodynamicOutput.dcmalpha_tail,
            result.aerodynamicOutput.cmalpha,
            result.aerodynamicOutput.cm,
            result.aerodynamicOutput.pitch_moment_nm,
            result.aerodynamicOutput.cnbeta_fr,
            result.aerodynamicOutput.dcnbeta_nose,
            result.aerodynamicOutput.dcnbeta_tail,
            result.aerodynamicOutput.cnbeta,
            result.aerodynamicOutput.cn,
            result.aerodynamicOutput.yaw_moment_nm
        }))
    {
        throw std::runtime_error(
            "FuselageComponent produced a non-finite result; check the "
            "Reynolds-number domain and configuration."
        );
    }

    return result;
}

const FuselageComponentConfig& FuselageComponent::config() const noexcept
{
    return config_;
}

FuselageGeometry makeNicolosiReferenceFuselageGeometry()
{
    FuselageGeometry geometry;
    geometry.Lf_m = 30.0;
    geometry.df_m = 3.45;
    geometry.Ln_m = 5.7;
    geometry.Lc_m = 13.0;
    geometry.Lt_m = 11.3;
    geometry.db_m = 0.0;
    geometry.theta_rad = degreesToRadians(14.0);
    geometry.psi_rad = degreesToRadians(40.0);
    geometry.Sw_m2 = 75.5;
    geometry.mac_m = 2.58;
    geometry.b_m = 0.0;
    return geometry;
}

FuselageGeometry makeT6cReferenceFuselageGeometry()
{
    FuselageGeometry geometry;
    geometry.Lf_m = 10.18;
    geometry.df_m = 1.05;
    geometry.Ln_m = 1.93;
    geometry.Lc_m = 4.41;
    geometry.Lt_m = 3.84;
    geometry.db_m = 0.0;
    geometry.theta_rad = degreesToRadians(14.0);
    geometry.psi_rad = degreesToRadians(40.0);
    geometry.Sw_m2 = 16.35;
    geometry.mac_m = 1.606;
    geometry.b_m = 10.18;
    return geometry;
}

FuselageTuning makeT6cReferenceFuselageTuning()
{
    FuselageTuning tuning;
    tuning.windshield_drag_increment = 0.0;

    auto& pitch = tuning.pitch;
    pitch.fineness_ratio_ref = 8.7;
    pitch.nose_fineness_ratio_ref = 1.6;
    pitch.windshield_angle_ref_rad = degreesToRadians(40.0);
    pitch.tail_fineness_ratio_ref = 2.8;
    pitch.upsweep_angle_ref_rad = degreesToRadians(14.0);
    pitch.cm0_fineness_ratio_anchor = -0.3159;
    pitch.cm0_fineness_ratio_sensitivity = 0.03;
    pitch.cm0_nose_correction_anchor = 0.0013;
    pitch.cm0_nose_windshield_angle_sensitivity_per_rad = 0.01;
    pitch.cm0_nose_fineness_ratio_sensitivity = 0.01;
    pitch.cm0_tail_correction_anchor = 0.0013;
    pitch.cm0_tail_upsweep_angle_sensitivity_per_rad = 0.005;
    pitch.cm0_tail_fineness_ratio_sensitivity = 0.005;
    pitch.cmalpha_fineness_ratio_anchor = 0.1815;
    pitch.cmalpha_fineness_ratio_sensitivity = 0.02;
    pitch.cmalpha_nose_correction_anchor = 0.0005;
    pitch.cmalpha_nose_windshield_angle_sensitivity_per_rad = 0.0005;
    pitch.cmalpha_nose_fineness_ratio_sensitivity = 0.0005;
    pitch.cmalpha_tail_correction_anchor = 0.0006;
    pitch.cmalpha_tail_upsweep_angle_sensitivity_per_rad = 0.0005;
    pitch.cmalpha_tail_fineness_ratio_sensitivity = 0.0005;

    auto& yaw = tuning.yaw;
    yaw.fineness_ratio_ref = 8.7;
    yaw.nose_fineness_ratio_ref = 1.6;
    yaw.tail_fineness_ratio_ref = 2.8;
    yaw.cnbeta_fineness_ratio_anchor = 0.1817;
    yaw.cnbeta_fineness_ratio_sensitivity = -0.02;
    yaw.cnbeta_nose_correction_anchor = -0.0003;
    yaw.cnbeta_nose_fineness_ratio_sensitivity = -0.0005;
    yaw.cnbeta_tail_correction_anchor = -0.0002;
    yaw.cnbeta_tail_fineness_ratio_sensitivity = -0.0005;
    return tuning;
}

FuselageComponentConfig makeT6cReferenceFuselageConfig()
{
    FuselageComponentConfig config;
    config.geometry = makeT6cReferenceFuselageGeometry();
    config.tuning = makeT6cReferenceFuselageTuning();
    return config;
}

FuselageComponentConfig makeProvisionalNavionFuselageConfig()
{
    // Stage 5 does not yet contain an independently validated Navion fuselage
    // data set. Keep the exact uploaded proxy values but expose their role as
    // provisional active configuration. Reference provenance remains under
    // reference_data/t6c/ and docs/fuselage/original/.
    return makeT6cReferenceFuselageConfig();
}

} // namespace navion::fuselage
