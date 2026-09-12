#include "navion/components/stabilizers/VATC_VerticalStabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace navion
{
namespace
{

constexpr double EPSILON_VELOCITY_MPS = 1.0e-8;

bool allFinite(std::initializer_list<double> values) noexcept
{
    return std::all_of(
        values.begin(),
        values.end(),
        [](double value) { return std::isfinite(value); }
    );
}

void validateConfig(const VATC_VerticalStabilizerConfig& config)
{
    if (!allFinite({
            config.Area,
            config.RudderArea,
            config.TailSpan,
            config.TailMAC,
            config.TailAR,
            config.PositionWrtCG[0],
            config.PositionWrtCG[1],
            config.PositionWrtCG[2],
            config.MomentArm,
            config.FinSettingAngle,
            config.RudderMechanicalLimit[0],
            config.RudderMechanicalLimit[1],
            config.SideForceCurveSlope,
            config.RudderEffectiveness,
            config.ZeroLiftDragCoefficient,
            config.InducedDragFactor,
            config.YawMomentCoefficient,
            config.YawMomentCurveSlope,
            config.RudderYawMomentEffectiveness
        }))
    {
        throw std::invalid_argument(
            "VerticalStabilizer configuration must be finite."
        );
    }

    if (config.Area <= 0.0 || config.RudderArea < 0.0 ||
        config.RudderArea > config.Area || config.TailSpan < 0.0 ||
        config.TailMAC < 0.0)
    {
        throw std::invalid_argument(
            "VerticalStabilizer geometry is outside its valid range."
        );
    }

    if (config.SideForceCurveSlope < 0.0 ||
        config.RudderEffectiveness < 0.0 ||
        config.ZeroLiftDragCoefficient < 0.0 ||
        config.InducedDragFactor < 0.0)
    {
        throw std::invalid_argument(
            "VerticalStabilizer side-force slope, drag parameters, and rudder "
            "effectiveness must be >= 0."
        );
    }
}

void validateContext(const EvaluationContext& context)
{
    if (!context.flightCondition.isFinite() ||
        !context.state.angularRateBodyRadps.isFinite() ||
        !std::isfinite(context.controls.rudderRad) ||
        !std::isfinite(context.environment.airDensityKgM3) ||
        context.environment.airDensityKgM3 < 0.0)
    {
        throw std::invalid_argument(
            "VerticalStabilizer received an invalid EvaluationContext."
        );
    }
}

} // namespace

struct VATC_VerticalStabilizer::WorkingData
{
    Vec3 localVelocityBodyMps{};
    Vec3 forceBodyN{};
    Vec3 intrinsicMomentAtAerodynamicCenterBodyNm{};

    double airspeedMps{0.0};
    double flowSideSlipRad{0.0};
    double effectiveSideSlipRad{0.0};
    double dynamicPressurePa{0.0};
    double rudderDeflectionRad{0.0};
    double sideForceCoefficient{0.0};
    double dragCoefficient{0.0};
    double sideForceN{0.0};
    double dragForceN{0.0};
};

VATC_VerticalStabilizer::VATC_VerticalStabilizer(
    const VATC_VerticalStabilizerConfig& config,
    std::shared_ptr<const ILocalFlowField> localFlowField
)
    : config_(config),
      localFlowField_(std::move(localFlowField))
{
    validateConfig(config_);

    if (config_.TailAR <= 0.0 && config_.TailSpan > 0.0)
    {
        config_.TailAR = config_.TailSpan * config_.TailSpan / config_.Area;
    }

    if (config_.RudderMechanicalLimit[0] >
        config_.RudderMechanicalLimit[1])
    {
        std::swap(
            config_.RudderMechanicalLimit[0],
            config_.RudderMechanicalLimit[1]
        );
    }
}

BodyLoad VATC_VerticalStabilizer::computeLoad(
    const EvaluationContext& context
) const
{
    return evaluateDetailed(context).bodyLoad;
}

std::string_view VATC_VerticalStabilizer::name() const noexcept
{
    return "VerticalStabilizer";
}

VerticalStabilizerEvaluation VATC_VerticalStabilizer::evaluateDetailed(
    const EvaluationContext& context
) const
{
    validateContext(context);

    WorkingData data;
    data.rudderDeflectionRad = std::clamp(
        context.controls.rudderRad,
        config_.RudderMechanicalLimit[0],
        config_.RudderMechanicalLimit[1]
    );

    calculateVelocityComponents(context, data);
    calculateSideSlipAngle(data);
    calculateDynamicPressure(context, data);
    calculateAerodynamicCoefficients(data);
    calculateForces(data);
    calculateAerodynamicCenterMoment(data);
    calculateBodyForces(data);

    VerticalStabilizerEvaluation result;
    result.bodyLoad = makeBodyLoadAtPoint(
        data.forceBodyN,
        data.intrinsicMomentAtAerodynamicCenterBodyNm,
        positionFromCgBodyM()
    );
    result.localVelocityBodyMps = data.localVelocityBodyMps;
    result.intrinsicMomentAtAerodynamicCenterBodyNm =
        data.intrinsicMomentAtAerodynamicCenterBodyNm;
    result.airspeedMps = data.airspeedMps;
    result.flowSideSlipRad = data.flowSideSlipRad;
    result.effectiveSideSlipRad = data.effectiveSideSlipRad;
    result.dynamicPressurePa = data.dynamicPressurePa;
    result.rudderDeflectionRad = data.rudderDeflectionRad;
    result.sideForceCoefficient = data.sideForceCoefficient;
    result.dragCoefficient = data.dragCoefficient;
    result.sideForceN = data.sideForceN;
    result.dragForceN = data.dragForceN;

    if (!result.bodyLoad.isFinite() ||
        !result.localVelocityBodyMps.isFinite() ||
        !result.intrinsicMomentAtAerodynamicCenterBodyNm.isFinite() ||
        !allFinite({
            result.airspeedMps,
            result.flowSideSlipRad,
            result.effectiveSideSlipRad,
            result.dynamicPressurePa,
            result.rudderDeflectionRad,
            result.sideForceCoefficient,
            result.dragCoefficient,
            result.sideForceN,
            result.dragForceN
        }))
    {
        throw std::runtime_error(
            "VerticalStabilizer produced a non-finite result."
        );
    }

    return result;
}

const VATC_VerticalStabilizerConfig&
VATC_VerticalStabilizer::getConfig() const noexcept
{
    return config_;
}

void VATC_VerticalStabilizer::calculateVelocityComponents(
    const EvaluationContext& context,
    WorkingData& data
) const
{
    const Vec3 position = positionFromCgBodyM();
    Vec3 localFlowIncrementBodyMps{};
    if (localFlowField_)
    {
        localFlowIncrementBodyMps =
            localFlowField_->velocityIncrementBodyMps(context, position);
        if (!localFlowIncrementBodyMps.isFinite())
        {
            throw std::runtime_error(
                "VerticalStabilizer local-flow field returned a non-finite vector."
            );
        }
    }

    data.localVelocityBodyMps =
        context.flightCondition.airRelativeVelocityBodyMps +
        cross(context.state.angularRateBodyRadps, position) +
        localFlowIncrementBodyMps;
    data.airspeedMps = data.localVelocityBodyMps.norm();
}

void VATC_VerticalStabilizer::calculateSideSlipAngle(
    WorkingData& data
) const
{
    if (data.airspeedMps < EPSILON_VELOCITY_MPS)
    {
        data.flowSideSlipRad = 0.0;
    }
    else
    {
        const Vec3& velocity = data.localVelocityBodyMps;
        data.flowSideSlipRad = std::atan2(
            velocity.y,
            std::hypot(velocity.x, velocity.z)
        );
    }

    data.effectiveSideSlipRad =
        data.flowSideSlipRad + config_.FinSettingAngle -
        config_.RudderEffectiveness * data.rudderDeflectionRad;
}

void VATC_VerticalStabilizer::calculateDynamicPressure(
    const EvaluationContext& context,
    WorkingData& data
) const
{
    data.dynamicPressurePa =
        0.5 * context.environment.airDensityKgM3 *
        data.airspeedMps * data.airspeedMps;
}

void VATC_VerticalStabilizer::calculateAerodynamicCoefficients(
    WorkingData& data
) const
{
    // A positive beta produces negative BODY-Y force for a conventional fin.
    data.sideForceCoefficient =
        -config_.SideForceCurveSlope * data.effectiveSideSlipRad;
    data.dragCoefficient =
        config_.ZeroLiftDragCoefficient +
        config_.InducedDragFactor *
            data.sideForceCoefficient * data.sideForceCoefficient;
}

void VATC_VerticalStabilizer::calculateForces(
    WorkingData& data
) const
{
    data.sideForceN =
        data.dynamicPressurePa * config_.Area * data.sideForceCoefficient;
    data.dragForceN =
        data.dynamicPressurePa * config_.Area * data.dragCoefficient;
}

void VATC_VerticalStabilizer::calculateAerodynamicCenterMoment(
    WorkingData& data
) const
{
    const double momentCoefficient =
        config_.YawMomentCoefficient +
        config_.YawMomentCurveSlope * data.effectiveSideSlipRad +
        config_.RudderYawMomentEffectiveness * data.rudderDeflectionRad;

    data.intrinsicMomentAtAerodynamicCenterBodyNm = {
        0.0,
        0.0,
        data.dynamicPressurePa * config_.Area * config_.TailMAC *
            momentCoefficient
    };
}

void VATC_VerticalStabilizer::calculateBodyForces(
    WorkingData& data
) const
{
    if (data.airspeedMps < EPSILON_VELOCITY_MPS)
    {
        data.forceBodyN = {};
        return;
    }

    const Vec3& velocity = data.localVelocityBodyMps;
    const Vec3 dragDirection = -velocity / data.airspeedMps;

    Vec3 sideForceDirection{};
    const double horizontalSpeed = std::hypot(velocity.x, velocity.y);
    if (horizontalSpeed > EPSILON_VELOCITY_MPS)
    {
        // Correct signed-CY basis. At beta=0, positive signed side force maps
        // to BODY +Y. This fixes the sign reversal in the uploaded VS source.
        sideForceDirection = {
            -velocity.y / horizontalSpeed,
            velocity.x / horizontalSpeed,
            0.0
        };
    }

    data.forceBodyN =
        data.sideForceN * sideForceDirection +
        data.dragForceN * dragDirection;
}

Vec3 VATC_VerticalStabilizer::positionFromCgBodyM() const noexcept
{
    return {
        config_.PositionWrtCG[0],
        config_.PositionWrtCG[1],
        config_.PositionWrtCG[2]
    };
}

} // namespace navion
