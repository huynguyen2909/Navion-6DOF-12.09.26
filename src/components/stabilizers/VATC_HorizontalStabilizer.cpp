#include "navion/components/stabilizers/VATC_HorizontalStabilizer.hpp"

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

void validateConfig(const VATC_HorizontalStabilizerConfig& config)
{
    if (!allFinite({
            config.Area,
            config.TailSpan,
            config.TailMAC,
            config.TailAR,
            config.IncidenceAngle,
            config.PositionWrtCG[0],
            config.PositionWrtCG[1],
            config.PositionWrtCG[2],
            config.MomentArm,
            config.ElevatorArea,
            config.ElevatorMechanicalLimit[0],
            config.ElevatorMechanicalLimit[1],
            config.LiftCurveSlope,
            config.ZeroLiftAngle,
            config.ZeroLiftDragCoefficient,
            config.InducedDragFactor,
            config.ElevatorEffectiveness,
            config.PitchMomentCoefficient,
            config.PitchMomentCurveSlope,
            config.ElevatorPitchMomentEffectiveness
        }))
    {
        throw std::invalid_argument(
            "HorizontalStabilizer configuration must be finite."
        );
    }

    if (config.Area <= 0.0 || config.TailSpan <= 0.0 ||
        config.TailMAC <= 0.0)
    {
        throw std::invalid_argument(
            "HorizontalStabilizer Area, TailSpan, and TailMAC must be > 0."
        );
    }

    if (config.ElevatorArea < 0.0 || config.ElevatorArea > config.Area)
    {
        throw std::invalid_argument(
            "HorizontalStabilizer ElevatorArea must be in [0, Area]."
        );
    }

    if (config.LiftCurveSlope < 0.0 ||
        config.ZeroLiftDragCoefficient < 0.0 ||
        config.InducedDragFactor < 0.0 ||
        config.ElevatorEffectiveness < 0.0)
    {
        throw std::invalid_argument(
            "HorizontalStabilizer lift slope, drag parameters, and elevator "
            "effectiveness must be >= 0."
        );
    }
}

void validateContext(const EvaluationContext& context)
{
    if (!context.flightCondition.isFinite() ||
        !context.state.angularRateBodyRadps.isFinite() ||
        !std::isfinite(context.controls.elevatorRad) ||
        !std::isfinite(context.environment.airDensityKgM3) ||
        context.environment.airDensityKgM3 < 0.0)
    {
        throw std::invalid_argument(
            "HorizontalStabilizer received an invalid EvaluationContext."
        );
    }
}

} // namespace

struct VATC_HorizontalStabilizer::WorkingData
{
    Vec3 localVelocityBodyMps{};
    Vec3 forceBodyN{};
    Vec3 intrinsicMomentAtAerodynamicCenterBodyNm{};

    double airspeedMps{0.0};
    double flowAngleOfAttackRad{0.0};
    double angleOfAttackRad{0.0};
    double effectiveAngleOfAttackRad{0.0};
    double sideSlipRad{0.0};
    double dynamicPressurePa{0.0};
    double elevatorDeflectionRad{0.0};
    double liftCoefficient{0.0};
    double dragCoefficient{0.0};
    double liftForceN{0.0};
    double dragForceN{0.0};
};

VATC_HorizontalStabilizer::VATC_HorizontalStabilizer(
    const VATC_HorizontalStabilizerConfig& config,
    std::shared_ptr<const ILocalFlowField> localFlowField
)
    : config_(config),
      localFlowField_(std::move(localFlowField))
{
    validateConfig(config_);

    if (config_.TailAR <= 0.0)
    {
        config_.TailAR = config_.TailSpan * config_.TailSpan / config_.Area;
    }

    if (config_.ElevatorMechanicalLimit[0] >
        config_.ElevatorMechanicalLimit[1])
    {
        std::swap(
            config_.ElevatorMechanicalLimit[0],
            config_.ElevatorMechanicalLimit[1]
        );
    }
}

BodyLoad VATC_HorizontalStabilizer::computeLoad(
    const EvaluationContext& context
) const
{
    return evaluateDetailed(context).bodyLoad;
}

std::string_view VATC_HorizontalStabilizer::name() const noexcept
{
    return "HorizontalStabilizer";
}

HorizontalStabilizerEvaluation
VATC_HorizontalStabilizer::evaluateDetailed(
    const EvaluationContext& context
) const
{
    validateContext(context);

    WorkingData data;
    data.elevatorDeflectionRad = std::clamp(
        context.controls.elevatorRad,
        config_.ElevatorMechanicalLimit[0],
        config_.ElevatorMechanicalLimit[1]
    );

    calculateVelocityComponents(context, data);
    calculateAngleOfAttack(data);
    calculateSideSlipAngle(data);
    calculateDynamicPressure(context, data);
    calculateAerodynamicCoefficients(data);
    calculateForces(data);
    calculateAerodynamicCenterMoment(data);
    calculateBodyForces(data);

    HorizontalStabilizerEvaluation result;
    result.bodyLoad = makeBodyLoadAtPoint(
        data.forceBodyN,
        data.intrinsicMomentAtAerodynamicCenterBodyNm,
        positionFromCgBodyM()
    );
    result.localVelocityBodyMps = data.localVelocityBodyMps;
    result.intrinsicMomentAtAerodynamicCenterBodyNm =
        data.intrinsicMomentAtAerodynamicCenterBodyNm;
    result.airspeedMps = data.airspeedMps;
    result.flowAngleOfAttackRad = data.flowAngleOfAttackRad;
    result.angleOfAttackRad = data.angleOfAttackRad;
    result.effectiveAngleOfAttackRad = data.effectiveAngleOfAttackRad;
    result.sideSlipRad = data.sideSlipRad;
    result.dynamicPressurePa = data.dynamicPressurePa;
    result.elevatorDeflectionRad = data.elevatorDeflectionRad;
    result.liftCoefficient = data.liftCoefficient;
    result.dragCoefficient = data.dragCoefficient;
    result.liftForceN = data.liftForceN;
    result.dragForceN = data.dragForceN;

    if (!result.bodyLoad.isFinite() ||
        !result.localVelocityBodyMps.isFinite() ||
        !result.intrinsicMomentAtAerodynamicCenterBodyNm.isFinite() ||
        !allFinite({
            result.airspeedMps,
            result.flowAngleOfAttackRad,
            result.angleOfAttackRad,
            result.effectiveAngleOfAttackRad,
            result.sideSlipRad,
            result.dynamicPressurePa,
            result.elevatorDeflectionRad,
            result.liftCoefficient,
            result.dragCoefficient,
            result.liftForceN,
            result.dragForceN
        }))
    {
        throw std::runtime_error(
            "HorizontalStabilizer produced a non-finite result."
        );
    }

    return result;
}

const VATC_HorizontalStabilizerConfig&
VATC_HorizontalStabilizer::getConfig() const noexcept
{
    return config_;
}

void VATC_HorizontalStabilizer::calculateVelocityComponents(
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
                "HorizontalStabilizer local-flow field returned a non-finite vector."
            );
        }
    }

    data.localVelocityBodyMps =
        context.flightCondition.airRelativeVelocityBodyMps +
        cross(context.state.angularRateBodyRadps, position) +
        localFlowIncrementBodyMps;
    data.airspeedMps = data.localVelocityBodyMps.norm();
}

void VATC_HorizontalStabilizer::calculateAngleOfAttack(
    WorkingData& data
) const
{
    const double u = data.localVelocityBodyMps.x;
    const double w = data.localVelocityBodyMps.z;

    if (std::abs(u) < EPSILON_VELOCITY_MPS &&
        std::abs(w) < EPSILON_VELOCITY_MPS)
    {
        data.flowAngleOfAttackRad = 0.0;
    }
    else
    {
        data.flowAngleOfAttackRad = std::atan2(w, u);
    }

    data.angleOfAttackRad =
        data.flowAngleOfAttackRad + config_.IncidenceAngle;
    data.effectiveAngleOfAttackRad =
        data.angleOfAttackRad - config_.ZeroLiftAngle +
        config_.ElevatorEffectiveness * data.elevatorDeflectionRad;
}

void VATC_HorizontalStabilizer::calculateSideSlipAngle(
    WorkingData& data
) const
{
    if (data.airspeedMps < EPSILON_VELOCITY_MPS)
    {
        data.sideSlipRad = 0.0;
        return;
    }

    const Vec3& velocity = data.localVelocityBodyMps;
    data.sideSlipRad = std::atan2(
        velocity.y,
        std::hypot(velocity.x, velocity.z)
    );
}

void VATC_HorizontalStabilizer::calculateDynamicPressure(
    const EvaluationContext& context,
    WorkingData& data
) const
{
    data.dynamicPressurePa =
        0.5 * context.environment.airDensityKgM3 *
        data.airspeedMps * data.airspeedMps;
}

void VATC_HorizontalStabilizer::calculateAerodynamicCoefficients(
    WorkingData& data
) const
{
    data.liftCoefficient =
        config_.LiftCurveSlope * data.effectiveAngleOfAttackRad;
    data.dragCoefficient =
        config_.ZeroLiftDragCoefficient +
        config_.InducedDragFactor *
            data.liftCoefficient * data.liftCoefficient;
}

void VATC_HorizontalStabilizer::calculateForces(
    WorkingData& data
) const
{
    data.liftForceN =
        data.dynamicPressurePa * config_.Area * data.liftCoefficient;
    data.dragForceN =
        data.dynamicPressurePa * config_.Area * data.dragCoefficient;
}

void VATC_HorizontalStabilizer::calculateAerodynamicCenterMoment(
    WorkingData& data
) const
{
    const double momentCoefficient =
        config_.PitchMomentCoefficient +
        config_.PitchMomentCurveSlope * data.effectiveAngleOfAttackRad +
        config_.ElevatorPitchMomentEffectiveness *
            data.elevatorDeflectionRad;

    data.intrinsicMomentAtAerodynamicCenterBodyNm = {
        0.0,
        data.dynamicPressurePa * config_.Area * config_.TailMAC *
            momentCoefficient,
        0.0
    };
}

void VATC_HorizontalStabilizer::calculateBodyForces(
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

    Vec3 liftDirection{};
    const double longitudinalSpeed = std::hypot(velocity.x, velocity.z);
    if (longitudinalSpeed > EPSILON_VELOCITY_MPS)
    {
        liftDirection = {
            velocity.z / longitudinalSpeed,
            0.0,
            -velocity.x / longitudinalSpeed
        };
    }

    data.forceBodyN =
        data.liftForceN * liftDirection +
        data.dragForceN * dragDirection;
}

Vec3 VATC_HorizontalStabilizer::positionFromCgBodyM() const noexcept
{
    return {
        config_.PositionWrtCG[0],
        config_.PositionWrtCG[1],
        config_.PositionWrtCG[2]
    };
}

} // namespace navion
