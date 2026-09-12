#pragma once

#include "navion/components/ILoadComponent.hpp"
#include "navion/components/ILocalFlowField.hpp"

#include <memory>
#include <string_view>

namespace navion
{

struct VATC_HorizontalStabilizerConfig
{
    double Area{0.0};
    double TailSpan{0.0};
    double TailMAC{0.0};
    double TailAR{0.0};
    double IncidenceAngle{0.0};
    double PositionWrtCG[3]{0.0, 0.0, 0.0};
    double MomentArm{0.0}; // Retained for configuration diagnostics only.

    double ElevatorArea{0.0};
    double ElevatorMechanicalLimit[2]{0.0, 0.0};

    double LiftCurveSlope{0.0};
    double ZeroLiftAngle{0.0};
    double ZeroLiftDragCoefficient{0.0};
    double InducedDragFactor{0.0};
    double ElevatorEffectiveness{0.0};

    // Intrinsic pitching moment about the tail aerodynamic center:
    // Cm_ac = Cm0 + Cm_alpha * alpha_eff + Cm_delta_e * delta_e.
    double PitchMomentCoefficient{0.0};
    double PitchMomentCurveSlope{0.0};
    double ElevatorPitchMomentEffectiveness{0.0};
};

struct HorizontalStabilizerEvaluation
{
    BodyLoad bodyLoad{};
    Vec3 localVelocityBodyMps{};
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

// Horizontal stabilizer + elevator. Runtime evaluation is stateless: all
// stage-dependent data comes from EvaluationContext and local working data.
class VATC_HorizontalStabilizer final : public ILoadComponent
{
public:
    explicit VATC_HorizontalStabilizer(
        const VATC_HorizontalStabilizerConfig& config,
        std::shared_ptr<const ILocalFlowField> localFlowField = nullptr
    );

    [[nodiscard]] BodyLoad computeLoad(
        const EvaluationContext& context
    ) const override;

    [[nodiscard]] std::string_view name() const noexcept override;

    [[nodiscard]] HorizontalStabilizerEvaluation evaluateDetailed(
        const EvaluationContext& context
    ) const;

    [[nodiscard]] const VATC_HorizontalStabilizerConfig&
    getConfig() const noexcept;

private:
    struct WorkingData;

    void calculateVelocityComponents(
        const EvaluationContext& context,
        WorkingData& data
    ) const;
    void calculateAngleOfAttack(WorkingData& data) const;
    void calculateSideSlipAngle(WorkingData& data) const;
    void calculateDynamicPressure(
        const EvaluationContext& context,
        WorkingData& data
    ) const;
    void calculateAerodynamicCoefficients(WorkingData& data) const;
    void calculateForces(WorkingData& data) const;
    void calculateAerodynamicCenterMoment(WorkingData& data) const;
    void calculateBodyForces(WorkingData& data) const;

    [[nodiscard]] Vec3 positionFromCgBodyM() const noexcept;

    VATC_HorizontalStabilizerConfig config_{};
    std::shared_ptr<const ILocalFlowField> localFlowField_{};
};

// Clean names for new call sites while retaining the original VATC type names
// and configuration layout for low-effort migration.
using HorizontalStabilizerConfig = VATC_HorizontalStabilizerConfig;
using HorizontalStabilizer = VATC_HorizontalStabilizer;

} // namespace navion
