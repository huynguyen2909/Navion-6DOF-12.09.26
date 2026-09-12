#pragma once

#include "navion/components/ILoadComponent.hpp"
#include "navion/components/ILocalFlowField.hpp"

#include <memory>
#include <string_view>

namespace navion
{

struct VATC_VerticalStabilizerConfig
{
    double Area{0.0};
    double RudderArea{0.0};
    double TailSpan{0.0};
    double TailMAC{0.0};
    double TailAR{0.0};
    double PositionWrtCG[3]{0.0, 0.0, 0.0};
    double MomentArm{0.0}; // Retained for configuration diagnostics only.

    // Positive setting increases effective fin sideslip.
    double FinSettingAngle{0.0};
    double RudderMechanicalLimit[2]{0.0, 0.0};

    double SideForceCurveSlope{0.0};
    double RudderEffectiveness{0.0};
    double ZeroLiftDragCoefficient{0.0};
    double InducedDragFactor{0.0};

    // Intrinsic yawing moment about the fin aerodynamic center:
    // Cn_ac = Cn0 + Cn_beta * beta_eff + Cn_delta_r * delta_r.
    double YawMomentCoefficient{0.0};
    double YawMomentCurveSlope{0.0};
    double RudderYawMomentEffectiveness{0.0};
};

struct VerticalStabilizerEvaluation
{
    BodyLoad bodyLoad{};
    Vec3 localVelocityBodyMps{};
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

// Vertical stabilizer + rudder. Positive rudder means trailing edge left.
class VATC_VerticalStabilizer final : public ILoadComponent
{
public:
    explicit VATC_VerticalStabilizer(
        const VATC_VerticalStabilizerConfig& config,
        std::shared_ptr<const ILocalFlowField> localFlowField = nullptr
    );

    [[nodiscard]] BodyLoad computeLoad(
        const EvaluationContext& context
    ) const override;

    [[nodiscard]] std::string_view name() const noexcept override;

    [[nodiscard]] VerticalStabilizerEvaluation evaluateDetailed(
        const EvaluationContext& context
    ) const;

    [[nodiscard]] const VATC_VerticalStabilizerConfig&
    getConfig() const noexcept;

private:
    struct WorkingData;

    void calculateVelocityComponents(
        const EvaluationContext& context,
        WorkingData& data
    ) const;
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

    VATC_VerticalStabilizerConfig config_{};
    std::shared_ptr<const ILocalFlowField> localFlowField_{};
};

using VerticalStabilizerConfig = VATC_VerticalStabilizerConfig;
using VerticalStabilizer = VATC_VerticalStabilizer;

} // namespace navion
