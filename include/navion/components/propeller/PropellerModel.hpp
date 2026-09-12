#pragma once

#include "navion/components/ILoadComponent.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace navion::propeller
{

inline constexpr double pi = 3.141592653589793238462643383279502884;

[[nodiscard]] constexpr double degreesToRadians(double valueDegrees) noexcept
{
    return valueDegrees * pi / 180.0;
}

struct PolarPoint
{
    double alphaRad{0.0};
    double cl{0.0};
    double cd{0.0};
};

struct PolarLookup
{
    double cl{0.0};
    double cd{0.0};
    bool clamped{false};
};

class AirfoilPolar
{
public:
    AirfoilPolar() = default;
    explicit AirfoilPolar(std::vector<PolarPoint> points);

    [[nodiscard]] PolarLookup lookup(double alphaRad) const;
    [[nodiscard]] const std::vector<PolarPoint>& points() const noexcept;
    void validate() const;

private:
    std::vector<PolarPoint> points_{};
};

struct BladeStation
{
    double radiusFraction{0.0}; // r/R
    double chordM{0.0};
    double pitchRad{0.0};       // chord line relative to disk plane
};

struct InflowSolverSettings
{
    double lambdaMinimum{0.0};
    double lambdaMaximum{0.50};
    double initialGuess{0.08};
    double residualTolerance{1.0e-9};
    double lambdaTolerance{1.0e-10};
    double derivativeStep{1.0e-5};
    std::size_t maximumIterations{50U};
    std::size_t bracketScanIntervals{100U};
};

struct PropellerParameters
{
    std::string propellerName{};
    std::string airfoilName{};

    int bladeCount{2};
    double radiusM{1.0};
    double rootCutoutFraction{0.20};
    double rotationRateRadps{1.0};

    // +1 means right-hand-positive rotation about +x_p. For an aligned shaft,
    // it appears clockwise to a pilot looking forward.
    int rotationSign{+1};
    double rotatingInertiaKgM2{0.0};

    // Vector from aircraft CG to hub, resolved in BODY FRD axes.
    Vec3 hubPositionFromCgBodyM{};

    // C_b<-p transforms propeller-frame components into BODY components.
    // Propeller +x_p is the positive-thrust shaft direction.
    Matrix3 bodyFromPropeller{Matrix3::identity()};

    std::vector<BladeStation> bladeGeometry{};
    AirfoilPolar airfoilPolar{};

    std::size_t radialElementCount{64U};
    std::size_t azimuthStationCount{96U};
    double compressibilityWarningMach{0.70};
    InflowSolverSettings inflow{};

    [[nodiscard]] double diameterM() const noexcept;
    [[nodiscard]] double diskAreaM2() const noexcept;
    [[nodiscard]] double tipSpeedMps() const noexcept;
    [[nodiscard]] double revolutionsPerSecond() const noexcept;
    [[nodiscard]] double rpm() const noexcept;

    void validate() const;
};

// Kernel-level input retained for BEMT diagnostics and isolated unit tests.
// Aircraft integration normally uses PropellerComponent and EvaluationContext.
struct RuntimeInput
{
    Vec3 velocityCgRelativeAirBodyMps{};
    Vec3 angularRateBodyWrtInertialBodyRadps{};
    double airDensityKgM3{1.225};
    double speedOfSoundMps{340.294}; // diagnostic only in V1
    // Multiplier applied to PropellerParameters::rotationRateRadps.
    double rotationRateScale{1.0};
    bool enabled{true};
};

struct HubKinematics
{
    Vec3 velocityRelativeAirPropellerMps{};
    Vec3 bodyAngularRatePropellerRadps{};
    double lambdaFreestream{0.0};
    double muY{0.0};
    double muZ{0.0};
    double muMagnitude{0.0};
    double advanceRatioJ{0.0};
};

struct ElementSample
{
    double azimuthRad{0.0};
    double radiusM{0.0};
    double radiusFraction{0.0};
    double chordM{0.0};
    double pitchRad{0.0};
    double axialVelocityMps{0.0};
    double tangentialVelocityMps{0.0};
    double inflowAngleRad{0.0};
    double angleOfAttackRad{0.0};
    double cl{0.0};
    double cd{0.0};
    Vec3 differentialMeanForcePropellerN{};
    Vec3 differentialMeanMomentPropellerNm{};
    bool polarClamped{false};
};

struct DiskLoads
{
    Vec3 forcePropellerN{};
    Vec3 aerodynamicMomentAtHubPropellerNm{};

    double thrustN{0.0};
    double torqueRequiredNm{0.0};
    double shaftPowerRequiredW{0.0};
    double thrustCoefficientRotor{0.0};
    double thrustCoefficientPropeller{0.0};
    double torqueCoefficientPropeller{0.0};

    double minimumAlphaRad{0.0};
    double maximumAlphaRad{0.0};
    double maximumSectionMach{0.0};
    bool compressibilityWarning{false};
    std::size_t polarClampCount{0U};
    std::size_t reverseTangentialFlowCount{0U};
    std::size_t elementCount{0U};
};

struct DiskEvaluation
{
    HubKinematics kinematics{};
    DiskLoads loads{};
    std::vector<ElementSample> elementSamples{};
};

enum class InflowStatus
{
    Converged,
    RootNotBracketed,
    MaximumIterations
};

[[nodiscard]] const char* toString(InflowStatus status) noexcept;

struct InflowDiagnostics
{
    InflowStatus status{InflowStatus::RootNotBracketed};
    bool converged{false};
    bool rootBracketed{false};
    std::size_t iterations{0U};
    std::size_t residualEvaluations{0U};
    double lambdaInduced{0.0};
    double inducedVelocityMps{0.0};
    double momentumResidual{0.0};
    double momentumThrustCoefficient{0.0};
};

struct PropellerOutput
{
    HubKinematics kinematics{};
    DiskLoads disk{};
    InflowDiagnostics inflow{};

    double rotationRateRadps{0.0};
    double rpm{0.0};

    Vec3 forceBodyN{};
    Vec3 aerodynamicMomentAtHubBodyNm{};
    Vec3 reactionTorqueBodyNm{};
    Vec3 hubBendingMomentBodyNm{};
    Vec3 momentArmBodyNm{};
    Vec3 gyroscopicMomentBodyNm{};
    Vec3 totalMomentAtCgBodyNm{};
};

// Numerical/physical BEMT kernel. It has no retained runtime state.
class PropellerModel
{
public:
    explicit PropellerModel(PropellerParameters parameters);

    [[nodiscard]] const PropellerParameters& parameters() const noexcept;

    [[nodiscard]] DiskEvaluation evaluateDiskAtInflow(
        const RuntimeInput& input,
        double lambdaInduced,
        bool captureElementSamples = false
    ) const;

    // previousLambdaInduced is only a numerical initial guess. Uniform inflow
    // remains an algebraic hidden variable solved during this evaluation.
    [[nodiscard]] PropellerOutput evaluate(
        const RuntimeInput& input,
        double previousLambdaInduced
    ) const;

    [[nodiscard]] PropellerOutput evaluate(const RuntimeInput& input) const;

private:
    struct LocalGeometry
    {
        double chordM{0.0};
        double pitchRad{0.0};
    };

    struct ResidualEvaluation
    {
        double lambdaInduced{0.0};
        double residual{0.0};
        double momentumThrustCoefficient{0.0};
        DiskEvaluation disk{};
    };

    [[nodiscard]] LocalGeometry interpolateGeometry(
        double radiusFraction
    ) const;
    [[nodiscard]] HubKinematics calculateHubKinematics(
        const RuntimeInput& input,
        double rotationRateRadps
    ) const;
    [[nodiscard]] DiskEvaluation evaluateDiskWithKinematics(
        const RuntimeInput& input,
        const HubKinematics& kinematics,
        double rotationRateRadps,
        double lambdaInduced,
        bool captureElementSamples
    ) const;
    [[nodiscard]] ResidualEvaluation evaluateResidual(
        const RuntimeInput& input,
        const HubKinematics& kinematics,
        double rotationRateRadps,
        double lambdaInduced
    ) const;

    PropellerParameters parameters_{};
};

// Aircraft-facing adapter. This is the only propeller type registered with
// NavionModel. It is deterministic and stateless across RK4 stages.
class PropellerComponent final : public ILoadComponent
{
public:
    explicit PropellerComponent(PropellerModel model);
    explicit PropellerComponent(PropellerParameters parameters);

    [[nodiscard]] BodyLoad computeLoad(
        const EvaluationContext& context
    ) const override;

    [[nodiscard]] std::string_view name() const noexcept override;

    [[nodiscard]] PropellerOutput evaluateDetailed(
        const EvaluationContext& context
    ) const;

    [[nodiscard]] const PropellerModel& model() const noexcept;

private:
    [[nodiscard]] static RuntimeInput makeRuntimeInput(
        const EvaluationContext& context
    );

    PropellerModel model_;
};

// Every numerical value in this factory is an explicitly provisional V1
// engineering estimate. Replace it with authoritative digitized geometry and
// validated polar data without changing the common component contract.
[[nodiscard]] PropellerParameters makeEstimatedNavionNaca5868_9Parameters();

} // namespace navion::propeller
