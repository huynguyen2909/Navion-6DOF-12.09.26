#pragma once

#include "navion/components/IGroundContactComponent.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace navion::landing_gear
{

enum class SteeringMode
{
    Fixed,
    Steered
};

enum class BrakeGroup
{
    None,
    Left,
    Right
};

struct GearParameters
{
    std::string name{};
    Vec3 locationFromCgBodyM{};
    double springStiffnessNpm{0.0};
    double dampingCoefficientNsPm{0.0};
    double staticFrictionCoefficient{0.0};
    double rollingFrictionCoefficient{0.0};
    double maximumSteeringAngleRad{0.0};
    SteeringMode steeringMode{SteeringMode::Fixed};
    BrakeGroup brakeGroup{BrakeGroup::None};
    double pacejkaStiffnessB{0.06};
    double pacejkaShapeC{2.8};
    double pacejkaCurvatureE{1.03};

    void validate() const;
};

struct LandingGearParameters
{
    int solverMaximumIterations{50};
    double solverConvergenceTolerance{1.0e-5};
    double rollingFrictionFactor{1.0};
    double staticFrictionFactor{1.0};
    double maximumStrutForceN{0.0}; // zero means unlimited
    double groundDownPositionNedM{0.0};
    std::vector<GearParameters> gears{};

    void validate() const;
};

// Pure PGS implementation of the uploaded Jac*M^-1*Jac^T solver.
class PGSFrictionSolver final
{
public:
    PGSFrictionSolver(int maximumIterations, double convergenceTolerance);

    [[nodiscard]] FrictionSolveResult solve(
        const std::vector<FrictionConstraint>& constraints,
        const BodyLoad& preFrictionLoadIncludingGravity,
        const RigidBodyState& state,
        const MassProperties& massProperties,
        double timeStepS,
        const Matrix3* inverseInertiaBodyKgM2 = nullptr
    ) const;

private:
    int maximumIterations_{50};
    double convergenceTolerance_{1.0e-5};
};

class LandingGearComponent final : public IGroundContactComponent
{
public:
    explicit LandingGearComponent(LandingGearParameters parameters);

    [[nodiscard]] GroundContactEvaluation evaluateContacts(
        const EvaluationContext& context,
        double timeStepS
    ) const override;

    [[nodiscard]] FrictionSolveResult solveFriction(
        const GroundContactEvaluation& contacts,
        const BodyLoad& preFrictionLoadIncludingGravity,
        const EvaluationContext& context,
        double timeStepS
    ) const override;

    void applyFrictionResult(
        GroundContactEvaluation& contacts,
        const FrictionSolveResult& friction
    ) const override;

    void commitAcceptedStep(
        const GroundContactEvaluation& contacts,
        const FrictionSolveResult& friction
    ) override;

    void resetHistory() noexcept override;

    [[nodiscard]] std::string_view name() const noexcept override;
    [[nodiscard]] const LandingGearParameters& parameters() const noexcept;

private:
    struct GearHistory
    {
        bool weightOnWheels{false};
        double compressionM{0.0};
        double normalForceN{0.0};
        double steeringAngleRad{0.0};
        double wheelSlipDeg{0.0};
        Vec3 rollingDirectionBody{1.0, 0.0, 0.0};
        Vec3 lateralDirectionBody{0.0, 1.0, 0.0};
        Vec3 normalDirectionBody{0.0, 0.0, -1.0};
        Vec3 contactPositionFromCgBodyM{};
        double previousRollingMultiplierN{0.0};
        double previousLateralMultiplierN{0.0};
    };

    [[nodiscard]] GroundContactPoint evaluateGear(
        const GearParameters& gear,
        const GearHistory& history,
        const EvaluationContext& context,
        double timeStepS
    ) const;

    [[nodiscard]] double brakeCommand(
        BrakeGroup group,
        const ControlInputs& controls
    ) const noexcept;

    LandingGearParameters parameters_{};
    PGSFrictionSolver solver_;
    std::vector<GearHistory> committedHistory_{};
};

// Exact SI values from the uploaded T-6C YAML; reference only, not Navion data.
[[nodiscard]] LandingGearParameters makeT6cReferenceLandingGearParameters();
[[nodiscard]] MassProperties makeT6cReferenceMassProperties();

} // namespace navion::landing_gear
