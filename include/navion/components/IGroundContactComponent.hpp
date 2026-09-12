#pragma once

#include "navion/components/ILoadComponent.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace navion
{

enum class FrictionAxis
{
    Rolling,
    Lateral
};

struct GroundContactPoint
{
    std::string name{};
    bool weightOnWheels{false};
    double compressionM{0.0};
    double normalForceN{0.0};
    double rollingFrictionN{0.0};
    double lateralFrictionN{0.0};
    double wheelSlipDeg{0.0};
    double steeringAngleRad{0.0};
    Vec3 rollingDirectionBody{};
    Vec3 lateralDirectionBody{};
    Vec3 normalDirectionBody{};
    Vec3 contactPositionFromCgBodyM{};
};

struct FrictionConstraint
{
    Vec3 directionBody{};
    Vec3 leverArmFromCgBodyM{};
    double minimumForceN{0.0};
    double maximumForceN{0.0};
    double warmStartForceN{0.0};
    std::size_t owningContactIndex{0U};
    FrictionAxis axis{FrictionAxis::Rolling};
};

struct GroundContactEvaluation
{
    BodyLoad normalLoad{};
    BodyLoad totalLoad{};
    std::vector<GroundContactPoint> contacts{};
    std::vector<FrictionConstraint> frictionConstraints{};
};

struct FrictionSolveResult
{
    BodyLoad frictionLoad{};
    std::vector<double> multipliersN{};
    std::size_t iterations{0U};
    bool converged{true};
};

// Ground contact is a coupled load producer: unlike an ordinary
// ILoadComponent, friction must be solved after all preliminary aircraft loads
// are known. Evaluation is read-only; history is changed only by commit.
class IGroundContactComponent
{
public:
    virtual ~IGroundContactComponent() = default;

    [[nodiscard]] virtual GroundContactEvaluation evaluateContacts(
        const EvaluationContext& context,
        double timeStepS
    ) const = 0;

    [[nodiscard]] virtual FrictionSolveResult solveFriction(
        const GroundContactEvaluation& contacts,
        const BodyLoad& preFrictionLoadIncludingGravity,
        const EvaluationContext& context,
        double timeStepS
    ) const = 0;

    virtual void applyFrictionResult(
        GroundContactEvaluation& contacts,
        const FrictionSolveResult& friction
    ) const = 0;

    virtual void commitAcceptedStep(
        const GroundContactEvaluation& contacts,
        const FrictionSolveResult& friction
    ) = 0;

    virtual void resetHistory() noexcept = 0;

    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

} // namespace navion
