#pragma once

#include "navion/core/FlightTypes.hpp"

#include <string_view>

namespace navion
{

struct EvaluationContext
{
    double timeS{0.0};
    const RigidBodyState& state;
    const ControlInputs& controls;
    const Environment& environment;
    const FlightCondition& flightCondition;
    const MassProperties& massProperties;
    // Optional cached inverse owned by RigidBody6DOF. Standalone component
    // tests may leave this null; coupled solvers then compute it locally.
    const Matrix3* inverseInertiaBodyKgM2{nullptr};
};

class ILoadComponent
{
public:
    virtual ~ILoadComponent() = default;

    [[nodiscard]] virtual BodyLoad computeLoad(
        const EvaluationContext& context
    ) const = 0;

    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

} // namespace navion
