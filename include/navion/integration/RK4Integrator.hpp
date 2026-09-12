#pragma once

#include "navion/core/FlightTypes.hpp"

#include <functional>

namespace navion
{

class RK4Integrator final
{
public:
    using DerivativeFunction = std::function<StateDerivative(
        double timeS,
        const RigidBodyState& state
    )>;

    [[nodiscard]] RigidBodyState step(
        double timeS,
        double timeStepS,
        const RigidBodyState& state,
        const DerivativeFunction& evaluateDerivative
    ) const;

private:
    [[nodiscard]] static RigidBodyState advance(
        const RigidBodyState& state,
        const StateDerivative& derivative,
        double scaleS
    );
};

} // namespace navion

