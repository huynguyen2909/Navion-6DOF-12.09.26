#include "navion/integration/RK4Integrator.hpp"

#include <cmath>
#include <stdexcept>

namespace navion
{

RigidBodyState RK4Integrator::step(
    double timeS,
    double timeStepS,
    const RigidBodyState& state,
    const DerivativeFunction& evaluateDerivative
) const
{
    if (!std::isfinite(timeS) ||
        !std::isfinite(timeStepS) ||
        timeStepS <= 0.0)
    {
        throw std::invalid_argument("RK4 requires finite time and dt > 0.");
    }
    if (!state.isFinite())
    {
        throw std::invalid_argument("RK4 received a non-finite state.");
    }
    if (!evaluateDerivative)
    {
        throw std::invalid_argument("RK4 requires a derivative function.");
    }

    const RigidBodyState normalizedState{
        state.positionNedM,
        state.velocityBodyMps,
        state.attitudeBodyToNed.normalized(),
        state.angularRateBodyRadps
    };

    const StateDerivative k1 = evaluateDerivative(timeS, normalizedState);
    const RigidBodyState state2 = advance(normalizedState, k1, 0.5 * timeStepS);

    const StateDerivative k2 = evaluateDerivative(
        timeS + 0.5 * timeStepS,
        state2
    );
    const RigidBodyState state3 = advance(normalizedState, k2, 0.5 * timeStepS);

    const StateDerivative k3 = evaluateDerivative(
        timeS + 0.5 * timeStepS,
        state3
    );
    const RigidBodyState state4 = advance(normalizedState, k3, timeStepS);

    const StateDerivative k4 = evaluateDerivative(timeS + timeStepS, state4);

    if (!k1.isFinite() || !k2.isFinite() ||
        !k3.isFinite() || !k4.isFinite())
    {
        throw std::runtime_error("RK4 derivative function returned non-finite data.");
    }

    const double weight = timeStepS / 6.0;

    RigidBodyState result = normalizedState;
    result.positionNedM += weight * (
        k1.positionRateNedMps +
        2.0 * k2.positionRateNedMps +
        2.0 * k3.positionRateNedMps +
        k4.positionRateNedMps
    );
    result.velocityBodyMps += weight * (
        k1.velocityRateBodyMps2 +
        2.0 * k2.velocityRateBodyMps2 +
        2.0 * k3.velocityRateBodyMps2 +
        k4.velocityRateBodyMps2
    );
    result.attitudeBodyToNed += weight * (
        k1.attitudeRate +
        2.0 * k2.attitudeRate +
        2.0 * k3.attitudeRate +
        k4.attitudeRate
    );
    result.angularRateBodyRadps += weight * (
        k1.angularAccelerationBodyRadps2 +
        2.0 * k2.angularAccelerationBodyRadps2 +
        2.0 * k3.angularAccelerationBodyRadps2 +
        k4.angularAccelerationBodyRadps2
    );

    result.attitudeBodyToNed = result.attitudeBodyToNed.normalized();
    if (!result.isFinite())
    {
        throw std::runtime_error("RK4 produced a non-finite state.");
    }

    return result;
}

RigidBodyState RK4Integrator::advance(
    const RigidBodyState& state,
    const StateDerivative& derivative,
    double scaleS
)
{
    if (!derivative.isFinite() || !std::isfinite(scaleS))
    {
        throw std::invalid_argument("RK4 cannot advance using non-finite data.");
    }

    RigidBodyState result;
    result.positionNedM = state.positionNedM + scaleS * derivative.positionRateNedMps;
    result.velocityBodyMps = state.velocityBodyMps + scaleS * derivative.velocityRateBodyMps2;
    result.attitudeBodyToNed =
        (state.attitudeBodyToNed + scaleS * derivative.attitudeRate).normalized();
    result.angularRateBodyRadps =
        state.angularRateBodyRadps + scaleS * derivative.angularAccelerationBodyRadps2;
    return result;
}

} // namespace navion

