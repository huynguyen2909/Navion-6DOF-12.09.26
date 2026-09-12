#include "navion/dynamics/RigidBody6DOF.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navion
{
namespace
{

constexpr double MINIMUM_MASS_KG = 1.0e-12;
constexpr double MINIMUM_INERTIA_MINOR = 1.0e-12;

} // namespace

RigidBody6DOF::RigidBody6DOF(const MassProperties& massProperties)
    : massProperties_(massProperties)
{
    validateMassProperties(massProperties_);
    inverseInertiaBody_ = massProperties_.inertiaBodyKgM2.inverse();
}

StateDerivative RigidBody6DOF::evaluate(
    const RigidBodyState& state,
    const BodyLoad& totalComponentLoad,
    const Vec3& gravityNedMps2
) const
{
    if (!state.isFinite())
    {
        throw std::invalid_argument("RigidBody6DOF received a non-finite state.");
    }
    if (!totalComponentLoad.isFinite())
    {
        throw std::invalid_argument("RigidBody6DOF received a non-finite load.");
    }
    if (!gravityNedMps2.isFinite())
    {
        throw std::invalid_argument("RigidBody6DOF received non-finite gravity.");
    }

    const Quaternion attitude = state.attitudeBodyToNed.normalized();
    const Quaternion attitudeNedToBody = attitude.conjugate();

    const Vec3 gravityBodyMps2 =
        attitudeNedToBody.rotate(gravityNedMps2);

    const Vec3 transportAccelerationBodyMps2 =
        cross(state.angularRateBodyRadps, state.velocityBodyMps);

    const Vec3 linearAccelerationBodyMps2 =
        totalComponentLoad.forceBodyN / massProperties_.massKg +
        gravityBodyMps2 -
        transportAccelerationBodyMps2;

    const Vec3 angularMomentumBody =
        massProperties_.inertiaBodyKgM2 * state.angularRateBodyRadps;

    const Vec3 angularAccelerationBodyRadps2 =
        inverseInertiaBody_ *
        (
            totalComponentLoad.momentAboutCgBodyNm -
            cross(state.angularRateBodyRadps, angularMomentumBody)
        );

    StateDerivative derivative;
    derivative.positionRateNedMps = attitude.rotate(state.velocityBodyMps);
    derivative.velocityRateBodyMps2 = linearAccelerationBodyMps2;
    derivative.attitudeRate = quaternionDerivativeBodyToNed(
        attitude,
        state.angularRateBodyRadps
    );
    derivative.angularAccelerationBodyRadps2 = angularAccelerationBodyRadps2;

    if (!derivative.isFinite())
    {
        throw std::runtime_error("RigidBody6DOF produced a non-finite derivative.");
    }

    return derivative;
}

const MassProperties& RigidBody6DOF::massProperties() const noexcept
{
    return massProperties_;
}

const Matrix3& RigidBody6DOF::inverseInertiaBodyKgM2() const noexcept
{
    return inverseInertiaBody_;
}

void RigidBody6DOF::validateMassProperties(
    const MassProperties& massProperties
)
{
    if (!std::isfinite(massProperties.massKg) ||
        massProperties.massKg <= MINIMUM_MASS_KG)
    {
        throw std::invalid_argument("Mass must be finite and greater than zero.");
    }

    const Matrix3& inertia = massProperties.inertiaBodyKgM2;
    if (!inertia.isFinite())
    {
        throw std::invalid_argument("The BODY inertia tensor must be finite.");
    }

    const double matrixScale = std::max({
        1.0,
        std::abs(inertia(0, 0)),
        std::abs(inertia(1, 1)),
        std::abs(inertia(2, 2))
    });

    if (!inertia.isSymmetric(1.0e-12 * matrixScale))
    {
        throw std::invalid_argument("The BODY inertia tensor must be symmetric.");
    }

    // Sylvester's criterion for a symmetric positive-definite 3x3 matrix.
    const double firstMinor = inertia(0, 0);
    const double secondMinor =
        inertia(0, 0) * inertia(1, 1) -
        inertia(0, 1) * inertia(1, 0);
    const double thirdMinor = inertia.determinant();

    if (firstMinor <= MINIMUM_INERTIA_MINOR ||
        secondMinor <= MINIMUM_INERTIA_MINOR ||
        thirdMinor <= MINIMUM_INERTIA_MINOR)
    {
        throw std::invalid_argument(
            "The BODY inertia tensor must be positive definite."
        );
    }
}

} // namespace navion
