#pragma once

#include "navion/core/MathTypes.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace navion
{

struct BodyLoad
{
    // Non-gravitational component loads, expressed in BODY FRD axes.
    Vec3 forceBodyN{};
    Vec3 momentAboutCgBodyNm{};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return forceBodyN.isFinite() && momentAboutCgBodyNm.isFinite();
    }

    BodyLoad& operator+=(const BodyLoad& rhs) noexcept
    {
        forceBodyN += rhs.forceBodyN;
        momentAboutCgBodyNm += rhs.momentAboutCgBodyNm;
        return *this;
    }
};

[[nodiscard]] inline BodyLoad makeBodyLoadAtPoint(
    const Vec3& forceBodyN,
    const Vec3& intrinsicMomentAtPointBodyNm,
    const Vec3& positionFromCgToPointBodyM
) noexcept
{
    return {
        forceBodyN,
        intrinsicMomentAtPointBodyNm +
            cross(positionFromCgToPointBodyM, forceBodyN)
    };
}

[[nodiscard]] inline BodyLoad operator+(BodyLoad lhs, const BodyLoad& rhs) noexcept
{
    lhs += rhs;
    return lhs;
}

struct MassProperties
{
    double massKg{0.0};
    Matrix3 inertiaBodyKgM2{};
};

struct RigidBodyState
{
    Vec3 positionNedM{};
    Vec3 velocityBodyMps{};
    Quaternion attitudeBodyToNed{};
    Vec3 angularRateBodyRadps{};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return
            positionNedM.isFinite() &&
            velocityBodyMps.isFinite() &&
            attitudeBodyToNed.isFinite() &&
            angularRateBodyRadps.isFinite();
    }
};

struct StateDerivative
{
    Vec3 positionRateNedMps{};
    Vec3 velocityRateBodyMps2{};
    Quaternion attitudeRate{};
    Vec3 angularAccelerationBodyRadps2{};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return
            positionRateNedMps.isFinite() &&
            velocityRateBodyMps2.isFinite() &&
            attitudeRate.isFinite() &&
            angularAccelerationBodyRadps2.isFinite();
    }
};

struct ControlInputs
{
    double throttle{0.0};
    double elevatorRad{0.0};
    double aileronRad{0.0};
    double rudderRad{0.0};
    double flapRad{0.0};
    double brakeLeft{0.0};
    double brakeRight{0.0};
    double noseWheelSteeringRad{0.0};

    // Multiplies the configured PropellerParameters rotation rate. 0.0 means
    // stopped and 1.0 selects the configured 2300-RPM design point. This is a
    // prescribed speed command, not an engine/governor dynamic state.
    double propellerSpeedScale{1.0};

    // Discrete engine/propeller availability. V1 keeps blade pitch fixed and
    // accepts the prescribed speed scale above; engine dynamics are deferred.
    bool propellerEnabled{true};

    // Common extension switch for the current fixed-geometry V1 gear model.
    bool landingGearExtended{true};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return
            std::isfinite(throttle) &&
            std::isfinite(elevatorRad) &&
            std::isfinite(aileronRad) &&
            std::isfinite(rudderRad) &&
            std::isfinite(flapRad) &&
            std::isfinite(brakeLeft) &&
            std::isfinite(brakeRight) &&
            std::isfinite(noseWheelSteeringRad) &&
            std::isfinite(propellerSpeedScale) &&
            propellerSpeedScale >= 0.0;
    }
};

struct Environment
{
    double airDensityKgM3{1.225};
    double dynamicViscosityPaS{1.7894e-5};
    double speedOfSoundMps{340.294};

    // Air-mass velocity relative to Earth, expressed in NED axes.
    Vec3 windVelocityNedMps{};

    // NED uses +Down, so standard gravity is positive on the third axis.
    Vec3 gravityNedMps2{0.0, 0.0, 9.80665};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return
            std::isfinite(airDensityKgM3) &&
            std::isfinite(dynamicViscosityPaS) &&
            std::isfinite(speedOfSoundMps) &&
            windVelocityNedMps.isFinite() &&
            gravityNedMps2.isFinite();
    }
};

struct FlightCondition
{
    Vec3 airRelativeVelocityBodyMps{};
    double airspeedMps{0.0};
    double angleOfAttackRad{0.0};
    double sideSlipRad{0.0};
    double mach{0.0};
    double dynamicPressurePa{0.0};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return
            airRelativeVelocityBodyMps.isFinite() &&
            std::isfinite(airspeedMps) &&
            std::isfinite(angleOfAttackRad) &&
            std::isfinite(sideSlipRad) &&
            std::isfinite(mach) &&
            std::isfinite(dynamicPressurePa);
    }
};

// Per-contact diagnostics returned by NavionModel without exposing the
// concrete LandingGearComponent type to the simulation driver.
struct GroundContactReport
{
    std::string name{};
    bool weightOnWheels{false};
    double compressionM{0.0};
    double normalForceN{0.0};
    double rollingFrictionN{0.0};
    double lateralFrictionN{0.0};
    double wheelSlipDeg{0.0};
    double steeringAngleRad{0.0};
};

// Diagnostic copy of one component's BODY-axis load about the aircraft CG.
// It does not participate in load summation a second time; NavionModel stores
// it while adding the same BodyLoad to LoadAccumulator.
struct ComponentLoadReport
{
    std::string name{};
    BodyLoad load{};
};

struct ModelEvaluation
{
    FlightCondition flightCondition{};
    BodyLoad totalComponentLoad{};
    StateDerivative stateDerivative{};
    std::size_t contributingComponentCount{0U};
    BodyLoad groundNormalLoad{};
    BodyLoad groundFrictionLoad{};
    std::size_t groundContactCount{0U};
    std::size_t frictionSolverIterations{0U};
    bool frictionSolverConverged{true};
    std::vector<GroundContactReport> groundContacts{};
    std::vector<ComponentLoadReport> componentLoads{};
};

} // namespace navion
