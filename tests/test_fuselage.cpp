#include "navion/components/fuselage/FuselageComponent.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected)
        );
    }
}

void requireVecNear(
    const navion::Vec3& actual,
    const navion::Vec3& expected,
    double tolerance,
    const std::string& message
)
{
    requireNear(actual.x, expected.x, tolerance, message + " [x]");
    requireNear(actual.y, expected.y, tolerance, message + " [y]");
    requireNear(actual.z, expected.z, tolerance, message + " [z]");
}

struct Fixture
{
    navion::RigidBodyState state{};
    navion::ControlInputs controls{};
    navion::Environment environment{};
    navion::FlightCondition flight{};
    navion::MassProperties mass{
        1000.0,
        navion::Matrix3::diagonal(1000.0, 1500.0, 1800.0)
    };

    navion::EvaluationContext context() const
    {
        return {0.0, state, controls, environment, flight, mass};
    }
};

void testOriginalNicolosiDragRegression()
{
    navion::fuselage::FuselageAerodynamics model(
        navion::fuselage::makeNicolosiReferenceFuselageGeometry(),
        navion::fuselage::FuselageTuning{}
    );
    navion::fuselage::FuselageAeroState state;
    state.rho_kgm3 = 1.0;
    state.mu_pas = 1.0;
    state.V_mps = 202.0e6 / 30.0;
    state.mach = 0.52;

    const auto output = model.evaluate(state);
    requireNear(output.reynolds, 202.0e6, 1.0e-6,
                "Original fuselage Reynolds regression");
    requireNear(output.cd_fuselage_sfront, 0.05450, 1.0e-5,
                "Original fuselage drag regression");
}

void testZeroSpeedBoundary()
{
    navion::fuselage::FuselageComponent fuselage(
        navion::fuselage::makeT6cReferenceFuselageConfig()
    );
    Fixture fixture;
    const auto result = fuselage.evaluateDetailed(fixture.context());
    require(result.zeroDynamicPressureBypass,
            "Fuselage marks exact V=0 limiting branch");
    require(result.bodyLoad.isFinite(), "Fuselage V=0 result is finite");
    requireVecNear(result.bodyLoad.forceBodyN, {}, 0.0, "Fuselage V=0 force");
    requireVecNear(result.bodyLoad.momentAboutCgBodyNm, {}, 0.0,
                   "Fuselage V=0 moment");
}

void testAdapterDirectionAndMomentTransfer()
{
    auto config = navion::fuselage::makeT6cReferenceFuselageConfig();
    config.aerodynamicReferencePositionFromCgBodyM = {1.2, -0.1, 0.4};
    navion::fuselage::FuselageComponent fuselage(config);

    Fixture fixture;
    fixture.flight.airRelativeVelocityBodyMps = {58.0, 5.0, 14.0};
    fixture.flight.airspeedMps =
        fixture.flight.airRelativeVelocityBodyMps.norm();
    fixture.flight.angleOfAttackRad = std::atan2(14.0, 58.0);
    fixture.flight.sideSlipRad = std::atan2(5.0, std::hypot(58.0, 14.0));
    fixture.flight.mach =
        fixture.flight.airspeedMps / fixture.environment.speedOfSoundMps;
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 *
        fixture.flight.airspeedMps * fixture.flight.airspeedMps;

    const auto result = fuselage.evaluateDetailed(fixture.context());
    requireNear(result.dragForceBodyN.norm(), result.aerodynamicOutput.drag_n,
                1.0e-10, "Adapter preserves scalar drag magnitude");
    require(navion::dot(
                result.dragForceBodyN,
                fixture.flight.airRelativeVelocityBodyMps
            ) < 0.0,
            "Fuselage drag opposes air-relative velocity");
    const navion::Vec3 expectedMoment =
        result.intrinsicMomentAtReferenceBodyNm +
        navion::cross(
            config.aerodynamicReferencePositionFromCgBodyM,
            result.dragForceBodyN
        );
    requireVecNear(result.bodyLoad.momentAboutCgBodyNm, expectedMoment, 1.0e-10,
                   "Fuselage transfers r cross F exactly once");
    require(!result.warnings.empty(), "Fuselage calibration warning retained");
}

void testBodyNegativeXMode()
{
    auto config = navion::fuselage::makeT6cReferenceFuselageConfig();
    config.dragDirectionMode =
        navion::fuselage::DragDirectionMode::BodyNegativeX;
    navion::fuselage::FuselageComponent fuselage(config);

    Fixture fixture;
    fixture.flight.airRelativeVelocityBodyMps = {40.0, 2.0, 3.0};
    fixture.flight.airspeedMps = fixture.flight.airRelativeVelocityBodyMps.norm();
    fixture.flight.angleOfAttackRad = std::atan2(3.0, 40.0);
    fixture.flight.sideSlipRad = std::atan2(2.0, std::hypot(40.0, 3.0));
    fixture.flight.mach =
        fixture.flight.airspeedMps / fixture.environment.speedOfSoundMps;
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 *
        fixture.flight.airspeedMps * fixture.flight.airspeedMps;

    const auto result = fuselage.evaluateDetailed(fixture.context());
    requireNear(result.dragForceBodyN.x, -result.aerodynamicOutput.drag_n,
                1.0e-12, "BODY -X drag magnitude");
    requireNear(result.dragForceBodyN.y, 0.0, 0.0, "BODY -X drag y");
    requireNear(result.dragForceBodyN.z, 0.0, 0.0, "BODY -X drag z");
}

} // namespace

int main()
{
    testOriginalNicolosiDragRegression();
    testZeroSpeedBoundary();
    testAdapterDirectionAndMomentTransfer();
    testBodyNegativeXMode();
    std::cout << "Fuselage tests passed.\n";
    return 0;
}
