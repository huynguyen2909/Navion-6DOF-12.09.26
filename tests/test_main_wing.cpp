#include "navion/components/wings/VATC_MainWing.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

constexpr double PI = 3.14159265358979323846;

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

void testUploadedDefaultRegression()
{
    navion::MainWing wing(navion::MainWingConfig{});
    Fixture fixture;
    fixture.environment.airDensityKgM3 = 1.0556;
    fixture.flight.airspeedMps = 43.9;
    fixture.flight.airRelativeVelocityBodyMps = {43.9, 0.0, 0.0};
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 * 43.9 * 43.9;
    fixture.flight.mach = 43.9 / fixture.environment.speedOfSoundMps;
    fixture.controls.flapRad = 20.0 * PI / 180.0;

    const auto result = wing.evaluateDetailed(fixture.context());
    requireNear(result.liftCoefficient, 1.0281250404461793, 1.0e-12,
                "Uploaded MainWing CL regression");
    requireNear(result.dragCoefficient, 0.14181238828955073, 1.0e-12,
                "Uploaded MainWing CD regression");
    requireVecNear(
        result.bodyLoad.forceBodyN,
        {-2468.3876738450695, 0.0, -17895.553467636},
        1.0e-8,
        "Uploaded MainWing dimensional-force regression"
    );
    requireVecNear(
        result.bodyLoad.momentAboutCgBodyNm,
        {0.0, -6247.197464641232, 0.0},
        1.0e-8,
        "Uploaded MainWing dimensional-moment regression"
    );
    require(!result.warnings.empty(), "MainWing provenance warnings retained");
}

void testZeroSpeedAndControlPaths()
{
    navion::MainWing wing(navion::MainWingConfig{});
    Fixture fixture;
    fixture.controls.flapRad = 0.0;

    const auto zero = wing.evaluateDetailed(fixture.context());
    require(zero.bodyLoad.isFinite(), "MainWing V=0 result is finite");
    requireVecNear(zero.bodyLoad.forceBodyN, {}, 0.0, "MainWing V=0 force");
    requireVecNear(zero.bodyLoad.momentAboutCgBodyNm, {}, 0.0,
                   "MainWing V=0 moment");

    fixture.flight.airspeedMps = 50.0;
    fixture.flight.airRelativeVelocityBodyMps = {50.0, 0.0, 0.0};
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 * 50.0 * 50.0;
    fixture.flight.mach = 50.0 / fixture.environment.speedOfSoundMps;
    fixture.controls.aileronRad = 0.1;
    const auto aileron = wing.evaluateDetailed(fixture.context());
    require(std::abs(aileron.bodyLoad.momentAboutCgBodyNm.x) > 0.0,
            "Aileron produces wing roll moment");

    fixture.controls.flapRad = 41.0 * PI / 180.0;
    bool threw = false;
    try
    {
        (void)wing.computeLoad(fixture.context());
    }
    catch (const std::invalid_argument&)
    {
        threw = true;
    }
    require(threw, "MainWing retains 0..40 degree flap domain");
}

void testMomentTransferToCg()
{
    navion::MainWingConfig config;
    config.reference.output_h = 0.50;
    config.reference.output_z_m = -0.20;
    navion::MainWing wing(config);

    Fixture fixture;
    fixture.flight.airspeedMps = 45.0;
    fixture.flight.airRelativeVelocityBodyMps = {44.9, 2.0, 2.2};
    fixture.flight.angleOfAttackRad = 0.049;
    fixture.flight.sideSlipRad = 0.044;
    fixture.flight.dynamicPressurePa =
        0.5 * fixture.environment.airDensityKgM3 * 45.0 * 45.0;
    fixture.flight.mach = 45.0 / fixture.environment.speedOfSoundMps;
    fixture.controls.flapRad = 15.0 * PI / 180.0;

    const auto result = wing.evaluateDetailed(fixture.context());
    const navion::Vec3 referenceFromCg{
        result.momentTransferDxM,
        0.0,
        result.momentTransferDzM
    };
    const navion::Vec3 expectedMoment =
        result.intrinsicMomentAtAerodynamicReferenceBodyNm +
        navion::cross(referenceFromCg, result.forceBodyN);
    requireVecNear(result.bodyLoad.momentAboutCgBodyNm, expectedMoment, 1.0e-8,
                   "MainWing transfers r cross F exactly once");
}

} // namespace

int main()
{
    testUploadedDefaultRegression();
    testZeroSpeedAndControlPaths();
    testMomentTransferToCg();
    std::cout << "MainWing tests passed.\n";
    return 0;
}
