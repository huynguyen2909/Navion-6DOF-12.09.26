#include "navion/config/ProvisionalNavionConfig.hpp"

#include <memory>

namespace navion
{
namespace
{

HorizontalStabilizerConfig makeProvisionalHorizontalStabilizer()
{
    HorizontalStabilizerConfig config;
    config.Area = 3.99483072;
    config.TailSpan = 3.99741452;
    config.TailMAC = 1.01236110;
    config.PositionWrtCG[0] = -4.35074096;
    config.PositionWrtCG[2] = 0.4;
    config.ElevatorArea = 1.1;
    config.ElevatorMechanicalLimit[0] = -0.52;
    config.ElevatorMechanicalLimit[1] = 0.35;
    config.LiftCurveSlope = 3.324;
    config.ZeroLiftDragCoefficient = 0.010;
    config.InducedDragFactor = 0.0884;
    config.ElevatorEffectiveness = 0.447;
    return config;
}

VerticalStabilizerConfig makeProvisionalVerticalStabilizer()
{
    VerticalStabilizerConfig config;
    config.Area = 1.356384384;
    config.RudderArea = 0.7738823232;
    config.TailSpan = 1.50;
    config.TailMAC = 0.90;
    config.PositionWrtCG[0] = -4.649228943;
    config.PositionWrtCG[2] = -0.765145070;
    config.RudderMechanicalLimit[0] = -0.40;
    config.RudderMechanicalLimit[1] = 0.30;
    config.SideForceCurveSlope = 3.694542443;
    config.RudderEffectiveness = 0.523912357;
    config.ZeroLiftDragCoefficient = 0.012;
    config.InducedDragFactor = 0.10;
    return config;
}

landing_gear::LandingGearParameters makeProvisionalLandingGear()
{
    using landing_gear::BrakeGroup;
    using landing_gear::GearParameters;
    using landing_gear::LandingGearParameters;
    using landing_gear::SteeringMode;

    LandingGearParameters parameters;
    parameters.solverMaximumIterations = 50;
    parameters.solverConvergenceTolerance = 1.0e-5;
    parameters.maximumStrutForceN = 50000.0;

    GearParameters nose;
    nose.name = "NOSE_LG";
    nose.locationFromCgBodyM = {2.5, 0.0, 1.0};
    nose.springStiffnessNpm = 60000.0;
    nose.dampingCoefficientNsPm = 12000.0;
    nose.staticFrictionCoefficient = 0.5;
    nose.rollingFrictionCoefficient = 0.02;
    nose.maximumSteeringAngleRad = 0.12;
    nose.steeringMode = SteeringMode::Steered;
    nose.brakeGroup = BrakeGroup::None;

    GearParameters left = nose;
    left.name = "LEFT_MLG";
    left.locationFromCgBodyM = {-1.0, -1.3, 1.0};
    left.springStiffnessNpm = 75000.0;
    left.dampingCoefficientNsPm = 15000.0;
    left.maximumSteeringAngleRad = 0.0;
    left.steeringMode = SteeringMode::Fixed;
    left.brakeGroup = BrakeGroup::Left;

    GearParameters right = left;
    right.name = "RIGHT_MLG";
    right.locationFromCgBodyM.y = 1.3;
    right.brakeGroup = BrakeGroup::Right;
    parameters.gears = {nose, left, right};
    return parameters;
}

} // namespace

ProvisionalNavionConfig makeProvisionalNavionConfig()
{
    ProvisionalNavionConfig config;
    config.massProperties = {
        1246.32,
        Matrix3::diagonal(1420.9, 4067.45, 5328.36)
    };
    config.mainWing = MainWingConfig{};
    config.horizontalStabilizer = makeProvisionalHorizontalStabilizer();
    config.verticalStabilizer = makeProvisionalVerticalStabilizer();
    config.fuselage = fuselage::makeProvisionalNavionFuselageConfig();
    config.propeller = propeller::makeEstimatedNavionNaca5868_9Parameters();
    config.propeller.radialElementCount = 12U;
    config.propeller.azimuthStationCount = 16U;
    config.landingGear = makeProvisionalLandingGear();
    return config;
}

void addProvisionalNavionComponents(
    NavionModel& model,
    const ProvisionalNavionConfig& config
)
{
    model.addLoadComponent(
        std::make_unique<MainWing>(config.mainWing)
    );
    model.addLoadComponent(
        std::make_unique<HorizontalStabilizer>(config.horizontalStabilizer)
    );
    model.addLoadComponent(
        std::make_unique<VerticalStabilizer>(config.verticalStabilizer)
    );
    model.addLoadComponent(
        std::make_unique<fuselage::FuselageComponent>(config.fuselage)
    );
    model.addLoadComponent(
        std::make_unique<propeller::PropellerComponent>(config.propeller)
    );
    model.setGroundContactComponent(
        std::make_unique<landing_gear::LandingGearComponent>(
            config.landingGear
        )
    );
}

} // namespace navion
