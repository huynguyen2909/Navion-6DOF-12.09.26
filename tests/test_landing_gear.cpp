#include "navion/components/ILoadComponent.hpp"
#include "navion/components/landing_gear/LandingGearComponent.hpp"
#include "navion/model/NavionModel.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

using navion::BodyLoad;
using navion::ControlInputs;
using navion::Environment;
using navion::EvaluationContext;
using navion::FlightCondition;
using navion::MassProperties;
using navion::Matrix3;
using navion::RigidBodyState;
using navion::Vec3;
using navion::landing_gear::BrakeGroup;
using navion::landing_gear::GearParameters;
using navion::landing_gear::LandingGearComponent;
using navion::landing_gear::LandingGearParameters;
using navion::landing_gear::PGSFrictionSolver;
using navion::landing_gear::SteeringMode;

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

template<typename Function>
void requireThrows(Function&& function, const std::string& message)
{
    bool threw = false;
    try
    {
        function();
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    require(threw, message);
}

MassProperties massProperties()
{
    return {1000.0, Matrix3::diagonal(1000.0, 1000.0, 1000.0)};
}

LandingGearParameters singleGearParameters()
{
    LandingGearParameters parameters;
    parameters.solverMaximumIterations = 100;
    parameters.solverConvergenceTolerance = 1.0e-10;

    GearParameters gear;
    gear.name = "TEST_GEAR";
    gear.locationFromCgBodyM = {0.0, 0.0, 1.0};
    gear.springStiffnessNpm = 10000.0;
    gear.dampingCoefficientNsPm = 0.0;
    gear.staticFrictionCoefficient = 0.5;
    gear.rollingFrictionCoefficient = 0.02;
    gear.maximumSteeringAngleRad = 0.2;
    gear.steeringMode = SteeringMode::Steered;
    gear.brakeGroup = BrakeGroup::Left;
    parameters.gears.push_back(gear);
    return parameters;
}

RigidBodyState contactingState()
{
    RigidBodyState state;
    // The uncompressed wheel is 0.05 m below NED z=0.
    state.positionNedM = {0.0, 0.0, -0.95};
    return state;
}

class ConstantLoadComponent final : public navion::ILoadComponent
{
public:
    explicit ConstantLoadComponent(BodyLoad load) : load_(load) {}

    BodyLoad computeLoad(const EvaluationContext&) const override
    {
        return load_;
    }

    std::string_view name() const noexcept override
    {
        return "ConstantLoad";
    }

private:
    BodyLoad load_{};
};

void testPurePgsProjection()
{
    PGSFrictionSolver solver(50, 1.0e-12);
    navion::FrictionConstraint constraint;
    constraint.directionBody = {1.0, 0.0, 0.0};
    constraint.minimumForceN = -100.0;
    constraint.maximumForceN = 100.0;

    RigidBodyState state;
    state.velocityBodyMps = {10.0, 0.0, 0.0};
    const auto result = solver.solve(
        {constraint},
        {},
        state,
        massProperties(),
        0.1
    );

    require(result.converged, "One-row PGS solve must converge.");
    requireNear(result.multipliersN.at(0), -100.0, 1.0e-12,
                "PGS must project to the friction lower bound");
    requireNear(result.frictionLoad.forceBodyN.x, -100.0, 1.0e-12,
                "PGS force must oppose forward velocity");
}

void testNoBrakeKeepsRollingFrictionAndPgs()
{
    LandingGearComponent gear(singleGearParameters());
    RigidBodyState state = contactingState();
    state.velocityBodyMps = {10.0, 0.0, 0.0};
    ControlInputs controls;
    controls.brakeLeft = 0.0;
    Environment environment;
    environment.gravityNedMps2 = {};
    const FlightCondition flightCondition{};
    const MassProperties mass = massProperties();
    const EvaluationContext context{
        0.0, state, controls, environment, flightCondition, mass
    };

    auto contacts = gear.evaluateContacts(context, 0.01);
    require(contacts.contacts.at(0).weightOnWheels,
            "The test wheel must contact the ground.");
    requireNear(contacts.contacts.at(0).compressionM, 0.05, 1.0e-12,
                "Strut compression");
    requireNear(contacts.contacts.at(0).normalForceN, 500.0, 1.0e-9,
                "Spring normal force");
    requireNear(contacts.frictionConstraints.at(0).maximumForceN, 10.0,
                1.0e-9, "Brake-zero rolling bound must be mu_roll times Fn");

    BodyLoad pre = contacts.normalLoad;
    const auto friction = gear.solveFriction(contacts, pre, context, 0.01);
    gear.applyFrictionResult(contacts, friction);
    require(friction.iterations > 0U,
            "PGS must run even when the brake command is zero.");
    requireNear(contacts.contacts.at(0).rollingFrictionN, -10.0, 1.0e-9,
                "Free-rolling friction must oppose takeoff-roll velocity");
}

void testBrakeSteeringAndPacejkaRemainAvailable()
{
    LandingGearComponent gear(singleGearParameters());
    RigidBodyState state = contactingState();
    state.velocityBodyMps = {10.0, 1.0, 0.0};
    ControlInputs controls;
    controls.brakeLeft = 1.0;
    controls.noseWheelSteeringRad = 0.5;
    Environment environment;
    const FlightCondition flightCondition{};
    const MassProperties mass = massProperties();
    const EvaluationContext context{
        0.0, state, controls, environment, flightCondition, mass
    };

    const auto contacts = gear.evaluateContacts(context, 0.01);
    requireNear(contacts.contacts.at(0).steeringAngleRad, 0.2, 1.0e-12,
                "Physical nose-wheel steering must clamp to its configured limit");
    requireNear(contacts.frictionConstraints.at(0).maximumForceN, 250.0,
                1.0e-9, "Full brake must recover the original static-friction bound");
    require(contacts.contacts.at(0).wheelSlipDeg != 0.0,
            "Lateral motion must update wheel slip.");
    require(contacts.frictionConstraints.at(1).maximumForceN > 0.0,
            "Pacejka slip must produce a lateral constraint bound.");
}

void testEvaluationIsReadOnlyUntilCommit()
{
    LandingGearComponent gear(singleGearParameters());
    ControlInputs controls;
    Environment environment;
    const FlightCondition flightCondition{};
    const MassProperties mass = massProperties();

    RigidBodyState moving = contactingState();
    moving.velocityBodyMps = {10.0, 2.0, 0.0};
    const EvaluationContext movingContext{
        0.0, moving, controls, environment, flightCondition, mass
    };
    auto movingContacts = gear.evaluateContacts(movingContext, 0.01);
    const double computedSlip = movingContacts.contacts.at(0).wheelSlipDeg;
    require(std::abs(computedSlip) > 1.0,
            "Moving trial state must generate a visible slip angle.");

    RigidBodyState stopped = contactingState();
    const EvaluationContext stoppedContext{
        0.0, stopped, controls, environment, flightCondition, mass
    };
    const auto beforeCommit = gear.evaluateContacts(stoppedContext, 0.01);
    requireNear(beforeCommit.contacts.at(0).wheelSlipDeg, 0.0, 0.0,
                "Trial-stage evaluation must not mutate accepted slip history");

    const auto friction = gear.solveFriction(
        movingContacts,
        movingContacts.normalLoad,
        movingContext,
        0.01
    );
    gear.applyFrictionResult(movingContacts, friction);
    gear.commitAcceptedStep(movingContacts, friction);

    const auto afterCommit = gear.evaluateContacts(stoppedContext, 0.01);
    requireNear(afterCommit.contacts.at(0).wheelSlipDeg, computedSlip,
                1.0e-12, "Accepted-step commit must retain low-speed slip history");
}

void testNavionTwoPhaseAssemblyAndDiagnostics()
{
    navion::NavionModel model(massProperties());
    model.addLoadComponent(std::make_unique<ConstantLoadComponent>(
        BodyLoad{{200.0, 0.0, 0.0}, {}}
    ));
    model.setGroundContactComponent(
        std::make_unique<LandingGearComponent>(singleGearParameters())
    );

    RigidBodyState state = contactingState();
    ControlInputs controls;
    controls.brakeLeft = 0.0;
    Environment environment;
    environment.gravityNedMps2 = {};

    const auto evaluation = model.evaluate(
        0.0,
        0.01,
        state,
        controls,
        environment
    );
    require(model.componentCount() == 2U,
            "Regular and ground-contact components must both be counted.");
    require(evaluation.contributingComponentCount == 2U,
            "The accumulator must add Landing Gear exactly once.");
    require(evaluation.groundContactCount == 1U,
            "NavionModel must expose weight-on-wheels diagnostics.");
    require(evaluation.groundContacts.size() == 1U,
            "Per-gear diagnostics must survive the common OOP boundary.");
    require(evaluation.frictionSolverIterations > 0U,
            "NavionModel must execute PGS after preliminary load assembly.");
    require(evaluation.groundFrictionLoad.forceBodyN.x < 0.0,
            "PGS must react to a sibling component's forward force.");
    requireNear(
        evaluation.totalComponentLoad.forceBodyN.x,
        200.0 + evaluation.groundFrictionLoad.forceBodyN.x,
        1.0e-10,
        "Final load must contain regular load plus solved ground load"
    );

    requireThrows(
        [&model, &state, &controls, &environment]() {
            (void)model.evaluate(0.0, state, controls, environment);
        },
        "The dt-free API must reject a configured PGS ground component."
    );

    model.commitAcceptedStep(0.01, 0.01, state, controls, environment);
    model.resetGroundContactHistory();
}

void testGravityIsNotDoubleCounted()
{
    navion::NavionModel model(massProperties());
    model.setGroundContactComponent(
        std::make_unique<LandingGearComponent>(singleGearParameters())
    );
    const RigidBodyState state = contactingState();
    const ControlInputs controls;
    const Environment environment;

    const auto evaluation = model.evaluate(
        0.0,
        0.01,
        state,
        controls,
        environment
    );
    requireNear(
        evaluation.stateDerivative.velocityRateBodyMps2.z,
        9.80665 - 500.0 / 1000.0,
        1.0e-10,
        "Gravity may feed the PGS RHS but must enter Newton-Euler exactly once"
    );
}

void testRetractionDisablesContactOnlyWhenRequested()
{
    LandingGearComponent gear(singleGearParameters());
    RigidBodyState state = contactingState();
    ControlInputs controls;
    controls.landingGearExtended = false;
    Environment environment;
    const FlightCondition flightCondition{};
    const MassProperties mass = massProperties();
    const EvaluationContext context{
        0.0, state, controls, environment, flightCondition, mass
    };

    const auto contacts = gear.evaluateContacts(context, 0.01);
    require(!contacts.contacts.at(0).weightOnWheels,
            "Retracted gear must not create ground contact.");
    require(contacts.frictionConstraints.empty(),
            "Retracted gear must not create PGS rows.");
}

void testOriginalThreeGearGlobalPgsPath()
{
    LandingGearComponent gear(
        navion::landing_gear::makeT6cReferenceLandingGearParameters()
    );
    RigidBodyState state;
    state.positionNedM.z = -2.0;
    state.velocityBodyMps = {12.0, 1.0, 0.0};
    ControlInputs controls;
    controls.brakeLeft = 0.0;
    controls.brakeRight = 0.0;
    Environment environment;
    const FlightCondition flightCondition{};
    const MassProperties mass =
        navion::landing_gear::makeT6cReferenceMassProperties();
    const EvaluationContext context{
        0.0, state, controls, environment, flightCondition, mass
    };

    auto contacts = gear.evaluateContacts(context, 0.01);
    require(contacts.contacts.size() == 3U,
            "The uploaded reference configuration must retain all three gears.");
    require(contacts.frictionConstraints.size() == 6U,
            "Three contacting gears must create one roll and one side PGS row each.");
    BodyLoad pre = contacts.normalLoad;
    pre.forceBodyN += {1500.0, 300.0, mass.massKg * 9.80665};
    const auto friction = gear.solveFriction(contacts, pre, context, 0.01);
    gear.applyFrictionResult(contacts, friction);
    require(friction.multipliersN.size() == 6U,
            "Global PGS must solve every coupled multiplier.");
    require(friction.frictionLoad.isFinite(),
            "Global three-gear PGS output must remain finite.");
}

} // namespace

int main()
{
    try
    {
        testPurePgsProjection();
        testNoBrakeKeepsRollingFrictionAndPgs();
        testBrakeSteeringAndPacejkaRemainAvailable();
        testEvaluationIsReadOnlyUntilCommit();
        testNavionTwoPhaseAssemblyAndDiagnostics();
        testGravityIsNotDoubleCounted();
        testRetractionDisablesContactOnlyWhenRequested();
        testOriginalThreeGearGlobalPgsPath();
        std::cout << "All Navion Stage 4 Landing Gear/PGS tests passed.\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] " << exception.what() << '\n';
        return 1;
    }
    return 0;
}
