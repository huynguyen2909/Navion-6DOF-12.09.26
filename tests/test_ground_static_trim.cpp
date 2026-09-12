#include "navion/config/ProvisionalNavionConfig.hpp"
#include "navion/initialization/GroundStaticTrim.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace
{

void require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

} // namespace

int main()
{
    const navion::ProvisionalNavionConfig aircraft =
        navion::makeProvisionalNavionConfig();
    navion::NavionModel model(aircraft.massProperties);
    navion::addProvisionalNavionComponents(model, aircraft);

    navion::RigidBodyState initialGuess;
    initialGuess.positionNedM.z = -0.90;
    initialGuess.attitudeBodyToNed =
        navion::Quaternion{0.9999, 0.005, -0.010, 0.0}.normalized();
    initialGuess.velocityBodyMps = {1.0, -0.5, 0.25};
    initialGuess.angularRateBodyRadps = {0.1, -0.1, 0.05};

    navion::ControlInputs controls;
    controls.throttle = 0.0;
    controls.propellerEnabled = false;
    controls.landingGearExtended = true;
    controls.brakeLeft = 1.0;
    controls.brakeRight = 1.0;

    navion::GroundStaticTrimSolver solver;
    const navion::GroundStaticTrimResult result = solver.solve(
        model,
        initialGuess,
        controls,
        navion::Environment{}
    );

    require(result.converged, "Ground static trim did not converge.");
    require(result.iterations > 0U, "Perturbed trim guess was not iterated.");
    require(result.evaluation.groundContactCount == 3U,
            "Ground static trim did not retain all three contacts.");
    require(std::abs(result.residual.netForceBodyN.z) <= 0.1,
            "Ground trim vertical-force residual is too large.");
    require(std::abs(result.residual.netMomentAboutCgBodyNm.x) <= 0.1,
            "Ground trim roll-moment residual is too large.");
    require(std::abs(result.residual.netMomentAboutCgBodyNm.y) <= 0.1,
            "Ground trim pitch-moment residual is too large.");
    require(result.state.velocityBodyMps.norm() == 0.0,
            "Ground trim must return zero translational velocity.");
    require(result.state.angularRateBodyRadps.norm() == 0.0,
            "Ground trim must return zero angular rate.");

    std::cout << "Ground static trim test passed in "
              << result.iterations << " iterations.\n";
    return 0;
}
