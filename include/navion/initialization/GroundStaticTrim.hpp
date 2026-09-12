#pragma once

#include "navion/model/NavionModel.hpp"

#include <cstddef>

namespace navion
{

// Solves the three static ground-equilibrium unknowns used by the V1 model:
// CG Down position, roll attitude and pitch attitude. North/East position and
// yaw are retained from the initial guess; translational and angular rates are
// forced to zero during every residual evaluation.
struct GroundStaticTrimSettings
{
    double timeStepS{0.002};
    std::size_t maximumIterations{20U};
    double forceToleranceN{0.1};
    double momentToleranceNm{0.1};
    double verticalDifferenceStepM{1.0e-5};
    double attitudeDifferenceStepRad{1.0e-6};
    double maximumVerticalCorrectionM{0.02};
    double maximumAttitudeCorrectionRad{0.01};
    std::size_t requiredContactCount{3U};
};

struct GroundStaticTrimResidual
{
    Vec3 netForceBodyN{};
    Vec3 netMomentAboutCgBodyNm{};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return netForceBodyN.isFinite() &&
            netMomentAboutCgBodyNm.isFinite();
    }
};

struct GroundStaticTrimResult
{
    RigidBodyState state{};
    ModelEvaluation evaluation{};
    GroundStaticTrimResidual residual{};
    std::size_t iterations{0U};
    bool converged{false};
};

class GroundStaticTrimSolver final
{
public:
    explicit GroundStaticTrimSolver(
        GroundStaticTrimSettings settings = {}
    );

    // For a pre-power ground trim, pass controls with throttle = 0,
    // propellerEnabled = false, gear extended and parking brakes applied.
    [[nodiscard]] GroundStaticTrimResult solve(
        const NavionModel& model,
        const RigidBodyState& initialGuess,
        const ControlInputs& staticControls,
        const Environment& environment
    ) const;

    [[nodiscard]] const GroundStaticTrimSettings& settings() const noexcept;

private:
    GroundStaticTrimSettings settings_{};
};

} // namespace navion
