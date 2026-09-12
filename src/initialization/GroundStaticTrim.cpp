#include "navion/initialization/GroundStaticTrim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace navion
{
namespace
{

struct EulerAngles
{
    double rollRad{0.0};
    double pitchRad{0.0};
    double yawRad{0.0};
};

struct StaticEvaluation
{
    RigidBodyState state{};
    ModelEvaluation model{};
    GroundStaticTrimResidual residual{};
};

[[nodiscard]] EulerAngles quaternionToEuler(const Quaternion& attitude)
{
    const Quaternion q = attitude.normalized();
    EulerAngles result;
    result.rollRad = std::atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    );
    result.pitchRad = std::asin(std::clamp(
        2.0 * (q.w * q.y - q.z * q.x),
        -1.0,
        1.0
    ));
    result.yawRad = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    return result;
}

[[nodiscard]] Quaternion eulerToQuaternion(
    double rollRad,
    double pitchRad,
    double yawRad
)
{
    const double halfRoll = 0.5 * rollRad;
    const double halfPitch = 0.5 * pitchRad;
    const double halfYaw = 0.5 * yawRad;
    const double cr = std::cos(halfRoll);
    const double sr = std::sin(halfRoll);
    const double cp = std::cos(halfPitch);
    const double sp = std::sin(halfPitch);
    const double cy = std::cos(halfYaw);
    const double sy = std::sin(halfYaw);
    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

void validateSettings(const GroundStaticTrimSettings& settings)
{
    if (!std::isfinite(settings.timeStepS) || settings.timeStepS <= 0.0 ||
        settings.maximumIterations == 0U ||
        !std::isfinite(settings.forceToleranceN) ||
        settings.forceToleranceN <= 0.0 ||
        !std::isfinite(settings.momentToleranceNm) ||
        settings.momentToleranceNm <= 0.0 ||
        !std::isfinite(settings.verticalDifferenceStepM) ||
        settings.verticalDifferenceStepM <= 0.0 ||
        !std::isfinite(settings.attitudeDifferenceStepRad) ||
        settings.attitudeDifferenceStepRad <= 0.0 ||
        !std::isfinite(settings.maximumVerticalCorrectionM) ||
        settings.maximumVerticalCorrectionM <= 0.0 ||
        !std::isfinite(settings.maximumAttitudeCorrectionRad) ||
        settings.maximumAttitudeCorrectionRad <= 0.0 ||
        settings.requiredContactCount == 0U)
    {
        throw std::invalid_argument("Ground-static-trim settings are invalid.");
    }
}

[[nodiscard]] RigidBodyState makeStaticState(
    const RigidBodyState& reference,
    const std::array<double, 3>& unknowns,
    double yawRad
)
{
    RigidBodyState state = reference;
    state.positionNedM.z = unknowns[0];
    state.attitudeBodyToNed = eulerToQuaternion(
        unknowns[1],
        unknowns[2],
        yawRad
    ).normalized();
    state.velocityBodyMps = {};
    state.angularRateBodyRadps = {};
    return state;
}

[[nodiscard]] StaticEvaluation evaluateStatic(
    const NavionModel& model,
    const RigidBodyState& reference,
    const std::array<double, 3>& unknowns,
    double yawRad,
    const ControlInputs& controls,
    const Environment& environment,
    double timeStepS
)
{
    StaticEvaluation result;
    result.state = makeStaticState(reference, unknowns, yawRad);
    result.model = model.evaluate(
        0.0,
        timeStepS,
        result.state,
        controls,
        environment
    );

    const Quaternion attitude =
        result.state.attitudeBodyToNed.normalized();
    const Vec3 gravityBodyMps2 =
        attitude.conjugate().rotate(environment.gravityNedMps2);
    result.residual.netForceBodyN =
        result.model.totalComponentLoad.forceBodyN +
        model.massProperties().massKg * gravityBodyMps2;
    result.residual.netMomentAboutCgBodyNm =
        result.model.totalComponentLoad.momentAboutCgBodyNm;
    return result;
}

[[nodiscard]] Vec3 trimResidualVector(
    const GroundStaticTrimResidual& residual
) noexcept
{
    return {
        residual.netForceBodyN.z,
        residual.netMomentAboutCgBodyNm.x,
        residual.netMomentAboutCgBodyNm.y
    };
}

[[nodiscard]] double scaledResidualNorm(
    const GroundStaticTrimResidual& residual,
    const GroundStaticTrimSettings& settings
) noexcept
{
    const Vec3 vector = trimResidualVector(residual);
    return std::sqrt(
        (vector.x / settings.forceToleranceN) *
            (vector.x / settings.forceToleranceN) +
        (vector.y / settings.momentToleranceNm) *
            (vector.y / settings.momentToleranceNm) +
        (vector.z / settings.momentToleranceNm) *
            (vector.z / settings.momentToleranceNm)
    );
}

[[nodiscard]] bool converged(
    const StaticEvaluation& evaluation,
    const GroundStaticTrimSettings& settings
) noexcept
{
    const Vec3 residual = trimResidualVector(evaluation.residual);
    return evaluation.model.groundContactCount >=
            settings.requiredContactCount &&
        std::abs(residual.x) <= settings.forceToleranceN &&
        std::abs(residual.y) <= settings.momentToleranceNm &&
        std::abs(residual.z) <= settings.momentToleranceNm;
}

} // namespace

GroundStaticTrimSolver::GroundStaticTrimSolver(
    GroundStaticTrimSettings settings
)
    : settings_(settings)
{
    validateSettings(settings_);
}

GroundStaticTrimResult GroundStaticTrimSolver::solve(
    const NavionModel& model,
    const RigidBodyState& initialGuess,
    const ControlInputs& staticControls,
    const Environment& environment
) const
{
    if (!initialGuess.isFinite() || !staticControls.isFinite() ||
        !environment.isFinite())
    {
        throw std::invalid_argument(
            "Ground static trim received non-finite input."
        );
    }
    if (!model.hasGroundContactComponent())
    {
        throw std::logic_error(
            "Ground static trim requires a ground-contact component."
        );
    }

    const EulerAngles initialEuler =
        quaternionToEuler(initialGuess.attitudeBodyToNed);
    std::array<double, 3> unknowns{
        initialGuess.positionNedM.z,
        initialEuler.rollRad,
        initialEuler.pitchRad
    };

    StaticEvaluation current = evaluateStatic(
        model,
        initialGuess,
        unknowns,
        initialEuler.yawRad,
        staticControls,
        environment,
        settings_.timeStepS
    );

    GroundStaticTrimResult result;
    for (std::size_t iteration = 0U;
         iteration < settings_.maximumIterations;
         ++iteration)
    {
        result.iterations = iteration;
        if (converged(current, settings_))
        {
            result.converged = true;
            break;
        }

        const Vec3 baseResidual = trimResidualVector(current.residual);
        const std::array<double, 3> differenceSteps{
            settings_.verticalDifferenceStepM,
            settings_.attitudeDifferenceStepRad,
            settings_.attitudeDifferenceStepRad
        };
        std::array<double, 9> jacobianValues{};

        for (std::size_t column = 0U; column < 3U; ++column)
        {
            std::array<double, 3> perturbed = unknowns;
            perturbed[column] += differenceSteps[column];
            const StaticEvaluation sample = evaluateStatic(
                model,
                initialGuess,
                perturbed,
                initialEuler.yawRad,
                staticControls,
                environment,
                settings_.timeStepS
            );
            const Vec3 difference =
                (trimResidualVector(sample.residual) - baseResidual) /
                differenceSteps[column];
            jacobianValues[column] = difference.x;
            jacobianValues[3U + column] = difference.y;
            jacobianValues[6U + column] = difference.z;
        }

        const Matrix3 jacobian(jacobianValues);
        Vec3 correction = -(jacobian.inverse() * baseResidual);
        correction.x = std::clamp(
            correction.x,
            -settings_.maximumVerticalCorrectionM,
            settings_.maximumVerticalCorrectionM
        );
        correction.y = std::clamp(
            correction.y,
            -settings_.maximumAttitudeCorrectionRad,
            settings_.maximumAttitudeCorrectionRad
        );
        correction.z = std::clamp(
            correction.z,
            -settings_.maximumAttitudeCorrectionRad,
            settings_.maximumAttitudeCorrectionRad
        );

        const double currentNorm =
            scaledResidualNorm(current.residual, settings_);
        bool accepted = false;
        for (double scale = 1.0; scale >= 1.0 / 64.0; scale *= 0.5)
        {
            std::array<double, 3> candidateUnknowns = unknowns;
            candidateUnknowns[0] += scale * correction.x;
            candidateUnknowns[1] += scale * correction.y;
            candidateUnknowns[2] += scale * correction.z;
            StaticEvaluation candidate = evaluateStatic(
                model,
                initialGuess,
                candidateUnknowns,
                initialEuler.yawRad,
                staticControls,
                environment,
                settings_.timeStepS
            );
            if (candidate.model.groundContactCount >=
                    settings_.requiredContactCount &&
                scaledResidualNorm(candidate.residual, settings_) <
                    currentNorm)
            {
                unknowns = candidateUnknowns;
                current = candidate;
                accepted = true;
                break;
            }
        }
        if (!accepted)
        {
            break;
        }
        result.iterations = iteration + 1U;
    }

    if (!result.converged && converged(current, settings_))
    {
        result.converged = true;
    }
    result.state = current.state;
    result.evaluation = current.model;
    result.residual = current.residual;
    return result;
}

const GroundStaticTrimSettings&
GroundStaticTrimSolver::settings() const noexcept
{
    return settings_;
}

} // namespace navion
