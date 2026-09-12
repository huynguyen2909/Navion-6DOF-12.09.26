#include "navion/components/propeller/PropellerModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace navion::propeller
{
namespace
{

bool finite(double value) {
    return std::isfinite(value);
}

bool oppositeSignsOrZero(double lhs, double rhs) {
    return lhs == 0.0 || rhs == 0.0 || std::signbit(lhs) != std::signbit(rhs);
}

double linearInterpolate(double x, double x0, double x1, double y0, double y1) {
    if (!(x1 > x0)) {
        return y0;
    }
    const double fraction = (x - x0) / (x1 - x0);
    return y0 + fraction * (y1 - y0);
}

void validateRuntimeInput(const RuntimeInput& input)
{
    if (!input.velocityCgRelativeAirBodyMps.isFinite() ||
        !input.angularRateBodyWrtInertialBodyRadps.isFinite()) {
        throw std::invalid_argument("Runtime vectors must contain finite values.");
    }
    if (!(input.airDensityKgM3 > 0.0) || !finite(input.airDensityKgM3)) {
        throw std::invalid_argument("airDensityKgM3 must be finite and positive.");
    }
    if (!(input.speedOfSoundMps > 0.0) || !finite(input.speedOfSoundMps)) {
        throw std::invalid_argument("speedOfSoundMps must be finite and positive.");
    }
    if (!(input.rotationRateScale >= 0.0) ||
        !finite(input.rotationRateScale)) {
        throw std::invalid_argument(
            "rotationRateScale must be finite and nonnegative."
        );
    }
}

} // namespace

AirfoilPolar::AirfoilPolar(std::vector<PolarPoint> points)
    : points_(std::move(points))
{
    validate();
}

void AirfoilPolar::validate() const {
    if (points_.size() < 2) {
        throw std::invalid_argument("Airfoil polar needs at least two points.");
    }
    for (std::size_t index = 0; index < points_.size(); ++index) {
        const PolarPoint& point = points_[index];
        if (!finite(point.alphaRad) || !finite(point.cl) || !finite(point.cd) ||
            point.cd < 0.0) {
            throw std::invalid_argument("Airfoil polar contains invalid values.");
        }
        if (index > 0 && !(point.alphaRad > points_[index - 1].alphaRad)) {
            throw std::invalid_argument("Airfoil alpha grid must be strictly increasing.");
        }
    }
}

PolarLookup AirfoilPolar::lookup(double alphaRad) const {
    if (!finite(alphaRad)) {
        throw std::invalid_argument("Airfoil lookup alpha must be finite.");
    }
    if (alphaRad <= points_.front().alphaRad) {
        return {
            points_.front().cl,
            points_.front().cd,
            alphaRad < points_.front().alphaRad,
        };
    }
    if (alphaRad >= points_.back().alphaRad) {
        return {
            points_.back().cl,
            points_.back().cd,
            alphaRad > points_.back().alphaRad,
        };
    }

    const auto upper = std::upper_bound(
        points_.begin(),
        points_.end(),
        alphaRad,
        [](double value, const PolarPoint& point) {
            return value < point.alphaRad;
        }
    );
    const PolarPoint& high = *upper;
    const PolarPoint& low = *(upper - 1);
    return {
        linearInterpolate(alphaRad, low.alphaRad, high.alphaRad, low.cl, high.cl),
        linearInterpolate(alphaRad, low.alphaRad, high.alphaRad, low.cd, high.cd),
        false,
    };
}

const std::vector<PolarPoint>& AirfoilPolar::points() const noexcept
{
    return points_;
}

double PropellerParameters::diameterM() const noexcept
{
    return 2.0 * radiusM;
}

double PropellerParameters::diskAreaM2() const noexcept
{
    return pi * radiusM * radiusM;
}

double PropellerParameters::tipSpeedMps() const noexcept
{
    return rotationRateRadps * radiusM;
}

double PropellerParameters::revolutionsPerSecond() const noexcept
{
    return rotationRateRadps / (2.0 * pi);
}

double PropellerParameters::rpm() const noexcept
{
    return 60.0 * revolutionsPerSecond();
}

void PropellerParameters::validate() const {
    airfoilPolar.validate();
    if (bladeCount < 1) {
        throw std::invalid_argument("bladeCount must be positive.");
    }
    if (!(radiusM > 0.0) || !finite(radiusM)) {
        throw std::invalid_argument("radiusM must be finite and positive.");
    }
    if (!(rootCutoutFraction > 0.0 && rootCutoutFraction < 1.0) ||
        !finite(rootCutoutFraction)) {
        throw std::invalid_argument("rootCutoutFraction must lie in (0,1).");
    }
    if (!(rotationRateRadps > 0.0) || !finite(rotationRateRadps)) {
        throw std::invalid_argument("rotationRateRadps must be finite and positive.");
    }
    if (rotationSign != -1 && rotationSign != +1) {
        throw std::invalid_argument("rotationSign must equal -1 or +1.");
    }
    if (!(rotatingInertiaKgM2 >= 0.0) || !finite(rotatingInertiaKgM2)) {
        throw std::invalid_argument("rotatingInertiaKgM2 must be finite and nonnegative.");
    }
    if (!hubPositionFromCgBodyM.isFinite()) {
        throw std::invalid_argument("hubPositionFromCgBodyM must be finite.");
    }
    if (bladeGeometry.size() < 2) {
        throw std::invalid_argument("At least two blade geometry stations are required.");
    }
    for (std::size_t index = 0; index < bladeGeometry.size(); ++index) {
        const BladeStation& station = bladeGeometry[index];
        if (!(station.radiusFraction > 0.0 && station.radiusFraction <= 1.0) ||
            !(station.chordM > 0.0) || !finite(station.pitchRad)) {
            throw std::invalid_argument("Blade geometry station is invalid.");
        }
        if (index > 0 &&
            !(station.radiusFraction > bladeGeometry[index - 1].radiusFraction)) {
            throw std::invalid_argument("Blade radius grid must be strictly increasing.");
        }
    }
    if (bladeGeometry.front().radiusFraction > rootCutoutFraction ||
        bladeGeometry.back().radiusFraction < 1.0) {
        throw std::invalid_argument(
            "Blade geometry must cover rootCutoutFraction through r/R=1."
        );
    }
    if (radialElementCount < 4 || azimuthStationCount < 8) {
        throw std::invalid_argument("BEMT grid must contain at least 4 radial and 8 azimuth cells.");
    }
    if (!(compressibilityWarningMach > 0.0) ||
        !finite(compressibilityWarningMach)) {
        throw std::invalid_argument("compressibilityWarningMach must be finite and positive.");
    }

    const auto& solver = inflow;
    if (!(solver.lambdaMinimum >= 0.0) ||
        !(solver.lambdaMaximum > solver.lambdaMinimum) ||
        !finite(solver.initialGuess) ||
        !(solver.residualTolerance > 0.0) ||
        !(solver.lambdaTolerance > 0.0) ||
        !(solver.derivativeStep > 0.0) ||
        solver.maximumIterations == 0 || solver.bracketScanIntervals == 0) {
        throw std::invalid_argument("Uniform-inflow solver settings are invalid.");
    }

    // Check C_b<-p is a proper orthonormal rotation.
    const Matrix3 propellerFromBody = bodyFromPropeller.transposed();
    constexpr Vec3 basis[]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for (const Vec3& axis : basis) {
        const Vec3 roundTrip = bodyFromPropeller * (propellerFromBody * axis);
        if (!roundTrip.isFinite() || (roundTrip - axis).norm() > 1.0e-10) {
            throw std::invalid_argument("bodyFromPropeller must be orthonormal.");
        }
    }
    if (!finite(bodyFromPropeller.determinant()) ||
        std::abs(bodyFromPropeller.determinant() - 1.0) > 1.0e-10) {
        throw std::invalid_argument("bodyFromPropeller must have determinant +1.");
    }
}

const char* toString(InflowStatus status) noexcept {
    switch (status) {
    case InflowStatus::Converged:
        return "converged";
    case InflowStatus::RootNotBracketed:
        return "root_not_bracketed";
    case InflowStatus::MaximumIterations:
        return "maximum_iterations";
    }
    return "unknown";
}

PropellerModel::PropellerModel(PropellerParameters parameters)
    : parameters_(std::move(parameters)) {
    parameters_.validate();
}

const PropellerParameters& PropellerModel::parameters() const noexcept
{
    return parameters_;
}

PropellerModel::LocalGeometry PropellerModel::interpolateGeometry(
    double radiusFraction
) const {
    const auto& stations = parameters_.bladeGeometry;
    if (radiusFraction <= stations.front().radiusFraction) {
        return {stations.front().chordM, stations.front().pitchRad};
    }
    if (radiusFraction >= stations.back().radiusFraction) {
        return {stations.back().chordM, stations.back().pitchRad};
    }

    const auto upper = std::upper_bound(
        stations.begin(),
        stations.end(),
        radiusFraction,
        [](double value, const BladeStation& station) {
            return value < station.radiusFraction;
        }
    );
    const BladeStation& high = *upper;
    const BladeStation& low = *(upper - 1);
    return {
        linearInterpolate(
            radiusFraction,
            low.radiusFraction,
            high.radiusFraction,
            low.chordM,
            high.chordM
        ),
        linearInterpolate(
            radiusFraction,
            low.radiusFraction,
            high.radiusFraction,
            low.pitchRad,
            high.pitchRad
        ),
    };
}

HubKinematics PropellerModel::calculateHubKinematics(
    const RuntimeInput& input,
    double rotationRateRadps
) const {
    const Vec3 hubVelocity_body_m_s =
        input.velocityCgRelativeAirBodyMps +
        cross(
            input.angularRateBodyWrtInertialBodyRadps,
            parameters_.hubPositionFromCgBodyM
        );
    const Matrix3 propellerFromBody = parameters_.bodyFromPropeller.transposed();
    const Vec3 hubVelocity_propeller_m_s =
        propellerFromBody * hubVelocity_body_m_s;
    const Vec3 angularRate_propeller_rad_s =
        propellerFromBody * input.angularRateBodyWrtInertialBodyRadps;
    const double tipSpeedMps = rotationRateRadps * parameters_.radiusM;
    const double revolutionsPerSecond = rotationRateRadps / (2.0 * pi);

    HubKinematics result{};
    result.velocityRelativeAirPropellerMps = hubVelocity_propeller_m_s;
    result.bodyAngularRatePropellerRadps = angularRate_propeller_rad_s;
    result.lambdaFreestream = hubVelocity_propeller_m_s.x / tipSpeedMps;
    result.muY = hubVelocity_propeller_m_s.y / tipSpeedMps;
    result.muZ = hubVelocity_propeller_m_s.z / tipSpeedMps;
    result.muMagnitude = std::hypot(result.muY, result.muZ);
    result.advanceRatioJ =
        hubVelocity_propeller_m_s.x /
        (revolutionsPerSecond * parameters_.diameterM());
    return result;
}

DiskEvaluation PropellerModel::evaluateDiskAtInflow(
    const RuntimeInput& input,
    double lambdaInduced,
    bool captureElementSamples
) const {
    validateRuntimeInput(input);
    if (!finite(lambdaInduced) || lambdaInduced < 0.0) {
        throw std::invalid_argument("lambdaInduced must be finite and nonnegative in V1.");
    }
    const double rotationRateRadps =
        parameters_.rotationRateRadps * input.rotationRateScale;
    if (!(rotationRateRadps > 1.0e-12)) {
        DiskEvaluation stopped{};
        stopped.loads.minimumAlphaRad = 0.0;
        stopped.loads.maximumAlphaRad = 0.0;
        return stopped;
    }
    const HubKinematics kinematics = calculateHubKinematics(
        input,
        rotationRateRadps
    );
    return evaluateDiskWithKinematics(
        input,
        kinematics,
        rotationRateRadps,
        lambdaInduced,
        captureElementSamples
    );
}

DiskEvaluation PropellerModel::evaluateDiskWithKinematics(
    const RuntimeInput& input,
    const HubKinematics& kinematics,
    double rotationRateRadps,
    double lambdaInduced,
    bool captureElementSamples
) const {
    DiskEvaluation evaluation{};
    evaluation.kinematics = kinematics;
    DiskLoads& loads = evaluation.loads;
    loads.minimumAlphaRad = std::numeric_limits<double>::infinity();
    loads.maximumAlphaRad = -std::numeric_limits<double>::infinity();

    if (!input.enabled) {
        loads.minimumAlphaRad = 0.0;
        loads.maximumAlphaRad = 0.0;
        return evaluation;
    }

    if (captureElementSamples) {
        evaluation.elementSamples.reserve(
            parameters_.radialElementCount * parameters_.azimuthStationCount
        );
    }

    constexpr Vec3 shaftAxis_propeller{1.0, 0.0, 0.0};
    const double radiusM = parameters_.radiusM;
    const double tipSpeedMps = rotationRateRadps * parameters_.radiusM;
    const double inducedVelocityMps = lambdaInduced * tipSpeedMps;
    const double radialStart_m = parameters_.rootCutoutFraction * radiusM;
    const double dr_m =
        (radiusM - radialStart_m) /
        static_cast<double>(parameters_.radialElementCount);

    // B/(2*pi) integral(dpsi) becomes B/Npsi for a midpoint azimuth rule.
    const double diskAverageWeight =
        static_cast<double>(parameters_.bladeCount) /
        static_cast<double>(parameters_.azimuthStationCount);

    for (std::size_t azimuthIndex = 0;
         azimuthIndex < parameters_.azimuthStationCount;
         ++azimuthIndex) {
        const double azimuthRad =
            2.0 * pi * (static_cast<double>(azimuthIndex) + 0.5) /
            static_cast<double>(parameters_.azimuthStationCount);

        // psi=0: blade points toward +y_p. e_t follows physical rotation.
        const Vec3 radialUnit_propeller{
            0.0,
            std::cos(azimuthRad),
            std::sin(azimuthRad),
        };
        const Vec3 tangentUnit_propeller =
            static_cast<double>(parameters_.rotationSign) *
            cross(shaftAxis_propeller, radialUnit_propeller);

        for (std::size_t radialIndex = 0;
             radialIndex < parameters_.radialElementCount;
             ++radialIndex) {
            const double radialPosition_m =
                radialStart_m + (static_cast<double>(radialIndex) + 0.5) * dr_m;
            const double radiusFraction = radialPosition_m / radiusM;
            const LocalGeometry geometry = interpolateGeometry(radiusFraction);
            const Vec3 elementPosition_propeller_m =
                radialPosition_m * radialUnit_propeller;

            // Material point velocity relative to local air. Positive induced
            // inflow represents air moving aft (-x_p), so it increases the
            // material-relative-air component along +x_p.
            const Vec3 elementVelocityRelativeAir_propeller_m_s =
                kinematics.velocityRelativeAirPropellerMps +
                cross(
                    kinematics.bodyAngularRatePropellerRadps,
                    elementPosition_propeller_m
                ) +
                rotationRateRadps * radialPosition_m *
                    tangentUnit_propeller +
                inducedVelocityMps * shaftAxis_propeller;

            // V1 is a 2-D section model; the radial/spanwise component is not
            // used by the airfoil polar.
            const double axialVelocityMps = dot(
                elementVelocityRelativeAir_propeller_m_s,
                shaftAxis_propeller
            );
            const double tangentialVelocityMps = dot(
                elementVelocityRelativeAir_propeller_m_s,
                tangentUnit_propeller
            );
            if (tangentialVelocityMps <= 0.0) {
                ++loads.reverseTangentialFlowCount;
            }

            const double speedSquared_m2_s2 =
                axialVelocityMps * axialVelocityMps +
                tangentialVelocityMps * tangentialVelocityMps;
            if (speedSquared_m2_s2 <= 1.0e-16) {
                continue;
            }

            const double sectionSpeed_m_s = std::sqrt(speedSquared_m2_s2);
            const double inflowAngleRad = std::atan2(
                axialVelocityMps,
                tangentialVelocityMps
            );
            const double angleOfAttackRad = geometry.pitchRad - inflowAngleRad;
            const PolarLookup coefficients =
                parameters_.airfoilPolar.lookup(angleOfAttackRad);

            const double dynamicPressure_Pa =
                0.5 * input.airDensityKgM3 * speedSquared_m2_s2;
            const double liftPerSpan_N_m =
                dynamicPressure_Pa * geometry.chordM * coefficients.cl;
            const double dragPerSpan_N_m =
                dynamicPressure_Pa * geometry.chordM * coefficients.cd;

            // Lift is normal to the local 2-D relative velocity and positive
            // toward +x_p for positive alpha. Drag opposes relative motion.
            const double axialForcePerSpan_N_m =
                liftPerSpan_N_m * std::cos(inflowAngleRad) -
                dragPerSpan_N_m * std::sin(inflowAngleRad);
            const double tangentForcePerSpan_N_m =
                -liftPerSpan_N_m * std::sin(inflowAngleRad) -
                dragPerSpan_N_m * std::cos(inflowAngleRad);
            const Vec3 forcePerSpan_propeller_N_m =
                axialForcePerSpan_N_m * shaftAxis_propeller +
                tangentForcePerSpan_N_m * tangentUnit_propeller;

            const Vec3 differentialMeanForcePropellerN =
                forcePerSpan_propeller_N_m * (dr_m * diskAverageWeight);
            const Vec3 differentialMeanMomentPropellerNm =
                cross(
                    elementPosition_propeller_m,
                    differentialMeanForcePropellerN
                );

            loads.forcePropellerN += differentialMeanForcePropellerN;
            loads.aerodynamicMomentAtHubPropellerNm +=
                differentialMeanMomentPropellerNm;
            loads.minimumAlphaRad = std::min(
                loads.minimumAlphaRad,
                angleOfAttackRad
            );
            loads.maximumAlphaRad = std::max(
                loads.maximumAlphaRad,
                angleOfAttackRad
            );
            loads.maximumSectionMach = std::max(
                loads.maximumSectionMach,
                sectionSpeed_m_s / input.speedOfSoundMps
            );
            loads.polarClampCount += coefficients.clamped ? 1U : 0U;
            ++loads.elementCount;

            if (captureElementSamples) {
                evaluation.elementSamples.push_back({
                    azimuthRad,
                    radialPosition_m,
                    radiusFraction,
                    geometry.chordM,
                    geometry.pitchRad,
                    axialVelocityMps,
                    tangentialVelocityMps,
                    inflowAngleRad,
                    angleOfAttackRad,
                    coefficients.cl,
                    coefficients.cd,
                    differentialMeanForcePropellerN,
                    differentialMeanMomentPropellerNm,
                    coefficients.clamped,
                });
            }
        }
    }

    if (!finite(loads.minimumAlphaRad)) {
        loads.minimumAlphaRad = 0.0;
        loads.maximumAlphaRad = 0.0;
    }

    loads.thrustN = loads.forcePropellerN.x;
    loads.torqueRequiredNm =
        -static_cast<double>(parameters_.rotationSign) *
        loads.aerodynamicMomentAtHubPropellerNm.x;
    loads.shaftPowerRequiredW =
        loads.torqueRequiredNm * rotationRateRadps;

    const double rotorForceReference_N =
        input.airDensityKgM3 * parameters_.diskAreaM2() *
        tipSpeedMps * tipSpeedMps;
    const double propellerForceReference_N =
        input.airDensityKgM3 *
        std::pow(rotationRateRadps / (2.0 * pi), 2) *
        std::pow(parameters_.diameterM(), 4);
    const double propellerTorqueReference_Nm =
        input.airDensityKgM3 *
        std::pow(rotationRateRadps / (2.0 * pi), 2) *
        std::pow(parameters_.diameterM(), 5);

    loads.thrustCoefficientRotor = loads.thrustN / rotorForceReference_N;
    loads.thrustCoefficientPropeller = loads.thrustN / propellerForceReference_N;
    loads.torqueCoefficientPropeller =
        loads.torqueRequiredNm / propellerTorqueReference_Nm;
    loads.compressibilityWarning =
        loads.maximumSectionMach >= parameters_.compressibilityWarningMach;
    return evaluation;
}

PropellerModel::ResidualEvaluation PropellerModel::evaluateResidual(
    const RuntimeInput& input,
    const HubKinematics& kinematics,
    double rotationRateRadps,
    double lambdaInduced
) const {
    DiskEvaluation disk = evaluateDiskWithKinematics(
        input,
        kinematics,
        rotationRateRadps,
        lambdaInduced,
        false
    );
    const double totalAxialInflow =
        kinematics.lambdaFreestream + lambdaInduced;
    const double momentumThrustCoefficient =
        2.0 * lambdaInduced *
        std::hypot(kinematics.muMagnitude, totalAxialInflow);
    return {
        lambdaInduced,
        disk.loads.thrustCoefficientRotor - momentumThrustCoefficient,
        momentumThrustCoefficient,
        std::move(disk),
    };
}

PropellerOutput PropellerModel::evaluate(
    const RuntimeInput& input,
    double previousLambdaInduced
) const {
    validateRuntimeInput(input);
    if (!finite(previousLambdaInduced)) {
        throw std::invalid_argument("previousLambdaInduced must be finite.");
    }

    PropellerOutput output{};
    const double rotationRateRadps =
        parameters_.rotationRateRadps * input.rotationRateScale;
    output.rotationRateRadps = rotationRateRadps;
    output.rpm = 60.0 * rotationRateRadps / (2.0 * pi);
    if (!input.enabled || !(rotationRateRadps > 1.0e-12)) {
        output.inflow.status = InflowStatus::Converged;
        output.inflow.converged = true;
        output.inflow.rootBracketed = true;
        return output;
    }
    output.kinematics = calculateHubKinematics(input, rotationRateRadps);

    const InflowSolverSettings& settings = parameters_.inflow;
    std::size_t evaluationCount = 0;
    const auto residualAt = [&](double lambda) {
        ++evaluationCount;
        return evaluateResidual(
            input,
            output.kinematics,
            rotationRateRadps,
            lambda
        );
    };

    double lowerLambda = settings.lambdaMinimum;
    double upperLambda = settings.lambdaMaximum;
    ResidualEvaluation lower = residualAt(lowerLambda);
    ResidualEvaluation upper = residualAt(upperLambda);

    ResidualEvaluation best =
        std::abs(lower.residual) <= std::abs(upper.residual) ? lower : upper;
    bool bracketed = oppositeSignsOrZero(lower.residual, upper.residual);

    // A scan handles a non-monotone BEMT residual and selects the first
    // positive-inflow root found within the configured powered-flight interval.
    if (!bracketed) {
        ResidualEvaluation previous = lower;
        double previousLambda = lowerLambda;
        for (std::size_t index = 1;
             index <= settings.bracketScanIntervals;
             ++index) {
            const double fraction =
                static_cast<double>(index) /
                static_cast<double>(settings.bracketScanIntervals);
            const double candidateLambda =
                settings.lambdaMinimum + fraction *
                (settings.lambdaMaximum - settings.lambdaMinimum);
            ResidualEvaluation candidate = residualAt(candidateLambda);
            if (std::abs(candidate.residual) < std::abs(best.residual)) {
                best = candidate;
            }
            if (oppositeSignsOrZero(previous.residual, candidate.residual)) {
                lowerLambda = previousLambda;
                lower = previous;
                upperLambda = candidateLambda;
                upper = candidate;
                bracketed = true;
                break;
            }
            previousLambda = candidateLambda;
            previous = std::move(candidate);
        }
    }

    ResidualEvaluation current = best;
    InflowStatus status = InflowStatus::RootNotBracketed;
    std::size_t iterationCount = 0;

    if (bracketed) {
        if (std::abs(lower.residual) <= settings.residualTolerance) {
            current = lower;
            status = InflowStatus::Converged;
        } else if (std::abs(upper.residual) <= settings.residualTolerance) {
            current = upper;
            status = InflowStatus::Converged;
        } else {
            double lambda = std::clamp(
                previousLambdaInduced,
                lowerLambda,
                upperLambda
            );
            if (!(lambda > lowerLambda && lambda < upperLambda)) {
                lambda = 0.5 * (lowerLambda + upperLambda);
            }
            current = residualAt(lambda);

            for (std::size_t iteration = 1;
                 iteration <= settings.maximumIterations;
                 ++iteration) {
                iterationCount = iteration;
                if (std::abs(current.residual) <= settings.residualTolerance) {
                    status = InflowStatus::Converged;
                    break;
                }

                if (oppositeSignsOrZero(lower.residual, current.residual)) {
                    upperLambda = current.lambdaInduced;
                    upper = current;
                } else {
                    lowerLambda = current.lambdaInduced;
                    lower = current;
                }

                const double h = settings.derivativeStep *
                    std::max(1.0, std::abs(current.lambdaInduced));
                const double derivativeLowLambda = std::max(
                    settings.lambdaMinimum,
                    current.lambdaInduced - h
                );
                const double derivativeHighLambda = std::min(
                    settings.lambdaMaximum,
                    current.lambdaInduced + h
                );

                double derivative = std::numeric_limits<double>::quiet_NaN();
                if (derivativeHighLambda > derivativeLowLambda) {
                    const ResidualEvaluation derivativeLow =
                        residualAt(derivativeLowLambda);
                    const ResidualEvaluation derivativeHigh =
                        residualAt(derivativeHighLambda);
                    derivative =
                        (derivativeHigh.residual - derivativeLow.residual) /
                        (derivativeHighLambda - derivativeLowLambda);
                }

                double candidateLambda = std::numeric_limits<double>::quiet_NaN();
                if (finite(derivative) && std::abs(derivative) > 1.0e-12) {
                    candidateLambda =
                        current.lambdaInduced - current.residual / derivative;
                }
                if (!finite(candidateLambda) ||
                    !(candidateLambda > lowerLambda && candidateLambda < upperLambda)) {
                    candidateLambda = 0.5 * (lowerLambda + upperLambda);
                }

                const double lambdaChange =
                    std::abs(candidateLambda - current.lambdaInduced);
                current = residualAt(candidateLambda);
                if (std::abs(current.residual) <= settings.residualTolerance ||
                    (lambdaChange <= settings.lambdaTolerance &&
                     std::abs(current.residual) <= 10.0 * settings.residualTolerance)) {
                    status = InflowStatus::Converged;
                    break;
                }
            }
            if (status != InflowStatus::Converged) {
                status = InflowStatus::MaximumIterations;
            }
        }
    }

    output.disk = std::move(current.disk.loads);
    output.kinematics = current.disk.kinematics;
    output.inflow.status = status;
    output.inflow.converged = status == InflowStatus::Converged;
    output.inflow.rootBracketed = bracketed;
    output.inflow.iterations = iterationCount;
    output.inflow.residualEvaluations = evaluationCount;
    output.inflow.lambdaInduced = current.lambdaInduced;
    output.inflow.inducedVelocityMps =
        current.lambdaInduced * rotationRateRadps * parameters_.radiusM;
    output.inflow.momentumResidual = current.residual;
    output.inflow.momentumThrustCoefficient =
        current.momentumThrustCoefficient;

    output.forceBodyN =
        parameters_.bodyFromPropeller * output.disk.forcePropellerN;
    output.aerodynamicMomentAtHubBodyNm =
        parameters_.bodyFromPropeller *
        output.disk.aerodynamicMomentAtHubPropellerNm;

    const Vec3 shaftAxis_body =
        parameters_.bodyFromPropeller * Vec3{1.0, 0.0, 0.0};
    output.reactionTorqueBodyNm =
        dot(output.aerodynamicMomentAtHubBodyNm, shaftAxis_body) *
        shaftAxis_body;
    output.hubBendingMomentBodyNm =
        output.aerodynamicMomentAtHubBodyNm -
        output.reactionTorqueBodyNm;
    output.momentArmBodyNm =
        cross(parameters_.hubPositionFromCgBodyM, output.forceBodyN);

    const Vec3 propellerAngularMomentum_body_Nm_s =
        static_cast<double>(parameters_.rotationSign) *
        parameters_.rotatingInertiaKgM2 *
        rotationRateRadps *
        shaftAxis_body;
    output.gyroscopicMomentBodyNm = -cross(
        input.angularRateBodyWrtInertialBodyRadps,
        propellerAngularMomentum_body_Nm_s
    );
    output.totalMomentAtCgBodyNm =
        output.aerodynamicMomentAtHubBodyNm +
        output.momentArmBodyNm +
        output.gyroscopicMomentBodyNm;
    return output;
}

PropellerOutput PropellerModel::evaluate(const RuntimeInput& input) const
{
    return evaluate(input, parameters_.inflow.initialGuess);
}

PropellerComponent::PropellerComponent(PropellerModel model)
    : model_(std::move(model))
{
}

PropellerComponent::PropellerComponent(PropellerParameters parameters)
    : model_(PropellerModel(std::move(parameters)))
{
}

BodyLoad PropellerComponent::computeLoad(
    const EvaluationContext& context
) const
{
    const PropellerOutput output = evaluateDetailed(context);
    if (context.controls.propellerEnabled && !output.inflow.converged)
    {
        throw std::runtime_error(
            std::string("Propeller inflow solver failed: ") +
            toString(output.inflow.status)
        );
    }

    const BodyLoad load{output.forceBodyN, output.totalMomentAtCgBodyNm};
    if (!load.isFinite())
    {
        throw std::runtime_error("Propeller produced a non-finite BodyLoad.");
    }
    return load;
}

std::string_view PropellerComponent::name() const noexcept
{
    return "Propeller";
}

PropellerOutput PropellerComponent::evaluateDetailed(
    const EvaluationContext& context
) const
{
    return model_.evaluate(makeRuntimeInput(context));
}

const PropellerModel& PropellerComponent::model() const noexcept
{
    return model_;
}

RuntimeInput PropellerComponent::makeRuntimeInput(
    const EvaluationContext& context
)
{
    if (!context.flightCondition.isFinite() ||
        !context.state.angularRateBodyRadps.isFinite() ||
        !std::isfinite(context.environment.airDensityKgM3) ||
        !std::isfinite(context.environment.speedOfSoundMps))
    {
        throw std::invalid_argument(
            "PropellerComponent received an invalid EvaluationContext."
        );
    }

    RuntimeInput input;
    input.velocityCgRelativeAirBodyMps =
        context.flightCondition.airRelativeVelocityBodyMps;
    input.angularRateBodyWrtInertialBodyRadps =
        context.state.angularRateBodyRadps;
    input.airDensityKgM3 = context.environment.airDensityKgM3;
    input.speedOfSoundMps = context.environment.speedOfSoundMps;
    input.rotationRateScale = context.controls.propellerSpeedScale;
    input.enabled = context.controls.propellerEnabled;
    return input;
}

PropellerParameters makeEstimatedNavionNaca5868_9Parameters() {
    PropellerParameters parameters{};
    parameters.propellerName =
        "Estimated NACA 5868-9-like Navion V1 propeller";
    parameters.airfoilName = "Estimated Clark-Y alpha-only polar";
    parameters.bladeCount = 2;
    parameters.radiusM = 1.0668; // 84 in diameter; provisional Navion variant
    parameters.rootCutoutFraction = 0.20;
    parameters.rotationRateRadps =
        2300.0 * 2.0 * pi / 60.0; // provisional configured design RPM
    parameters.rotationSign = +1;
    parameters.rotatingInertiaKgM2 = 5.5; // provisional rotating assembly
    parameters.hubPositionFromCgBodyM = {1.65, 0.0, -0.08};
    parameters.bodyFromPropeller = Matrix3::identity();

    // Provisional NACA 5868-9-like planform. Pitch is an estimated constant-
    // geometric-pitch helix referenced at r=30 in on an R=42 in propeller:
    // x_ref=30/42=0.7142857. It is intentionally not mislabeled theta_75.
    constexpr double pitchReferenceRadiusFraction = 30.0 / 42.0;
    constexpr double pitchAtReference_rad = 0.27;
    const std::vector<std::pair<double, double>> planform{
        {0.20, 0.090},
        {0.25, 0.120},
        {0.30, 0.145},
        {0.40, 0.175},
        {0.50, 0.190},
        {0.60, 0.185},
        {0.70, 0.170},
        {0.80, 0.145},
        {0.90, 0.105},
        {0.96, 0.065},
        {1.00, 0.025},
    };
    parameters.bladeGeometry.reserve(planform.size());
    for (const auto& [radiusFraction, chordM] : planform) {
        const double pitchRad = std::atan(
            pitchReferenceRadiusFraction * std::tan(pitchAtReference_rad) /
            radiusFraction
        );
        parameters.bladeGeometry.push_back({
            radiusFraction,
            chordM,
            pitchRad,
        });
    }

    // Hand-built placeholder only. These are not measured Clark-Y data and do
    // not contain Reynolds or Mach dependence. Wide alpha coverage avoids
    // silently extrapolating the static inner-blade operating points.
    const std::vector<PolarPoint> estimatedPolar{
        {degreesToRadians(-90.0),  0.00, 1.90},
        {degreesToRadians(-75.0), -0.30, 1.65},
        {degreesToRadians(-60.0), -0.55, 1.20},
        {degreesToRadians(-45.0), -0.80, 0.75},
        {degreesToRadians(-30.0), -0.95, 0.40},
        {degreesToRadians(-20.0), -0.90, 0.22},
        {degreesToRadians(-15.0), -0.78, 0.13},
        {degreesToRadians(-10.0), -0.52, 0.065},
        {degreesToRadians( -5.0), -0.08, 0.026},
        {degreesToRadians(  0.0),  0.42, 0.014},
        {degreesToRadians(  5.0),  0.92, 0.019},
        {degreesToRadians( 10.0),  1.32, 0.038},
        {degreesToRadians( 12.0),  1.42, 0.052},
        {degreesToRadians( 15.0),  1.38, 0.090},
        {degreesToRadians( 20.0),  1.20, 0.17},
        {degreesToRadians( 30.0),  0.98, 0.36},
        {degreesToRadians( 45.0),  0.78, 0.72},
        {degreesToRadians( 60.0),  0.55, 1.18},
        {degreesToRadians( 75.0),  0.30, 1.62},
        {degreesToRadians( 90.0),  0.00, 1.90},
    };
    parameters.airfoilPolar = AirfoilPolar(estimatedPolar);

    parameters.radialElementCount = 64;
    parameters.azimuthStationCount = 96;
    parameters.compressibilityWarningMach = 0.70;
    parameters.inflow.lambdaMinimum = 0.0;
    parameters.inflow.lambdaMaximum = 0.50;
    parameters.inflow.initialGuess = 0.08;
    parameters.inflow.residualTolerance = 1.0e-9;
    parameters.inflow.lambdaTolerance = 1.0e-10;
    parameters.inflow.derivativeStep = 1.0e-5;
    parameters.inflow.maximumIterations = 50;
    parameters.inflow.bracketScanIntervals = 100;
    return parameters;
}

} // namespace navion::propeller
