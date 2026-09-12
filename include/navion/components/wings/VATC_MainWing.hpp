#pragma once

// VATC-style OOP refactor of the original standalone Navion wing calculator.
//
// Design rule:
//   - VATC_MainWingConfig retains EVERY original Inputs variable, name, default
//     value and lookup table from navion_model_direct_plots(5).cpp.
//   - EvaluationContext supplies the live simulation state/environment/controls,
//     consistent with VATC_HorizontalStabilizer.
//   - WorkingData contains transient/derived quantities.
//   - computeLoad() returns the standard VATC BodyLoad.

#include "navion/components/ILoadComponent.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace navion
{

struct VATC_LookupTable
{
    std::vector<double> x;
    std::vector<double> values;
    std::string label;
};

struct VATC_MainWingConfig
{
    // =====================================================================
    // EXACT ORIGINAL INPUT BLOCKS / NAMES / DEFAULT VALUES
    // =====================================================================

    struct ModelInputs
    {
        // Data provenance label only; changing it does not isolate the wing.
        std::string scope{"provisional_aircraft_proxy"};

        // "body" or "stability": axes of supplied lateral derivatives.
        std::string derivative_axes{"body"};

        // "zero_referenced" or "literal_roskam".
        std::string flap_moment_mode{"zero_referenced"};
    } model;

    struct FlightInputs
    {
        // rho: air density [kg/m^3]
        // ISA estimate at 1525 m, NOT a measured NASA value.
        double rho_kg_m3{1.0556};

        // V: true airspeed [m/s], NASA Table II condition II.
        double speed_m_s{43.9};

        // alpha: body angle of attack [deg]
        double alpha_deg{0.0};

        // beta: sideslip [deg]
        double beta_deg{0.0};

        // p, q, r: body angular rates [deg/s]
        double p_deg_s{0.0};
        double q_deg_s{0.0};
        double r_deg_s{0.0};
    } flight;

    struct ControlsInputs
    {
        // delta_a: aileron deflection [deg]
        double aileron_deg{0.0};

        // delta_f: symmetric plain-flap deflection [deg]
        double flap_deg{20.0};
    } controls;

    struct WingInputs
    {
        // S: reference wing area [m^2]
        double area_m2{17.112};

        // A or AR = b^2/S [-]
        double aspect_ratio{6.04};

        // cbar: mean aerodynamic chord [m]
        double mean_chord_m{1.74};

        // lambda = c_tip/c_root [-]
        double taper{0.54};

        // Lambda_LE: leading-edge sweep [deg]
        double sweep_le_deg{2.996};

        // i_w: wing incidence [deg]
        double incidence_deg{2.0};
    } wing;

    struct ReferenceInputs
    {
        // h_ref = x_ref/cbar: aerodynamic moment origin, positive aft [-]
        double aero_h{0.25};

        // z_ref: aerodynamic origin, BODY z positive down [m]
        double aero_z_m{0.0};

        // h_output: output/CG moment origin, positive aft [-]
        double output_h{0.25};

        // z_output: output/CG origin, BODY z positive down [m]
        double output_z_m{0.0};

        // alpha_ref: fixed stability-to-body axes angle [deg]
        double stability_angle_deg{0.0};
    } reference;

    struct AeroInputs
    {
        // c_l_alpha: section (2D) lift slope [1/rad]
        double cl_alpha_per_rad{6.0};

        // C_L_alpha: finite-wing lift slope [1/rad]
        double CL_alpha_per_rad{5.23};

        // C_L0: clean lift intercept [-]
        double CL0{0.20};

        // C_D0: clean zero-lift drag coefficient [-]
        double CD0{0.030};

        // e: Oswald efficiency [-]
        double oswald_e{0.80};

        // C_m0: clean pitch intercept at aero_h [-]
        double Cm0{0.0};

        // C_m_alpha: clean pitch slope at aero_h [1/rad]
        double Cm_alpha_per_rad{-1.26};
    } aero;

    struct FlapInputs
    {
        // eta_i = 2|y_i|/b: inboard flap semispan fraction [-]
        double eta_in{0.13};

        // eta_o = 2|y_o|/b: outboard flap semispan fraction [-]
        double eta_out{0.65};

        // cf/c: flap/retracted local chord ratio [-]
        double chord_ratio{0.21};

        // t/c: representative thickness ratio [-]
        double thickness_ratio{0.10};

        // Effective section flap lift derivative [1/rad]
        double section_effectiveness_per_rad{3.65};

        // R_actual: finite-wing / section effectiveness ratio [-]
        double effectiveness_ratio_actual{1.10};

        // R_ref: same ratio for Roskam reference wing [-]
        double effectiveness_ratio_reference{1.10};

        // K: extra flap induced-drag factor [-]
        double induced_factor_K{0.25};

        // K_int: profile-drag interference multiplier [-]
        double interference_factor{0.0};
    } flap;

    struct TablesInputs
    {
        VATC_LookupTable kprime{
            {0.0, 10.0, 15.0, 20.0, 30.0, 40.0},
            {1.0, 1.0, 0.94, 0.88, 0.68, 0.59},
            "kprime"
        };

        VATC_LookupTable profile_drag{
            {0.0, 15.0, 60.0},
            {0.0, 0.014, 0.15},
            "profile_drag"
        };

        VATC_LookupTable Kb_cumulative{
            {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
            {0.0, 0.28, 0.53, 0.74, 0.91, 1.0},
            "Kb_cumulative"
        };

        VATC_LookupTable Kp_cumulative{
            {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
            {0.0, 0.310, 0.584, 0.784, 0.929, 1.055},
            "Kp_cumulative"
        };

        VATC_LookupTable Klambda_cumulative{
            {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
            {0.0, 0.044, 0.060, 0.055, 0.032, 0.0},
            "Klambda_cumulative"
        };

        VATC_LookupTable moment_ratio{
            {0.20, 0.25, 0.30, 0.35, 0.40},
            {-0.308, -0.282, -0.259, -0.239, -0.220},
            "moment_ratio"
        };
    } tables;

    struct LateralInputs
    {
        // d(CY)/d(beta), d(CY)/d(p_hat), d(CY)/d(r_hat), d(CY)/d(delta_a)
        double CY_beta{-0.35};
        double CY_p{-0.30};
        double CY_r{0.50};
        double CY_da{0.0};

        // d(Cl)/d(beta), d(Cl)/d(p_hat), d(Cl)/d(r_hat), d(Cl)/d(delta_a)
        double Cl_beta{-0.06};
        double Cl_p{-0.45};
        double Cl_r{0.04};
        double Cl_da{0.15};

        // d(Cn)/d(beta), d(Cn)/d(p_hat), d(Cn)/d(r_hat), d(Cn)/d(delta_a)
        double Cn_beta{0.049};
        double Cn_p{-0.024};
        double Cn_r{-0.082};
        double Cn_da{-0.005};
    } lateral;
};

struct MainWingEvaluation
{
    BodyLoad bodyLoad{};

    Vec3 forceBodyN{};
    Vec3 intrinsicMomentAtAerodynamicReferenceBodyNm{};
    Vec3 momentAboutCgBodyNm{};

    double airDensityKgM3{0.0};
    double airspeedMps{0.0};
    double angleOfAttackRad{0.0};
    double sideSlipRad{0.0};
    double rollRateRadps{0.0};
    double pitchRateRadps{0.0};
    double yawRateRadps{0.0};
    double aileronDeflectionRad{0.0};
    double flapDeflectionRad{0.0};
    double flapDeflectionDeg{0.0};
    double dynamicPressurePa{0.0};

    double wingSpanM{0.0};
    double wingAngleOfAttackRad{0.0};
    double quarterChordSweepAngleRad{0.0};
    double flappedWingAreaM2{0.0};
    double projectedChordRatio{0.0};
    double flapChordToProjectedChordRatio{0.0};
    double thicknessToProjectedChordRatio{0.0};
    double flapNonlinearityFactor{0.0};
    double profileDragTableValue{0.0};
    double liftSpanFactor{0.0};
    double pitchSpanFactor{0.0};
    double sweepPitchFactor{0.0};
    double pitchMomentRatio{0.0};
    double sectionSlopeParameter{0.0};
    double referenceLiftCurveSlope{0.0};

    double sectionFlapLiftCoefficientIncrement{0.0};
    double flapLiftCoefficientIncrement{0.0};
    double referenceFlapLiftCoefficientIncrement{0.0};
    double cleanLiftCoefficient{0.0};
    double liftCoefficient{0.0};
    double cleanPitchMomentCoefficient{0.0};
    double dragCoefficient{0.0};
    double profileDragCoefficientIncrement{0.0};
    double flapInducedDragCoefficientIncrement{0.0};
    double interferenceDragCoefficientIncrement{0.0};
    double flapDragCoefficientIncrement{0.0};

    double pitchMomentTerm1{0.0};
    double pitchMomentTerm2{0.0};
    double pitchMomentTerm3{0.0};
    double pitchMomentTerm4{0.0};
    double pitchMomentTerm5{0.0};
    double literalFlapPitchMomentCoefficientIncrement{0.0};
    double zeroFlapPitchMomentBaseline{0.0};
    double flapPitchMomentCoefficientIncrement{0.0};
    double pitchMomentCoefficientAtAerodynamicReference{0.0};

    double derivativeFrameRollRateRadps{0.0};
    double derivativeFrameYawRateRadps{0.0};
    double normalizedRollRate{0.0};
    double normalizedPitchRate{0.0};
    double normalizedYawRate{0.0};

    double axialForceCoefficient{0.0};
    double sideForceCoefficient{0.0};
    double normalForceCoefficient{0.0};
    double rollMomentCoefficientAtAerodynamicReference{0.0};
    double yawMomentCoefficientAtAerodynamicReference{0.0};
    double rollMomentCoefficient{0.0};
    double pitchMomentCoefficient{0.0};
    double yawMomentCoefficient{0.0};
    double upwardNormalForceCoefficient{0.0};

    double momentTransferDxM{0.0};
    double momentTransferDzM{0.0};

    std::vector<std::string> warnings;
};

class VATC_MainWing final : public ILoadComponent
{
public:
    explicit VATC_MainWing(const VATC_MainWingConfig& config);

    BodyLoad computeLoad(const EvaluationContext& context) const override;
    std::string_view name() const noexcept override;

    MainWingEvaluation evaluateDetailed(const EvaluationContext& context) const;

    const VATC_MainWingConfig& getConfig() const noexcept;

private:
    struct WorkingData;

    void readRuntimeInputs(
        const EvaluationContext& context,
        WorkingData& data
    ) const;

    void calculateGeometry(WorkingData& data) const;
    void calculateFlapLookupData(WorkingData& data) const;
    void calculateLongitudinalAerodynamics(WorkingData& data) const;
    void calculateLateralAerodynamics(WorkingData& data) const;
    void calculateBodyAxisCoefficients(WorkingData& data) const;
    void calculateDimensionalLoads(WorkingData& data) const;

    VATC_MainWingConfig config_;
};

// Common-model aliases. The VATC names remain the canonical source-compatible
// API so no uploaded MainWing feature or identifier is removed.
using MainWingConfig = VATC_MainWingConfig;
using MainWing = VATC_MainWing;

} // namespace navion
