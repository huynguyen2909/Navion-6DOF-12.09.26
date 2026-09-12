#pragma once
// t6c_fuselage.hpp
//
// T-6C Texan II fuselage aerodynamic coefficient model. Given fuselage
// geometry and the current flow/attitude state, FuselageModel::Update()
// returns the fuselage's parasite drag, pitching moment, and yawing moment
// coefficients (and the corresponding forces/moments) -- nothing else.
//
// Method: F. Nicolosi, P. Della Vecchia, D. Ciliberti, V. Cusati, "Fuselage
// aerodynamic prediction methods," Aerospace Science and Technology, 55
// (2016) 332-343. See docs/model_formulas.pdf for every formula below, in
// the exact order they are evaluated, with full derivations and citations.
//
// Two different formula families are used, deliberately, because they have
// different amounts of real evidence behind them (see docs/model_formulas.pdf
// intro for the full reasoning):
//   - Drag: the paper's own Kn/Kc/Kt shape factors have ZERO printed value
//     anywhere in the paper (chart-only, digitized from CFD this project
//     does not have), so drag instead uses the closed-form Roskam/Kroo
//     formulas the paper itself cites as its comparison baseline (Eq.6,7,9)
//     -- real, validated, no fabricated constant.
//   - Pitching/yawing moment (CM0, CMalpha, CNbeta): each has exactly ONE
//     real anchor point -- the paper's reference-fuselage worked example is
//     printed as actual numbers, not a chart reading (Eq.13/14/16). No
//     better-validated closed-form alternative exists (see project README),
//     so each curve here is a line through that one real point, with an
//     explicitly UNCALIBRATED trend-guessed slope -- see
//     t6c_fuselage_params.yaml and docs/model_formulas.pdf Sec.11
//     ("Calibration workflow") for how to replace the slopes with real
//     numbers once CFD on an actual T-6C fuselage geometry exists.
//
// This model is stateless: FuselageModel caches only geometry-derived
// constants (frontal/wetted areas, fineness ratios) computed once at
// construction. Update() is a pure function of the AeroState passed in --
// unlike t6c_landing_gear, there is no per-step history.
//
// Only dependency: <cmath>. No Eigen -- everything here is a scalar
// coefficient/force/moment, not a body-axis vector; direction/frame
// resolution is deliberately left to the caller (see docs/model_formulas.pdf
// Sec.10, "Direction, deliberately left to the caller").
//
// Units: SI throughout -- metres, newtons, kilograms, radians, seconds.

#include <string>

namespace t6c_fuselage {

// Fixed fuselage shape (SI units: metres, radians). See
// t6c_fuselage_params.yaml for the T-6C's actual numbers and which fields
// are VERIFIED vs ESTIMATED.
struct Geometry {
  double Lf_m = 0.0;   // total fuselage length
  double df_m = 0.0;   // max diameter (equivalent, frontal-area basis)
  double Ln_m = 0.0;   // nose length
  double Lc_m = 0.0;   // cabin length
  double Lt_m = 0.0;   // tailcone length
  double db_m = 0.0;   // tailcone base diameter (0 = pointed tail, no base drag)
  double theta_rad = 0.0;  // tailcone upsweep angle
  double psi_rad = 0.0;    // windshield angle

  double Sw_m2 = 0.0;   // wing reference area (drag-assembly rescale + Eq.9)
  double mac_m = 0.0;   // wing mean aerodynamic chord (pitch-assembly rescale)
  double b_m = 0.0;     // wingspan (yaw-assembly rescale)
};

// Parametric correction-curve coefficients for CM0/CMalpha (Nicolosi
// Eq.10-14). Every "*_anchor" is a real number printed in the paper
// (Eq.13/14); every "*_sensitivity" is an UNCALIBRATED trend-guessed slope
// -- see docs/model_formulas.pdf Sec.5-7 and t6c_fuselage_params.yaml.
struct PitchTuning {
  // shared reference point -- the same reference-fuselage geometry Eq.13/14
  // were computed at (FR=8.7, FRn=1.6, psi=40deg, FRt=2.8, theta=14deg)
  double fineness_ratio_ref = 0.0;
  double nose_fineness_ratio_ref = 0.0;
  double windshield_angle_ref_rad = 0.0;
  double tail_fineness_ratio_ref = 0.0;
  double upsweep_angle_ref_rad = 0.0;

  double cm0_fineness_ratio_anchor = 0.0;       // Eq.13 @ fineness_ratio_ref
  double cm0_fineness_ratio_sensitivity = 0.0;  // per unit FR

  double cm0_nose_correction_anchor = 0.0;      // Eq.13
  double cm0_nose_windshield_angle_sensitivity_per_rad = 0.0;
  double cm0_nose_fineness_ratio_sensitivity = 0.0;

  double cm0_tail_correction_anchor = 0.0;      // Eq.13
  double cm0_tail_upsweep_angle_sensitivity_per_rad = 0.0;
  double cm0_tail_fineness_ratio_sensitivity = 0.0;

  double cmalpha_fineness_ratio_anchor = 0.0;       // Eq.14, 1/rad
  double cmalpha_fineness_ratio_sensitivity = 0.0;

  double cmalpha_nose_correction_anchor = 0.0;      // Eq.14
  double cmalpha_nose_windshield_angle_sensitivity_per_rad = 0.0;
  double cmalpha_nose_fineness_ratio_sensitivity = 0.0;

  double cmalpha_tail_correction_anchor = 0.0;      // Eq.14
  double cmalpha_tail_upsweep_angle_sensitivity_per_rad = 0.0;
  double cmalpha_tail_fineness_ratio_sensitivity = 0.0;
};

// Parametric correction-curve coefficients for CNbeta (Nicolosi Eq.15-16).
// Same anchor/sensitivity pattern as PitchTuning; only FRn/FRt appear (the
// paper states no windshield/upsweep-angle dependence for yaw, Sec.6).
struct YawTuning {
  double fineness_ratio_ref = 0.0;
  double nose_fineness_ratio_ref = 0.0;
  double tail_fineness_ratio_ref = 0.0;

  double cnbeta_fineness_ratio_anchor = 0.0;        // Eq.16 @ fineness_ratio_ref
  double cnbeta_fineness_ratio_sensitivity = 0.0;   // per unit FR -- sign
                                                     // uncertain, see
                                                     // t6c_fuselage_params.yaml

  double cnbeta_nose_correction_anchor = 0.0;       // Eq.16
  double cnbeta_nose_fineness_ratio_sensitivity = 0.0;

  double cnbeta_tail_correction_anchor = 0.0;       // Eq.16
  double cnbeta_tail_fineness_ratio_sensitivity = 0.0;
};

struct TuningParams {
  // Roskam Eq.8 windshield-drag term, omitted (needs a graphical chart, no
  // closed form) -- manual calibration hook, default 0. Drag otherwise has
  // no tunable parameters at all: it is fully closed-form (Roskam/Kroo).
  double windshield_drag_increment = 0.0;

  PitchTuning pitch;
  YawTuning yaw;
};

struct AeroState {
  double V_mps = 0.0;
  double rho_kgm3 = 0.0;
  double mu_pas = 0.0;   // dynamic viscosity
  double mach = 0.0;
  double alpha_rad = 0.0;
  double beta_rad = 0.0;
};

// Every physics quantity Update() computes, in the order
// docs/model_formulas.pdf lists them. Scalars only -- see the header
// comment above on direction/frame resolution being left to the caller.
struct Output {
  // --- flow state (Sec.2) -------------------------------------------
  double reynolds = 0.0;
  double cf = 0.0;   // equivalent flat-plate coefficient, Eq.2

  // --- drag: Roskam/Kroo closed-form (Sec.3-4) ------------------------
  double cd_skin_friction = 0.0;   // Eq.6
  double cd_base = 0.0;            // Eq.7 (0 if db=0)
  double cd_upsweep = 0.0;         // Eq.9 (linearized-centreline approx.)
  double cd_fuselage_sfront = 0.0; // referenced to Sfront (paper convention)
  double cd_fuselage_sw = 0.0;     // rescaled to Sw (t6texan2.xml CDo/CDgear convention)
  double drag_n = 0.0;             // q * Sfront * cd_fuselage_sfront

  // --- pitching moment: Nicolosi anchored (Sec.5-7) -------------------
  double cm0_fr = 0.0;
  double dcm0_nose = 0.0;
  double dcm0_tail = 0.0;
  double cm0 = 0.0;                // Eq.11
  double cmalpha_fr = 0.0;
  double dcmalpha_nose = 0.0;
  double dcmalpha_tail = 0.0;
  double cmalpha = 0.0;            // Eq.12, 1/rad
  double cm = 0.0;                 // Eq.10: cm0 + cmalpha*alpha
  double pitch_moment_nm = 0.0;    // q * Sfront * df * cm

  // --- yawing moment: Nicolosi anchored (Sec.8-9) ---------------------
  double cnbeta_fr = 0.0;
  double dcnbeta_nose = 0.0;
  double dcnbeta_tail = 0.0;
  double cnbeta = 0.0;             // Eq.15, 1/rad
  double cn = 0.0;                 // Eq. (Sec.8): cnbeta*beta
  double yaw_moment_nm = 0.0;      // q * Sfront * df * cn
};

class FuselageModel {
 public:
  FuselageModel(Geometry geometry, TuningParams tuning);

  // Pure function of `state` and the geometry cached at construction --
  // Sfront/Swet/FR/FRn/FRt (Sec.1) are never recomputed here.
  Output Update(const AeroState& state) const;

  const Geometry& geometry() const { return geometry_; }
  const TuningParams& tuning() const { return tuning_; }

 private:
  Geometry geometry_;
  TuningParams tuning_;

  // Cached in the constructor (Sec.1 of model_formulas.pdf).
  double sfront_m2_ = 0.0;
  double swet_nose_m2_ = 0.0;
  double swet_cabin_m2_ = 0.0;
  double swet_tail_m2_ = 0.0;
  double swet_total_m2_ = 0.0;
  double fr_ = 0.0;
  double frn_ = 0.0;
  double frt_ = 0.0;
};

}  // namespace t6c_fuselage
