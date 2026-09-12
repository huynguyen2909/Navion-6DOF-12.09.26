#include "navion/components/fuselage/FuselageAerodynamics.hpp"

#include <cmath>

namespace navion::fuselage {

namespace {

constexpr double PI = 3.14159265358979323846;

// Section 1 helpers (model_formulas.pdf) -- exact nose-cone / cabin-cylinder
// / tailcone-frustum decomposition of a body of revolution. Closed-form
// solid-of-revolution geometry, not an empirical fit; the only
// approximation is modelling the nose/tailcone as straight-line cones
// instead of the true curved ogive.

double FrontalArea(const FuselageGeometry& g) { return (PI / 4.0) * g.df_m * g.df_m; }

double NoseWettedArea(const FuselageGeometry& g) {
  const double slant = std::sqrt(g.Ln_m * g.Ln_m + (g.df_m / 2.0) * (g.df_m / 2.0));
  return PI * (g.df_m / 2.0) * slant;
}

double CabinWettedArea(const FuselageGeometry& g) { return PI * g.df_m * g.Lc_m; }

double TailWettedArea(const FuselageGeometry& g) {
  const double half_taper = (g.df_m - g.db_m) / 2.0;
  const double slant = std::sqrt(g.Lt_m * g.Lt_m + half_taper * half_taper);
  return PI * (g.df_m / 2.0 + g.db_m / 2.0) * slant;
}

}  // namespace

FuselageAerodynamics::FuselageAerodynamics(
    FuselageGeometry geometry,
    FuselageTuning tuning
)
    : geometry_(geometry), tuning_(tuning) {
  sfront_m2_ = FrontalArea(geometry_);
  swet_nose_m2_ = NoseWettedArea(geometry_);
  swet_cabin_m2_ = CabinWettedArea(geometry_);
  swet_tail_m2_ = TailWettedArea(geometry_);
  swet_total_m2_ = swet_nose_m2_ + swet_cabin_m2_ + swet_tail_m2_;

  fr_ = geometry_.Lf_m / geometry_.df_m;
  frn_ = geometry_.Ln_m / geometry_.df_m;
  frt_ = geometry_.Lt_m / geometry_.df_m;
}

FuselageAerodynamicOutput FuselageAerodynamics::evaluate(
    const FuselageAeroState& state
) const {
  FuselageAerodynamicOutput out;

  // --- Section 2: flow state ------------------------------------------
  const double q = 0.5 * state.rho_kgm3 * state.V_mps * state.V_mps;
  out.reynolds = state.rho_kgm3 * state.V_mps * geometry_.Lf_m / state.mu_pas;
  // The turbulent flat-plate correlation below is undefined at Re <= 1.
  // During the stopped-propeller hold and the first smooth-ramp samples the
  // aerodynamic load is exactly negligible, so return the zero-load result
  // instead of evaluating log10(0) or a fractional power of a negative value.
  if (!(out.reynolds > 1.0) || !(q > 0.0)) {
    return out;
  }
  out.cf = 0.455 / (std::pow(std::log10(out.reynolds), 2.58) *
                     std::pow(1.0 + 0.144 * state.mach * state.mach, 0.65));

  // --- Section 3-4: drag, Roskam/Kroo closed-form -----------------------
  // 0.0025 (not the extracted-PDF text's 0.0225) -- matches the standard
  // Raymer/Hoerner fuselage form factor (l/d)/400; verified against the
  // paper's own Table 3 reference-case DATCOM value, see model_formulas.pdf
  // Sec.3 and main.cpp's regression test.
  out.cd_skin_friction = out.cf *
      (1.0 + 60.0 / (fr_ * fr_ * fr_) + 0.0025 * fr_) *
      (swet_total_m2_ / sfront_m2_);

  out.cd_base = 0.0;
  if (geometry_.db_m > 0.0) {
    const double db_over_df = geometry_.db_m / geometry_.df_m;
    out.cd_base = (0.029 * db_over_df * db_over_df * db_over_df /
                    std::sqrt(out.cd_skin_friction * sfront_m2_ / swet_total_m2_)) *
                   (swet_total_m2_ / sfront_m2_);
  }

  // Linearized-centreline approximation for Kroo's (h/l)_0.75lt -- this
  // project's own addition, see model_formulas.pdf Sec.3. Ratio is
  // Sfront/Sw (not the extracted-PDF text's Sw/Sfront) -- verified against
  // the paper's own Table 3 reference-case DATCOM value.
  const double h_over_l_075lt = 0.75 * std::tan(geometry_.theta_rad);
  out.cd_upsweep = 0.075 * (sfront_m2_ / geometry_.Sw_m2) * h_over_l_075lt;

  out.cd_fuselage_sfront = out.cd_skin_friction + out.cd_base + out.cd_upsweep +
                            tuning_.windshield_drag_increment;
  out.cd_fuselage_sw = out.cd_fuselage_sfront * (sfront_m2_ / geometry_.Sw_m2);
  out.drag_n = q * sfront_m2_ * out.cd_fuselage_sfront;

  // --- Section 5-7: pitching moment, Nicolosi anchored -------------------
  const FuselagePitchTuning& pt = tuning_.pitch;

  out.cm0_fr = pt.cm0_fineness_ratio_anchor +
               pt.cm0_fineness_ratio_sensitivity * (fr_ - pt.fineness_ratio_ref);
  out.dcm0_nose = pt.cm0_nose_correction_anchor +
                   pt.cm0_nose_windshield_angle_sensitivity_per_rad *
                       (geometry_.psi_rad - pt.windshield_angle_ref_rad) +
                   pt.cm0_nose_fineness_ratio_sensitivity * (frn_ - pt.nose_fineness_ratio_ref);
  out.dcm0_tail = pt.cm0_tail_correction_anchor +
                   pt.cm0_tail_upsweep_angle_sensitivity_per_rad *
                       (geometry_.theta_rad - pt.upsweep_angle_ref_rad) +
                   pt.cm0_tail_fineness_ratio_sensitivity * (frt_ - pt.tail_fineness_ratio_ref);
  out.cm0 = out.cm0_fr + out.dcm0_nose + out.dcm0_tail;

  out.cmalpha_fr = pt.cmalpha_fineness_ratio_anchor +
                    pt.cmalpha_fineness_ratio_sensitivity * (fr_ - pt.fineness_ratio_ref);
  out.dcmalpha_nose = pt.cmalpha_nose_correction_anchor +
                        pt.cmalpha_nose_windshield_angle_sensitivity_per_rad *
                            (geometry_.psi_rad - pt.windshield_angle_ref_rad) +
                        pt.cmalpha_nose_fineness_ratio_sensitivity *
                            (frn_ - pt.nose_fineness_ratio_ref);
  out.dcmalpha_tail = pt.cmalpha_tail_correction_anchor +
                        pt.cmalpha_tail_upsweep_angle_sensitivity_per_rad *
                            (geometry_.theta_rad - pt.upsweep_angle_ref_rad) +
                        pt.cmalpha_tail_fineness_ratio_sensitivity *
                            (frt_ - pt.tail_fineness_ratio_ref);
  out.cmalpha = out.cmalpha_fr + out.dcmalpha_nose + out.dcmalpha_tail;

  out.cm = out.cm0 + out.cmalpha * state.alpha_rad;
  out.pitch_moment_nm = q * sfront_m2_ * geometry_.df_m * out.cm;

  // --- Section 8-9: yawing moment, Nicolosi anchored ----------------------
  const FuselageYawTuning& yt = tuning_.yaw;

  out.cnbeta_fr = yt.cnbeta_fineness_ratio_anchor +
                   yt.cnbeta_fineness_ratio_sensitivity * (fr_ - yt.fineness_ratio_ref);
  out.dcnbeta_nose = yt.cnbeta_nose_correction_anchor +
                       yt.cnbeta_nose_fineness_ratio_sensitivity *
                           (frn_ - yt.nose_fineness_ratio_ref);
  out.dcnbeta_tail = yt.cnbeta_tail_correction_anchor +
                       yt.cnbeta_tail_fineness_ratio_sensitivity *
                           (frt_ - yt.tail_fineness_ratio_ref);
  out.cnbeta = out.cnbeta_fr + out.dcnbeta_nose + out.dcnbeta_tail;

  out.cn = out.cnbeta * state.beta_rad;
  out.yaw_moment_nm = q * sfront_m2_ * geometry_.df_m * out.cn;

  return out;
}

}  // namespace navion::fuselage
