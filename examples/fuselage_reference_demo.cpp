// Standalone reference/regression driver for the generic fuselage kernel.
// Numeric source data are preserved in
// reference_data/t6c/fuselage_source_parameters.yaml and in Nicolosi's paper.
//
//   --regression  Reproduces Nicolosi's own Table 3 "Reference" fuselage
//                 case (Lf=30m, df=3.45m, ...) and checks the Roskam/Kroo
//                 drag output against the paper's own DATCOM column
//                 (0.0532) -- this is what caught the 0.0225->0.0025 and
//                 Sw/Sfront->Sfront/Sw transcription errors documented in
//                 docs/fuselage/original/docs/model_formulas.pdf Sec.3.
//   (default)     Sweeps V for the T-6C's own (partly ESTIMATED) geometry
//                 and prints CD/CM/CN and the resulting forces/moments.
//
// No visualization and no real-time loop: the aerodynamic kernel is stateless.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "navion/components/fuselage/FuselageAerodynamics.hpp"

namespace {

constexpr double PI = 3.14159265358979323846;

double DegToRad(double deg) { return deg * PI / 180.0; }

// ---------------------------------------------------------------------
// Nicolosi's own reference-fuselage geometry (paper Table 1, with df/Sfront
// taken from Table 3's "Reference" row -- the paper itself uses df=3.45m
// there, not Table 1's 3.4m; a ~1.5% discrepancy in the paper, not ours).
// Used only by the regression test.
navion::fuselage::FuselageGeometry MakeNicolosiReferenceGeometry() {
  navion::fuselage::FuselageGeometry g;
  g.Lf_m = 30.0;
  g.df_m = 3.45;
  g.Ln_m = 5.7;
  g.Lc_m = 13.0;
  g.Lt_m = 11.3;
  g.db_m = 0.0;  // not stated in the paper; pointed tail assumed
  g.theta_rad = DegToRad(14.0);
  g.psi_rad = DegToRad(40.0);
  g.Sw_m2 = 75.5;
  g.mac_m = 2.58;
  g.b_m = 0.0;  // not needed for the drag-only regression check
  return g;
}

// Source-aircraft geometry preserved for reference verification only. See the
// YAML file above for which values were VERIFIED versus ESTIMATED.
navion::fuselage::FuselageGeometry MakeSourceAircraftGeometry() {
  navion::fuselage::FuselageGeometry g;
  g.Lf_m = 10.18;
  g.df_m = 1.05;
  g.Ln_m = 1.93;
  g.Lc_m = 4.41;
  g.Lt_m = 3.84;
  g.db_m = 0.0;
  g.theta_rad = DegToRad(14.0);
  g.psi_rad = DegToRad(40.0);
  g.Sw_m2 = 16.35;
  g.mac_m = 1.606;
  g.b_m = 10.18;
  return g;
}

// Source-aircraft tuning, hand-transcribed from the reference YAML. Every
// pitch/yaw field here is UNCALIBRATED.
navion::fuselage::FuselageTuning MakeSourceAircraftTuning() {
  navion::fuselage::FuselageTuning t;
  t.windshield_drag_increment = 0.0;

  navion::fuselage::FuselagePitchTuning& p = t.pitch;
  p.fineness_ratio_ref = 8.7;
  p.nose_fineness_ratio_ref = 1.6;
  p.windshield_angle_ref_rad = DegToRad(40.0);
  p.tail_fineness_ratio_ref = 2.8;
  p.upsweep_angle_ref_rad = DegToRad(14.0);

  p.cm0_fineness_ratio_anchor = -0.3159;
  p.cm0_fineness_ratio_sensitivity = 0.03;
  p.cm0_nose_correction_anchor = 0.0013;
  p.cm0_nose_windshield_angle_sensitivity_per_rad = 0.01;
  p.cm0_nose_fineness_ratio_sensitivity = 0.01;
  p.cm0_tail_correction_anchor = 0.0013;
  p.cm0_tail_upsweep_angle_sensitivity_per_rad = 0.005;
  p.cm0_tail_fineness_ratio_sensitivity = 0.005;

  p.cmalpha_fineness_ratio_anchor = 0.1815;
  p.cmalpha_fineness_ratio_sensitivity = 0.02;
  p.cmalpha_nose_correction_anchor = 0.0005;
  p.cmalpha_nose_windshield_angle_sensitivity_per_rad = 0.0005;
  p.cmalpha_nose_fineness_ratio_sensitivity = 0.0005;
  p.cmalpha_tail_correction_anchor = 0.0006;
  p.cmalpha_tail_upsweep_angle_sensitivity_per_rad = 0.0005;
  p.cmalpha_tail_fineness_ratio_sensitivity = 0.0005;

  navion::fuselage::FuselageYawTuning& y = t.yaw;
  y.fineness_ratio_ref = 8.7;
  y.nose_fineness_ratio_ref = 1.6;
  y.tail_fineness_ratio_ref = 2.8;

  y.cnbeta_fineness_ratio_anchor = 0.1817;
  y.cnbeta_fineness_ratio_sensitivity = -0.02;
  y.cnbeta_nose_correction_anchor = -0.0003;
  y.cnbeta_nose_fineness_ratio_sensitivity = -0.0005;
  y.cnbeta_tail_correction_anchor = -0.0002;
  y.cnbeta_tail_fineness_ratio_sensitivity = -0.0005;

  return t;
}

// ---------------------------------------------------------------------
void RunRegressionTest() {
  std::printf("=== Regression test: Nicolosi reference fuselage (Table 1/3) ===\n");
  std::printf("Geometry: Lf=30m df=3.45m FR=%.3f  Re=202e6  M=0.52\n",
              30.0 / 3.45);

  navion::fuselage::FuselageAerodynamics model(
      MakeNicolosiReferenceGeometry(),
      navion::fuselage::FuselageTuning{});  // drag needs no tuning

  navion::fuselage::FuselageAeroState state;
  // rho/mu are arbitrary (only Re, computed from them below, and M feed
  // CD -- force isn't checked here); V is solved backwards so that
  // Re = rho*V*Lf/mu lands exactly on the paper's own Table 2 value (2.02e8).
  state.rho_kgm3 = 1.0;
  state.mu_pas = 1.0;
  state.V_mps = 202e6 * state.mu_pas / (state.rho_kgm3 * 30.0);
  state.mach = 0.52;
  state.alpha_rad = 0.0;
  state.beta_rad = 0.0;

  const navion::fuselage::FuselageAerodynamicOutput out = model.evaluate(state);

  std::printf("Re = %.4e (target 2.02e8)\n", out.reynolds);
  std::printf("Cf (Eq.2)          = %.6f\n", out.cf);
  std::printf("CDsf  (Eq.6)       = %.5f\n", out.cd_skin_friction);
  std::printf("CDbase(Eq.7)       = %.5f\n", out.cd_base);
  std::printf("CDupsweep (Eq.9)   = %.5f\n", out.cd_upsweep);
  std::printf("CD_fus (Sfront-ref) = %.5f   paper DATCOM = 0.0532   paper CFD = 0.0472\n",
              out.cd_fuselage_sfront);
  const double error_pct =
      100.0 * (out.cd_fuselage_sfront - 0.0532) / 0.0532;
  std::printf("Error vs paper's own DATCOM column: %+.1f%%\n", error_pct);
  std::printf("(Method column, 0.0470, is Nicolosi's CFD-chart Kn/Kc/Kt result --\n"
              " not reproducible here, no CFD dataset; see project README.)\n\n");
}

// ---------------------------------------------------------------------
void RunSourceAircraftSweep() {
  std::printf("=== Source-aircraft fuselage sweep (partly ESTIMATED -- see\n");
  std::printf("    reference_data/t6c/fuselage_source_parameters.yaml) ===\n");

  navion::fuselage::FuselageAerodynamics model(
      MakeSourceAircraftGeometry(), MakeSourceAircraftTuning());

  std::printf("%8s %10s %10s %10s %10s %10s\n", "V(m/s)", "Re", "CD_Sfront",
              "CD_Sw", "Drag(N)", "CM");
  for (double v = 40.0; v <= 150.0; v += 10.0) {
    navion::fuselage::FuselageAeroState state;
    state.V_mps = v;
    state.rho_kgm3 = 1.225;  // ISA sea level
    state.mu_pas = 1.789e-5;  // ISA sea level
    state.mach = v / 340.3;
    state.alpha_rad = DegToRad(2.0);  // fixed small AoA for the sweep
    state.beta_rad = 0.0;

    const navion::fuselage::FuselageAerodynamicOutput out = model.evaluate(state);
    std::printf("%8.1f %10.3e %10.5f %10.5f %10.2f %10.5f\n", v, out.reynolds,
                out.cd_fuselage_sfront, out.cd_fuselage_sw, out.drag_n, out.cm);
  }
  std::printf("\n(CM/CN use UNCALIBRATED trend-guessed slopes -- do not treat as\n"
              " validated; see docs/fuselage/original/docs/model_formulas.pdf"
              " Sec.11.)\n");
}

}  // namespace

int main(int argc, char** argv) {
  const bool regression_only = argc > 1 && std::strcmp(argv[1], "--regression") == 0;

  RunRegressionTest();
  if (!regression_only) {
    RunSourceAircraftSweep();
  }
  return 0;
}
