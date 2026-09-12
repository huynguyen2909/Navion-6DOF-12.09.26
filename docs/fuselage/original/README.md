# t6c_fuselage

T-6C Texan II fuselage aerodynamic coefficient model: parasite drag
(Roskam/Kroo closed-form), pitching moment, and yawing moment
(Nicolosi's shape/anchor structure), ported from F. Nicolosi,
P. Della Vecchia, D. Ciliberti, V. Cusati, "Fuselage aerodynamic
prediction methods," *Aerospace Science and Technology*, 55 (2016)
332-343.

## Files

| File | Role |
|---|---|
| [`t6c_fuselage.hpp`](t6c_fuselage.hpp) / [`.cpp`](t6c_fuselage.cpp) | The library: geometry, drag, pitching moment, yawing moment |
| [`t6c_fuselage_params.yaml`](t6c_fuselage_params.yaml) | T-6C's geometry and tuning numbers, human-readable (data only -- not parsed by any code here) |
| [`main.cpp`](main.cpp) | Everything the library leaves out: real numbers, a regression test against the paper's own Table 3, and a demo sweep for the T-6C |
| [`docs/model_formulas.pdf`](docs/model_formulas.pdf) | Every formula, in the exact order evaluated, with citations for every constant |
| [`docs/t6c_fuselage_class_diagram.puml`](docs/t6c_fuselage_class_diagram.puml) / [`t6c_fuselage_sequence.puml`](docs/t6c_fuselage_sequence.puml) | API shape and call sequence (open in VS Code, Alt+D to preview) |

## Decisions -- read this before the code

This model deliberately does **not** follow Nicolosi's paper end to end.
The paper's own method computes fuselage drag from shape factors
$K_n,K_c,K_t$ and computes $C_{M0},C_{M\alpha},C_{N\beta}$ from an
$FR$-term plus nose/tail corrections -- but **every one of those 12
curves is digitized from the authors' own STAR-CCM+ CFD sweep** (their
Figs.~4-6, 8-16), which this project has no access to. Two different
resolutions were used, on purpose, for two different reasons:

- **Drag**: $K_n,K_c,K_t$ have **zero** printed numeric value anywhere
  in the paper's text (chart-only) -- there is no real anchor to seed
  them from. Rather than invent a curve from nothing, drag instead uses
  the closed-form Roskam/Kroo formulas the paper itself cites as its own
  semi-empirical comparison baseline (Eq.6,7,9) -- real, validated, and
  requiring no fabricated constant. (Two likely OCR transcription errors
  in the extracted paper text were caught and corrected by checking
  against the paper's own Table 3 numbers -- see
  `docs/model_formulas.pdf` Sec.3 and `main.cpp`'s regression test.)
- **Pitching/yawing moment** ($C_{M0},C_{M\alpha},C_{N\beta}$): each
  *does* have exactly one real anchor point -- the paper's
  reference-fuselage worked example is printed as actual numbers, not a
  chart reading (Eq.13/14/16). No independently-validated closed-form
  alternative exists (Roskam's own fuselage-$C_m$ treatment turns out to
  be the same Multhopp lineage, and that lineage's own literature --
  see the AIAC-2017-066 conference paper -- flags it as unreliable
  without a case-specific correction factor found by wind-tunnel test).
  So each curve here is a line through that one real point, with an
  **uncalibrated** trend-guessed slope.

Also deliberately **not** in scope: fuselage lift ($C_L$) and rolling
moment ($C_l$) -- the paper itself states these are not modelled
("very low relevance in isolated fuselage geometry design").

## How the library is organized

Unlike `t6c_landing_gear`, this model is **stateless**: `FuselageModel`
caches only geometry-derived constants (frontal/wetted areas, fineness
ratios) once at construction; `Update()` is a pure function of the
`AeroState` passed in, with no per-step history. Read
`t6c_fuselage.cpp` top to bottom -- it runs in the same order as
`docs/model_formulas.pdf`:

| Step | Section (PDF) | What it does |
|---|---|---|
| Constructor | Sec.1 | Precompute $S_{\text{front}}$, $S_{\text{wet}}$ (nose/cabin/tail), $FR,FR_n,FR_t$ |
| `Update()` | Sec.2 | Flow state: $q$, $Re$, $C_{Dfp}$ |
| | Sec.3-4 | Drag: Roskam/Kroo skin-friction + base + upsweep, rescaled, force |
| | Sec.5-7 | Pitching moment: $C_{M0}$, $C_{M\alpha}$, assembled with $\alpha$, moment |
| | Sec.8-9 | Yawing moment: $C_{N\beta}$, assembled with $\beta$, moment |

## Calibrating with real CFD later

`docs/model_formulas.pdf` Sec.11 ("Calibration workflow") spells out
exactly how: run CFD on an actual T-6C fuselage geometry, fit a real
slope through the resulting $C_{M0}/C_{M\alpha}/C_{N\beta}$ points
(the trend-guess defaults in `t6c_fuselage_params.yaml` are placeholders
for exactly this), and update **only the yaml**, never the formulas in
`t6c_fuselage.cpp`. Drag needs no calibration -- it is already
closed-form. With $\geq$3-4 real data points per curve, consider
upgrading from the current fixed-linear form to a proper
piecewise-linear lookup table (`ChartTable1D`/`ChartTable2D`) -- not
built yet, noted here as the natural next step.

## Build & run

Needs a C++17 compiler. No external dependencies (no Eigen -- this
model is scalar coefficients only, not body-axis vectors):

```bash
cd T6C_fuselage
g++ -std=c++17 -Wall -Wextra -O2 main.cpp t6c_fuselage.cpp -o main_exe
```

```bash
./main_exe              # regression test + T-6C sweep
./main_exe --regression # regression test only
```

The regression test reproduces Nicolosi's own reference-fuselage case
(Table 1/3) and checks the Roskam/Kroo drag output against the paper's
own DATCOM column (0.0532) -- expect **+2.4%** error, which is what
caught the two transcription errors described above. The paper's
"Method" column (0.0470) is Nicolosi's own CFD-chart $K_n/K_c/K_t$
result and is not reproducible here (no CFD dataset).

The T-6C sweep prints `CD` (both $S_{\text{front}}$- and
$S_w$-referenced, the latter directly addable to `t6texan2.xml`'s
`CDo`/`CDgear` axis), drag force, and `CM` over a speed range at a
fixed 2&deg; angle of attack -- **CM/CN are uncalibrated**, printed for
visibility only, not validated against anything.
