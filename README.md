# Navion nonlinear 6-DOF OOP — Stage 5.3 RPM ramp and 30-second run

Stage 5.3 extends the Stage 5.2 ground-static-trim model with a prescribed,
stage-wise propeller-speed input, a complete 30-second scenario timeline and
per-component load diagnostics. It does not remove or replace any Main Wing,
Fuselage, VS/HS, Propeller or Landing Gear/PGS calculation.

## Component and dynamics architecture

- Main Wing + aileron + symmetric plain flap;
- Horizontal Stabilizer + elevator;
- Vertical Stabilizer + rudder;
- Fuselage drag, pitching moment and yawing moment;
- fixed-blade-pitch BEMT Propeller with prescribed RPM scale;
- three-point Landing Gear with the original global warm-started PGS solver;
- `LoadAccumulator`, Newton–Euler `RigidBody6DOF`, `NavionModel` and RK4.

Five ordinary subsystems implement `ILoadComponent`. Landing Gear retains the
two-phase `IGroundContactComponent` contract because its PGS solve needs the
complete pre-friction aircraft load. `NavionModel` now also copies each
component's already-computed `BodyLoad` into `ModelEvaluation::componentLoads`
for logging; those copies are diagnostic and are not added twice.

All component loads are BODY FRD and all moments are about CG. Gravity remains
outside `BodyLoad` and is added exactly once by `RigidBody6DOF`.

## Stage 5.3 scenario

The example uses one timeline:

| Time | Phase | RPM | Brakes |
| ---: | --- | ---: | ---: |
| `-0.002 s` | initial ground-trim guess snapshot | 0 | 1, 1 |
| `0 s` | converged ground static trim | 0 | 1, 1 |
| `0 <= t < 2 s` | post-trim hold | 0 | 1, 1 |
| `2 <= t < 7 s` | takeoff roll and smoothstep RPM ramp | 0 to 2300 | 0, 0 |
| `7 <= t <= 30 s` | full-RPM continuation | 2300 | 0, 0 |

For the ramp, with `tau = clamp((t - 2)/5, 0, 1)`:

```text
RPM(t) = 2300 * tau^2 * (3 - 2*tau)
```

Blade pitch is constant. Flap, elevator, aileron, rudder and nose-wheel
steering stay at zero. BODY velocity is never scheduled: RK4 integrates it
from the assembled forces and moments. The run is time-limited at 30 seconds
and is deliberately not stopped at `V_rotation`.

`propellerSpeedScale` is a prescribed algebraic input evaluated at every RK4
stage. It is not an engine/governor or shaft-inertia state; `throttle` remains
diagnostic in the current model.

## Source layout

```text
include/navion/
├── components/
│   ├── fuselage/
│   ├── landing_gear/
│   ├── propeller/
│   ├── stabilizers/
│   └── wings/
├── config/
├── core/
├── dynamics/
├── initialization/
├── integration/
└── model/

src/
├── components/
│   ├── fuselage/
│   ├── landing_gear/
│   ├── propeller/
│   ├── stabilizers/
│   └── wings/
├── config/
└── initialization/
```

The active Fuselage implementation has two layers: `FuselageAerodynamics.*`
is the retained scalar kernel and `FuselageComponent.*` is its common
`ILoadComponent` adapter. Both are compiled. T-6C-labelled YAML/source files
under `reference_data/` and `docs/fuselage/original/` are provenance material;
the executable does not load YAML at runtime. Active assembly parameters are
in `src/config/ProvisionalNavionConfig.cpp` and remain provisional proxies.

## Build, test, simulate and plot

Linux/macOS or a single-configuration CMake generator:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
mkdir -p results
./build/navion_stage5_full_aircraft_takeoff_roll results/takeoff_roll.csv
python tools/plot_takeoff_results.py results/takeoff_roll.csv --output-dir results/plots
```

PowerShell with Ninja/MinGW:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
New-Item -ItemType Directory -Force results
.\build\navion_stage5_full_aircraft_takeoff_roll.exe results\takeoff_roll.csv
& .\.venv\Scripts\python.exe tools\plot_takeoff_results.py `
    results\takeoff_roll.csv --output-dir results\plots
```

Calling `.venv\Scripts\python.exe` directly avoids the PowerShell
`Activate.ps1` execution-policy restriction.

The plotting script creates:

- `takeoff_states.png`;
- `takeoff_total_loads.png`;
- `takeoff_component_loads.png`;
- `takeoff_ground_contact.png`.

## Liftoff interpretation

NED uses positive Down, so decreasing `position_down_m` or positive
`height_above_trim_m` means that the CG moved upward. This alone does not prove
liftoff: strut unloading and aircraft rotation can move the CG while wheels
remain in contact. The example therefore marks an airborne candidate only
when all contacts are absent and the total normal load is below 5 N, and
reports liftoff only if that condition persists for 0.25 s.

The supplied verification run reached 45.9084 m/s at 30 s, but retained three
contacts and about -4279.6 N BODY-Z normal load. The CG rose only 0.03781 m
relative to trim. It therefore did **not** lift off in this neutral-control,
fixed-pitch provisional case.

See `docs/BUILD_RUN_PLOT.md`, `docs/TAKEOFF_ROLL_SCENARIO.md`,
`docs/ARCHITECTURE.md`, `docs/CONVENTIONS.md` and
`docs/VERIFICATION_STAGE5.md` for details and interpretation limits.
