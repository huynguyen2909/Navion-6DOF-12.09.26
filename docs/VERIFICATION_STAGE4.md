# Stage 4 verification record

## Compiler checks

All library translation units, four test executables and all examples were
compiled as C++17 with:

```text
-O2 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
```

The verification environment did not provide CMake, so the same source list
was built directly with `g++ 13.3.0`. `CMakeLists.txt` includes the new Landing
Gear source, test and takeoff-roll executable for normal project builds.

## Executed regression suites

| Suite | Result |
| --- | --- |
| Stage 1 core math/load/rigid-body/NavionModel/RK4 | Pass |
| Stage 2 Horizontal/Vertical Stabilizer | Pass |
| Stage 3 Propeller kernel/component/integration (15 groups) | Pass |
| Stage 4 Landing Gear/PGS/two-phase integration | Pass |

The Stage 4 suite covers the original three-gear/six-row PGS path in addition
to isolated analytic and contract tests. See
`docs/LANDING_GEAR_INTEGRATION.md` for the detailed list.

All four suites were also run with AddressSanitizer and UndefinedBehaviorSanitizer.
They passed with leak detection disabled because LeakSanitizer is unsupported
under the verification container's process tracing; address and UB checks were
active.

## Executed takeoff-roll smoke scenario

The supplied example was run from `V=0` with both brake channels fixed at zero.
Observed end record:

```text
stopped_at_t_s=10.2940
speed_mps=30.0004
target_V_rotation_mps=30.0000
active_contacts=3 throughout sampled output
brakes=0,0 throughout sampled output
```

This verifies code flow and numerical execution, not aircraft fidelity. Wing,
Fuselage and validated Navion data are still required for a performance-grade
takeoff simulation.

## Recommended local verification

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/navion_stage4_takeoff_roll
```

For contact stability, also repeat relevant cases with `dt`, `dt/2` and `dt/4`
and inspect penetration, normal load, PGS convergence and trajectory rather
than relying only on a finite result.
