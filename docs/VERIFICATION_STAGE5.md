# Stage 5.3 verification record

## Build environment

- C++17 compiler with `-Wall -Wextra -Wpedantic -Wconversion -Wshadow`;
- optimized full scenario build (`-O2`);
- direct compiler source list because CMake was unavailable in the validation
  container; `CMakeLists.txt` contains the equivalent targets.

The library and 30-second scenario compiled without diagnostics.

## Regression tests

| Test group | Result | Main coverage |
| --- | --- | --- |
| core Stage 1 | PASS | math/data types, accumulator, 6-DOF, RK4 |
| stabilizers | PASS | VS/HS loads, controls, local flow, CG moments |
| propeller | PASS, 16 cases | BEMT/inflow plus zero/intermediate RPM scale |
| landing gear | PASS | contact, steering, braking/free roll, PGS/history |
| main wing | PASS | uploaded regression, controls, tables, CG transfer |
| fuselage | PASS | retained kernel regression, zero-speed and adapter |
| full aircraft | PASS | six components and reconstructable load reports |
| ground static trim | PASS | perturbed pose to three-contact equilibrium |

The full-aircraft test verifies that the vector sum of six
`ComponentLoadReport` objects reproduces `totalComponentLoad` within numerical
roundoff. No load is evaluated or accumulated twice.

## Schedule checks

The generated CSV contains 15,002 rows and 94 numeric columns, covering the
initial-guess snapshot plus accepted states through 30 s at `dt=0.002 s`.

| Time | Phase | RPM | Airspeed | Contacts |
| ---: | ---: | ---: | ---: | ---: |
| 0.000 s | trim | 0 | 0 | 3 |
| 1.000 s | hold | 0 | approximately 0 | 3 |
| 2.000 s | ramp start/release | 0 | approximately 0 | 3 |
| 3.000 s | ramp | 239.2 | 0.00019 m/s | 3 |
| 4.500 s | ramp midpoint | 1150 | 0.28859 m/s | 3 |
| 7.000 s | full RPM | 2300 | 5.91968 m/s | 3 |
| 16.002 s | full RPM | 2300 | 30.0389 m/s | 3 |
| 30.000 s | final | 2300 | 45.9084 m/s | 3 |

Thus the run crosses the former 30 m/s demonstration threshold but continues
to the requested final time.

The largest absolute reconstruction difference between per-component columns
and the corresponding total-load column over all rows was about
`9.82e-8` in the column's SI unit, caused by decimal CSV formatting.

## Liftoff result

At 30 s:

```text
position Down             = -0.979437 m
height above trimmed CG   =  0.0378099 m
NED Down velocity         = -0.000883668 m/s
active contacts           = 3
ground normal BODY Fz     = -4279.58 N
airborne candidate        = false
```

The Main Wing BODY-Z load reached approximately `-8043.43 N`, reducing the
normal reaction from about `-12258.3 N` at rest to `-4279.6 N`. The aircraft
therefore unloads the gear and the CG moves upward by about 3.8 cm, but all
three wheels remain loaded. No 0.25-second airborne interval was found.

This distinction is why z-Down is necessary but not sufficient: strut
extension can reduce Down position without wheel separation.

## Plot verification

The plotting tool read the 94-column CSV and generated four PNG files:

- `takeoff_states.png`;
- `takeoff_total_loads.png`;
- `takeoff_component_loads.png`;
- `takeoff_ground_contact.png`.

All four were rendered and visually inspected. The phase shading and event
markers identify trim completion, brake release/RPM-ramp start and 2300 RPM.

## Scope limit

This verifies deterministic integration and diagnostic accounting, not real
Navion performance. Aerodynamic, mass, inertia, gear and installation values
remain provisional/mixed-source; the example does not command elevator
rotation or flap deployment; and prescribed RPM is not a dynamic engine and
governor model.
