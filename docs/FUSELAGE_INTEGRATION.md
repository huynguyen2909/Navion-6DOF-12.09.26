# Fuselage integration

## Decision: retain the kernel, add an adapter

The low-level `navion::fuselage::FuselageAerodynamics` kernel is stateless and
encapsulates its geometry-derived values. It does not require the globally
coupled treatment used by Landing Gear. Its output is deliberately scalar,
however: drag magnitude plus pitch/yaw moments about the paper reference.

Stage 5 therefore keeps the complete scalar kernel and adds
`navion::fuselage::FuselageComponent`, an `ILoadComponent` adapter. This is a
component-boundary extension, not a central architecture change.

## Preserved kernel functionality

The kernel and `FuselageAerodynamicOutput` retain:

- Reynolds number and compressible turbulent flat-plate `Cf`;
- Roskam/Kroo skin-friction, base, upsweep and manual windshield drag terms;
- drag coefficients referenced to frontal area and wing area;
- Nicolosi-anchored `CM0`, `CMalpha`, `CNbeta` correction terms;
- dimensional drag, pitch moment and yaw moment;
- all geometry and tuning fields;
- the original T-6C YAML, derivation PDF/TeX, PUML diagrams and
  regression/sweep example.

The factory functions copy the original example values exactly:

```cpp
makeNicolosiReferenceFuselageGeometry();
makeT6cReferenceFuselageGeometry();
makeT6cReferenceFuselageTuning();
makeT6cReferenceFuselageConfig();
makeProvisionalNavionFuselageConfig();
```

## Runtime and frame mapping

The adapter maps `EvaluationContext` as follows:

| Kernel input | Common source |
| --- | --- |
| V, Mach, alpha, beta | `context.flightCondition` |
| density, dynamic viscosity | `context.environment` |

The kernel returns scalar drag magnitude `D`, pitching moment `M_ref` and
yawing moment `N_ref`. For the default direction mode,

```text
e_D_BODY = -V_air_BODY / |V_air_BODY|
F_BODY   = D e_D_BODY
M_ref_BODY = [0, M_ref, N_ref]
M_CG_BODY  = M_ref_BODY + r_CG_to_ref x F_BODY
```

`DragDirectionMode::BodyNegativeX` retains the caller option to apply the
same scalar drag strictly along BODY `-X`. In either mode, the adapter returns
one `BodyLoad(F_BODY, M_CG_BODY)` and `LoadAccumulator` performs no additional
translation.

The aerodynamic-reference position is configuration data. It defaults to
zero because the uploaded source does not supply the offset from the
paper-reference point (`x/Lf=0.50`, `z/df=0.50`) to the aircraft CG. A real
Navion installation must set
`aerodynamicReferencePositionFromCgBodyM` from its mass/geometry data.

## Exact V=0 boundary

The original skin-friction expression contains `log10(Re)` and is undefined
at `Re=0`. At `V=0` or zero density, dynamic pressure and every dimensional
aerodynamic load have the exact limiting value zero. The adapter returns that
zero `BodyLoad` and marks `zeroDynamicPressureBypass=true`; it does not change
the positive-speed kernel. This makes the component valid at the first
takeoff-roll state without fabricating a drag coefficient.

## Data limitation

The `makeT6cReference...()` functions explicitly preserve T-6C source data
for regression. The main scenario calls
`makeProvisionalNavionFuselageConfig()`; its current values still inherit that
proxy geometry/tuning and are not re-labelled as validated Navion data.

The integrated file pair is deliberately split:

- `FuselageAerodynamics.*`: generic scalar equation kernel;
- `FuselageComponent.*`: common OOP/`BodyLoad` adapter used by `NavionModel`.

Both are compiled. The unchanged uploaded source snapshot remains under
`docs/fuselage/original/`, while its YAML is under `reference_data/t6c/`.

The component models no fuselage lift or rolling moment because those terms
are absent from the uploaded model. That is a model-scope limitation, not an
integration omission.

## Tests

`test_fuselage.cpp` preserves the original Nicolosi reference regression,
checks the exact V=0 limit, verifies both drag-direction modes and proves that
`r x F` is included exactly once. `test_full_aircraft.cpp` evaluates the
adapter through the common aircraft model.
