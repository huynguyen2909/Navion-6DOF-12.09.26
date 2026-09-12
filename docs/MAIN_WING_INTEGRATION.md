# MainWing integration

## Decision

The uploaded `VATC_MainWing` already satisfies `ILoadComponent` and is
stateless. It is therefore registered with `NavionModel::addLoadComponent()`;
no change to `NavionModel`, `RigidBody6DOF`, `LoadAccumulator`, RK4 or the
Landing Gear two-phase/PGS path is needed.

The source was placed under the common project paths and the convenience
aliases `MainWing` and `MainWingConfig` were added. The original `VATC_...`
names remain available.

## Runtime mapping

| MainWing quantity | Common source |
| --- | --- |
| density | `context.environment.airDensityKgM3` |
| V, alpha, beta | `context.flightCondition` |
| p, q, r | `context.state.angularRateBodyRadps` |
| aileron | `context.controls.aileronRad` |
| symmetric flap | `context.controls.flapRad` |

The legacy `config.flight` and `config.controls` fields are retained for
source/data compatibility and default documentation, but live simulation
values overwrite them through `EvaluationContext`.

## Retained calculation chain

`evaluateDetailed()` preserves the uploaded stage order:

1. read live runtime input;
2. calculate span, quarter-chord sweep, flapped area, incidence and `qbar`;
3. interpolate all flap/geometry tables;
4. build clean plus flap longitudinal coefficients;
5. build lateral coefficients from beta, p, r and aileron;
6. transform force coefficients to BODY axes;
7. transfer aerodynamic-reference moments to the configured output/CG point;
8. dimensionalize to `BodyLoad`.

The returned axes are BODY FRD. The moment is already about CG/output and the
coefficient-level transfer is algebraically the same as

```text
M_CG = M_aero_reference + r_CG_to_reference x F_BODY
```

so `LoadAccumulator` must only add it; it must not form another `r x F`.

All uploaded table values, geometry defaults, aerodynamic derivatives,
`body`/`stability` derivative-axis modes, `zero_referenced`/
`literal_roskam` flap-moment modes, intermediate diagnostic fields and
warnings remain present.

## Scope warning

The default `model.scope` is `provisional_aircraft_proxy`. The source itself
states that the lateral derivatives are initial whole-aircraft proxy values,
not validated isolated-wing derivatives. Combining them with an independent
Fuselage and Vertical Stabilizer can double-count side force, rolling moment
or yawing moment. Stage 5 preserves these numbers and the warning rather than
silently changing physics. A later data-validation stage must replace them
with isolated MainWing derivatives or adopt a clearly partitioned
whole-aircraft coefficient model.

`model.scope = "wing_only"` changes the provenance label only; it does not
magically identify new coefficients.

## Tests

`test_main_wing.cpp` checks the uploaded numerical default, V=0 behavior,
aileron response, flap-domain enforcement and the single CG moment transfer.
`test_full_aircraft.cpp` then evaluates MainWing through the shared
`NavionModel` alongside all other components.
