# Model-data status: Navion target versus surrogate sources

The simulated aircraft target is Navion, but Stage 5 is still a V1 integration
model. Its software architecture and its parameter fidelity must not be
confused.

## Active runtime source

The executable does not read YAML. Its active data are constructed by
`makeProvisionalNavionConfig()` in
`src/config/ProvisionalNavionConfig.cpp`.

| Subsystem | Active Stage 5 data status |
| --- | --- |
| Mass/inertia | provisional integration values, not validated Navion data |
| MainWing | Navion-oriented geometry/defaults; lateral derivatives remain a whole-aircraft proxy |
| Horizontal/Vertical Stabilizer | Navion-oriented estimated configuration |
| Fuselage | T-6C-derived geometry/tuning used explicitly as a provisional proxy |
| Propeller | estimated Navion installation using the documented NACA 5868-9 surrogate |
| Landing Gear | provisional three-point geometry/stiffness used by the takeoff example |

`ProvisionalNavionConfig` is named this way intentionally: it is the coherent
software assembly point for the Navion-target simulation, not a claim that
all numbers have been validated for Navion.

## Why T-6C-labelled files remain

T-6C labels now appear only where provenance requires them:

- `reference_data/t6c/`: uploaded source-value snapshots, never parsed;
- `docs/fuselage/original/`: an unchanged original-module snapshot;
- explicit `makeT6cReference...()` factories and regression tests.

Those factories remain so the uploaded Landing Gear/Fuselage results can be
reproduced. The main Stage 5 scenario does not call them by name; it calls the
provisional Navion assembly factory.

Deleting or relabelling those reference sources as Navion would make the data
lineage less trustworthy. Moving them outside runtime configuration makes
their actual role unambiguous.

## Next data-fidelity step

Before performance validation, replace the fields in
`makeProvisionalNavionConfig()` with one consistent Navion data set and add
component-level regression targets. In particular:

1. replace fuselage geometry/tuning and set its aerodynamic-reference-to-CG
   position;
2. replace MainWing whole-aircraft lateral proxy derivatives with isolated
   wing contributions, avoiding double counting with VS/Fuselage;
3. validate mass/inertia, landing-gear geometry/stiffness/friction and
   propulsion settings against the same Navion variant;
4. only then introduce a runtime YAML/JSON schema if external configuration
   is required.
