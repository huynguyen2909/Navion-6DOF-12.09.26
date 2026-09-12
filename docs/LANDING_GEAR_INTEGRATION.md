# Landing Gear and PGS integration

## Scope

The uploaded module was converted from Eigen and its standalone aircraft
types to the Stage 3 shared math/data types. The contact, tyre and PGS
algorithms are retained. `NavionModel` was extended because PGS needs the
whole-aircraft preliminary load and therefore cannot be called correctly from
an independent one-pass `computeLoad()`.

## Source-to-Stage-4 mapping

| Uploaded Landing Gear API/data | Stage 4 API/data | Treatment |
| --- | --- | --- |
| `Vec3`, `Mat3` (Eigen) | `navion::Vec3`, `navion::Matrix3` | Mechanical type migration |
| `GearParams` | `landing_gear::GearParameters` | Field names normalized; equations unchanged |
| `Params` | `LandingGearParameters` + common `MassProperties` | Aircraft mass/inertia now have one owner |
| `State` | `EvaluationContext::state` | Euler attitude replaced by common quaternion |
| `Controls` | `EvaluationContext::controls` | Uses common aircraft control object |
| `GearOutput` | `GroundContactPoint` and `GroundContactReport` | Full per-gear diagnostics retained |
| `MultiplierSpec` | `FrictionConstraint` | Adds owning gear and row kind explicitly |
| `GearReactionOutput` | `GroundContactEvaluation` | Normal and final ground loads retained |
| `SolveFriction()` | `PGSFrictionSolver::solve()` | Global PGS math retained |
| `UpdateGears()` | `evaluateContacts()` | Read-only trial evaluation |
| `ApplyFrictionResult()` | `applyFrictionResult()` + `commitAcceptedStep()` | Output assembly separated from accepted-history commit |

`makeT6cReferenceLandingGearParameters()` contains the exact uploaded SI gear,
spring, damping, friction, steering, Pacejka and solver values.
`makeT6cReferenceMassProperties()` retains the uploaded reference aircraft
mass and inertia (including its documented negated-Ixz conversion), but mass
and inertia are not duplicated inside the component: the PGS solver reads the
authoritative `EvaluationContext::massProperties` owned by `RigidBody6DOF`.
`EvaluationContext` also exposes a read-only pointer to the rigid body's cached
inverse inertia, so k1–k4 do not repeat a 3x3 inversion. Standalone solver tests
may omit that pointer, in which case `PGSFrictionSolver` computes the inverse.

## Preserved calculation path

For every configured gear:

1. transform the flat NED upward normal into BODY;
2. transform the uncompressed wheel point from BODY/CG to NED;
3. detect penetration below the ground plane;
4. project penetration onto the body strut axis;
5. form the compressed contact point and `v_contact = v_CG + omega x r`;
6. clamp steering and build roll/side/normal ground directions;
7. compute compression rate, including the original `compression/dt` clamp;
8. compute spring/damper strut force, non-tension rule and optional force cap;
9. update wheel-slip angle above the retained low-speed threshold;
10. build rolling and lateral multiplier bounds;
11. globally solve all contacting-gear rows with warm-started PGS;
12. route each lambda back to its gear diagnostic and sum `r x F` at CG.

No explicit tyre-force replacement has been introduced.

## Friction bounds and controls

Rolling/braking coefficient:

```text
mu_long = rolling_factor * mu_roll
        + brake * static_factor * (mu_static - mu_roll)
```

The second term applies only to a left/right brake-group wheel. Consequently,
at `brake=0`, `mu_long = rolling_factor * mu_roll`: free-rolling resistance
and its PGS row remain active.

The lateral coefficient retains the uploaded Pacejka form:

```text
x = B * slip_deg
mu_side = mu_static * sin(C * atan(x - E * (x - atan(x))))
          * static_factor
```

Each constraint is independently projected to `[-abs(mu*Fn), +abs(mu*Fn)]`,
matching the source model. This is not a combined ellipse/cone limit between
longitudinal and lateral force; changing that would be a future physics-model
change.

The common API carries nose-wheel steering as a physical angle in radians.
It is clamped to `maximumSteeringAngleRad`. If an upstream pilot/control model
has the original normalized command, convert it at the input boundary:

```cpp
controls.noseWheelSteeringRad =
    std::clamp(commandNorm, -1.0, 1.0) * noseMaximumSteeringRad;
```

## Two-phase `NavionModel` orchestration

Registration is intentionally separate from ordinary components:

```cpp
model.addLoadComponent(std::make_unique<HorizontalStabilizer>(hs));
model.addLoadComponent(std::make_unique<VerticalStabilizer>(vs));
model.addLoadComponent(
    std::make_unique<propeller::PropellerComponent>(propeller)
);
model.setGroundContactComponent(
    std::make_unique<landing_gear::LandingGearComponent>(gear)
);
```

Only one global ground-contact component can be installed. This prevents two
independent PGS solves from incorrectly treating mutually coupled contact rows
as separate systems.

At evaluation time, ordinary loads are summed before normal reaction and PGS.
The final Landing Gear `BodyLoad` is normal plus friction, in BODY axes about
CG, and is added once. `RigidBody6DOF` sees no Landing Gear-specific types.

## History, warm start and RK4

The source module cached compression, steering, slip and prior multipliers in
`LandingGearModel`. Calling its mutating update indiscriminately at k1–k4 would
make the answer depend on RK4 call order. Stage 4 keeps the same physical
history but divides its lifecycle:

- `evaluateContacts/solveFriction/applyFrictionResult`: read-only trial work;
- `commitAcceptedStep`: cache accepted contact/slip and solved multipliers;
- `resetGroundContactHistory`: clear history for reset/reinitialization.

PGS is still executed at every k1–k4 stage. Only temporal history mutation is
deferred. This preserves warm start without allowing k2, k3 or a rejected step
to overwrite the last accepted state.

## Diagnostics

`ModelEvaluation` exposes aggregate normal/friction load, contact count,
iteration count, convergence flag and per-gear:

- name and weight-on-wheels;
- compression and normal force;
- solved rolling and lateral multipliers;
- wheel slip in degrees;
- physical steering angle in radians.

If PGS reaches its configured iteration limit, its best projected solution is
retained and `frictionSolverConverged=false` is reported. This matches the
uploaded solver's non-throwing behavior while making the condition observable.

## Verification coverage

`tests/test_landing_gear.cpp` checks:

- analytic one-row projection;
- normal spring force/contact geometry;
- active rolling PGS with zero brakes;
- full-brake bound, steering clamp and Pacejka lateral bound;
- evaluate-versus-commit history isolation;
- two-phase inclusion of another component's force in PGS;
- use of gravity in PGS without double-counting it in Newton–Euler;
- per-gear diagnostics and no double accumulation;
- retracted/no-contact behavior;
- the original three-gear, six-row global PGS path.

The unchanged Stage 1–3 tests remain in the same CTest suite.

## Model limitations, not removed features

- Ground is a single fixed flat NED plane; runway slope/roughness is absent.
- Wheel spin dynamics and anti-skid are absent from the uploaded model.
- Longitudinal and lateral bounds are independent rather than combined.
- Touchdown/contact switching is nonsmooth and may require a smaller `dt`.
- The provided numeric factory is T-6C reference data, not validated Navion
  landing-gear data.

These limitations are documented inherited scope; Stage 4 does not silently
replace them with different laws.
