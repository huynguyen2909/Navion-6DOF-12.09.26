# Stage 5.3 architecture

## Dependency and ownership rules

`RigidBody6DOF` knows no aircraft subsystem. A physical component never
integrates aircraft state. `NavionModel` alone owns component evaluation order,
load assembly and the handoff to Newton–Euler. `RK4Integrator` remains a
generic state integrator.

| Type | Responsibility | Prohibited responsibility |
| --- | --- | --- |
| `ILoadComponent` | Independently return one BODY/CG `BodyLoad` | Depend on another component's load or mutate RK4 history |
| `IGroundContactComponent` | Create normal/contact constraints, solve coupled friction and commit accepted history | Integrate rigid-body state or hide gravity in its returned `BodyLoad` |
| `LandingGearComponent` | Spring/damper contact, steering, brake/rolling/Pacejka bounds, per-gear history | Sum unrelated aircraft components |
| `PGSFrictionSolver` | Solve global projected `J M^-1 J^T` multiplier system | Know gear names or aircraft component types |
| `LoadAccumulator` | Validate and sum BODY-axis loads about CG | Recompute `r x F`, add gravity or solve constraints |
| `RigidBody6DOF` | Evaluate Newton–Euler and kinematics | Know PGS, components or numerical integration |
| `NavionModel` | Build common context and coordinate both load phases | Advance a time step |
| `GroundStaticTrimSolver` | Solve pre-power `Down/roll/pitch` from `Fz/L/M` equilibrium | Prescribe takeoff velocity or advance simulation time |
| `RK4Integrator` | Build k1–k4 trial states and combine derivatives | Own physical history or commit component state |

## Ordinary component contract

MainWing, Fuselage, Horizontal Stabilizer, Vertical Stabilizer and Propeller
are order-independent load producers:

```text
EvaluationContext -> ILoadComponent::computeLoad() -> BodyLoad(F_BODY, M_CG_BODY)
```

Their complete moments already include the point-transfer term:

```text
M_CG = M_intrinsic_at_load_point + r_CG_to_point x F + owned extra moments
```

Stage 5.3 adds `ComponentLoadReport` to `ModelEvaluation`. `NavionModel`
stores the exact `BodyLoad` returned by each component while adding it to the
accumulator, then stores Landing Gear's normal-plus-friction result. This is a
read-only diagnostic path: no component is evaluated twice and no reported
load is summed twice.

## Why ground contact is evaluated in two phases

The original Landing Gear PGS equations require the aircraft acceleration
that would exist before friction. That acceleration depends on the sum of all
other loads. Ground friction therefore cannot be a normal, independent
`ILoadComponent` without either using incomplete data or introducing hidden
ordering.

For each derivative evaluation, `NavionModel::evaluateCoupled()` performs:

```text
L_regular = sum(MainWing, Fuselage, HS, VS, Propeller)
L_normal  = LandingGear.evaluateContacts(context, dt).normalLoad
L_pre     = L_regular + L_normal + [m*g_BODY, 0]
L_friction = PGS(L_pre, state, mass, inertia, dt, constraints)
L_ground  = L_normal + L_friction
L_components = L_regular + L_ground
xdot = RigidBody6DOF(state, L_components, gravity_NED)
```

Gravity appears in `L_pre` only because PGS needs it to predict contact-point
velocity. It is not stored in `L_ground` or `L_components`; the rigid body adds
gravity exactly once.

## PGS internals retained from the uploaded source

For multiplier rows `i,j`, the solver builds:

```text
A_ij = u_j . (u_i/m + (I^-1 (r_i x u_i)) x r_j)
```

The pre-friction rigid-body derivatives are:

```text
v_dot_pre     = F_pre/m - omega x v
omega_dot_pre = I^-1 (M_pre - omega x (I omega))
```

and the stabilization input is:

```text
v_dot_pre + v/dt
omega_dot_pre + omega/dt
```

The solver normalizes each row, warm-starts from accepted multipliers, sweeps
all rows with Projected Gauss–Seidel and clamps every rolling/lateral lambda to
its friction bound. The resulting point forces are summed and transferred to
CG with `r x F`.

## RK4 history rule

Every k1–k4 evaluation reads the same last-accepted Landing Gear history but
uses its own trial state, controls and common outer-step `dt`. Evaluation does
not modify compression/slip/steering or warm-start values.

After RK4 produces the accepted state, the driver calls
`NavionModel::commitAcceptedStep()`. `NavionModel` re-evaluates the coupled
loads at that accepted state and commits exactly once. This prevents rejected
or intermediate stages from becoming physical history while retaining the
original low-speed slip hold and temporal-coherence warm start.

Because the original PGS law includes `v/dt`, contact dynamics are a
time-step-dependent constrained update rather than a smooth autonomous ODE.
RK4 still evaluates all four stage states correctly, but classical fourth-order
convergence should not be assumed across contact/touchdown discontinuities.

## Extension rule

Further ordinary aerodynamic components implement `ILoadComponent`; no
Landing Gear or rigid-body code changes are needed. Another globally coupled
contact system would require either replacing the
single `IGroundContactComponent` or introducing a higher-level constraint
coordinator so all contact rows are solved in one system.

The Fuselage scalar-to-vector conversion is isolated in
`FuselageComponent`; the original scalar kernel remains independently
testable. MainWing requires no adapter because it already returns a complete
BODY/CG `BodyLoad`.

See `stage5_classes.puml` and `stage5_sequence.puml`.

## Initialization rule

Ground static trim is outside the time integrator. It repeatedly calls the
same complete `NavionModel::evaluate()` path with zero rates and adjusts only
CG Down position, roll and pitch. After convergence, the caller commits the
contact state once. The Stage 5.3 scenario then integrates a two-second
zero-RPM hold, releases the brakes, applies a five-second stage-wise RPM ramp
and continues to 30 seconds. See
`GROUND_STATIC_TRIM.md`.
