# Stage 5.3 ground trim, RPM ramp and 30-second takeoff roll

## Purpose

`examples/stage5_full_aircraft_takeoff_roll.cpp` exercises the complete path:

```text
ordinary component loads -> Landing Gear normal load -> global PGS friction
-> LoadAccumulator -> RigidBody6DOF -> RK4 -> accepted-step contact commit
```

It logs every phase on one time axis. It is an architecture and numerical
integration scenario, not a validated Navion takeoff-performance case.

## Timeline and controls

Ground static trim solves CG Down, roll and pitch from `Fz/L/M` equilibrium at
zero velocity with the propeller stopped and parking brakes applied. An
initial-guess snapshot is logged at `t=-0.002 s` and the converged point at
`t=0`.

| Interval | RPM rule | Brake L/R | Other controls |
| --- | --- | --- | --- |
| `0 <= t < 2 s` | 0 | 1 / 1 | neutral |
| `2 <= t < 7 s` | `2300*smoothstep((t-2)/5)` | 0 / 0 | neutral |
| `7 <= t <= 30 s` | 2300 | 0 / 0 | neutral |

`smoothstep(x) = x^2(3-2x)` after clamping `x` to `[0,1]`. The first derivative
of RPM is zero at both ramp ends, avoiding an RPM-slope discontinuity.

Blade pitch remains the configured geometry distribution. Flap, elevator,
aileron, rudder and nose steering are zero for the full run. The current
`throttle` field mirrors RPM scale for diagnostics but does not drive an
engine/governor model.

The schedule function is called for every RK4 trial time, so k1-k4 receive
their own instantaneous RPM. BODY velocity is not prescribed. The scenario
runs to 30 s even after crossing the 30 m/s demonstration rotation-speed
value used in earlier stages.

## Component-load reporting

`NavionModel::evaluateCoupled()` evaluates each ordinary component exactly
once, adds its `BodyLoad` to `LoadAccumulator` and stores a diagnostic copy in
`ModelEvaluation::componentLoads`. After the two-phase Landing Gear solve, its
normal-plus-PGS-friction load is stored in the same report vector.

The CSV contains six BODY force/moment channels for each of:

- Main Wing;
- Horizontal Stabilizer;
- Vertical Stabilizer;
- Fuselage;
- Propeller;
- Landing Gear + PGS.

Their sum reconstructs `totalComponentLoad`; gravity is still separate.

## PGS and accepted-step history

For each outer time step:

1. RK4 requests four derivatives at trial states;
2. each derivative recomputes ordinary loads, contacts, normal loads and PGS;
3. all four calls receive the same positive outer `dt`;
4. RK4 returns the accepted end state;
5. `NavionModel::commitAcceptedStep()` advances gear/slip/warm-start history
   once.

Releasing the brakes does not disable PGS. Rolling and lateral rows remain
active whenever the corresponding wheel is in contact.

## Liftoff diagnostic

Because the frame is NED, `position_down_m` decreasing means upward CG motion
and `velocity_down_ned_mps < 0` means upward NED velocity. Neither alone proves
wheel separation. Suspension decompression and aircraft attitude changes can
produce both while contact is retained.

The scenario logs `airborne_candidate=1` only when:

```text
ground_contact_count == 0
and norm(groundNormalForceBody) <= 5 N
```

The terminal reports a liftoff candidate only after the condition persists
for 0.25 s. A physically validated takeoff assessment should additionally
check sustained climb, reasonable attitude/rates, runway data and validated
aerodynamic/propulsive parameters.

## Current verified result

With the provisional configuration and all control surfaces neutral:

```text
t_final                 = 30.0000 s
airspeed                 = 45.9084 m/s
contacts                 = 3
ground normal BODY Fz    = -4279.58 N
height above trimmed CG  = 0.03781 m
airborne candidate       = false
```

The decreasing normal reaction and small upward CG displacement show wing
unloading of the gear, not liftoff. Rotation is not commanded, and this result
must not be treated as a real Navion takeoff limit.
