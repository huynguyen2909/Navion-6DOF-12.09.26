# Ground static trim before takeoff roll

## Purpose

The trim stage establishes a stationary aircraft on the runway before any
propeller load is applied. It prevents an arbitrary initial strut penetration
or attitude from being treated as a valid equilibrium state.

## Controls used during trim

| Input | Value |
| --- | ---: |
| throttle | 0 |
| propeller enabled | false |
| landing gear extended | true |
| left/right parking brake | 1, 1 |
| flap/elevator/aileron/rudder | 0 rad |
| nose-wheel steering | 0 rad |

The solver explicitly disables the propeller and sets its speed scale to zero.
`throttle` is not yet coupled to an engine, governor or shaft-power state.

## Unknowns and residual

The state is made static by setting BODY velocity and angular rate to zero.
The unknown vector is

```text
x = [position_Down, roll, pitch]
```

North/East position and yaw are retained from the initial guess. At every
iteration the full `NavionModel::evaluate()` path is called, including normal
strut loads and PGS friction. Gravity is transformed to BODY axes and the
solver drives

```text
R(x) = [Fz_components + Fz_gravity, L_CG, M_CG] -> 0.
```

The Jacobian is calculated by finite differences. A bounded Newton correction
and backtracking line search are used, and the configured number of wheel
contacts must remain active. Default convergence tolerances are 0.1 N for
vertical force and 0.1 N m for roll/pitch moment.

## Stage 5.3 transition

After convergence:

1. the static contact state is committed once;
2. the converged point is logged at timeline `t=0`;
3. RK4 integrates a two-second, zero-RPM hold with parking brakes applied;
4. at `t=2 s`, the brakes are released and the fixed-pitch propeller begins a
   five-second smoothstep ramp from 0 to 2300 RPM;
5. RK4 continues at 2300 RPM until timeline `t=30 s`.

No velocity is assigned after trim. The `u` increase in the CSV is generated
by the force balance.

## Current limitation

The RPM ramp is prescribed, not the response of an engine/governor/shaft
dynamic model. It smooths the applied BEMT load but does not model engine
torque balance, shaft acceleration or a variable-pitch governor.
