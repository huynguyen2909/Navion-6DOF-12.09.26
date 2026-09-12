# Frozen Stage 5.3 conventions

These conventions are part of the component contract.

## Frames and signs

BODY is right-handed FRD: `+X` forward, `+Y` right, `+Z` down. Body rates
`p,q,r` and moments follow the right-hand rule. NED is `+North,+East,+Down`;
standard gravity is `[0,0,+9.80665] m/s^2`.

`RigidBodyState::attitudeBodyToNed` is Hamilton, scalar-first `q_nb` and rotates
BODY components to NED. `velocityBodyMps` is CG velocity relative to Earth,
resolved in BODY.

The ground is the flat NED plane
`positionNedM.z = LandingGearParameters::groundDownPositionNedM`, with outward
normal `[0,0,-1]` in NED. Gear positions and friction lever arms run from CG to
the point and are expressed in BODY.

## Units

SI is mandatory: metre, second, kilogram, radian, m/s, rad/s, newton, N.m,
kg.m², kg/m³ and pascal. `wheelSlipDeg` is the one intentional degree-valued
diagnostic because the retained Pacejka parameterization expects degrees.

## Load contract

All returned forces use BODY axes and all returned moments are about aircraft
CG in BODY axes. `BodyLoad` excludes gravity. An owner computes `r x F` exactly
once; `LoadAccumulator` only sums.

Landing Gear returns:

```text
groundNormalLoad   = spring/damper normal force and its CG moment
groundFrictionLoad = solved PGS point forces and their CG moments
ground total        = normal + friction
```

The PGS preprocessing receives gravity in its temporary preliminary load.
`RigidBody6DOF` independently adds gravity to the final translational equation;
gravity is not accumulated as a component load.

## Air and component controls

Common air-relative velocity is CG Earth-relative velocity minus NED wind
transformed to BODY. VS/HS and Propeller add their local `omega x r` terms.
MainWing consumes CG flight condition plus explicit body rates. Fuselage uses
CG flight condition because the uploaded scalar kernel defines no local-flow
offset.

- positive elevator: trailing edge down;
- positive rudder: trailing edge left;
- positive aileron and symmetric flap follow the uploaded MainWing convention;
- Fuselage drag defaults opposite `airRelativeVelocityBodyMps`, with BODY
  `-X` available as an explicit configuration mode;
- `propellerEnabled`: enable the fixed-pitch V1 propeller;
- `propellerSpeedScale`: nonnegative multiplier of the configured propeller
  speed, evaluated at each RK4 stage; zero is a stopped propeller;
- `throttle`: reserved until engine/governor coupling is implemented;
- `landingGearExtended`: enable geometric ground contact;
- `noseWheelSteeringRad`: requested physical wheel angle in radians, clamped
  to each steered gear's `maximumSteeringAngleRad`;
- `brakeLeft`, `brakeRight`: normalized `[0,1]` commands; out-of-range values
  are clamped at the Landing Gear boundary.

The uploaded standalone module accepted normalized nose steering in `[-1,1]`.
Stage 5 puts the unit conversion at the aircraft-control boundary: the common
API carries physical radians. Full steering range and per-gear limiting are
unchanged.

## Takeoff-roll control invariant

The supplied scenario uses parking brakes during ground trim and the
zero-RPM hold from t=0 to t=2 s. Both channels are exactly zero after release
at t=2 s and through the end at t=30 s. Nose steering, rudder, elevator,
aileron and flap are zero. PGS remains active for free rolling and lateral
constraints; `brake=0` is not a solver-disable switch.

## Time-step and history convention

The same positive outer-step `dt` is passed to every RK4 derivative stage and
the accepted-step commit. Component history is not changed during derivative
evaluation. Contact/slip and warm-start history advance only after
`commitAcceptedStep()`.
