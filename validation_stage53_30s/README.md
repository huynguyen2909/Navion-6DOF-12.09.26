# Stage 5.3 validation artifacts

- `takeoff_roll.csv`: 15,002 rows, 94 numeric columns, t=-0.002..30 s;
- `run.log`: trim result, schedule samples and final liftoff diagnostic;
- `plot.log`: plotting tool diagnostic;
- `plots/`: state, total-load, per-component-load and ground-contact figures;
- `tests/`: output from all eight regression-test groups.

The supplied run did not lift off. At 30 s, airspeed is 45.9084 m/s, all
three contacts remain active, BODY normal reaction is about -4279.6 N and the
CG is 0.03781 m above its trimmed level.
