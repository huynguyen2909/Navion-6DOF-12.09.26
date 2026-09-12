# Runtime configuration in Stage 5

No YAML file is parsed at runtime in this release. The active takeoff-roll
configuration is assembled as strongly typed C++ data by:

```text
include/navion/config/ProvisionalNavionConfig.hpp
src/config/ProvisionalNavionConfig.cpp
```

`makeProvisionalNavionConfig()` is the single entry point used by the Stage 5
simulation and full-aircraft integration test. It gathers mass/inertia,
MainWing, HS, VS, Fuselage, Propeller and Landing Gear parameters.

This directory intentionally contains no apparent aircraft YAML configuration
until a real parser/schema and a validated Navion data set are implemented.
Placing an unparsed YAML here would make it too easy to assume that editing the
file changes the simulation.

Historical T-6C input snapshots have been moved to `reference_data/t6c/`.
They are provenance/regression material only and are not opened by the
executable.
