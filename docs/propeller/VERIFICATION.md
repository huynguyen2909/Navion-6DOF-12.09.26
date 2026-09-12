# Propeller verification in Stage 3

`tests/test_propeller.cpp` contains fifteen test groups:

1. polar interpolation and endpoint clamping;
2. analytic constant-coefficient blade-element integral;
3. static momentum closure;
4. axial symmetry and reaction-torque bookkeeping;
5. propeller-to-BODY installation transform;
6. blade-element sample sums;
7. P-factor sign under positive and negative rotation;
8. aerodynamic pitch-rate damping;
9. gyroscopic and total-CG-moment identities;
10. density scaling;
11. radial/azimuth grid convergence;
12. takeoff-speed sweep with an explicit numerical inflow guess;
13. the common `ILoadComponent`/`BodyLoad` contract;
14. end-to-end HS + VS + Propeller + `NavionModel` + RK4 integration;
15. disabled-propeller zero load.

The common-contract tests verify:

```text
BodyLoad.forceBodyN == PropellerOutput.forceBodyN
BodyLoad.momentAboutCgBodyNm == PropellerOutput.totalMomentAtCgBodyNm
momentArmBodyNm == hubPositionFromCgBodyM × forceBodyN
```

The Stage 3 integration test registers three components, checks that
`LoadAccumulator` receives three contributions, and advances the nonlinear
rigid-body state through an RK4 step.

Build and run through CMake:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The propeller executable is named `navion_propeller_tests`.
