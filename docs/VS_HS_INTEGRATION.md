# Integrating the Stage 2 VS_HS overlay

## Recommended installation

Start from the root of the extracted Stage 1 project—the directory containing
`CMakeLists.txt`, `include`, `src`, `tests`, and `docs`.

1. Make a backup or commit the Stage 1 directory.
2. Extract `Navion_6DOF_OOP_Stage2_VS_HS_Overlay.zip` directly into that root.
3. Allow files such as `CMakeLists.txt`, `README.md`, and
   `docs/ARCHITECTURE.md` to be replaced.
4. Delete the old CMake `build` directory if one exists; target names changed.
5. Configure, build, and test from the project root.

PowerShell:

```powershell
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The expected tests are `navion_core_tests` and `navion_stabilizer_tests`.

## Files added by the overlay

```text
include/navion/components/ILocalFlowField.hpp
include/navion/components/stabilizers/VATC_HorizontalStabilizer.hpp
include/navion/components/stabilizers/VATC_VerticalStabilizer.hpp
src/components/stabilizers/VATC_HorizontalStabilizer.cpp
src/components/stabilizers/VATC_VerticalStabilizer.cpp
tests/test_stabilizers.cpp
examples/stabilizer_integration.cpp
docs/VS_HS_INTEGRATION.md
docs/stage2_vs_hs_classes.puml
docs/stage2_vs_hs_sequence.puml
```

It also replaces `CMakeLists.txt`, `README.md`, `docs/ARCHITECTURE.md`, and
`docs/CONVENTIONS.md`. Stage 1 core sources are otherwise unchanged.

## Old-to-new API mapping

| Uploaded VS_HS API | Stage 2 source |
| --- | --- |
| `setVelocity(V_CG)` | `context.flightCondition.airRelativeVelocityBodyMps` |
| `setAngularRate(p,q,r)` | `context.state.angularRateBodyRadps` |
| `setAirDensity(rho)` | `context.environment.airDensityKgM3` |
| `setMach(M)` | Available in `context.flightCondition.mach`; current equations do not use it |
| `setElevatorDeflection(delta_e)` | `context.controls.elevatorRad` |
| `setRudderDeflection(delta_r)` | `context.controls.rudderRad` |
| downwash/sidewash/propwash/gust setters | injected `ILocalFlowField` |
| `update()` | `computeLoad(context)` or `evaluateDetailed(context)` |
| `getForceBody()` | `BodyLoad::forceBodyN` |
| `getMomentBody()` | `BodyLoad::momentAboutCgBodyNm` |
| scalar diagnostic getters | fields of `HorizontalStabilizerEvaluation` or `VerticalStabilizerEvaluation` |
| `getConfig()` | retained as a const accessor |

Configuration member names and array layout were retained so existing geometry
and coefficient assignment code needs only the `navion::` namespace and the
new constructor/registration path. Clean aliases are available:

```cpp
navion::HorizontalStabilizerConfig
navion::HorizontalStabilizer
navion::VerticalStabilizerConfig
navion::VerticalStabilizer
```

## Registration

```cpp
navion::NavionModel model(massProperties);

model.addLoadComponent(
    std::make_unique<navion::HorizontalStabilizer>(horizontalConfig)
);
model.addLoadComponent(
    std::make_unique<navion::VerticalStabilizer>(verticalConfig)
);
```

Do not manually call either stabilizer from the RK4 loop. RK4 calls the model,
and the model calls every registered component at each stage.

## Optional local-flow model

The uploaded code accepted three mutable velocity corrections. The Stage 2
replacement uses one summed velocity increment from `ILocalFlowField` because
all three terms entered the same local-velocity equation.

```cpp
class TailFlow final : public navion::ILocalFlowField
{
public:
    navion::Vec3 velocityIncrementBodyMps(
        const navion::EvaluationContext& context,
        const navion::Vec3& positionFromCgBodyM
    ) const override
    {
        return calculateDownwash(context, positionFromCgBodyM)
             + calculatePropwash(context, positionFromCgBodyM)
             + calculateLocalGust(context, positionFromCgBodyM);
    }
};

auto tailFlow = std::make_shared<TailFlow>();
auto horizontalTail =
    std::make_unique<navion::HorizontalStabilizer>(horizontalConfig, tailFlow);
```

The provider is evaluated from every RK4 stage context. It must return a finite
BODY-axis velocity increment in metres per second. With no provider, the
increment is exactly zero.

## Moment ownership check

Each stabilizer calls `makeBodyLoadAtPoint()` before returning. Therefore its
moment already equals:

```text
M_CG_BODY = M_AC_BODY + r_CG_to_AC_BODY × F_BODY
```

Do not add `M_AC` or `r × F` in `NavionModel`, `LoadAccumulator`, or
`RigidBody6DOF`; doing so would double count the stabilizer moment.

## Intentional behavior change in the vertical tail

For positive beta, the model computes negative `CY`. The uploaded code then
multiplied that signed value by a direction that pointed toward BODY `-Y` at
zero beta, reversing the physical sign. Stage 2 uses a direction that points
toward BODY `+Y` at zero beta. Tests now assert:

- positive beta → negative BODY-Y force → positive restoring yaw moment for an
  aft fin;
- positive rudder (trailing edge left) → positive BODY-Y force → negative
  nose-left yaw moment for an aft fin.

The vertical-tail drag direction also uses the complete 3-D local velocity, so
its BODY-Z drag component is no longer discarded when local `w` is nonzero.
