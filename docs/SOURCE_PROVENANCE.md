# Source provenance for Stage 5

Stage 5 was produced from the Stage 4 release plus the two latest component
archives uploaded by the user.

| Input | SHA-256 |
| --- | --- |
| `Navion_6DOF_Stage4_VS_HS_Propeller_LandingGear_PGS_Integrated.zip` | `551377a0796c1b994f587d1c76d98c9d90885b2a311805ac43f8be55337f8edc` |
| `Fuselage(2).zip` | `885d3f2d88f17fb7ae554ce1f1a6031eacede7fa431e2f25f3ad15d2be45d3a2` |
| `MainWing(2).zip` | `84ed16637719cd7ddbd192499194998c9f88a7c6c547b26951074505d5328a0b` |

Relevant component-member hashes before integration:

| Original member | SHA-256 |
| --- | --- |
| `t6c_fuselage.hpp` | `784dec22a10ddf5af14cac98bfa2c6e90084fe735291148c30045e2a10d64762` |
| `t6c_fuselage.cpp` | `6e54a89c476685efc5f75341dff87ac2efe931d12389ca7e1504fc1ac4f5df12` |
| `t6c_fuselage_params.yaml` | `da08bd93fd05ca869727cc7fefeef2c8e65eecf14c55ac4363ef8801291fef22` |
| `VATC_MainWing.hpp` | `35c7741bf58146f057c3936347136ac7186b7ed3072b04ee35e8559b3504c437` |
| `VATC_MainWing.cpp` | `2a22fbf2c424cf298b9d11369ba7db7fdb26793f00024f50e7ceff0cfe0d6571` |

`Fuselage(2).zip` is byte-identical to the earlier `Fuselage(1).zip` supplied
in this task. `MainWing(2).zip` has different archive packaging, but its HPP
and CPP hashes are identical to the previously supplied MainWing files.

## Integration and Stage 5.1 organization

MainWing implementation equations were not altered. Files were moved under
the common include/source tree and source-compatible `MainWing`/
`MainWingConfig` aliases were added.

The Fuselage scalar kernel is retained as the public, independently testable
`FuselageAerodynamics` type. `FuselageComponent` resolves the caller-owned
frame/reference operations described by the original documentation. It does
not remove any kernel output or tuning field. Compatibility aliases preserve
the uploaded scalar API for the reference executable.

The original Fuselage YAML, formula PDF/TeX, PUML files and regression/sweep
driver are retained in `reference_data/t6c/`, `docs/fuselage/original/` and
`examples/`. T-6C YAML is provenance only and is not runtime configuration.
