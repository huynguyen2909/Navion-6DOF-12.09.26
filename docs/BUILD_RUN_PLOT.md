# Build, run and plot Stage 5.3

## Required tools

- CMake 3.16 or newer;
- a C++17 compiler (MSVC, MinGW-w64 GCC, GCC or Clang);
- Python 3 and Matplotlib for plotting.

## Windows PowerShell with Ninja/MinGW

Open PowerShell in the extracted project directory:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure

New-Item -ItemType Directory -Force results
.\build\navion_stage5_full_aircraft_takeoff_roll.exe results\takeoff_roll.csv
```

Do not use the Unix form `./build/...` in Windows Command Prompt. In
PowerShell use `.\build\...exe` as shown above.

## Windows with Visual Studio 2022

```powershell
cmake -S . -B build-msvc -G "Visual Studio 17 2022" -A x64
cmake --build build-msvc --config Release
ctest --test-dir build-msvc -C Release --output-on-failure

New-Item -ItemType Directory -Force results
.\build-msvc\Release\navion_stage5_full_aircraft_takeoff_roll.exe `
    results\takeoff_roll.csv
```

Use separate build directories when changing generators.

## Plotting without activating the virtual environment

This form works even when PowerShell blocks `Activate.ps1`:

```powershell
py -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r tools\requirements-plot.txt
& .\.venv\Scripts\python.exe tools\plot_takeoff_results.py `
    results\takeoff_roll.csv --output-dir results\plots
```

The outputs are:

- `takeoff_states.png`: state, attitude/rates, NED position, relative CG
  height, NED Down velocity and airborne-candidate flag;
- `takeoff_total_loads.png`: total non-gravitational component load, gravity
  and net load;
- `takeoff_component_loads.png`: six load components from every aircraft
  subsystem;
- `takeoff_ground_contact.png`: normal/friction forces, contacts, PGS
  iterations, RPM schedule and brake commands.

All plots show the ground-trim snapshot, 0–2 s hold, 2–7 s RPM ramp and
7–30 s full-RPM phases.

## CSV conventions

The CSV is numeric and uses SI, BODY FRD and NED:

| Column/prefix | Meaning |
| --- | --- |
| `phase_id` | 0 trim, 1 hold, 2 RPM ramp, 3 full RPM |
| `propeller_rpm` | prescribed instantaneous propeller speed |
| `component_f*_body_n` | all non-gravitational component forces, including gear |
| `gravity_f*_body_n` | weight transformed to BODY |
| `net_f*_body_n` | force used by translation dynamics |
| `net_l/m/n_body_nm` | total component moment about CG |
| `<component>_f*/l/m/n_*` | one named component's BODY/CG contribution |
| `ground_normal_*` | gear spring/damper normal load |
| `ground_friction_*` | global PGS friction load |
| `height_above_trim_m` | `trimmedDown - currentDown`; positive is upward |
| `velocity_down_ned_mps` | positive downward, negative upward |
| `airborne_candidate` | zero contacts and normal-force norm no greater than 5 N |

Per-component prefixes are `main_wing`, `horizontal_stabilizer`,
`vertical_stabilizer`, `fuselage`, `propeller` and `landing_gear`.

## Linux/macOS equivalent

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
mkdir -p results
./build/navion_stage5_full_aircraft_takeoff_roll results/takeoff_roll.csv
python3 tools/plot_takeoff_results.py results/takeoff_roll.csv --output-dir results/plots
```

## Interpretation limit

The 30-second continuation intentionally goes beyond the former demonstration
`V_rotation` stop, but no elevator rotation or flap deployment is commanded.
The model uses provisional mixed-source parameters and lacks a dynamic engine,
governor and validated full-envelope aerodynamic data. The plots demonstrate
integrated software behaviour; they are not certified Navion performance.
