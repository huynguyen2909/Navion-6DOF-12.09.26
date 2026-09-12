#!/usr/bin/env python3
"""Plot the Stage 5.3 trim/hold/RPM-ramp/30-second timeline."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COMPONENTS = (
    ("main_wing", "Main Wing"),
    ("horizontal_stabilizer", "Horizontal Stabilizer"),
    ("vertical_stabilizer", "Vertical Stabilizer"),
    ("fuselage", "Fuselage"),
    ("propeller", "Propeller"),
    ("landing_gear", "Landing Gear + PGS"),
)


def read_csv(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        columns = {name: [] for name in reader.fieldnames}
        for row_index, row in enumerate(reader, start=2):
            for name in reader.fieldnames:
                try:
                    columns[name].append(float(row[name]))
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f"Invalid numeric value at row {row_index}, column {name}"
                    ) from error
    if not columns or not columns[reader.fieldnames[0]]:
        raise ValueError(f"CSV has no data rows: {path}")
    return columns


def require_columns(data: dict[str, list[float]], names: list[str]) -> None:
    missing = [name for name in names if name not in data]
    if missing:
        raise ValueError("CSV is missing columns: " + ", ".join(missing))


def mark_scenario_phases(axes, data: dict[str, list[float]]) -> None:
    """Shade ground trim, zero-RPM hold, RPM ramp and full-RPM phases."""
    require_columns(data, ["time_s", "phase_id"])
    time = data["time_s"]
    phase = [int(round(value)) for value in data["phase_id"]]

    starts: dict[int, float] = {}
    for phase_id, sample_time in zip(phase, time):
        starts.setdefault(phase_id, sample_time)

    trim_start = min(time)
    trim_samples = [sample for sample, phase_id in zip(time, phase) if phase_id == 0]
    trim_end = max(trim_samples) if trim_samples else trim_start
    release = starts.get(2, trim_end)
    full_rpm = starts.get(3, max(time))
    end = max(time)
    regions = (
        (trim_start, trim_end, "#bdbdbd"),
        (trim_end, release, "#9ecae1"),
        (release, full_rpm, "#fdae6b"),
        (full_rpm, end, "#a1d99b"),
    )

    axes_list = list(axes)
    for axis in axes_list:
        for start, stop, color in regions:
            if stop > start:
                axis.axvspan(start, stop, color=color, alpha=0.11, zorder=-10)
        for boundary in (trim_end, release, full_rpm):
            axis.axvline(boundary, color="0.35", linewidth=0.8, alpha=0.7)

    if axes_list:
        top = axes_list[0]
        for x, label in (
            (trim_end, "trim complete"),
            (release, "brake release / RPM ramp"),
            (full_rpm, "2300 RPM"),
        ):
            top.annotate(
                label,
                xy=(x, 1.0),
                xycoords=("data", "axes fraction"),
                xytext=(3, -3),
                textcoords="offset points",
                rotation=90,
                va="top",
                ha="left",
                fontsize=8,
                color="0.25",
            )


def finish_figure(figure, axes, data, title: str, output: Path) -> Path:
    for axis in axes:
        axis.grid(True, alpha=0.3)
    mark_scenario_phases(axes, data)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output


def save_states(data: dict[str, list[float]], output_dir: Path) -> Path:
    required = [
        "time_s",
        "u_mps",
        "v_mps",
        "w_mps",
        "airspeed_mps",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "p_radps",
        "q_radps",
        "r_radps",
        "position_north_m",
        "position_east_m",
        "position_down_m",
        "height_above_trim_m",
        "velocity_down_ned_mps",
        "airborne_candidate",
    ]
    require_columns(data, required)
    time = data["time_s"]
    rad_to_deg = 180.0 / math.pi

    figure, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    axes[0, 0].plot(time, data["u_mps"], label="u")
    axes[0, 0].plot(time, data["v_mps"], label="v")
    axes[0, 0].plot(time, data["w_mps"], label="w")
    axes[0, 0].plot(time, data["airspeed_mps"], "k--", label="Vair")
    axes[0, 0].set_ylabel("Velocity [m/s]")
    axes[0, 0].legend(ncol=2)

    for column, label in (
        ("roll_rad", "roll phi"),
        ("pitch_rad", "pitch theta"),
        ("yaw_rad", "yaw psi"),
    ):
        axes[0, 1].plot(
            time, [value * rad_to_deg for value in data[column]], label=label
        )
    axes[0, 1].set_ylabel("Euler angle [deg]")
    axes[0, 1].legend()

    for column, label in (
        ("p_radps", "p"),
        ("q_radps", "q"),
        ("r_radps", "r"),
    ):
        axes[1, 0].plot(
            time, [value * rad_to_deg for value in data[column]], label=label
        )
    axes[1, 0].set_ylabel("Body rate [deg/s]")
    axes[1, 0].legend()

    axes[1, 1].plot(time, data["position_north_m"], label="North")
    axes[1, 1].plot(time, data["position_east_m"], label="East")
    axes[1, 1].set_ylabel("Horizontal NED position [m]")
    axes[1, 1].legend()

    axes[2, 0].plot(time, data["position_down_m"], label="Down coordinate")
    axes[2, 0].plot(
        time, data["height_above_trim_m"], "--", label="Height above trim"
    )
    axes[2, 0].set_xlabel("Timeline time [s]")
    axes[2, 0].set_ylabel("Vertical position [m]")
    axes[2, 0].legend()

    axes[2, 1].plot(
        time, data["velocity_down_ned_mps"], label="NED Down velocity"
    )
    airborne_axis = axes[2, 1].twinx()
    airborne_axis.step(
        time,
        data["airborne_candidate"],
        "--",
        where="post",
        label="Airborne candidate",
    )
    axes[2, 1].set_xlabel("Timeline time [s]")
    axes[2, 1].set_ylabel("Down velocity [m/s]")
    airborne_axis.set_ylabel("Candidate flag")
    lines = axes[2, 1].get_lines() + airborne_axis.get_lines()
    axes[2, 1].legend(lines, [line.get_label() for line in lines])

    return finish_figure(
        figure,
        axes.flat,
        data,
        "Stage 5.3 ground trim, RPM ramp and 30-second rigid-body states",
        output_dir / "takeoff_states.png",
    )


def save_total_loads(data: dict[str, list[float]], output_dir: Path) -> Path:
    required = [
        "time_s",
        "component_fx_body_n",
        "component_fy_body_n",
        "component_fz_body_n",
        "gravity_fx_body_n",
        "gravity_fy_body_n",
        "gravity_fz_body_n",
        "net_fx_body_n",
        "net_fy_body_n",
        "net_fz_body_n",
        "net_l_body_nm",
        "net_m_body_nm",
        "net_n_body_nm",
    ]
    require_columns(data, required)
    time = data["time_s"]

    figure, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    for axis, suffix, label in zip(
        axes[0], ("x", "y", "z"), ("Fx", "Fy", "Fz")
    ):
        axis.plot(time, data[f"net_f{suffix}_body_n"], label="net")
        axis.plot(
            time,
            data[f"component_f{suffix}_body_n"],
            "--",
            label="components",
        )
        axis.plot(
            time,
            data[f"gravity_f{suffix}_body_n"],
            ":",
            label="gravity",
        )
        axis.set_title(label)
        axis.legend()

    for axis, column, label in zip(
        axes[1],
        ("net_l_body_nm", "net_m_body_nm", "net_n_body_nm"),
        ("L", "M", "N"),
    ):
        axis.plot(time, data[column], label=label)
        axis.set_title(label)
        axis.set_xlabel("Timeline time [s]")
        axis.legend()

    axes[0, 0].set_ylabel("BODY force [N]")
    axes[1, 0].set_ylabel("Moment about CG [N m]")
    return finish_figure(
        figure,
        axes.flat,
        data,
        "Stage 5.3 total aircraft BODY loads across all phases",
        output_dir / "takeoff_total_loads.png",
    )


def save_component_loads(data: dict[str, list[float]], output_dir: Path) -> Path:
    required = ["time_s"]
    for prefix, _ in COMPONENTS:
        required.extend(
            [
                f"{prefix}_fx_body_n",
                f"{prefix}_fy_body_n",
                f"{prefix}_fz_body_n",
                f"{prefix}_l_body_nm",
                f"{prefix}_m_body_nm",
                f"{prefix}_n_body_nm",
            ]
        )
    require_columns(data, required)
    time = data["time_s"]
    columns = (
        ("fx_body_n", "Fx", "Force [N]"),
        ("fy_body_n", "Fy", "Force [N]"),
        ("fz_body_n", "Fz", "Force [N]"),
        ("l_body_nm", "L", "Moment about CG [N m]"),
        ("m_body_nm", "M", "Moment about CG [N m]"),
        ("n_body_nm", "N", "Moment about CG [N m]"),
    )

    figure, axes = plt.subplots(2, 3, figsize=(17, 9), sharex=True)
    legend_handles = []
    legend_labels = []
    for axis, (suffix, title, ylabel) in zip(axes.flat, columns):
        for prefix, label in COMPONENTS:
            line = axis.plot(time, data[f"{prefix}_{suffix}"], label=label)[0]
            if len(legend_handles) < len(COMPONENTS):
                legend_handles.append(line)
                legend_labels.append(label)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        if suffix in ("l_body_nm", "m_body_nm", "n_body_nm"):
            axis.set_xlabel("Timeline time [s]")

    figure.legend(
        legend_handles,
        legend_labels,
        loc="lower center",
        ncol=3,
        bbox_to_anchor=(0.5, 0.005),
    )
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
    mark_scenario_phases(axes.flat, data)
    figure.suptitle("Stage 5.3 BODY load contribution of each component")
    figure.tight_layout(rect=(0.0, 0.06, 1.0, 0.97))
    output = output_dir / "takeoff_component_loads.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output


def save_ground(data: dict[str, list[float]], output_dir: Path) -> Path:
    required = [
        "time_s",
        "ground_normal_fz_body_n",
        "ground_friction_fx_body_n",
        "ground_friction_fy_body_n",
        "ground_contact_count",
        "pgs_iterations",
        "propeller_rpm",
        "brake_left",
        "brake_right",
        "airborne_candidate",
    ]
    require_columns(data, required)
    time = data["time_s"]

    figure, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(time, data["ground_normal_fz_body_n"], label="Normal Fz")
    axes[0].plot(time, data["ground_friction_fx_body_n"], label="Friction Fx")
    axes[0].plot(time, data["ground_friction_fy_body_n"], label="Friction Fy")
    axes[0].set_ylabel("Ground load [N]")
    axes[0].legend()

    axes[1].step(
        time, data["ground_contact_count"], where="post", label="Contacts"
    )
    axes[1].plot(time, data["pgs_iterations"], label="PGS iterations")
    axes[1].step(
        time,
        data["airborne_candidate"],
        "--",
        where="post",
        label="Airborne candidate",
    )
    axes[1].set_ylabel("Count / flag")
    axes[1].legend()

    axes[2].plot(time, data["propeller_rpm"], label="Propeller RPM")
    axes[2].set_xlabel("Timeline time [s]")
    axes[2].set_ylabel("RPM")
    brake_axis = axes[2].twinx()
    brake_axis.step(
        time, data["brake_left"], "--", where="post", label="Brake L"
    )
    brake_axis.step(
        time, data["brake_right"], ":", where="post", label="Brake R"
    )
    brake_axis.set_ylabel("Brake command")
    lines = axes[2].get_lines() + brake_axis.get_lines()
    axes[2].legend(lines, [line.get_label() for line in lines], loc="center right")

    return finish_figure(
        figure,
        axes,
        data,
        "Stage 5.3 landing gear, PGS and prescribed propeller schedule",
        output_dir / "takeoff_ground_contact.png",
    )


def print_liftoff_diagnostic(data: dict[str, list[float]]) -> None:
    require_columns(
        data,
        [
            "time_s",
            "airborne_candidate",
            "position_down_m",
            "height_above_trim_m",
            "velocity_down_ned_mps",
            "ground_contact_count",
        ],
    )
    persistent_start = None
    run_start = None
    for time, candidate in zip(data["time_s"], data["airborne_candidate"]):
        if candidate >= 0.5:
            if run_start is None:
                run_start = time
            if time - run_start >= 0.25:
                persistent_start = run_start
                break
        else:
            run_start = None

    if persistent_start is None:
        print("liftoff diagnostic: no persistent airborne interval found")
    else:
        print(f"liftoff diagnostic: persistent airborne candidate at t={persistent_start:.3f} s")
    print(
        "final vertical diagnostic: "
        f"Down={data['position_down_m'][-1]:.6g} m, "
        f"height_above_trim={data['height_above_trim_m'][-1]:.6g} m, "
        f"V_Down={data['velocity_down_ned_mps'][-1]:.6g} m/s, "
        f"contacts={int(round(data['ground_contact_count'][-1]))}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="CSV written by the C++ scenario")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("plots"),
        help="directory for PNG files (default: plots)",
    )
    args = parser.parse_args()

    data = read_csv(args.csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    outputs = [
        save_states(data, args.output_dir),
        save_total_loads(data, args.output_dir),
        save_component_loads(data, args.output_dir),
        save_ground(data, args.output_dir),
    ]
    print_liftoff_diagnostic(data)
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
