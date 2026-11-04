#!/usr/bin/env python3
"""Offline metrics for an already uniform seven-axis CSV; never controls hardware.

CSV columns: time_s,cmd_j1..cmd_j6,cmd_gripper, optionally complete fb_* and
ref_* groups. JSON explicitly freezes roles, units, period, filter and thresholds.
No interpolation, timestamp rewriting, inferred feedback or physical f_exec.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

AXES = tuple(f"j{i}" for i in range(1, 7))+("gripper",)
UNITS = ("deg",)*6+("m",)
COMMAND_EPSILON = (1e-9,)*6+(1e-12,)
FILTER = {"kind": "binomial", "coefficients": [1, 4, 6, 4, 1], "normalization": 16}
TRIM = 5  # Two FIR neighbours plus three centred numerical derivatives.


def finite_number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    return float(value)


def vector(value, name, *, positive=False):
    if not isinstance(value, list) or len(value) != 7:
        raise ValueError(f"{name} must contain seven numbers")
    return [finite_number(item, name, positive=positive) for item in value]


def validate_config(config):
    required = {"schema", "input_role", "units", "sample_period_s", "uniform_tolerance_s",
                "filter", "velocity_jump_threshold", "settling"}
    if not isinstance(config, dict) or set(config) != required:
        raise ValueError("configuration keys must match the documented schema exactly")
    if config["schema"] != "robot_rt.smoothness.v1":
        raise ValueError("unsupported smoothness configuration schema")
    if config["input_role"] not in ("offline_replay", "recorded_commands", "recorded_commands_feedback"):
        raise ValueError("input_role must explicitly identify offline or recorded command data")
    if config["units"] != list(UNITS):
        raise ValueError("units must be six deg axes followed by one m axis")
    dt = finite_number(config["sample_period_s"], "sample_period_s", positive=True)
    tolerance = finite_number(config["uniform_tolerance_s"], "uniform_tolerance_s")
    if not 0 <= tolerance <= dt*0.001:
        raise ValueError("uniform_tolerance_s must be between zero and 0.001 sample periods")
    if config["filter"] != FILTER:
        raise ValueError("v1 requires exactly the five-tap binomial [1,4,6,4,1]/16 filter")
    vector(config["velocity_jump_threshold"], "velocity_jump_threshold", positive=True)
    settling = config["settling"]
    if settling is not None:
        keys = {"goal", "window_start_s", "window_end_s", "stable_s", "joint_tolerance_deg",
                "gripper_tolerance_m", "command_epsilon"}
        if not isinstance(settling, dict) or set(settling) != keys:
            raise ValueError("settling keys must match the documented schema exactly, or be null")
        vector(settling["goal"], "settling.goal")
        start = finite_number(settling["window_start_s"], "window_start_s")
        end = finite_number(settling["window_end_s"], "window_end_s")
        if start < 0 or end <= start:
            raise ValueError("settling window must be a positive interval relative to the first CSV time")
        if settling["stable_s"] != 0.5 or settling["joint_tolerance_deg"] != 0.5:
            raise ValueError("v1 fixes stable_s=0.5 and joint_tolerance_deg=0.5")
        finite_number(settling["stable_s"], "stable_s", positive=True)
        finite_number(settling["joint_tolerance_deg"], "joint_tolerance_deg", positive=True)
        finite_number(settling["gripper_tolerance_m"], "gripper_tolerance_m", positive=True)
        if vector(settling["command_epsilon"], "command_epsilon") != list(COMMAND_EPSILON):
            raise ValueError("command_epsilon is fixed at 1e-9 deg / 1e-12 m in v1")
    return config


def load_csv(path):
    import numpy as np
    required = ["time_s"]+[f"cmd_{name}" for name in AXES]
    groups = {prefix: [f"{prefix}_{name}" for name in AXES] for prefix in ("fb", "ref")}
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)) or not set(required) <= set(fields):
            raise ValueError("CSV needs unique time_s and all seven cmd_* columns")
        allowed = set(required+groups["fb"]+groups["ref"])
        if not set(fields) <= allowed:
            raise ValueError("CSV contains undocumented columns")
        for prefix, names in groups.items():
            if set(fields) & set(names) and not set(names) <= set(fields):
                raise ValueError(f"optional {prefix}_* columns must be a complete seven-axis group")
        rows = []
        for line, row in enumerate(reader, start=2):
            try:
                if None in row or any(row[name] is None for name in fields):
                    raise ValueError("row width differs from header")
                values = [float(row[name]) for name in fields]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("nonfinite value")
                rows.append(values)
            except (ValueError, TypeError) as exc:
                raise ValueError(f"CSV line {line}: invalid numeric row ({exc})") from exc
    if not rows:
        raise ValueError("CSV has no numeric samples")
    data = np.asarray(rows, dtype=float)
    result = {"time": data[:, fields.index("time_s")],
              "command": data[:, [fields.index(name) for name in required[1:]]]}
    for prefix, key in (("fb", "feedback"), ("ref", "reference")):
        result[key] = data[:, [fields.index(name) for name in groups[prefix]]] if set(groups[prefix]) <= set(fields) else None
    return result


def settling_metrics(time, command, feedback, config):
    import numpy as np
    unavailable = lambda reason: [{"status": "unavailable", "reason": reason,
        "arrival_time_s": None, "entry_time_s": None, "confirmed_time_s": None} for _ in AXES]
    setting = config["settling"]
    if setting is None:
        return unavailable("no_predeclared_fixed_goal_and_observation_window")
    if feedback is None:
        return unavailable("feedback_columns_absent")
    relative = time-time[0]
    start, end = setting["window_start_s"], setting["window_end_s"]
    epsilon = config["uniform_tolerance_s"]
    if end > relative[-1]+epsilon or start > relative[-1]+epsilon:
        return unavailable("incomplete_declared_observation_window")
    indices = np.flatnonzero((relative >= start-epsilon) & (relative <= end+epsilon))
    if len(indices) < 2 or relative[indices[-1]]-relative[indices[0]] < 0.5-epsilon:
        return unavailable("observation_window_has_less_than_0.5_s_of_samples")
    goal = np.asarray(setting["goal"])
    if np.any(abs(command[indices]-goal) > np.asarray(COMMAND_EPSILON)):
        return unavailable("command_not_constant_at_declared_goal_in_window")
    tolerances = [0.5]*6+[setting["gripper_tolerance_m"]]
    results = []
    for axis, tolerance in enumerate(tolerances):
        run_start = None
        found = None
        for index in indices:
            if abs(feedback[index, axis]-goal[axis]) <= tolerance:
                if run_start is None:
                    run_start = index
                if relative[index]-relative[run_start] >= 0.5-epsilon:
                    found = {"status": "sampled_stable_interval_found", "reason": None,
                        "arrival_time_s": float(relative[run_start]-start),
                        "entry_time_s": float(relative[run_start]), "confirmed_time_s": float(relative[index])}
                    break
            else:
                run_start = None
        results.append(found or {"status": "not_settled_in_observation_window",
            "reason": "no_continuous_0.5_s_sampled_in_tolerance_interval",
            "arrival_time_s": None, "entry_time_s": None, "confirmed_time_s": None})
    return results


def analyze(data, config):
    import numpy as np
    validate_config(config)
    time, command = np.asarray(data["time"], dtype=float), np.asarray(data["command"], dtype=float)
    feedback, reference = data.get("feedback"), data.get("reference")
    if time.ndim != 1 or command.shape != (len(time), 7):
        raise ValueError("time must be N and command must be Nx7")
    for name, values in (("time", time), ("command", command), ("feedback", feedback), ("reference", reference)):
        if values is not None and (not np.all(np.isfinite(values)) or name not in ("time", "command") and np.shape(values) != command.shape):
            raise ValueError(f"{name} has invalid dimensions or nonfinite values")
    if len(time) < 2*TRIM+3:
        raise ValueError("at least 13 samples are required for the filter and three derivatives")
    dt = config["sample_period_s"]
    differences = np.diff(time)
    if np.any(differences <= 0) or np.any(abs(differences-dt) > config["uniform_tolerance_s"]):
        raise ValueError("CSV time must be strictly increasing and uniform at the declared sample period; no resampling is performed")
    if feedback is not None and config["input_role"] != "recorded_commands_feedback":
        raise ValueError("feedback requires explicit input_role=recorded_commands_feedback; offline/reference arrays are not feedback")
    feedback = None if feedback is None else np.asarray(feedback, dtype=float)
    reference = None if reference is None else np.asarray(reference, dtype=float)
    kernel = np.asarray(FILTER["coefficients"], dtype=float)/FILTER["normalization"]
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        filtered = np.column_stack([np.convolve(command[:, axis], kernel, mode="same") for axis in range(7)])
        velocity = np.gradient(filtered, dt, axis=0)
        acceleration = np.gradient(velocity, dt, axis=0)
        jerk = np.gradient(acceleration, dt, axis=0)
    if any(not np.all(np.isfinite(values)) for values in (filtered, velocity, acceleration, jerk)):
        raise ValueError("nonfinite filtered/derived velocity, acceleration or jerk; finite input exceeded numerical range")
    keep = slice(TRIM, -TRIM)
    valid_velocity, valid_jerk = velocity[keep], jerk[keep]
    with np.errstate(over="ignore", invalid="ignore"):
        velocity_delta = np.diff(valid_velocity, axis=0)
        errors = [None if values is None else abs(values[keep]-command[keep]) for values in (feedback, reference)]
    if not np.all(np.isfinite(velocity_delta)) or any(values is not None and not np.all(np.isfinite(values)) for values in errors):
        raise ValueError("nonfinite velocity difference or position error; finite input exceeded numerical range")
    jumps = np.sum(abs(velocity_delta) > np.asarray(config["velocity_jump_threshold"]), axis=0)
    tracking, deviation = [None if values is None else np.quantile(values, 0.99, axis=0, method="linear") for values in errors]
    # Scaling keeps squares <=1 even when finite jerk values exceed sqrt(DBL_MAX).
    jerk_scale = np.max(abs(valid_jerk), axis=0)
    jerk_rms = np.sqrt(np.mean((valid_jerk/np.where(jerk_scale > 0, jerk_scale, 1))**2, axis=0))*jerk_scale
    arrival = settling_metrics(time, command, feedback, config)
    axes = []
    for axis, (name, unit) in enumerate(zip(AXES, UNITS)):
        axes.append({"name": name, "unit": unit, "jerk_unit": unit+"/s^3", "velocity_unit": unit+"/s",
            "jerk_rms": float(jerk_rms[axis]),
            "velocity_jump_count": int(jumps[axis]), "velocity_jump_threshold": config["velocity_jump_threshold"][axis],
            "tracking_error_p99": None if tracking is None else float(tracking[axis]),
            "tracking_error_reason": "feedback_columns_absent" if tracking is None else None,
            "reference_deviation_p99": None if deviation is None else float(deviation[axis]),
            "reference_deviation_reason": "reference_columns_absent" if deviation is None else None,
            "settling": arrival[axis]})
    return {"schema": "robot_rt.smoothness_result.v1", "provenance": "offline_analysis",
        "hardware_unverified": True, "physical_f_exec_hz": None, "input_role": config["input_role"],
        "numpy_version": np.__version__, "python_version": sys.version.split()[0],
        "sample_count": len(time), "sample_period_s": dt, "duration_s": float(time[-1]-time[0]),
        "analysis_sample_count": len(valid_jerk), "analysis_duration_s": float(time[-TRIM-1]-time[TRIM]),
        "velocity_adjacent_pair_count": len(valid_velocity)-1, "trim_samples_per_end": TRIM,
        "configuration": config, "axes": axes,
        "limits": ["No hardware connection, physical smoothness acceptance or physical f_exec is inferred.",
            "Feedback roles are supplied by the input author; the tool cannot authenticate a measurement.",
            "Tracking P99 and reference deviation use raw same-time values on the common trimmed window; no time-shift fitting.",
            "Settling uses raw feedback within the explicit constant-command window; sampled persistence does not prove between-sample behaviour.",
            "Only runs with identical configuration SHA256 and comparable input roles/timing boundaries may be compared.",
            "No resampling, interpolation, timestamp rewriting or IMU metric is performed."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New JSON file; existing evidence is never overwritten")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError("output exists; choose a new evidence path")
        config_bytes = args.config.read_bytes()
        config = json.loads(config_bytes)
        result = analyze(load_csv(args.csv), config)
        result["config_sha256"] = hashlib.sha256(config_bytes).hexdigest()
        result["input_csv_sha256"] = hashlib.sha256(args.csv.read_bytes()).hexdigest()
        result["implementation_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        result["invocation"] = [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
        print(json.dumps({"output": str(args.output.resolve()), "provenance": "offline_analysis",
            "hardware_unverified": True, "physical_f_exec_hz": None, "config_sha256": result["config_sha256"]}))
        return 0
    except (ValueError, TypeError, KeyError, OSError) as exc:
        parser.exit(2, f"offline smoothness: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
