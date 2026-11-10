"""Offline seven-axis measured-demonstration timing plans.

No device, subprocess, network, control, restoration or execution interfaces.
The returned object is deliberately blocked from physical replay.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


JOINTS = [f"J{i}" for i in range(7)]
UNITS = ["degree"] * 6 + ["uncalibrated_sdk_raw"]
LIMITS = [[-135.0, 135.0], [-90.0, 90.0], [-90.0, 90.0],
          [-135.0, 135.0], [-90.0, 90.0], [-135.0, 135.0], [0.0, 50.0]]
ARM_ACCELERATION_DEG_S2 = 1.0
MAX_INPUT_GAP_S = 0.5
MIN_SEGMENT_S = 0.1
QUINTIC_MAX_FIRST_DERIVATIVE = 1.875
QUINTIC_MAX_SECOND_DERIVATIVE = 10.0 / math.sqrt(3.0)
START_LABELS = {"开始演示", "demonstration_started", "manual_demo_started"}


class PlanValidationError(ValueError):
    """Stable machine-readable code plus a human-readable diagnostic."""

    def __init__(self, code: str, message: str, **context):
        self.code = code
        self.message = message
        self.context = context
        super().__init__(f"{code}: {message}")


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _load(path, hashes, *, optional=False, jsonl=False):
    if optional and not path.exists():
        return [] if jsonl else {}
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise PlanValidationError("SOURCE_IO", "Unable to read a required source file", path=str(path)) from exc
    hashes[path.name] = {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()}
    try:
        text = raw.decode("utf-8")
        if jsonl:
            rows = []
            for number, line in enumerate(text.splitlines(), 1):
                if not line.strip():
                    continue
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise PlanValidationError("INVALID_RECORD", "JSONL record must be an object", path=str(path), line=number)
                rows.append((number, value))
            return rows
        value = json.loads(text)
        if not isinstance(value, dict):
            raise PlanValidationError("INVALID_RECORD", "JSON document must be an object", path=str(path))
        return value
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise PlanValidationError("INVALID_JSON", "Source cannot be decoded as JSON", path=str(path)) from exc


def canonical_plan_sha256(plan: dict) -> str:
    """Hash sorted compact JSON with the top-level plan_sha256 field excluded."""
    payload = {key: value for key, value in plan.items() if key != "plan_sha256"}
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def prepare_session(session_path: Path, speed_deg_s: float = 0.5) -> dict:
    """Return a blocked offline plan; never writes source or output files.

    Speed is a requested arm timing parameter, not a verified controller rating.
    Values above 0.5 are permitted here. Arm acceleration stays at 1 deg/s².
    All seven axes share each segment's normalized quintic time, while J6's
    raw-unit speed and acceleration are reported without invented thresholds.
    """
    if not _finite(speed_deg_s) or speed_deg_s <= 0:
        raise PlanValidationError("INVALID_SPEED", "Arm speed must be finite and strictly positive")
    speed_deg_s = float(speed_deg_s)
    session = Path(session_path).resolve()
    hashes = {}
    if not (session / "summary.json").is_file():
        raise PlanValidationError("SESSION_NOT_ENDED", "summary.json is required before preparing a replay plan")
    summary = _load(session / "summary.json", hashes)
    if summary.get("state") not in {"stopped", "error"}:
        raise PlanValidationError("SESSION_NOT_ENDED", "The recorder summary must confirm that recording has ended", state=summary.get("state"))
    if summary.get("subscriber_pipe_drained") is False:
        raise PlanValidationError("INCOMPLETE_STOP_DRAIN", "The recorder did not finish draining received samples")
    metadata = _load(session / "metadata.json", hashes, optional=True)
    events = _load(session / "events.jsonl", hashes, optional=True, jsonl=True)
    rows = _load(session / "joint_samples.jsonl", hashes, jsonl=True)
    if "joint_order" in metadata and metadata["joint_order"] != JOINTS:
        raise PlanValidationError("JOINT_ORDER_MISMATCH", "Expected SDK axes J0 through J6")
    if "joint_units" in metadata:
        units = metadata["joint_units"]
        if not isinstance(units, list) or len(units) != 7 or any(u not in {"deg", "degree"} for u in units[:6]) or units[6] not in {"sdk_raw", "uncalibrated_sdk_raw"}:
            raise PlanValidationError("UNITS_MISMATCH", "Expected six degree axes and an SDK-raw gripper axis")

    marker = None
    marker_line = None
    for line, event in events:
        if event.get("label") in START_LABELS:
            if not _finite(event.get("elapsed_s")):
                raise PlanValidationError("INVALID_MARKER_TIME", "The first start marker has an invalid elapsed timestamp", line=line)
            marker, marker_line = event, line
            break
    start_elapsed = marker["elapsed_s"] if marker else None
    end_marker = next((event for _, event in events if event.get("label") == "grasp_recording_ended" and _finite(event.get("elapsed_s")) and (start_elapsed is None or event["elapsed_s"] >= start_elapsed)), None)
    points = []
    omitted_preparation_rows = 0
    cutoff = summary.get("stop_cutoff_monotonic_s")
    for line, row in rows:
        elapsed = row.get("elapsed_s")
        if not _finite(elapsed):
            raise PlanValidationError("NONFINITE_TIME", "Cannot place a record with an invalid elapsed time", line=line)
        if start_elapsed is not None and elapsed < start_elapsed:
            omitted_preparation_rows += 1
            continue
        stamp = row.get("dds_received_monotonic_s")
        if not _finite(stamp):
            raise PlanValidationError("NONFINITE_TIME", "DDS receipt time must be finite", line=line)
        q = row.get("q")
        if not isinstance(q, list) or len(q) != 7 or not all(_finite(value) for value in q):
            raise PlanValidationError("INVALID_Q", "Each selected position must contain seven finite numbers", line=line)
        for joint, (value, bounds) in enumerate(zip(q, LIMITS)):
            if not bounds[0] <= value <= bounds[1]:
                raise PlanValidationError("JOINT_LIMIT", "Selected position exceeds the host/onboard intersection; no clipping or wrapping is allowed", line=line, joint=JOINTS[joint], value=value, allowed=bounds)
        if _finite(cutoff) and stamp > cutoff:
            raise PlanValidationError("AFTER_STOP_CUTOFF", "A selected sample follows the recorded stop cutoff", line=line)
        if points:
            dt = stamp - points[-1]["received_monotonic_s"]
            elapsed_dt = elapsed - points[-1]["elapsed_s"]
            if dt <= 0 or elapsed_dt <= 0:
                raise PlanValidationError("NONMONOTONIC_TIME", "Selected timestamps must strictly increase in recorded order", line=line)
            if dt > MAX_INPUT_GAP_S or row.get("gap_before") is True:
                raise PlanValidationError("FEEDBACK_GAP", "A selected interval exceeds 0.5 s or is explicitly marked as a gap", line=line, gap_s=dt)
            if abs(dt - elapsed_dt) > 0.001:
                raise PlanValidationError("TIME_BASIS_MISMATCH", "DDS and elapsed intervals differ by more than 1 ms", line=line)
        points.append({"source_line": line, "sample": row.get("sample"),
                       "elapsed_s": elapsed, "received_monotonic_s": stamp, "q": list(q)})
    if len(points) < 2:
        raise PlanValidationError("INSUFFICIENT_POINTS", "At least two valid selected samples are required", points=len(points))

    segments = []
    cursor = 0.0
    peak_velocity = [0.0] * 7
    peak_acceleration = [0.0] * 7
    for index, (first, second) in enumerate(zip(points, points[1:])):
        delta = [b-a for a, b in zip(first["q"], second["q"])]
        max_arm_delta = max(abs(value) for value in delta[:6])
        dt = second["received_monotonic_s"] - first["received_monotonic_s"]
        duration = max(dt, QUINTIC_MAX_FIRST_DERIVATIVE * max_arm_delta / speed_deg_s,
                       math.sqrt(QUINTIC_MAX_SECOND_DERIVATIVE * max_arm_delta / ARM_ACCELERATION_DEG_S2), MIN_SEGMENT_S)
        if not math.isfinite(duration) or not math.isfinite(cursor + duration):
            raise PlanValidationError("TIMING_OVERFLOW", "Requested speed creates a nonfinite segment duration", segment=index)
        velocity = [QUINTIC_MAX_FIRST_DERIVATIVE * abs(value) / duration for value in delta]
        acceleration = [QUINTIC_MAX_SECOND_DERIVATIVE * (abs(value) / duration) / duration for value in delta]
        peak_velocity = [max(old, new) for old, new in zip(peak_velocity, velocity)]
        peak_acceleration = [max(old, new) for old, new in zip(peak_acceleration, acceleration)]
        segments.append({"index": index, "q0": first["q"], "q1": second["q"], "T": duration,
                         "plan_t0": cursor, "plan_t1": cursor+duration,
                         "original_dt": dt, "original_t0": first["elapsed_s"], "original_t1": second["elapsed_s"],
                         "source_lines": [first["source_line"], second["source_line"]],
                         "maximum_absolute_velocity": velocity,
                         "maximum_absolute_acceleration": acceleration})
        cursor += duration

    plan = {
        "schema": "d1-offline-seven-axis-demo-plan-v1", "session": str(session),
        "joint_order": JOINTS, "joint_units": UNITS,
        "limits": LIMITS,
        "limits_basis": "Requested intersection of existing host and onboard limits; J6 is 0..50 SDK raw, not degrees.",
        "selection": {"mode": "explicit_start_marker" if marker else "whole_session_no_start_marker",
                      "start_marker": marker, "start_marker_event_line": marker_line,
                      "omitted_preparation_rows": omitted_preparation_rows,
                      "automatic_static_trimming": False},
        "original_points": points, "segments": segments,
        "point_count": len(points), "segment_count": len(segments),
        "source_duration": points[-1]["received_monotonic_s"]-points[0]["received_monotonic_s"],
        "source_marker_duration": end_marker["elapsed_s"]-start_elapsed if end_marker and start_elapsed is not None else None,
        "duration": cursor, "speed_deg_s": speed_deg_s,
        "acceleration_deg_s2": ARM_ACCELERATION_DEG_S2,
        "start_q": points[0]["q"], "end_q": points[-1]["q"],
        "interpolation": {"type": "per_segment_synchronized_quintic", "u": "(t-plan_t0)/T, 0<=u<=1",
                          "formula": "q(t)=q0+(q1-q0)*(10*u^3-15*u^4+6*u^5)",
                          "all_seven_axes_share_u": True,
                          "segment_endpoint_velocity": 0.0, "segment_endpoint_acceleration": 0.0,
                          "minimum_segment_s": MIN_SEGMENT_S,
                          "original_positions_modified": False, "angle_unwrap": False, "clipping": False,
                          "note": "Stops at every recorded waypoint; preserves waypoint geometry but changes timing. This may be slow and is not a verified physical replay."},
        "predicted_peak_absolute_velocity": peak_velocity,
        "predicted_peak_absolute_acceleration": peak_acceleration,
        "arm_timing_constraints_satisfied": all(v <= speed_deg_s*(1+1e-12) for v in peak_velocity[:6]) and all(a <= ARM_ACCELERATION_DEG_S2*(1+1e-12) for a in peak_acceleration[:6]),
        "gripper_timing": {"position_unit": "uncalibrated_sdk_raw", "velocity_unit": "sdk_raw/s", "acceleration_unit": "sdk_raw/s^2",
                           "predicted_peak_absolute_velocity": peak_velocity[6],
                           "predicted_peak_absolute_acceleration": peak_acceleration[6],
                           "verified_velocity_limit": None, "verified_acceleration_limit": None, "verified": False},
        "blockers": ["controller_hold_not_verified", "gripper_timing_not_verified", "physical_path_not_reviewed"],
        "actual_motion_replay_ready": False, "hardware_accessed": False, "robot_commands_sent": 0,
        "source_files_modified": False, "sources": hashes,
    }
    plan["plan_sha256"] = canonical_plan_sha256(plan)
    return plan
