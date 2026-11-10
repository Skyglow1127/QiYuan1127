#!/usr/bin/env python3
"""Prepare D1-only statistics and real offline smoke inputs without hardware access.

The source consists of two historical lift fragments, not complete grasp episodes.
Only the original training split contributes normalization statistics. Actions are
deduplicated by original command identity, excluding padded target steps.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_SOURCE = Path(
    "/home/aup/Camera/output/act_lift_pilot_20261024/data/examples.jsonl"
)
UNITS = ["degree"] * 6 + ["uncalibrated_sdk_raw"]
AXES = [f"J{i}" for i in range(7)]
STD_FLOOR = 1e-6
LOW_VARIANCE_STD_THRESHOLD = 1e-3
TASK = {
    "text": "Lift the already grasped red cube.",
    "origin": "manually_added_technical_label",
    "source_has_natural_language": False,
    "scope": "Constant lift-stage label for technical smoke only; not original human instructions or language-grounding evidence.",
}


def sha256(path: Path | str) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def finite_vector(value, n: int, context: str) -> None:
    if not isinstance(value, list) or len(value) != n:
        raise ValueError(f"{context}: expected {n} values")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{context}: values must be finite numbers")


def validate_row(row: dict, line: int) -> None:
    name = f"source line {line} ({row.get('id')})"
    if row.get("split") not in {"train", "val"}:
        raise ValueError(f"{name}: unknown split; refusing to silently resplit")
    finite_vector(row["q"], 7, name + " q")
    if len(row["actions"]) != 10:
        raise ValueError(f"{name}: expected 10 action steps")
    for index, action in enumerate(row["actions"]):
        finite_vector(action, 7, f"{name} action {index}")
    pad = row["is_pad"]
    if len(pad) != 10 or any(type(p) is not bool for p in pad):
        raise ValueError(f"{name}: invalid padding mask")
    valid_count = sum(not p for p in pad)
    if valid_count == 0 or pad != [False] * valid_count + [True] * (10 - valid_count):
        raise ValueError(f"{name}: expected a nonempty unpadded prefix")
    lines = row["action_source_lines"]
    times = row["actual_target_times"]
    if len(lines) != valid_count or len(times) != valid_count:
        raise ValueError(f"{name}: real command metadata must cover exactly nonpad steps")
    finite_vector(times, valid_count, name + " actual target times")
    if any(type(n) is not int or n < 1 for n in lines):
        raise ValueError(f"{name}: invalid source line identities")
    if any(b <= a for a, b in zip(lines, lines[1:])):
        raise ValueError(f"{name}: source command lines are not strictly ordered")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"{name}: command times are not strictly increasing")
    anchor = row["command_host_monotonic"]
    if not isinstance(anchor, (float, int)) or not math.isfinite(anchor):
        raise ValueError(f"{name}: invalid observation anchor")
    if times[0] < anchor - 1e-8:
        raise ValueError(f"{name}: target predates the command anchor")
    for role in ("global", "wrist"):
        camera = row["cameras"][role]
        if camera["available_t"] > anchor or camera["available_age_s"] < 0:
            raise ValueError(f"{name}: {role} image is from the future")
        if not Path(camera["path"]).is_file():
            raise FileNotFoundError(camera["path"])


def statistics_for(vectors: list[list[float]], definition: str) -> dict:
    columns = list(zip(*vectors))
    raw_std = [statistics.pstdev(col) for col in columns]
    std = [max(s, STD_FLOOR) for s in raw_std]
    return {
        "definition": definition,
        "count": len(vectors),
        "axes": AXES,
        "units": UNITS,
        "mean": [statistics.fmean(col) for col in columns],
        "std": std,
        "raw_std": raw_std,
        "min": [min(col) for col in columns],
        "max": [max(col) for col in columns],
        "std_definition": "population standard deviation, ddof=0",
        "std_floor": STD_FLOOR,
        "std_floored_axes": [AXES[i] for i, s in enumerate(raw_std) if s < STD_FLOOR],
        "constant_axes": [AXES[i] for i, col in enumerate(columns) if max(col) == min(col)],
        "low_variance_axes": [AXES[i] for i, s in enumerate(raw_std) if s < LOW_VARIANCE_STD_THRESHOLD],
        "low_variance_std_threshold": LOW_VARIANCE_STD_THRESHOLD,
        "low_variance_note": "Threshold uses each axis' raw recorded units and is a numerical diagnostic, not a calibrated physical threshold. Floored variance does not establish gripper coverage.",
    }


def uniformly_spaced_indices(length: int, count: int) -> list[int]:
    if length < count:
        raise ValueError("Not enough original examples for requested smoke selection")
    return [round(i * (length - 1) / (count - 1)) for i in range(count)]


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")
    temporary.replace(path)


def prepare(source: Path, output: Path, audit_path: Path) -> dict:
    audit = json.loads(audit_path.read_text())
    source_hash = sha256(source)
    if str(source.resolve()) != audit["examples_file"] or source_hash != audit["examples_sha256"]:
        raise ValueError("Source differs from the independently audited file; audit the new source first")
    rows = []
    for line, text in enumerate(source.open(), 1):
        if not text.strip():
            continue
        row = json.loads(text)
        validate_row(row, line)
        row["source_line_1_based"] = line
        rows.append(row)
    if len({row["id"] for row in rows}) != len(rows):
        raise ValueError("Duplicate example identity")
    split_rows = {split: [r for r in rows if r["split"] == split] for split in ("train", "val")}
    train, val = split_rows["train"], split_rows["val"]
    if not train or not val:
        raise ValueError("Both original train and validation splits are required")
    for field in ("episode_group", "segment_id", "action_source_log"):
        if {r[field] for r in train} & {r[field] for r in val}:
            raise ValueError(f"Cross-split leakage in {field}")

    unique_actions: dict[tuple[str, int], list[float]] = {}
    valid_occurrences = 0
    padded_occurrences = 0
    for row in train:
        padded_occurrences += sum(row["is_pad"])
        for step, line in enumerate(row["action_source_lines"]):
            if row["is_pad"][step]:
                raise ValueError("Padded target included in source command metadata")
            key = (row["action_source_log"], line)
            action = row["actions"][step]
            valid_occurrences += 1
            if key in unique_actions and unique_actions[key] != action:
                raise ValueError(f"One recorded command has inconsistent labels: {key}")
            unique_actions[key] = action

    common = {
        "source": {"path": str(source), "sha256": source_hash},
        "audit": {"path": str(audit_path), "sha256": sha256(audit_path)},
        "hardware_accessed": False,
        "robot_commands_sent": 0,
        "source_modified": False,
    }
    stats = {
        "schema": "d1-smolvla-train-statistics-v1", **common,
        "fitted_on_split": "train", "so100_statistics_used": False,
        "state": statistics_for([r["q"] for r in train], "Actual observed D1 SDK positions; all original training examples, no validation observations."),
        "action": statistics_for(list(unique_actions.values()), "Unpadded actual absolute SDK targets; training split only, deduplicated by (action_source_log, action_source_line)."),
        "counts": {
            "all_examples": len(rows), "train_examples": len(train), "val_examples": len(val),
            "train_valid_action_occurrences_before_deduplication": valid_occurrences,
            "train_padded_action_occurrences_excluded": padded_occurrences,
            "train_unique_action_commands": len(unique_actions),
        },
        "split_membership": {s: [r["id"] for r in rs] for s, rs in split_rows.items()},
        "split_policy": "Every source row retains its original split; there is no resampling, reassignment or window-level random split.",
        "action_semantics": "absolute_sdk_joint_target",
        "sample_rate_hz": 5, "action_chunk_size": 10,
        "constant_gripper_warning": "This lift-only source does not train grasp/open transitions. A small/floored gripper standard deviation must not be treated as calibrated or general gripper behavior.",
    }
    selected = []
    selection = {}
    for split, count in (("train", 4), ("val", 2)):
        indices = uniformly_spaced_indices(len(split_rows[split]), count)
        selection[split] = {"indices_0_based_within_split": indices, "count": count}
        for index in indices:
            row = dict(split_rows[split][index])
            for camera in row["cameras"].values():
                if sha256(camera["path"]) != camera["sha256"]:
                    raise ValueError(f"Image checksum mismatch: {camera['path']}")
            row["units"] = UNITS
            row["image_sha256_verified"] = True
            row["task"] = dict(TASK)
            selected.append(row)
    smoke = {
        "schema": "d1-smolvla-real-smoke-samples-v1", **common,
        "task": TASK, "units": UNITS, "selection": selection,
        "selection_method": "Uniformly spaced original-order indices including endpoints; no new synthetic images, states or action labels.",
        "validated": ["7D finite states", "10x7 finite action chunks", "unpadded source command identity", "strictly ordered nonpad times", "images available before command", "selected image SHA256"],
        "samples": selected,
    }
    readiness = {
        "schema": "d1-smolvla-training-readiness-v1", **common,
        "offline_inference_smoke_allowed": True,
        "few_step_technical_training_allowed": True,
        "full_grasp_training_allowed": False,
        "complete_grasp_episodes": audit["counts"]["complete_continuous_grasp_episodes"],
        "source_fragments": 2, "source_sessions": 2,
        "counts": stats["counts"],
        "source_split_preserved": True,
        "reason": "1004 overlapping action-chunk windows come from two lift fragments, not 1004 independent demonstrations. Complete continuous grasp episodes: 0.",
        "technical_weights_scope": "A few optimization steps may validate gradient/update/save/reload mechanics only. They cannot be promoted as a complete or general D1 grasp policy.",
        "gaps": audit["full_training_gaps"],
        "language_label": TASK,
        "historical_contact_record": audit["latest_related_historical_collection_state"],
        "held_out_validation_limit": "Source session is isolated, but these records have already been used for ACT development; this is not a fresh unseen-object or physical test set.",
        "npu_support_established": False,
    }
    for name, data in (("data_stats.json", stats), ("smoke_samples.json", smoke), ("training_readiness.json", readiness)):
        write_json(output / name, data)
    return {
        "status": "prepared", "output": str(output), "counts": stats["counts"],
        "selected_examples": len(selected),
        "state_constant_axes": stats["state"]["constant_axes"],
        "action_constant_axes": stats["action"]["constant_axes"],
        "full_grasp_training_allowed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts")
    parser.add_argument("--audit", type=Path, default=ROOT / "data_audit.json")
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output, args.audit), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
