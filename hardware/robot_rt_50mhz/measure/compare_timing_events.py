#!/usr/bin/env python3
"""Compare declared PS/PL event logs offline, without resampling or hardware access.

Strict schema robot_rt.timing_events.v1 is documented in docs/SMOOTHNESS.md.
Example (both inputs must already exist; this tool never creates capture data):
  python3 measure/compare_timing_events.py --ps /capture/ps.json \\
    --pl /capture/pl.json --output /new/comparison.json

Timestamp source/evidence declarations are preserved, never authenticated here.
Descriptor events describe an internal boundary, never CAN transmission.
"""
from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import re
import sys

SCHEMA = "robot_rt.timing_events.v1"
HASH_FIELDS = ("trajectory_sha256", "initial_state_sha256", "algorithm_sha256",
               "limits_sha256", "integer_rounding_sha256", "failure_strategy_sha256")
CAN_SOURCES = {"bus_capture_hardware", "controller_tx_hardware"}
DESCRIPTOR_SOURCES = {"ps_descriptor", "pl_descriptor", "logic_capture_hardware"}
I64_MIN, I64_MAX = -(1 << 63), (1 << 63)-1


def exact_keys(value, keys, name):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f"{name} keys must match the documented schema exactly")


def integer(value, name, *, minimum=I64_MIN, maximum=I64_MAX):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def text_field(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


def sha256_field(value, name):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{name} must be a lowercase SHA256 hex string")


def positive_rational(value, name):
    exact_keys(value, {"numerator", "denominator"}, name)
    numerator = integer(value["numerator"], name+".numerator", minimum=1)
    denominator = integer(value["denominator"], name+".denominator", minimum=1)
    if math.gcd(numerator, denominator) != 1:
        raise ValueError(f"{name} must be in reduced rational form")
    return Fraction(numerator, denominator)


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def validate_log(log):
    exact_keys(log, {"schema", "run_kind", "data_origin", "comparison_config",
                     "measurement_boundary", "actual_timestamp_source", "clock_mapping", "events"}, "log")
    if log["schema"] != SCHEMA or log["run_kind"] not in ("ps", "pl"):
        raise ValueError("unsupported schema or run_kind (expected ps or pl)")
    if log["data_origin"] not in ("recorded", "synthetic"):
        raise ValueError("data_origin must explicitly be recorded or synthetic")
    config = log["comparison_config"]
    exact_keys(config, {*HASH_FIELDS, "f_exec_hz", "planned_phase_error_bound_ns",
                        "sequence_start", "tick_count", "payload_encoding"}, "comparison_config")
    for field in HASH_FIELDS:
        sha256_field(config[field], "comparison_config."+field)
    rate = positive_rational(config["f_exec_hz"], "f_exec_hz")
    phase_bound = integer(config["planned_phase_error_bound_ns"], "planned_phase_error_bound_ns", minimum=0)
    start = integer(config["sequence_start"], "sequence_start", minimum=0)
    count = integer(config["tick_count"], "tick_count", minimum=2)
    integer(start+count-1, "last sequence", minimum=0)
    text_field(config["payload_encoding"], "payload_encoding")
    boundary = log["measurement_boundary"]
    if boundary not in ("descriptor_valid", "can_group_first_frame_start"):
        raise ValueError("unsupported measurement_boundary")
    source = log["actual_timestamp_source"]
    exact_keys(source, {"kind", "clock_id", "description", "evidence_sha256"}, "actual_timestamp_source")
    for field in ("clock_id", "description"):
        text_field(source[field], "actual_timestamp_source."+field)
    sha256_field(source["evidence_sha256"], "actual_timestamp_source.evidence_sha256")
    allowed = CAN_SOURCES if boundary == "can_group_first_frame_start" else DESCRIPTOR_SOURCES
    if not isinstance(source["kind"], str) or source["kind"] not in allowed:
        raise ValueError("actual_timestamp_source.kind does not measure the declared boundary; CAN requires bus_capture_hardware or controller_tx_hardware")
    if boundary == "descriptor_valid" and source["kind"] in {"ps_descriptor", "pl_descriptor"} and source["kind"] != log["run_kind"]+"_descriptor":
        raise ValueError("descriptor source must match run_kind")
    mapping = log["clock_mapping"]
    exact_keys(mapping, {"kind", "planned_clock_id", "actual_clock_id", "scale", "offset_ns",
                         "evidence_sha256", "max_error_ns"}, "clock_mapping")
    if mapping["kind"] != "affine_planned_to_actual":
        raise ValueError("clock_mapping.kind must be affine_planned_to_actual")
    for field in ("planned_clock_id", "actual_clock_id"):
        text_field(mapping[field], "clock_mapping."+field)
    if mapping["actual_clock_id"] != source["clock_id"]:
        raise ValueError("clock_mapping actual_clock_id must match the actual timestamp source")
    positive_rational(mapping["scale"], "clock_mapping.scale")
    integer(mapping["offset_ns"], "clock_mapping.offset_ns")
    integer(mapping["max_error_ns"], "clock_mapping.max_error_ns", minimum=0)
    sha256_field(mapping["evidence_sha256"], "clock_mapping.evidence_sha256")
    events = log["events"]
    if not isinstance(events, list) or len(events) != count:
        raise ValueError("events must contain every declared tick, including dropped/fault ticks")
    first_plan = previous_plan = previous_actual = None
    for index, event in enumerate(events):
        exact_keys(event, {"seq", "planned_ns", "payload_hex", "status", "actual_ns", "reason"}, f"events[{index}]")
        seq = integer(event["seq"], "seq", minimum=0)
        if seq != start+index:
            raise ValueError("sequence must be complete, unique and ordered; missing ticks must be explicit dropped/fault events")
        planned = integer(event["planned_ns"], "planned_ns")
        if previous_plan is not None and planned <= previous_plan:
            raise ValueError("planned_ns must be strictly increasing")
        if first_plan is None:
            first_plan = planned
        phase_error = Fraction(planned-first_plan)-Fraction(index*1_000_000_000, 1)/rate
        if abs(phase_error) > phase_bound:
            raise ValueError("planned timing is inconsistent with f_exec_hz and planned_phase_error_bound_ns")
        previous_plan = planned
        if not isinstance(event["payload_hex"], str) or re.fullmatch(r"(?:[0-9a-f]{2})+", event["payload_hex"]) is None:
            raise ValueError("payload_hex must be nonempty, even-length lowercase hex bytes")
        status = event["status"]
        if status == "emitted":
            actual = integer(event["actual_ns"], "actual_ns")
            if event["reason"] is not None:
                raise ValueError("emitted events require reason=null")
            if previous_actual is not None and actual <= previous_actual:
                raise ValueError("actual_ns of emitted events must have strictly positive intervals")
            previous_actual = actual
        elif status in ("dropped", "fault"):
            if event["actual_ns"] is not None:
                raise ValueError("dropped/fault events require actual_ns=null")
            text_field(event["reason"], status+".reason")
        else:
            raise ValueError("status must be emitted, dropped or fault")
    return log


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError(f"nonfinite JSON constant: {value}")


def load_log(path):
    raw = Path(path).read_bytes()
    log = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object,
                     parse_constant=_invalid_constant)
    return validate_log(log), hashlib.sha256(raw).hexdigest()


def rational_record(value):
    value = Fraction(value)
    return {"numerator": value.numerator, "denominator": value.denominator}


def numeric(value):
    value = Fraction(value)
    return value.numerator if value.denominator == 1 else float(value)


def quantile(values, numerator, denominator):
    ordered = sorted(Fraction(value) for value in values)
    position = Fraction((len(ordered)-1)*numerator, denominator)
    low = position.numerator//position.denominator
    fraction = position-low
    return ordered[low]+fraction*(ordered[min(low+1, len(ordered)-1)]-ordered[low])


def ns_statistics(values):
    if not values:
        return {"count": 0, "p50_ns": None, "p99_ns": None, "min_ns": None,
                "max_ns": None, "max_abs_ns": None}
    return {"count": len(values), "p50_ns": numeric(quantile(values, 1, 2)),
            "p99_ns": numeric(quantile(values, 99, 100)), "min_ns": numeric(min(values)),
            "max_ns": numeric(max(values)), "max_abs_ns": numeric(max(abs(value) for value in values))}


def summarize_run(log):
    mapping = log["clock_mapping"]
    scale = positive_rational(mapping["scale"], "clock_mapping.scale")
    offset, bound = mapping["offset_ns"], mapping["max_error_ns"]
    emitted = [event for event in log["events"] if event["status"] == "emitted"]
    lateness, intervals = [], []
    rate = positive_rational(log["comparison_config"]["f_exec_hz"], "f_exec_hz")
    first_plan = log["events"][0]["planned_ns"]
    phase_errors = [Fraction(event["planned_ns"]-first_plan)-Fraction(index*1_000_000_000, 1)/rate
                    for index, event in enumerate(log["events"])]
    for event in emitted:
        expected = event["planned_ns"]*scale+offset
        delta = event["actual_ns"]-expected
        lateness.append({"seq": event["seq"], "mapped_planned_ns": rational_record(expected),
                         "lateness_ns": rational_record(delta),
                         "lateness_with_mapping_error_ns": {"lower": rational_record(delta-bound),
                                                           "upper": rational_record(delta+bound)}})
    for first, last in zip(emitted, emitted[1:]):
        actual_delta = last["actual_ns"]-first["actual_ns"]
        planned_delta = last["planned_ns"]-first["planned_ns"]
        mapped_delta = planned_delta*scale
        intervals.append({"from_seq": first["seq"], "to_seq": last["seq"],
                          "sequence_span": last["seq"]-first["seq"],
                          "crosses_non_emitted_ticks": last["seq"]-first["seq"] > 1,
                          "actual_interval_ns": actual_delta, "planned_interval_ns": planned_delta,
                          "mapped_planned_interval_ns": rational_record(mapped_delta),
                          "interval_error_ns": rational_record(actual_delta-mapped_delta)})
    missing = len(log["events"])-len(emitted)
    span = emitted[-1]["actual_ns"]-emitted[0]["actual_ns"] if len(emitted) >= 2 else None
    rate = Fraction((len(emitted)-1)*1_000_000_000, span) if span is not None else None
    return {"run_kind": log["run_kind"], "data_origin": log["data_origin"],
            "measurement_boundary": log["measurement_boundary"],
            "actual_timestamp_source": log["actual_timestamp_source"], "clock_mapping": mapping,
            "declared_tick_count": len(log["events"]), "emitted_count": len(emitted),
            "dropped_count": sum(event["status"] == "dropped" for event in log["events"]),
            "fault_count": sum(event["status"] == "fault" for event in log["events"]),
            "non_emitted_count": missing, "events": log["events"],
            "planned_phase_errors": [{"seq": event["seq"], "phase_error_ns": rational_record(error)}
                                     for event, error in zip(log["events"], phase_errors)],
            "planned_phase_error_statistics": ns_statistics(phase_errors),
            "emitted_intervals": intervals, "lateness": lateness,
            "statistics_clock_domain": {"planned_phase_error": mapping["planned_clock_id"],
                                        "actual_interval": mapping["actual_clock_id"],
                                        "interval_error": mapping["actual_clock_id"],
                                        "lateness": mapping["actual_clock_id"],
                                        "mean_observed_event_rate": mapping["actual_clock_id"]},
            "actual_interval_statistics": ns_statistics([item["actual_interval_ns"] for item in intervals]),
            "interval_error_statistics": ns_statistics([Fraction(item["interval_error_ns"]["numerator"], item["interval_error_ns"]["denominator"]) for item in intervals]),
            "lateness_statistics": ns_statistics([Fraction(item["lateness_ns"]["numerator"], item["lateness_ns"]["denominator"]) for item in lateness]),
            "interval_mapping_error_bound_ns": 2*bound,
            "mean_observed_event_rate_hz": None if rate is None else numeric(rate),
            "mean_observed_event_rate_hz_exact": None if rate is None else rational_record(rate),
            "rate_formula": "(emitted_count-1)*1e9/(last_actual_ns-first_actual_ns)",
            "rate_scope": "emitted_events_only_not_tick_rate" if missing else "emitted_event_intervals",
            "rate_unavailable_reason": "fewer_than_two_emitted_events" if rate is None else None}


def compare_logs(ps, pl):
    for log in (ps, pl):
        validate_log(log)
    if ps["run_kind"] != "ps" or pl["run_kind"] != "pl":
        raise ValueError("--ps must identify ps and --pl must identify pl")
    if ps["comparison_config"] != pl["comparison_config"]:
        raise ValueError("comparison_config must match exactly, including hashes, rational f_exec and sequence extent")
    if ps["measurement_boundary"] != pl["measurement_boundary"]:
        raise ValueError("measurement_boundary must match between runs")
    if ps["data_origin"] != pl["data_origin"]:
        raise ValueError("data_origin must match; synthetic and recorded logs cannot be paired")
    ps_origin, pl_origin = ps["events"][0]["planned_ns"], pl["events"][0]["planned_ns"]
    for first, second in zip(ps["events"], pl["events"]):
        if first["seq"] != second["seq"] or first["payload_hex"] != second["payload_hex"]:
            raise ValueError("per-tick sequence and payload must match, including non-emitted ticks")
        if first["planned_ns"]-ps_origin != second["planned_ns"]-pl_origin:
            raise ValueError("raw planned relative times must match exactly; actual times are never used for alignment")
    descriptor = ps["measurement_boundary"] == "descriptor_valid"
    return {"schema": "robot_rt.timing_comparison.v1", "provenance": "offline_declared_event_comparison",
            "hardware_independently_authenticated": False, "hardware_unverified": True,
            "e3_hardware_acceptance": False, "physical_f_exec_hz": None,
            "comparison_scope": "internal_descriptor_events_only" if descriptor else "declared_can_group_first_frame_start_events",
            "configuration": ps["comparison_config"],
            "configuration_sha256": canonical_sha256(ps["comparison_config"]),
            "planned_relative_ns": [event["planned_ns"]-ps_origin for event in ps["events"]],
            "runs": {"ps": summarize_run(ps), "pl": summarize_run(pl)},
            "statistical_method": "linear quantiles at (N-1)*p; integer/rational arithmetic before final statistic formatting",
            "limits": ["No device access, resampling, timestamp rewriting, actual-time alignment or hardware acceptance is performed.",
                       "Timestamp sources, evidence SHA256, clock mappings and error bounds are input declarations, not independently authenticated artifacts.",
                       "Declared f_exec is locked between runs; this tool does not certify its physical realization or infer it from packet arrival times.",
                       "Each raw planned elapsed time must match tick_index*1e9/f_exec within the locked, declared planned-phase error bound; this does not authenticate that bound or its source clock.",
                       "Interval errors compare successive emitted events against their corresponding planned span, including intervening dropped/fault ticks.",
                       "Lateness is actual_ns minus the affine-mapped planned_ns; negative values are retained. Mapping uncertainty is not subtracted to improve a result.",
                       "Mapping max_error_ns bounds each timestamp; interval-error uncertainty is conservatively twice that bound, without an independence assumption.",
                       "Payload bytes/encoding and CAN timestamp semantics are declared; no codec, complete CAN group, actuator acceptance or mechanical response is certified.",
                       "Descriptor comparisons concern an internal event boundary only, never physical CAN transmission or an E3 improvement claim."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ps", required=True, type=Path, help="Existing strict JSON PS event log")
    parser.add_argument("--pl", required=True, type=Path, help="Existing strict JSON PL event log")
    parser.add_argument("--output", required=True, type=Path, help="New JSON file; existing outputs are never overwritten")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError("output exists; choose a new evidence path")
        ps, ps_hash = load_log(args.ps)
        pl, pl_hash = load_log(args.pl)
        result = compare_logs(ps, pl)
        result.update({"input_ps_sha256": ps_hash, "input_pl_sha256": pl_hash,
                       "input_paths": {"ps": str(args.ps.resolve()), "pl": str(args.pl.resolve())},
                       "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       "python_version": sys.version.split()[0], "working_directory": str(Path.cwd()),
                       "invocation": [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)]})
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
        print(json.dumps({"output": str(args.output.resolve()), "comparison_scope": result["comparison_scope"],
                          "hardware_independently_authenticated": False, "e3_hardware_acceptance": False}))
        return 0
    except (ValueError, TypeError, KeyError, OSError, UnicodeError) as exc:
        parser.exit(2, f"offline timing comparison: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
