"""Explicitly synthetic event fixtures only; never hardware measurement evidence."""
import copy
from fractions import Fraction
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/"measure/compare_timing_events.py"
SPEC = importlib.util.spec_from_file_location("timing_events", SCRIPT)
timing = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(timing)


def fixture(kind="ps", boundary="descriptor_valid", lateness=(-5, 10, -20, 0, 5)):
    """Invented deterministic timestamps, deliberately marked synthetic."""
    origin = 1_000_000_000_000_000_000+(0 if kind == "ps" else 1_000_000_000_000)
    offset = 1_000_000_000
    source = kind+"_descriptor" if boundary == "descriptor_valid" else "bus_capture_hardware"
    config = {field: hashlib.sha256(("synthetic-"+field).encode()).hexdigest() for field in timing.HASH_FIELDS}
    config.update({"f_exec_hz": {"numerator": 10, "denominator": 1}, "planned_phase_error_bound_ns": 0, "sequence_start": 7,
                   "tick_count": len(lateness), "payload_encoding": "synthetic opaque test bytes; not a CAN capture"})
    return {"schema": timing.SCHEMA, "run_kind": kind, "data_origin": "synthetic",
            "comparison_config": config, "measurement_boundary": boundary,
            "actual_timestamp_source": {"kind": source, "clock_id": kind+"_synthetic_actual",
                                        "description": "Invented test timestamps, no hardware capture",
                                        "evidence_sha256": "a"*64},
            "clock_mapping": {"kind": "affine_planned_to_actual", "planned_clock_id": kind+"_synthetic_plan",
                              "actual_clock_id": kind+"_synthetic_actual", "scale": {"numerator": 1, "denominator": 1},
                              "offset_ns": offset, "evidence_sha256": "b"*64, "max_error_ns": 3},
            "events": [{"seq": 7+index, "planned_ns": origin+index*100_000_000,
                        "payload_hex": f"{index:02x}a5", "status": "emitted",
                        "actual_ns": origin+index*100_000_000+offset+delay, "reason": None}
                       for index, delay in enumerate(lateness)]}


def pair(boundary="descriptor_valid"):
    return fixture("ps", boundary), fixture("pl", boundary)


class TimingEventTests(unittest.TestCase):
    def test_jittered_raw_integer_times_and_negative_lateness_are_preserved(self):
        ps, pl = pair()
        before = copy.deepcopy((ps, pl))
        result = timing.compare_logs(ps, pl)
        run = result["runs"]["ps"]
        self.assertEqual((ps, pl), before)
        self.assertEqual(run["events"], ps["events"])
        self.assertEqual([item["actual_interval_ns"] for item in run["emitted_intervals"]],
                         [100_000_015, 99_999_970, 100_000_020, 100_000_005])
        self.assertEqual(run["lateness_statistics"], {"count": 5, "p50_ns": 0,
                         "p99_ns": 9.8, "min_ns": -20, "max_ns": 10, "max_abs_ns": 20})
        self.assertEqual(run["interval_error_statistics"]["max_abs_ns"], 30)
        self.assertEqual(run["interval_mapping_error_bound_ns"], 6)
        self.assertEqual(result["planned_relative_ns"], [0, 100_000_000, 200_000_000, 300_000_000, 400_000_000])
        self.assertEqual(run["planned_phase_error_statistics"]["max_abs_ns"], 0)
        self.assertEqual(run["statistics_clock_domain"]["lateness"], "ps_synthetic_actual")
        self.assertEqual(result["comparison_scope"], "internal_descriptor_events_only")
        self.assertFalse(result["hardware_independently_authenticated"])
        self.assertFalse(result["e3_hardware_acceptance"])
        self.assertTrue(result["hardware_unverified"])
        self.assertIsNone(result["physical_f_exec_hz"])

    def test_drop_and_fault_have_explicit_rows_and_corresponding_planned_span(self):
        ps, pl = pair()
        ps["events"][1].update(status="dropped", actual_ns=None, reason="synthetic drop")
        ps["events"][3].update(status="fault", actual_ns=None, reason="synthetic fault")
        result = timing.compare_logs(ps, pl)["runs"]["ps"]
        self.assertEqual((result["emitted_count"], result["dropped_count"], result["fault_count"]), (3, 1, 1))
        self.assertEqual(len(result["events"]), 5)
        self.assertEqual([item["sequence_span"] for item in result["emitted_intervals"]], [2, 2])
        self.assertEqual([item["planned_interval_ns"] for item in result["emitted_intervals"]], [200_000_000]*2)
        self.assertEqual([item["interval_error_ns"]["numerator"] for item in result["emitted_intervals"]], [-15, 25])
        self.assertTrue(all(item["crosses_non_emitted_ticks"] for item in result["emitted_intervals"]))
        self.assertEqual(result["rate_scope"], "emitted_events_only_not_tick_rate")
        self.assertLess(result["mean_observed_event_rate_hz"], 6)

    def test_zero_or_one_emitted_event_has_no_interval_or_rate(self):
        for keep in (0, 1):
            ps, pl = pair()
            for event in ps["events"][keep:]:
                event.update(status="dropped", actual_ns=None, reason="synthetic missing emission")
            run = timing.compare_logs(ps, pl)["runs"]["ps"]
            self.assertEqual(run["emitted_count"], keep)
            self.assertEqual(run["actual_interval_statistics"]["count"], 0)
            self.assertIsNone(run["actual_interval_statistics"]["p99_ns"])
            self.assertIsNone(run["mean_observed_event_rate_hz"])
            self.assertEqual(run["rate_unavailable_reason"], "fewer_than_two_emitted_events")

    def test_affine_mapping_uses_exact_rationals_before_subtracting_large_epochs(self):
        ps, pl = pair()
        for log in (ps, pl):
            log["clock_mapping"]["scale"] = {"numerator": 3, "denominator": 2}
            for index, event in enumerate(log["events"]):
                event["planned_ns"] += 1
                expected = Fraction(event["planned_ns"]*3, 2)+log["clock_mapping"]["offset_ns"]
                event["actual_ns"] = expected.numerator//expected.denominator+index
        run = timing.compare_logs(ps, pl)["runs"]["ps"]
        self.assertEqual(run["lateness"][0]["lateness_ns"], {"numerator": -1, "denominator": 2})
        self.assertEqual(run["lateness"][0]["lateness_with_mapping_error_ns"],
                         {"lower": {"numerator": -7, "denominator": 2}, "upper": {"numerator": 5, "denominator": 2}})
        self.assertEqual(run["emitted_intervals"][0]["mapped_planned_interval_ns"], {"numerator": 150_000_000, "denominator": 1})
        self.assertEqual(run["interval_error_statistics"]["p50_ns"], 1)

    def test_can_only_accepts_explicit_hardware_source_semantics_but_never_authenticates(self):
        for source in ("bus_capture_hardware", "controller_tx_hardware"):
            ps, pl = pair("can_group_first_frame_start")
            for log in (ps, pl):
                log["actual_timestamp_source"]["kind"] = source
            result = timing.compare_logs(ps, pl)
            self.assertEqual(result["comparison_scope"], "declared_can_group_first_frame_start_events")
            self.assertFalse(result["hardware_independently_authenticated"])
            self.assertFalse(result["e3_hardware_acceptance"])
        for source in ("host_receive", "host_api_return", "pl_descriptor", "software_tx", "logic_capture_hardware"):
            ps, pl = pair("can_group_first_frame_start")
            ps["actual_timestamp_source"]["kind"] = source
            with self.subTest(source=source), self.assertRaisesRegex(ValueError, "CAN requires"):
                timing.compare_logs(ps, pl)

    def test_every_frozen_hash_and_configuration_value_must_match(self):
        for field in timing.HASH_FIELDS:
            ps, pl = pair()
            pl["comparison_config"][field] = "c"*64
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "comparison_config"):
                timing.compare_logs(ps, pl)
        for field, value in (("f_exec_hz", {"numerator": 21, "denominator": 2}),
                             ("planned_phase_error_bound_ns", 1),
                             ("payload_encoding", "different byte meaning")):
            ps, pl = pair()
            pl["comparison_config"][field] = value
            if field == "f_exec_hz":
                pl["comparison_config"]["planned_phase_error_bound_ns"] = 100_000_000
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "comparison_config"):
                timing.compare_logs(ps, pl)

    def test_payload_including_dropped_tick_must_match(self):
        ps, pl = pair()
        pl["events"][2].update(status="dropped", actual_ns=None, reason="synthetic drop", payload_hex="deadbeef")
        with self.assertRaisesRegex(ValueError, "payload"):
            timing.compare_logs(ps, pl)

    def test_planned_relative_times_not_actual_times_determine_alignment(self):
        ps, pl = pair()
        for log in (ps, pl):
            log["comparison_config"]["planned_phase_error_bound_ns"] = 1
        pl["events"][2]["planned_ns"] += 1
        with self.assertRaisesRegex(ValueError, "raw planned relative"):
            timing.compare_logs(ps, pl)

    def test_declared_frequency_cannot_hide_inconsistent_plan_intervals(self):
        ps, pl = pair()
        for log in (ps, pl):
            origin = log["events"][0]["planned_ns"]
            log["comparison_config"]["f_exec_hz"] = {"numerator": 5, "denominator": 1}
            for index, event in enumerate(log["events"]):
                event["planned_ns"] = origin+index
        with self.assertRaisesRegex(ValueError, "inconsistent with f_exec_hz"):
            timing.compare_logs(ps, pl)

    def test_explicit_dds_phase_bound_supports_rational_rate_without_resampling(self):
        ps, pl = pair()
        for log in (ps, pl):
            log["comparison_config"]["f_exec_hz"] = {"numerator": 3, "denominator": 1}
            log["comparison_config"]["planned_phase_error_bound_ns"] = 1
            origin = log["events"][0]["planned_ns"]
            for index, event in enumerate(log["events"]):
                event["planned_ns"] = origin+index*1_000_000_000//3
                event["actual_ns"] = event["planned_ns"]+1_000_000_000
        result = timing.compare_logs(ps, pl)
        self.assertEqual(result["runs"]["ps"]["planned_phase_errors"][1]["phase_error_ns"],
                         {"numerator": -1, "denominator": 3})
        self.assertAlmostEqual(result["runs"]["ps"]["planned_phase_error_statistics"]["max_abs_ns"], 2/3)
        for log in (ps, pl):
            log["comparison_config"]["planned_phase_error_bound_ns"] = 0
        with self.assertRaisesRegex(ValueError, "planned_phase_error_bound_ns"):
            timing.compare_logs(ps, pl)

    def test_missing_duplicate_reordered_and_wrong_extent_sequences_rejected(self):
        mutations = (lambda log: log["events"].pop(2),
                     lambda log: log["events"][2].update(seq=log["events"][1]["seq"]),
                     lambda log: log["events"].__setitem__(slice(1, 3), log["events"][1:3][::-1]),
                     lambda log: log["comparison_config"].update(sequence_start=0))
        for mutation in mutations:
            ps = fixture()
            mutation(ps)
            with self.assertRaises(ValueError):
                timing.validate_log(ps)

    def test_invalid_actual_status_and_nonpositive_intervals_rejected(self):
        mutations = (lambda event: event.update(actual_ns=None),
                     lambda event: event.update(status="dropped", reason="drop"),
                     lambda event: event.update(status="fault", actual_ns=None, reason=""),
                     lambda event: event.update(status="unknown"),
                     lambda event: event.update(reason="must be null when emitted"))
        for mutation in mutations:
            log = fixture()
            mutation(log["events"][1])
            with self.assertRaises(ValueError):
                timing.validate_log(log)
        for difference in (0, -1):
            log = fixture()
            log["events"][2]["actual_ns"] = log["events"][1]["actual_ns"]+difference
            with self.assertRaisesRegex(ValueError, "strictly positive"):
                timing.validate_log(log)

    def test_integer_ns_reject_float_bool_null_string_and_out_of_range(self):
        for field in ("planned_ns", "actual_ns", "seq"):
            for value in (1.0, True, None, "1", float("nan"), float("inf"), 1 << 63):
                log = fixture()
                log["events"][1][field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    timing.validate_log(log)

    def test_rational_rate_and_mapping_scale_require_reduced_positive_integers(self):
        for value in ({"numerator": 0, "denominator": 1}, {"numerator": 1, "denominator": 0},
                      {"numerator": -1, "denominator": 2}, {"numerator": 2, "denominator": 2},
                      {"numerator": True, "denominator": 1}, {"numerator": 1.0, "denominator": 1},
                      {"numerator": 1, "denominator": 1, "extra": 0}):
            for target in ("rate", "scale"):
                log = fixture()
                if target == "rate":
                    log["comparison_config"]["f_exec_hz"] = value
                else:
                    log["clock_mapping"]["scale"] = value
                with self.subTest(target=target, value=value), self.assertRaises(ValueError):
                    timing.validate_log(log)

    def test_unknown_missing_nested_fields_hashes_mapping_and_boundary_rejected(self):
        mutations = (lambda log: log.update(extra=1),
                     lambda log: log.pop("data_origin"),
                     lambda log: log["events"][0].update(host_receive_ns=123),
                     lambda log: log["comparison_config"].update(trajectory_sha256="not-a-hash"),
                     lambda log: log["actual_timestamp_source"].update(evidence_sha256="A"*64),
                     lambda log: log["clock_mapping"].update(evidence_sha256=None),
                     lambda log: log["clock_mapping"].update(max_error_ns=-1),
                     lambda log: log["comparison_config"].update(planned_phase_error_bound_ns=-1),
                     lambda log: log["clock_mapping"].update(actual_clock_id="wrong-clock"),
                     lambda log: log.update(measurement_boundary="host_receive"),
                     lambda log: log["events"][1].update(planned_ns=log["events"][0]["planned_ns"]),
                     lambda log: log["events"][0].update(payload_hex="ABC"))
        for mutation in mutations:
            log = fixture()
            mutation(log)
            with self.assertRaises(ValueError):
                timing.validate_log(log)

    def test_roles_boundaries_and_origins_cannot_be_mixed(self):
        ps, pl = pair()
        with self.assertRaisesRegex(ValueError, "--ps"):
            timing.compare_logs(pl, ps)
        pl = fixture("pl", "can_group_first_frame_start")
        with self.assertRaisesRegex(ValueError, "measurement_boundary"):
            timing.compare_logs(ps, pl)
        pl = fixture("pl")
        pl["data_origin"] = "recorded"
        with self.assertRaisesRegex(ValueError, "data_origin"):
            timing.compare_logs(ps, pl)

    def test_raw_json_duplicate_keys_nan_infinity_and_non_object_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"synthetic.json"
            for raw in ('{"schema": 1, "schema": 2}', '{"schema": NaN}', '{"schema": Infinity}', '[]'):
                path.write_text(raw)
                with self.subTest(raw=raw), self.assertRaises(ValueError):
                    timing.load_log(path)

    def test_cli_hashes_invocation_no_overwrite_and_synthetic_labels(self):
        self.assertEqual(subprocess.run([sys.executable, str(SCRIPT), "--help"], capture_output=True).returncode, 0)
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            ps, pl = pair()
            ps_path, pl_path, output = directory/"synthetic_ps.json", directory/"synthetic_pl.json", directory/"synthetic_comparison.json"
            for path, log in ((ps_path, ps), (pl_path, pl)):
                path.write_text(json.dumps(log))
            command = [sys.executable, str(SCRIPT), "--ps", str(ps_path), "--pl", str(pl_path), "--output", str(output)]
            completed = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(output.read_text())
            self.assertEqual(result["input_ps_sha256"], hashlib.sha256(ps_path.read_bytes()).hexdigest())
            self.assertEqual(result["input_pl_sha256"], hashlib.sha256(pl_path.read_bytes()).hexdigest())
            self.assertEqual(result["implementation_sha256"], hashlib.sha256(SCRIPT.read_bytes()).hexdigest())
            self.assertEqual(result["configuration_sha256"], timing.canonical_sha256(ps["comparison_config"]))
            self.assertEqual(result["invocation"][2:], command[2:])
            self.assertTrue(all(run["data_origin"] == "synthetic" for run in result["runs"].values()))
            before = output.read_bytes()
            again = subprocess.run(command, text=True, capture_output=True)
            self.assertNotEqual(again.returncode, 0)
            self.assertIn("output exists", again.stderr)
            self.assertEqual(output.read_bytes(), before)
            bad_output = directory/"must_not_exist.json"
            ps_path.write_text(ps_path.read_text().replace('"actual_ns":', '"actual_ns": NaN, "discarded":'))
            rejected = subprocess.run(command[:-1]+[str(bad_output)], capture_output=True)
            self.assertNotEqual(rejected.returncode, 0)
            self.assertFalse(bad_output.exists())


if __name__ == "__main__":
    unittest.main()
