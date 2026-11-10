"""Synthetic offline tests only; no devices, SDK or live feedback."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from planner import PlanValidationError, canonical_plan_sha256, prepare_session
from seven_axis_reference import ClockState, build_reference_sequence, clock_step


class SevenAxisReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="d1_seven_axis_reference_test_")
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.qs = [[0., -10., 20., 0., 0., 0., 10.],
                   [.5, -10., 20., 0., 0., 0., 40.],
                   [-.2, -10., 20., 0., 0., 0., 15.]]
        self.plan = self.make_plan(self.qs)

    def make_plan(self, qs):
        rows = [{"sample": i, "elapsed_s": i*.1, "dds_received_monotonic_s": 100+i*.1, "q": q}
                for i, q in enumerate(qs)]
        (self.path/"joint_samples.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
        (self.path/"summary.json").write_text(json.dumps({"state": "stopped"}))
        return prepare_session(self.path, speed_deg_s=1.)

    def assert_code(self, code, callback):
        with self.assertRaises(PlanValidationError) as caught:
            callback()
        self.assertEqual(caught.exception.code, code)

    def rehash(self, plan):
        plan["plan_sha256"] = canonical_plan_sha256(plan)
        return plan

    def test_dynamic_gripper_and_all_waypoints_remain_synchronized(self):
        before = copy.deepcopy(self.plan)
        seq = build_reference_sequence(self.plan, expected_sha256=self.plan["plan_sha256"])
        self.assertEqual(self.plan, before)
        self.assertFalse(seq.physical_execution_ready)
        self.assertEqual(seq.joint_units, ("degree",)*6+("uncalibrated_sdk_raw",))
        self.assertEqual(seq.points[0].q, tuple(self.qs[0]))
        self.assertEqual(seq.points[-1].q, tuple(self.qs[-1]))
        self.assertEqual(seq.points[-1].plan_time_s, self.plan["duration"])
        self.assertEqual(seq.duration_s, self.plan["duration"])
        self.assertEqual([p.q for p in seq.points if p.is_waypoint], [tuple(q) for q in self.qs])
        self.assertEqual(max(p.q[6] for p in seq.points), 40.)
        for first, second in zip(seq.points, seq.points[1:]):
            self.assertGreaterEqual(second.plan_time_s-first.plan_time_s, .1-1e-9)
        interiors = [p for p in seq.points if not p.is_waypoint]
        self.assertTrue(interiors)
        for point in interiors:
            segment = self.plan["segments"][point.segment_index]
            arm_fraction = (point.q[0]-segment["q0"][0])/(segment["q1"][0]-segment["q0"][0])
            grip_fraction = (point.q[6]-segment["q0"][6])/(segment["q1"][6]-segment["q0"][6])
            self.assertAlmostEqual(arm_fraction, grip_fraction, places=12)
            self.assertEqual(len(point.q), 7)

    def test_gripper_only_open_close_is_not_dropped_at_ten_hz(self):
        qs = [[0.]*6+[raw] for raw in [10., 40., 12., 45.]]
        seq = build_reference_sequence(self.make_plan(qs))
        self.assertEqual([p.q[6] for p in seq.points if p.is_waypoint], [10., 40., 12., 45.])
        self.assert_code("WAYPOINT_RATE_CONFLICT", lambda: build_reference_sequence(self.make_plan(qs), rate_hz=5.))

    def test_exact_clock_outputs_all_refs_once_without_burst(self):
        seq = build_reference_sequence(self.plan)
        state, emitted = ClockState(), []
        for point in seq.points:
            step = clock_step(seq, state, point.plan_time_s)
            self.assertEqual(step.reference, point)
            emitted.append(step.reference)
            duplicate = clock_step(seq, step.state, point.plan_time_s)
            self.assertIsNone(duplicate.reference)
            state = duplicate.state
        self.assertTrue(step.done)
        self.assertEqual(emitted, list(seq.points))

    def test_late_clock_fails_without_skipping_or_changing_deadlines(self):
        seq = build_reference_sequence(self.plan)
        initial = clock_step(seq, ClockState(), 0.)
        idle = clock_step(seq, initial.state, .01)
        self.assertIsNone(idle.reference)
        late = seq.points[1].plan_time_s+.021
        self.assert_code("CLOCK_OVERRUN", lambda: clock_step(seq, idle.state, late))
        self.assertEqual(idle.state.next_index, 1)
        self.assertEqual(seq.points[-1].plan_time_s, self.plan["duration"])

    def test_actual_emissions_cannot_exceed_ten_hz_under_small_jitter(self):
        seq = build_reference_sequence(self.make_plan([[0.]*6+[x] for x in [10., 20., 30., 40.]]))
        first = clock_step(seq, ClockState(), .01)
        waiting = clock_step(seq, first.state, seq.points[1].plan_time_s)
        self.assertIsNone(waiting.reference)
        second = clock_step(seq, waiting.state, .11)
        self.assertEqual(second.reference, seq.points[1])
        self.assertEqual(second.reference.plan_time_s, seq.points[1].plan_time_s)
        self.assert_code("CLOCK_OVERRUN", lambda: clock_step(seq, second.state, .25))
        self.assert_code("CLOCK_REVERSED", lambda: clock_step(seq, second.state, .10))

    def test_hash_units_and_limits_reject_even_rehashed_invalid_content(self):
        tampered = copy.deepcopy(self.plan)
        tampered["duration"] += 1
        self.assert_code("PLAN_HASH", lambda: build_reference_sequence(tampered))
        self.assert_code("PLAN_HASH", lambda: build_reference_sequence(self.plan, expected_sha256="0"*64))
        for key, value, code in [("joint_units", ["degree"]*7, "UNITS_MISMATCH"),
                                 ("limits", [[-999., 999.]]*7, "LIMITS_MISMATCH")]:
            bad = copy.deepcopy(self.plan)
            bad[key] = value
            self.assert_code(code, lambda bad=bad: build_reference_sequence(self.rehash(bad)))
        bad = copy.deepcopy(self.plan)
        bad["original_points"][0]["q"][6] = 51.
        self.assert_code("JOINT_LIMIT", lambda: build_reference_sequence(self.rehash(bad)))

    def test_invalid_durations_missing_phase_and_allocation_fail(self):
        bad = copy.deepcopy(self.plan)
        bad["duration"] += .1
        self.assert_code("PLAN_DURATION", lambda: build_reference_sequence(self.rehash(bad)))
        bad = copy.deepcopy(self.plan)
        bad["segments"].pop()
        self.assert_code("PLAN_STRUCTURE", lambda: build_reference_sequence(self.rehash(bad)))
        for rate in [0, -1, 10.1, True, float("nan")]:
            self.assert_code("REFERENCE_RATE", lambda rate=rate: build_reference_sequence(self.plan, rate_hz=rate))
        self.assert_code("REFERENCE_BUDGET", lambda: build_reference_sequence(self.plan, max_references=2))


if __name__ == "__main__":
    unittest.main()
