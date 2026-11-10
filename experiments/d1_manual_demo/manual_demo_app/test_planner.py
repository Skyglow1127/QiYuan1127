import json
import math
from pathlib import Path
import tempfile
import unittest

from planner import PlanValidationError, canonical_plan_sha256, prepare_session


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="d1_planner_offline_test_")
        self.addCleanup(self.temp.cleanup)
        self.session = Path(self.temp.name)
        self.qs = [
            [0., -10., 20., -3., 4., -5., 20.],
            [2., -12., 21., -4., 4., -5., 25.],
            [-1., -11., 23., -5., 4., -4., 22.],
        ]
        self.write(self.qs)

    def write(self, qs, times=None, events=None, summary=None):
        times = times or [.1*i for i in range(len(qs))]
        rows = [{"sample": i+1, "elapsed_s": t, "dds_received_monotonic_s": 100+t,
                 "q": q, "gap_before": False} for i, (q, t) in enumerate(zip(qs, times))]
        (self.session/"joint_samples.jsonl").write_text("\n".join(json.dumps(row) for row in rows)+"\n")
        (self.session/"summary.json").write_text(json.dumps(summary or {"state": "stopped", "subscriber_pipe_drained": True}))
        (self.session/"metadata.json").write_text(json.dumps({"joint_order": [f"J{i}" for i in range(7)], "joint_units": ["deg"]*6+["sdk_raw"]}))
        (self.session/"events.jsonl").write_text("\n".join(json.dumps(event) for event in (events or [])))

    def assert_code(self, code, callback):
        with self.assertRaises(PlanValidationError) as caught:
            callback()
        self.assertEqual(caught.exception.code, code)

    def test_speed_up_retains_acceleration_bound_and_sources(self):
        before = {p.name: p.read_bytes() for p in self.session.iterdir()}
        slow = prepare_session(self.session)
        fast = prepare_session(self.session, speed_deg_s=2.)
        self.assertLess(fast["duration"], slow["duration"])
        self.assertTrue(fast["arm_timing_constraints_satisfied"])
        for segment in fast["segments"]:
            self.assertGreaterEqual(segment["T"], segment["original_dt"])
            self.assertGreaterEqual(segment["T"], .1)
            self.assertLessEqual(max(segment["maximum_absolute_velocity"][:6]), 2.+1e-12)
            self.assertLessEqual(max(segment["maximum_absolute_acceleration"][:6]), 1.+1e-12)
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.session.iterdir()})
        self.assertFalse(fast["actual_motion_replay_ready"])
        self.assertIsNone(fast["gripper_timing"]["verified_velocity_limit"])
        self.assertEqual(fast["plan_sha256"], canonical_plan_sha256(fast))
        self.assertEqual(fast["plan_sha256"], prepare_session(self.session, 2.)["plan_sha256"])

    def test_shared_quintic_endpoints_direction_and_derivatives(self):
        plan = prepare_session(self.session, 1.)
        for index, segment in enumerate(plan["segments"]):
            if index:
                self.assertEqual(plan["segments"][index-1]["q1"], segment["q0"])
                self.assertEqual(plan["segments"][index-1]["plan_t1"], segment["plan_t0"])
            self.assertEqual(segment["q0"], self.qs[index])
            self.assertEqual(segment["q1"], self.qs[index+1])
            for u in [0., .1, .2113248654051871, .5, .7886751345948129, .9, 1.]:
                blend=10*u**3-15*u**4+6*u**5
                first=30*u*u*(1-u)*(1-u)
                second=60*u-180*u*u+120*u**3
                for axis, (a,b) in enumerate(zip(segment["q0"], segment["q1"])):
                    q=a+(b-a)*blend
                    self.assertGreaterEqual(q, min(a,b)-1e-12)
                    self.assertLessEqual(q, max(a,b)+1e-12)
                    if axis < 6:
                        self.assertLessEqual(abs((b-a)*first/segment["T"]), 1.+1e-12)
                        self.assertLessEqual(abs((b-a)*second/segment["T"]**2), 1.+1e-12)
                    if u in [0.,1.]:
                        self.assertAlmostEqual(q, a if u==0. else b)
                        self.assertAlmostEqual(first,0.)
                        self.assertAlmostEqual(second,0.)
        self.assertEqual(plan["start_q"],self.qs[0])
        self.assertEqual(plan["end_q"],self.qs[-1])

    def test_first_marker_selects_formal_segment_without_trimming_static(self):
        bad_preparation=[0.,0.,0.,0.,0.,-168.,20.]
        self.write([bad_preparation,*self.qs],events=[{"label":"开始演示","elapsed_s":.1},
                                                    {"label":"开始演示","elapsed_s":.2},
                                                    {"label":"grasp_recording_ended","elapsed_s":.31}])
        plan=prepare_session(self.session)
        self.assertEqual(plan["point_count"],3)
        self.assertEqual(plan["selection"]["omitted_preparation_rows"],1)
        self.assertEqual(plan["original_points"][0]["source_line"],2)
        self.assertAlmostEqual(plan["source_duration"],.2)
        self.assertAlmostEqual(plan["source_marker_duration"],.21)
        self.write([self.qs[0],self.qs[0],self.qs[1]])
        whole=prepare_session(self.session)
        self.assertEqual(whole["point_count"],3)
        self.assertEqual(whole["selection"]["mode"],"whole_session_no_start_marker")
        self.assertEqual(whole["segments"][0]["q0"],whole["segments"][0]["q1"])

    def test_gap_nonmonotonic_limit_and_nan_are_rejected(self):
        cases=[('FEEDBACK_GAP',self.qs,[0.,.6,.7]),
               ('NONMONOTONIC_TIME',self.qs,[0.,.1,.1]),
               ('JOINT_LIMIT',[[0.,0.,0.,0.,0.,0.,50.1],*self.qs[1:]],None),
               ('JOINT_LIMIT',[[0.,0.,0.,0.,0.,-168.,20.],*self.qs[1:]],None),
               ('INVALID_Q',[[float('nan'),0.,0.,0.,0.,0.,20.],*self.qs[1:]],None)]
        for code,qs,times in cases:
            with self.subTest(code=code):
                self.write(qs,times)
                self.assert_code(code,lambda:prepare_session(self.session))

    def test_unended_and_invalid_speed(self):
        (self.session/'summary.json').unlink()
        self.assert_code('SESSION_NOT_ENDED',lambda:prepare_session(self.session))
        self.write(self.qs,summary={'state':'recording'})
        self.assert_code('SESSION_NOT_ENDED',lambda:prepare_session(self.session))
        self.write(self.qs)
        for value in [0.,-1.,float('nan'),float('inf'),True]:
            self.assert_code('INVALID_SPEED',lambda value=value:prepare_session(self.session,value))


if __name__ == '__main__':
    unittest.main()
