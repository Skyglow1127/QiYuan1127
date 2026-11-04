"""Recorded post-placement rejection and synthetic closing; no hardware.

Synthetic exact following does not establish real gripper overshoot bounds.
"""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import launch_act
from act_chunk_timing import ACTActionTiming
from lerobot_robot_piper_executor.guard import ACTION_KEYS, ActionGuard, GuardError, GuardLimits
from lerobot_robot_piper_executor.policy_bounds import joint_policy_target_bounds, project_joint_predictions
from lerobot_robot_piper_executor.robot import Observation, ROLLOUT_KEYS
from test_executor import ready
import test_policy_target_execution as fixture


class GripperProjectionTests(unittest.TestCase):
    # Last accepted target and next observed state/raw proposal, run005929.
    previous = (18.146, 98.119, -79.736, 5.966, 65.946, -48.111000000000004, .039)
    actual = (19.302, 99.244, -80.994, 5.909, 66.278, -48.716, .0409)
    raw = (7.082040786743164, 79.50973510742188, -74.70478057861328,
           5.96844482421875, 65.44891357421875, -47.43794250488281, -.0006396882236003876)

    def setUp(self):
        self.defaults = launch_act.read(launch_act.DEFAULTS)
        self.limits = GuardLimits(**self.defaults['policy']['limits'])
        self.enterContext(patch('lerobot.robots.utils.logging.warning'))
        self.enterContext(patch('lerobot_robot_piper_executor.robot.LOG.warning'))

    def make_robot(self):
        robot, backend, clock = fixture.PolicyTargetExecutionTests().make_robot()
        robot.guard, backend.remote_guard = ActionGuard(self.limits), ActionGuard(self.limits)
        for guard, offset in ((robot.guard, 0.), (backend.remote_guard, backend.remote_offset)):
            guard.arm(self.previous, sampled_at=clock()+offset, now=clock()+offset, readiness=ready())
        backend.feedback = self.actual
        backend.config.update(policy_joint_target_margin_deg=2., policy_gripper_target_margin_m=.001)
        robot.last_policy_command_at = clock()
        clock.now += .05
        robot.latest = Observation(dict(zip(ACTION_KEYS, self.actual)), clock()-.032048305)
        source = clock()-.828237402
        robot.policy_action_timing = ACTActionTiming(source, source+.210505101, clock(), 13, 20, 20.)
        self.addCleanup(robot.disarm, 'synthetic gripper projection done')
        return robot, backend, clock

    def test_learned_closed_widths_and_upper_proposals_project_inside_reviewed_bounds(self):
        bounds = joint_policy_target_bounds(self.limits, 2., .001)
        self.assertEqual(bounds[6], (.001, .099))
        for raw, expected in ((self.raw[6], .001), (-.005200000014156103, .001),
                              (-1e300, .001), (0., .001), (.04, .04),
                              (.099, .099), (.1, .099), (.1005, .099), (1e300, .099)):
            with self.subTest(raw=raw):
                action = self.actual[:6] + (raw,)
                projected, corrections = project_joint_predictions(action, limits=self.limits,
                    margin_deg=2., gripper_margin_m=.001)
                self.assertEqual(projected[6], expected)
                self.assertEqual(action[6], raw)
                gripper_corrections = [row for row in corrections if row['key'] == 'gripper.m']
                self.assertEqual(gripper_corrections,
                    [] if raw == expected else [{'key': 'gripper.m', 'original': raw, 'projected': expected}])
        for raw in (math.nan, math.inf, -math.inf):
            with self.subTest(raw=raw), self.assertRaises(GuardError):
                project_joint_predictions(self.actual[:6]+(raw,), limits=self.limits,
                    margin_deg=2., gripper_margin_m=.001)

    def test_absent_option_preserves_strict_gripper_input_and_invalid_margins_reject(self):
        with self.assertRaisesRegex(GuardError, 'gripper.m.*no prediction projection'):
            project_joint_predictions(self.raw, limits=self.limits, margin_deg=2.)
        for margin in (True, False, '0.001', -.000001, .005001, math.nan, math.inf):
            with self.subTest(margin=margin), self.assertRaisesRegex(GuardError, 'policy_gripper_target_margin_m'):
                project_joint_predictions(self.raw, limits=self.limits,
                    margin_deg=2., gripper_margin_m=margin)

    def test_recorded_rejected_frame_passes_codec_two_guards_and_keeps_original_deadline(self):
        robot, backend, clock = self.make_robot()
        temp = self.enterContext(tempfile.TemporaryDirectory())
        path = Path(temp)/'policy_targets.jsonl'
        backend.config['policy_target_trace_path'] = str(path)
        timing, sample = robot.policy_action_timing, robot.latest.sampled_at
        original_feedback = dict(robot.latest.values)
        action = dict(zip(ROLLOUT_KEYS, self.raw))
        result = robot.send_action(action)
        host, remote, frames, _ = backend.accepted[0]
        self.assertEqual(host.target, remote.target)
        # 1mm is only the proposal: current feedback's3mm cap stays effective.
        self.assertAlmostEqual(host.target[6], .037901)
        self.assertLessEqual(self.actual[6]-host.target[6], .00300000001)
        self.assertEqual(int.from_bytes(frames[3].data[:4], 'big', signed=True), 37901)
        self.assertEqual(result['gripper.pos'], host.target[6])
        self.assertEqual(action['gripper.pos'], self.raw[6])
        self.assertEqual(robot.latest.values, original_feedback)
        self.assertEqual(robot.latest.sampled_at, sample)
        self.assertAlmostEqual(host.expires_at, min(sample+.4, timing.scheduled_at+.4))
        robot.disarm('flush synthetic trace')
        row, = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(row['raw_prediction']['gripper.m'], self.raw[6])
        self.assertEqual(row['policy_projected_prediction']['gripper.m'], .001)
        self.assertEqual(row['policy_target_bounds']['gripper.m'], [.001, .099])
        self.assertEqual(row['policy_projection_counts']['gripper.m'], 1)
        self.assertEqual(row['outcome'], 'accepted')

    def test_synthetic_return_closing_converges_to_one_mm_with_original_guards(self):
        robot, backend, clock = self.make_robot()
        for index in range(40):
            robot.latest = Observation(dict(zip(ACTION_KEYS, backend.feedback)), clock()-.001)
            if index == 0 or index % 20 == 0:
                source = clock()-.001
                clock.now += .16
                generated = clock()
            robot.policy_action_timing = ACTActionTiming(source, generated, clock(), index % 20, 20, 20.)
            robot.send_action(dict(zip(ROLLOUT_KEYS, self.raw)))
            host, remote, _, _ = backend.accepted[-1]
            self.assertEqual(host.target, remote.target)
            self.assertGreaterEqual(host.target[6], .001)
            self.assertLessEqual(host.target[6], .099)
            clock.now += .012
        self.assertAlmostEqual(backend.feedback[6], .001)
        self.assertTrue(robot.guard.armed)
        self.assertTrue(backend.remote_guard.armed)

    def test_actual_negative_or_overwidth_feedback_is_never_projected(self):
        for actual in (-.000001, .100001):
            robot, backend, clock = self.make_robot()
            backend.feedback = self.actual[:6] + (actual,)
            robot.latest = Observation(dict(zip(ACTION_KEYS, backend.feedback)), clock()-.001)
            with self.subTest(actual=actual), self.assertRaisesRegex(GuardError, 'gripper.m.*no clipping'):
                robot.send_action(dict(zip(ROLLOUT_KEYS, self.raw)))
            self.assertEqual(backend.send_attempts, 0)

    def test_missing_mode_bad_margin_or_missing_safety_prerequisites_fail_before_transport(self):
        for case in ('mode', 'null', 'bool', 'stale', 'timing', 'cap', 'rate'):
            robot, backend, clock = self.make_robot()
            if case == 'mode':
                del backend.config['policy_joint_target_margin_deg']
            elif case == 'null':
                backend.config['policy_gripper_target_margin_m'] = None
            elif case == 'bool':
                backend.config['policy_gripper_target_margin_m'] = True
            elif case == 'stale':
                clock.now += .5
                robot.latest = Observation(dict(zip(ACTION_KEYS, backend.feedback)), clock()-.001)
            elif case == 'timing':
                robot.require_policy_timing = False
                robot.policy_action_timing = None
            elif case == 'cap':
                del backend.config['max_relative_target']
            else:
                del backend.config['policy_target_rate_fraction']
            with self.subTest(case=case), self.assertRaises(GuardError):
                robot.send_action(dict(zip(ROLLOUT_KEYS, self.raw)))
            self.assertEqual(backend.send_attempts, 0)


if __name__ == '__main__':
    unittest.main()
