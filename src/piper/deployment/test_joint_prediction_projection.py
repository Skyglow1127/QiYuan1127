"""Finite policy proposals versus actual feedback; SDK bytes, no hardware.

Continuous following is synthetic, not a physical overshoot guarantee.
"""
import json
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

from act_chunk_timing import ACTActionTiming
from lerobot_robot_piper_executor.guard import ACTION_KEYS, ActionGuard, GuardError, GuardLimits
from lerobot_robot_piper_executor.policy_bounds import joint_policy_target_bounds, project_joint_predictions
from lerobot_robot_piper_executor.robot import Observation, ROLLOUT_KEYS
from test_executor import ready
import test_policy_target_execution as fixture


class JointProjectionTests(unittest.TestCase):
    def setUp(self):
        self.defaults = json.loads((Path(__file__).parent/'act_launch_defaults.json').read_text())
        self.limits = GuardLimits(**self.defaults['policy']['limits'])
        self.enterContext(patch('lerobot.robots.utils.logging.warning'))
        self.enterContext(patch('lerobot_robot_piper_executor.robot.LOG.warning'))

    def robot(self):
        robot, backend, clock = fixture.PolicyTargetExecutionTests().make_robot()
        state = (0., 60., -50., 0., 30., 0., .04)
        robot.guard = ActionGuard(self.limits)
        backend.remote_guard = ActionGuard(self.limits)
        robot.guard.arm(state, sampled_at=clock(), now=clock(), readiness=ready())
        backend.remote_guard.arm(state, sampled_at=clock()+backend.remote_offset,
            now=clock()+backend.remote_offset, readiness=ready())
        backend.feedback = state
        backend.config['policy_joint_target_margin_deg'] = 2.
        robot.latest = Observation(dict(zip(ACTION_KEYS, state)), clock())
        self.addCleanup(robot.disarm, 'synthetic projection done')
        return robot, backend, clock

    def fresh(self, robot, backend, clock):
        clock.now += .05
        robot.latest = Observation(dict(zip(ACTION_KEYS, backend.feedback)), clock()-.001)
        robot.policy_action_timing = ACTActionTiming(clock()-.05, clock(), clock(), 0, 20, 20.)

    def test_all_six_proposals_have_inward_bounds_and_gripper_is_strict(self):
        raw = (100., 150., -130., 50., 72., -100., .02)
        projected, corrections = project_joint_predictions(raw, limits=self.limits, margin_deg=2.)
        self.assertEqual(projected, (38., 133., -118., 43., 68., -78., .02))
        self.assertEqual(len(corrections), 6)
        self.assertEqual(joint_policy_target_bounds(self.limits, 2.)[0], (-58., 38.))
        for width in (-.000001, .100001):
            with self.assertRaisesRegex(GuardError, 'gripper.m'):
                project_joint_predictions((*raw[:6], width), limits=self.limits, margin_deg=2.)
        for index in range(7):
            for value in (float('nan'), float('inf'), -float('inf')):
                bad = list(raw); bad[index] = value
                with self.subTest(index=index, value=value), self.assertRaises(GuardError):
                    project_joint_predictions(bad, limits=self.limits, margin_deg=2.)

    def test_invalid_margin_never_produces_a_target(self):
        for margin in (True, None, '2', -.1, 5.001, float('nan'), float('inf'), 10**500):
            with self.subTest(margin=margin), self.assertRaises(GuardError):
                joint_policy_target_bounds(self.limits, margin)

    def test_reported_j1_prediction_goes_through_cap_codec_and_two_guards(self):
        robot, backend, clock = self.robot()
        state = (22.057, 112.304, -86.719, 9.846, 67.773, -63.679, .0433)
        for guard, offset in ((robot.guard, 0.), (backend.remote_guard, backend.remote_offset)):
            guard.arm(state, sampled_at=clock()+offset, now=clock()+offset, readiness=ready())
        backend.feedback = state
        self.fresh(robot, backend, clock)
        original_sample = robot.latest.sampled_at
        timing = robot.policy_action_timing
        action = dict(zip(ROLLOUT_KEYS, state)); action['joint_1.pos'] = 30.175174713134766
        action['joint_2.pos'] = 124.2865982055664
        result = robot.send_action(action)
        host, remote, frames, _ = backend.accepted[0]
        self.assertEqual(host.target, remote.target)
        self.assertLessEqual(result['joint_1.pos'], 38.)
        self.assertLessEqual(abs(host.target[0]-state[0]), 2.)
        self.assertLessEqual(struct.unpack('>ii', frames[0].data)[0], 38000)
        self.assertNotIn('joint_1.deg', robot.joint_projection_counts)
        self.assertNotIn('joint_2.deg', robot.joint_projection_counts)
        self.assertEqual(action['joint_1.pos'], 30.175174713134766)
        self.assertEqual(robot.latest.sampled_at, original_sample)
        self.assertAlmostEqual(host.expires_at, min(original_sample+.4, timing.scheduled_at+.4))

    def test_140_steps_across_chunks_and_reversals_remain_on_the_allowed_can_grid(self):
        robot, backend, clock = self.robot()
        bounds = joint_policy_target_bounds(self.limits, 2.)
        source = generated = None
        for index in range(140):
            slot, chunk = index % 20, index // 20
            clock.now += .048 if not index else .012
            sampled = clock()-.001
            robot.latest = Observation(dict(zip(ACTION_KEYS, backend.feedback)), sampled)
            if not slot:
                source = sampled; clock.now += .16; generated = clock()
            timing = ACTActionTiming(source, generated, clock(), slot, 20, 20.)
            robot.policy_action_timing = timing
            raw = tuple(hi+10 if chunk % 2 == 0 else lo-10 for lo, hi in self.limits.bounds[:6])
            robot.send_action(dict(zip(ROLLOUT_KEYS, (*raw, .08 if chunk % 2 == 0 else .005))))
            host, remote, frames, _ = backend.accepted[-1]
            decoded = tuple(v*.001 for frame in frames[:3] for v in struct.unpack('>ii', frame.data))
            self.assertEqual(host.target, remote.target)
            self.assertEqual(host.target[:6], decoded)
            for target, (lo, hi) in zip(decoded, bounds[:6], strict=True):
                self.assertLessEqual(lo, target); self.assertLessEqual(target, hi)
            self.assertAlmostEqual(host.expires_at, min(sampled+.4, timing.scheduled_at+.4))
            # Explicit diagnostic follow model: last command plus0.1deg.
            backend.feedback = tuple(v+.1 for v in host.target[:6]) + (host.target[6],)
        self.assertEqual(len(backend.accepted), 140)
        self.assertTrue(robot.guard.armed)
        self.assertTrue(backend.remote_guard.armed)

    def test_actual_feedback_above_j1_or_j5_bounds_is_never_projected(self):
        for index, value in ((0, 40.001), (4, 70.072)):
            robot, backend, clock = self.robot()
            state = list(backend.feedback); state[index] = value; backend.feedback = tuple(state)
            self.fresh(robot, backend, clock)
            with self.subTest(index=index), self.assertRaisesRegex(GuardError, 'outside.*no clipping'):
                robot.send_action(dict(zip(ROLLOUT_KEYS, backend.feedback)))
            self.assertEqual(backend.send_attempts, 0)

    def test_stale_slot_and_missing_timing_or_legacy_conflict_fail_before_transport(self):
        for case in ('stale', 'missing', 'legacy'):
            robot, backend, clock = self.robot()
            self.fresh(robot, backend, clock)
            if case == 'stale':
                clock.now += .5
                robot.latest = Observation(robot.latest.values, clock()-.001)
            elif case == 'missing':
                robot.require_policy_timing = False; robot.policy_action_timing = None
            else:
                backend.config['j5_upper_prediction_saturation'] = True
            with self.subTest(case=case), self.assertRaises(GuardError):
                robot.send_action(dict(zip(ROLLOUT_KEYS, backend.feedback)))
            self.assertEqual(backend.send_attempts, 0)

    def test_absent_joint_mode_preserves_original_raw_j1_rejection(self):
        robot, backend, clock = self.robot()
        del backend.config['policy_joint_target_margin_deg']
        self.fresh(robot, backend, clock)
        action = dict(zip(ROLLOUT_KEYS, backend.feedback))
        action['joint_1.pos'] = self.limits.bounds[0][1] + .175
        with self.assertRaisesRegex(GuardError, 'joint_1.deg.*outside'):
            robot.send_action(action)
        self.assertEqual(backend.send_attempts, 0)


if __name__ == '__main__':
    unittest.main()
