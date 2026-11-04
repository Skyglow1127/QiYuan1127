"""Offline chunk timing tests; no robot, camera, SSH or CAN connection."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from act_chunk_timing import ACTActionTiming, track_act_chunks
from lerobot_robot_piper_executor.guard import ACTION_KEYS, GuardError, GuardLimits
from lerobot_robot_piper_executor.robot import Observation, PiperPolicyExecutor, PiperPolicyExecutorConfig, ROLLOUT_KEYS
from test_executor import limits, ready


class ChunkTimingTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.robot = SimpleNamespace(clock=lambda: self.now, latest=SimpleNamespace(sampled_at=9.99))
        owner = self

        class Policy:
            config = SimpleNamespace(n_action_steps=100, chunk_size=100, temporal_ensemble_coeff=None)

            def __init__(self):
                self._action_queue = deque()
                self.predictions = 0

            def select_action(self, batch):
                if not self._action_queue:
                    self.predictions += 1
                    owner.now += .16
                    self._action_queue.extend(range(self.config.n_action_steps))
                return self._action_queue.popleft()

        self.policy = Policy()

    def test_action_counts_keep_one_source_and_next_prediction_refreshes(self):
        for count in (10, 20, 30, 40, 100):
            with self.subTest(n_action_steps=count):
                self.now = 10.
                self.robot.latest = SimpleNamespace(sampled_at=9.99)
                self.policy = type(self.policy)()
                self.policy.config = SimpleNamespace(
                    n_action_steps=count, chunk_size=100, temporal_ensemble_coeff=None)
                with track_act_chunks(self.policy, self.robot, fps=20.):
                    for index in range(count):
                        if index:
                            self.now += .05
                        self.robot.latest = SimpleNamespace(sampled_at=self.now-.01)
                        self.assertEqual(self.policy.select_action({}), index)
                        timing = self.robot.policy_action_timing
                        self.assertEqual(timing.action_index, index)
                        self.assertEqual(timing.action_count, count)
                        self.assertEqual(timing.observed_at, 9.99)
                        self.assertAlmostEqual(
                            timing.proposal_at(now=self.now, max_age_s=.4),
                            9.99 + index/20.)
                        self.assertEqual(len(self.policy._action_queue), count-index-1)
                        self.robot.policy_action_timing = None
                    self.assertEqual(self.policy.predictions, 1)
                    self.now += .05
                    self.robot.latest = SimpleNamespace(sampled_at=self.now-.01)
                    self.assertEqual(self.policy.select_action({}), 0)
                    self.assertEqual(self.policy.predictions, 2)
                    self.assertEqual(self.robot.policy_action_timing.action_index, 0)
                    self.assertEqual(self.robot.policy_action_timing.action_count, count)
                    self.assertNotEqual(self.robot.policy_action_timing.observed_at, 9.99)
                    self.assertEqual(len(self.policy._action_queue), count-1)
                self.assertNotIn('select_action',vars(self.policy))
                self.assertFalse(self.robot.require_policy_timing)

    def test_fresh_feedback_does_not_revive_late_cached_action(self):
        with track_act_chunks(self.policy, self.robot, fps=20.):
            self.policy.select_action({})
            self.robot.policy_action_timing = None
            self.now += .5
            self.robot.latest = SimpleNamespace(sampled_at=self.now-.001)
            self.policy.select_action({})
            timing=self.robot.policy_action_timing
            self.assertEqual(timing.observed_at,9.99)
            with self.assertRaisesRegex(GuardError,'original scheduled window'):
                timing.proposal_at(now=self.now,max_age_s=.4)

    def test_unconsumed_action_cannot_be_replaced(self):
        with track_act_chunks(self.policy,self.robot,fps=20.):
            self.policy.select_action({})
            with self.assertRaisesRegex(GuardError,'not consumed'):
                self.policy.select_action({})

    def test_robot_uses_original_slot_deadline_and_consumes_metadata_once(self):
        backend=Mock()
        backend.config={}
        now=15.05
        robot=PiperPolicyExecutor(PiperPolicyExecutorConfig(id='offline-timing'),backend=backend,limits=limits(),clock=lambda:now)
        robot.connected=True
        robot.latest=Observation(dict.fromkeys(ACTION_KEYS,0.),now-.01)
        robot.guard.arm(dict.fromkeys(ACTION_KEYS,0.),sampled_at=now-.01,now=now,readiness=ready())
        robot.require_policy_timing=True
        robot.policy_action_timing=ACTActionTiming(10.,10.16,now,99,100,20.)
        robot.send_action(dict.fromkeys(ROLLOUT_KEYS,0.))
        command=backend.send_checked.call_args.args[0]
        self.assertAlmostEqual(command.expires_at,14.95+.15)
        self.assertIsNone(robot.policy_action_timing)
        with self.assertRaisesRegex(GuardError,'Missing original ACT chunk timing'):
            robot.send_action(dict.fromkeys(ROLLOUT_KEYS,0.))
        self.assertEqual(backend.send_checked.call_count,1)


class FakeClock:
    def __init__(self):
        self.now = 10.
        self.sleeps = []
        self.sleep_overrun = 0.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds + self.sleep_overrun


class CommandPacingTests(unittest.TestCase):
    """Synthetic limits/transport only; these values do not approve hardware."""

    def make_robot(self, *, source_at=9.84):
        clock = FakeClock()
        backend = Mock()
        backend.config = {}

        def send_checked(command):
            clock.now += .002  # Deterministic successful two-millisecond RPC.

        backend.send_checked.side_effect = send_checked
        synthetic_limits = GuardLimits(
            bounds=((-100., 100.),) * 6 + ((-.01, .1),),
            max_step=(6.,) * 6 + (.008,),
            max_target_speed=(45.,) * 6 + (.05,),
            max_tracking_error=(8.,) * 6 + (.015,),
            max_observation_age_s=.4, max_command_age_s=.4,
            max_command_gap_s=.75)
        robot = PiperPolicyExecutor(
            PiperPolicyExecutorConfig(id='offline-pacing'), backend=backend,
            limits=synthetic_limits, clock=clock, sleep=clock.sleep)
        robot.connected = True
        state = dict.fromkeys(ACTION_KEYS, 0.)
        state['joint_2.deg'] = 44.488
        robot.latest = Observation(state, clock.now - .001)
        robot.guard.arm(state, sampled_at=robot.latest.sampled_at,
                        now=clock.now, readiness=ready())
        robot.require_policy_timing = True
        robot.policy_action_timing = ACTActionTiming(
            source_at, 10., clock.now, 0, 100, 20.)
        action = dict.fromkeys(ROLLOUT_KEYS, 0.)
        action['joint_2.pos'] = 44.488
        return robot, backend, clock, action

    def prepare_cached(self, robot, clock, *, source_at=9.84, index=1):
        clock.now += .012  # Next call is 14 ms after the prior creation time.
        robot.latest = Observation(dict(robot.latest.values), clock.now - .001)
        robot.policy_action_timing = ACTActionTiming(
            source_at, 10., clock.now, index, 100, 20.)

    def test_inference_boundary_cached_target_waits_before_rate_check(self):
        robot, backend, clock, action = self.make_robot()
        robot.send_action(action)
        self.prepare_cached(robot, clock)
        action['joint_2.pos'] = 46.152

        result = robot.send_action(action)

        commands = [call.args[0] for call in backend.send_checked.call_args_list]
        self.assertEqual(len(commands), 2)
        self.assertAlmostEqual(clock.sleeps[0], .036)
        self.assertAlmostEqual(commands[1].created_at - commands[0].created_at, .05)
        self.assertAlmostEqual(result['joint_2.pos'], 46.152)
        self.assertTrue(robot.guard.armed)

    def test_wait_cannot_refresh_original_slot_or_feedback(self):
        for stale_source in ('slot', 'feedback'):
            with self.subTest(stale_source=stale_source):
                source_at = 9.64 if stale_source == 'slot' else 9.84
                robot, backend, clock, action = self.make_robot(source_at=source_at)
                robot.send_action(action)
                self.prepare_cached(robot, clock, source_at=source_at)
                if stale_source == 'slot':
                    # A delayed wakeup must not refresh the chunk's deadline.
                    clock.sleep_overrun = .06
                    expected = 'original scheduled window'
                else:
                    # Fresh enough at entry, expired when the pacing wait ends.
                    robot.latest = Observation(dict(robot.latest.values), 9.63)
                    self.assertLess(clock.now - robot.latest.sampled_at, .4)
                    expected = 'Stale, future or invalid timestamp'
                timing = robot.policy_action_timing
                timing.proposal_at(now=clock.now, max_age_s=.4)

                with self.assertRaisesRegex(GuardError, expected):
                    robot.send_action(action)

                self.assertEqual(backend.send_checked.call_count, 1)
                self.assertEqual(len(clock.sleeps), 1)
                self.assertFalse(robot.guard.armed)
                self.assertIsNone(robot.policy_action_timing)

    def test_100_frames_include_rpc_time_without_accumulating_slot_lateness(self):
        robot, backend, clock, action = self.make_robot()
        for index in range(100):
            if index:
                self.prepare_cached(robot, clock, index=index)
            robot.send_action(action)

        commands = [call.args[0] for call in backend.send_checked.call_args_list]
        self.assertEqual(len(commands), 100)
        for index, command in enumerate(commands):
            self.assertAlmostEqual(command.created_at, 10. + index / 20.)
            self.assertAlmostEqual(command.created_at - (9.84 + index / 20.), .16)
        self.assertAlmostEqual(clock.now, 14.952)
        self.assertAlmostEqual(robot.last_policy_command_at, 14.95)
        self.assertAlmostEqual(robot.guard.last_command_at, 14.952)

    def test_pacing_keeps_hard_speed_limit(self):
        robot, backend, clock, action = self.make_robot()
        robot.send_action(action)
        self.prepare_cached(robot, clock)
        action['joint_2.pos'] += 2.5  # Exceeds 45 deg/s at a 50 ms command period.

        with self.assertRaisesRegex(GuardError, 'joint_2.deg: target rate too large'):
            robot.send_action(action)

        self.assertAlmostEqual(clock.sleeps[0], .036)
        self.assertEqual(backend.send_checked.call_count, 1)
        self.assertFalse(robot.guard.armed)


if __name__=='__main__':
    unittest.main()
