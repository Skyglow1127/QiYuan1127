"""Offline tests only. Test values are synthetic, NOT approved robot limits."""

import math
from pathlib import Path
import struct
import tempfile
import unittest

from lerobot_robot_piper_executor.guard import (
    ACTION_KEYS, ActionGuard, GuardError, GuardLimits, LocalOwner, Readiness)
from lerobot_robot_piper_executor.sdk_codec import encode_j_position_speed, encode_target


def limits():
    return GuardLimits(bounds=((-10., 10.),) * 6 + ((-.01, .1),),
        max_step=(1.,) * 6 + (.002,), max_target_speed=(20.,) * 6 + (.04,),
        max_tracking_error=(2.,) * 6 + (.004,),
        max_observation_age_s=.15, max_command_age_s=.15, max_command_gap_s=.25)


def ready():
    return Readiness(**{name: "SYNTHETIC OFFLINE TEST" for name in Readiness.__dataclass_fields__})


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = ActionGuard(limits())
        self.zero = (0.,) * 7

    def arm(self):
        self.guard = ActionGuard(limits())
        self.guard.arm(self.zero, sampled_at=1., now=1., readiness=ready())

    def prepare(self, action, **kw):
        args = dict(observed_state=self.zero, sampled_at=1., proposal_at=1., now=1.05)
        args.update(kw)
        return self.guard.prepare(action, **args)

    def test_disarmed_and_incomplete_review(self):
        with self.assertRaises(GuardError): self.prepare(self.zero)
        with self.assertRaises(GuardError):
            self.guard.arm(self.zero, sampled_at=1., now=1., readiness=Readiness())
        self.assertFalse(self.guard.armed)

    def test_anchor_and_commit_without_clipping(self):
        self.arm()
        target = (.5, 0., 0., 0., 0., 0., .001)
        cmd = self.prepare(dict(zip(ACTION_KEYS, target)))
        self.assertEqual(self.guard.last_target, self.zero)
        self.assertEqual(cmd.target, target)
        self.guard.commit(cmd, now=1.06)
        self.assertEqual(self.guard.last_target, target)
        self.assertIsNone(self.guard.pending)

    def test_nonfinite_wrong_dimension_and_keys(self):
        for target in [(math.nan,) + self.zero[1:], self.zero[:6], {"wrong": 0}]:
            with self.subTest(target=target):
                self.arm()
                with self.assertRaises(GuardError): self.prepare(target)
                self.assertFalse(self.guard.armed)

    def test_bounds_step_tracking(self):
        for target, state in [((11.,) + self.zero[1:], self.zero),
                              ((1.1,) + self.zero[1:], self.zero),
                              ((.5,) + self.zero[1:], (-2.,) + self.zero[1:])]:
            with self.subTest(target=target):
                self.arm()
                with self.assertRaises(GuardError): self.prepare(target, observed_state=state)
                self.assertFalse(self.guard.armed)

    def test_fast_repeated_calls_cannot_bypass_target_speed(self):
        self.arm()
        with self.assertRaisesRegex(GuardError, "target rate"):
            self.prepare((.5,) + self.zero[1:], now=1.001)

    def test_stale_feedback_proposal_future_clock_and_gap(self):
        for args in [dict(sampled_at=.8), dict(proposal_at=.8), dict(sampled_at=1.1),
                     dict(now=1.3), dict(now=.9)]:
            with self.subTest(args=args):
                self.guard = ActionGuard(limits())
                self.arm()
                with self.assertRaises(GuardError): self.prepare(self.zero, **args)
                self.assertFalse(self.guard.armed)

    def test_expired_and_replayed_command(self):
        self.arm()
        cmd = self.prepare(self.zero)
        with self.assertRaises(GuardError): self.guard.require_sendable(cmd, now=1.16)
        self.assertFalse(self.guard.armed)
        self.guard = ActionGuard(limits())
        self.arm()
        cmd = self.prepare(self.zero)
        self.guard.commit(cmd, now=1.06)
        with self.assertRaises(GuardError): self.guard.commit(cmd, now=1.07)

    def test_uncertain_transport_is_not_committed(self):
        self.arm()
        self.prepare((.5,) + self.zero[1:])
        with self.assertRaises(GuardError): self.prepare(self.zero, now=1.06)
        self.assertEqual(self.guard.last_target, self.zero)

    def test_local_owner_exclusion_and_release(self):
        with tempfile.TemporaryDirectory() as folder:
            first, second = LocalOwner(Path(folder)/"owner"), LocalOwner(Path(folder)/"owner")
            first.acquire()
            try:
                with self.assertRaises(GuardError): second.acquire()
            finally: first.release()
            second.acquire()
            second.release()


class SDKCodecTests(unittest.TestCase):
    def test_j_pos_vel_low_speed_is_explicit(self):
        frame = encode_j_position_speed(speed_percent=5, installation_pos=1)
        self.assertEqual(frame.arbitration_id, 0x151)
        self.assertEqual(frame.data, bytes([1, 1, 5, 0, 0, 1, 0, 0]))
        with self.assertRaises(ValueError): encode_j_position_speed(speed_percent=50, installation_pos=1)

    def test_absolute_degrees_and_signed_gripper_width(self):
        # Includes the negative-width encoding present in the data; encoding
        # support does not approve sending it before physical mapping review.
        frames = encode_target([1., -2., 3., -4., 5., -6., -.0052], gripper_force_n=1.)
        self.assertEqual([f.arbitration_id for f in frames], [0x155, 0x156, 0x157, 0x159])
        for frame, expected in zip(frames[:3], [(1000,-2000),(3000,-4000),(5000,-6000)]):
            self.assertEqual(frame.data, struct.pack(">ii", *expected))
        self.assertEqual(frames[3].data, struct.pack(">iHBB", -5200, 1000, 1, 0))


class LeRobotInterfaceTests(unittest.TestCase):
    def test_default_has_no_hardware_backend(self):
        from lerobot_robot_piper_executor.robot import PiperPolicyExecutor, PiperPolicyExecutorConfig, ROLLOUT_KEYS
        from unittest.mock import Mock, patch
        robot = PiperPolicyExecutor(PiperPolicyExecutorConfig(id="offline-test"))
        self.assertFalse(robot.is_connected)
        self.assertEqual(tuple(robot.action_features), ROLLOUT_KEYS)
        self.assertEqual(robot.observation_features["wrist"], (480, 640, 3))
        backend = Mock(guard_limits=None)
        with patch("lerobot_robot_piper_executor.backend.PiperBackend.from_file", return_value=backend):
            robot.connect()
        backend.open_read_only.assert_called_once_with()
        with self.assertRaises(GuardError): robot.send_action(dict.fromkeys(ACTION_KEYS, 0.))
        robot.disconnect()

    def test_injected_fake_backend_routes_only_guarded_target(self):
        from lerobot_robot_piper_executor.robot import (
            Observation, PiperPolicyExecutor, PiperPolicyExecutorConfig, ROLLOUT_KEYS)

        class Image:
            shape = (480, 640, 3)

        class FakeBackend:
            def __init__(self): self.sent, self.invalidations = [], []
            def open_read_only(self): pass
            def observation(self):
                return Observation({**dict.fromkeys(ACTION_KEYS, 0.),
                    "global": Image(), "wrist": Image()}, now[0])
            def require_execution_lease(self, readiness): readiness.require_complete()
            def send_checked(self, command): self.sent.append(command)
            def invalidate_execution_lease(self, reason): self.invalidations.append(reason)
            def close(self): pass

        now = [1.]
        backend = FakeBackend()
        robot = PiperPolicyExecutor(PiperPolicyExecutorConfig(id="offline-test"),
            backend=backend, limits=limits(), clock=lambda: now[0])
        robot.connect()
        try:
            robot.get_observation()
            with self.assertRaises(GuardError): robot.send_action(dict.fromkeys(ACTION_KEYS, 0.))
            self.assertEqual(backend.sent, [])
            robot.arm_from_current_state(ready())
            now[0] = 1.05
            action = dict.fromkeys(ROLLOUT_KEYS, 0.)
            action[ROLLOUT_KEYS[0]] = .5
            self.assertEqual(robot.send_action(action), action)
            self.assertEqual(backend.sent[0].target, (.5, 0., 0., 0., 0., 0., 0.))
            self.assertEqual(len(backend.sent), 1)
            now[0] = 1.2
            with self.assertRaises(GuardError): robot.send_action(action)
            self.assertEqual(len(backend.sent), 1)
            self.assertFalse(robot.guard.armed)
            now[0] = 1.25
            robot.arm_from_current_state(ready())
            with self.assertRaises(GuardError): robot.arm_from_current_state(Readiness())
            self.assertFalse(robot.guard.armed)
            backend.config = {"max_relative_target": dict.fromkeys(ACTION_KEYS, .25)}
            robot.arm_from_current_state(ready())
            now[0] = 1.3
            action[ROLLOUT_KEYS[0]] = 9.
            capped = robot.send_action(action)
            self.assertEqual(capped[ROLLOUT_KEYS[0]], .25)
            self.assertEqual(len(backend.sent),2)
            robot.get_observation()
            action[ROLLOUT_KEYS[0]] = 20.
            with self.assertRaises(GuardError): robot.send_action(action)
            self.assertEqual(len(backend.sent),2)
        finally:
            robot.disconnect()

    def test_official_factory_preserves_seven_rollout_dimensions(self):
        from lerobot.robots.utils import make_robot_from_config
        from lerobot_robot_piper_executor.robot import PiperPolicyExecutorConfig, ROLLOUT_KEYS
        robot = make_robot_from_config(PiperPolicyExecutorConfig(id="offline-factory-test"))
        self.assertEqual(tuple(robot.action_features), ROLLOUT_KEYS)
        filtered = {k: v for k, v in robot.observation_features.items()
            if isinstance(v, tuple) or (v is float and k.endswith((".pos", ".vel")))}
        self.assertEqual([k for k in filtered if filtered[k] is float], list(ROLLOUT_KEYS))
        self.assertFalse(robot.is_connected)


if __name__ == "__main__":
    unittest.main()
