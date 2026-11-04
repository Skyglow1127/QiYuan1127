"""VLA runtime wiring tests with synthetic RGB, clocks and transport only.

The final test uses the existing real executor, SDK codec and both physical
Guards against MemoryReceiver; no socket, camera, model or CAN is opened.
"""
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
import socket
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from offline_verification import (Clock, DEFAULTS, LIMITS, MemoryReceiver,
                                  Observation, PiperPolicyExecutor,
                                  PiperPolicyExecutorConfig, READINESS)
from nativequeuepolicy import NativeQueueTiming
from lerobot_robot_piper_executor.guard import GuardError
from visual_gripper_runtime import (ALIASES, KEYS, MAX_DEFER_S, WRIST_SERIAL,
                                    visual_gripper_control)


CONFIG = {
    'center_x_range': [.40, .60], 'center_y_min': .70, 'bottom_y_min': .95,
    'area_fraction_range': [.03, .20], 'second_largest_ratio_max': .25,
    'opening_seen_m': .035, 'closing_delta_min_m': .001, 'near_frames': 2,
    'max_joint_error_deg': 3.,
    'grasp_joint_bounds_deg': [[-27, -14], [96, 113], [-71, -46], [-4, 13], [35, 67], [-34, -12]],
    'reset_opening_m': .055, 'reset_j1_max_deg': -10.,
}
JOINTS = (-20., 101., -58., 2., 51., -25.)


def image(near=False):
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    y = 350 if near else 150
    rgb[y:y+110, 270:370] = (255, 0, 0)
    return rgb


def camera(frame_id=1):
    return SimpleNamespace(config=SimpleNamespace(serial=WRIST_SERIAL), last_index=frame_id)


class Trace:
    """In-memory implementation of the executor's existing trace interface."""
    def __init__(self):
        self.records, self.timing_objects = [], []

    def start(self, *, raw_prediction, observed_state, sampled_at, timing, requested_at):
        self.timing_objects.append(timing)
        return {'raw_prediction': deepcopy(raw_prediction),
                'observed_state': deepcopy(observed_state), 'sampled_at': sampled_at,
                'requested_at': requested_at, 'act_timing': asdict(timing),
                'backend_ack_at': None, 'outcome': None,
                'transport_outcome_unknown': False, 'error': None}

    def append(self, record):
        self.records.append(deepcopy(record))

    def close(self):
        pass


class Robot:
    """Synthetic logical sender; each call is one explicit test request."""
    def __init__(self, state=(*JOINTS, .06)):
        self.clock = Clock(10.)
        self.backend = SimpleNamespace(cameras={'wrist': camera()})
        self.guard = SimpleNamespace(armed=True, last_target=tuple(state))
        self.latest = Observation({**dict(zip(KEYS, state)), 'wrist': image()}, self.clock())
        self.policy_target_trace = Trace()
        self.policy_action_timing = None
        self.calls = []
        self.failure = None

    def send_action(self, action):
        timing = self.policy_action_timing
        self.calls.append((dict(action), timing))
        record = self.policy_target_trace.start(
            raw_prediction=dict(zip(KEYS, (action[key] for key in ALIASES))),
            observed_state={key: self.latest.values[key] for key in KEYS},
            sampled_at=self.latest.sampled_at, timing=timing, requested_at=self.clock())
        self.policy_action_timing = None
        if self.failure is not None:
            record.update(outcome='rejected', error=str(self.failure), transport_outcome_unknown=True)
            self.policy_target_trace.append(record)
            raise self.failure
        self.guard.last_target = tuple(action[key] for key in ALIASES)
        record.update(outcome='accepted', backend_ack_at=self.clock())
        self.policy_target_trace.append(record)
        return dict(action)


def prepare(robot, width=.02, *, near=False, frame=1, state=None, joints=JOINTS):
    if state is None:
        state = tuple(robot.latest.values[key] for key in KEYS)
    robot.latest = Observation({**dict(zip(KEYS, state)), 'wrist': image(near)}, robot.clock())
    robot.backend.cameras['wrist'].last_index = frame
    timing = NativeQueueTiming(robot.clock()-.2, robot.clock(), robot.clock(), 0, 20, 20.)
    robot.policy_action_timing = timing
    return dict(zip(ALIASES, (*joints, width))), timing


class VisualGripperRuntimeTests(unittest.TestCase):
    def setUp(self):
        # This is a hard offline boundary even if a future helper is changed.
        self.network = patch.object(socket.socket, 'connect', side_effect=AssertionError('No network in visual runtime tests'))
        self.network.start()
        self.addCleanup(self.network.stop)

    def test_far_closure_preserves_six_axes_timing_source_and_separate_raw_trace(self):
        robot = Robot()
        changed_joints = (-19.5, 101.5, -58.5, 2.5, 51.5, -24.5)
        action, timing = prepare(robot, joints=changed_joints)
        action_before, timing_before = deepcopy(action), asdict(timing)
        source_before = robot.latest.sampled_at
        with visual_gripper_control(robot, CONFIG) as counts:
            result = robot.send_action(action)
            self.assertEqual(counts, {'calls': 1, 'held': 1, 'closing_started': 0})
        transmitted, passed_timing = robot.calls[0]
        self.assertIs(passed_timing, timing)
        self.assertIs(robot.policy_target_trace.timing_objects[0], timing)
        self.assertEqual(asdict(timing), timing_before)
        self.assertIsNone(robot.policy_action_timing)
        self.assertEqual(robot.latest.sampled_at, source_before)
        self.assertEqual(tuple(transmitted[key] for key in ALIASES[:6]), changed_joints)
        self.assertEqual(transmitted['gripper.pos'], .06)
        self.assertEqual(result, transmitted)
        self.assertEqual(action, action_before)
        record = robot.policy_target_trace.records[0]
        self.assertEqual(tuple(record['raw_prediction'].values()), (*changed_joints, .02))
        self.assertEqual(tuple(record['visual_gated_prediction'].values()), (*changed_joints, .06))
        self.assertEqual(record['act_timing'], timing_before)
        self.assertEqual(record['visual_gripper_gate']['source_sampled_at'], source_before)
        self.assertFalse(record['visual_gripper_gate']['phase_before']['opening_seen'])
        self.assertTrue(record['visual_gripper_gate']['phase_candidate']['opening_seen'])

    def test_rejected_opening_does_not_commit_intent_or_retry_original(self):
        robot = Robot(state=(*JOINTS, .02))
        failure = GuardError('SYNTHETIC unknown transport outcome')
        with visual_gripper_control(robot, CONFIG) as counts:
            action, _ = prepare(robot, width=.055)
            robot.failure = failure
            with self.assertRaises(GuardError) as caught:
                robot.send_action(action)
            self.assertIs(caught.exception, failure)
            self.assertEqual(len(robot.calls), 1)
            self.assertEqual(counts, {'calls': 0, 'held': 0, 'closing_started': 0})
            # A separately requested synthetic next call exposes the unchanged
            # phase. It is not an automatic retry of the rejected opening.
            robot.failure = None
            action, _ = prepare(robot, width=.019, frame=2)
            robot.send_action(action)
            record = robot.policy_target_trace.records[-1]
            self.assertFalse(record['visual_gripper_gate']['phase_before']['opening_seen'])
            self.assertEqual(record['visual_gated_prediction']['gripper.m'], .019)
            self.assertEqual(counts['calls'], 1)

    def test_rejected_closure_does_not_latch_phase_or_advance_counts(self):
        robot = Robot()
        with visual_gripper_control(robot, CONFIG) as counts:
            action, _ = prepare(robot, near=True, frame=1)
            robot.send_action(action)  # First near frame succeeds, still held.
            action, _ = prepare(robot, near=True, frame=2)
            robot.failure = GuardError('SYNTHETIC send rejected')
            with self.assertRaises(GuardError):
                robot.send_action(action)
            self.assertEqual(len(robot.calls), 2)
            self.assertEqual(counts, {'calls': 1, 'held': 1, 'closing_started': 0})
            rejected = robot.policy_target_trace.records[-1]['visual_gripper_gate']
            self.assertTrue(rejected['phase_candidate']['closing_started'])
            self.assertFalse(rejected['phase_before']['closing_started'])
            robot.failure = None
            action, _ = prepare(robot, near=False, frame=3)
            robot.send_action(action)
            last = robot.policy_target_trace.records[-1]
            self.assertFalse(last['visual_gripper_gate']['phase_before']['closing_started'])
            self.assertEqual(last['visual_gated_prediction']['gripper.m'], .06)
            self.assertEqual(counts['closing_started'], 0)

    def test_context_restores_inherited_and_instance_owned_methods_after_error(self):
        for own_methods in (False, True):
            robot = Robot()
            original, trace_start = robot.send_action, robot.policy_target_trace.start
            if own_methods:
                robot.send_action = original
                robot.policy_target_trace.start = trace_start
            with self.subTest(instance_owned=own_methods):
                with self.assertRaisesRegex(RuntimeError, 'SYNTHETIC context exit'):
                    with visual_gripper_control(robot, CONFIG):
                        self.assertIn('send_action', vars(robot))
                        self.assertIn('start', vars(robot.policy_target_trace))
                        raise RuntimeError('SYNTHETIC context exit')
                self.assertEqual(robot.send_action, original)
                self.assertEqual(robot.policy_target_trace.start, trace_start)
                self.assertEqual('send_action' in vars(robot), own_methods)
                self.assertEqual('start' in vars(robot.policy_target_trace), own_methods)

    def test_eight_second_defer_timeout_happens_before_original_send_or_new_trace(self):
        robot = Robot()
        with visual_gripper_control(robot, CONFIG) as counts:
            action, _ = prepare(robot)
            robot.send_action(action)
            robot.clock.now += MAX_DEFER_S + .001
            action, timing = prepare(robot, frame=2)
            with self.assertRaisesRegex(GuardError, 'within8s'):
                robot.send_action(action)
            self.assertEqual(len(robot.calls), 1)
            self.assertEqual(len(robot.policy_target_trace.records), 1)
            self.assertIs(robot.policy_action_timing, timing)
            self.assertEqual(counts, {'calls': 1, 'held': 1, 'closing_started': 0})

    def test_opening_successful_closure_carry_and_release_pass_through(self):
        robot = Robot(state=(*JOINTS, .02))
        with visual_gripper_control(robot, CONFIG) as counts:
            action, _ = prepare(robot, width=.06)
            self.assertEqual(robot.send_action(action)['gripper.pos'], .06)
            for frame in (2, 3):
                action, _ = prepare(robot, near=True, frame=frame, state=(*JOINTS, .06))
                self.assertEqual(robot.send_action(action)['gripper.pos'], .06 if frame == 2 else .02)
            carry = (30., 122., -91., 3., 66., -55.)
            for frame, width in ((4, .025), (5, .07)):
                action, _ = prepare(robot, width=width, frame=frame, state=(*carry, .025), joints=carry)
                self.assertEqual(robot.send_action(action), action)
                self.assertTrue(robot.policy_target_trace.records[-1]['visual_gripper_gate']['phase_before']['closing_started'])
            self.assertEqual(counts, {'calls': 5, 'held': 1, 'closing_started': 1})
            decisions = [r['visual_gripper_gate']['decision'] for r in robot.policy_target_trace.records]
            self.assertEqual(decisions, ['opening_passthrough', 'closure_held', 'closure_latched',
                                         'latched_passthrough', 'latched_passthrough'])

    def test_wrong_camera_identity_rejects_before_original_send(self):
        robot = Robot()
        robot.backend.cameras['wrist'].config.serial = 'SYNTHETIC wrong camera'
        with visual_gripper_control(robot, CONFIG) as counts:
            action, timing = prepare(robot)
            with self.assertRaisesRegex(GuardError, 'camera identity'):
                robot.send_action(action)
            self.assertEqual(robot.calls, [])
            self.assertEqual(robot.policy_target_trace.records, [])
            self.assertIs(robot.policy_action_timing, timing)
            self.assertEqual(counts['calls'], 0)

    def test_real_executor_sdk_and_independent_receiver_guards_keep_original_timing(self):
        clock = Clock(10.)
        anchor = (*JOINTS, .06)
        receiver = MemoryReceiver(clock, anchor=anchor, state=anchor)
        receiver.config = deepcopy(DEFAULTS['command_conditioning'])
        receiver.cameras = {'wrist': camera()}
        robot = PiperPolicyExecutor(PiperPolicyExecutorConfig(id='PURE OFFLINE VISUAL GATE'),
                                    backend=receiver, limits=LIMITS, clock=clock, sleep=clock.sleep)
        robot.connected = robot.require_policy_timing = True
        robot.policy_target_trace = Trace()
        robot.guard.arm(anchor, sampled_at=clock(), now=clock(), readiness=READINESS)
        clock.now = 10.2
        raw_joints = (-19.5, 101.5, -58.5, 2.5, 51.5, -24.5)
        action, timing = prepare(robot, joints=raw_joints, state=anchor)
        original_timing = asdict(timing)
        with visual_gripper_control(robot, CONFIG) as counts:
            result = robot.send_action(action)
            self.assertEqual(counts, {'calls': 1, 'held': 1, 'closing_started': 0})
        self.assertEqual(len(receiver.accepted), 1)
        host_target, receiver_target = receiver.accepted[0]
        self.assertEqual(host_target, receiver_target)
        self.assertEqual(host_target, (*raw_joints, .06))
        self.assertEqual(tuple(result.values()), host_target)
        self.assertIsNone(robot.guard.pending)
        self.assertIsNone(receiver.guard.pending)
        self.assertTrue(robot.guard.armed)
        self.assertTrue(receiver.guard.armed)
        self.assertIsNone(robot.policy_action_timing)
        self.assertIs(robot.policy_target_trace.timing_objects[0], timing)
        self.assertEqual(asdict(timing), original_timing)
        record = robot.policy_target_trace.records[0]
        self.assertEqual(record['outcome'], 'accepted')
        self.assertFalse(record['transport_outcome_unknown'])
        self.assertEqual(record['act_timing'], original_timing)
        self.assertEqual(record['sampled_at'], 10.2)
        self.assertEqual(record['proposal_at'], timing.scheduled_at)
        self.assertAlmostEqual(record['expires_at'], timing.scheduled_at + .4)
        self.assertEqual(record['raw_prediction']['gripper.m'], .02)
        self.assertEqual(record['visual_gated_prediction']['gripper.m'], .06)
        self.assertEqual(record['final_target']['gripper.m'], .06)
        self.assertEqual(tuple(record['raw_prediction'][k] for k in KEYS[:6]), raw_joints)
        self.assertEqual(tuple(record['visual_gated_prediction'][k] for k in KEYS[:6]), raw_joints)


if __name__ == '__main__':
    unittest.main(verbosity=2)
