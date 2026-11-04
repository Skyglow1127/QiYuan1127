"""Offline runtime integration checks for coordinated PI05 closure.

No hardware, model service, socket, camera or CAN is opened. Logical failure
paths use a synthetic sender; the last test passes through the real SDK codec
and independent host/receiver ActionGuards via the existing MemoryReceiver.
"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

BRIDGE = Path('/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge')
sys.path.insert(0, str(BRIDGE))
from test_visual_gripper_runtime import (CONFIG, JOINTS, Robot, Trace, camera, image,
    Clock, DEFAULTS, LIMITS, MemoryReceiver, Observation, PiperPolicyExecutor,
    PiperPolicyExecutorConfig, READINESS)
from nativequeuepolicy import NativeQueueTiming
from lerobot_robot_piper_executor.guard import GuardError
from visual_gripper_runtime import ALIASES, KEYS, visual_gripper_control


class SnapshotClient:
    def __init__(self):
        self.next_snapshot = None
        self.calls = []

    def request(self, op, **kwargs):
        self.calls.append((op, deepcopy(kwargs)))
        if op != 'snapshot':
            raise AssertionError('Only the already-required snapshot RPC is allowed')
        return deepcopy(self.next_snapshot)

    def host_source_time(self, source_ns):
        return source_ns / 1e9

    def require_clock(self):
        pass


class Policy:
    def __init__(self):
        self._action_queue = deque(['discard old lift', 'discard old reopening'])
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1
        self._action_queue.clear()


def closure_config():
    return deepcopy(json.loads((BRIDGE/'pi05_execution_defaults.json').read_text())['vla_closure_sync'])


def attach(robot):
    robot.backend.client = SnapshotClient()
    return robot


def prepare(robot, *, actual=.06, width=.02, near=True, frame=1,
            joints=JOINTS, observed_joints=JOINTS, age=0., index=0,
            healthy=True, force=-.08, model_actual=None, model_age=0.):
    state = (*observed_joints, actual)
    sampled_at = robot.clock() - age
    client = robot.backend.client
    client.next_snapshot = {
        'state': list(state), 'source_ns': int(sampled_at*1e9),
        'group_ages_s': [age]*5,
        'healthy': healthy, 'health_reasons': [] if healthy else ['synthetic fault'],
        'status': {'ctrl_mode': 1, 'arm_status': 0, 'mode_feedback': 1,
            'teach_status': 0, 'motion_status': 0, 'err_code': 0,
            'gripper_status': 0x40, 'gripper_mode': 0,
            'gripper_force_n': force, 'gripper_raw': round(actual*1e6)},
    }
    # The new runtime owns one explicit read-only snapshot RPC before sending.
    # This observation is the older model/camera acquisition; it must retain its
    # original timestamp even when execution uses newer seven-axis feedback.
    model_state = (*observed_joints, actual if model_actual is None else model_actual)
    robot.latest = Observation({**dict(zip(KEYS, model_state)), 'wrist': image(near)}, robot.clock()-model_age)
    robot.backend.cameras['wrist'].last_index = frame
    if isinstance(robot.backend, MemoryReceiver):
        robot.backend.state = state
    # A valid source-derived slot from this model acquisition. The 200ms
    # inference-age case is explicit via model_age; no source is refreshed.
    source = robot.latest.sampled_at - index/20.
    timing = NativeQueueTiming(source, robot.clock(), robot.clock(), index, 20, 20.)
    robot.policy_action_timing = timing
    return dict(zip(ALIASES, (*joints, width))), timing


def step(robot, **kwargs):
    action, timing = prepare(robot, **kwargs)
    before = asdict(timing)
    result = robot.send_action(action)
    assert asdict(timing) == before, 'Runtime mutated original model timing'
    return result


def latch(robot, *, actual=.06):
    step(robot, actual=actual, frame=1)
    robot.clock.now += .05
    return step(robot, actual=actual, frame=2)


def settle_until_reset(robot, policy):
    for frame in range(3, 20):
        robot.clock.now += .05
        step(robot, frame=frame, actual=.0445)
        if policy.reset_count:
            return frame
    raise AssertionError('Fresh stable candidate never requested queue reset')


class ClosureSyncRuntimeIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.network = patch.object(socket.socket, 'connect', side_effect=AssertionError('No network in offline closure integration'))
        self.network.start()
        self.addCleanup(self.network.stop)
        self.config = closure_config()

    def context(self, robot, policy=None):
        return visual_gripper_control(robot, CONFIG, closure_config=self.config,
                                      policy=policy or Policy())

    def test_exactly_one_fresh_readonly_snapshot_rpc_and_no_extra_motion_rpc(self):
        robot = attach(Robot())
        client = robot.backend.client
        original = client.request
        with self.context(robot):
            step(robot, near=False)
            self.assertEqual([op for op, _ in client.calls], ['snapshot'])
            self.assertEqual(len(robot.calls), 1)
        self.assertEqual(client.request, original)
        self.assertNotIn('request', vars(client))

    def test_fresh_snapshot_updates_execution_feedback_without_refreshing_model_or_camera_source(self):
        robot = attach(Robot())
        with self.context(robot):
            action, timing = prepare(robot, actual=.058, model_actual=.06,
                model_age=.2, near=False)
            old_sampled_at = robot.latest.sampled_at
            before = asdict(timing)
            robot.send_action(action)
        record = robot.policy_target_trace.records[-1]
        self.assertEqual(record['model_input_observed_state']['gripper.m'], .06)
        self.assertEqual(record['model_input_sampled_at'], old_sampled_at)
        self.assertEqual(record['observed_state']['gripper.m'], .058)
        self.assertEqual(record['sampled_at'], old_sampled_at)
        self.assertEqual(record['act_timing'], before)
        self.assertIs(robot.policy_target_trace.timing_objects[-1], timing)

    def test_latched_closure_freezes_measured_pose_and_preserves_original_trace_timing(self):
        robot = attach(Robot())
        changed = (-19.5, 101.5, -58.5, 2.5, 51.5, -24.5)
        with self.context(robot):
            step(robot, frame=1, joints=changed)
            robot.clock.now += .05
            action, timing = prepare(robot, frame=2, joints=changed)
            before = asdict(timing)
            result = robot.send_action(action)
        self.assertEqual(tuple(result[k] for k in ALIASES[:6]), JOINTS)
        record = robot.policy_target_trace.records[-1]
        self.assertEqual(tuple(record['raw_prediction'][k] for k in KEYS[:6]), changed)
        self.assertEqual(record['raw_prediction']['gripper.m'], .02)
        self.assertEqual(record['act_timing'], before)
        self.assertIs(robot.policy_target_trace.timing_objects[-1], timing)
        self.assertIsNone(robot.policy_action_timing)

    def test_reopening_and_lift_are_deferred_until_actual_closure(self):
        robot = attach(Robot())
        with self.context(robot):
            latch(robot)
            robot.clock.now += .05
            lift = (-20., 90., -58., 2., 51., -42.)
            result = step(robot, actual=.056, width=.07, frame=3, joints=lift)
            self.assertEqual(tuple(result[k] for k in ALIASES[:6]), JOINTS)
            self.assertLessEqual(result['gripper.pos'], .02)
            self.assertEqual(robot.policy_target_trace.records[-1]['raw_prediction']['gripper.m'], .07)

    def test_rejected_latch_does_not_commit_freeze_or_reset_policy(self):
        robot, policy = attach(Robot()), Policy()
        with self.context(robot, policy):
            step(robot, frame=1)
            robot.clock.now += .05
            action, _ = prepare(robot, frame=2)
            robot.failure = GuardError('synthetic ACK unknown')
            with self.assertRaisesRegex(GuardError, 'ACK unknown'):
                robot.send_action(action)
            self.assertEqual(policy.reset_count, 0)
            robot.failure = None
            robot.clock.now += .05
            changed = (-19.5, 101.5, -58.5, 2.5, 51.5, -24.5)
            result = step(robot, frame=3, near=False, joints=changed)
            self.assertEqual(tuple(result[k] for k in ALIASES[:6]), changed)
            self.assertEqual(policy.reset_count, 0)

    def test_fresh_stable_cube_width_resets_fifo_once_after_ack(self):
        robot, policy = attach(Robot(state=(*JOINTS, .046))), Policy()
        with self.context(robot, policy):
            latch(robot, actual=.046)
            frame = settle_until_reset(robot, policy)
            self.assertGreaterEqual(frame, 9, 'Insufficient stable duration confirmed grasp')
            self.assertEqual(policy.reset_count, 1)
            self.assertFalse(policy._action_queue)
            self.assertEqual(robot.policy_target_trace.records[-1]['outcome'], 'accepted')
            robot.clock.now += .05
            step(robot, frame=frame+1, actual=.0445, index=0)
            self.assertEqual(policy.reset_count, 1)

    def test_confirmed_grasp_rejects_reopening_on_fresh_replan_at_anchor(self):
        robot, policy = attach(Robot(state=(*JOINTS, .046))), Policy()
        with self.context(robot, policy):
            latch(robot, actual=.046)
            frame = settle_until_reset(robot, policy)
            self.assertEqual(policy.reset_count, 1)
            calls = len(robot.calls)
            robot.clock.now += .05
            action, _ = prepare(robot, frame=frame+1, actual=.0445, width=.06, index=0)
            with self.assertRaises(GuardError):
                robot.send_action(action)
            self.assertEqual(len(robot.calls), calls)

    def test_confirmation_ack_failure_does_not_reset_policy(self):
        robot, policy = attach(Robot(state=(*JOINTS, .046))), Policy()
        with self.context(robot, policy):
            latch(robot, actual=.046)
            for frame in range(3, 8):
                robot.clock.now += .05
                step(robot, frame=frame, actual=.0445)
            self.assertEqual(policy.reset_count, 0)
            robot.clock.now += .11
            action, _ = prepare(robot, frame=8, actual=.0445)
            robot.failure = GuardError('synthetic confirmation ACK failure')
            with self.assertRaisesRegex(GuardError, 'confirmation ACK failure'):
                robot.send_action(action)
            self.assertEqual(policy.reset_count, 0)
            self.assertTrue(policy._action_queue)
            self.assertEqual(robot.policy_target_trace.records[-1]['local_closure_sync']['phase_candidate'], 'contact_ready')
            self.assertTrue(robot.policy_target_trace.records[-1]['local_closure_sync']['request_replan'])

    def test_confirmed_grasp_rejects_old_fifo_index_instead_of_faking_new_prediction(self):
        robot, policy = attach(Robot(state=(*JOINTS, .046))), Policy()
        with self.context(robot, policy):
            latch(robot, actual=.046)
            frame = settle_until_reset(robot, policy)
            self.assertEqual(policy.reset_count, 1)
            calls = len(robot.calls)
            robot.clock.now += .05
            action, _ = prepare(robot, frame=frame+1, actual=.0445, index=5)
            with self.assertRaises(GuardError):
                robot.send_action(action)
            self.assertEqual(len(robot.calls), calls)

    def test_carry_reopening_stops_after_leaving_anchor_but_model_release_passes_in_placement_window(self):
        for placement in (False, True):
            robot, policy = attach(Robot(state=(*JOINTS, .046))), Policy()
            with self.subTest(placement=placement), self.context(robot, policy):
                latch(robot, actual=.046)
                frame = settle_until_reset(robot, policy)
                robot.clock.now += .05
                step(robot, frame=frame+1, actual=.0445, index=0)
                joints = (30., 122., -91., 3., 66., -55.) if placement else (-15., 90., -60., 3., 52., -42.)
                robot.clock.now += .05
                action, _ = prepare(robot, frame=frame+2, actual=.0445,
                    width=.07, joints=joints, observed_joints=joints, near=False)
                calls = len(robot.calls)
                if placement:
                    result = robot.send_action(action)
                    self.assertEqual(result, action)
                    self.assertEqual(len(robot.calls), calls+1)
                else:
                    with self.assertRaises(GuardError):
                        robot.send_action(action)
                    self.assertEqual(len(robot.calls), calls)

    def test_missing_stale_or_invalid_snapshot_rejects_before_send(self):
        for condition in ('missing', 'stale', 'invalid', 'unhealthy'):
            robot = attach(Robot())
            with self.subTest(condition=condition), self.context(robot):
                action, _ = prepare(robot,
                    age=.3 if condition=='stale' else 0., healthy=condition!='unhealthy')
                if condition == 'missing':
                    del robot.backend.client.next_snapshot['status']
                elif condition == 'invalid':
                    robot.backend.client.next_snapshot['state'] = [0.]*6
                with self.assertRaises(GuardError):
                    robot.send_action(action)
                self.assertEqual(robot.calls, [])

    def test_empty_closure_anchor_drift_and_timeout_stop_before_send(self):
        for condition in ('empty', 'drift', 'timeout'):
            robot = attach(Robot())
            with self.subTest(condition=condition), self.context(robot):
                latch(robot)
                robot.clock.now += 4.01 if condition == 'timeout' else .05
                observed_joints = (-16.9, *JOINTS[1:]) if condition == 'drift' else JOINTS
                action, _ = prepare(robot, frame=3,
                    actual=.034 if condition=='empty' else .055,
                    observed_joints=observed_joints)
                calls = len(robot.calls)
                with self.assertRaises(GuardError):
                    robot.send_action(action)
                self.assertEqual(len(robot.calls), calls)

    def test_exception_restores_all_wrapped_methods(self):
        robot = attach(Robot())
        saved = (robot.send_action, robot.policy_target_trace.start, robot.backend.client.request)
        with self.assertRaisesRegex(RuntimeError, 'context error'):
            with self.context(robot):
                raise RuntimeError('context error')
        self.assertEqual((robot.send_action, robot.policy_target_trace.start, robot.backend.client.request), saved)
        for owner, name in ((robot, 'send_action'), (robot.policy_target_trace, 'start'), (robot.backend.client, 'request')):
            self.assertNotIn(name, vars(owner))

    def test_real_sdk_and_both_guards_freeze_arm_keep_feedback_cap_and_original_timing(self):
        clock = Clock(10.)
        anchor = (*JOINTS, .06)
        receiver = MemoryReceiver(clock, anchor=anchor, state=anchor)
        receiver.config = deepcopy(DEFAULTS['command_conditioning'])
        receiver.cameras = {'wrist': camera()}
        robot = attach(PiperPolicyExecutor(PiperPolicyExecutorConfig(id='PURE OFFLINE CLOSURE SYNC'),
            backend=receiver, limits=LIMITS, clock=clock, sleep=clock.sleep))
        robot.connected = robot.require_policy_timing = True
        robot.policy_target_trace = Trace()
        robot.guard.arm(anchor, sampled_at=clock(), now=clock(), readiness=READINESS)
        clock.now = 10.2
        with self.context(robot):
            latch(robot)
            clock.now += .05
            action, timing = prepare(robot, frame=3, actual=.059,
                joints=(-19.5, 90., -59., 3., 52., -42.), width=.07)
            before = asdict(timing)
            result = robot.send_action(action)
        host, remote = receiver.accepted[-1]
        self.assertEqual(host, remote)
        self.assertEqual(host[:6], JOINTS)
        self.assertLessEqual(abs(host[6]-.059), .003+1e-12)
        self.assertLess(host[6], .059)
        self.assertEqual(tuple(result.values()), host)
        record = robot.policy_target_trace.records[-1]
        self.assertEqual(record['act_timing'], before)
        self.assertIs(robot.policy_target_trace.timing_objects[-1], timing)
        self.assertEqual(record['proposal_at'], timing.scheduled_at)
        self.assertAlmostEqual(record['expires_at'], timing.scheduled_at+.4)
        self.assertTrue(robot.guard.armed)
        self.assertTrue(receiver.guard.armed)
        self.assertIsNone(robot.guard.pending)
        self.assertIsNone(receiver.guard.pending)


if __name__ == '__main__':
    unittest.main(verbosity=2)
