#!/usr/bin/env python3
"""CPU-only native FIFO/timing/guard/identity verification. No physical IO.

Cloud responses, clocks and receiver transport are synthetic. Official queue
methods, saved config, physical guard and SDK encoder/decoder are real code.
"""
from copy import deepcopy
import hashlib
import io
import json
import logging
from pathlib import Path
import socket
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
DEPLOYMENT = Path('/home/aup/Lerobot/projects/piper/deployment')
sys.path[:0] = [str(HERE), str(DEPLOYMENT), str(DEPLOYMENT.parent/'official_adapter'),
               '/home/aup/piper SDK/pyAgxArm']
import client
import launch_grasp
import plan_workflow
from nativequeuepolicy import (CloudExpertPolicy, NativeQueueTiming, track_native_queue,
    QUEUE_CONTRACT, RUNTIME_ACTION_STEPS, CHUNK_SIZE, DATASET_FPS)
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.policies.pi05.configuration_pi05 import PI05Config
from lerobot_robot_piper_executor.guard import ACTION_KEYS, ActionGuard, GuardError, GuardLimits, Readiness
from lerobot_robot_piper_executor.robot import Observation, PiperPolicyExecutor, PiperPolicyExecutorConfig, ROLLOUT_KEYS
from lerobot_robot_piper_executor.remote_bridge import ControlEngine
from lerobot_robot_piper_executor.sdk_codec import encode_target
from newconditioning import CONDITIONING_ID, profile_sha256, condition_first_command

DEFAULTS = json.loads(client.DEFAULTS.read_text())
LIMITS = GuardLimits(**DEFAULTS['policy']['limits'])
STATE = (0., 10., -10., 0., 0., 0., .01)
READINESS = Readiness(**{name: 'SYNTHETIC OFFLINE TEST ONLY'
                         for name in Readiness.__dataclass_fields__})
FIXTURE = Path('/tmp/pi05_infer_v8_r23279/report.json')


class Clock:
    def __init__(self, now=100.):
        self.now = now
    def __call__(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


class FakeCloud:
    def __init__(self, clock=None):
        self.calls, self.clock = [], clock
        self.bad = None
    def infer(self, state, global_image, wrist_image, task):
        self.calls.append(np.asarray(state).copy())
        if self.clock:
            self.clock.now += .18
        actions = np.tile(np.asarray(STATE, dtype=np.float32), (50, 1))
        actions[:, 0] = np.arange(50)+100*(len(self.calls)-1)
        if self.bad is not None:
            actions = self.bad
        return {'actions': actions, 'request_id': f'synthetic-{len(self.calls)}',
            'timing_s': {'model': .1, 'input_ready_to_validated_actions': .18}}


def proxy(cloud=None):
    config = launch_grasp.load_proxy_config(launch_grasp.CHECKPOINT)
    return CloudExpertPolicy(config, cloud or FakeCloud())


def batch(state=STATE):
    return {'task': client.TASK, 'observation.state': np.asarray(state),
            'observation.images.global': np.zeros((3, 224, 224), dtype=np.uint8),
            'observation.images.wrist': np.zeros((3, 224, 224), dtype=np.uint8)}


def recorded_frame():
    item = deepcopy(json.loads(FIXTURE.read_text())['requests'][0])
    metadata = item['source_metadata']
    observed = {'kind': 'live', 'state': np.asarray(metadata['snapshot']['state'], dtype=np.float32),
        'sampled_at': min(metadata['backend_sampled_at'], metadata['state_at'],
            *(camera['estimated_capture_monotonic'] for camera in metadata['camera_metadata'].values())),
        'source_metadata': metadata}
    # Use a synthetic constant in-bounds chunk to isolate each check. This is
    # never represented as an actual model success or a new live observation.
    observed['state'] = np.asarray(STATE, dtype=np.float32)
    result = {**item, 'actions': np.tile(observed['state'], (50, 1))}
    return observed, result


class MemoryReceiver:
    """No transport: real SDK bytes, real decoder and independent receiver guard."""
    config = {}
    def __init__(self, clock, anchor=STATE, state=STATE):
        self.clock, self.state, self.accepted = clock, tuple(state), []
        self.guard = ActionGuard(LIMITS)
        self.guard.arm(anchor, sampled_at=clock(), now=clock(), readiness=READINESS)
    def send_checked(self, command):
        encoded = encode_target(command.target, gripper_force_n=1.)
        frames = [{'id': item.arbitration_id, 'data': item.data.hex()} for item in encoded]
        decoder = SimpleNamespace(policy=SimpleNamespace(max_gripper_force_n=1.))
        _, target = ControlEngine._decode_transaction(decoder, frames)
        now = self.clock()
        checked = self.guard.prepare(target, observed_state=self.state,
            sampled_at=now, proposal_at=now, now=now)
        self.guard.commit(checked, now=now)
        self.accepted.append((command.target, checked.target))
    def invalidate_execution_lease(self, reason):
        self.guard.disarm('SYNTHETIC cleanup: '+reason)


class NativeQueueTests(unittest.TestCase):
    def test_task_workspace_only_changes_three_reviewed_upper_bounds(self):
        archived = json.loads((HERE/'before_task_workspace_v8/pi05_execution_defaults.json').read_text())
        old = deepcopy(archived['policy']['limits'])
        expected = deepcopy(old)
        expected['bounds'][0][1], expected['bounds'][1][1], expected['bounds'][4][1] = 40.,135.,70.
        self.assertEqual(DEFAULTS['policy']['limits'], expected)
        candidate = deepcopy(DEFAULTS['policy'])
        candidate['limits']['bounds'] = old['bounds']
        self.assertEqual(candidate, archived['policy'])
        self.assertEqual(DEFAULTS['command_conditioning'], archived['command_conditioning'])
        for field in ('tx_ttl_s','normal_gripper_force_n','default_duration_s','verified_controller_joint_bounds_deg'):
            self.assertEqual(DEFAULTS[field], archived[field])
        launch_grasp.validate_limits(DEFAULTS['policy']['limits'])
        self.assertFalse(DEFAULTS['workspace_candidate']['physical_approval_granted'])
        proof = json.loads(Path(DEFAULTS['workspace_candidate']['controller_limits_source']).read_text())
        source = json.loads(Path(DEFAULTS['workspace_candidate']['basis_manifest']).read_text())
        self.assertEqual(expected['bounds'], source['settings']['policy']['limits']['bounds'])
        for axis,(lo,hi) in enumerate(expected['bounds'][:6],1):
            self.assertGreaterEqual(lo,proof['joint_limits'][str(axis)]['min_deg'])
            self.assertLessEqual(hi,proof['joint_limits'][str(axis)]['max_deg'])
        self.assertEqual([expected['bounds'][i][1]-2 for i in (0,1,4)],[38.,133.,68.])

    def test_recorded_placement_goals_pass_real_sdk_dual_guards_beyond_old_envelope(self):
        # Fixed follower feedback from dataset episode35/frame298; no model,
        # future feedback simulation, transport or physical movement. Goal0 is
        # the numeric placement-window median, goal1 that recorded row's action.
        measured = (30.677000045776367,122.81300354003906,-91.01799774169922,
                    3.5739998817443848,67.46900177001953,-54.606998443603516,.043699998408555984)
        goals = ((30.873,122.728,-91.614,4.519,66.917,-56.935,.0252),
                 (30.38800048828125,122.8479995727539,-91.5979995727539,
                  3.2290000915527344,66.90799713134766,-54.36000061035156,.025100000202655792))
        clock = Clock(10.)
        receiver = MemoryReceiver(clock,measured,measured)
        receiver.config = deepcopy(DEFAULTS['command_conditioning'])
        robot = PiperPolicyExecutor(PiperPolicyExecutorConfig(id='PURE OFFLINE TASK WORKSPACE'),
            backend=receiver,limits=LIMITS,clock=clock,sleep=clock.sleep)
        robot.connected,robot.require_policy_timing = True,True
        robot.guard.arm(measured,sampled_at=clock(),now=clock(),readiness=READINESS)
        for index,goal in enumerate(goals):
            clock.now = 10.2+index*.2
            robot.latest = Observation(dict(zip(ACTION_KEYS,measured)),clock())
            robot.policy_action_timing = NativeQueueTiming(clock()-.1,clock()-.05,clock(),0,20,20.)
            sent = robot.send_action(dict(zip(ROLLOUT_KEYS,goal)))
            host,decoded = receiver.accepted[-1]
            self.assertEqual(host,decoded)
            self.assertEqual(tuple(sent.values()),host)
            for axis,old_upper in ((0,28.),(1,118.),(4,63.)):
                self.assertGreater(host[axis],old_upper)
                self.assertAlmostEqual(host[axis],goal[axis],delta=.001001)
            self.assertTrue(all(abs(x-y)<=cap+1e-9 for x,y,cap in
                zip(host,measured,(2.,)*6+(.003,),strict=True)))
        self.assertEqual(len(receiver.accepted),2)
        self.assertEqual(receiver.state,measured)
        self.assertIsNone(robot.guard.pending)

    def test_task_workspace_actual_invalid_values_and_missing_approval_still_reject(self):
        from lerobot_robot_piper_executor.guard import CheckedCommand
        invalid = [(40.001,*STATE[1:]),(150.001,*STATE[1:]),
                   (STATE[0],135.001,*STATE[2:]),(*STATE[:4],70.001,*STATE[5:]),
                   (float('nan'),*STATE[1:])]
        for target in invalid:
            with self.subTest(target=target):
                host=ActionGuard(LIMITS)
                host.arm(STATE,sampled_at=10.,now=10.,readiness=READINESS)
                with self.assertRaises(GuardError):
                    host.prepare(target,observed_state=STATE,sampled_at=10.,proposal_at=10.,now=10.2)
                receiver=MemoryReceiver(Clock(10.),STATE,STATE)
                receiver.clock.now=10.2
                with self.assertRaises((GuardError,ValueError,OverflowError)):
                    receiver.send_checked(CheckedCommand(1,target,10.2,10.4))
                self.assertFalse(receiver.accepted)
        unapproved=ActionGuard(LIMITS)
        with self.assertRaisesRegex(GuardError,'review incomplete'):
            unapproved.arm(STATE,sampled_at=10.,now=10.,readiness=Readiness())
        self.assertFalse(unapproved.armed)
        altered=deepcopy(DEFAULTS['policy']['limits'])
        altered['bounds'][4][1]=70.001
        with self.assertRaises(ValueError):
            launch_grasp.validate_limits(altered)

    def test_inherited_official_fifo_consumes_runtime_twenty_and_forty_calls_use_two_chunks(self):
        cloud, policy = FakeCloud(), None
        policy = proxy(cloud)
        self.assertIs(type(policy).select_action, PI05Policy.select_action)
        self.assertIs(type(policy).reset, PI05Policy.reset)
        self.assertEqual(sum(parameter.numel() for parameter in policy.parameters()), 0)
        actual = [policy.select_action(batch((index, *STATE[1:])))[0, 0].item() for index in range(40)]
        self.assertEqual(actual, list(range(20))+list(range(100, 120)))
        self.assertEqual(len(cloud.calls), 2)
        self.assertEqual([call[0] for call in cloud.calls], [0, 20])
        self.assertFalse(policy._action_queue)

    def test_saved_config_ten_is_preserved_and_runtime_twenty_is_explicit_override(self):
        before = hashlib.sha256((launch_grasp.CHECKPOINT/'config.json').read_bytes()).hexdigest()
        config = launch_grasp.load_proxy_config(launch_grasp.CHECKPOINT)
        self.assertEqual(config.n_action_steps, 20)
        self.assertEqual(json.loads((launch_grasp.CHECKPOINT/'config.json').read_text())['n_action_steps'],10)
        self.assertEqual(QUEUE_CONTRACT['n_action_steps_source'],'explicit user-authorized runtime reduction20; checkpoint saved10')
        self.assertEqual(PI05Config.__dataclass_fields__['n_action_steps'].default, 50)
        self.assertEqual((config.chunk_size, config.num_inference_steps, config.dtype), (50, 10, 'bfloat16'))
        self.assertEqual(hashlib.sha256((launch_grasp.CHECKPOINT/'config.json').read_bytes()).hexdigest(), before)
        for count in (1, 10, 50):
            config.n_action_steps = count
            with self.assertRaises(ValueError):
                CloudExpertPolicy(config, FakeCloud())

    def test_reset_discards_all_cached_targets_using_official_method(self):
        cloud = FakeCloud()
        policy = proxy(cloud)
        policy.select_action(batch())
        self.assertEqual(len(policy._action_queue), 19)
        policy.reset()
        self.assertEqual(policy.select_action(batch())[0, 0].item(), 100.)
        self.assertEqual(len(cloud.calls), 2)

    def test_timing_tracks_original_chunk_source_while_current_feedback_changes(self):
        clock = Clock()
        policy = proxy(FakeCloud(clock))
        robot = SimpleNamespace(latest=None, policy_action_timing=None,
            require_policy_timing=False, guard=ActionGuard(LIMITS), clock=clock)
        timings = []
        with track_native_queue(policy, robot, fps=DATASET_FPS):
            for index in range(40):
                robot.latest = Observation(dict(zip(ACTION_KEYS, STATE)), clock())
                policy.select_action(batch())
                timing = robot.policy_action_timing
                timings.append(timing)
                self.assertEqual(timing.action_index, index % 20)
                self.assertAlmostEqual(timing.slot_lateness_s(now=clock()), .18)
                self.assertEqual(timing.proposal_at(now=clock(), max_age_s=.4), timing.scheduled_at)
                robot.policy_action_timing = None  # Synthetic consumer, no hardware send.
                clock.now += .05
        self.assertEqual(len(policy.client.calls), 2)
        self.assertEqual({timing.observed_at for timing in timings[:20]}, {100.})
        self.assertAlmostEqual(timings[19].selected_at-timings[19].observed_at, 1.13)
        self.assertGreater(timings[20].observed_at, timings[0].observed_at)
        self.assertFalse(policy._action_queue)
        self.assertFalse(robot.require_policy_timing)

    def test_late_cached_slot_cannot_be_refreshed_by_a_new_feedback_frame(self):
        clock = Clock()
        policy = proxy(FakeCloud(clock))
        robot = SimpleNamespace(latest=Observation(dict(zip(ACTION_KEYS, STATE)), clock()),
            policy_action_timing=None, require_policy_timing=False, guard=ActionGuard(LIMITS), clock=clock)
        with track_native_queue(policy, robot, fps=20.):
            for index in range(19):
                robot.latest = Observation(dict(zip(ACTION_KEYS, STATE)), clock())
                policy.select_action(batch())
                robot.policy_action_timing = None
                clock.now += .05
            # slot19 derives from original100 +19/20=100.95, not newfeedback.
            clock.now = 101.351
            robot.latest = Observation(dict(zip(ACTION_KEYS, STATE)), clock())
            with self.assertRaisesRegex(GuardError, 'missed its original'):
                policy.select_action(batch())
        self.assertFalse(policy._action_queue)

    def test_invalid_future_expired_or_nonfinite_slot_metadata_rejects(self):
        timing = NativeQueueTiming(10., 10.2, 10.5, 6, 20, 20.)
        self.assertEqual(timing.proposal_at(now=10.5, max_age_s=.4), 10.3)
        self.assertAlmostEqual(timing.original_source_age_s(now=10.5), .5)
        self.assertAlmostEqual(timing.slot_lateness_s(now=10.5), .2)
        for values, now in (((10., 10.2, 10.2, 19, 20, 20.), 10.2),
                            ((10., 10.2, 10.9, 1, 20, 20.), 10.9),
                            ((10., 10.2, 10.3, 20, 20, 20.), 10.3),
                            ((10., 10.2, float('nan'), 1, 20, 20.), 10.3)):
            with self.subTest(values=values), self.assertRaises(GuardError):
                NativeQueueTiming(*values).proposal_at(now=now, max_age_s=.4)

    def test_invalid_cloud_chunks_are_never_enqueued_or_repaired(self):
        for bad in (np.zeros((49, 7)), np.full((50, 7), np.nan), np.full((50, 7), np.inf)):
            cloud = FakeCloud()
            cloud.bad = bad
            policy = proxy(cloud)
            with self.subTest(shape=bad.shape), self.assertRaises(RuntimeError):
                policy.select_action(batch())
            self.assertFalse(policy._action_queue)

    def test_existing_act_adapter_generates_guarded_sdk_targets_without_changing_raw_fifo(self):
        clock = Clock(10.)
        boundary = (0.,0.,0.,0.,0.,0.,.01)
        receiver = MemoryReceiver(clock, boundary, boundary)
        receiver.config = deepcopy(DEFAULTS['command_conditioning'])
        robot = PiperPolicyExecutor(PiperPolicyExecutorConfig(id='PURE OFFLINE ACT BRIDGE'),
            backend=receiver, limits=LIMITS, clock=clock, sleep=clock.sleep)
        robot.connected, robot.require_policy_timing = True, True
        robot.latest = Observation(dict(zip(ACTION_KEYS,boundary)), clock())
        robot.guard.arm(boundary, sampled_at=clock(),now=clock(),readiness=READINESS)
        raw = (.123456,-1.377,.473,-1.031,.817,-.413,.009998)
        cloud = FakeCloud()
        cloud.bad = np.tile(np.asarray(raw,dtype=np.float32),(50,1))
        policy = proxy(cloud)
        for index in range(40):
            clock.now = 10.2+index*.05
            # Synthetic feedback follows the last accepted target exactly;
            # this tests real adapter/SDK/dual Guards, not physical tracking.
            measured = receiver.guard.last_target
            receiver.state = measured
            robot.latest = Observation(dict(zip(ACTION_KEYS,measured)),clock())
            chunk_source = 10.+(index//20)*1.
            selected = policy.select_action(batch(measured))[0].tolist()
            np.testing.assert_array_equal(np.asarray(selected,dtype=np.float32),np.asarray(raw,dtype=np.float32))
            robot.policy_action_timing = NativeQueueTiming(chunk_source,chunk_source+.2,clock(),index%20,20,20.)
            sent = robot.send_action(dict(zip(ROLLOUT_KEYS,selected)))
            host,decoded = receiver.accepted[-1]
            self.assertEqual(host,decoded)
            self.assertEqual(tuple(sent.values()),host)
            self.assertGreaterEqual(host[1],0.)
            self.assertLessEqual(host[2],0.)
            self.assertTrue(all(abs(a-b)<=maximum+1e-10 for a,b,maximum in
                zip(host,measured,(2.,)*6+(.003,),strict=True)))
            self.assertEqual(raw[1],-1.377)
        self.assertEqual(len(receiver.accepted),40)
        self.assertEqual(len(cloud.calls),2)
        self.assertFalse(policy._action_queue)
        self.assertAlmostEqual(receiver.accepted[-1][0][1],2.)
        self.assertAlmostEqual(receiver.accepted[-1][0][2],-2.)

    def test_latest_ten_terminal_raw_chunks_are_not_rejected_before_act_adaptation(self):
        report=json.loads(FIXTURE.read_text())
        self.assertEqual(len(report['requests']),10)
        for item in report['requests']:
            metadata=item['source_metadata']
            observed={'kind':'live','state':np.asarray(metadata['snapshot']['state']),
                'sampled_at':min(metadata['backend_sampled_at'],metadata['state_at'],
                    *(camera['estimated_capture_monotonic'] for camera in metadata['camera_metadata'].values())),
                'source_metadata':metadata}
            result={**item,'actions':np.asarray(item['response']['actions'])}
            before=result['actions'].copy()
            assessment=client.evaluate_prediction(observed,result)
            self.assertFalse(assessment['consumed_prefix_bounds_pass'])
            self.assertTrue(assessment['raw_prefix_bounds_are_diagnostic_only'])
            self.assertTrue(assessment['preview_numeric_and_source_checks_pass'],assessment['rejection_reasons'])
            self.assertTrue(assessment['first_command_conditioning']['numerical_guard_prepare_pass'])
            self.assertFalse(assessment['fresh_feedback_after_inference_verified'])
            np.testing.assert_array_equal(result['actions'],before)

    def test_host_and_sdk_receiver_keep_actual_bounds_step_rate_tracking_and_feedback_age(self):
        from lerobot_robot_piper_executor.guard import CheckedCommand
        cases = ((STATE, (LIMITS.bounds[0][1]+.001, *STATE[1:]), STATE, .2, 'outside'),
                 (STATE, (6.001, *STATE[1:]), STATE, .2, 'step too large'),
                 (STATE, (1., *STATE[1:]), STATE, .01, 'rate too large'),
                 ((9., *STATE[1:]), (9., *STATE[1:]), STATE, .2, 'too far from current feedback'))
        for anchor, target, observed, elapsed, reason in cases:
            guard = ActionGuard(LIMITS)
            guard.arm(anchor, sampled_at=10., now=10., readiness=READINESS)
            with self.assertRaisesRegex(GuardError, reason):
                guard.prepare(target, observed_state=observed, sampled_at=10., proposal_at=10., now=10.+elapsed)
            receiver = MemoryReceiver(Clock(10.), anchor, observed)
            receiver.clock.now += elapsed
            with self.assertRaisesRegex(GuardError, reason):
                receiver.send_checked(CheckedCommand(1, target, receiver.clock(), 10.4))
            self.assertFalse(receiver.accepted)
        guard = ActionGuard(LIMITS)
        guard.arm(STATE, sampled_at=10., now=10., readiness=READINESS)
        with self.assertRaisesRegex(GuardError, 'Stale'):
            guard.prepare(STATE, observed_state=STATE, sampled_at=9.799, proposal_at=10.2, now=10.2)

    def test_receiver_fault_stale_camera_or_feedback_are_not_fixed_by_policy(self):
        for fault in ('receiver', 'camera_age', 'feedback_age', 'feedback_skew'):
            observation, result = recorded_frame()
            metadata = observation['source_metadata']
            if fault == 'receiver':
                metadata['snapshot'].update(healthy=False, health_reasons=['SYNTHETIC fault'])
            elif fault == 'camera_age':
                metadata['camera_metadata']['global']['estimated_capture_monotonic'] = metadata['input_ready_monotonic']-.151
            elif fault == 'feedback_age':
                metadata['state_at'] = metadata['input_ready_monotonic']-.101
            else:
                metadata['state_group_skew_s'] = .041
            with self.subTest(fault=fault):
                self.assertFalse(client.evaluate_prediction(observation, result)['preview_numeric_and_source_checks_pass'])

    def test_preview_first_twenty_consumed_and_remaining_thirty_bounds_are_diagnostic_only(self):
        observation, result = recorded_frame()
        assessment = client.evaluate_prediction(observation, result)
        self.assertTrue(assessment['preview_numeric_and_source_checks_pass'])
        self.assertFalse(assessment['future_queue_feedback_or_commit_history_verified'])
        self.assertFalse(assessment['execution_ready'])
        result['actions'][49, 0] = LIMITS.bounds[0][1]+1.
        assessment = client.evaluate_prediction(observation, result)
        self.assertTrue(assessment['preview_numeric_and_source_checks_pass'])
        self.assertFalse(assessment['full_chunk_bounds']['all_50_within_hard_bounds'])
        self.assertTrue(assessment['consumed_prefix_bounds_pass'])
        self.assertEqual(assessment['full_chunk_bounds']['indices_to_be_consumed'],list(range(20)))
        self.assertEqual(assessment['full_chunk_bounds']['remaining_indices_are_diagnostic_only'],list(range(20,50)))
        result['actions'][9, 0] = LIMITS.bounds[0][1]+1.
        assessment = client.evaluate_prediction(observation, result)
        self.assertTrue(assessment['preview_numeric_and_source_checks_pass'])
        self.assertTrue(assessment['first_action_static_checks_pass'])
        self.assertFalse(assessment['consumed_prefix_bounds_pass'])
        self.assertTrue(assessment['raw_prefix_bounds_are_diagnostic_only'])

    def test_actual_feedback_outside_bounds_and_source_deadline_remain_hard_errors(self):
        for axis,value in ((1,-.001),(2,.001),(6,-.001)):
            observed,result=recorded_frame()
            observed['state'][axis]=value
            result['actions']=np.tile(np.asarray(STATE),(50,1))
            assessment=client.evaluate_prediction(observed,result)
            self.assertFalse(assessment['preview_numeric_and_source_checks_pass'])
            self.assertIsNone(assessment['first_command_conditioning'])
        observed,result=recorded_frame()
        result['validated_monotonic']=observed['sampled_at']+.401
        self.assertFalse(client.evaluate_prediction(observed,result)['preview_numeric_and_source_checks_pass'])
        # Omitting an anchor preserves the original zero-budget assumption.
        with self.assertRaisesRegex(GuardError,'expire during'):
            condition_first_command((0.,)*6+(.01,), (0.,)*6+(.01,),
                limits=LIMITS, profile=DEFAULTS['command_conditioning'],
                sampled_at=10., validated_at=10.36, source_kind='live')

    def test_preview_uses_only_input_ready_rate_budget_with_original_source_expiry(self):
        boundary = (0.,)*6+(.01,)
        args = dict(limits=LIMITS,profile=DEFAULTS['command_conditioning'],
                    sampled_at=10.,source_kind='live')
        #372ms source age,350ms elapsed since input-ready: no extra49ms wait.
        result=condition_first_command(boundary,boundary,validated_at=10.372,
                                       rate_anchor_at=10.022,**args)
        self.assertEqual(result['computed_first_command_wait_s'],0.)
        self.assertAlmostEqual(result['source_age_at_hypothetical_command_s'],.372)
        self.assertAlmostEqual(result['assumptions']['initial_rate_elapsed_s'],.350)
        self.assertFalse(result['assumptions']['real_arming_anchor_verified'])
        self.assertTrue(result['assumptions']['rate_anchor_is_hypothetical_not_verified_arm_time'])
        for anchor in (10.,10.022,10.401):
            with self.subTest(expired_anchor=anchor),self.assertRaisesRegex(GuardError,'already expired'):
                condition_first_command(boundary,boundary,validated_at=10.401,
                                        rate_anchor_at=anchor,**args)
        zero=condition_first_command(boundary,boundary,validated_at=10.2,**args)
        self.assertAlmostEqual(zero['computed_first_command_wait_s'],(2.+.002)/(45.*.9))
        self.assertEqual(zero['assumptions']['initial_rate_elapsed_s'],0.)
        for anchor in (9.999,10.373,float('nan'),float('inf'),True):
            with self.subTest(invalid_anchor=anchor),self.assertRaises(GuardError):
                condition_first_command(boundary,boundary,validated_at=10.372,
                                        rate_anchor_at=anchor,**args)

    def test_recorded_remote_warmup_discards_actions_and_precedes_live_source(self):
        observed, inference = recorded_frame()
        interface, _, _ = client.service_settings()
        canonical = Path(interface['runtime_config']).parent/'canonical_input.npz'
        recorded = client.load_recorded(canonical)
        self.assertEqual(recorded['kind'], 'recorded')
        self.assertIsNone(recorded['sampled_at'])
        observed.update(task=client.TASK)
        observed['global'], observed['wrist'] = recorded['global'], recorded['wrist']
        events = []
        class WarmSocket:
            def __init__(self, timeout):
                self.timeout = timeout
            def gettimeout(self):
                return self.timeout
            def settimeout(self, timeout):
                self.timeout = timeout
        class WarmClient:
            def __init__(self, *args):
                self.count = 0
                self.timeout_s = args[2]
                self.connection = SimpleNamespace(timeout=self.timeout_s, sock=WarmSocket(self.timeout_s))
            def open(self):
                events.append('open_inference')
                return {'SYNTHETIC': 'offline health only'}
            def infer(self, state, global_image, wrist_image, task):
                self.count += 1
                events.append('warmup_infer' if self.count <=3 else 'live_infer')
                if self.count <=3:
                    if (self.timeout_s, self.connection.timeout, self.connection.sock.gettimeout()) != (30.,30.,30.):
                        raise AssertionError('Recorded-only startup timeout was not widened')
                    np.testing.assert_array_equal(state, recorded['state'])
                    np.testing.assert_array_equal(global_image, recorded['global'])
                    np.testing.assert_array_equal(wrist_image, recorded['wrist'])
                elif (self.timeout_s, self.connection.timeout, self.connection.sock.gettimeout()) != (3.,3.,3.):
                    raise AssertionError('Live inference timeout was not restored')
                return {**inference, 'request_id': 'synthetic-'+str(self.count)}
            def close(self):
                events.append('close_inference')
        class ReadOnlySource:
            def __init__(self, *args):
                self.assert_warmed()
                events.append('construct_live_source')
            def assert_warmed(self):
                if 'warmup_infer' not in events:
                    raise AssertionError('Live source before recorded warmup')
            def open(self):
                self.assert_warmed()
                events.append('open_live_source')
            def capture(self):
                self.assert_warmed()
                events.append('capture_live_source')
                return observed
            def close(self):
                events.append('close_live_source')
        with tempfile.TemporaryDirectory(prefix='pi05-recorded-warmup-offline-') as folder:
            output = Path(folder)/'preview'
            with patch.object(sys,'argv',['client.py','--live','--count','1','--output',str(output)]), \
                    patch.object(client,'InferenceClient',WarmClient), \
                    patch.object(client,'PassiveSource',ReadOnlySource), \
                    patch.object(client.signal,'signal'), \
                    patch.object(torch,'set_num_threads'), patch.object(torch,'set_num_interop_threads'), \
                    patch.object(CloudExpertPolicy,'select_action',side_effect=AssertionError('No policy/FIFO in warmup')), \
                    patch.object(PiperPolicyExecutor,'arm_from_current_state',side_effect=AssertionError('No arm')), \
                    patch.object(sys,'stdout',io.StringIO()):
                client.main()
            report = json.loads((output/'report.json').read_text())
        self.assertEqual(events[:7],['open_inference','warmup_infer','warmup_infer','warmup_infer',
                                   'construct_live_source','open_live_source','capture_live_source'])
        self.assertEqual(events.count('warmup_infer'),3)
        self.assertEqual(events.count('live_infer'),1)
        warmup = report['recorded_remote_warmup']
        self.assertFalse(warmup['live_evidence'])
        self.assertTrue(warmup['actions_discarded'])
        self.assertEqual(warmup['queued_actions'],0)
        self.assertEqual(warmup['robot_commands_sent'],0)
        self.assertEqual(warmup['prediction_count'],3)
        self.assertEqual(len(warmup['discarded_predictions']),3)
        self.assertEqual(warmup['startup_timeout_s'],30.)
        self.assertEqual(warmup['restored_inference_timeout_s'],3.)
        self.assertNotIn('actions',warmup)
        self.assertTrue(all('actions' not in item for item in warmup['discarded_predictions']))
        self.assertEqual(len(report['requests']),1)
        self.assertEqual(report['requests'][0]['request_id'],'synthetic-4')

    def test_recorded_warmup_restores_execution_timeout_and_reraises_failure_without_retry(self):
        for live_timeout in (.4,3.):
            for failure_mode in ('success','raise','close_then_raise'):
                fail_on_second=failure_mode!='success'
                with self.subTest(live_timeout=live_timeout,failure_mode=failure_mode):
                    class FakeSocket:
                        value = live_timeout
                        def gettimeout(self):
                            return self.value
                        def settimeout(self,value):
                            self.value=value
                    sock=FakeSocket()
                    inference_client=client.InferenceClient(timeout_s=live_timeout)
                    connection=SimpleNamespace(timeout=live_timeout,sock=sock)
                    inference_client.connection=connection
                    calls=[]
                    failure=RuntimeError('SYNTHETIC warmup failure')
                    def infer(*_args):
                        calls.append((inference_client.timeout_s,inference_client.connection.timeout,sock.gettimeout()))
                        if fail_on_second and len(calls)==2:
                            if failure_mode=='close_then_raise':
                                inference_client.connection=None
                                connection.sock=None
                            raise failure
                        return {'request_id':'synthetic-'+str(len(calls)), 'timing_s':{'model':.1},
                                'actions':'MUST BE DISCARDED'}
                    with patch.object(inference_client,'infer',side_effect=infer):
                        if fail_on_second:
                            with self.assertRaises(RuntimeError) as caught:
                                client.warmup_recorded(inference_client)
                            self.assertIs(caught.exception,failure)
                        else:
                            report=client.warmup_recorded(inference_client)
                            self.assertEqual(report['prediction_count'],3)
                            self.assertEqual(report['restored_inference_timeout_s'],live_timeout)
                    self.assertEqual(calls,[(30.,30.,30.)]*(2 if fail_on_second else 3))
                    self.assertEqual((inference_client.timeout_s,connection.timeout),(live_timeout,live_timeout))
                    if failure_mode=='close_then_raise':
                        self.assertIsNone(inference_client.connection)
                    else:
                        self.assertEqual(sock.gettimeout(),live_timeout)

    def test_known_complete_model_identity_is_required_without_network(self):
        interface, settings, checksum = client.service_settings()
        metadata = {'model_sha256': client.MODEL_SHA256, 'model_format': 'full-native-PI05',
            'peft_present': False, 'adapter_tensors': 0, 'chunk_size': 50, 'num_inference_steps': 10,
            'batch_size': 1, 'model_verification': {'bytes': client.MODEL_BYTES}, 'warmup_predictions': 1,
            'model_id': settings['model_id'], 'pipeline_files_sha256': settings['expected_pipeline_sha256'],
            'tokenizer_files_sha256': settings['expected_tokenizer_sha256'],
            'compile_model': settings['compile_inference'], 'compile_target': settings['compile_target'],
            'compile_mode': 'reduce-overhead', 'float32_matmul_precision': settings['float32_matmul_precision'],
            'device_api': settings['device'], 'config_sha256': checksum, 'config_path': interface['remote_config'],
            'torch_cuda': 'SYNTHETIC', 'torch_hip': None,
            'full_loading': {'native_complete_checkpoint': True, 'source_tensors': 813,
                             'destination_tensors': 813, 'missing_keys': [], 'unexpected_keys': []}}
        if 'production_image_codec' in settings:
            metadata.update(production_image_codec=settings['production_image_codec'],
                            wire_image_codecs=[client.RAW_CODEC, client.PNG_CODEC])
        health = {'metadata': metadata, 'ready': True, 'warmed_up': True,
                  'protocol_version': client.PROTOCOL, 'model_sha256': client.MODEL_SHA256}
        self.assertIs(client.validate_health(health), health)
        for field, bad in (('model_sha256', 'unknown'), ('config_sha256', 'unknown'),
                           ('config_path', '/old/service/config.json'), ('peft_present', True),
                           ('compile_model', not settings['compile_inference']), ('compile_target','sample_actions'),
                           ('compile_mode','default'), ('float32_matmul_precision','high'), ('full_loading', {})):
            invalid = deepcopy(health)
            invalid['metadata'][field] = bad
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                client.validate_health(invalid)
        if 'production_image_codec' in settings:
            for field, bad in (('production_image_codec', client.RAW_CODEC),
                               ('wire_image_codecs', [client.RAW_CODEC])):
                invalid = deepcopy(health)
                invalid['metadata'][field] = bad
                with self.subTest(field=field), self.assertRaises(RuntimeError):
                    client.validate_health(invalid)

    def test_launch_requires_compiled_denoise_highest_and_existing_model_identity(self):
        metadata={'model_sha256':launch_grasp.MODEL_SHA256,'peft_present':False,
                  'model_format':'full-native-PI05','chunk_size':50,'num_inference_steps':10,
                  'compile_model':True,'compile_target':'denoise_step','compile_mode':'reduce-overhead',
                  'float32_matmul_precision':'highest'}
        launch_grasp.validate_compiled_runtime_health(metadata)
        for field,value in (('compile_model',False),('compile_target','sample_actions'),('compile_mode','default'),
                            ('float32_matmul_precision','high'),('model_sha256','unknown'),('peft_present',True),
                            ('chunk_size',20),('num_inference_steps',5)):
            with self.subTest(field=field),self.assertRaises(RuntimeError):
                launch_grasp.validate_compiled_runtime_health({**metadata,field:value})

    def test_no_old_custom_conditioning_or_incomplete_readiness_can_be_authorized(self):
        for field in launch_grasp.FORBIDDEN_CONDITIONING_FIELDS:
            with self.subTest(field=field), self.assertRaises(ValueError):
                launch_grasp.reject_custom_conditioning({'nested': [{field: None}]})
        self.assertEqual(len(Readiness.__dataclass_fields__), 7)
        for missing in Readiness.__dataclass_fields__:
            readiness = Readiness(**{key: '' if key == missing else 'SYNTHETIC ONLY'
                                    for key in Readiness.__dataclass_fields__})
            with self.subTest(missing=missing), self.assertRaises(GuardError):
                ActionGuard(LIMITS).arm(STATE, sampled_at=10., now=10., readiness=readiness)
        for duration in (0, 5, 60, 1, 300):
            self.assertEqual(plan_workflow.validate_duration(duration), float(duration))
            self.assertEqual(plan_workflow.duration_argument(str(duration)), float(duration))
        for duration in (-1, .5, 300.001, float('nan'), float('inf'), True, False, None, '60'):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                plan_workflow.validate_duration(duration)
        for duration in ('0.5', 'nan', 'inf', 'true', '-1', '301', True, False):
            with self.subTest(cli_duration=duration), self.assertRaises(ValueError):
                plan_workflow.duration_argument(duration)
        self.assertEqual(DEFAULTS['default_duration_s'], plan_workflow.DEFAULT_DURATION_S)
        import start_trial
        old = json.loads((HERE.parent/'cloud_grasp_controlled/grasp_plan.json').read_text())
        with self.assertRaises(ValueError):
            start_trial.display(old, old['review_content_sha256'])

    def test_prepare_candidate_in_temporary_directory_keeps_new_approval_empty(self):
        import prepare_trial
        original = json.loads(FIXTURE.read_text())
        preview = deepcopy(original)
        preview['requests'] = preview['requests'][:5]
        preview.update(purpose='pi05_expert_newcloud_act_bridge_read_only_preview',
            queue_contract=QUEUE_CONTRACT, runtime_config_sha256=client.service_settings()[2],
            visual_gripper_gate_sha256=profile_sha256(DEFAULTS['vla_visual_gripper_gate']),
            inference_config_sha256=plan_workflow.digest(client.INFERENCE_CONFIG))
        for item in preview['requests']:
            metadata = item['source_metadata']
            observation = {'kind': 'live', 'state': np.asarray(metadata['snapshot']['state'], dtype=np.float32),
                'sampled_at': min(metadata['backend_sampled_at'], metadata['state_at'],
                    *(camera['estimated_capture_monotonic'] for camera in metadata['camera_metadata'].values())),
                'source_metadata': metadata}
            result = {**item, 'actions': np.asarray(item['response']['actions'], dtype=np.float32)}
            item['guard_assessment'] = client.evaluate_prediction(observation, result)
            item['camera_pair_image'] = str(FIXTURE.parent/item['camera_pair_image'])
        # Existing result may be absent on a clean first run. Supply only a
        # local synthetic evidence object via read(), never publish it.
        evidence = {'success': True, 'network_calls': 0, 'robot_commands_sent': 0,
                    'queue_contract': QUEUE_CONTRACT, 'command_conditioning_id': CONDITIONING_ID,
                    'command_conditioning_sha256': profile_sha256(DEFAULTS['command_conditioning']),
                    'tested_code_sha256': plan_workflow.code_review_hashes()}
        real_read = prepare_trial.read
        def fixture_read(path):
            return evidence if Path(path).name == 'offline_verification_result.json' else real_read(path)
        with tempfile.TemporaryDirectory(prefix='pi05-native-queue-offline-') as name:
            temporary = Path(name)
            report = temporary/'SYNTHETIC_UNPUBLISHED_report.json'
            report.write_text(json.dumps(preview))
            staging = temporary/'trial'
            staging.mkdir()
            # The complete latest batch is reviewed: larger all-pass batches
            # qualify, while neither fewer than five nor one hidden rejection
            # can be promoted to an executable plan. Repeated fixture entries
            # here are explicitly synthetic and never published as live proof.
            for count, rejected_index in ((4, None), (5, None), (20, None), (20, 7)):
                batch_preview = deepcopy(preview)
                batch_preview['requests'] = [deepcopy(preview['requests'][index % 5])
                                             for index in range(count)]
                for index, item in enumerate(batch_preview['requests']):
                    item['request_id'] = f'SYNTHETIC-UNPUBLISHED-{count}-{rejected_index}-{index}'
                if rejected_index is not None:
                    assessment = batch_preview['requests'][rejected_index]['guard_assessment']
                    assessment['preview_numeric_and_source_checks_pass'] = False
                    assessment['rejection_reasons'] = ['SYNTHETIC retained rejection']
                report.write_text(json.dumps(batch_preview))
                args = SimpleNamespace(preview=report, onsite_conditions_confirmed=False, duration=0)
                with self.subTest(preview_count=count, rejected_index=rejected_index), \
                        patch.object(prepare_trial, 'read', side_effect=fixture_read), \
                        patch.object(prepare_trial, 'digest', side_effect=lambda path:
                            '0'*64 if Path(path).name == 'offline_verification_result.json' else plan_workflow.digest(path)):
                    candidate = prepare_trial.build_candidate(
                        args, staging, HERE/'trial', '', batch_preview, batch_preview['requests'])
                    expected_ready = count >= 5 and rejected_index is None
                    self.assertIs(candidate['execution_ready'], expected_ready)
                    self.assertEqual(candidate['preview_evidence']['request_count'], count)
                    self.assertEqual(candidate['preview_evidence']['rejected_count'], int(rejected_index is not None))
                    self.assertEqual(len(candidate['preview_evidence']['requests']), count)
                    with patch.object(plan_workflow, 'latest_preview', return_value=report):
                        if expected_ready:
                            self.assertEqual(plan_workflow.verify_latest_preview(candidate), report.resolve())
                        else:
                            self.assertTrue(candidate['blocking_reasons'])
                            with self.assertRaisesRegex(ValueError, '并未全部通过'):
                                plan_workflow.verify_latest_preview({**candidate, 'execution_ready': True})
            report.write_text(json.dumps(preview))
            review_ids = []
            for duration in (0, 5, 60, 1, 300):
                args = SimpleNamespace(preview=report, onsite_conditions_confirmed=False, duration=duration)
                with patch.object(prepare_trial, 'read', side_effect=fixture_read), \
                        patch.object(prepare_trial, 'digest', side_effect=lambda path:
                            '0'*64 if Path(path).name == 'offline_verification_result.json' else plan_workflow.digest(path)):
                    plan = prepare_trial.build_candidate(args, staging, HERE/'trial', '', preview, preview['requests'])
                deployment = json.loads((staging/'execute_config.json').read_text())
                self.assertEqual(plan['duration_s'], duration)
                self.assertEqual(plan_workflow.validate_duration_binding(plan, deployment), duration)
                self.assertIn(plan_workflow.duration_description(duration),plan['goal'])
                self.assertIn(plan_workflow.duration_stop_description(duration),plan['path_intent'])
                if duration == 0:
                    self.assertIn('持续至Ctrl+C/保护停止',plan['goal'])
                    self.assertNotIn('0秒',plan['goal'])
                self.assertEqual(plan['readiness']['approved_goal_path_parameters'], '')
                self.assertTrue(plan['physical_approval_pending'])
                review_ids.append(plan['review_content_sha256'])
                for invalid in (-1, float('nan'), True, 60 if duration != 60 else 5):
                    bad = {**deployment, 'rollout_duration_s': invalid}
                    with self.subTest(duration=duration, configured=invalid), self.assertRaises(ValueError):
                        plan_workflow.validate_duration_binding(plan, bad)
            self.assertEqual(len(set(review_ids)), 5)
            # An approval transition must preserve the previously shown duration;
            # reject a changed duration before any temporary file or publication.
            approved_before = {**plan, 'execution_ready': True}
            approval_args = SimpleNamespace(duration=60, approval_note='EXPLICIT SYNTHETIC APPROVAL ONLY',
                onsite_conditions_confirmed=True, preview=report, refresh=False, approve_review_id='synthetic-id')
            with patch.object(prepare_trial, 'latest_preview', return_value=report), \
                    patch.object(prepare_trial, 'read', return_value=approved_before), \
                    patch.object(prepare_trial, 'verify_review_content', return_value='synthetic-id'), \
                    patch.object(prepare_trial, 'publish_candidate', side_effect=AssertionError('Must not publish')):
                with self.assertRaisesRegex(ValueError, 'duration'):
                    prepare_trial.prepare_transaction(approval_args)
            self.assertEqual(plan['n_action_steps'], 20)
            self.assertEqual(plan['action_indices'], list(range(20)))
            self.assertEqual(plan['trial_kind'], 'lerobot_native_queue_act_bridge_trial')
            self.assertNotIn('action_index', plan)
            self.assertEqual(plan['readiness']['approved_goal_path_parameters'], '')
            self.assertTrue(plan['physical_approval_pending'])
            self.assertTrue(plan['configuration_does_not_authorize_physical_grasp'])
            self.assertEqual(set(plan['readiness']), set(Readiness.__dataclass_fields__))
            launch_grasp.reject_custom_conditioning(plan)
            launch_grasp.reject_custom_conditioning(json.loads((staging/'execute_config.json').read_text()))


if __name__ == '__main__':
    logging.disable(logging.CRITICAL)
    torch.set_num_threads(1)
    suite = unittest.TestSuite([
        unittest.defaultTestLoader.loadTestsFromTestCase(NativeQueueTests),
        unittest.defaultTestLoader.loadTestsFromName('test_closure_sync'),
        unittest.defaultTestLoader.loadTestsFromName('test_closure_sync_integration'),
        unittest.defaultTestLoader.loadTestsFromName('test_visual_gripper_gate'),
        unittest.defaultTestLoader.loadTestsFromName('test_visual_gripper_runtime')])
    with patch.object(socket.socket, 'connect', side_effect=AssertionError('Network forbidden in offline test')):
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    artifact = {'purpose': 'CPU-only closure coordination, ACK-only phase commits, fixed carry ceiling, visual gate and official PI05 FIFO/runtime20 and unchanged saved10 config/source-slot timing/SDK/physical guard/identity verification',
        'success': result.wasSuccessful(), 'tests_run': result.testsRun,
        'failures': len(result.failures), 'errors': len(result.errors),
        'network_calls': 0, 'robot_commands_sent': 0,
        'queue_contract': QUEUE_CONTRACT, 'command_conditioning_id': CONDITIONING_ID,
        'command_conditioning_sha256': profile_sha256(DEFAULTS['command_conditioning']),
        'tested_code_sha256': plan_workflow.code_review_hashes(),
        'verification_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'fixture': str(FIXTURE), 'fixture_sha256': hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
        'scope_limit': 'Synthetic cloud, clocks, feedback and receiver only; no live future feedback, lease, collision-path or grasp success proof.'}
    (HERE/'offline_verification_result.json').write_text(json.dumps(artifact, ensure_ascii=False, indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
