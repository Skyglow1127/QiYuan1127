"""Local diagnostic-file checks with synthetic guards/feedback; no hardware."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from lerobot_robot_piper_executor.guard import ACTION_KEYS, GuardError
from lerobot_robot_piper_executor.policy_target_trace import PolicyTargetTrace
from lerobot_robot_piper_executor.robot import Observation, ROLLOUT_KEYS
from test_executor import ready
import test_policy_target_execution as execution_fixture


class Scalar:
    def __float__(self):
        return 1.25


class TraceTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / 'policy_targets.jsonl'
        self.fixture = execution_fixture.PolicyTargetExecutionTests()

    def make_robot(self):
        robot, backend, clock = self.fixture.make_robot()
        backend.config['policy_target_trace_path'] = str(self.path)
        self.addCleanup(robot.disarm, 'synthetic trace cleanup')
        return robot, backend, clock

    def prepare_action(self, robot, backend, clock):
        clock.now += .047
        self.fixture.fresh_timing(robot, backend, clock)
        return dict(zip(ROLLOUT_KEYS, backend.feedback))

    def rows(self):
        return [json.loads(line) for line in self.path.read_text().splitlines()]

    def test_scalar_snapshots_and_nonfinite_rejections_use_buffered_json(self):
        stream = Mock()
        with patch.object(Path, 'open', return_value=stream):
            trace = PolicyTargetTrace(self.path)
        raw = {'joint_1.deg': Scalar(), 'joint_2.deg': float('nan'),
               'joint_3.deg': float('inf'), 'joint_4.deg': -float('inf')}
        record = trace.start(raw_prediction=raw, observed_state={'joint_1.deg': Scalar()},
            sampled_at=10., timing=None, requested_at=10.01)
        raw['joint_1.deg'] = 99.
        trace.append(record)
        row = json.loads(stream.write.call_args.args[0])
        self.assertEqual(row['raw_prediction']['joint_1.deg'], 1.25)
        self.assertEqual(row['raw_prediction']['joint_2.deg'], 'NaN')
        self.assertEqual(row['raw_prediction']['joint_3.deg'], 'Infinity')
        self.assertEqual(row['raw_prediction']['joint_4.deg'], '-Infinity')
        self.assertEqual(row['observed_state']['joint_1.deg'], 1.25)
        stream.flush.assert_not_called()
        trace.close()
        stream.close.assert_called_once_with()

    def test_accepted_row_keeps_all_stages_original_times_and_rpc_return(self):
        robot, backend, clock = self.make_robot()
        backend.config['j5_upper_prediction_tolerance_deg'] = 1.
        action = self.prepare_action(robot, backend, clock)
        action['joint_1.pos'] = 5.
        source = robot.policy_action_timing
        sample = robot.latest.sampled_at
        feedback = {key: robot.latest.values[key] for key in ACTION_KEYS}
        warning = patch('lerobot.robots.utils.logging.warning')
        with warning:
            result = robot.send_action(action)
        robot.disarm('flush synthetic accepted trace')

        row, = self.rows()
        command, _, _, _ = backend.accepted[0]
        self.assertEqual(row['raw_prediction']['joint_1.deg'], 5.)
        self.assertEqual(row['j5_projected_prediction']['joint_1.deg'], 5.)
        self.assertEqual(row['relative_capped_prediction']['joint_1.deg'], 2.)
        self.assertLess(row['quantized_target']['joint_1.deg'], 2.)
        self.assertEqual(row['final_target']['joint_1.deg'], result['joint_1.pos'])
        self.assertEqual(row['observed_state'], feedback)
        self.assertEqual(set(row['observed_state']), set(ACTION_KEYS))
        self.assertEqual(row['sampled_at'], sample)
        self.assertEqual(row['act_timing']['observed_at'], source.observed_at)
        self.assertEqual(row['act_timing']['generated_at'], source.generated_at)
        self.assertEqual(row['act_timing']['selected_at'], source.selected_at)
        self.assertEqual(row['act_timing']['action_index'], source.action_index)
        self.assertEqual(row['act_timing']['action_count'], source.action_count)
        self.assertEqual(row['act_timing']['fps'], source.fps)
        self.assertEqual(row['created_at'], command.created_at)
        self.assertEqual(row['expires_at'], command.expires_at)
        self.assertAlmostEqual(row['expires_at'], source.scheduled_at + .4)
        self.assertAlmostEqual(row['backend_ack_at'], command.created_at + .002)
        self.assertEqual(row['outcome'], 'accepted')
        self.assertFalse(row['transport_outcome_unknown'])
        self.assertIsNone(row['error'])

    def test_missing_timing_rejection_still_keeps_raw_and_same_feedback(self):
        robot, backend, clock = self.make_robot()
        action = self.prepare_action(robot, backend, clock)
        robot.policy_action_timing = None
        with self.assertRaisesRegex(GuardError, 'Missing original ACT chunk timing'):
            robot.send_action(action)
        row, = self.rows()
        self.assertEqual(row['raw_prediction'], dict(zip(ACTION_KEYS, backend.feedback)))
        self.assertEqual(row['observed_state'], dict(zip(ACTION_KEYS, backend.feedback)))
        self.assertEqual(row['sampled_at'], robot.latest.sampled_at)
        self.assertIsNone(row['act_timing'])
        self.assertEqual(row['outcome'], 'rejected')
        self.assertIn('Missing original ACT chunk timing', row['error']['message'])
        self.assertFalse(row['transport_outcome_unknown'])
        self.assertEqual(backend.send_attempts, 0)

    def test_nonfinite_raw_prediction_keeps_original_guard_error_and_json_evidence(self):
        robot, backend, clock = self.make_robot()
        action = self.prepare_action(robot, backend, clock)
        action['joint_1.pos'] = float('nan')
        with self.assertRaisesRegex(GuardError, 'finite'):
            robot.send_action(action)
        row, = self.rows()
        self.assertEqual(row['raw_prediction']['joint_1.deg'], 'NaN')
        self.assertEqual(row['outcome'], 'rejected')
        self.assertEqual(backend.send_attempts, 0)

    def test_missing_rpc_ack_marks_transport_outcome_unknown(self):
        robot, backend, clock = self.make_robot()
        action = self.prepare_action(robot, backend, clock)
        original = GuardError('synthetic RPC outcome unknown')
        with patch.object(backend, 'send_checked', side_effect=original) as send:
            with self.assertRaises(GuardError) as caught:
                robot.send_action(action)
        self.assertIs(caught.exception, original)
        send.assert_called_once()
        row, = self.rows()
        self.assertEqual(row['outcome'], 'rejected')
        self.assertTrue(row['transport_outcome_unknown'])
        self.assertIsNone(row['backend_ack_at'])
        self.assertIsNotNone(row['created_at'])
        self.assertEqual(row['error']['message'], str(original))

    def test_trace_append_and_close_failures_never_replace_original_guard_error(self):
        robot, backend, clock = self.make_robot()
        action = self.prepare_action(robot, backend, clock)
        trace = robot._ensure_policy_target_trace()
        self.addCleanup(trace._stream.close)
        original = GuardError('synthetic original guard refusal')
        with patch.object(robot.guard, 'prepare', side_effect=original), \
             patch.object(trace, 'append', side_effect=OSError('synthetic disk full')) as append, \
             patch.object(trace, 'close', side_effect=OSError('synthetic close failure')), \
             self.assertLogs('lerobot_robot_piper_executor.robot', level='ERROR'):
            with self.assertRaises(GuardError) as caught:
                robot.send_action(action)
        self.assertIs(caught.exception, original)
        self.assertTrue(any('disk full' in note for note in original.__notes__))
        self.assertTrue(any('close failure' in note for note in original.__notes__))
        append.assert_called_once()
        self.assertEqual(backend.send_attempts, 0)
        self.assertFalse(robot.guard.armed)

    def test_trace_failure_after_ack_reports_accepted_outcome_and_disarms_without_retry(self):
        robot, backend, clock = self.make_robot()
        action = self.prepare_action(robot, backend, clock)
        trace = robot._ensure_policy_target_trace()
        failure = GuardError('synthetic trace disk full')
        with patch.object(trace, 'append', side_effect=failure) as append, \
             self.assertLogs('lerobot_robot_piper_executor.robot', level='ERROR') as log:
            with self.assertRaises(GuardError) as caught:
                robot.send_action(action)
        self.assertIs(caught.exception, failure)
        self.assertEqual(backend.send_attempts, 1)
        self.assertEqual(len(backend.accepted), 1)
        append.assert_called_once()
        self.assertTrue(any('outcome=accepted' in message for message in log.output))
        self.assertFalse(robot.guard.armed)

    def test_normal_arm_opens_before_feedback_lease_and_guard_and_first_send_reuses_file(self):
        robot, backend, clock = self.make_robot()
        events = []
        state = dict(zip(ACTION_KEYS, backend.feedback))
        observation = Observation({**state, 'global': SimpleNamespace(shape=(480, 640, 3)),
                                   'wrist': SimpleNamespace(shape=(480, 640, 3))}, clock() - .001)
        real_arm = robot.guard.arm
        real_remote_arm = backend.remote_guard.arm

        def open_trace(path):
            events.append('trace_open')
            return PolicyTargetTrace(path)

        def lease(_):
            events.append('lease')
            real_remote_arm(backend.feedback, sampled_at=clock() + backend.remote_offset,
                            now=clock() + backend.remote_offset, readiness=ready())

        def guard_arm(*args, **kwargs):
            events.append('guard_arm')
            return real_arm(*args, **kwargs)

        with patch('lerobot_robot_piper_executor.robot.PolicyTargetTrace', side_effect=open_trace) as construct, \
             patch.object(backend, 'observation', side_effect=lambda: events.append('observation') or observation, create=True), \
             patch.object(backend, 'require_execution_lease', side_effect=lease, create=True), \
             patch.object(robot.guard, 'arm', side_effect=guard_arm):
            robot.arm_from_current_state(ready())
            self.assertEqual(events, ['trace_open', 'observation', 'lease', 'guard_arm'])
            construct.side_effect = AssertionError('first target must not reopen trace')
            action = self.prepare_action(robot, backend, clock)
            robot.send_action(action)
            self.assertEqual(construct.call_count, 1)
        robot.disarm('flush normal arm trace')
        self.assertEqual(self.rows()[0]['outcome'], 'accepted')

    def test_trace_open_failure_prevents_observation_lease_or_command(self):
        robot, backend, _ = self.make_robot()
        with patch('lerobot_robot_piper_executor.robot.PolicyTargetTrace', side_effect=GuardError('synthetic trace open failed')), \
             patch.object(backend, 'observation', create=True) as observe, \
             patch.object(backend, 'require_execution_lease', create=True) as lease:
            with self.assertRaisesRegex(GuardError, 'trace open failed'):
                robot.arm_from_current_state(ready())
        observe.assert_not_called()
        lease.assert_not_called()
        self.assertEqual(backend.send_attempts, 0)

    def test_legacy_config_never_opens_trace(self):
        robot, backend, clock = self.fixture.make_robot()
        self.addCleanup(robot.disarm, 'legacy trace cleanup')
        action = self.prepare_action(robot, backend, clock)
        with patch('lerobot_robot_piper_executor.robot.PolicyTargetTrace') as trace:
            robot.send_action(action)
            robot.disarm('legacy done')
        trace.assert_not_called()
        self.assertEqual(backend.send_attempts, 1)
        self.assertFalse(self.path.exists())


if __name__ == '__main__':
    unittest.main()
