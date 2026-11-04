"""Bounded CAN freshness recovery with synthetic snapshots; no hardware IO."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from lerobot_robot_piper_shared import devices as d


def stale_snapshot():
    return {
        'ready': False, 'error': None, 'raw_error': None, 'link': {'ready': True},
        'issues': [f'{kind} 第 {part} 组数据过期'
                   for kind in ('state', 'action') for part in range(1, 5)] +
                  ['执行臂状态反馈缺失或过期'],
        'state': [0.] * 7, 'action': [1.] * 7,
        'arm_status': {'arm_status': 0, 'err_code': 0, 'ctrl_mode': 1, 'mode_feedback': 1},
        'ages_ms': {f'{kind}_{part}': 110.
                    for kind in ('state', 'action') for part in range(4)},
    }


class FakeClock:
    def __init__(self):
        self.now, self.sleeps = 100., []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


class CANFeedbackTests(unittest.TestCase):
    def context(self, snapshots):
        context = d.SharedContext.__new__(d.SharedContext)
        context.bus = SimpleNamespace(snapshot=Mock(side_effect=snapshots))
        context.sample_file = None
        context.sample_number = 17
        return context

    def clock(self):
        clock = FakeClock()
        monotonic = patch.object(d.time, 'monotonic', side_effect=clock.monotonic)
        sleep = patch.object(d.time, 'sleep', side_effect=clock.sleep)
        monotonic.start()
        sleep.start()
        self.addCleanup(monotonic.stop)
        self.addCleanup(sleep.stop)
        return clock

    def test_transient_staleness_returns_the_new_ready_snapshot(self):
        stale = stale_snapshot()
        fresh = copy.deepcopy(stale)
        fresh.update(ready=True, issues=[], state=[2.] * 7, action=[3.] * 7)
        fresh['ages_ms'] = dict.fromkeys(stale['ages_ms'], 1.)
        context = self.context([stale, fresh])
        clock = self.clock()
        actual = context.checked_snapshot(wait_for_fresh_s=.02)
        self.assertIs(actual, fresh)
        self.assertEqual(context.bus.snapshot.call_count, 2)
        self.assertEqual(clock.sleeps, [.005])
        self.assertEqual(stale['state'], [0.] * 7)

    def test_persistent_staleness_fails_within_deadline_and_saves_evidence(self):
        stale = stale_snapshot()
        context = self.context(None)
        context.bus.snapshot = Mock(return_value=stale)
        clock = self.clock()
        started = clock.now
        with tempfile.TemporaryDirectory(prefix='piper_can_feedback_') as directory:
            with (Path(directory)/'samples.jsonl').open('w') as sample_file:
                context.sample_file = sample_file
                with self.assertRaisesRegex(RuntimeError, 'state 第 1 组数据过期'):
                    context.checked_snapshot(wait_for_fresh_s=.012)
            errors = list(Path(directory).glob('can_feedback_error_*.json'))
            self.assertEqual(len(errors), 1)
            evidence = json.loads(errors[0].read_text())
        self.assertAlmostEqual(clock.now-started, .012)
        self.assertGreater(context.bus.snapshot.call_count, 1)
        self.assertEqual(evidence['snapshot'], stale)
        self.assertEqual(evidence['sample_number'], 17)
        self.assertAlmostEqual(evidence['waited_s'], .012)

    def test_fault_link_and_incomplete_feedback_do_not_wait(self):
        cases = []
        fault = stale_snapshot()
        fault['issues'].append('执行臂夹爪报告故障位')
        cases.append(fault)
        link = stale_snapshot()
        link['link']['ready'] = False
        cases.append(link)
        missing = stale_snapshot()
        missing['state'] = None
        cases.append(missing)
        missing_status = stale_snapshot()
        missing_status['arm_status'] = None
        cases.append(missing_status)
        source_error = stale_snapshot()
        source_error['error'] = 'SSH断开'
        cases.append(source_error)
        raw_error = stale_snapshot()
        raw_error['raw_error'] = 'disk full'
        cases.append(raw_error)
        clock = self.clock()
        for snapshot in cases:
            with self.subTest(snapshot=snapshot):
                context = self.context([snapshot])
                with self.assertRaises(RuntimeError):
                    context.checked_snapshot(wait_for_fresh_s=.05)
                self.assertEqual(context.bus.snapshot.call_count, 1)
        self.assertEqual(clock.sleeps, [])

    def test_default_check_fails_immediately(self):
        context = self.context([stale_snapshot()])
        clock = self.clock()
        with self.assertRaisesRegex(RuntimeError, '执行臂状态反馈缺失或过期'):
            context.checked_snapshot()
        self.assertEqual(context.bus.snapshot.call_count, 1)
        self.assertEqual(clock.sleeps, [])

    def test_old_arm_fault_or_wrong_mode_is_never_retried(self):
        clock = self.clock()
        for key, value in (('arm_status', 1), ('err_code', 0x100),
                           ('ctrl_mode', 0), ('mode_feedback', 0)):
            with self.subTest(key=key):
                snapshot = stale_snapshot()
                snapshot['arm_status'][key] = value
                context = self.context([snapshot])
                with self.assertRaises(RuntimeError):
                    context.checked_snapshot(wait_for_fresh_s=.05)
                self.assertEqual(context.bus.snapshot.call_count, 1)
        self.assertEqual(clock.sleeps, [])

    def test_diagnostic_write_failure_preserves_original_error(self):
        context = self.context([stale_snapshot()])
        context.sample_file = SimpleNamespace(name='/unused/samples.jsonl')
        with patch.object(Path, 'write_text', side_effect=OSError('disk full')), \
             self.assertLogs(level='WARNING'):
            with self.assertRaisesRegex(RuntimeError, 'state 第 1 组数据过期'):
                context.checked_snapshot()

    def test_acknowledge_stays_immediate_and_clears_failed_cycle(self):
        context = self.context([stale_snapshot()])
        action = dict.fromkeys(d.KEYS, 1.)
        context.expected_action = dict(action)
        context.cycle = {'previous': True}
        clock = self.clock()
        with self.assertRaisesRegex(RuntimeError, 'state 第 1 组数据过期'):
            context.acknowledge(action)
        self.assertEqual(context.bus.snapshot.call_count, 1)
        self.assertEqual(clock.sleeps, [])
        self.assertIsNone(context.cycle)
        self.assertIsNone(context.expected_action)


if __name__ == '__main__':
    unittest.main()
