"""No-IO control tests with real PiPER send_checked, Robot and Guard code."""
from contextlib import contextmanager
from copy import deepcopy
import io
import json
import logging
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), '/home/aup/Lerobot/projects/piper/deployment',
               '/home/aup/Lerobot/projects/piper/official_adapter', '/home/aup/piper SDK/pyAgxArm']
import control_logging as diagnostics
import plan_workflow
from nativequeuepolicy import NativeQueueTiming
from lerobot_robot_piper_executor.backend import PiperBackend
from lerobot_robot_piper_executor.guard import ACTION_KEYS, GuardError, GuardLimits, Readiness
from lerobot_robot_piper_executor.robot import Observation, PiperPolicyExecutor, PiperPolicyExecutorConfig, ROLLOUT_KEYS

STATE = (0., 10., -10., 0., 0., 0., .01)
READY = Readiness(**{key: 'SYNTHETIC NO HARDWARE' for key in Readiness.__dataclass_fields__})
REPRODUCTION_RESULTS = []


@contextmanager
def handlers_for_test(handlers, level=logging.INFO):
    root = logging.getLogger()
    old, old_level = list(root.handlers), root.level
    root.handlers, root.level = list(handlers), level
    try:
        yield root
    finally:
        root.handlers, root.level = old, old_level


class Recorder(logging.Handler):
    def __init__(self, *, delay=0., entered=None, release=None, level=logging.NOTSET):
        super().__init__(level)
        self.records, self.delay, self.entered, self.release_event = [], delay, entered, release
    def emit(self, record):
        if self.entered is not None:
            self.entered.set()
        if self.release_event is not None and not self.release_event.wait(2):
            raise RuntimeError('SYNTHETIC test release missing')
        if self.delay:
            time.sleep(self.delay)
        self.records.append((record.name, record.getMessage(), record.created, record.args))


class FakeRPC:
    def __init__(self, *, delay=0., error=None):
        self.calls, self.delay, self.error = [], delay, error
    def remote_deadline(self, deadline):
        return int(deadline*1e9)
    def request(self, op, **payload):
        self.calls.append((op, payload))
        if op == 'target':
            if self.delay:
                time.sleep(self.delay)
            if self.error:
                raise self.error
            return {'accepted_sequence': payload['sequence']}
        return {}
    def close(self):
        pass


def ready_robot(folder, *, rpc=None, source_age=.253):
    config = json.loads((HERE/'trial/execute_config.json').read_text())
    io_config = json.loads((HERE/'trial/io_config.json').read_text())
    config['policy_target_trace_path'] = str(Path(folder)/'trace.jsonl')
    rpc = rpc or FakeRPC()
    backend = PiperBackend(config, io_config, client=rpc)
    backend.lease, backend.sequence = 'SYNTHETIC-LEASE', 2
    limits = GuardLimits(**config['guard_limits'])
    robot = PiperPolicyExecutor(PiperPolicyExecutorConfig(id='PURE OFFLINE LOGGING'),
                                backend=backend, limits=limits)
    robot.connected = True
    robot._ensure_policy_target_trace()
    now = time.monotonic()
    observed_at = now-source_age
    robot.latest = Observation(dict(zip(ACTION_KEYS, STATE)), observed_at)
    robot.guard.arm(STATE, sampled_at=observed_at, now=now, readiness=READY)
    robot.guard.sequence = 2
    robot.require_policy_timing = True
    robot.policy_action_timing = NativeQueueTiming(observed_at-.1, now-.05, now, 2, 20, 20.)
    return robot, rpc, observed_at


class LoggingTests(unittest.TestCase):
    def tearDown(self):
        worker = diagnostics._active_logging_worker
        if worker is not None and worker.thread.is_alive():
            worker.thread.join(timeout=2)
        self.assertTrue(worker is None or not worker.thread.is_alive())

    def test_real_seq3_old_200ms_sink_expires_and_async_path_commits_without_changing_deadline(self):
        for asynchronous in (False, True):
            with self.subTest(asynchronous=asynchronous), tempfile.TemporaryDirectory(prefix='pi05-log-guard-') as folder:
                sink = Recorder(delay=.200)
                with handlers_for_test([sink]):
                    robot, rpc, source_at = ready_robot(folder)
                    action = dict(zip(ROLLOUT_KEYS, STATE))
                    began = time.monotonic()
                    if asynchronous:
                        with diagnostics.asynchronous_control_logging() as worker:
                            robot.send_action(action)
                            elapsed = time.monotonic()-began
                            self.assertIsNone(robot.guard.pending)
                            self.assertTrue(robot.guard.armed)
                            robot.disarm('SYNTHETIC cleanup before listener shutdown')
                        self.assertLess(elapsed, .100)
                        self.assertGreaterEqual(worker.statistics()['max_handler_duration_ms'], 190.)
                    else:
                        with self.assertRaisesRegex(GuardError, 'Disarmed, foreign or expired command'):
                            robot.send_action(action)
                        elapsed = time.monotonic()-began
                        self.assertGreaterEqual(elapsed, .200)
                robot._close_policy_target_trace()
                rows = [json.loads(line) for line in (Path(folder)/'trace.jsonl').read_text().splitlines()]
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertEqual(row['command_sequence'], 3)
                self.assertAlmostEqual(row['expires_at'], source_at+.4)
                self.assertAlmostEqual(row['proposal_at'], source_at)
                self.assertFalse(row['transport_outcome_unknown'])
                self.assertEqual(row['outcome'], 'accepted' if asynchronous else 'rejected')
                self.assertEqual([op for op, _ in rpc.calls].count('target'), 1)
                self.assertLessEqual(next(payload['timeout_s'] for op, payload in rpc.calls if op == 'target'), .12)
                REPRODUCTION_RESULTS.append({'asynchronous_logging': asynchronous,
                    'sink_delay_ms': 200., 'send_action_elapsed_ms': elapsed*1000,
                    'sequence': row['command_sequence'], 'outcome': row['outcome'],
                    'budget_at_creation_ms': (row['expires_at']-row['created_at'])*1000,
                    'created_to_backend_return_ms': (row['backend_ack_at']-row['created_at'])*1000,
                    'backend_return_after_expiry_ms': (row['backend_ack_at']-row['expires_at'])*1000,
                    'target_requests': 1, 'original_source_and_deadline_preserved': True})

    def test_real_late_ack_and_unknown_transport_still_reject_without_retry(self):
        for rpc in (FakeRPC(delay=.17), FakeRPC(error=TimeoutError('SYNTHETIC unknown transport'))):
            with self.subTest(delay=rpc.delay), tempfile.TemporaryDirectory(prefix='pi05-log-expiry-') as folder:
                with handlers_for_test([Recorder()]):
                    with diagnostics.asynchronous_control_logging():
                        robot, _, _ = ready_robot(folder, rpc=rpc)
                        with self.assertRaises((GuardError, TimeoutError)):
                            robot.send_action(dict(zip(ROLLOUT_KEYS, STATE)))
                        self.assertFalse(robot.guard.armed)
                row = json.loads((Path(folder)/'trace.jsonl').read_text().splitlines()[0])
                self.assertEqual([op for op, _ in rpc.calls].count('target'), 1)
                self.assertEqual(row['outcome'], 'rejected')
                self.assertEqual(row['transport_outcome_unknown'], rpc.error is not None)

    def test_stuck_sink_does_not_block_control_cleanup_or_bounded_shutdown(self):
        entered, release = threading.Event(), threading.Event()
        sink = Recorder(entered=entered, release=release)
        with tempfile.TemporaryDirectory(prefix='pi05-log-cleanup-') as folder:
            report = Path(folder)/'statistics.json'
            try:
                with handlers_for_test([sink]) as root:
                    began = time.monotonic()
                    with diagnostics.asynchronous_control_logging(report_path=report, stop_timeout=.02) as worker:
                        root.warning('SYNTHETIC blocked sink')
                        self.assertTrue(entered.wait(1))
                        with tempfile.TemporaryDirectory(prefix='pi05-log-action-') as action_folder:
                            robot, _, _ = ready_robot(action_folder)
                            robot.send_action(dict(zip(ROLLOUT_KEYS, STATE)))
                            robot.disarm('SYNTHETIC cleanup')
                            self.assertFalse(robot.guard.armed)
                    self.assertLess(time.monotonic()-began, .120)
                    self.assertEqual(root.handlers, [sink])
                    stats = json.loads(report.read_text())
                    self.assertTrue(stats['worker_alive'])
                    self.assertEqual(stats['pending'], 0)
                    self.assertGreater(stats['in_flight_duration_ms'], 0.)
                    with self.assertRaises(RuntimeError):
                        with diagnostics.asynchronous_control_logging():
                            self.fail('Must not accumulate another blocked logging daemon')
            finally:
                release.set()
                worker.thread.join(timeout=1)

    def test_queue_overflow_only_drops_diagnostics_without_stderr_fallback(self):
        entered, release = threading.Event(), threading.Event()
        sink = Recorder(entered=entered, release=release)
        try:
            with handlers_for_test([sink]) as root, patch.object(sys, 'stderr', io.StringIO()) as stderr:
                with diagnostics.asynchronous_control_logging(capacity=2, stop_timeout=.01) as worker:
                    root.warning('first')
                    self.assertTrue(entered.wait(1))
                    for index in range(100):
                        root.warning('overflow%s', index)
                    self.assertGreater(worker.statistics()['dropped'], 0)
                self.assertEqual(stderr.getvalue(), '')
                self.assertEqual(worker.statistics()['pending'], 0)
                self.assertGreater(worker.statistics()['discarded_on_shutdown'], 0)
        finally:
            release.set()
            worker.thread.join(timeout=1)

    def test_levels_filters_child_routing_handler_identity_and_exception_restore(self):
        root_sink = Recorder(level=logging.WARNING)
        root_sink.addFilter(lambda record: 'filtered' not in record.getMessage())
        child_sink = Recorder(level=logging.INFO)
        child = logging.getLogger('pi05-test-child-own-handler')
        old = list(child.handlers), child.level, child.propagate
        child.handlers, child.level, child.propagate = [child_sink], logging.INFO, True
        try:
            with handlers_for_test([root_sink]) as root:
                with self.assertRaisesRegex(ValueError, 'SYNTHETIC body'):
                    with diagnostics.asynchronous_control_logging() as worker:
                        child.info('child-info')
                        child.warning('both-warning')
                        child.error('filtered')
                        raise ValueError('SYNTHETIC body')
                self.assertEqual(root.handlers, [root_sink])
                self.assertEqual(child.handlers, [child_sink])
                self.assertEqual((child.level, child.propagate), (logging.INFO, True))
            self.assertEqual([record[1] for record in root_sink.records], ['both-warning'])
            self.assertEqual([record[1] for record in child_sink.records], ['child-info', 'both-warning', 'filtered'])
            self.assertFalse(worker.statistics()['worker_alive'])
        finally:
            child.handlers, child.level, child.propagate = old

    def test_mutable_counts_are_snapshotted_and_original_creation_time_is_kept(self):
        entered, release = threading.Event(), threading.Event()
        sink = Recorder(entered=entered, release=release)
        try:
            with handlers_for_test([sink]) as root:
                with diagnostics.asynchronous_control_logging() as worker:
                    root.warning('block-worker')
                    self.assertTrue(entered.wait(1))
                    counts = {'joint_2': 3, 'source': {'at': [100., 101.]}}
                    created_before = time.time()
                    root.warning('counts=%s', counts)
                    created_after = time.time()
                    counts['joint_2'] = 999
                    counts['source']['at'][0] = -1.
                    release.set()
                event = sink.records[-1]
                self.assertIn("'joint_2': 3", event[1])
                self.assertIn('[100.0, 101.0]', event[1])
                self.assertGreaterEqual(event[2], created_before)
                self.assertLessEqual(event[2], created_after)
                self.assertEqual(worker.statistics()['dropped'], 0)
        finally:
            release.set()

    def test_parent_persists_all_raw_lines_even_when_terminal_mirror_is_blocked(self):
        entered, release = threading.Event(), threading.Event()
        class Terminal:
            def write(self, _line):
                entered.set()
                release.wait(2)
            def flush(self):
                pass
        mirror = diagnostics.AsyncTerminalMirror(Terminal(), capacity=2)
        log = io.StringIO()
        lines = ['line%s\n' % index for index in range(100)]
        try:
            mirror.enqueue('block-terminal\n')
            self.assertTrue(entered.wait(1))
            began = time.monotonic()
            diagnostics.persist_and_mirror(iter(lines), log, mirror)
            stats = mirror.stop(timeout=.02)
            self.assertLess(time.monotonic()-began, .100)
            self.assertEqual(log.getvalue(), ''.join(lines))
            self.assertGreater(stats['dropped'], 0)
            self.assertTrue(stats['worker_alive'])
            self.assertEqual(stats['pending'], 0)
        finally:
            release.set()
            mirror.worker.thread.join(timeout=1)

    def test_decorator_covers_cleanup_and_new_sources_are_review_pinned(self):
        events = []
        @diagnostics.with_asynchronous_control_logging
        def control(*, startup_result):
            try:
                events.append('run')
                raise RuntimeError('SYNTHETIC control error')
            finally:
                events.extend(['disarm', 'teardown', 'disconnect'])
        with tempfile.TemporaryDirectory(prefix='pi05-logging-scope-') as folder:
            path = Path(folder)/'startup.json'
            with handlers_for_test([Recorder()]):
                with self.assertRaises(RuntimeError):
                    control(startup_result=path)
            report = json.loads(path.with_name('control_logging_statistics.json').read_text())
            self.assertFalse(report['worker_alive'])
            self.assertEqual(events, ['run', 'disarm', 'teardown', 'disconnect'])
        hashes = plan_workflow.code_review_hashes()
        self.assertIn(str(HERE/'control_logging.py'), hashes)
        self.assertIn(str(HERE/'test_control_logging.py'), hashes)
        physical = json.loads((HERE/'trial/execute_config.json').read_text())
        self.assertEqual(physical['tx_ttl_s'], .12)
        self.assertEqual(physical['guard_limits']['max_command_age_s'], .4)
        self.assertEqual(physical['guard_limits']['max_observation_age_s'], .4)

    def test_statistics_failure_preserves_original_control_error(self):
        original = GuardError('SYNTHETIC original guard rejection')
        with tempfile.TemporaryDirectory(prefix='pi05-stat-failure-') as folder:
            with handlers_for_test([Recorder()]):
                with patch.object(Path, 'write_text', side_effect=OSError('SYNTHETIC disk failure')):
                    with self.assertRaises(GuardError) as caught:
                        with diagnostics.asynchronous_control_logging(report_path=Path(folder)/'stats.json'):
                            raise original
        self.assertIs(caught.exception, original)
        self.assertTrue(any('statistics' in note for note in original.__notes__))

    def test_raw_log_failure_requests_bounded_child_cleanup_and_keeps_original_error(self):
        events = []
        class BrokenLog:
            def write(self, _line):
                raise OSError('SYNTHETIC raw log failure')
            def flush(self):
                pass
        class Child:
            def send_signal(self, sig):
                events.append(('signal', sig))
            def terminate(self):
                events.append('SIGTERM')
            def wait(self, timeout):
                events.append(('wait', timeout))
                if timeout in (5, 3):
                    raise diagnostics.subprocess.TimeoutExpired('SYNTHETIC child', timeout)
                return 0
            def kill(self):
                events.append('kill after cleanup timeout')
        mirror = diagnostics.AsyncTerminalMirror(io.StringIO())
        try:
            with self.assertRaises(OSError) as caught:
                try:
                    diagnostics.persist_and_mirror(['one\n'], BrokenLog(), mirror)
                except BaseException as error:
                    diagnostics.stop_failed_log_child(Child(), error)
                    raise
            self.assertIn('raw log failure', str(caught.exception))
            self.assertEqual(events, [('signal', diagnostics.signal.SIGINT), ('wait', 5),
                                      'SIGTERM', ('wait', 3), 'kill after cleanup timeout', ('wait', 2)])
        finally:
            mirror.stop()


if __name__ == '__main__':
    with patch.object(socket.socket, 'connect', side_effect=AssertionError('Network forbidden')):
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(LoggingTests)
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = {'purpose': 'VLA-only asynchronous diagnostic logging verification',
        'success': result.wasSuccessful(), 'tests_run': result.testsRun,
        'failures': len(result.failures), 'errors': len(result.errors),
        'network_calls': 0, 'robot_commands_sent': 0, 'real_control_code_with_synthetic_rpc': True,
        'reproduction': REPRODUCTION_RESULTS, 'tested_code_sha256': plan_workflow.code_review_hashes(),
        'scope': 'Real Guard/Robot/send_checked with synthetic RPC and blocking diagnostic sinks; no physical motion or live scheduling guarantee.'}
    (HERE/'control_logging_verification_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
