"""Native probe contract tests with synthetic feedback and a fake clock only."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

TOOL = Path(__file__).resolve().parents[1]/'tools/native_target_probe.py'
spec = importlib.util.spec_from_file_location('native_target_probe', TOOL)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def snapshot(now=1., q=None, power=1, enable=1):
    return dict(q=list(probe.EXPECTED if q is None else q), q_time=now, q_seq=int(now*1000),
                joint_q_time=[now]*7, joint_feedback_pending=[False]*7, fault=None,
                status=dict(power_status=power, enable_status=enable, error_status=0),
                status_time=dict(power_status=now, enable_status=now, error_status=now))


class Cameras:
    def check(self):
        return


class FakeStdin:
    def __init__(self, transport):
        self.transport = transport
        self.lines = []

    def write(self, line):
        self.lines.append(line.strip())
        if line.strip() == 'power on':
            raise AssertionError('Global power must never use the J0 motion session')
        elif line.startswith('j '):
            parts = line.split()
            self.transport.target = float(parts[2])
            self.transport.enable = 1
        else:
            raise AssertionError('Unexpected native command')
        self.transport.acks += 1

    def flush(self):
        return


class FakeTransport:
    def __init__(self, moving=True):
        self.clock = 1.
        self.q = list(probe.EXPECTED)
        self.power = 0
        self.enable = 0
        self.target = self.q[probe.JOINT]
        self.acks = 0
        self.commands = 0
        self.lock = threading.RLock()
        self.moving = moving
        self.power_lines = []
        self.power_ready_at = None
        self.proc = type('Proc', (), {})()
        self.proc.stdin = FakeStdin(self)

    def now(self):
        return self.clock

    def wait(self, duration):
        self.clock += max(duration, .000001)
        if self.power_ready_at is not None and self.clock >= self.power_ready_at:
            self.power = 1
        if self.moving:
            delta = self.target-self.q[probe.JOINT]
            step = min(abs(delta), duration*.5)
            self.q[probe.JOINT] += step if delta > 0 else -step

    def snapshot(self):
        return snapshot(self.clock, self.q, self.power, self.enable)

    def power_factory(self, binary, nic, output, now):
        return FakePowerSession(self)


class FakePowerSession:
    def __init__(self, transport, *, rejected=False, published=True):
        self.transport = transport
        self.rejected = rejected
        self.published = published
        self.transport.power_lines.append('power on')
        if not rejected:
            self.transport.power_ready_at = self.transport.now()+.12

    def state(self):
        return dict(finished=True, fault='local scope rejection' if self.rejected else None,
                    cli_publish_attempts=int(self.published), received=int(not self.rejected),
                    rejected=int(self.rejected), ok=not self.rejected,
                    returncode=1 if self.rejected else 0)

    def close(self):
        return


def fake_review():
    return dict(expected_angles=list(probe.EXPECTED), allow_power_on=True,
                expected_status=dict(power_status=0, enable_status=[0, 1], error_status=0),
                parameters=dict(probe.PARAMETERS), binary='fake_binary', nic='fake0')


class NativeProbeTests(unittest.TestCase):
    def test_make_review_is_offline_and_hash_bound(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder)/'binary'
            binary.write_bytes(b'fake native binary')
            review = probe.make_review(probe.EXPECTED, binary, 'fake0', True)
            probe.verify_review(review)
            self.assertEqual(review['targets_deg'], [-2.8, -4.8, -6.8, -4.8])
            self.assertEqual(review['parameters']['max_position_commands'], 4)
            binary.write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError, 'changed'):
                probe.verify_review(review)

    def test_modified_review_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder)/'binary'
            binary.write_bytes(b'x')
            review = probe.make_review(probe.EXPECTED, binary, 'fake0', True)
            review['targets_deg'][0] += 10
            with self.assertRaisesRegex(RuntimeError, 'SHA'):
                probe.verify_review(review)

    def test_feedback_freshness_and_guard_failures(self):
        baseline = list(probe.EXPECTED)
        probe.checked_snapshot(snapshot(), 1., baseline, -4.8)
        stale = snapshot()
        stale['joint_q_time'][1] = .3
        with self.assertRaisesRegex(RuntimeError, 'Stale'):
            probe.checked_snapshot(stale, 1., baseline)
        drift = snapshot()
        drift['q'][4] += .51
        with self.assertRaisesRegex(RuntimeError, 'Non-target'):
            probe.checked_snapshot(drift, 1., baseline)
        with self.assertRaisesRegex(RuntimeError, 'tracking'):
            probe.checked_snapshot(snapshot(), 1., baseline, 4.)

    def test_near_limit_feedback_uses_original_point_two_only(self):
        q = list(probe.EXPECTED)
        q[1] = -90.2
        probe.checked_snapshot(snapshot(q=q), 1., None)
        q[1] = -90.21
        with self.assertRaisesRegex(RuntimeError, 'limits'):
            probe.checked_snapshot(snapshot(q=q), 1., None)

    def test_encoder_statistics_do_not_claim_physical_accuracy(self):
        samples = [dict(t=i/10, q=[1.2]+[0.]*6) for i in range(51)]
        result = probe.endpoint_metrics(samples, 0., 5., 1.)
        self.assertEqual(result['samples'], 21)
        self.assertAlmostEqual(result['encoder_signed_residual_deg'], .2)
        self.assertEqual(result['stable_to_end_after_s'], 0.)
        self.assertFalse(result['physical_accuracy_verified'])
        self.assertTrue(result['task_1deg_pass'])
        self.assertTrue(result['assessment_0_3deg_pass'])

    def test_precision_requires_enough_stable_samples(self):
        sparse = [dict(t=4.5, q=[1.]+[0.]*6)]
        result = probe.endpoint_metrics(sparse, 0., 5., 1.)
        self.assertFalse(result['task_1deg_pass'])
        self.assertFalse(result['assessment_0_3deg_pass'])
        samples = [dict(t=i/10, q=[1.5]+[0.]*6) for i in range(51)]
        result = probe.endpoint_metrics(samples, 0., 5., 1.)
        self.assertTrue(result['task_1deg_pass'])
        self.assertFalse(result['assessment_0_3deg_pass'])

    def test_power_ack_before_state_publication_waits_for_new_readback(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(probe, 'verify_review'):
            transport = FakeTransport()
            runner = probe.Probe(fake_review(), transport, Cameras(), folder,
                                 power_session_factory=transport.power_factory)
            began = transport.now()
            try:
                runner.write('power on', power=True)
            finally:
                runner.events.close()
            self.assertGreaterEqual(transport.now()-began, .12)
            self.assertEqual(transport.power, 1)
            self.assertEqual(transport.power_lines, ['power on'])
            self.assertEqual(transport.proc.stdin.lines, [])
            self.assertEqual(runner.power_delivery['sent'], 1)

    def test_power_only_argv_has_one_fixed_direct_command_and_no_motion_scope_change(self):
        argv = probe.power_on_argv('/fake/arm_control', 'fake0')
        self.assertEqual(argv[-2:], ['power', 'on'])
        self.assertEqual(argv[argv.index('--joints')+1], 'all')
        self.assertNotIn('--exec', argv)
        self.assertNotIn('j', argv)

    def test_power_process_closed_stdin_and_protocol_delivery_counts(self):
        tx = dict(seq=42, address=1, funcode=6, data=dict(power=1))
        rx = dict(seq=42, address=3, funcode=1, data=dict(recv_status=1))
        output = '[tx] '+json.dumps(tx)+'\n[rx] '+json.dumps(rx)+'\n[ok] seq=42\n'
        class Process:
            def __init__(self, text):
                self.stdout = io.StringIO(text)
            def poll(self):
                return 0
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(probe.subprocess, 'Popen', return_value=Process(output)) as popen:
            session = probe.PowerOnSession('/fake/binary', 'fake0', Path(folder)/'power.jsonl')
            session.reader.join(timeout=1.)
            state = session.state()
            session.close()
            self.assertEqual(state['cli_publish_attempts'], 1)
            self.assertEqual(state['received'], 1)
            self.assertTrue(state['ok'])
            self.assertEqual(popen.call_args.kwargs['stdin'], probe.subprocess.DEVNULL)
            self.assertEqual(popen.call_args.args[0][-2:], ['power', 'on'])

    def test_power_process_local_reject_has_no_publish_or_receipt(self):
        class Process:
            stdout = io.StringIO('[参数错误] global scope rejected\n')
            def poll(self):
                return 2
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(probe.subprocess, 'Popen', return_value=Process()):
            session = probe.PowerOnSession('/fake/binary', 'fake0', Path(folder)/'power.jsonl')
            session.reader.join(timeout=1.)
            state = session.state()
            session.close()
            self.assertEqual(state['cli_publish_attempts'], 0)
            self.assertEqual(state['received'], 0)
            self.assertEqual(state['rejected'], 1)

    def test_power_receive_timeout_is_unknown_not_rejection(self):
        tx = dict(seq=42, address=1, funcode=6, data=dict(power=1))
        class Process:
            stdout = io.StringIO('[tx] '+json.dumps(tx)+'\n[接收超时] seq=42\n')
            def poll(self):
                return 1
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(probe.subprocess, 'Popen', return_value=Process()):
            session = probe.PowerOnSession('/fake/binary', 'fake0', Path(folder)/'power.jsonl')
            session.reader.join(timeout=1.)
            state = session.state()
            session.close()
            self.assertEqual(state['cli_publish_attempts'], 1)
            self.assertEqual(state['received'], 0)
            self.assertEqual(state['rejected'], 0)
            self.assertIsNotNone(state['fault'])

    def test_local_power_rejection_is_attempted_but_not_sent_and_not_retried(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(probe, 'verify_review'):
            transport = FakeTransport()
            def rejected(*args):
                return FakePowerSession(transport, rejected=True, published=False)
            runner = probe.Probe(fake_review(), transport, Cameras(), folder, power_session_factory=rejected)
            try:
                with self.assertRaisesRegex(RuntimeError, 'rejection'):
                    runner.power_on()
                self.assertEqual(runner.power_delivery['attempted'], 1)
                self.assertEqual(runner.power_delivery['sent'], 0)
                self.assertEqual(runner.power_delivery['rejected'], 1)
                self.assertEqual(runner.power_commands, 0)
                with self.assertRaisesRegex(RuntimeError, 'already attempted'):
                    runner.power_on()
            finally:
                runner.events.close()
            self.assertEqual(transport.proc.stdin.lines, [])
            self.assertEqual(len(transport.power_lines), 1)

    def test_review_consumed_once_and_approval_text_is_bound(self):
        with tempfile.TemporaryDirectory() as folder:
            review = dict(review_sha256='abc123')
            registry = Path(folder)/'claims'
            marker, claim = probe.consume_review(review, '允许本次 J0 诊断', Path(folder)/'run1', registry)
            self.assertEqual(json.loads(marker.read_text())['approval_text'], '允许本次 J0 诊断')
            self.assertEqual(len(claim['approval_text_sha256']), 64)
            with self.assertRaisesRegex(RuntimeError, 'already consumed'):
                probe.consume_review(review, 'another approval', Path(folder)/'run2', registry)

    def test_camera_startup_failure_still_consumes_review(self):
        class FailedCamera:
            def __init__(self, folder):
                self.ready = False
                self.error = 'camera missing'
            def check(self):
                raise RuntimeError(self.error)
            def close(self):
                return
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            binary = root/'binary'
            binary.write_bytes(b'offline')
            review = probe.make_review(probe.EXPECTED, binary, 'fake0', True)
            review_path = root/'review.json'
            review_path.write_text(json.dumps(review))
            output = root/'run'
            modules = {'handeye_joint_step': types.SimpleNamespace(Cameras=FailedCamera),
                       'd1_pick_place.sdk_executor': types.SimpleNamespace(CliTransport=object)}
            argv = [str(TOOL), '--execute', '--review', str(review_path), '--output', str(output),
                    '--approval-text', 'offline fake approval']
            with patch.object(probe, 'ROOT', root), patch.dict(probe.sys.modules, modules), \
                    patch.object(probe.sys, 'argv', argv), patch.object(probe.signal, 'signal'), \
                    patch('builtins.print'):
                self.assertEqual(probe.main(), 1)
            self.assertTrue((root/'validation/native_probe_consumed'/(review['review_sha256']+'.json')).exists())
            summary = json.loads((output/'summary.json').read_text())
            self.assertFalse(summary['completed'])
            self.assertIn('camera missing', summary['error'])

    def test_fake_run_has_one_power_at_most_four_positions_and_no_other_axis(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(probe, 'verify_review'):
            transport = FakeTransport()
            runner = probe.Probe(fake_review(), transport, Cameras(), folder,
                                 power_session_factory=transport.power_factory)
            try:
                runner.run()
            finally:
                runner.events.close()
            lines = transport.proc.stdin.lines
            self.assertEqual(transport.power_lines, ['power on'])
            self.assertNotIn('power on', lines)
            positions = [line for line in lines if line.startswith('j ')]
            self.assertLessEqual(len(positions), 4)
            self.assertTrue(all(line.startswith('j 0 ') for line in positions))
            self.assertEqual(len(runner.holds), 4)

    def test_no_actual_motion_skips_same_position_returns(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(probe, 'verify_review'):
            transport = FakeTransport(moving=False)
            runner = probe.Probe(fake_review(), transport, Cameras(), folder,
                                 power_session_factory=transport.power_factory)
            try:
                runner.run()
            finally:
                runner.events.close()
            self.assertFalse(any(line.startswith('j 0 -4.800 ') for line in transport.proc.stdin.lines))

    def test_powered_arm_accepts_enable_zero_bootstrap_or_enable_one_without_power_command(self):
        for enable in (0, 1):
            with self.subTest(enable=enable), tempfile.TemporaryDirectory() as folder, patch.object(probe, 'verify_review'):
                transport = FakeTransport()
                transport.power = 1
                transport.enable = enable
                review = fake_review()
                review['allow_power_on'] = False
                review['expected_status']['power_status'] = 1
                runner = probe.Probe(review, transport, Cameras(), folder,
                                     power_session_factory=lambda *args: self.fail('No power command allowed'))
                try:
                    runner.run()
                finally:
                    runner.events.close()
                self.assertEqual(runner.power_attempts, 0)
                self.assertEqual(runner.position_commands, 4)
                self.assertTrue(all(line.startswith('j 0 ') for line in transport.proc.stdin.lines))
                self.assertEqual([result['phase'] for result in runner.holds],
                                 ['positive', 'return_from_positive', 'negative', 'return_from_negative'])

    def test_raw_feedback_quality_stale_and_uart_integrity_metrics(self):
        def row(t, function, data):
            message = dict(address=2, funcode=function, data=data)
            return dict(t=t, kind='output', line='[rx] '+json.dumps(message))
        angles = {f'angle{i}': value for i, value in enumerate(probe.EXPECTED)}
        first = dict(schema='d1_uart_integrity_v1', event='feedback_scan', queries_total=7,
                     queries_invalid=1, invalid_by_status=dict(checksum=1), valid_frames=1, dropped_frames=0)
        last = dict(first, queries_total=21, queries_invalid=3,
                    invalid_by_status=dict(checksum=3), valid_frames=2, dropped_frames=1)
        events = [row(1., 1, angles), row(1.1, 90, first), row(1.2, 1, angles),
                  row(2.2, 1, angles), row(2.3, 90, last)]
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder)/'raw.jsonl'
            log.write_text(''.join(json.dumps(event)+'\n' for event in events))
            result = probe.feedback_quality(log)
        self.assertEqual(result['valid_feedback_count'], 3)
        self.assertEqual(result['stale_interval_count'], 1)
        self.assertAlmostEqual(result['stale_total_s'], .4)
        self.assertTrue(result['uart_integrity_available'])
        self.assertEqual(result['protocol_invalid_count'], 2)
        self.assertEqual(result['uart_invalid_by_status']['checksum']['delta'], 2)

    def test_missing_uart_diagnostic_is_unknown_and_old_review_schema_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder)/'raw.jsonl'
            log.write_text('')
            quality = probe.feedback_quality(log)
            self.assertFalse(quality['uart_integrity_available'])
            self.assertIsNone(quality['protocol_invalid_count'])
            binary = Path(folder)/'binary'
            binary.write_bytes(b'fake')
            review = probe.make_review(probe.EXPECTED, binary, 'fake0', False)
            review['schema'] = 'd1_native_target_probe_v1'
            review['review_sha256'] = probe.hashlib.sha256(probe.canonical(
                {key: value for key, value in review.items() if key != 'review_sha256'})).hexdigest()
            with self.assertRaisesRegex(RuntimeError, 'Unsupported'):
                probe.verify_review(review)

    def test_cancel_sends_nothing(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(probe, 'verify_review'):
            transport = FakeTransport()
            runner = probe.Probe({}, transport, Cameras(), folder, cancel=lambda: True)
            try:
                with self.assertRaisesRegex(RuntimeError, 'Cancelled'):
                    runner.run()
            finally:
                runner.events.close()
            self.assertEqual(transport.proc.stdin.lines, [])

    def test_changed_code_before_write_sends_nothing(self):
        with tempfile.TemporaryDirectory() as folder:
            transport = FakeTransport()
            transport.power = 1
            runner = probe.Probe(dict(parameters=dict(probe.PARAMETERS)), transport, Cameras(), folder)
            try:
                with patch.object(probe, 'verify_review', side_effect=RuntimeError('Code changed')):
                    with self.assertRaisesRegex(RuntimeError, 'changed'):
                        runner.target_once('test', -2.8, 1000)
            finally:
                runner.events.close()
            self.assertEqual(transport.proc.stdin.lines, [])

    def test_camera_fault_prevents_power(self):
        class BrokenCamera:
            def check(self):
                raise RuntimeError('Camera stale')
        with tempfile.TemporaryDirectory() as folder:
            transport = FakeTransport()
            runner = probe.Probe({}, transport, BrokenCamera(), folder)
            try:
                with self.assertRaisesRegex(RuntimeError, 'Camera'):
                    runner.run()
            finally:
                runner.events.close()
            self.assertEqual(transport.proc.stdin.lines, [])

    def test_quantized_point_two_boundary_skips_native_target(self):
        for actual in (-4.8, -4.7, -4.6, -4.599999999):
            with self.subTest(actual=actual), tempfile.TemporaryDirectory() as folder:
                transport = FakeTransport()
                transport.power = 1
                transport.q[probe.JOINT] = actual
                runner = probe.Probe(dict(parameters=dict(probe.PARAMETERS)), transport, Cameras(), folder)
                try:
                    self.assertFalse(runner.target_once('near-zero', -4.8, 1000))
                finally:
                    runner.events.close()
                self.assertEqual(transport.proc.stdin.lines, [])


if __name__ == '__main__':
    unittest.main()
