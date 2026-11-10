#!/usr/bin/env python3
"""Offline contract tests: synthetic feedback only; never starts the DDS reader."""
import functools
import hashlib
import http.client
import http.server
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('real_monitor', ROOT / 'scripts/real_monitor.py')
MONITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MONITOR)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def angles(values=(10, 20, 30, 40, 50, 60, 73)):
    return {'address': 2, 'funcode': 1,
            'data': {f'angle{i}': value for i, value in enumerate(values)}}


def status(power=1, enabled=0, error=0):
    return {'address': 2, 'funcode': 3,
            'data': {'power_status': power, 'enable_status': enabled, 'error_status': error}}


class MonitorTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='d1t-monitor-test-')
        self.addCleanup(temporary.cleanup)
        mapping_path = Path(temporary.name) / 'candidate.json'
        self.candidate = {'mapping': {
            'signs': [1, -1, 1, -1, 1, -1],
            'zero_offsets_deg': [5, 0, -10, 20, 0, 90],
        }}
        mapping_path.write_text(json.dumps(self.candidate))
        self.config = {'max_age_s': 0.6, 'mapping_candidate': str(mapping_path),
                       'description': 'offline fixture', 'interface': 'NO_HARDWARE'}
        # Reuse the actual model's limits without changing production artifacts.
        model = json.loads((ROOT / 'generated/model.json').read_text())
        self.clock = FakeClock()
        self.feedback = MONITOR.Feedback(self.config, self.candidate, model, self.clock)
        self.reader = Mock()
        self.reader.poll.return_value = None
        self.feedback.process = self.reader

    def send(self, message):
        self.feedback.ingest(json.dumps(message))

    def prime(self):
        self.send(angles())
        self.send(status())
        self.assertTrue(self.feedback.state()['connected'])

    def test_complete_feedback_requires_both_angles_and_status(self):
        self.assertFalse(self.feedback.state()['connected'])
        self.send(angles())
        self.assertFalse(self.feedback.state()['connected'])
        self.assertIsNone(self.feedback.state()['status_age'])
        self.send(status())
        state = self.feedback.state()
        self.assertTrue(state['connected'])
        self.assertEqual(state['angles_deg'], [10, 20, 30, 40, 50, 60, 73])
        self.assertEqual(state['frames'], 1)
        self.assertEqual(state['status'], status()['data'])
        self.assertFalse(state['command_capable'])

    def test_new_status_cannot_hide_stale_angles(self):
        self.prime()
        self.clock.advance(0.7)
        self.send(status())
        state = self.feedback.state()
        self.assertFalse(state['connected'])
        self.assertAlmostEqual(state['age'], 0.7)
        self.assertEqual(state['status_age'], 0)
        self.send(angles())
        self.assertTrue(self.feedback.state()['connected'])

    def test_new_angles_cannot_hide_stale_status(self):
        self.prime()
        self.clock.advance(0.7)
        self.send(angles())
        state = self.feedback.state()
        self.assertFalse(state['connected'])
        self.assertEqual(state['age'], 0)
        self.assertAlmostEqual(state['status_age'], 0.7)
        self.send(status())
        self.assertTrue(self.feedback.state()['connected'])

    def test_acknowledgements_and_unrelated_messages_do_not_refresh_feedback(self):
        self.prime()
        self.clock.advance(0.7)
        for code in (1, 3):
            message = angles() if code == 1 else status()
            message['address'] = 1
            self.send(message)
        self.send({'address': 2, 'funcode': 99, 'data': angles()['data']})
        state = self.feedback.state()
        self.assertFalse(state['connected'])
        self.assertAlmostEqual(state['age'], 0.7)
        self.assertAlmostEqual(state['status_age'], 0.7)
        self.assertEqual(state['frames'], 1)

    def test_bad_frames_preserve_last_measurements_and_timestamps(self):
        self.prime()
        baseline = self.feedback.state()
        missing_angle = angles()
        del missing_angle['data']['angle6']
        missing_status = status()
        del missing_status['data']['error_status']
        invalid = [
            '{not JSON', '[]', 'null',
            json.dumps(missing_angle), json.dumps(missing_status),
            json.dumps({'address': 2, 'funcode': 1, 'data': None}),
            json.dumps(angles((10, 20, 30, 40, 50, 60, float('nan')))),
            json.dumps(angles((float('inf'), 20, 30, 40, 50, 60, 73))),
            json.dumps(angles((True, 20, 30, 40, 50, 60, 73))),
            json.dumps(angles(('10', 20, 30, 40, 50, 60, 73))),
            json.dumps(status(power=True)), json.dumps(status(enabled=2)),
            json.dumps(status(error=float('nan'))), json.dumps(status(error=False)),
        ]
        self.clock.advance(0.7)
        for index, line in enumerate(invalid):
            with self.subTest(line=line):
                self.feedback.ingest(line)
                state = self.feedback.state()
                self.assertEqual(state['invalid_frames'], index + 1)
                self.assertEqual(state['angles_deg'], baseline['angles_deg'])
                self.assertEqual(state['status'], baseline['status'])
                self.assertEqual(state['frames'], 1)
                self.assertAlmostEqual(state['age'], 0.7)
                self.assertAlmostEqual(state['status_age'], 0.7)
                self.assertFalse(state['connected'])

    def test_reader_exit_immediately_marks_fresh_data_disconnected(self):
        self.prime()
        self.reader.poll.return_value = 1
        state = self.feedback.state()
        self.assertEqual(state['age'], 0)
        self.assertEqual(state['status_age'], 0)
        self.assertFalse(state['reader_alive'])
        self.assertFalse(state['connected'])
        self.assertEqual(state['angles_deg'][-1], 73)

    def test_preview_mapping_and_gripper_remains_raw_only(self):
        self.prime()
        before = self.feedback.state()
        expected_degrees = [15, -20, 20, -20, 50, 30]
        self.assertEqual(set(before['positions']), {f'arm_Joint{i}' for i in range(1, 7)})
        for index, expected in enumerate(expected_degrees, 1):
            self.assertAlmostEqual(before['positions'][f'arm_Joint{index}'], math.radians(expected))
        # SDK angle6 is the gripper; changing it cannot drive either simulated finger.
        self.send(angles((10, 20, 30, 40, 50, 60, 991)))
        after = self.feedback.state()
        self.assertEqual(after['positions'], before['positions'])
        self.assertEqual(after['angles_deg'][6], 991)
        self.assertTrue(after['mapping']['preview_only'])
        self.assertEqual(after['mapping']['sha256'], hashlib.sha256(
            Path(self.config['mapping_candidate']).read_bytes()).hexdigest())

    def test_out_of_range_readings_are_visible_and_warned_not_clamped(self):
        self.send(angles((999, 20, 30, 40, 50, 60, 73)))
        self.send(status(error=42))
        state = self.feedback.state()
        self.assertEqual(state['angles_deg'][0], 999)
        self.assertAlmostEqual(state['positions']['arm_Joint1'], math.radians(1004))
        self.assertTrue(any('arm_Joint1' in text for text in state['limits_warning']))
        self.assertTrue(any('42' in text for text in state['limits_warning']))

    def test_read_loop_accepts_feedback_without_sending_commands(self):
        self.reader.stdout = iter([json.dumps(angles()) + '\n', json.dumps(status()) + '\n'])
        self.feedback.read()
        self.assertTrue(self.feedback.state()['connected'])
        self.reader.stdin.write.assert_not_called()

    def test_http_exposes_feedback_but_refuses_all_posts(self):
        self.prime()
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0),
            functools.partial(MONITOR.Handler, feedback=self.feedback))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
        try:
            connection.request('GET', '/api/state')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(response.getheader('Cache-Control'), 'no-store')
            self.assertTrue(json.loads(response.read())['connected'])
            for path in ('/api/control', '/api/enable', '/api/grasp'):
                connection.request('POST', path, json.dumps({'action': 'grasp'}),
                                   {'Content-Type': 'application/json'})
                response = connection.getresponse()
                self.assertEqual(response.status, 403)
                self.assertIn('只读', json.loads(response.read())['error'])
            self.assertFalse(self.feedback.state()['command_capable'])
            self.reader.stdin.write.assert_not_called()
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
