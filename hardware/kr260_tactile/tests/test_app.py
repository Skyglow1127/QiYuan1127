import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import Recorder, configuration, reader_command


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.r = Recorder(self.tmp.name)

    def tearDown(self):
        self.r.stop()
        self.tmp.cleanup()

    def prepare(self, polarity=None):
        self.r.config = configuration({'source': 'demo', 'polarity': polarity or ['unknown', 'unknown']})
        self.r.csv_file = io.StringIO()
        from app import FIELDS
        self.r.writer = csv.DictWriter(self.r.csv_file, fieldnames=FIELDS)
        self.r.writer.writeheader()

    def sample(self, seq, t, a=0, b=0, source='simulated'):
        return dict(seq=seq, t_s=t, do1=a, do2=b, source=source, chip='demo', offsets=[0, 1],
                    wall_time='2026-10-25T00:00:00+00:00')

    def test_raw_values_preserved_and_polarity_applied_once(self):
        self.prepare(['low', 'high'])
        self.r.ingest(self.sample(1, 0, 0, 0))
        self.assertEqual((self.r.latest['do1'], self.r.latest['trigger1']), (0, 1))
        self.assertEqual((self.r.latest['do2'], self.r.latest['trigger2']), (0, 0))

    def test_unknown_polarity_is_not_invented(self):
        self.prepare()
        self.r.ingest(self.sample(1, 0, 1, 0))
        self.assertIsNone(self.r.latest['trigger1'])
        self.assertIsNone(self.r.latest['trigger2'])

    def test_initial_active_not_counted_and_gaps_excluded(self):
        self.prepare(['high', 'low'])
        for row in [(1, 0, 1), (2, .01, 1), (3, .02, 0), (4, .03, 1), (5, 1., 0)]:
            self.r.ingest(self.sample(*row))
        self.assertEqual(self.r.edges[0], 1)
        self.assertAlmostEqual(self.r.active_s[0], .02)
        self.assertEqual(self.r.latest['gap'], 1)

    def test_corrupt_values_or_provenance_stop_before_recording(self):
        self.prepare()
        for sample in (self.sample(1, 0, 2), self.sample(1, float('nan')),
                       self.sample(2, 0), self.sample(1, 0, source='real')):
            with self.assertRaises(ValueError):
                self.r.ingest(sample)
        self.assertEqual(self.r.count, 0)

    def test_nonmonotonic_timestamps_rejected(self):
        self.prepare()
        self.r.ingest(self.sample(1, .1))
        with self.assertRaises(ValueError):
            self.r.ingest(self.sample(2, .09))
        self.assertEqual(self.r.count, 1)

    def test_mismatched_reported_hardware_identity_rejected(self):
        self.prepare()
        for changes in ({'chip': '/dev/gpiochip7'}, {'offsets': [1, 0]},
                        {'offsets': [False, True]}, {'offsets': None}):
            item = self.sample(1, 0)
            item.update(changes)
            with self.assertRaises(ValueError):
                self.r.ingest(item)
        self.assertEqual(self.r.count, 0)

    def test_metadata_write_failure_closes_file_before_start(self):
        with mock.patch.object(Path, 'write_text', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.r.start({'source': 'demo'})
        self.assertIsNone(self.r.csv_file)
        self.assertIsNone(self.r.writer)
        self.assertIsNone(self.r.thread)
        self.assertEqual(self.r.state, 'error')
        self.assertIn('disk full', self.r.error)
        self.assertEqual(self.r.count, 0)

    def test_error_stops_drain_workers_even_with_full_sample_queue(self):
        from app import Recorder as RecorderClass
        workers = []
        original_thread = threading.Thread
        def track_thread(*args, **kwargs):
            worker = original_thread(*args, **kwargs)
            if kwargs.get('target').__name__ == 'drain':
                workers.append(worker)
            return worker
        original_ingest = RecorderClass.ingest
        def delayed_ingest(instance, item):
            # Give the drain worker time to fill its bounded queue, then fail.
            time.sleep(.15)
            return original_ingest(instance, item)
        script = 'import sys,time; sys.stdout.write((\'{"seq":1,"source":"real"}\\n\')*10000); sys.stdout.flush(); time.sleep(10)'
        with mock.patch('app.reader_command', return_value=([sys.executable, '-u', '-c', script], None)), \
             mock.patch('app.threading.Thread', side_effect=track_thread), \
             mock.patch.object(RecorderClass, 'ingest', delayed_ingest):
            self.r.start({'source': 'demo'})
            self.r.thread.join(timeout=5)
        self.assertFalse(self.r.thread.is_alive())
        self.assertEqual(self.r.state, 'error')
        self.assertEqual(self.r.count, 0)
        self.assertEqual(len(workers), 2)
        self.assertTrue(all(not worker.is_alive() for worker in workers))

    def test_real_mapping_required_and_no_physical_pin_guess(self):
        with self.assertRaises(ValueError):
            configuration({'source': 'local'})
        with self.assertRaises(ValueError):
            configuration({'source': 'local', 'chip': '/dev/gpiochip0'})
        with self.assertRaises(ValueError):
            configuration({'source': 'ssh', 'host': 'kr260; echo BAD'})
        config = configuration({'source': 'ssh', 'host': 'kr260', 'chip': '/dev/gpiochip7',
                                'offsets': [2, 3], 'mapping_confirmed': True})
        cmd, script = reader_command(config)
        self.assertIn('StrictHostKeyChecking=yes', cmd)
        self.assertIn('--offsets 2 3', cmd[-1])
        self.assertTrue(script.startswith(b'#!/usr/bin/env python3'))

    def test_demo_end_to_end_csv_stops_and_restarts(self):
        self.r.start({'source': 'demo', 'rate_hz': 100})
        deadline = time.monotonic() + 5
        while self.r.count < 20 and time.monotonic() < deadline:
            time.sleep(.02)
        self.r.stop()
        self.assertEqual(self.r.state, 'stopped', self.r.error)
        self.assertGreaterEqual(self.r.count, 20)
        path = self.r.csv_path
        with path.open(encoding='utf-8-sig') as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), self.r.count)
        self.assertTrue(all(row['source'] == 'simulated' for row in rows))
        self.assertTrue(all(row['trigger1'] == '' for row in rows))
        count = self.r.count
        time.sleep(.05)
        self.assertEqual(self.r.count, count)
        old_session = self.r.session_id
        self.r.start({'source': 'demo', 'rate_hz': 50})
        time.sleep(.1)
        self.r.stop()
        self.assertNotEqual(self.r.session_id, old_session)
        self.assertTrue(path.exists())

    def test_real_failure_never_switches_to_simulation(self):
        self.r.start({'source': 'local', 'chip': '/dev/gpiochip999999', 'mapping_confirmed': True})
        self.r.thread.join(timeout=5)
        self.assertEqual(self.r.state, 'error')
        self.assertEqual(self.r.count, 0)
        self.assertEqual(self.r.status()['source'], 'real')
        self.assertTrue(self.r.error)


if __name__ == '__main__':
    unittest.main()
