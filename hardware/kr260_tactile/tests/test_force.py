import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import subprocess
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import force_app as f


def fixture():
    ch = dict(sensor_id='TEST_ONLY', external_sensor_verified=True,
              device='/sys/bus/iio/devices/iio:device9', channel='in_voltage0',
              device_name='TEST_ONLY', of_node='/sys/firmware/devicetree/base/test',
              min_v=0, max_v=1, raw_min=0, raw_max=1000, scale_mv=1, offset_raw=0,
              calibration=dict(reference='TEST_ONLY', mounting='TEST_ONLY', points=[[.1, 0], [.5, 4], [.9, 8]]))
    c = dict(electrical_verified=True, verification_record='TEST_ONLY', host='test-only',
             rate_hz=20, stale_s=.5, channels=[ch, copy.deepcopy(ch)])
    c['channels'][1]['channel'] = 'in_voltage1'
    return c


class ForceTests(unittest.TestCase):
    def test_default_blocks(self):
        with self.assertRaisesRegex(ValueError, '等待'):
            f.validate(json.loads((f.ROOT / 'analog_config.json').read_text()))

    def test_no_duplicate_sensor_channel(self):
        c = fixture(); c['channels'][1]['channel'] = 'in_voltage0'
        with self.assertRaises(ValueError): f.validate(c)

    def test_external_identity_required(self):
        c = fixture(); c['channels'][0]['external_sensor_verified'] = False
        with self.assertRaises(ValueError): f.validate(c)

    def test_voltage_limit(self):
        c = fixture(); c['channels'][0]['max_v'] = 3.3
        with self.assertRaises(ValueError): f.validate(c)

    def test_calibration_interpolates_without_extrapolation(self):
        c = f.validate(fixture())['channels'][0]
        self.assertEqual(f.convert(300, c), (.3, 1.9999999999999998, 'ok'))
        self.assertEqual(f.convert(950, c), (.95, None, 'outside_calibration'))

    def test_reverse_sensor_response(self):
        c = fixture(); c['channels'][0]['calibration']['points'] = [[.1, 8], [.5, 4], [.9, 0]]
        f.validate(c)
        self.assertAlmostEqual(f.convert(700, c['channels'][0])[1], 2)

    def test_uncalibrated_is_not_force(self):
        c = fixture()['channels'][0]; c['calibration'] = {}
        self.assertEqual(f.convert(500, c), (.5, None, 'uncalibrated'))

    def test_offset_and_millivolts(self):
        c = fixture()['channels'][0]; c['offset_raw'] = 10; c['scale_mv'] = .5
        self.assertAlmostEqual(f.convert(500, c)[0], .255)

    def test_saturation_and_range_are_invalid(self):
        c = fixture()['channels'][0]
        self.assertEqual(f.convert(1000, c)[1:], (None, 'adc_rail'))
        self.assertEqual(f.convert(1100, c)[1:], (None, 'out_of_range'))

    def test_calibration_rejects_nonmonotonic_or_missing_provenance(self):
        c = fixture(); c['channels'][0]['calibration']['points'][2][1] = 2
        with self.assertRaises(ValueError): f.validate(c)
        c = fixture(); c['channels'][0]['calibration']['reference'] = ''
        with self.assertRaises(ValueError): f.validate(c)

    def test_nan_rejected(self):
        c = fixture(); c['channels'][0]['scale_mv'] = float('nan')
        with self.assertRaises(ValueError): f.validate(c)

    def test_stale_and_stopped_invalidate_robot_value(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 'config.json'; p.write_text(json.dumps(fixture()))
            r = f.Recorder(p); r.config = fixture(); r.file = io.StringIO()
            r.writer = f.csv.DictWriter(r.file, fieldnames=f.FIELDS)
            frame = dict(seq=1, t_s=.01, read_span_s=.001, source='real_iio', wall_time='TEST_ONLY', raw=[300, 700])
            with patch.object(f.time, 'monotonic', return_value=10):
                r.ingest(frame)
                self.assertTrue(r.status()['channels'][0]['force_valid'])
            with patch.object(f.time, 'monotonic', return_value=11):
                self.assertIsNone(r.status()['channels'][0]['force_n'])
            r.state = 'stopped'
            with patch.object(f.time, 'monotonic', return_value=10.1):
                self.assertFalse(r.status()['channels'][0]['force_valid'])
            self.assertIn('real_iio', r.file.getvalue())

    def test_no_demo_or_reordered_frames(self):
        r = f.Recorder('/nonexistent'); r.config = fixture()
        with self.assertRaises(ValueError): r.ingest(dict(seq=1, source='simulated'))
        with self.assertRaises(ValueError): r.ingest(dict(seq=2, source='real_iio'))

    def test_buffered_network_delay_rejected(self):
        r = f.Recorder('/nonexistent'); r.config = fixture(); r.file = io.StringIO()
        r.writer = f.csv.DictWriter(r.file, fieldnames=f.FIELDS)
        frame = dict(seq=1, t_s=.01, read_span_s=.001, source='real_iio', wall_time='TEST_ONLY', raw=[300, 700])
        with patch.object(f.time, 'monotonic', return_value=10): r.ingest(frame)
        frame.update(seq=2, t_s=.06)
        with patch.object(f.time, 'monotonic', return_value=11):
            with self.assertRaisesRegex(ValueError, '延迟'): r.ingest(frame)

    def test_device_identity_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td); (p / 'name').write_text('rail-monitor')
            ch = fixture()['channels'][0]; ch['device'] = str(p)
            with self.assertRaisesRegex(ValueError, '名称'): f.verify_device(ch)

    def test_sysfs_scale_identity_and_raw_read_path(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td); node = p / 'node'; node.mkdir()
            (p / 'name').write_text('TEST_ONLY'); (p / 'of_node').symlink_to(node)
            (p / 'in_voltage_scale').write_text('1')
            (p / 'in_voltage0_raw').write_text('300')
            ch = fixture()['channels'][0]; ch.update(device=str(p), of_node=str(node))
            self.assertEqual(int(f.verify_device(ch).read_text()), 300)
            (p / 'in_voltage_scale').write_text('2')
            with self.assertRaisesRegex(ValueError, '刻度'): f.verify_device(ch)

    def test_stream_csv_stop_and_restart_with_test_subprocess(self):
        # Generated frames are confined to a temporary test directory, never real evidence.
        original_popen = subprocess.Popen
        frame = dict(seq=1, t_s=.01, read_span_s=.001, source='real_iio', wall_time='TEST_ONLY', raw=[300, 700])
        code = 'import sys,time;sys.stdin.read();print(' + repr(json.dumps(frame)) + ',flush=True);time.sleep(30)'
        def child(*args, **kwargs):
            return original_popen([sys.executable, '-u', '-c', code], **kwargs)
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 'config.json'; p.write_text(json.dumps(fixture()))
            r = f.Recorder(p, Path(td) / 'records')
            with patch.object(f.subprocess, 'Popen', side_effect=child):
                for _ in range(2):
                    r.start()
                    deadline = time.monotonic() + 3
                    while r.latest is None and time.monotonic() < deadline: time.sleep(.02)
                    self.assertIsNotNone(r.latest, r.error)
                    self.assertEqual(r.latest['voltage1_v'], .3)
                    r.stop()
                    self.assertFalse(r.thread.is_alive())
                    self.assertEqual(r.state, 'stopped')
                    self.assertIn('TEST_ONLY', r.path.read_text())
                    self.assertIsNone(r.status()['channels'][0]['force_n'])
            self.assertEqual(len(list((Path(td) / 'records').glob('*.csv'))), 2)


if __name__ == '__main__': unittest.main()
