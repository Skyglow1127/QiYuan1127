#!/usr/bin/env python3
"""Offline recorder checks: synthetic MJPEG, temporary files, loopback HTTP only.

No camera node, SDK, CAN interface or existing preview service is opened.
"""
import contextlib
import csv
from fractions import Fraction
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import urllib.error
import urllib.request

import av


MODULE_PATH = Path(__file__).with_name('record_wrist_video.py')
SPEC = importlib.util.spec_from_file_location('wrist_video_recorder_under_test', MODULE_PATH)
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)
SERIAL = 'CC1365301EY'
PROFILE = dict(format='MJPG', width=32, height=24, fps=30.)


def preview_status(*, opened=True, **overrides):
    status = dict(role='wrist', requested_serial=SERIAL, device={'serial': SERIAL},
                  desired_open=opened, stopped=not opened, resources_released=not opened)
    status.update(overrides)
    return status


class FakeCapture:
    def __init__(self, frames):
        self.frames = iter(frames)
        self.reads = 0

    def read(self, stop):
        self.reads += 1
        return next(self.frames, None)


class RecorderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Actual MJPEG encoder packets, with distinct frame content.
        encoder = av.CodecContext.create('mjpeg', 'w')
        encoder.width, encoder.height, encoder.pix_fmt = 32, 24, 'yuvj420p'
        encoder.time_base = Fraction(1, 30)
        cls.jpeg = []
        for index in range(4):
            frame = av.VideoFrame(32, 24, 'yuvj420p')
            for plane_index, plane in enumerate(frame.planes):
                value = 32 + 20 * index if plane_index == 0 else 128
                plane.update(bytes([value]) * plane.buffer_size)
            frame.pts = index
            cls.jpeg.extend(bytes(packet) for packet in encoder.encode(frame))
        cls.jpeg.extend(bytes(packet) for packet in encoder.encode(None))
        if len(cls.jpeg) != 4:
            raise AssertionError('Synthetic MJPEG encoder did not produce four packets')

    def test_choose_highest_resolution_and_fps(self):
        modes = [dict(format='YUYV', width=2048, height=1536, fps=30.),
                 dict(format='MJPG', width=640, height=480, fps=30.),
                 dict(format='MJPG', width=1632, height=1224, fps=15.),
                 dict(format='MJPG', width=1632, height=1224, fps=30.)]
        self.assertEqual(recorder.choose_mode(modes), modes[-1])
        self.assertEqual(recorder.choose_mode(modes, 640, 480, 30), modes[1])

    def test_choose_rejects_missing_unsupported_and_conflicting_modes(self):
        modes = [dict(format='MJPG', width=640, height=480, fps=30.),
                 dict(format='MJPG', width=1632, height=1224, fps=15.)]
        for values in ((640, None, None), (None, 480, 30), (640, 480, None),
                       (800, 600, 30), (640, 480, 60)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                recorder.choose_mode(modes, *values)
        with self.assertRaises(RuntimeError):
            recorder.choose_mode(modes)
        with self.assertRaises(RuntimeError):
            recorder.choose_mode([dict(format='YUYV', width=640, height=480, fps=30.)])

    def test_preview_lease_rejects_wrong_identity_without_action(self):
        for changed in ({'role': 'global'}, {'requested_serial': 'WRONG'},
                        {'device': {'serial': 'WRONG'}}):
            lease = recorder.PreviewLease(SERIAL, True)
            with self.subTest(changed=changed), mock.patch.object(
                    lease, 'request', return_value=preview_status(**changed)) as request:
                with self.assertRaises(RuntimeError):
                    lease.__enter__()
                self.assertEqual(request.call_args_list, [mock.call('/status')])

    def test_preview_lease_requires_takeover_authorization(self):
        lease = recorder.PreviewLease(SERIAL, False)
        with mock.patch.object(lease, 'request', return_value=preview_status()) as request:
            with self.assertRaises(RuntimeError):
                lease.__enter__()
            self.assertEqual(request.call_args_list, [mock.call('/status')])

    def test_preview_lease_restores_previously_open_service(self):
        lease = recorder.PreviewLease(SERIAL, True)
        with mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), \
                mock.patch.object(lease, 'request', side_effect=[
                    preview_status(), {'ok': True}, preview_status(opened=False), {'ok': True}
                ]) as request, contextlib.redirect_stdout(io.StringIO()):
            with lease:
                pass
            # Repeated cleanup must not issue another reconnect.
            lease.__exit__(None, None, None)
            self.assertEqual(request.call_args_list, [mock.call('/status'),
                mock.call('/camera/close', True), mock.call('/status'),
                mock.call('/camera/reconnect', True)])

    def test_preview_lease_keeps_previously_closed_service_closed(self):
        lease = recorder.PreviewLease(SERIAL, True)
        with mock.patch.object(lease, 'request', return_value=preview_status(opened=False)) as request:
            with lease:
                pass
            self.assertEqual(request.call_args_list, [mock.call('/status')])

    def test_preview_lease_finishes_prior_close_without_reopening(self):
        lease = recorder.PreviewLease(SERIAL, True)
        with mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), \
                mock.patch.object(lease, 'request', side_effect=[
                    preview_status(opened=False, stopped=False, resources_released=False),
                    {'ok': True}, preview_status(opened=False)
                ]) as request, contextlib.redirect_stdout(io.StringIO()):
            with lease:
                pass
            self.assertEqual(request.call_args_list, [mock.call('/status'),
                mock.call('/camera/close', True), mock.call('/status')])

    def test_preview_lease_restores_when_close_response_is_lost(self):
        lease = recorder.PreviewLease(SERIAL, True)
        failure = urllib.error.URLError(TimeoutError('response lost after close'))
        with mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), \
                mock.patch.object(lease, 'request', side_effect=[
                    preview_status(), failure, {'ok': True}
                ]) as request, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(urllib.error.URLError):
                lease.__enter__()
            self.assertEqual(request.call_args_list, [mock.call('/status'),
                mock.call('/camera/close', True), mock.call('/camera/reconnect', True)])

    def test_preview_lease_refuses_active_robot_client(self):
        lease = recorder.PreviewLease(SERIAL, True)
        with mock.patch.object(recorder, 'robot_camera_clients', return_value=[{'pid': 123}]), \
                mock.patch.object(lease, 'request', return_value=preview_status()) as request:
            with self.assertRaises(RuntimeError):
                lease.__enter__()
            self.assertEqual(request.call_args_list, [mock.call('/status')])

    def test_robot_client_detection_includes_execed_launchers(self):
        for script in ('launch_act.py', 'record_piper.py'):
            with self.subTest(script=script):
                argv = ['/usr/bin/python3', '/synthetic/project/' + script, '--run']
                cmdline = mock.Mock()
                cmdline.parent = Path('/proc/1234')
                cmdline.read_bytes.return_value = ('\0'.join(argv) + '\0').encode()
                with mock.patch.object(recorder.Path, 'glob', return_value=[cmdline]):
                    self.assertEqual(recorder.robot_camera_clients(),
                                     [dict(pid=1234, argv=argv)])

    def assert_recording_matches(self, output, state, accepted):
        raw_frames = [item[0] for item in accepted]
        status = state.status()
        self.assertEqual(status['frames'], len(accepted))
        self.assertEqual(status['payload_bytes'], sum(map(len, raw_frames)))
        self.assertEqual(status['source_mjpeg_sha256'],
                         hashlib.sha256(b''.join(raw_frames)).hexdigest())
        self.assertEqual(status['bytes_written'], output.stat().st_size)
        with output.with_suffix('.timestamps.csv').open(newline='') as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), len(accepted))
        self.assertEqual([int(row['driver_sequence']) for row in rows],
                         [item[2] for item in accepted])
        self.assertEqual([int(row['driver_timestamp_us']) for row in rows],
                         [item[1] for item in accepted])
        self.assertEqual([int(row['mjpeg_bytes']) for row in rows], list(map(len, raw_frames)))
        # Demux verifies exact packet preservation, not merely successful decoding.
        with av.open(str(output)) as video:
            packets = [packet for packet in video.demux(video=0) if packet.size]
            self.assertEqual([bytes(packet) for packet in packets], raw_frames)
            times = [float(packet.pts * packet.time_base) for packet in packets]
        expected_times = [(item[1] - accepted[0][1]) / 1_000_000 for item in accepted]
        for actual, expected in zip(times, expected_times):
            self.assertAlmostEqual(actual, expected, delta=.001)
        with av.open(str(output)) as video:
            decoded = list(video.decode(video=0))
        self.assertEqual(len(decoded), len(accepted))
        self.assertTrue(all((frame.width, frame.height) == (32, 24) for frame in decoded))

    def test_invalid_sequence_or_timestamp_keeps_all_outputs_consistent(self):
        for fault in ('repeated_sequence', 'backward_sequence', 'backward_timestamp'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / 'sample.mkv'
                state = recorder.LiveState(SERIAL, 'Synthetic DC1', PROFILE, output)
                accepted = [(self.jpeg[0], 1_000_000, 10, 0x2000),
                            (self.jpeg[1], 1_033_333, 11, 0x2000)]
                timestamp = 1_020_000 if fault == 'backward_timestamp' else 1_066_666
                sequence = {'repeated_sequence': 11, 'backward_sequence': 9,
                            'backward_timestamp': 12}[fault]
                capture = FakeCapture(accepted + [(self.jpeg[2], timestamp, sequence, 0x2000)])
                with self.assertRaises(RuntimeError), contextlib.redirect_stdout(io.StringIO()):
                    recorder.record(state, capture, output, 0)
                self.assert_recording_matches(output, state, accepted)

    def test_sequence_wrap_and_missing_frame_accounting(self):
        for sequences, missing in (((0xFFFFFFFE, 0xFFFFFFFF, 0), 0), ((10, 13, 14), 2)):
            with self.subTest(sequences=sequences), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / 'sample.mkv'
                state = recorder.LiveState(SERIAL, 'Synthetic DC1', PROFILE, output)
                accepted = [(self.jpeg[i], 1_000_000 + i * 33_333, sequence, 0x2000)
                            for i, sequence in enumerate(sequences)]
                with contextlib.redirect_stdout(io.StringIO()):
                    recorder.record(state, FakeCapture(accepted), output, 0)
                self.assert_recording_matches(output, state, accepted)
                self.assertEqual(state.status()['estimated_missing_frames'], missing)

    def test_loopback_preview_and_stop_header_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            state = recorder.LiveState(SERIAL, 'Synthetic DC1', PROFILE,
                                       Path(directory) / 'sample.mkv')
            server = recorder.preview_server(state, 0)
            try:
                self.assertEqual(server.server_address[0], '127.0.0.1')
                base = f'http://127.0.0.1:{server.server_port}'
                with recorder.OPENER.open(base + '/status', timeout=2) as response:
                    self.assertEqual(json.load(response)['serial'], SERIAL)
                denied = urllib.request.Request(base + '/stop', data=b'')
                with self.assertRaises(urllib.error.HTTPError) as error:
                    recorder.OPENER.open(denied, timeout=2)
                self.assertEqual(error.exception.code, 403)
                self.assertFalse(state.stop.is_set())
                allowed = urllib.request.Request(base + '/stop', data=b'',
                            headers={'X-Recorder-Control': '1'})
                with recorder.OPENER.open(allowed, timeout=2) as response:
                    self.assertTrue(json.load(response)['ok'])
                self.assertTrue(state.stop.is_set())
                self.assertEqual(state.status()['state'], 'stopping')
            finally:
                state.stop.set()
                server.shutdown()
                server.server_close()

    def test_existing_video_or_sidecars_are_not_overwritten(self):
        for suffix in ('.mkv', '.json', '.timestamps.csv'):
            with self.subTest(suffix=suffix), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / 'sample.mkv'
                existing = output.with_suffix(suffix)
                existing.write_bytes(b'keep this recording')
                with mock.patch.object(recorder, 'wrist_identity',
                        return_value=(SERIAL, 'Synthetic DC1', Path('/never-open-camera'))), \
                        mock.patch.object(recorder, 'enumerate_modes', return_value=[PROFILE]), \
                        mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), \
                        mock.patch.object(recorder, 'PreviewLease') as lease, \
                        mock.patch.object(recorder, 'UVCCapture') as capture, \
                        mock.patch.object(recorder, 'preview_server') as server, \
                        contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(recorder.main(['--output', str(output), '--no-browser']), 1)
                    lease.assert_not_called()
                    capture.assert_not_called()
                    server.assert_not_called()
                self.assertEqual(existing.read_bytes(), b'keep this recording')
                self.assertEqual(list(Path(directory).iterdir()), [existing])

    def test_record_itself_uses_exclusive_video_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'sample.mkv'
            output.write_bytes(b'existing video')
            state = recorder.LiveState(SERIAL, 'Synthetic DC1', PROFILE, output)
            capture = FakeCapture([])
            with self.assertRaises(FileExistsError):
                recorder.record(state, capture, output, 0)
            self.assertEqual(output.read_bytes(), b'existing video')
            self.assertEqual(capture.reads, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
