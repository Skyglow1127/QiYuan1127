"""Offline recorder tests: synthetic frames/codecs and mock HTTP only; no SDK access."""
import contextlib
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import urllib.error
import urllib.request

import av
import cv2
import numpy as np

import record_global_depth as recorder


SERIAL = 'TEST-GLOBAL-ONLY'
PROFILE = dict(index=0, width=16, height=12, fps=30, format_code=11, format='Y11')


def make_state(path):
    state = recorder.LiveState(SERIAL, 'Synthetic camera', dict(PROFILE), path)
    state.update(preview_near_mm=100, preview_far_mm=2000)
    return state


def samples():
    # High/low bytes, zero, and uint16 extremes catch 8-bit loss and byte swaps.
    base = np.resize(np.array([0, 1, 255, 256, 1023, 2048, 65535], dtype='<u2'),
                     (PROFILE['height'], PROFILE['width']))
    result = []
    for index, (timestamp, sequence, scale) in enumerate(zip(
            [9000000, 9033333, 9071001, 9104000], [20, 21, 24, 25], [1., .1, 2., .25])):
        depth = np.roll(base, index, axis=1).copy()
        result.append(dict(raw=depth.tobytes(), width=PROFILE['width'], height=PROFILE['height'],
            sequence=sequence, device_timestamp_us=timestamp, sdk_host_received_us=timestamp+123456,
            received_monotonic_ns=123000000000+timestamp*1000, mm_per_unit=scale))
    return result


class FakeCamera:
    resources_released = True

    def __init__(self, items):
        self.items = iter(items)
        self.reads = 0

    def read(self, stop):
        self.reads += 1
        return next(self.items, None)


class ProfileAndColorTests(unittest.TestCase):
    def test_profile_priorities_and_explicit_selection(self):
        profiles = [dict(PROFILE), dict(PROFILE, width=640, height=480, fps=90),
                    dict(PROFILE, width=1280, height=800, fps=10),
                    dict(PROFILE, width=9999, height=9999, fps=1000, format_code=5)]
        self.assertEqual(recorder.choose_profile(profiles)['fps'], 90)
        self.assertEqual(recorder.choose_profile(profiles, 'resolution')['width'], 1280)
        self.assertEqual(recorder.choose_profile(profiles, width=16, height=12, fps=30), PROFILE)
        equivalent = [dict(PROFILE, format_code=n) for n in [8, 12, 11]]
        self.assertEqual(recorder.choose_profile(equivalent)['format_code'], 11)
        self.assertEqual(profiles[0], PROFILE, 'selection must not mutate caller profiles')

    def test_profile_rejects_unsupported_or_incomplete_requests(self):
        for kwargs in [dict(width=16), dict(width=16, height=12),
                       dict(width=16, height=12, fps=31), dict(mode='unknown')]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                recorder.choose_profile([PROFILE], **kwargs)
        with self.assertRaises(RuntimeError):
            recorder.choose_profile([dict(PROFILE, format_code=5)])

    def test_colorize_preserves_raw_and_converts_units(self):
        depth = np.array([[0, 100, 200], [1000, 2000, 65535]], dtype=np.uint16)
        before = depth.copy()
        colors, stats = recorder.colorize_depth(depth, .25, 25, 500)
        np.testing.assert_array_equal(depth, before)
        np.testing.assert_array_equal(colors[0, 0], [0, 0, 0])
        self.assertEqual(colors.shape, (2, 3, 3))
        self.assertEqual(colors.dtype, np.uint8)
        self.assertAlmostEqual(stats['valid_fraction'], 5/6)
        self.assertEqual(stats['depth_min_mm'], 25)
        self.assertEqual(stats['depth_max_mm'], 65535*.25)
        # Endpoint colors must reflect distance * scale, with near warm and far cool.
        table = cv2.applyColorMap(np.array([[255, 0]], dtype=np.uint8), cv2.COLORMAP_TURBO)
        np.testing.assert_array_equal(colors[0, 1], table[0, 0])
        np.testing.assert_array_equal(colors[1, 1], table[0, 1])
        black, stats = recorder.colorize_depth(np.zeros((2, 2), np.uint16), 1, 0, 100)
        self.assertFalse(black.any())
        self.assertEqual(stats, dict(valid_fraction=0., depth_min_mm=None, depth_max_mm=None))


class CodecRoundTripTests(unittest.TestCase):
    def run_recording(self, directory, items, color_video=True):
        state = make_state(directory)
        with contextlib.redirect_stdout(io.StringIO()):
            recorder.record_depth(state, FakeCamera(items), directory, color_video=color_video)
        return state

    def assert_frames(self, directory, items, color_video=True):
        expected_hash = hashlib.sha256(b''.join(item['raw'] for item in items)).hexdigest()
        with av.open(str(directory/'depth.mkv')) as video:
            self.assertEqual(video.streams.video[0].codec_context.name, 'ffv1')
            frames = list(video.decode(video=0))
        self.assertEqual(len(frames), len(items))
        decoded = [frame.to_ndarray(format='gray16le').astype('<u2', copy=False) for frame in frames]
        for depth, item, frame in zip(decoded, items, frames):
            np.testing.assert_array_equal(depth, np.frombuffer(item['raw'], '<u2').reshape(12, 16))
            relative = (item['device_timestamp_us']-items[0]['device_timestamp_us'])/1000000
            # Matroska stores this stream at 1 ms precision; CSV retains exact device us.
            self.assertAlmostEqual(float(frame.pts*frame.time_base), relative, delta=.000501)
        self.assertEqual(hashlib.sha256(b''.join(d.tobytes() for d in decoded)).hexdigest(), expected_hash)
        with (directory/'timestamps.csv').open(newline='') as csv_file:
            rows = list(csv.DictReader(csv_file))
        self.assertEqual(len(rows), len(items))
        for index, (row, item) in enumerate(zip(rows, items), 1):
            self.assertEqual(int(row['frame']), index)
            for key in ['sdk_host_received_us', 'received_monotonic_ns', 'device_timestamp_us']:
                self.assertEqual(int(row[key]), item[key])
            self.assertEqual(int(row['sdk_frame_index']), item['sequence'])
            self.assertEqual(float(row['mm_per_unit']), item['mm_per_unit'])
            self.assertEqual(int(row['raw_bytes']), len(item['raw']))
        if color_video:
            with av.open(str(directory/'depth_preview.mp4')) as video:
                self.assertEqual(video.streams.video[0].codec_context.name, 'h264')
                color_frames = list(video.decode(video=0))
            self.assertEqual(len(color_frames), len(items))
            for frame, item in zip(color_frames, items):
                relative = (item['device_timestamp_us']-items[0]['device_timestamp_us'])/1000000
                self.assertAlmostEqual(float(frame.pts*frame.time_base), relative, delta=.000002)
        else:
            self.assertFalse((directory/'depth_preview.mp4').exists())
        return expected_hash

    def test_actual_ffv1_roundtrip_csv_mp4_timestamps_and_scale(self):
        items = samples()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            state = self.run_recording(directory, items)
            digest = self.assert_frames(directory, items)
            status = state.status()
            self.assertEqual(status['raw_depth_sha256'], digest)
            self.assertEqual(status['frames'], 4)
            self.assertEqual(status['estimated_missing_frames'], 2)
            self.assertAlmostEqual(status['elapsed_s'], .104)
            self.assertAlmostEqual(status['measured_fps'], 3/.104)
            self.assertEqual(status['mm_per_unit'], .25)
            self.assertEqual(status['depth_max_mm'], 65535*.25)
            self.assertGreater(status['bytes_written'], 0)

    def test_raw_only_option(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.run_recording(directory, samples()[:2], color_video=False)
            self.assert_frames(directory, samples()[:2], color_video=False)

    def test_color_encoder_failure_keeps_saved_depth_metadata_consistent(self):
        items = samples()
        original = recorder.DepthWriter.write_preview
        preview_calls = 0

        def fail_second_preview(writer, colors, relative_us):
            nonlocal preview_calls
            preview_calls += 1
            if preview_calls == 2:
                raise RuntimeError('injected preview encoder failure')
            return original(writer, colors, relative_us)

        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            state, camera = make_state(directory), FakeCamera(items)
            with mock.patch.object(recorder.DepthWriter, 'write_preview', fail_second_preview), \
                    contextlib.redirect_stdout(io.StringIO()), \
                    self.assertRaisesRegex(RuntimeError, 'injected preview encoder failure'):
                recorder.record_depth(state, camera, directory)
            self.assertEqual(camera.reads, 2, 'must stop immediately after optional encoder failure')
            with av.open(str(directory/'depth.mkv')) as video:
                decoded = [frame.to_ndarray(format='gray16le').astype('<u2', copy=False)
                           for frame in video.decode(video=0)]
            self.assertEqual(len(decoded), 2)
            for depth, item in zip(decoded, items[:2]):
                np.testing.assert_array_equal(depth, np.frombuffer(item['raw'], '<u2').reshape(12, 16))
            with (directory/'timestamps.csv').open(newline='') as csv_file:
                rows = list(csv.DictReader(csv_file))
            self.assertEqual(len(rows), len(decoded))
            for index, (row, item) in enumerate(zip(rows, items[:2]), 1):
                self.assertEqual(int(row['frame']), index)
                self.assertEqual(int(row['sdk_frame_index']), item['sequence'])
                self.assertEqual(int(row['device_timestamp_us']), item['device_timestamp_us'])
                self.assertEqual(float(row['mm_per_unit']), item['mm_per_unit'])
            status = state.status()
            self.assertEqual(status['frames'], len(decoded))
            self.assertEqual(status['raw_depth_sha256'],
                             hashlib.sha256(b''.join(depth.tobytes() for depth in decoded)).hexdigest())
            with av.open(str(directory/'depth_preview.mp4')) as video:
                preview_frames = list(video.decode(video=0))
            self.assertEqual(len(preview_frames), 1)
            self.assertEqual(status['color_video_frames'], len(preview_frames))

    def test_bad_frame_stops_without_losing_prior_frame(self):
        good, following = samples()[:2]
        bad_fields = [dict(sequence=good['sequence']), dict(sequence=-1),
                      dict(device_timestamp_us=good['device_timestamp_us']), dict(device_timestamp_us=-1),
                      dict(mm_per_unit=0), dict(mm_per_unit=-1), dict(mm_per_unit=float('nan')),
                      dict(mm_per_unit=float('inf')), dict(width=18), dict(height=10), dict(raw=b'bad')]
        for fields in bad_fields:
            with self.subTest(fields=fields), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                state = make_state(directory)
                camera = FakeCamera([good, dict(following, **fields), samples()[2]])
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                    recorder.record_depth(state, camera, directory)
                self.assertEqual(camera.reads, 2, 'must not continue after a malformed frame')
                self.assertEqual(state.status()['frames'], 1)
                self.assertEqual(state.status()['raw_depth_sha256'], self.assert_frames(directory, [good]))


class SDKCleanupTests(unittest.TestCase):
    def test_frame_release_failure_remains_failed_after_other_cleanup_succeeds(self):
        # Fake pointers and byte buffers exercise ctypes copying without loading SDK.
        for failing_handle in ('child', 'frameset'):
            with self.subTest(failing_handle=failing_handle):
                camera = recorder.SDKDepthCamera(SERIAL)
                payload = recorder.C.create_string_buffer(b'\x01\x02')
                released = []

                def release(handle):
                    released.append(handle)
                    if handle == failing_handle:
                        raise RuntimeError('injected '+handle+' release failure')

                camera.api = {
                    'ob_pipeline_wait_for_frameset': lambda *_: 'frameset',
                    'ob_frameset_depth_frame': lambda *_: 'child',
                    'ob_video_frame_width': lambda *_: 1,
                    'ob_video_frame_height': lambda *_: 1,
                    'ob_frame_format': lambda *_: 8,
                    'ob_frame_data_size': lambda *_: 2,
                    'ob_frame_data': lambda *_: recorder.C.addressof(payload),
                    'ob_frame_index': lambda *_: 1,
                    'ob_frame_time_stamp_us': lambda *_: 123,
                    'ob_frame_system_time_stamp_us': lambda *_: 456,
                    'ob_depth_frame_get_value_scale': lambda *_: .25,
                    'ob_delete_frame': release,
                }
                for name in ('ob_pipeline_stop', 'ob_delete_stream_profile',
                             'ob_delete_stream_profile_list', 'ob_delete_config',
                             'ob_delete_pipeline', 'ob_delete_device',
                             'ob_delete_device_list', 'ob_delete_context'):
                    camera.api[name] = release
                camera.handles = dict(context='context', device_list='device_list',
                    device='device', pipeline='pipeline', config='config')
                camera.profile_handles, camera.list_handles = {0: 'profile'}, ['profile_list']
                camera.started, camera.resources_released = True, False
                with self.assertRaisesRegex(RuntimeError, 'injected '+failing_handle):
                    camera.read(recorder.threading.Event())
                with contextlib.redirect_stderr(io.StringIO()):
                    camera.close()
                    camera.close()
                self.assertEqual(released, ['child', 'frameset', 'pipeline', 'profile',
                    'profile_list', 'config', 'pipeline', 'device', 'device_list', 'context'])
                self.assertEqual(len(camera.cleanup_errors), 1)
                self.assertIn('ob_delete_frame', camera.cleanup_errors[0])
                self.assertIn(failing_handle, camera.cleanup_errors[0])
                self.assertFalse(camera.resources_released)
                self.assertFalse(camera.started)
                self.assertEqual(camera.handles, {})
                lease = recorder.GlobalPreviewLease(SERIAL, True, camera)
                lease.restore = True
                with mock.patch.object(lease, 'request') as request, \
                        contextlib.redirect_stderr(io.StringIO()):
                    lease.__exit__()
                request.assert_not_called()
                self.assertIn('SDK', lease.restore_error)

    def test_device_info_release_failure_cleans_handles_and_prevents_reopen(self):
        camera = recorder.SDKDepthCamera(SERIAL)
        released = []

        def release(handle):
            released.append(handle)
            if handle == 'info':
                raise RuntimeError('injected info release failure')

        api = {
            'ob_set_logger_to_console': mock.Mock(),
            'ob_create_context': lambda: 'context',
            'ob_query_device_list': lambda *_: 'device_list',
            'ob_device_list_device_count': lambda *_: 1,
            'ob_device_list_get_device_uid': lambda *_: b'fake-device-uid',
            'ob_device_list_get_device_serial_number': lambda *_: SERIAL.encode(),
            'ob_device_list_get_device_by_uid': lambda *_: 'device',
            'ob_device_get_device_info': lambda *_: 'info',
            'ob_device_info_name': lambda *_: b'Synthetic DC1',
            'ob_device_info_serial_number': lambda *_: SERIAL.encode(),
            'ob_delete_device_info': release,
            'ob_delete_device': release,
            'ob_delete_device_list': release,
            'ob_delete_context': release,
            'ob_create_pipeline_with_device': mock.Mock(),
        }
        binding = mock.Mock(api=api)
        with mock.patch.object(recorder, 'Camera', return_value=binding), \
                contextlib.redirect_stderr(io.StringIO()), \
                self.assertRaisesRegex(RuntimeError, 'injected info release failure'):
            camera.__enter__()
        self.assertEqual(released, ['info', 'device', 'device_list', 'context'])
        api['ob_create_pipeline_with_device'].assert_not_called()
        self.assertEqual(len(camera.cleanup_errors), 1)
        self.assertIn('ob_delete_device_info', camera.cleanup_errors[0])
        self.assertFalse(camera.resources_released)
        self.assertEqual(camera.handles, {})


class PreviewLeaseTests(unittest.TestCase):
    def lease(self, takeover=True, released=True):
        return recorder.GlobalPreviewLease(SERIAL, takeover, mock.Mock(resources_released=released))

    def status(self, opened=True, **values):
        return dict(role='global', requested_serial=SERIAL, device={'serial': SERIAL},
                    desired_open=opened, stopped=not opened, resources_released=not opened, **values)

    def test_requests_use_only_global_loopback_and_control_header(self):
        response = mock.MagicMock()
        response.__enter__.side_effect = lambda: io.BytesIO(b'{"ok":true}')
        with mock.patch.object(recorder.OPENER, 'open', return_value=response) as opener:
            lease = self.lease()
            lease.request('/status')
            lease.request('/camera/close', True)
            lease.request('/camera/reconnect', True)
        requests = [call.args[0] for call in opener.call_args_list]
        self.assertEqual([request.full_url for request in requests], [
            'http://127.0.0.1:18761/status', 'http://127.0.0.1:18761/camera/close',
            'http://127.0.0.1:18761/camera/reconnect'])
        self.assertEqual([request.get_method() for request in requests], ['GET', 'POST', 'POST'])
        self.assertIsNone(requests[0].get_header('X-camera-control'))
        self.assertEqual(requests[1].get_header('X-camera-control'), '1')

    def test_identity_mismatch_never_takes_over(self):
        for fields in [dict(role='wrist'), dict(requested_serial='WRONG'), dict(device={'serial': 'WRONG'})]:
            lease = self.lease()
            status = self.status()
            status.update(fields)
            with self.subTest(fields=fields), mock.patch.object(lease, 'request', return_value=status) as request:
                with self.assertRaises(RuntimeError):
                    lease.__enter__()
                self.assertEqual(request.call_args_list, [mock.call('/status')])

    def test_open_service_requires_flag_and_no_robot_clients(self):
        for takeover, clients in [(False, []), (True, [{'pid': 123}])]:
            lease = self.lease(takeover)
            with self.subTest(takeover=takeover), mock.patch.object(lease, 'request', return_value=self.status()) as request, \
                    mock.patch.object(recorder, 'robot_camera_clients', return_value=clients):
                with self.assertRaises(RuntimeError):
                    lease.__enter__()
                self.assertEqual(request.call_args_list, [mock.call('/status')])

    def test_previously_closed_stays_closed(self):
        lease = self.lease()
        with mock.patch.object(lease, 'request', return_value=self.status(False)) as request:
            with lease:
                pass
        self.assertEqual(request.call_args_list, [mock.call('/status')])
        self.assertFalse(lease.restore)

    def test_previously_open_is_restored(self):
        lease = self.lease()
        with mock.patch.object(lease, 'request', side_effect=[self.status(), {}, self.status(False), {}]) as request, \
                mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), contextlib.redirect_stdout(io.StringIO()):
            with lease:
                self.assertTrue(lease.restore)
            lease.__exit__()  # Idempotent: do not reconnect twice.
        self.assertEqual(request.call_args_list, [mock.call('/status'), mock.call('/camera/close', True),
                         mock.call('/status'), mock.call('/camera/reconnect', True)])
        self.assertIsNone(lease.restore_error)

    def test_sdk_cleanup_failure_prevents_reopen(self):
        lease = self.lease()
        with mock.patch.object(lease, 'request', side_effect=[self.status(), {}, self.status(False)]) as request, \
                mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            with lease:
                lease.camera.resources_released = False
        self.assertEqual(request.call_count, 3)
        self.assertIn('SDK', lease.restore_error)

    def test_lost_close_response_still_restores_previous_open_state(self):
        lease = self.lease()
        with mock.patch.object(lease, 'request', side_effect=[self.status(), TimeoutError('lost response'), {}]) as request, \
                mock.patch.object(recorder, 'robot_camera_clients', return_value=[]), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(TimeoutError):
                lease.__enter__()
        self.assertEqual(request.call_args_list[-1], mock.call('/camera/reconnect', True))
        self.assertFalse(lease.restore)

    def test_absent_service_is_allowed_but_unconfirmed_errors_are_rejected(self):
        lease = self.lease()
        with mock.patch.object(lease, 'request', side_effect=urllib.error.URLError(ConnectionRefusedError())):
            self.assertIs(lease.__enter__(), lease)
        for error in [urllib.error.URLError(TimeoutError()),
                      urllib.error.HTTPError('mock', 503, 'Unavailable', {}, None)]:
            with self.subTest(error=error), mock.patch.object(lease, 'request', side_effect=error):
                with self.assertRaises(RuntimeError):
                    lease.__enter__()


class HttpControlTests(unittest.TestCase):
    def test_loopback_server_status_and_control_header(self):
        # This ephemeral server is the recorder's test server, never port 18761/18760.
        state = make_state(Path('/not-a-real-recording'))
        server = recorder.make_preview_server(state, 0)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        base = 'http://127.0.0.1:'+str(server.server_port)
        try:
            self.assertEqual(server.server_address[0], '127.0.0.1')
            with opener.open(base+'/status') as response:
                self.assertEqual(json.load(response)['serial'], SERIAL)
            for headers in [{}, {'X-Recorder-Control': '0'}]:
                request = urllib.request.Request(base+'/stop', data=b'', headers=headers)
                with self.assertRaises(urllib.error.HTTPError) as error:
                    opener.open(request)
                self.assertEqual(error.exception.code, 403)
                self.assertFalse(state.stop.is_set())
            with opener.open(urllib.request.Request(base+'/stop', data=b'',
                            headers={'X-Recorder-Control': '1'})) as response:
                self.assertTrue(json.load(response)['ok'])
            self.assertTrue(state.stop.is_set())
            self.assertEqual(state.status()['state'], 'stopping')
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
