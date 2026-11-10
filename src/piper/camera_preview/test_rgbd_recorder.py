"""Offline RGBD recording checks: real codecs, synthetic captures, local HTTP only."""
import contextlib
import csv
from fractions import Fraction
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock
import urllib.error
import urllib.request

import av
import numpy as np

import record_rgbd as recorder


RGB_ID = dict(role='wrist', serial='TEST-WRIST', label='腕部相机')
DEPTH_ID = dict(role='global', serial='TEST-GLOBAL', label='全局相机')
PROFILE = dict(width=32, height=24, fps=30, format='MJPEG', index=0, format_code=11)
START_NS = 123000000000


def state_at(directory):
    for folder in ('RGB', 'Depth'):
        (directory/folder).mkdir()
    state = recorder.RecordingState(RGB_ID, DEPTH_ID, directory, 200, 2000)
    state.session_start_ns = START_NS
    state.update(state='recording')
    for name in ('rgb', 'depth'):
        state.update_stream(name, profile=dict(PROFILE))
    return state


def jpeg_image(width=32, height=24, value=100):
    image = np.empty((height, width, 3), np.uint8)
    image[:] = [value, 80, 210]
    codec = av.CodecContext.create('mjpeg', 'w')
    codec.width, codec.height = width, height
    codec.pix_fmt, codec.time_base = 'yuvj420p', Fraction(1, 30)
    frame = av.VideoFrame.from_ndarray(image, format='bgr24')
    return b''.join(bytes(packet) for packet in codec.encode(frame)+codec.encode(None))


def depth_items():
    depth = np.full((24, 32), 700, dtype='<u2')
    depth[:12, :16] = 0
    depth[:, 16:] = 1500
    return [dict(raw=depth.tobytes(), width=32, height=24, sequence=40+i,
        device_timestamp_us=7000000+i*33333, sdk_host_received_us=9000000+i*33333,
        received_monotonic_ns=START_NS+offset*1000, mm_per_unit=.5 if i == 2 else 1.)
        for i, offset in enumerate((20000, 53333, 86666, 120000))]


def rgb_items():
    return [(jpeg_image(value=90+i*10), 1000000+i*33333, 20+i, 0)
            for i in range(4)]


class FakeCapture:
    def __init__(self, items, clock=None):
        self.items, self.clock, self.reads = iter(items), clock, 0

    def read(self, stop):
        self.reads += 1
        if self.clock is not None:
            self.clock[0] = START_NS+(10000+(self.reads-1)*33333)*1000
        return next(self.items, None)


def read_video(path):
    with av.open(str(path)) as video:
        codec = video.streams.video[0].codec_context.name
        frames = list(video.decode(video=0))
    return codec, frames


class RealCodecTests(unittest.TestCase):
    def test_startup_frames_are_discarded_until_common_session_start(self):
        for name,folder,items,first_saved_sequence in (
                ('rgb','RGB',rgb_items(),22),('depth','Depth',depth_items(),42)):
            with self.subTest(stream=name), tempfile.TemporaryDirectory() as temporary:
                directory=Path(temporary)
                state=state_at(directory)
                state.session_start_ns=None
                clock=[START_NS]
                common_start=START_NS+70000*1000
                test=self

                class WarmupCapture:
                    reads=0
                    def read(self,stop):
                        if self.reads==2:
                            test.assertTrue(state.ready[name].is_set())
                            test.assertEqual(state.status()['streams'][name]['frames'],0)
                            test.assertEqual(state.status()['streams'][name]['warmup_discarded_frames'],2)
                            state.session_start_ns=common_start
                        if self.reads==len(items):
                            return None
                        item=items[self.reads]
                        clock[0]=(START_NS+(10000+self.reads*33333)*1000 if name=='rgb'
                                  else item['received_monotonic_ns'])
                        self.reads+=1
                        return item

                with mock.patch.object(recorder.time,'monotonic_ns',side_effect=lambda:clock[0]):
                    recorder.record_stream(name,state,WarmupCapture(),directory,PROFILE)
                self.assertFalse(state.stop.is_set(),state.status()['error'])
                self.assertTrue(state.ready[name].is_set())
                self.assertEqual(state.status()['streams'][name]['frames'],2)
                path=directory/folder/(name+'.mp4')
                codec,frames=read_video(path)
                self.assertEqual(codec,'h264')
                self.assertEqual(len(frames),2,'startup frames must not be encoded')
                with path.with_suffix('.timestamps.csv').open(newline='') as stream:
                    rows=list(csv.DictReader(stream))
                self.assertEqual([int(row['source_sequence']) for row in rows],
                                 [first_saved_sequence,first_saved_sequence+1])
                self.assertEqual([int(row['frame']) for row in rows],[1,2])
                for frame,row in zip(frames,rows):
                    pts=(int(row['received_monotonic_ns'])-common_start)//1000
                    self.assertGreaterEqual(pts,0)
                    self.assertEqual(int(row['session_pts_us']),pts)
                    self.assertAlmostEqual(float(frame.pts*frame.time_base),pts/1e6,delta=.000002)

    def test_rgb_decoder_initialization_failure_stops_other_stream_immediately(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)
            state=state_at(directory)
            capture=FakeCapture(rgb_items())
            failing_codec=mock.Mock()
            failing_codec.create.side_effect=RuntimeError('injected decoder initialization failure')
            with mock.patch.object(recorder.av,'CodecContext',failing_codec):
                recorder.record_stream('rgb',state,capture,directory,PROFILE)
            self.assertTrue(state.stop.is_set())
            self.assertEqual(state.status()['state'],'stopping')
            self.assertIn('injected decoder initialization failure',state.status()['error'])
            self.assertEqual(capture.reads,0)
            self.assertEqual(state.status()['streams']['rgb']['frames'],0)
            self.assertFalse((directory/'RGB/rgb.mp4').exists())
            self.assertFalse((directory/'RGB/rgb.timestamps.csv').exists())

    def test_two_h264_mp4s_csv_shared_time_offsets_and_pseudocolor(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            state, clock = state_at(directory), [START_NS]
            with mock.patch.object(recorder.time, 'monotonic_ns', side_effect=lambda:clock[0]):
                recorder.record_stream('rgb', state, FakeCapture(rgb_items(), clock), directory, PROFILE)
                recorder.record_stream('depth', state, FakeCapture(depth_items()), directory, PROFILE)
            self.assertFalse(state.stop.is_set(), state.status()['error'])
            for name, folder in (('rgb', 'RGB'), ('depth', 'Depth')):
                path = directory/folder/(name+'.mp4')
                codec, frames = read_video(path)
                self.assertEqual(codec, 'h264')
                self.assertEqual(len(frames), 4)
                self.assertEqual([(f.width, f.height) for f in frames], [(32, 24)]*4)
                self.assertTrue(all(f.format.name == 'yuv420p' for f in frames))
                with path.with_suffix('.timestamps.csv').open(newline='') as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), len(frames))
                for index, (frame, row) in enumerate(zip(frames, rows)):
                    receipt = int(row['received_monotonic_ns'])
                    pts = (receipt-START_NS)//1000
                    self.assertEqual(int(row['session_pts_us']), pts)
                    self.assertAlmostEqual(float(frame.pts*frame.time_base), pts/1e6, delta=.000002)
                    self.assertEqual(int(row['source_sequence']), (20 if name == 'rgb' else 40)+index)
                    self.assertEqual(int(row['source_timestamp_us']), (1000000 if name=='rgb' else 7000000)+index*33333)
                    self.assertEqual(int(row['frame']), index+1)
                    if name=='depth':
                        self.assertEqual(int(row['sdk_host_received_us']),9000000+index*33333)
                        self.assertEqual(float(row['mm_per_unit']),.5 if index==2 else 1.)
                        self.assertEqual(row['source_flags'],'')
                    else:
                        self.assertEqual(row['sdk_host_received_us'],'')
                        self.assertEqual(row['mm_per_unit'],'')
                        self.assertEqual(row['source_flags'],'0')
                # Each file retains its own initial receipt offset in the one session clock.
                self.assertAlmostEqual(float(frames[0].pts*frames[0].time_base), .01 if name=='rgb' else .02, delta=.000002)
                self.assertEqual(state.status()['streams'][name]['frames'], 4)
                self.assertGreater(state.status()['streams'][name]['bytes_written'], 0)
            _, depth_frames = read_video(directory/'Depth/depth.mp4')
            colors = depth_frames[0].to_ndarray(format='bgr24')
            self.assertEqual(colors.dtype, np.uint8)
            self.assertEqual(colors.shape, (24, 32, 3))
            self.assertLess(int(colors[3:9, 3:11].max()), 8, 'invalid depth remains black away from chroma edges')
            self.assertGreater(int(colors[16, 8].max())-int(colors[16, 8].min()), 50)
            self.assertFalse(list(directory.rglob('*.mkv')))
            self.assertFalse(list(directory.rglob('*.raw')))
            status = state.status()
            self.assertFalse(status['spatially_aligned'])
            self.assertFalse(status['exposure_synchronized'])
            self.assertIn('pseudocolor', status['video_codec'])

    def test_depth_fault_stops_both_streams_and_retains_playable_prefix(self):
        good, following, later = depth_items()[:3]
        failures = [dict(sequence=good['sequence']), dict(sequence=-1),
                    dict(device_timestamp_us=good['device_timestamp_us']), dict(device_timestamp_us=-1),
                    dict(mm_per_unit=0), dict(mm_per_unit=-1), dict(mm_per_unit=float('nan')),
                    dict(mm_per_unit=float('inf')), dict(width=30), dict(height=22), dict(raw=b'bad')]
        for fields in failures:
            with self.subTest(fields=fields), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                state = state_at(directory)
                capture = FakeCapture([good, dict(following, **fields), later])
                recorder.record_stream('depth', state, capture, directory, PROFILE)
                self.assertTrue(state.stop.is_set())
                self.assertEqual(state.status()['state'], 'stopping')
                self.assertTrue(state.status()['streams']['depth']['error'])
                self.assertEqual(capture.reads, 2)
                self.assertEqual(len(read_video(directory/'Depth/depth.mp4')[1]), 1)
                with (directory/'Depth/depth.timestamps.csv').open(newline='') as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))), 1)

    def test_rgb_fault_stops_both_and_retains_playable_prefix(self):
        good, following, later = rgb_items()[:3]
        failures = [(following[0], good[1], following[2], 0),
                    (following[0], following[1], good[2], 0),
                    (following[0], -1, following[2], 0),
                    (following[0], following[1], -1, 0),
                    (b'not-jpeg', following[1], following[2], 0),
                    (jpeg_image(width=30), following[1], following[2], 0)]
        for bad in failures:
            with self.subTest(bad_timestamp_sequence=bad[1:3]), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                state, clock = state_at(directory), [START_NS]
                capture = FakeCapture([good, bad, later], clock)
                with mock.patch.object(recorder.time, 'monotonic_ns', side_effect=lambda:clock[0]):
                    recorder.record_stream('rgb', state, capture, directory, PROFILE)
                self.assertTrue(state.stop.is_set())
                self.assertEqual(capture.reads, 2)
                self.assertEqual(len(read_video(directory/'RGB/rgb.mp4')[1]), 1)

    def test_existing_mp4_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'existing.mp4'
            path.write_bytes(b'keep this file')
            with self.assertRaises(FileExistsError):
                with recorder.MP4Writer(path, PROFILE):
                    pass
            self.assertEqual(path.read_bytes(), b'keep this file')


def preview_status(identity=DEPTH_ID, opened=True):
    return dict(role=identity['role'], requested_serial=identity['serial'],
                device=dict(serial=identity['serial']), desired_open=opened,
                stopped=not opened, resources_released=not opened)


class PreviewOwnershipTests(unittest.TestCase):
    def test_role_selects_bound_port_and_control_header(self):
        for identity, port in ((RGB_ID,18760), (DEPTH_ID,18761)):
            with self.subTest(role=identity['role']):
                requests=[]
                def open_request(request, timeout):
                    requests.append(request)
                    return io.BytesIO(b'{}')
                lease=recorder.PreviewLease(identity,True,lambda:True)
                with mock.patch.object(recorder.OPENER,'open',side_effect=open_request):
                    lease.request('/status')
                    lease.request('/camera/close',True)
                self.assertEqual(requests[0].full_url,f'http://127.0.0.1:{port}/status')
                self.assertEqual(requests[0].get_method(),'GET')
                self.assertEqual(requests[1].get_method(),'POST')
                self.assertEqual(requests[1].get_header('X-camera-control'),'1')

    def test_wrong_identity_missing_flag_or_active_clients_never_close(self):
        wrong = [dict(role='wrist'), dict(requested_serial='wrong'), dict(device={'serial':'wrong'})]
        for fields in wrong:
            with self.subTest(fields=fields):
                lease=recorder.PreviewLease(DEPTH_ID,True,lambda:True)
                with mock.patch.object(lease,'request',return_value=dict(preview_status(),**fields)) as request, \
                     self.assertRaisesRegex(RuntimeError,'身份'):
                    lease.__enter__()
                self.assertEqual(request.call_args_list,[mock.call('/status')])
        for takeover,clients in ((False,[]),(True,[{'pid':123}])):
            with self.subTest(takeover=takeover,clients=clients):
                lease=recorder.PreviewLease(DEPTH_ID,takeover,lambda:True)
                with mock.patch.object(lease,'request',return_value=preview_status()) as request, \
                     mock.patch.object(recorder,'robot_camera_clients',return_value=clients), \
                     self.assertRaises(RuntimeError):
                    lease.__enter__()
                self.assertEqual(request.call_args_list,[mock.call('/status')])

    def test_restore_only_previous_open_state_after_confirmed_cleanup(self):
        for was_open,cleaned in ((True,True),(True,False),(False,True)):
            with self.subTest(was_open=was_open,cleaned=cleaned):
                lease=recorder.PreviewLease(DEPTH_ID,True,lambda:cleaned)
                response=[preview_status(opened=was_open),{},preview_status(opened=False),{}]
                with mock.patch.object(lease,'request',side_effect=response) as request, \
                     mock.patch.object(recorder,'robot_camera_clients',return_value=[]), \
                     contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    with lease:
                        pass
                calls=request.call_args_list
                reconnect=mock.call('/camera/reconnect',True)
                self.assertEqual(reconnect in calls,was_open and cleaned)
                self.assertEqual(lease.restore_error is not None,was_open and not cleaned)
                if not was_open:
                    self.assertEqual(calls,[mock.call('/status')])

    def test_unconfirmed_service_release_and_nonrefusal_transport_fail_closed(self):
        lease=recorder.PreviewLease(DEPTH_ID,True,lambda:False)
        with mock.patch.object(lease,'request',side_effect=[preview_status(),{},preview_status(opened=True)]) as request, \
             mock.patch.object(recorder,'robot_camera_clients',return_value=[]), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()), \
             self.assertRaisesRegex(RuntimeError,'释放'):
            lease.__enter__()
        self.assertNotIn(mock.call('/camera/reconnect',True),request.call_args_list)
        for error in (urllib.error.URLError(TimeoutError('timeout')),
                      urllib.error.HTTPError('http://local',503,'busy',{},None)):
            lease=recorder.PreviewLease(DEPTH_ID,True,lambda:True)
            with mock.patch.object(lease,'request',side_effect=error), self.assertRaises(RuntimeError):
                lease.__enter__()
        lease=recorder.PreviewLease(DEPTH_ID,True,lambda:True)
        with mock.patch.object(lease,'request',side_effect=urllib.error.URLError(ConnectionRefusedError())):
            self.assertIs(lease.__enter__(),lease)
            self.assertFalse(lease.restore)


class HTTPAndMainTests(unittest.TestCase):
    def test_rgb_recovery_guard_runs_only_after_both_preview_leases_enter(self):
        events=[]
        test=self

        class Lease:
            restore_error=None
            def __init__(self,identity,takeover,can_restore):
                self.role=identity['role']
                self.entered=False
            def __enter__(self):
                self.entered=True
                events.append('enter-'+self.role)
                return self
            def check_closed(self):
                test.assertTrue(self.entered)
                test.assertIn('enter-wrist',events)
                test.assertIn('enter-global',events)
                events.append('closed-'+self.role)
            def __exit__(self,*_):
                events.append('exit-'+self.role)

        def resolve(serial, *, guard):
            self.assertEqual(serial,RGB_ID['serial'])
            self.assertEqual(events,['enter-wrist','enter-global'])
            guard()
            self.assertEqual(events,['enter-wrist','enter-global','closed-wrist','closed-global'])
            events.append('recover-rgb-interface')
            raise RuntimeError('stop after simulated recovery')

        def identity(role):
            return RGB_ID if role=='wrist' else DEPTH_ID

        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/'new-session'
            with mock.patch.object(recorder,'robot_camera_clients',return_value=[]) as clients, \
                 mock.patch.object(recorder,'camera_identity',side_effect=identity), \
                 mock.patch.object(recorder,'rgb_device',side_effect=resolve), \
                 mock.patch.object(recorder,'PreviewLease',Lease), \
                 mock.patch.object(recorder,'SDKDepthCamera') as camera, \
                 mock.patch.object(recorder,'UVCCapture') as capture, \
                 mock.patch.object(recorder,'make_server',return_value=mock.Mock(server_port=1)), \
                 mock.patch.object(recorder.time,'sleep'), \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                camera.return_value.resources_released=True
                camera.return_value.cleanup_errors=[]
                result=recorder.main(['--no-browser','--rgb-camera','wrist','--depth-camera','global',
                                      '--takeover-preview','--output-dir',str(directory)])
            self.assertEqual(result,1)
            self.assertEqual(clients.call_count,2,'recheck active robot clients immediately before recovery')
            capture.assert_not_called()
            camera.return_value.__enter__.assert_not_called()
            self.assertEqual(events,['enter-wrist','enter-global','closed-wrist','closed-global',
                                     'recover-rgb-interface','exit-global','exit-wrist'])
            summary=json.loads((directory/'recording.json').read_text())
            self.assertIn('stop after simulated recovery',summary['error'])

    def test_new_robot_client_or_reopened_preview_blocks_rgb_recovery(self):
        for reason in ('new-client','reopened-preview'):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as temporary:
                lease=mock.MagicMock()
                lease.restore_error=None
                lease.__enter__.return_value=lease
                if reason=='reopened-preview':
                    lease.check_closed.side_effect=RuntimeError('preview reopened')
                recovered=mock.Mock()
                def resolve(serial, *, guard):
                    guard()
                    recovered()
                    raise AssertionError('recovery must be refused')
                with mock.patch.object(recorder,'robot_camera_clients',
                          side_effect=[[],[{'pid':123}]] if reason=='new-client' else [[],[]]), \
                     mock.patch.object(recorder,'camera_identity',return_value=RGB_ID), \
                     mock.patch.object(recorder,'rgb_device',side_effect=resolve), \
                     mock.patch.object(recorder,'PreviewLease',return_value=lease), \
                     mock.patch.object(recorder,'SDKDepthCamera') as camera, \
                     mock.patch.object(recorder,'UVCCapture') as capture, \
                     mock.patch.object(recorder,'make_server',return_value=mock.Mock(server_port=1)), \
                     mock.patch.object(recorder.time,'sleep'), \
                     contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    camera.return_value.resources_released=True
                    camera.return_value.cleanup_errors=[]
                    result=recorder.main(['--no-browser','--output-dir',str(Path(temporary)/'session')])
                self.assertEqual(result,1)
                lease.__enter__.assert_called_once()
                recovered.assert_not_called()
                capture.assert_not_called()

    def test_second_worker_start_failure_joins_first_before_capture_cleanup(self):
        finished=threading.Event()
        cleanup=[]
        created=[]
        real_thread=threading.Thread

        def worker(name,state,*_):
            state.stop.wait(timeout=2)
            finished.set()

        def make_thread(**kwargs):
            thread=real_thread(**kwargs)
            created.append(thread)
            if kwargs.get('name')=='record-depth':
                thread.start=mock.Mock(side_effect=RuntimeError('injected second start failure'))
            return thread

        test=self
        class DepthCapture:
            resources_released=True
            cleanup_errors=[]
            profiles=[PROFILE]
            device_info={'serial':DEPTH_ID['serial']}
            def __enter__(self):
                self.resources_released=False
                return self
            def start(self,profile):
                pass
            def __exit__(self,*_):
                test.assertTrue(finished.is_set(),'worker must exit before SDK cleanup')
                self.resources_released=True
                cleanup.append('depth')

        class RGBCapture:
            fd=None
            def __enter__(self):
                self.fd=99
                return self
            def __exit__(self,*_):
                test.assertTrue(finished.is_set(),'worker must exit before UVC cleanup')
                self.fd=None
                cleanup.append('rgb')

        class Lease:
            restore_error=None
            def __init__(self,identity,takeover,can_restore):
                self.can_restore=can_restore
            def __enter__(self):
                return self
            def __exit__(self,*_):
                test.assertTrue(self.can_restore(),'restoration must follow both capture releases')
                cleanup.append('restore')

        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/'new-session'
            with mock.patch.object(recorder,'robot_camera_clients',return_value=[]), \
                 mock.patch.object(recorder,'camera_identity',return_value=DEPTH_ID), \
                 mock.patch.object(recorder,'rgb_device',return_value=Path('/synthetic-device')), \
                 mock.patch.object(recorder,'enumerate_modes',return_value=[PROFILE]), \
                 mock.patch.object(recorder,'choose_mode',return_value=PROFILE), \
                 mock.patch.object(recorder,'choose_profile',return_value=PROFILE), \
                 mock.patch.object(recorder,'PreviewLease',Lease), \
                 mock.patch.object(recorder,'SDKDepthCamera',return_value=DepthCapture()), \
                 mock.patch.object(recorder,'UVCCapture',return_value=RGBCapture()), \
                 mock.patch.object(recorder,'record_stream',side_effect=worker), \
                 mock.patch.object(recorder,'make_server',return_value=mock.Mock(server_port=1)), \
                 mock.patch.object(recorder.threading,'Thread',side_effect=make_thread), \
                 mock.patch.object(recorder.time,'sleep'), \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                result=recorder.main(['--no-browser','--output-dir',str(directory)])
            self.assertEqual(result,1)
            self.assertEqual(cleanup,['rgb','depth','restore'])
            self.assertEqual(len(created),2)
            self.assertFalse(created[0].is_alive())
            self.assertIsNone(created[1].ident,'failed start must never be joined')
            summary=json.loads((directory/'recording.json').read_text())
            self.assertEqual(summary['state'],'error')
            self.assertIn('injected second start failure',summary['error'])
            self.assertTrue(summary['sdk_resources_released'])

    def test_both_previews_status_and_stop_header_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            state=state_at(Path(temporary))
            image=jpeg_image()
            for name in ('rgb','depth'):
                state.publish(name,1,image)
            server=recorder.make_server(state,0)
            url=f'http://127.0.0.1:{server.server_port}'
            opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
            try:
                with opener.open(url+'/status',timeout=3) as response:
                    self.assertEqual(json.load(response)['state'],'recording')
                with opener.open(url+'/',timeout=3) as response:
                    page=response.read()
                    self.assertIn(b'/rgb.mjpg',page)
                    self.assertIn(b'/depth.mjpg',page)
                for name in ('rgb','depth'):
                    with opener.open(url+'/'+name+'.mjpg',timeout=3) as response:
                        self.assertIn('multipart/x-mixed-replace',response.headers['Content-Type'])
                        expected=b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(image)).encode()+b'\r\n\r\n'+image+b'\r\n'
                        self.assertEqual(response.read(len(expected)),expected)
                for headers in ({},{'X-Recorder-Control':'0'}):
                    request=urllib.request.Request(url+'/stop',data=b'',headers=headers)
                    with self.assertRaises(urllib.error.HTTPError) as failure:
                        opener.open(request,timeout=3)
                    self.assertEqual(failure.exception.code,403)
                    self.assertFalse(state.stop.is_set())
                request=urllib.request.Request(url+'/stop',data=b'',headers={'X-Recorder-Control':'1'})
                with opener.open(request,timeout=3) as response:
                    self.assertTrue(json.load(response)['ok'])
                self.assertTrue(state.stop.is_set())
                self.assertEqual(state.status()['state'],'stopping')
            finally:
                state.stop.set()
                with state.condition:
                    state.condition.notify_all()
                server.shutdown()
                server.server_close()

    def test_existing_output_directory_rejected_before_camera_takeover(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)
            marker=directory/'keep.txt'
            marker.write_text('do not overwrite')
            with mock.patch.object(recorder,'robot_camera_clients',return_value=[]), \
                 mock.patch.object(recorder,'camera_identity',return_value=DEPTH_ID), \
                 mock.patch.object(recorder,'rgb_device',return_value=Path('/synthetic-device')) as rgb_resolution, \
                 mock.patch.object(recorder,'enumerate_modes',return_value=[PROFILE]), \
                 mock.patch.object(recorder,'choose_mode',return_value=PROFILE), \
                 mock.patch.object(recorder,'PreviewLease') as lease, \
                 mock.patch.object(recorder,'SDKDepthCamera') as camera, \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(recorder.main(['--no-browser','--output-dir',str(directory)]),1)
            lease.assert_not_called()
            camera.assert_not_called()
            rgb_resolution.assert_not_called()
            self.assertEqual(marker.read_text(),'do not overwrite')
            self.assertEqual(list(directory.iterdir()),[marker])


if __name__ == '__main__':
    unittest.main()
