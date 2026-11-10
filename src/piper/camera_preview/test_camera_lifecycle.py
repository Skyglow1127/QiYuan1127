"""Software-only lifecycle regression checks. Never loads the camera SDK."""
import contextlib
import importlib.util
import io
from collections import deque
from pathlib import Path
import threading
import unittest


SOURCE = Path(__file__).with_name('camera_server.py')
SPEC = importlib.util.spec_from_file_location('camera_server_under_test', SOURCE)
server = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(server)


class FakeCamera:
    def __init__(self, factory):
        self.factory = factory
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.entered = threading.Event()
        self.cleanup_started = threading.Event()
        self.release_gate = threading.Event()
        self.release_gate.set()
        self.finished = threading.Event()
        self.resources_released = True
        self.cleanup_fails = False
        self.error = None

    def capture(self):
        with self.factory.lock:
            self.factory.active += 1
            self.factory.peak = max(self.factory.peak, self.factory.active)
        self.resources_released = False
        self.entered.set()
        self.stop.wait()
        self.cleanup_started.set()
        self.release_gate.wait(5)
        self.resources_released = not self.cleanup_fails
        if self.cleanup_fails:
            self.error = 'fake SDK release failed'
        with self.factory.lock:
            self.factory.active -= 1
            self.factory.order.append(('finished', self))
        self.finished.set()

    def status(self):
        return dict(stopped=self.stop.is_set(), resources_released=self.resources_released,
                    error=self.error, streams={'color': {'age_ms': None}})


class FakeFactory:
    def __init__(self):
        self.cameras = []
        self.order = []
        self.lock = threading.Lock()
        self.active = self.peak = 0

    def __call__(self):
        camera = FakeCamera(self)
        with self.lock:
            self.cameras.append(camera)
            self.order.append(('created', camera))
        return camera


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.factory = FakeFactory()
        self.controller = server.CameraController(self.factory)

    def tearDown(self):
        for camera in self.factory.cameras:
            camera.release_gate.set()
        try:
            self.controller.control('close', wait=True)
        except RuntimeError:
            pass

    def reconnect(self):
        self.controller.control('reconnect')
        camera = self.controller.current()
        self.assertTrue(camera.entered.wait(1), 'fake capture did not start')
        return camera

    def test_fresh_stop_events_and_repeatable_close(self):
        initial = self.controller.current()
        self.assertTrue(initial.stop.is_set())
        first = self.reconnect()
        first_event = first.stop
        self.controller.control('close')
        self.assertTrue(first.finished.is_set())
        self.assertEqual(self.controller.status()['state'], 'closed')
        second = self.reconnect()
        self.assertIsNot(first_event, second.stop)
        self.assertTrue(initial.stop.is_set())
        self.assertTrue(first_event.is_set(), 'old MJPEG request must stay stopped')
        self.assertFalse(second.stop.is_set())
        self.controller.control('close')
        self.controller.control('close')
        self.assertEqual(self.factory.active, 0)
        self.assertEqual(self.factory.peak, 1)
        self.assertEqual(self.controller.generation, 2)

    def test_reconnect_waits_for_release_and_rejects_overlapping_request(self):
        old = self.reconnect()
        old.release_gate.clear()
        results, errors = [], []

        def request():
            try:
                results.append(self.controller.control('reconnect'))
            except Exception as error:
                errors.append(error)

        request_thread = threading.Thread(target=request)
        request_thread.start()
        try:
            self.assertTrue(old.cleanup_started.wait(1))
            self.assertTrue(old.stop.is_set())
            self.assertFalse(old.finished.is_set())
            self.assertEqual(len(self.factory.cameras), 2,
                             'must not create a new camera before old capture finishes')
            with self.assertRaises(BlockingIOError):
                self.controller.control('reconnect')
            self.assertEqual(len(self.factory.cameras), 2)
        finally:
            old.release_gate.set()
            request_thread.join(2)
        self.assertFalse(request_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(len(results), 1)
        new = self.controller.current()
        self.assertTrue(new.entered.wait(1))
        self.assertTrue(old.finished.is_set())
        self.assertTrue(old.stop.is_set())
        self.assertLess(self.factory.order.index(('finished', old)),
                        self.factory.order.index(('created', new)))
        self.assertEqual(self.factory.peak, 1, 'capture generations overlapped')

    def test_failed_release_blocks_new_capture(self):
        old = self.reconnect()
        old.cleanup_fails = True
        with self.assertRaisesRegex(RuntimeError, 'fake SDK release failed'):
            self.controller.control('reconnect')
        self.assertTrue(old.finished.is_set())
        self.assertTrue(old.stop.is_set())
        self.assertIs(self.controller.current(), old)
        self.assertFalse(old.resources_released)
        self.assertEqual(len(self.factory.cameras), 2)
        self.assertEqual(self.factory.active, 0)


def camera_with_fake_sdk(fail_release=None):
    """Construct only Python fields; deliberately bypass Camera.__init__/CDLL."""
    camera = server.Camera.__new__(server.Camera)
    camera.requested_serial = 'TEST123'
    camera.role = 'global'
    camera.label = 'Fake camera'
    camera.condition = threading.Condition()
    camera.stop = threading.Event()
    camera.stop.set()  # Exercise allocation/release without a frame polling loop.
    camera.latest = {'color': (b'old-frame', {})}
    camera.arrivals = {name: deque([1.0]) for name in ('color', 'depth')}
    camera.counts = dict(color=0, depth=0)
    camera.error = None
    camera.device = {}
    camera.profiles = {}
    camera.calibration = None
    camera.calibration_error = None
    camera.resources_released = True
    releases = []

    def release(name):
        def run(handle):
            releases.append((name, handle))
            if (name, handle) == fail_release:
                raise RuntimeError(name + ': fake release failure')
        return run

    def fake_calibration(_):
        raise ValueError('fake calibration intentionally unavailable')

    api = dict(
        ob_set_logger_to_console=lambda level: None,
        ob_create_context=lambda: 'context',
        ob_query_device_list=lambda context: 'devices',
        ob_device_list_device_count=lambda devices: 1,
        ob_device_list_get_device_uid=lambda devices, index: b'fake-uid',
        ob_device_list_get_device_serial_number=lambda devices, index: b'TEST123',
        ob_device_list_get_device_by_uid=lambda devices, uid: 'device',
        ob_device_get_device_info=lambda device: 'info',
        ob_device_info_name=lambda info: b'Fake',
        ob_device_info_serial_number=lambda info: b'TEST123',
        ob_create_pipeline_with_device=lambda device: 'pipeline',
        ob_create_config=lambda: 'config',
        ob_pipeline_get_stream_profile_list=lambda pipeline, sensor: sensor,
        ob_stream_profile_list_count=lambda profiles: 1,
        ob_stream_profile_list_get_profile=lambda profiles, index: profiles,
        ob_video_stream_profile_width=lambda profile: 640,
        ob_video_stream_profile_height=lambda profile: {2: 480, 3: 400}[profile],
        ob_video_stream_profile_fps=lambda profile: 30,
        ob_stream_profile_format=lambda profile: {2: 5, 3: 11}[profile],
        ob_config_enable_stream=lambda config, profile: None,
        ob_config_set_frame_aggregate_output_mode=lambda config, mode: None,
        ob_pipeline_start_with_config=lambda pipeline, config: None,
        ob_pipeline_get_camera_param=fake_calibration,
    )
    for name in ('ob_delete_device_info', 'ob_pipeline_stop', 'ob_delete_stream_profile',
                 'ob_delete_stream_profile_list', 'ob_delete_config', 'ob_delete_pipeline',
                 'ob_delete_device', 'ob_delete_device_list', 'ob_delete_context'):
        api[name] = release(name)
    camera.api = api
    return camera, releases


class ResourceCleanupTests(unittest.TestCase):
    expected = [('ob_delete_device_info', 'info'), ('ob_pipeline_stop', 'pipeline'),
                ('ob_delete_stream_profile', 2), ('ob_delete_stream_profile', 3),
                ('ob_delete_stream_profile_list', 2), ('ob_delete_stream_profile_list', 3),
                ('ob_delete_config', 'config'), ('ob_delete_pipeline', 'pipeline'),
                ('ob_delete_device', 'device'), ('ob_delete_device_list', 'devices'),
                ('ob_delete_context', 'context')]

    def test_every_finalizer_is_attempted_after_any_finalizer_fails(self):
        # Include first-of-many profile/list release failures, not only stop.
        for failed in self.expected[1:]:
            with self.subTest(failed=failed):
                camera, releases = camera_with_fake_sdk(failed)
                with contextlib.redirect_stdout(io.StringIO()):
                    camera.capture()
                self.assertEqual(releases, self.expected)
                self.assertFalse(camera.resources_released)
                self.assertIn('fake release failure', camera.error)
                self.assertTrue(camera.stop.is_set())
                self.assertEqual(camera.latest, {})
                self.assertTrue(all(not arrivals for arrivals in camera.arrivals.values()))

    def test_successful_release_clears_cached_frames(self):
        camera, releases = camera_with_fake_sdk()
        with contextlib.redirect_stdout(io.StringIO()):
            camera.capture()
        self.assertEqual(releases, self.expected)
        self.assertTrue(camera.resources_released)
        self.assertIsNone(camera.error)
        self.assertEqual(camera.latest, {})


if __name__ == '__main__':
    unittest.main(verbosity=2)
