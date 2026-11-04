"""Deterministic camera-adapter tests; no camera, CAN, robot or network IO."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import hashlib
import inspect
import json
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), '/home/aup/Lerobot/projects/piper/deployment',
               '/home/aup/Lerobot/projects/piper/official_adapter', '/home/aup/piper SDK/pyAgxArm']
import camera_recovery as recovery
import client
import plan_workflow
import rollout_expert
from lerobot_robot_piper_executor.guard import ActionGuard, GuardError, GuardLimits
from lerobot_robot_piper_executor.robot import PiperPolicyExecutor
from lerobot_robot_piper_shared.devices import PiperHttpCameraConfig
from nativequeuepolicy import QUEUE_CONTRACT


class Clock:
    def __init__(self, now=100.):
        self.now = now
    def __call__(self):
        return self.now
    def advance(self, duration):
        self.now += duration


class Response:
    def __init__(self, clock, *, index=1, age=5., generation=1, serial='global-serial',
                 role='global', header_delay=0., read_delays=(.001,)):
        self.clock, self.header_delay, self.read_delays = clock, header_delay, list(read_delays)
        self.headers = {'X-Frame-Index': str(index), 'X-Board-Age-Ms': str(age),
            'X-Camera-Generation': str(generation), 'X-Camera-Serial': serial, 'X-Camera-Role': role,
            'Content-Length': str(len(self.read_delays))}
        self.timeouts = []
        self.fp = SimpleNamespace(raw=SimpleNamespace(_sock=SimpleNamespace(settimeout=self.timeouts.append)))
    def __enter__(self):
        return self
    def __exit__(self, *_):
        pass
    def read1(self, _size):
        self.clock.advance(self.read_delays.pop(0))
        return b'x'


class Opener:
    def __init__(self, clock, responses):
        self.clock, self.responses, self.calls = clock, list(responses), []
    def open(self, url, timeout):
        self.calls.append((url, timeout))
        item = self.responses.pop(0)
        if isinstance(item, BaseException):
            raise item
        self.clock.advance(item.header_delay)
        return item


class Future:
    def __init__(self, function, args, kwargs):
        self.function, self.args, self.kwargs = function, args, kwargs
    def result(self):
        return self.function(*self.args, **self.kwargs)


class Executor:
    def submit(self, function, *args, **kwargs):
        return Future(function, args, kwargs)


class ScriptCamera:
    def __init__(self, clock, script, role='global'):
        self.clock, self.script, self.calls = clock, list(script), []
        self.config = SimpleNamespace(max_age_ms=150, role=role, serial=role+'-serial')
        self.last_metadata = None
        self.last_fetch_diagnostic = None
        self.active = False
    def async_read(self, *, timeout_ms, deadline):
        if self.active:
            raise AssertionError('Concurrent reads of one camera')
        self.active = True
        self.calls.append((timeout_ms, deadline))
        try:
            step = self.script.pop(0)
            if isinstance(step, BaseException):
                raise step
            index, source_at = step(deadline) if callable(step) else step
            self.last_metadata = {'index': index, 'serial': self.config.serial,
                                  'estimated_capture_monotonic': source_at}
            return 'image-'+self.config.role+'-'+str(index)
        finally:
            self.active = False


def backend(clock, cameras):
    result = object.__new__(recovery.VLACameraRecoveryBackend)
    result._camera_clock = clock
    result.io = {'max_camera_age_ms': 150, 'max_camera_skew_ms': 60}
    result.cameras, result.pool = cameras, Executor()
    result.last_camera_pair_timing = None
    return result


def camera(clock, role='global'):
    result = recovery.VLABoundedHttpCamera(PiperHttpCameraConfig(role=role, serial=role+'-serial'),
                                          clock=clock, sleep=clock.advance)
    result.connected = True
    return result


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.decode = patch.object(recovery.cv2, 'imdecode', return_value=self.image)
        self.rgb = patch.object(recovery.cv2, 'cvtColor', side_effect=lambda value, _: value)
        self.decode.start()
        self.rgb.start()
        self.addCleanup(self.decode.stop)
        self.addCleanup(self.rgb.stop)

    def test_single_150ms_rejection_then_genuinely_new_frame_succeeds(self):
        clock = Clock()
        global_camera = camera(clock)
        opener = Opener(clock, [Response(clock, index=1, age=140., read_delays=(.02,)),
                                Response(clock, index=2, age=4., read_delays=(.001,))])
        wrist = ScriptCamera(clock, [lambda _: (10, clock())], 'wrist')
        pair = backend(clock, {'global': global_camera, 'wrist': wrist})
        with patch.object(recovery, 'OPENER', opener):
            images, source = pair._read_camera_pair()
        self.assertEqual(global_camera.last_index, 2)
        self.assertEqual(global_camera.last_metadata['index'], 2)
        self.assertEqual(len(wrist.calls), 1)
        self.assertEqual(pair.last_camera_pair_timing['attempts'], 2)
        self.assertAlmostEqual(global_camera.last_metadata['estimated_capture_monotonic'], 100.016)
        self.assertAlmostEqual(source, 100.016)
        self.assertEqual(len(images), 2)

    def test_persistent_timeouts_stop_at_one_shared_deadline_and_four_attempts(self):
        clock = Clock()
        class TimedOut(ScriptCamera):
            def async_read(self, *, timeout_ms, deadline):
                self.calls.append((timeout_ms, deadline))
                clock.advance(min(.051, max(0., deadline-clock())))
                raise TimeoutError('SYNTHETIC persistent timeout')
        failing = TimedOut(clock, [])
        pair = backend(clock, {'global': failing,
                              'wrist': ScriptCamera(clock, [lambda _: (10, clock())], 'wrist')})
        with self.assertRaises(GuardError):
            pair._read_camera_pair()
        self.assertLessEqual(len(failing.calls), 4)
        self.assertEqual({deadline for _, deadline in failing.calls}, {100.2})
        self.assertAlmostEqual(clock(), 100.2)

    def test_continuously_old_frames_are_discarded_after_four_attempts(self):
        clock = Clock()
        instance = camera(clock)
        opener = Opener(clock, [Response(clock, index=index, age=151.) for index in range(1, 5)])
        pair = backend(clock, {'global': instance,
                              'wrist': ScriptCamera(clock, [lambda _: (10, clock())], 'wrist')})
        with patch.object(recovery, 'OPENER', opener), self.assertRaises(GuardError):
            pair._read_camera_pair()
        self.assertEqual(len(opener.calls), 4)
        self.assertIsNone(instance.last_metadata)
        self.assertIsNone(instance.last_index)

    def test_only_older_frame_is_refreshed_and_its_old_source_is_not_relabelled(self):
        clock = Clock()
        old = ScriptCamera(clock, [(1, 99.99), lambda _: (2, clock())])
        def later(_):
            clock.advance(.080)
            return 8, clock()
        newest = ScriptCamera(clock, [later], 'wrist')
        pair = backend(clock, {'global': old, 'wrist': newest})
        _, source = pair._read_camera_pair()
        self.assertEqual(len(old.calls), 2)
        self.assertEqual(len(newest.calls), 1)
        self.assertEqual(old.last_metadata['index'], 2)
        self.assertAlmostEqual(source, 100.080)
        self.assertLessEqual(pair.last_camera_pair_timing['skew_ms'], 60.)

    def test_pair_must_recheck_images_that_aged_while_other_camera_waited(self):
        clock = Clock()
        first = ScriptCamera(clock, [(1, 99.99), lambda _: (2, clock())])
        def later(_):
            clock.advance(.160)
            return 8, clock()
        second = ScriptCamera(clock, [later], 'wrist')
        pair = backend(clock, {'global': first, 'wrist': second})
        _, source = pair._read_camera_pair()
        self.assertEqual(len(first.calls), 2)
        self.assertEqual(len(second.calls), 1)
        self.assertAlmostEqual(source, 100.16)

    def test_fatal_errors_are_not_retried_but_all_futures_are_drained(self):
        for error in (HTTPError('http://camera', 503, 'unavailable', {}, None),
                      URLError(ConnectionResetError('SYNTHETIC reset')), RuntimeError('identity mismatch')):
            with self.subTest(error=type(error).__name__):
                clock = Clock()
                first = ScriptCamera(clock, [error, (2, 100.)])
                second = ScriptCamera(clock, [(8, 100.)], 'wrist')
                pair = backend(clock, {'global': first, 'wrist': second})
                with self.assertRaises(type(error)):
                    pair._read_camera_pair()
                self.assertEqual(len(first.calls), 1)
                self.assertEqual(len(second.calls), 1)

    def test_url_wrapped_timeout_can_recover(self):
        clock = Clock()
        first = ScriptCamera(clock, [URLError(TimeoutError('SYNTHETIC')), (2, 100.)])
        second = ScriptCamera(clock, [(8, 100.)], 'wrist')
        pair = backend(clock, {'global': first, 'wrist': second})
        pair._read_camera_pair()
        self.assertEqual(len(first.calls), 2)
        self.assertEqual(len(second.calls), 1)

    def test_partial_body_and_decode_must_finish_before_original_deadline(self):
        for phase in ('read', 'decode'):
            with self.subTest(phase=phase):
                clock = Clock()
                instance = camera(clock)
                response = Response(clock, read_delays=(.08, .08, .08) if phase == 'read' else (.001,))
                opener = Opener(clock, [response])
                def decode(*_):
                    if phase == 'decode':
                        clock.advance(.201)
                    return self.image
                with patch.object(recovery, 'OPENER', opener), patch.object(recovery.cv2, 'imdecode', side_effect=decode):
                    with self.assertRaises(TimeoutError):
                        instance.async_read(deadline=100.2)
                self.assertIsNone(instance.last_metadata)
                self.assertIsNone(instance.last_index)
                if phase == 'read':
                    self.assertEqual(len(response.timeouts), 3)
                    self.assertGreater(response.timeouts[0], response.timeouts[1])
                    self.assertGreater(response.timeouts[1], response.timeouts[2])

    def test_metadata_rejects_identity_generation_rollback_and_bad_decode(self):
        for fault in ('serial', 'role', 'generation', 'rollback', 'decode'):
            with self.subTest(fault=fault):
                clock = Clock()
                instance = camera(clock)
                instance.last_index, instance._accepted_generation = 10, 1
                previous = {'index': 10, 'estimated_capture_monotonic': 99.99}
                instance.last_metadata = previous
                kwargs = {'index': 9 if fault == 'rollback' else 11}
                if fault == 'serial': kwargs['serial'] = 'wrong'
                if fault == 'role': kwargs['role'] = 'wrist'
                if fault == 'generation': kwargs['generation'] = 2
                opener = Opener(clock, [Response(clock, **kwargs)])
                with patch.object(recovery, 'OPENER', opener), \
                     patch.object(recovery.cv2, 'imdecode', return_value=None if fault == 'decode' else self.image):
                    with self.assertRaises(RuntimeError):
                        instance.async_read()
                self.assertIs(instance.last_metadata, previous)
                self.assertEqual(instance.last_index, 10)
                self.assertEqual(len(opener.calls), 1)

    def test_duplicate_latest_frame_keeps_source_and_ages_from_original_timestamp(self):
        clock = Clock()
        instance = camera(clock)
        previous = {'index': 10, 'estimated_capture_monotonic': 99.96, 'age_ms': 5.}
        instance.last_metadata, instance.last_index, instance._accepted_generation = previous, 10, 1
        opener = Opener(clock, [Response(clock, index=10, age=40.)])
        with patch.object(recovery, 'OPENER', opener), self.assertRaises(TimeoutError):
            instance.read_latest(max_age_ms=20)
        self.assertIs(instance.last_metadata, previous)
        self.assertEqual(instance.last_metadata['estimated_capture_monotonic'], 99.96)

    def test_both_active_futures_finish_before_fatal_returns(self):
        entered, release = threading.Event(), threading.Event()
        completed = []
        class First(ScriptCamera):
            def async_read(self, **_):
                if not entered.wait(1):
                    raise AssertionError('Second future never started')
                release.set()
                raise RuntimeError('SYNTHETIC fatal')
        class Second(ScriptCamera):
            def async_read(self, **_):
                entered.set()
                if not release.wait(1):
                    raise AssertionError('First future never drained')
                self.last_metadata = {'estimated_capture_monotonic': 100., 'index': 8}
                completed.append('second')
                return 'wrist-image'
        clock = Clock()
        pair = backend(clock, {'global': First(clock, []), 'wrist': Second(clock, [], 'wrist')})
        with ThreadPoolExecutor(max_workers=2) as pool:
            pair.pool = pool
            with self.assertRaises(RuntimeError):
                pair._read_camera_pair()
            self.assertEqual(completed, ['second'])

    def test_factory_keeps_tracking_original_connect_and_exact_guard_limits(self):
        created = []
        config = SimpleNamespace(deployment_config=HERE/'trial/execute_config.json')
        def factory(value):
            self.assertIs(value, config)
            robot = object.__new__(PiperPolicyExecutor)
            robot.connected, robot.backend, robot.guard = False, None, None
            created.append(robot)
            return robot
        context = SimpleNamespace(make_robot_from_config=factory)
        limits = GuardLimits(**json.loads((HERE/'trial/execute_config.json').read_text())['guard_limits'])
        supplied = SimpleNamespace(guard_limits=limits)
        with patch.object(recovery.VLACameraRecoveryBackend, 'from_file', return_value=supplied) as loader:
            with recovery.vla_camera_backend_injection(context):
                robot = context.make_robot_from_config(config)
                self.assertIs(robot, created[0])
                self.assertIs(robot.backend, supplied)
                self.assertIsInstance(robot.guard, ActionGuard)
                self.assertEqual(robot.guard.limits, limits)
                self.assertFalse(robot.is_connected)
            loader.assert_called_once_with(config.deployment_config)
        self.assertIs(context.make_robot_from_config, factory)
        self.assertIs(PiperPolicyExecutor.connect, recovery.PiperPolicyExecutor.connect)

    def test_preview_uses_same_backend_and_existing_command_contract_is_unchanged(self):
        config = json.loads(client.OBSERVE_CONFIG.read_text())
        fake = SimpleNamespace(execution_enabled=False, client=SimpleNamespace(request=lambda *_args, **_kwargs: {}),
                               open_read_only=lambda: None, close=lambda: None)
        with patch.object(recovery.VLACameraRecoveryBackend, 'from_file', return_value=fake) as factory:
            source = client.PassiveSource()
            source.open()
            self.assertIs(source.backend, fake)
            source.close()
            factory.assert_called_once_with(client.OBSERVE_CONFIG)
        self.assertFalse(config['execute'])
        self.assertIn('vla_camera_backend_injection(context_module)', inspect.getsource(rollout_expert.execute_official))
        hashes = plan_workflow.code_review_hashes()
        for path in (Path(recovery.__file__), Path(__file__),
                     Path('/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor/backend.py'),
                     Path('/home/aup/Lerobot/projects/piper/official_adapter/lerobot_robot_piper_shared/devices.py')):
            self.assertEqual(hashes[str(path)], hashlib.sha256(path.read_bytes()).hexdigest())
        physical = json.loads((HERE/'trial/execute_config.json').read_text())
        self.assertEqual(physical['tx_ttl_s'], .12)
        self.assertEqual(physical['guard_limits']['max_observation_age_s'], .4)
        self.assertEqual(physical['guard_limits']['max_command_age_s'], .4)
        self.assertEqual(QUEUE_CONTRACT['max_slot_lateness_s'], .4)
        self.assertEqual(QUEUE_CONTRACT['n_action_steps'], 20)


if __name__ == '__main__':
    with patch.object(socket.socket, 'connect', side_effect=AssertionError('Network forbidden')):
        unittest.main(verbosity=2)
