"""Camera pairing regressions; synthetic frames only, no hardware or SSH."""
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from lerobot_robot_piper_shared import devices as d


class FakeCamera:
    """Return a planned capture time while retaining image/frame identity."""
    def __init__(self, role, plans=None, barrier=None):
        self.config = SimpleNamespace(role=role, max_age_ms=150)
        self.last_metadata = None
        self.calls = 0
        self.plans = plans or [lambda: time.monotonic() - .005]
        self.barrier = barrier

    def async_read(self, timeout_ms=200):
        self.calls += 1
        if self.barrier is not None and self.calls == 1:
            self.barrier.wait(timeout=.15)
        plan = self.plans[min(self.calls - 1, len(self.plans) - 1)]
        if isinstance(plan, BaseException):
            raise plan
        captured = plan()
        received = time.monotonic()
        self.last_metadata = {
            'index': self.calls, 'serial': self.config.role,
            'receive_monotonic': received,
            'estimated_capture_monotonic': captured,
            'http_rtt_ms': 0., 'age_ms': (received - captured) * 1000,
        }
        return np.full((2, 2, 3), self.calls, dtype=np.uint8)


class CameraPairTests(unittest.TestCase):
    def observer(self, cameras):
        observer = d.PiperSharedObserver.__new__(d.PiperSharedObserver)
        observer.connected = True
        observer.cameras = cameras
        observer._camera_executor = None
        observer._last_pair_timing = None
        observer.context = SimpleNamespace(
            config={'max_camera_age_ms': 150, 'max_camera_skew_ms': 60},
            expected_action=None, cycle=None,
        )
        def cleanup():
            executor = observer._camera_executor
            if executor is not None:
                executor.shutdown(wait=True, cancel_futures=True)
        self.addCleanup(cleanup)
        return observer

    def test_first_reads_are_concurrent(self):
        barrier = threading.Barrier(2)
        observer = self.observer({
            role: FakeCamera(role, barrier=barrier) for role in ('global', 'wrist')
        })
        images = observer._read_images()
        self.assertEqual(set(images), {'global', 'wrist'})
        self.assertEqual(observer._last_pair_timing['attempts'], 1)
        self.assertLessEqual(observer._last_pair_timing['skew_ms'], 60)
        self.assertTrue(all(age <= 150 for age in observer._last_pair_timing['ages_ms'].values()))
        self.assertGreaterEqual(observer._last_pair_timing['acquisition_ms'], 0)

    def test_skew_refreshes_only_older_camera_and_keeps_frame_identity(self):
        base = time.monotonic()
        old = FakeCamera('global', [lambda: base - .090, lambda: time.monotonic() - .002])
        recent = FakeCamera('wrist', [lambda: base - .005])
        observer = self.observer({'global': old, 'wrist': recent})
        images = observer._read_images()
        self.assertEqual((old.calls, recent.calls), (2, 1))
        self.assertEqual(observer._last_pair_timing['attempts'], 2)
        self.assertLessEqual(observer._last_pair_timing['skew_ms'], 60)
        for role, camera in observer.cameras.items():
            self.assertTrue(np.all(images[role] == camera.last_metadata['index']))

    def test_persistent_skew_fails_with_bounded_work(self):
        observer = self.observer({
            'global': FakeCamera('global', [lambda: time.monotonic() - .090]),
            'wrist': FakeCamera('wrist', [lambda: time.monotonic() - .005]),
        })
        started = time.monotonic()
        with self.assertRaises((RuntimeError, TimeoutError)):
            observer._read_images()
        self.assertLess(time.monotonic() - started, .5)
        self.assertTrue(all(camera.calls <= 4 for camera in observer.cameras.values()))

    def test_frame_aging_while_other_camera_waits_is_rejected(self):
        base = time.monotonic()
        def delayed_wrist():
            time.sleep(.060)
            return base - .080
        observer = self.observer({
            'global': FakeCamera('global', [lambda: base - .120, TimeoutError('global stalled')]),
            'wrist': FakeCamera('wrist', [delayed_wrist, TimeoutError('wrist stalled')]),
        })
        # Both first frames pass their own receive-time age checks and differ by
        # only 40 ms, but the global image is 180 ms old at pair completion.
        with self.assertRaises((RuntimeError, TimeoutError)):
            observer._read_images()
        self.assertGreater(observer.cameras['global'].calls, 1)

    def test_duplicate_frame_waits_for_strictly_new_index(self):
        camera = d.PiperHttpCamera(d.PiperHttpCameraConfig(width=2, height=2))
        camera.connected, camera.last_index = True, 7
        image = np.zeros((2, 2, 3), np.uint8)
        with patch.object(camera, '_fetch', side_effect=[(7, image), (8, image)]) as fetch, \
             patch.object(d.time, 'sleep'):
            self.assertIs(camera.async_read(), image)
        self.assertEqual(camera.last_index, 8)
        self.assertEqual(fetch.call_count, 2)

    def test_duplicate_frame_timeout_and_backward_frame_remain_errors(self):
        camera = d.PiperHttpCamera(d.PiperHttpCameraConfig(width=2, height=2))
        camera.connected, camera.last_index = True, 7
        image = np.zeros((2, 2, 3), np.uint8)
        with patch.object(camera, '_fetch', return_value=(7, image)):
            with self.assertRaises(TimeoutError):
                camera.async_read(timeout_ms=0)
        with patch.object(camera, '_fetch', return_value=(6, image)):
            with self.assertRaisesRegex(RuntimeError, '倒退'):
                camera.async_read()
        self.assertEqual(camera.last_index, 7)

    def test_pair_does_not_retry_fatal_camera_error(self):
        broken = FakeCamera('global', [RuntimeError('相机帧号倒退，请重连')])
        observer = self.observer({'global': broken, 'wrist': FakeCamera('wrist')})
        with self.assertRaisesRegex(RuntimeError, '倒退'):
            observer._read_images()
        self.assertEqual(broken.calls, 1)

    def test_can_failure_after_pair_prevents_observation(self):
        observer = self.observer({role: FakeCamera(role) for role in ('global', 'wrist')})
        observer.context.expected_action = dict.fromkeys(d.KEYS, 99.)
        observer.context.cycle = {'previous': True}
        snapshot = {'state': [0.] * 7, 'action': [1.] * 7}
        observer.context.checked_snapshot = Mock(side_effect=[snapshot, RuntimeError('CAN fault')])
        with self.assertRaisesRegex(RuntimeError, 'CAN fault'):
            observer.get_observation()
        self.assertEqual(observer.context.checked_snapshot.call_count, 2)
        self.assertIsNone(observer.context.expected_action)
        self.assertIsNone(observer.context.cycle)

    def test_observation_uses_can_snapshot_after_camera_pair(self):
        observer = self.observer({role: FakeCamera(role) for role in ('global', 'wrist')})
        before = {'state': [0.] * 7, 'action': [1.] * 7}
        after = {'state': [2.] * 7, 'action': [3.] * 7}
        observer.context.checked_snapshot = Mock(side_effect=[before, after])
        observation = observer.get_observation()
        self.assertEqual([observation[key] for key in d.KEYS], after['state'])
        self.assertEqual(list(observer.context.expected_action.values()), after['action'])
        self.assertIs(observer.context.cycle['snapshot'], after)
        self.assertEqual(observer.context.cycle['camera_pair'], observer._last_pair_timing)
        for metadata in observer.context.cycle['cameras'].values():
            self.assertLessEqual(metadata['sample_age_ms'], 150)

    def test_can_recovery_reacquires_images_that_aged_while_waiting(self):
        clock = [1000.]
        observer = self.observer({role: FakeCamera(role, [lambda: clock[0]-.090,
            lambda: clock[0]-.005]) for role in ('global', 'wrist')})
        before = {'state': [0.] * 7, 'action': [1.] * 7}
        after = {'state': [2.] * 7, 'action': [3.] * 7}
        def snapshots(**kwargs):
            if observer.context.checked_snapshot.call_count == 2:
                clock[0] += .080
                return before
            return after
        observer.context.checked_snapshot = Mock(side_effect=snapshots)
        with patch.object(d.time, 'monotonic', side_effect=lambda: clock[0]):
            observation = observer.get_observation()
        self.assertEqual(observer.context.checked_snapshot.call_count, 3)
        self.assertTrue(all(camera.calls == 2 for camera in observer.cameras.values()))
        for role, camera in observer.cameras.items():
            self.assertTrue(np.all(observation[role] == 2))
            self.assertEqual(observer.context.cycle['cameras'][role]['index'], 2)
            self.assertAlmostEqual(camera.last_metadata['sample_age_ms'], 5.)
        self.assertIs(observer.context.cycle['snapshot'], after)
        self.assertEqual(observer.context.cycle['camera_pair']['observation_attempts'], 2)

    def test_final_camera_ages_include_can_recovery_time(self):
        clock = [1000.]
        observer = self.observer({role: FakeCamera(role, [lambda: clock[0]-.005])
                                  for role in ('global', 'wrist')})
        snapshot = {'state': [0.] * 7, 'action': [1.] * 7}
        def snapshots(**kwargs):
            if observer.context.checked_snapshot.call_count == 2:
                clock[0] += .040
            return snapshot
        observer.context.checked_snapshot = Mock(side_effect=snapshots)
        with patch.object(d.time, 'monotonic', side_effect=lambda: clock[0]):
            observer.get_observation()
        for role, metadata in observer.context.cycle['cameras'].items():
            self.assertAlmostEqual(metadata['sample_age_ms'], 45.)
            self.assertAlmostEqual(observer.context.cycle['camera_pair']['ages_ms'][role], 45.)
        self.assertTrue(all(camera.calls == 1 for camera in observer.cameras.values()))

    def test_can_recovery_cannot_repeat_image_acquisition_indefinitely(self):
        clock = [1000.]
        observer = self.observer({role: FakeCamera(role, [lambda: clock[0]-.090])
                                  for role in ('global', 'wrist')})
        observer.context.expected_action = dict.fromkeys(d.KEYS, 99.)
        observer.context.cycle = {'previous': True}
        snapshot = {'state': [0.] * 7, 'action': [1.] * 7}
        def snapshots(**kwargs):
            if observer.context.checked_snapshot.call_count > 1:
                clock[0] += .080
            return snapshot
        observer.context.checked_snapshot = Mock(side_effect=snapshots)
        with patch.object(d.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, '等待CAN新反馈'):
                observer.get_observation()
        self.assertEqual(observer.context.checked_snapshot.call_count, 3)
        self.assertTrue(all(camera.calls == 2 for camera in observer.cameras.values()))
        self.assertIsNone(observer.context.cycle)
        self.assertIsNone(observer.context.expected_action)

    def test_failure_before_cameras_clears_previous_cycle_without_fetching(self):
        observer = self.observer({role: FakeCamera(role) for role in ('global', 'wrist')})
        observer.context.expected_action = dict.fromkeys(d.KEYS, 99.)
        observer.context.cycle = {'previous': True}
        observer.context.checked_snapshot = Mock(side_effect=RuntimeError('CAN fault'))
        with self.assertRaisesRegex(RuntimeError, 'CAN fault'):
            observer.get_observation()
        self.assertTrue(all(camera.calls == 0 for camera in observer.cameras.values()))
        self.assertIsNone(observer.context.cycle)
        self.assertIsNone(observer.context.expected_action)

    def test_observation_retries_preempted_fetch_within_existing_budget(self):
        clock = [1000.]
        def delayed_timeout():
            clock[0] += .213
            raise TimeoutError('相机图像未更新')
        observer = self.observer({
            'global': FakeCamera('global', [delayed_timeout, lambda: clock[0]-.005]),
            'wrist': FakeCamera('wrist', [TimeoutError('相机图像未更新'), lambda: clock[0]-.005]),
        })
        snapshot = {'state': [0.] * 7, 'action': [1.] * 7}
        observer.context.checked_snapshot = Mock(return_value=snapshot)
        with patch.object(d.time, 'monotonic', side_effect=lambda: clock[0]):
            observation = observer.get_observation()
        self.assertTrue(all(camera.calls == 2 for camera in observer.cameras.values()))
        self.assertTrue(all(np.all(observation[role] == 2) for role in observer.cameras))
        self.assertEqual(observer._last_pair_timing['attempts'], 2)
        self.assertAlmostEqual(observer._last_pair_timing['observation_ms'], 213.)
        self.assertTrue(all(age <= 150 for age in observer._last_pair_timing['ages_ms'].values()))

    def test_preempted_fetch_cannot_exceed_shared_observation_budget(self):
        clock = [1000.]
        def delayed_timeout():
            clock[0] += .401
            raise TimeoutError('相机图像未更新')
        observer = self.observer({
            'global': FakeCamera('global', [delayed_timeout]),
            'wrist': FakeCamera('wrist', [TimeoutError('相机图像未更新')]),
        })
        observer.context.checked_snapshot = Mock(return_value={'state': [0.] * 7, 'action': [1.] * 7})
        with patch.object(d.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, '有限重配对'):
                observer.get_observation()
        self.assertTrue(all(camera.calls == 1 for camera in observer.cameras.values()))
        self.assertIsNone(observer.context.cycle)
        self.assertIsNone(observer.context.expected_action)

    def test_standalone_pair_keeps_200ms_default_budget(self):
        clock = [1000.]
        def delayed_timeout():
            clock[0] += .213
            raise TimeoutError('相机图像未更新')
        observer = self.observer({
            'global': FakeCamera('global', [delayed_timeout]),
            'wrist': FakeCamera('wrist', [TimeoutError('相机图像未更新')]),
        })
        with patch.object(d.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, '有限重配对'):
                observer._read_images()
        self.assertTrue(all(camera.calls == 1 for camera in observer.cameras.values()))


if __name__ == '__main__':
    unittest.main()
