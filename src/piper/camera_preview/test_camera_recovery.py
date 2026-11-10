"""Fault-injection tests: fake capture only, never opens USB or the SDK."""
import threading
import time
from types import SimpleNamespace
import unittest

from test_camera_lifecycle import FakeFactory, server


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = [10.]
        self.factory = FakeFactory()
        self.controller = server.CameraController(self.factory, clock=lambda:self.now[0])

    def tearDown(self):
        self.controller.stop_recovery()
        for camera in self.factory.cameras:
            camera.release_gate.set()
        try:
            self.controller.control('close',wait=True)
        except RuntimeError:
            pass

    def open(self):
        self.controller.control('reconnect')
        camera=self.controller.current()
        self.assertTrue(camera.entered.wait(1))
        return camera

    def fail(self, camera):
        camera.error='simulated USB disconnect'
        camera.stop.set()
        self.assertTrue(camera.finished.wait(1))

    def test_failed_capture_reopens_after_cleanup_without_overlapping_devices(self):
        old=self.open();self.fail(old)
        self.assertEqual(self.controller.status()['state'],'recovering')
        self.assertTrue(self.controller.recover_once())
        new=self.controller.current()
        self.assertTrue(new.entered.wait(1))
        self.assertIsNot(new,old)
        self.assertTrue(old.stop.is_set())
        self.assertEqual(self.factory.peak,1)
        self.assertEqual(self.controller.status()['recovery_total'],1)

    def test_explicit_close_is_never_automatically_reopened(self):
        old=self.open();self.fail(old)
        self.controller.control('close')
        count=len(self.factory.cameras)
        self.now[0]+=100
        self.assertFalse(self.controller.recover_once())
        self.assertEqual(len(self.factory.cameras),count)
        self.assertFalse(self.controller.status()['desired_open'])

    def test_repeated_disconnects_use_backoff(self):
        self.fail(self.open())
        self.assertTrue(self.controller.recover_once())
        camera=self.controller.current();self.assertTrue(camera.entered.wait(1));self.fail(camera)
        self.now[0]+=.5
        self.assertFalse(self.controller.recover_once())
        self.now[0]+=.5
        self.assertTrue(self.controller.recover_once())
        self.assertEqual(self.controller.status()['recovery_attempts'],2)
        self.assertAlmostEqual(self.controller.status()['retry_in_ms'],2000.)

    def test_failed_sdk_cleanup_blocks_new_open(self):
        camera=self.open();camera.cleanup_fails=True;self.fail(camera)
        count=len(self.factory.cameras)
        self.assertTrue(self.controller.recover_once())
        self.assertEqual(len(self.factory.cameras),count)
        self.assertIn('release failed',self.controller.status()['recovery_error'])


class FreshFrameTests(unittest.TestCase):
    def camera(self):
        return SimpleNamespace(role='wrist',condition=threading.Condition(),
            stop=threading.Event(),error=None,latest={})

    def test_stale_frame_waits_for_a_new_capture_without_refreshing_old_time(self):
        camera=self.camera()
        old_meta={'index':1,'received_mono':time.monotonic()-1}
        camera.latest['color']=(b'old',old_meta)
        def publish():
            time.sleep(.02)
            with camera.condition:
                camera.latest['color']=(b'new',{'index':2,'received_mono':time.monotonic()})
                camera.condition.notify_all()
        worker=threading.Thread(target=publish);worker.start()
        try:
            raw,meta=server.fresh_frame(camera,'color',wait_s=.2)
        finally:
            worker.join(1)
        self.assertEqual(raw,b'new')
        self.assertEqual(meta['index'],2)
        self.assertGreater(time.monotonic()-old_meta['received_mono'],1)

    def test_permanent_stall_or_manual_close_cannot_return_old_image(self):
        camera=self.camera()
        camera.latest['color']=(b'old',{'received_mono':time.monotonic()-1})
        with self.assertRaises(TimeoutError):
            server.fresh_frame(camera,'color',wait_s=0)
        camera.stop.set()
        with self.assertRaises(RuntimeError):
            server.fresh_frame(camera,'color',wait_s=0)

    def test_depth_updates_cannot_hide_color_stall(self):
        self.assertEqual(server.stale_streams({'color':10.,'depth':12.5},12.5),['color'])


if __name__=='__main__':
    unittest.main()
