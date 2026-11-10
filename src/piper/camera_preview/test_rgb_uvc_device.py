"""Offline, serial-bound RGB discovery and interface recovery regression checks.

Only temporary sysfs/dev trees and mocked usbfs calls are used. No real camera,
preview server, USB interface, or driver is opened or changed by these tests.
"""
from contextlib import ExitStack
import errno
import io
import os
from pathlib import Path
import stat
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import rgb_uvc_device as uvc


SERIAL = 'CC1365301EY'
OTHER_SERIAL = 'CC1365302B2'


class FakeUSBTree(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.usb_root = self.root/'sys/bus/usb/devices'
        self.video_root = self.root/'sys/class/video4linux'
        self.dev_root = self.root/'dev'
        self.by_id = self.dev_root/'v4l/by-id'
        for path in (self.usb_root, self.video_root, self.by_id):
            path.mkdir(parents=True)
        self.usb = self.add_usb('1-2', SERIAL)
        self.capture = self.add_video('video0', self.usb, index=0)
        self.metadata = self.add_video('video1', self.usb, index=1)
        self.other_usb = self.add_usb('1-3', OTHER_SERIAL, address=5)
        self.other_capture = self.add_video('video2', self.other_usb, index=0)

    def write(self, node, **values):
        for name, value in values.items():
            (node/name).write_text(str(value)+'\n')

    def add_usb(self, name, serial, address=4, vendor='2bc5', product='0557'):
        usb = self.usb_root/name
        usb.mkdir()
        self.write(usb, serial=serial, idVendor=vendor, idProduct=product,
                   busnum=1, devnum=address)
        for number, subclass in ((0, '01'), (1, '02')):
            interface = self.usb_root/(name+f':1.{number}')
            interface.mkdir()
            self.write(interface, bInterfaceClass='0e',
                       bInterfaceNumber=f'{number:02}', bInterfaceSubClass=subclass)
            driver = self.root/'drivers/uvcvideo'
            driver.mkdir(parents=True, exist_ok=True)
            (interface/'driver').symlink_to(driver)
        return usb

    def add_video(self, name, usb, index):
        video = self.video_root/name
        video.mkdir()
        self.write(video, index=index)
        # Real sysfs interface directories are descendants of their USB device.
        nested_interface = usb/(usb.name+':1.0')
        nested_interface.mkdir(exist_ok=True)
        (video/'device').symlink_to(nested_interface)
        node = self.dev_root/name
        node.touch()
        return node

    def find(self, serial=SERIAL):
        return uvc.find_video_node(serial, video_root=self.video_root,
                                   dev_root=self.dev_root, by_id=self.by_id)

    def recovery_mocks(self, *, guard=None, users=None, fstat_result=None):
        """Return mocks for every operation that could touch real USB state."""
        stack = ExitStack()
        self.addCleanup(stack.close)
        lock_fd, device_fd = 71, 72
        identity = uvc.usb_identity(self.usb)
        device = Path(f'/dev/bus/usb/{identity["bus"]:03}/{identity["address"]:03}')
        usb_stat = fstat_result or SimpleNamespace(
            st_mode=stat.S_IFCHR | 0o660, st_rdev=os.makedev(189, 3))
        original_stat = Path.stat

        def path_stat(path, *args, **kwargs):
            if path == device:
                return usb_stat
            return original_stat(path, *args, **kwargs)

        def open_only_expected(path, flags, *args):
            if Path(path) == device:
                self.assertEqual(flags, os.O_RDWR | os.O_CLOEXEC)
                return device_fd
            self.assertEqual(Path(path).name,
                             f'piper_rgb_uvc_{SERIAL}_{os.getuid()}.lock')
            self.assertTrue(flags & os.O_NOFOLLOW)
            return lock_fd

        opened = stack.enter_context(mock.patch.object(uvc.os, 'open', side_effect=open_only_expected))
        closed = stack.enter_context(mock.patch.object(uvc.os, 'close'))
        flock = stack.enter_context(mock.patch.object(uvc.fcntl, 'flock'))
        ioctl = stack.enter_context(mock.patch.object(uvc.fcntl, 'ioctl', return_value=1))
        stack.enter_context(mock.patch.object(uvc.os, 'fstat', return_value=usb_stat))
        stack.enter_context(mock.patch.object(Path, 'stat', path_stat))
        stack.enter_context(mock.patch.object(uvc, 'visible_usb_users', return_value=users or []))
        stack.enter_context(mock.patch.object(uvc, 'find_video_node', return_value=None))
        return SimpleNamespace(ioctl=ioctl, opened=opened, closed=closed, flock=flock,
                               guard=guard or mock.Mock(), device=device)


class DiscoveryTests(FakeUSBTree):
    def test_missing_udev_alias_uses_sysfs_serial_index0_only(self):
        self.assertEqual(self.find(), self.capture)
        self.assertEqual(self.find(OTHER_SERIAL), self.other_capture)
        self.capture.unlink()
        self.assertIsNone(self.find(), 'metadata index1 must never become RGB capture')

    def test_correct_alias_returned_only_after_sysfs_validation(self):
        link = self.by_id/f'Orbbec_Depth_Camera_{SERIAL}-video-index0'
        link.symlink_to(self.capture)
        self.assertEqual(self.find(), link)

    def test_alias_pointing_to_other_camera_or_metadata_is_ignored(self):
        link = self.by_id/f'Orbbec_Depth_Camera_{SERIAL}-video-index0'
        for wrong in (self.other_capture, self.metadata):
            with self.subTest(wrong=wrong):
                link.symlink_to(wrong)
                self.assertEqual(self.find(), self.capture)
                link.unlink()
        self.capture.unlink()
        link.symlink_to(self.other_capture)
        self.assertIsNone(self.find(), 'a plausible serial in a link name proves no identity')

    def test_duplicate_capture_nodes_refused(self):
        self.add_video('video3', self.usb, index=0)
        with self.assertRaisesRegex(RuntimeError, '多个RGB捕获节点'):
            self.find()

    def test_wrong_vid_or_pid_does_not_match_video_or_usb(self):
        for field, value in (('idVendor', '1234'), ('idProduct', '9999')):
            with self.subTest(field=field):
                original = (self.usb/field).read_text()
                (self.usb/field).write_text(value)
                self.assertIsNone(self.find())
                with self.assertRaisesRegex(RuntimeError, '找到0台'):
                    uvc.find_rgb_usb(SERIAL, self.usb_root)
                (self.usb/field).write_text(original)

    def test_unique_usb_identity_and_missing_or_duplicate_refused(self):
        self.assertEqual(uvc.find_rgb_usb(SERIAL, self.usb_root), self.usb)
        with self.assertRaisesRegex(RuntimeError, '找到0台'):
            uvc.find_rgb_usb('ABSENT', self.usb_root)
        self.add_usb('1-4', SERIAL, address=6)
        with self.assertRaisesRegex(RuntimeError, '找到2台'):
            uvc.find_rgb_usb(SERIAL, self.usb_root)

    def test_invalid_serial_rejected(self):
        for serial in ('', '../other', 'A B', None):
            with self.subTest(serial=serial), self.assertRaises(ValueError):
                uvc.find_rgb_usb(serial, self.usb_root)


class RecoveryTests(FakeUSBTree):
    def test_only_interface0_nested_disconnect_connect_and_connect_one_is_success(self):
        calls = self.recovery_mocks()
        uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(calls.guard.call_count, 2)
        self.assertEqual(calls.ioctl.call_count, 2)
        decoded = []
        for call in calls.ioctl.call_args_list:
            fd, outer, data, mutate = call.args
            self.assertEqual((fd, outer, mutate), (72, 0xc0105512, True))
            decoded.append(struct.unpack('<iiQ', data))
        self.assertEqual(decoded, [(0, 0x5516, 0), (0, 0x5517, 0)])
        self.assertEqual(calls.closed.call_args_list, [mock.call(72), mock.call(71)])
        # USBDEVFS_RESET (0x5514), sibling interface1, and module operations absent.
        self.assertNotIn(0x5514, [c.args[1] for c in calls.ioctl.call_args_list])

    def test_unbound_control_only_connects(self):
        (self.usb_root/'1-2:1.0/driver').unlink()
        calls = self.recovery_mocks()
        uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(calls.ioctl.call_count, 1)
        self.assertEqual(struct.unpack('<iiQ', calls.ioctl.call_args.args[2]), (0, 0x5517, 0))

    def test_disconnect_enodata_still_attempts_connect(self):
        calls = self.recovery_mocks()
        calls.ioctl.side_effect = [OSError(errno.ENODATA, 'already unbound'), 1]
        uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(calls.ioctl.call_count, 2)

    def test_guard_required_and_failures_do_not_issue_ioctl(self):
        calls = self.recovery_mocks()
        with self.assertRaisesRegex(RuntimeError, '必须先确认'):
            uvc.reprobe_rgb_control(self.usb, SERIAL, None)
        calls.opened.assert_not_called()
        for guard in (mock.Mock(side_effect=RuntimeError('preview opened')),
                      mock.Mock(side_effect=[None, RuntimeError('client started')])):
            with self.subTest(guard=guard), self.assertRaises(RuntimeError):
                uvc.reprobe_rgb_control(self.usb, SERIAL, guard)
        calls.ioctl.assert_not_called()

    def test_visible_owner_refuses_before_device_open(self):
        calls = self.recovery_mocks(users=[123])
        with self.assertRaisesRegex(RuntimeError, '仍被进程'):
            uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        calls.ioctl.assert_not_called()
        self.assertEqual(calls.opened.call_count, 1, 'only helper lock may be opened')

    def test_identity_changed_before_or_after_open_refused(self):
        calls = self.recovery_mocks()
        with self.assertRaisesRegex(RuntimeError, '身份变化'):
            uvc.reprobe_rgb_control(self.usb, OTHER_SERIAL, calls.guard)
        original_identity = uvc.usb_identity(self.usb)
        for field, value in (('serial', OTHER_SERIAL), ('address', 8), ('vendor', '1234')):
            with self.subTest(field=field), mock.patch.object(
                    uvc, 'usb_identity', side_effect=[original_identity, dict(original_identity, **{field:value})]):
                with self.assertRaisesRegex(RuntimeError, '重新枚举或身份变化'):
                    uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        calls.ioctl.assert_not_called()

    def test_invalid_usbfs_character_device_refused(self):
        for mode, device_id in ((stat.S_IFREG, os.makedev(189, 3)),
                                (stat.S_IFCHR, os.makedev(188, 3)),
                                (stat.S_IFCHR, os.makedev(189, 9))):
            with self.subTest(mode=mode, device_id=device_id):
                calls = self.recovery_mocks(fstat_result=SimpleNamespace(st_mode=mode, st_rdev=device_id))
                with self.assertRaisesRegex(RuntimeError, '重新枚举或身份变化'):
                    uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
                calls.ioctl.assert_not_called()

    def test_wrong_layout_or_non_uvc_driver_refused(self):
        for interface_name, field, value in (
                ('1-2:1.0', 'bInterfaceClass', 'ff'),
                ('1-2:1.0', 'bInterfaceNumber', '01'),
                ('1-2:1.1', 'bInterfaceSubClass', '01')):
            with self.subTest(interface=interface_name, field=field):
                interface = self.usb_root/interface_name
                original = (interface/field).read_text()
                (interface/field).write_text(value)
                calls = self.recovery_mocks()
                with self.assertRaisesRegex(RuntimeError, '接口布局'):
                    uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
                calls.ioctl.assert_not_called()
                calls.opened.assert_not_called()
                (interface/field).write_text(original)
        driver = self.root/'drivers/other'
        driver.mkdir()
        (self.usb_root/'1-2:1.1/driver').unlink()
        (self.usb_root/'1-2:1.1/driver').symlink_to(driver)
        calls = self.recovery_mocks()
        with self.assertRaisesRegex(RuntimeError, '其他驱动'):
            uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        calls.ioctl.assert_not_called()

    def test_driver_change_after_guard_refused(self):
        calls = self.recovery_mocks()
        with mock.patch.object(uvc, 'driver_name', side_effect=['uvcvideo', 'uvcvideo', 'other']):
            with self.assertRaisesRegex(RuntimeError, '占用状态变化'):
                uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        calls.ioctl.assert_not_called()

    def test_connect_failure_reports_error_and_closes_descriptors(self):
        calls = self.recovery_mocks()
        calls.ioctl.side_effect = [0, OSError(errno.EBUSY, 'busy')]
        with self.assertRaisesRegex(RuntimeError, '重新绑定失败'):
            uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(calls.closed.call_args_list, [mock.call(72), mock.call(71)])

    def test_connect_retries_unbound_control_then_succeeds_without_disconnect_repeat(self):
        calls = self.recovery_mocks()
        connect_attempts = 0
        def ioctl(fd, outer, data, mutate):
            nonlocal connect_attempts
            interface, code, pointer = struct.unpack('<iiQ', data)
            self.assertEqual((interface, pointer), (0, 0))
            if code == 0x5516:
                (self.usb_root/'1-2:1.0/driver').unlink()
                return 0
            self.assertEqual(code, 0x5517)
            connect_attempts += 1
            if connect_attempts < 3:
                raise OSError(errno.EBUSY, 'probe not ready')
            return 1
        calls.ioctl.side_effect = ioctl
        with mock.patch.object(uvc.time, 'sleep') as sleep:
            uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(connect_attempts, 3)
        self.assertEqual(calls.ioctl.call_count, 4)
        self.assertEqual(sleep.call_args_list, [mock.call(.05), mock.call(.05)])

    def test_permanent_connect_failure_is_bounded_to_three_attempts(self):
        calls = self.recovery_mocks()
        (self.usb_root/'1-2:1.0/driver').unlink()
        calls.ioctl.side_effect = OSError(errno.EBUSY, 'always busy')
        with mock.patch.object(uvc.time, 'sleep'), self.assertRaisesRegex(RuntimeError, '重新绑定失败'):
            uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(calls.ioctl.call_count, 3)
        self.assertTrue(all(struct.unpack('<iiQ', call.args[2]) == (0, 0x5517, 0)
                            for call in calls.ioctl.call_args_list))

    def test_identity_change_between_connect_attempts_stops_retry(self):
        calls = self.recovery_mocks()
        (self.usb_root/'1-2:1.0/driver').unlink()
        def fail_and_change_identity(*_):
            self.write(self.usb, serial=OTHER_SERIAL)
            raise OSError(errno.EBUSY, 'identity changed during probe')
        calls.ioctl.side_effect = fail_and_change_identity
        with mock.patch.object(uvc.time, 'sleep'), self.assertRaisesRegex(RuntimeError, '身份或驱动.*变化'):
            uvc.reprobe_rgb_control(self.usb, SERIAL, calls.guard)
        self.assertEqual(calls.ioctl.call_count, 1)


class ResolveTests(unittest.TestCase):
    def test_existing_node_never_reprobes_or_waits(self):
        node = Path('/fake/video0')
        with mock.patch.object(uvc, 'find_video_node', return_value=node), \
             mock.patch.object(uvc.os, 'access', return_value=True), \
             mock.patch.object(uvc, 'find_rgb_usb') as find_usb, \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe, \
             mock.patch.object(uvc.time, 'sleep') as sleep:
            self.assertEqual(uvc.resolve_rgb_device(SERIAL), node)
        find_usb.assert_not_called()
        reprobe.assert_not_called()
        sleep.assert_not_called()

    def test_udev_delay_is_allowed_without_reprobe(self):
        node = Path('/fake/video0')
        with mock.patch.object(uvc, 'find_video_node', side_effect=[None, None, node, node]), \
             mock.patch.object(uvc.os, 'access', return_value=True), \
             mock.patch.object(uvc, 'find_rgb_usb', return_value=Path('/fake/usb')) as find_usb, \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe, \
             mock.patch.object(uvc.time, 'sleep') as sleep:
            self.assertEqual(uvc.resolve_rgb_device(SERIAL), node)
        find_usb.assert_called_once_with(SERIAL)
        self.assertEqual(sleep.call_args_list, [mock.call(.05), mock.call(.05)])
        reprobe.assert_not_called()

    def test_recovery_waits_for_node_and_passes_guard_unchanged(self):
        node, usb, guard = Path('/fake/video0'), Path('/fake/usb'), mock.Mock()
        with mock.patch.object(uvc, 'find_video_node', side_effect=[None]*5+[node, node]), \
             mock.patch.object(uvc.os, 'access', return_value=True), \
             mock.patch.object(uvc, 'find_rgb_usb', return_value=usb), \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe, \
             mock.patch.object(uvc.time, 'sleep'), \
             mock.patch.object(uvc.time, 'monotonic', return_value=100), \
             mock.patch('sys.stdout', new=io.StringIO()):
            self.assertEqual(uvc.resolve_rgb_device(SERIAL, guard=guard), node)
        reprobe.assert_called_once_with(usb, SERIAL, guard)

    def test_post_recovery_timeout_explicit_error_and_no_repeated_probe(self):
        usb, guard = Path('/fake/usb'), mock.Mock()
        with mock.patch.object(uvc, 'find_video_node', return_value=None), \
             mock.patch.object(uvc, 'find_rgb_usb', return_value=usb), \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe, \
             mock.patch.object(uvc.time, 'sleep'), \
             mock.patch.object(uvc.time, 'monotonic', side_effect=[100, 100, 100.11]), \
             mock.patch('sys.stdout', new=io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, '恢复后仍无视频节点'):
                uvc.resolve_rgb_device(SERIAL, guard=guard, wait_s=.1)
        reprobe.assert_called_once_with(usb, SERIAL, guard)

    def test_missing_usb_refused_before_driver_changes(self):
        with mock.patch.object(uvc, 'find_video_node', return_value=None), \
             mock.patch.object(uvc, 'find_rgb_usb', side_effect=RuntimeError('no matching USB')), \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe:
            with self.assertRaisesRegex(RuntimeError, 'no matching USB'):
                uvc.resolve_rgb_device(SERIAL, guard=mock.Mock())
        reprobe.assert_not_called()

    def test_node_permissions_may_arrive_after_udev_node_without_rebinding(self):
        node = Path('/fake/video0')
        with mock.patch.object(uvc, 'find_video_node', return_value=node), \
             mock.patch.object(uvc.os, 'access', side_effect=[False, False, True]) as access, \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe, \
             mock.patch.object(uvc.time, 'monotonic', return_value=100), \
             mock.patch.object(uvc.time, 'sleep') as sleep:
            self.assertEqual(uvc.resolve_rgb_device(SERIAL), node)
        self.assertEqual(access.call_args_list, [mock.call(node, os.R_OK | os.W_OK)]*3)
        self.assertEqual(sleep.call_count, 2)
        reprobe.assert_not_called()

    def test_permissions_timeout_is_explicit_without_driver_rebinding(self):
        node = Path('/fake/video0')
        with mock.patch.object(uvc, 'find_video_node', return_value=node), \
             mock.patch.object(uvc.os, 'access', return_value=False), \
             mock.patch.object(uvc, 'reprobe_rgb_control') as reprobe, \
             mock.patch.object(uvc.time, 'monotonic', side_effect=[100, 100, 100.11]), \
             mock.patch.object(uvc.time, 'sleep'):
            with self.assertRaisesRegex(PermissionError, '权限未就绪'):
                uvc.resolve_rgb_device(SERIAL, wait_s=.1)
        reprobe.assert_not_called()


if __name__ == '__main__':
    unittest.main()
