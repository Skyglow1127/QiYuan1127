"""Serial-bound DC1 V4L2 discovery and narrowly scoped driver re-probing.

Call recovery only after the selected preview services have released devices.
USBDEVFS_IOCTL reattaches the target RGB control interface; it does not reset
the camera, its depth sibling, a hub, or the system-wide uvcvideo module.
"""
import errno
import fcntl
import os
from pathlib import Path
import stat
import struct
import sys
import tempfile
import time


def usb_identity(node):
    return dict(serial=(node/'serial').read_text().strip(),
                vendor=(node/'idVendor').read_text().strip(),
                product=(node/'idProduct').read_text().strip(),
                bus=int((node/'busnum').read_text()), address=int((node/'devnum').read_text()))


def find_rgb_usb(serial, usb_root=Path('/sys/bus/usb/devices')):
    if not isinstance(serial, str) or not serial.isalnum():
        raise ValueError('RGB相机序列号必须是已验证的字母数字标识')
    found = []
    for node in usb_root.iterdir():
        try:
            identity = usb_identity(node)
            if (identity['serial'], identity['vendor'], identity['product']) == (serial, '2bc5', '0557'):
                found.append(node)
        except (OSError, ValueError):
            continue
    if len(found) != 1:
        raise RuntimeError(f'USB未识别到唯一的RGB相机 {serial}（找到{len(found)}台）；请检查该相机USB连接')
    return found[0]


def find_video_node(serial, *, video_root=Path('/sys/class/video4linux'),
                    dev_root=Path('/dev'), by_id=Path('/dev/v4l/by-id')):
    """Validate sysfs identity even when a plausible udev link is present."""
    matches = set()
    for video in video_root.glob('video*'):
        try:
            if (video/'index').read_text().strip() != '0':
                continue  # index1 is the UVC metadata node, not RGB capture.
            interface = (video/'device').resolve(strict=True)
            for ancestor in (interface, *interface.parents):
                if not (ancestor/'idVendor').exists():
                    continue
                if ((ancestor/'idVendor').read_text().strip(),
                    (ancestor/'idProduct').read_text().strip(),
                    (ancestor/'serial').read_text().strip()) == ('2bc5', '0557', serial):
                    node = dev_root/video.name
                    if node.exists():
                        matches.add(node.resolve())
                break
        except (OSError, ValueError):
            continue
    if len(matches) > 1:
        raise RuntimeError(f'序列号{serial}对应多个RGB捕获节点，拒绝猜测：{sorted(map(str,matches))}')
    if not matches:
        return None
    target = next(iter(matches))
    for link in by_id.glob(f'*_{serial}-video-index0'):
        if link.exists() and link.resolve() == target:
            return link
    return target  # udev by-id links may be absent while sysfs identity is valid.


def driver_name(interface):
    driver = interface/'driver'
    return driver.resolve().name if driver.exists() else None


def visible_usb_users(device):
    """Best-effort owner check; usbfs/kernel ownership checks still apply."""
    found = []
    for process in Path('/proc').glob('[0-9]*'):
        if process.name == str(os.getpid()):
            continue
        try:
            for fd in (process/'fd').iterdir():
                try:
                    if os.readlink(fd) == str(device):
                        found.append(int(process.name))
                        break
                except OSError:
                    continue
        except OSError:
            continue  # Other UIDs/root may be hidden; this is not proof of idle.
    return found


def reprobe_rgb_control(usb, serial, guard):
    if guard is None:
        raise RuntimeError('RGB驱动恢复必须先确认原预览已关闭')
    if struct.calcsize('P') != 8 or sys.byteorder != 'little':
        raise RuntimeError('RGB驱动恢复当前仅支持Linux 64位小端平台')
    expected = usb_identity(usb)
    if (expected['serial'], expected['vendor'], expected['product']) != (serial, '2bc5', '0557'):
        raise RuntimeError('RGB设备身份变化，未恢复驱动')
    control = usb.parent/(usb.name+':1.0')
    streaming = usb.parent/(usb.name+':1.1')
    for interface, number, subclass in ((control, '00', '01'), (streaming, '01', '02')):
        if ((interface/'bInterfaceClass').read_text().strip(),
            (interface/'bInterfaceNumber').read_text().strip(),
            (interface/'bInterfaceSubClass').read_text().strip()) != ('0e', number, subclass):
            raise RuntimeError('RGB USB接口布局与DC1不符，未更改驱动')
        if driver_name(interface) not in (None, 'uvcvideo'):
            raise RuntimeError('RGB接口被其他驱动占用，未更改驱动')
    device = Path(f'/dev/bus/usb/{expected["bus"]:03}/{expected["address"]:03}')
    # Serialize recovery attempts by this helper without changing USB permissions.
    lock_path = Path(tempfile.gettempdir())/f'piper_rgb_uvc_{serial}_{os.getuid()}.lock'
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        guard()
        users = visible_usb_users(device)
        if users:
            raise RuntimeError(f'RGB USB设备仍被进程{users}占用，未更改驱动')
        fd = os.open(device, os.O_RDWR | os.O_CLOEXEC)
        try:
            actual = os.fstat(fd)
            if (not stat.S_ISCHR(actual.st_mode)
                    or os.major(actual.st_rdev) != 189
                    or os.minor(actual.st_rdev) != (expected['bus']-1)*128+expected['address']-1
                    or actual.st_rdev != device.stat().st_rdev
                    or usb_identity(usb) != expected):
                raise RuntimeError('RGB USB设备重新枚举或身份变化，未更改驱动')
            guard()
            if driver_name(control) not in (None, 'uvcvideo') or driver_name(streaming) not in (None, 'uvcvideo'):
                raise RuntimeError('RGB接口占用状态变化，未更改驱动')
            # struct usbdevfs_ioctl { int ifno; int ioctl_code; void *data; }
            # Local linux/usbdevice_fs.h and libusb linux_usbfs.c use this ABI.
            if driver_name(control) == 'uvcvideo':
                try:
                    fcntl.ioctl(fd, 0xc0105512, bytearray(struct.pack('<iiQ', 0, 0x5516, 0)), True)
                except OSError as exc:
                    if exc.errno != errno.ENODATA:
                        raise
            for attempt in range(3):
                if (usb_identity(usb) != expected
                        or driver_name(control) not in (None, 'uvcvideo')
                        or driver_name(streaming) not in (None, 'uvcvideo')):
                    raise RuntimeError('RGB身份或驱动在恢复时变化，已停止重新绑定')
                try:
                    # CONNECT may return 1 on successful probe, not only 0.
                    fcntl.ioctl(fd, 0xc0105512, bytearray(struct.pack('<iiQ', 0, 0x5517, 0)), True)
                    break
                except OSError as exc:
                    if driver_name(control) == 'uvcvideo' and find_video_node(serial) is not None:
                        break  # An EBUSY return alone is not sufficient.
                    if attempt == 2 or driver_name(control) is not None:
                        raise RuntimeError(f'目标RGB接口重新绑定失败：{exc}；请检查该相机USB连接') from exc
                    time.sleep(.05)
        finally:
            os.close(fd)
    finally:
        os.close(lock_fd)


def resolve_rgb_device(serial, *, guard=None, wait_s=3):
    node = find_video_node(serial)
    if node is not None:
        return wait_for_access(serial, wait_s)
    usb = find_rgb_usb(serial)
    # Give normal udev enumeration a short chance before touching a driver.
    for _ in range(3):
        time.sleep(.05)
        node = find_video_node(serial)
        if node is not None:
            return wait_for_access(serial, wait_s)
    print(f'RGB相机{serial}仍在USB中，但缺少视频节点；正在恢复该RGB接口。', flush=True)
    reprobe_rgb_control(usb, serial, guard)
    deadline = time.monotonic()+wait_s
    while time.monotonic() < deadline:
        node = find_video_node(serial)
        if node is not None:
            node = wait_for_access(serial, max(.1, deadline-time.monotonic()))
            print(f'已恢复RGB视频节点：{node}', flush=True)
            return node
        time.sleep(.05)
    raise RuntimeError(f'RGB接口恢复后仍无视频节点（{serial}）；请重新插拔该相机USB后重试')


def wait_for_access(serial, wait_s):
    """A new devnode can precede udev's video-group/ACL permissions."""
    deadline = time.monotonic()+wait_s
    while True:
        node = find_video_node(serial)
        if node is not None and os.access(node, os.R_OK | os.W_OK):
            return node
        if time.monotonic() >= deadline:
            if node is not None:
                raise PermissionError(f'RGB视频节点权限未就绪：{node}；请检查当前用户的video设备访问权限')
            raise RuntimeError(f'RGB视频节点在等待权限时消失：{serial}')
        time.sleep(.05)
