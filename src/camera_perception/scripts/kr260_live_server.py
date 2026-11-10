#!/usr/bin/env python3
"""Orbbec SDK 1.10.18 live preview. Bind loopback; reach through SSH forwarding.

Only latest frames are retained. RGB and depth are independent, NOT synchronized
or spatially aligned. No robot commands or inference are issued.
"""
import argparse
import base64
import ctypes as C
import hashlib
import json
import math
import signal
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class Intrinsic(C.Structure):
    _pack_ = 1  # Orbbec SDK 1.10.18 ObTypes.h uses #pragma pack(push, 1).
    _fields_ = [(n, C.c_float) for n in ('fx', 'fy', 'cx', 'cy')] + [('width', C.c_int16), ('height', C.c_int16)]


class Distortion(C.Structure):
    _pack_ = 1
    _fields_ = [(n, C.c_float) for n in ('k1', 'k2', 'k3', 'k4', 'k5', 'k6', 'p1', 'p2')]


class Transform(C.Structure):
    _pack_ = 1
    _fields_ = [('rot', C.c_float * 9), ('trans', C.c_float * 3)]


class CameraParam(C.Structure):
    # ABI from Orbbec SDK 1.10.18 ObTypes.h; SDK files remain unchanged.
    _pack_ = 1
    _fields_ = [('depthIntrinsic', Intrinsic), ('rgbIntrinsic', Intrinsic),
                ('depthDistortion', Distortion), ('rgbDistortion', Distortion),
                ('transform', Transform), ('isMirrored', C.c_bool)]


def color_calibration(param, profile):
    k, d = param.rgbIntrinsic, param.rgbDistortion
    if (k.width, k.height) != (profile['width'], profile['height']):
        raise ValueError('Active color profile and intrinsics dimensions differ')
    distortion = [getattr(d, n) for n in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')]
    if not all(math.isfinite(x) for x in [k.fx, k.fy, k.cx, k.cy, *distortion]) or min(k.fx, k.fy) <= 0:
        raise ValueError('Invalid camera calibration')
    if param.isMirrored:
        raise ValueError('Mirrored color calibration needs explicit validation')
    return dict(source='ob_pipeline_get_camera_param', sdk='1.10.18',
                width=k.width, height=k.height,
                camera_matrix=[[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]],
                distortion=distortion, distortion_order='k1,k2,p1,p2,k3,k4,k5,k6',
                mirrored=False, independently_calibrated=False)


def resolve_role(path, role):
    data = json.loads(path.read_text())
    cameras = data['cameras']
    item = cameras[role]
    serial = item['serial']
    if (not isinstance(serial, str) or not serial.isalnum()
            or item.get('identity_verified_from_image') is not True):
        raise ValueError('Role needs a verified camera serial')
    if sum(c.get('serial') == serial for c in cameras.values()) != 1:
        raise ValueError('One serial cannot have multiple camera roles')
    return serial, item.get('label_zh', role)


def usb_serial_for_uid(uid, root=Path('/sys/bus/usb/devices')):
    # Legacy DC1 enumeration exposes an empty serial until opened. Its RGB USB
    # sibling has the real serial in sysfs; identify siblings by physical parent.
    bus_port = uid.rsplit('-', 1)[0]
    depth = root / bus_port
    if not depth.exists():
        return ''
    parent = depth.resolve().parent
    found = []
    for node in root.iterdir():
        try:
            if (node.resolve().parent == parent
                    and (node/'idVendor').read_text().strip() == '2bc5'
                    and (node/'idProduct').read_text().strip() == '0557'):
                found.append((node/'serial').read_text().strip())
        except (OSError, ValueError):
            continue
    return found[0] if len(found) == 1 else ''


def select_uid(devices, serial):
    matches = [d['uid'] for d in devices if d['serial'] == serial]
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one camera {serial}; found {devices}')
    return matches[0]


class Camera:
    def __init__(self, sdk_dir, serial=None, role=None, label=None):
        self.requested_serial = serial
        self.role = role
        self.label = label
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.latest = {}
        self.arrivals = {k: deque(maxlen=150) for k in ('color', 'depth')}
        self.counts = dict(color=0, depth=0)
        self.error = None
        self.device = {}
        self.profiles = {}
        self.calibration = None
        self.calibration_error = None
        self.sdk = C.CDLL(str(sdk_dir / 'libOrbbecSDK.so.1.10.18'))
        p, u, i = C.c_void_p, C.c_uint32, C.c_int
        self.sdk.ob_error_message.argtypes = [p]
        self.sdk.ob_error_message.restype = C.c_char_p
        self.sdk.ob_delete_error.argtypes = [p]
        self.sdk.ob_delete_error.restype = None
        specs = {
            'ob_set_logger_to_console': (None, i),
            'ob_create_pipeline': (p,), 'ob_delete_pipeline': (None, p),
            'ob_create_context': (p,), 'ob_delete_context': (None, p),
            'ob_query_device_list': (p, p), 'ob_delete_device_list': (None, p),
            'ob_device_list_device_count': (u, p),
            'ob_device_list_get_device_serial_number': (C.c_char_p, p, u),
            'ob_device_list_get_device_uid': (C.c_char_p, p, u),
            'ob_device_list_get_device_by_uid': (p, p, C.c_char_p),
            'ob_create_pipeline_with_device': (p, p),
            'ob_pipeline_get_device': (p, p), 'ob_device_get_device_info': (p, p),
            'ob_device_info_name': (C.c_char_p, p),
            'ob_device_info_serial_number': (C.c_char_p, p),
            'ob_delete_device_info': (None, p), 'ob_delete_device': (None, p),
            'ob_pipeline_get_stream_profile_list': (p, p, i),
            'ob_stream_profile_list_count': (u, p),
            'ob_stream_profile_list_get_profile': (p, p, i),
            'ob_stream_profile_format': (i, p),
            'ob_video_stream_profile_width': (u, p),
            'ob_video_stream_profile_height': (u, p),
            'ob_video_stream_profile_fps': (u, p),
            'ob_delete_stream_profile': (None, p),
            'ob_delete_stream_profile_list': (None, p),
            'ob_create_config': (p,), 'ob_delete_config': (None, p),
            'ob_config_enable_stream': (None, p, p),
            'ob_config_set_frame_aggregate_output_mode': (None, p, i),
            'ob_pipeline_start_with_config': (None, p, p),
            'ob_pipeline_stop': (None, p),
            'ob_pipeline_wait_for_frameset': (p, p, u),
            'ob_frameset_color_frame': (p, p), 'ob_frameset_depth_frame': (p, p),
            'ob_frame_data': (p, p), 'ob_frame_data_size': (u, p),
            'ob_frame_index': (C.c_uint64, p),
            'ob_video_frame_width': (u, p), 'ob_video_frame_height': (u, p),
            'ob_depth_frame_get_value_scale': (C.c_float, p),
            'ob_delete_frame': (None, p),
            'ob_pipeline_get_camera_param': (CameraParam, p),
        }
        self.api = {name: self.bind(name, *spec) for name, spec in specs.items()}

    def bind(self, name, result, *args):
        fn = getattr(self.sdk, name)
        fn.restype, fn.argtypes = result, [*args, C.POINTER(C.c_void_p)]
        def call(*values):
            error = C.c_void_p()
            value = fn(*values, C.byref(error))
            if error:
                message = self.sdk.ob_error_message(error).decode(errors='replace')
                self.sdk.ob_delete_error(error)
                raise RuntimeError(name + ': ' + message)
            return value
        return call

    def capture(self):
        f = self.api
        pipeline = config = context = device_list = device = None
        profiles, lists = [], []
        started = False
        try:
            f['ob_set_logger_to_console'](3)
            if not self.requested_serial:
                raise ValueError('Explicit verified camera serial required; no default-device fallback')
            context = f['ob_create_context']()
            device_list = f['ob_query_device_list'](context)
            devices = []
            for index in range(f['ob_device_list_device_count'](device_list)):
                uid = f['ob_device_list_get_device_uid'](device_list, index).decode()
                serial = f['ob_device_list_get_device_serial_number'](device_list, index).decode()
                devices.append(dict(uid=uid, serial=serial or usb_serial_for_uid(uid)))
            uid = select_uid(devices, self.requested_serial)
            device = f['ob_device_list_get_device_by_uid'](device_list, uid.encode())
            info = f['ob_device_get_device_info'](device)
            try:
                self.device = {k: f[fn](info).decode(errors='replace') for k, fn in (
                    ('name', 'ob_device_info_name'), ('serial', 'ob_device_info_serial_number'))}
            finally:
                f['ob_delete_device_info'](info)
            if self.device['serial'] != self.requested_serial:
                raise ValueError('Opened camera serial differs from requested role; refusing stream')
            self.device.update(role=self.role, label_zh=self.label, identity_verified=True)
            pipeline = f['ob_create_pipeline_with_device'](device)
            config = f['ob_create_config']()
            for name, sensor, fmt, width, height in (
                    ('color', 2, 5, 640, 480), ('depth', 3, 11, 640, 400)):
                pl = f['ob_pipeline_get_stream_profile_list'](pipeline, sensor)
                lists.append(pl)
                chosen = None
                for index in range(f['ob_stream_profile_list_count'](pl)):
                    profile = f['ob_stream_profile_list_get_profile'](pl, index)
                    profiles.append(profile)
                    spec = {k: f[fn](profile) for k, fn in (
                        ('width', 'ob_video_stream_profile_width'),
                        ('height', 'ob_video_stream_profile_height'),
                        ('fps', 'ob_video_stream_profile_fps'), ('format', 'ob_stream_profile_format'))}
                    if spec == dict(width=width, height=height, fps=30, format=fmt):
                        chosen = profile
                        self.profiles[name] = spec
                if chosen is None:
                    raise RuntimeError('Required DC1 profile unavailable: ' + name)
                f['ob_config_enable_stream'](config, chosen)
            # ANY output: never wait for an exposure-synchronized pair on this DC1.
            f['ob_config_set_frame_aggregate_output_mode'](config, 0)
            f['ob_pipeline_start_with_config'](pipeline, config)
            started = True
            try:
                self.calibration = color_calibration(f['ob_pipeline_get_camera_param'](pipeline), self.profiles['color'])
            except Exception as error:
                # A calibration problem must not take the working preview down.
                self.calibration_error = str(error)
            print(json.dumps(dict(event='camera_started', device=self.device,
                                  profiles=self.profiles)), flush=True)
            last_frame = time.monotonic()
            while not self.stop.is_set():
                frames = f['ob_pipeline_wait_for_frameset'](pipeline, 1000)
                if not frames:
                    if time.monotonic() - last_frame > 5:
                        raise RuntimeError('Camera has delivered no frames for 5 seconds')
                    continue
                try:
                    for name in ('color', 'depth'):
                        frame = f['ob_frameset_' + name + '_frame'](frames)
                        if not frame:
                            continue
                        try:
                            index = f['ob_frame_index'](frame)
                            if name in self.latest and self.latest[name][1]['index'] == index:
                                continue
                            raw = C.string_at(f['ob_frame_data'](frame), f['ob_frame_data_size'](frame))
                            meta = dict(index=index, received_mono=time.monotonic(),
                                        width=f['ob_video_frame_width'](frame),
                                        height=f['ob_video_frame_height'](frame))
                            if name == 'depth':
                                meta['mm_per_unit'] = f['ob_depth_frame_get_value_scale'](frame)
                                if len(raw) != meta['width'] * meta['height'] * 2:
                                    raise RuntimeError('Unexpected depth layout')
                            elif not raw.startswith(b'\xff\xd8'):
                                raise RuntimeError('Expected MJPEG color frame')
                            with self.condition:
                                self.latest[name] = (raw, meta)
                                self.arrivals[name].append(meta['received_mono'])
                                self.counts[name] += 1
                                last_frame = meta['received_mono']
                                self.condition.notify_all()
                        finally:
                            f['ob_delete_frame'](frame)
                finally:
                    f['ob_delete_frame'](frames)
        except Exception as error:
            self.error = str(error)
            print(json.dumps(dict(event='camera_error', error=self.error)), flush=True)
        finally:
            self.stop.set()
            with self.condition:
                self.condition.notify_all()
            if started:
                f['ob_pipeline_stop'](pipeline)
            for profile in profiles:
                f['ob_delete_stream_profile'](profile)
            for pl in lists:
                f['ob_delete_stream_profile_list'](pl)
            if config:
                f['ob_delete_config'](config)
            if pipeline:
                f['ob_delete_pipeline'](pipeline)
            if device:
                f['ob_delete_device'](device)
            if device_list:
                f['ob_delete_device_list'](device_list)
            if context:
                f['ob_delete_context'](context)

    def status(self):
        with self.condition:
            streams = {}
            for name, arrivals in self.arrivals.items():
                delta = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0
                streams[name] = dict(frames=self.counts[name],
                    capture_fps=(len(arrivals) - 1) / delta if delta else 0,
                    age_ms=1000 * (time.monotonic() - arrivals[-1]) if arrivals else None)
            return dict(service='kr260-live-v1', requested_serial=self.requested_serial, role=self.role, label_zh=self.label, device=self.device, profiles=self.profiles,
                        streams=streams, error=self.error, stopped=self.stop.is_set(),
                        spatially_aligned=False, exposure_synchronized=False,
                        color_calibration_ready=self.calibration is not None,
                        color_calibration_error=self.calibration_error)

    def snapshot(self):
        if getattr(self, 'role', None) == 'global':
            raise ValueError('Global observer cannot supply wrist hand-eye samples')
        return self.color_snapshot(schema='kr260-color-handeye-v1')

    def color_snapshot(self, schema='kr260-color-reference-v1'):
        """Atomic image/intrinsics evidence for either role; no robot pose implied."""
        with self.condition:
            item = self.latest.get('color')
            if self.stop.is_set() or self.error or item is None:
                raise ValueError('No healthy color frame')
            raw, meta = item
            age = time.monotonic() - meta['received_mono']
            if age < 0 or age > .25:
                raise ValueError('Color frame stale >250 ms')
            if self.calibration is None:
                raise ValueError(self.calibration_error or 'No color intrinsics')
            return dict(schema=schema, device=self.device,
                        calibration=self.calibration, frame=meta, board_frame_age_ms=age*1000,
                        image_base64=base64.b64encode(raw).decode('ascii'),
                        image_sha256=hashlib.sha256(raw).hexdigest(),
                        spatially_aligned=False, exposure_synchronized=False,
                        note='RGB-only frame; no depth, D2C or FPGA processing assertion.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-dir', type=Path, default=Path(__file__).resolve().parent / 'lib')
    parser.add_argument('--page', type=Path, default=Path(__file__).with_name('kr260_live.html'))
    parser.add_argument('--port', type=int, default=18760)
    parser.add_argument('--role', choices=['wrist', 'global'], default='wrist')
    parser.add_argument('--roles-file', type=Path, default=Path(__file__).with_name('camera_roles.json'))
    args = parser.parse_args()
    try:
        serial, label = resolve_role(args.roles_file, args.role)
    except (OSError, KeyError, ValueError) as error:
        parser.error(str(error))
    page = args.page.read_bytes()
    camera = Camera(args.sdk_dir.resolve(), serial=serial, role=args.role, label=label)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def setup(self):
            super().setup()
            self.connection.settimeout(3)
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

        def reply(self, data, content_type, headers=None, code=200):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in (headers or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                path = self.path.split('?', 1)[0]
                if path == '/':
                    self.reply(page, 'text/html; charset=utf-8')
                elif path == '/status':
                    self.reply(json.dumps(camera.status()).encode(), 'application/json')
                elif path == '/handeye-snapshot':
                    try:
                        snapshot = camera.snapshot()
                    except ValueError as error:
                        self.reply(json.dumps(dict(error=str(error))).encode(), 'application/json', code=503)
                    else:
                        self.reply(json.dumps(snapshot, allow_nan=False).encode(), 'application/json')
                elif path == '/reference-snapshot':
                    try:
                        snapshot = camera.color_snapshot()
                    except ValueError as error:
                        self.reply(json.dumps(dict(error=str(error))).encode(), 'application/json', code=503)
                    else:
                        self.reply(json.dumps(snapshot, allow_nan=False).encode(), 'application/json')
                elif path in ('/color.jpg', '/depth.raw'):
                    name = 'color' if path == '/color.jpg' else 'depth'
                    with camera.condition:
                        item = camera.latest.get(name)
                    if not item or camera.stop.is_set() or time.monotonic() - item[1]['received_mono'] > 1:
                        self.reply(b'No fresh frame', 'text/plain', code=503)
                        return
                    raw, meta = item
                    headers = {'X-Frame-Index': meta['index'], 'X-Width': meta['width'],
                               'X-Height': meta['height'], 'X-Scale': meta.get('mm_per_unit', 1),
                               'X-Board-Age-Ms': 1000 * (time.monotonic() - meta['received_mono'])}
                    self.reply(raw, 'image/jpeg' if name == 'color' else 'application/octet-stream', headers)
                elif path == '/color.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    last = -1
                    while not camera.stop.is_set():
                        with camera.condition:
                            camera.condition.wait_for(lambda: camera.stop.is_set() or
                                ('color' in camera.latest and camera.latest['color'][1]['index'] != last), 2)
                            item = camera.latest.get('color')
                        if not item or camera.stop.is_set() or time.monotonic() - item[1]['received_mono'] > 1:
                            break
                        raw, meta = item
                        if meta['index'] == last:
                            continue
                        last = meta['index']
                        header = ('--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n'
                                  'X-Frame-Index: %d\r\nX-Board-Age-Ms: %.3f\r\n\r\n') % (
                                      len(raw), last, 1000 * (time.monotonic() - meta['received_mono']))
                        self.wfile.write(header.encode() + raw + b'\r\n')
                        self.wfile.flush()
                else:
                    self.reply(b'Not found', 'text/plain', code=404)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    worker = threading.Thread(target=camera.capture, daemon=True)
    worker.start()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop.set()
        server.server_close()
        worker.join(timeout=5)


if __name__ == '__main__':
    main()
