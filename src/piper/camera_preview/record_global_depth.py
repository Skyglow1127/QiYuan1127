#!/usr/bin/env python3
"""Global DC1 depth -> lossless uint16 FFV1/MKV + colored viewing MP4.

Standalone camera footage, not a LeRobot demonstration recorder. The existing
serial-bound camera services and robot/ACT configuration remain independent.
"""
import argparse
from collections import deque
import csv
import ctypes as C
from datetime import datetime
from fractions import Fraction
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from zoneinfo import ZoneInfo

import av
import cv2
import numpy as np

from camera_server import Camera, resolve_role, select_uid, usb_serial_for_uid
from record_wrist_video import LiveState, robot_camera_clients

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FORMATS = {8: 'Y16', 11: 'Y11', 12: 'Y12'}  # Official SDK 1.10.37 ObTypes.h.
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def choose_profile(profiles, mode='fps', width=None, height=None, fps=None):
    # Y11/Y12 are hardware profiles; SDK unpacks the received depth to Y16.
    choices = [dict(p) for p in profiles if p['format_code'] in FORMATS]
    if not choices:
        raise RuntimeError('没有可保存原始深度的 Y11/Y12/Y16 档位')
    if any(v is not None for v in (width, height, fps)):
        if not all(v is not None for v in (width, height, fps)):
            raise ValueError('--width、--height、--fps 必须一起指定')
        choices = [p for p in choices if (p['width'], p['height']) == (width, height)
                   and abs(p['fps']-fps) < 1e-6]
        if not choices:
            raise ValueError('指定深度档位不存在，请先运行 --list-formats')
    if mode not in ('fps', 'resolution'):
        raise ValueError('未知档位优先级')
    def key(p):
        pixels = p['width']*p['height']
        # Prefer known working native Y11 when equivalent profiles are exposed.
        rank = {11: 2, 8: 1, 12: 0}[p['format_code']]
        return (p['fps'], pixels, rank) if mode == 'fps' else (pixels, p['fps'], rank)
    return max(choices, key=key)


class GlobalPreviewLease:
    """Close only the verified global service, only with explicit CLI takeover."""
    def __init__(self, serial, takeover, camera):
        self.serial, self.takeover, self.camera = serial, takeover, camera
        self.restore = False
        self.restore_error = None

    def request(self, path, action=False):
        request = urllib.request.Request('http://127.0.0.1:18761'+path,
                  data=b'' if action else None,
                  headers={'X-Camera-Control': '1'} if action else {})
        with OPENER.open(request, timeout=15 if action else 2) as response:
            return json.load(response)

    def __enter__(self):
        try:
            status = self.request('/status')
        except urllib.error.HTTPError as exc:
            raise RuntimeError('全局相机预览服务状态读取失败') from exc
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, ConnectionRefusedError):
                return self
            raise RuntimeError(f'无法确认全局相机预览状态：{exc}') from exc
        if (status.get('role') != 'global' or status.get('requested_serial') != self.serial
                or status.get('device', {}).get('serial', self.serial) != self.serial):
            raise RuntimeError('18761 端口的全局相机身份不符，未接管')
        if status.get('desired_open') or not status.get('stopped', True):
            if not self.takeover:
                raise RuntimeError('全局预览占用相机；先结束 ACT/示范录制，再加 --takeover-preview 临时接管')
            if robot_camera_clients():
                raise RuntimeError('ACT/示范录制正在使用相机，未中断预览')
            self.restore = bool(status.get('desired_open'))
            try:
                print('临时暂停全局相机预览，退出时恢复原先开启状态。', flush=True)
                self.request('/camera/close', True)  # Never use the bulk/wrist endpoints.
                status = self.request('/status')
                if not status.get('resources_released') or not status.get('stopped'):
                    raise RuntimeError('全局预览还未释放 SDK 设备')
            except BaseException:
                self.__exit__(None, None, None)
                raise
        elif not status.get('resources_released', False):
            raise RuntimeError('全局相机还在释放设备，请稍后重试')
        return self

    def __exit__(self, *_):
        if not self.restore:
            return
        self.restore = False
        try:
            if not self.camera.resources_released:
                raise RuntimeError('本次 SDK 资源清理失败，未自动重开全局相机')
            self.request('/camera/reconnect', True)
            print('已请求恢复全局相机预览。', flush=True)
        except Exception as exc:
            self.restore_error = str(exc)
            print(f'全局预览恢复失败：{exc}；请检查原预览服务。', file=sys.stderr)


class SDKDepthCamera:
    """One verified device, one depth stream; all SDK handles owned here."""
    def __init__(self, serial, sdk_dir=HERE/'lib'):
        self.serial, self.sdk_dir = serial, sdk_dir
        self.api = None
        self.handles, self.profile_handles, self.list_handles = {}, {}, []
        self.profiles, self.device_info = [], {}
        self.started, self.resources_released = False, True
        self.cleanup_errors = []

    def __enter__(self):
        binding = Camera(self.sdk_dir, serial=self.serial, role='global')
        self.api = dict(binding.api)
        for name, result, arguments in (
                ('ob_frame_time_stamp_us', C.c_uint64, (C.c_void_p,)),
                ('ob_frame_system_time_stamp_us', C.c_uint64, (C.c_void_p,)),
                ('ob_frame_format', C.c_int, (C.c_void_p,)),
                ('ob_config_disable_all_stream', None, (C.c_void_p,)),
                ('ob_config_set_align_mode', None, (C.c_void_p, C.c_int))):
            self.api[name] = binding.bind(name, result, *arguments)
        f, h = self.api, self.handles
        self.resources_released = False
        try:
            f['ob_set_logger_to_console'](3)
            h['context'] = f['ob_create_context']()
            h['device_list'] = f['ob_query_device_list'](h['context'])
            devices = []
            for index in range(f['ob_device_list_device_count'](h['device_list'])):
                uid = f['ob_device_list_get_device_uid'](h['device_list'], index).decode()
                serial = f['ob_device_list_get_device_serial_number'](h['device_list'], index).decode()
                devices.append(dict(uid=uid, serial=serial or usb_serial_for_uid(uid)))
            uid = select_uid(devices, self.serial)
            h['device'] = f['ob_device_list_get_device_by_uid'](h['device_list'], uid.encode())
            info = f['ob_device_get_device_info'](h['device'])
            try:
                self.device_info = dict(name=f['ob_device_info_name'](info).decode(),
                                        serial=f['ob_device_info_serial_number'](info).decode(), uid=uid)
            finally:
                self.release('ob_delete_device_info', info)
            if self.device_info['serial'] != self.serial:
                raise RuntimeError('打开设备后的序列号与全局角色绑定不同')
            h['pipeline'] = f['ob_create_pipeline_with_device'](h['device'])
            profile_list = f['ob_pipeline_get_stream_profile_list'](h['pipeline'], 3)  # DEPTH
            self.list_handles.append(profile_list)
            for index in range(f['ob_stream_profile_list_count'](profile_list)):
                profile = f['ob_stream_profile_list_get_profile'](profile_list, index)
                self.profile_handles[index] = profile
                fmt = f['ob_stream_profile_format'](profile)
                self.profiles.append(dict(index=index, width=f['ob_video_stream_profile_width'](profile),
                    height=f['ob_video_stream_profile_height'](profile),
                    fps=f['ob_video_stream_profile_fps'](profile), format_code=fmt,
                    format=FORMATS.get(fmt, f'OB_FORMAT_{fmt}')))
            return self
        except BaseException:
            self.close()
            raise

    def start(self, profile):
        f, h = self.api, self.handles
        h['config'] = f['ob_create_config']()
        f['ob_config_disable_all_stream'](h['config'])
        f['ob_config_enable_stream'](h['config'], self.profile_handles[profile['index']])
        f['ob_config_set_align_mode'](h['config'], 0)  # ALIGN_DISABLE, no D2C
        f['ob_config_set_frame_aggregate_output_mode'](h['config'], 2)  # ANY_SITUATION
        f['ob_pipeline_start_with_config'](h['pipeline'], h['config'])
        self.started = True

    def read(self, stop):
        f, pipeline = self.api, self.handles['pipeline']
        deadline = time.monotonic()+3
        while not stop.is_set():
            if time.monotonic() >= deadline:
                raise TimeoutError('全局深度图连续 3 秒没有更新，已结束录制')
            frames = f['ob_pipeline_wait_for_frameset'](pipeline, 100)
            if not frames:
                continue
            try:
                frame = f['ob_frameset_depth_frame'](frames)
                if not frame:
                    continue
                try:
                    width = f['ob_video_frame_width'](frame)
                    height = f['ob_video_frame_height'](frame)
                    fmt = f['ob_frame_format'](frame)
                    length = f['ob_frame_data_size'](frame)
                    if fmt != 8 or length != width*height*2:
                        raise RuntimeError(f'预期SDK解包Y16深度，收到格式{fmt}、{length}字节')
                    return dict(raw=C.string_at(f['ob_frame_data'](frame), length),
                        width=width, height=height, sequence=f['ob_frame_index'](frame),
                        device_timestamp_us=f['ob_frame_time_stamp_us'](frame),
                        sdk_host_received_us=f['ob_frame_system_time_stamp_us'](frame),
                        received_monotonic_ns=time.monotonic_ns(),
                        mm_per_unit=f['ob_depth_frame_get_value_scale'](frame))
                finally:
                    self.release('ob_delete_frame', frame)
            finally:
                self.release('ob_delete_frame', frames)
        return None

    def release(self, name, handle):
        try:
            self.api[name](handle)
        except Exception as exc:
            # Preserve even transient frame/info cleanup failures: a later
            # successful pipeline deletion must not permit automatic reopening.
            self.cleanup_errors.append(f'{name}: {exc}')
            self.resources_released = False
            raise

    def close(self):
        if self.api is None:
            return
        operations = [('ob_pipeline_stop', self.handles['pipeline'])] if self.started else []
        operations += [('ob_delete_stream_profile', p) for p in self.profile_handles.values()]
        operations += [('ob_delete_stream_profile_list', p) for p in self.list_handles]
        operations += [('ob_delete_'+name, self.handles[name]) for name in (
            'config', 'pipeline', 'device', 'device_list', 'context') if name in self.handles]
        for name, handle in operations:
            try:
                self.release(name, handle)
            except Exception:
                pass
        self.started = False
        self.handles, self.profile_handles, self.list_handles = {}, {}, []
        self.resources_released = not self.cleanup_errors
        if self.cleanup_errors:
            print('SDK资源清理失败：'+'; '.join(self.cleanup_errors), file=sys.stderr)

    def __exit__(self, *_):
        self.close()


def colorize_depth(depth, scale, near_mm, far_mm):
    """Visualization only; never modify values passed to the depth encoder."""
    mm = depth.astype(np.float32)*scale
    valid = depth != 0
    normalized = np.clip((mm-near_mm)/(far_mm-near_mm), 0, 1)
    colors = cv2.applyColorMap(((1-normalized)*255).astype(np.uint8), cv2.COLORMAP_TURBO)
    colors[~valid] = 0
    values = mm[valid]
    return colors, dict(valid_fraction=float(valid.mean()),
        depth_min_mm=float(values.min()) if values.size else None,
        depth_max_mm=float(values.max()) if values.size else None)


class DepthWriter:
    def __init__(self, directory, profile, color_video=True):
        self.directory, self.profile, self.color_video = directory, profile, color_video
        self.files, self.containers, self.streams = {}, {}, {}
        self.closed = False

    def __enter__(self):
        try:
            for name, filename, fmt, codec, pixel_format in (
                    ('depth', 'depth.mkv', 'matroska', 'ffv1', 'gray16le'),
                    ('preview', 'depth_preview.mp4', 'mp4', 'libx264', 'yuv420p')):
                if name == 'preview' and not self.color_video:
                    continue
                file = (self.directory/filename).open('xb')
                self.files[name] = file
                container = av.open(file, 'w', format=fmt)
                self.containers[name] = container
                stream = container.add_stream(codec, rate=Fraction(str(self.profile['fps'])))
                stream.width, stream.height = self.profile['width'], self.profile['height']
                stream.pix_fmt = pixel_format
                stream.time_base = Fraction(1, 1000000)
                stream.codec_context.time_base = Fraction(1, 1000000)
                stream.codec_context.thread_count = 2
                if name == 'preview':
                    stream.options = {'preset': 'ultrafast', 'crf': '18', 'tune': 'zerolatency'}
                else:
                    stream.options = {'level': '3', 'slicecrc': '1'}
                self.streams[name] = stream
            return self
        except BaseException:
            self.close()
            raise

    def write(self, depth, colors, relative_us):
        self.write_depth(depth, relative_us)
        self.write_preview(colors, relative_us)

    def _write_frame(self, name, array, pixel_format, relative_us):
        frame = av.VideoFrame.from_ndarray(array, format=pixel_format)
        frame.pts, frame.time_base = relative_us, Fraction(1, 1000000)
        for packet in self.streams[name].encode(frame):
            self.containers[name].mux(packet)

    def write_depth(self, depth, relative_us):
        self._write_frame('depth', depth, 'gray16le', relative_us)

    def write_preview(self, colors, relative_us):
        if 'preview' in self.streams:
            self._write_frame('preview', colors, 'bgr24', relative_us)

    def size(self):
        return sum(file.tell() for file in self.files.values() if not file.closed)

    def close(self):
        if self.closed:
            return
        self.closed = True
        errors = []
        for name, container in self.containers.items():
            try:
                for packet in self.streams[name].encode(None):
                    container.mux(packet)
            except Exception as exc:
                errors.append(f'{name}写尾：{exc}')
            try:
                container.close()
            except Exception as exc:
                errors.append(f'{name}关闭：{exc}')
        for file in self.files.values():
            try:
                file.close()
            except OSError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))

    def __exit__(self, *_):
        self.close()


def record_depth(state, camera, directory, duration=0, color_video=True):
    first_ts = last_ts = last_seq = None
    frames = missing = color_frames = 0
    digest = hashlib.sha256()
    recent = deque(maxlen=90)
    started, last_print = time.monotonic(), 0.
    p = state.data['profile']
    with (directory/'timestamps.csv').open('x', newline='') as csv_file:
        times = csv.writer(csv_file)
        times.writerow(['frame', 'sdk_frame_index', 'device_timestamp_us', 'sdk_host_received_us',
                        'received_monotonic_ns', 'mm_per_unit', 'raw_bytes'])
        try:
            with DepthWriter(directory, p, color_video) as writer:
                while not state.stop.is_set():
                    if duration and time.monotonic()-started >= duration:
                        break
                    item = camera.read(state.stop)
                    if item is None:
                        break
                    ts, seq, scale = item['device_timestamp_us'], item['sequence'], item['mm_per_unit']
                    if ts < 0 or (last_ts is not None and ts <= last_ts):
                        raise RuntimeError('设备深度时间戳重复或倒退，已保留此前录像')
                    if seq < 0 or (last_seq is not None and seq <= last_seq):
                        raise RuntimeError('SDK深度帧序号重复或倒退，已保留此前录像')
                    if not math.isfinite(scale) or scale <= 0:
                        raise RuntimeError('深度比例系数无效，拒绝保存错误距离单位')
                    if ((item['width'], item['height']) != (p['width'], p['height'])
                            or len(item['raw']) != p['width']*p['height']*2):
                        raise RuntimeError('实际深度尺寸/字节布局不符，拒绝静默降低分辨率')
                    depth = np.frombuffer(item['raw'], dtype='<u2').reshape(p['height'], p['width'])
                    colors, stats = colorize_depth(depth, scale, state.data['preview_near_mm'], state.data['preview_far_mm'])
                    if first_ts is None:
                        first_ts = ts
                    writer.write_depth(depth, ts-first_ts)
                    digest.update(item['raw'])
                    frames += 1
                    missing += max(0, seq-last_seq-1) if last_seq is not None else 0
                    last_ts, last_seq = ts, seq
                    times.writerow([frames, seq, ts, item['sdk_host_received_us'],
                                    item['received_monotonic_ns'], scale, len(item['raw'])])
                    elapsed = (ts-first_ts)/1000000
                    recent.append(ts)
                    fps = (frames-1)/elapsed if elapsed else 0.
                    recent_fps = ((len(recent)-1)*1000000/(recent[-1]-recent[0]) if len(recent)>1 else 0.)
                    # Commit raw-depth metadata before optional viewing/preview
                    # encoders: their failure must not hide a saved depth frame.
                    state.update(state='recording', frames=frames, elapsed_s=elapsed,
                        measured_fps=fps, recent_fps=recent_fps,
                        estimated_missing_frames=missing, mm_per_unit=scale, **stats)
                    writer.write_preview(colors, ts-first_ts)
                    if color_video:
                        color_frames += 1
                    state.update(color_video_frames=color_frames)
                    # Limit JPEG work to 15 fps; all depth frames and MP4 frames are saved.
                    if frames == 1 or time.monotonic()-state.data.get('preview_updated_mono', 0) >= 1/15:
                        ok, jpeg = cv2.imencode('.jpg', colors, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if not ok:
                            raise RuntimeError('深度伪彩色预览编码失败')
                        with state.condition:
                            state.latest, state.latest_number = jpeg.tobytes(), frames
                            state.data['preview_updated_mono'] = time.monotonic()
                    state.update(bytes_written=writer.size())
                    if time.monotonic()-last_print >= 1:
                        print(f'\r深度录制 {elapsed:7.1f}s | {fps:5.1f} fps | {frames} 帧 | '
                              f'{writer.size()/1048576:.1f} MiB | SDK缺帧 {missing}   ', end='', flush=True)
                        last_print = time.monotonic()
                if not frames:
                    raise RuntimeError('录制结束前没有收到深度图')
        finally:
            state.update(raw_depth_sha256=digest.hexdigest(), wall_duration_s=time.monotonic()-started,
                         bytes_written=sum(f.stat().st_size for f in directory.glob('*') if f.is_file()))
            print(flush=True)


def make_preview_server(state, port):
    page = (HERE/'global_depth_recorder.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, data, kind='application/json', code=200):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                if self.path == '/':
                    self.reply(page, 'text/html; charset=utf-8')
                elif self.path == '/status':
                    self.reply(json.dumps(state.status(), ensure_ascii=False).encode())
                elif self.path == '/stream.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.connection.settimeout(2)
                    last = 0
                    while True:
                        with state.condition:
                            state.condition.wait_for(lambda: state.latest_number != last or state.stop.is_set(), timeout=1)
                            if state.latest_number == last:
                                if state.stop.is_set():
                                    break
                                continue
                            raw, last = state.latest, state.latest_number
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                            str(len(raw)).encode()+b'\r\n\r\n'+raw+b'\r\n')
                        self.wfile.flush()
                else:
                    self.reply(b'Not found', 'text/plain', 404)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def do_POST(self):
            try:
                if self.path != '/stop':
                    self.reply(b'{}', code=404)
                elif self.headers.get('X-Recorder-Control') != '1':
                    self.reply(b'{"error":"control header required"}', code=403)
                else:
                    if state.status()['state'] in ('starting', 'recording'):
                        state.stop.set()
                        state.update(state='stopping')
                    self.reply(b'{"ok":true}')
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .1}, daemon=True).start()
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description='全局深度图录制：16位无损MKV + 伪彩色MP4 + 实时网页预览。')
    parser.add_argument('--mode', choices=['fps', 'resolution'], default='fps', help='默认优先帧率；resolution优先像素数')
    parser.add_argument('--list-formats', action='store_true', help='只枚举SDK深度档位，不启动采集')
    parser.add_argument('--width', type=int)
    parser.add_argument('--height', type=int)
    parser.add_argument('--fps', type=float)
    parser.add_argument('--duration', type=float, default=0, help='秒；默认0持续至Ctrl+C/网页停止')
    parser.add_argument('--output-dir', type=Path, help='新建录制目录；已有目录拒绝覆盖')
    parser.add_argument('--port', type=int, default=18764, help='本机预览端口，默认18764')
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--no-color-video', action='store_true', help='只保存原始深度MKV，仍提供伪彩色实时预览')
    parser.add_argument('--near-mm', type=float, default=200, help='预览近端，单位毫米；不改变原始数据')
    parser.add_argument('--far-mm', type=float, default=2000, help='预览远端，单位毫米；不改变原始数据')
    parser.add_argument('--takeover-preview', action='store_true', help='无ACT/示范录制时临时接管全局预览，退出恢复')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration 必须是有限非负数')
    if (not math.isfinite(args.near_mm) or not math.isfinite(args.far_mm)
            or not 0 <= args.near_mm < args.far_mm):
        parser.error('预览范围必须满足 0 <= near-mm < far-mm，且为有限数')
    if not 0 <= args.port <= 65535:
        parser.error('--port 必须在0至65535之间')
    # Fail malformed overrides before opening/taking over a device.
    if any(x is not None for x in (args.width, args.height, args.fps)):
        if (not all(x is not None for x in (args.width, args.height, args.fps))
                or args.width <= 0 or args.height <= 0 or not math.isfinite(args.fps) or args.fps <= 0):
            parser.error('--width、--height、--fps 必须一起指定为正数')
    state = server = directory = lease = camera = None
    old_signals = {}
    result = 0
    try:
        serial, _ = resolve_role(HERE/'camera_roles.json', 'global')
        clients = robot_camera_clients()
        if clients:
            raise RuntimeError('ACT/示范录制正在使用相机，拒绝独占深度录制；PID：'+
                               ', '.join(str(c['pid']) for c in clients))
        # Verify codecs before camera access. PyAV bundles these; no system ffmpeg needed.
        av.Codec('ffv1', 'w')
        if not args.no_color_video:
            av.Codec('libx264', 'w')
        cv2.setNumThreads(1)
        camera = SDKDepthCamera(serial)
        lease = GlobalPreviewLease(serial, args.takeover_preview, camera)
        if not args.list_formats:
            stamp = datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S_%f')
            directory = (args.output_dir or ROOT/'recordings'/'global_depth'/f'global_depth_{stamp}').expanduser().resolve()
            directory.mkdir(parents=True, exist_ok=False)
            state = LiveState(serial, 'Orbbec DaBai DC1', {}, directory)
            state.update(mode=args.mode, preview_near_mm=args.near_mm, preview_far_mm=args.far_mm,
                mm_per_unit=None, valid_fraction=None, depth_min_mm=None, depth_max_mm=None,
                color_video_frames=0,
                raw_depth_file=str(directory/'depth.mkv'),
                color_video_file=None if args.no_color_video else str(directory/'depth_preview.mp4'),
                spatially_aligned=False, exposure_synchronized=False,
                depth_storage='uint16 little endian, lossless FFV1',
                timestamp_note='device_timestamp_us=raw ob_frame_time_stamp_us; sdk_host_received_us=raw ob_frame_system_time_stamp_us; received_monotonic_ns=script receipt. SDK/firmware may map frame timestamps to host time; no calibrated exposure/UTC clock assertion. MKV timestamps quantized by container time base; exact SDK values in CSV.',
                started_at=datetime.now(ZoneInfo('Asia/Singapore')).isoformat())
            for sig in (signal.SIGINT, signal.SIGTERM):
                old_signals[sig] = signal.signal(sig, lambda *_: state.stop.set())
            server = make_preview_server(state, args.port)
        with lease:
            with camera:
                profile = choose_profile(camera.profiles, args.mode, args.width, args.height, args.fps)
                if args.list_formats:
                    print(json.dumps(dict(serial=serial, device=camera.device_info, selected=profile,
                        max_fps=choose_profile(camera.profiles, 'fps'),
                        max_resolution=choose_profile(camera.profiles, 'resolution'), profiles=camera.profiles),
                        ensure_ascii=False, indent=2), flush=True)
                else:
                    state.update(profile=profile, device=camera.device_info, profiles=camera.profiles)
                    url = f'http://127.0.0.1:{server.server_port}/'
                    print(f'全局 {serial} | {profile["width"]}×{profile["height"]} @ {profile["fps"]:g} fps | {profile["format"]} -> Y16')
                    print(f'保存：{directory}\n实时预览：{url}\nCtrl+C 或预览页“停止并保存”结束。', flush=True)
                    camera.start(profile)
                    if not args.no_browser:
                        threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()
                    record_depth(state, camera, directory, args.duration, not args.no_color_video)
        if not camera.resources_released:
            raise RuntimeError('SDK资源清理失败：'+'; '.join(camera.cleanup_errors))
        if lease.restore_error:
            raise RuntimeError(lease.restore_error)
        if state is not None:
            state.update(state='finished')
    except Exception as exc:
        result = 1
        print(f'深度录制失败：{exc}', file=sys.stderr, flush=True)
        if state is not None:
            state.update(state='error', error=str(exc))
    finally:
        if state is not None:
            state.stop.set()
            state.update(ended_at=datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),
                sdk_resources_released=camera.resources_released,
                sdk_cleanup_errors=camera.cleanup_errors, preview_restore_error=lease.restore_error)
            summary = state.status()
            summary.pop('preview_updated_mono', None)
            try:
                with (directory/'recording.json').open('x') as file:
                    json.dump(summary, file, ensure_ascii=False, indent=2)
                    file.write('\n')
            except OSError as exc:
                print(f'统计保存失败：{exc}', file=sys.stderr)
                result = 1
            print(f'结束：{summary["state"]}，{summary["frames"]} 帧，实测 {summary["measured_fps"]:.2f} fps；{directory}', flush=True)
        if server is not None:
            time.sleep(.8)
            server.shutdown()
            server.server_close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
