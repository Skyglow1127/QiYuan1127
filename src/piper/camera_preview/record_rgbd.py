#!/usr/bin/env python3
"""One recording session, RGB/rgb.mp4 + Depth/depth.mp4, local dual preview.

Depth MP4 is a colored viewing video. No raw-depth MKV is created. RGB UVC and
depth SDK frames have independent exposure/coordinates; output PTS use a shared
host monotonic receipt timeline, without claiming hardware synchronization.
"""
import argparse
from collections import deque
from contextlib import ExitStack
import copy
import csv
from datetime import datetime
from fractions import Fraction
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

from camera_server import resolve_role
from record_wrist_video import UVCCapture, enumerate_modes, choose_mode, robot_camera_clients
from record_global_depth import SDKDepthCamera, choose_profile, colorize_depth
from rgb_uvc_device import resolve_rgb_device

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
PORTS = {'wrist': 18760, 'global': 18761}
LABELS = {'wrist': '腕部相机', 'global': '全局相机'}


def camera_identity(role):
    serial, label = resolve_role(HERE/'camera_roles.json', role)
    return dict(role=role, serial=serial, label=label)


def rgb_device(serial, *, guard=None):
    return resolve_rgb_device(serial, guard=guard)


class PreviewLease:
    def __init__(self, identity, takeover, can_restore):
        self.identity, self.takeover, self.can_restore = identity, takeover, can_restore
        self.restore = False
        self.restore_error = None

    def check_closed(self):
        """Recheck immediately before a missing-node driver recovery."""
        try:
            status = self.request('/status')
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, ConnectionRefusedError):
                return
            raise
        if (status.get('role') != self.identity['role']
                or status.get('requested_serial') != self.identity['serial']
                or status.get('desired_open') or not status.get('stopped')
                or not status.get('resources_released')):
            raise RuntimeError('原相机服务未保持关闭，拒绝恢复RGB驱动')

    def request(self, path, action=False):
        port = PORTS[self.identity['role']]
        req = urllib.request.Request(f'http://127.0.0.1:{port}'+path,
              data=b'' if action else None,
              headers={'X-Camera-Control': '1'} if action else {})
        with OPENER.open(req, timeout=15 if action else 2) as response:
            return json.load(response)

    def __enter__(self):
        try:
            status = self.request('/status')
        except urllib.error.HTTPError as exc:
            raise RuntimeError(self.identity['label']+'预览状态读取失败') from exc
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, ConnectionRefusedError):
                return self
            raise RuntimeError(f'无法确认相机服务状态：{exc}') from exc
        if (status.get('role') != self.identity['role']
                or status.get('requested_serial') != self.identity['serial']
                or status.get('device', {}).get('serial', self.identity['serial']) != self.identity['serial']):
            raise RuntimeError('预览服务的相机身份不符，未接管')
        if status.get('desired_open') or not status.get('stopped', True):
            if not self.takeover:
                raise RuntimeError(self.identity['label']+'预览占用设备；加 --takeover-preview 临时接管')
            if robot_camera_clients():
                raise RuntimeError('ACT/示范录制正在使用相机，未关闭预览')
            self.restore = bool(status.get('desired_open'))
            try:
                print('临时暂停'+self.identity['label']+'原预览，退出后恢复原先开启状态。', flush=True)
                self.request('/camera/close', True)
                status = self.request('/status')
                if not status.get('resources_released') or not status.get('stopped'):
                    raise RuntimeError('原预览尚未释放相机')
            except BaseException:
                self.__exit__(None, None, None)
                raise
        elif not status.get('resources_released', False):
            raise RuntimeError('原相机服务尚未完成设备释放')
        return self

    def __exit__(self, *_):
        if not self.restore:
            return
        self.restore = False
        try:
            if not self.can_restore():
                raise RuntimeError('录制设备清理未确认完成，未自动重开预览')
            self.request('/camera/reconnect', True)
            print('已请求恢复'+self.identity['label']+'原预览。', flush=True)
        except Exception as exc:
            self.restore_error = str(exc)
            print('预览恢复失败：'+str(exc), file=sys.stderr)


class RecordingState:
    def __init__(self, rgb_identity, depth_identity, directory, near, far):
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.latest = {'rgb': (0, None), 'depth': (0, None)}
        self.ready = {'rgb': threading.Event(), 'depth': threading.Event()}
        self.session_start_ns = None
        same = rgb_identity['role'] == depth_identity['role']
        self.data = dict(state='starting', error=None, output=str(directory),
            role=rgb_identity['role'] if same else 'mixed',
            label=rgb_identity['label'] if same else rgb_identity['label']+' RGB + '+depth_identity['label']+'深度',
            serial=rgb_identity['serial'] if same else rgb_identity['serial']+' / '+depth_identity['serial'],
            elapsed_s=0., started_at=datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),
            preview_near_mm=near, preview_far_mm=far,
            spatially_aligned=False, exposure_synchronized=False,
            video_timing='Both MP4 PTS use host monotonic receipt time relative to one session start; not exposure synchronization.',
            video_codec='H264 / yuv420p / CRF18; depth video is pseudocolor viewing only',
            streams={name:dict(identity=identity, profile={}, frames=0, elapsed_s=0.,
                measured_fps=0., recent_fps=0., bytes_written=0, estimated_missing_frames=0,
                output=str(directory/folder/filename), error=None)
                for name, identity, folder, filename in (
                    ('rgb', rgb_identity, 'RGB', 'rgb.mp4'),
                    ('depth', depth_identity, 'Depth', 'depth.mp4'))})

    def status(self):
        with self.condition:
            return copy.deepcopy(self.data)

    def update(self, **fields):
        with self.condition:
            self.data.update(fields)
            self.condition.notify_all()

    def update_stream(self, name, **fields):
        with self.condition:
            self.data['streams'][name].update(fields)
            if self.session_start_ns is not None:
                self.data['elapsed_s'] = (time.monotonic_ns()-self.session_start_ns)/1e9
            self.condition.notify_all()

    def publish(self, name, number, jpeg):
        with self.condition:
            self.latest[name] = (number, jpeg)
            self.condition.notify_all()

    def fail(self, name, error):
        message = f'{name}: {error}'
        with self.condition:
            self.data['streams'][name]['error'] = str(error)
            self.data['error'] = (self.data['error']+'; ' if self.data['error'] else '')+message
            self.data['state'] = 'stopping'
            self.stop.set()
            self.condition.notify_all()


class MP4Writer:
    def __init__(self, path, profile):
        self.path, self.profile = path, profile
        self.file = self.container = self.stream = None
        self.closed = False
        self.last_pts = None

    def __enter__(self):
        try:
            self.file = self.path.open('xb')
            self.container = av.open(self.file, 'w', format='mp4',
                                     options={'movie_timescale': '1000000'})
            self.stream = self.container.add_stream('libx264', rate=Fraction(str(self.profile['fps'])))
            self.stream.width, self.stream.height = self.profile['width'], self.profile['height']
            self.stream.pix_fmt = 'yuv420p'
            self.stream.time_base = self.stream.codec_context.time_base = Fraction(1, 1000000)
            self.stream.codec_context.thread_count = 2
            self.stream.options = {'preset': 'ultrafast', 'crf': '18', 'tune': 'zerolatency'}
            return self
        except BaseException:
            self.close()
            raise

    def write(self, frame, pts_us):
        if (frame.width, frame.height) != (self.profile['width'], self.profile['height']):
            raise RuntimeError('实际图像尺寸与选定档位不同，拒绝静默降档')
        if pts_us < 0 or (self.last_pts is not None and pts_us <= self.last_pts):
            raise RuntimeError('共同主机时间轴重复或倒退')
        frame.pts, frame.time_base = pts_us, Fraction(1, 1000000)
        for packet in self.stream.encode(frame):
            self.container.mux(packet)
        self.last_pts = pts_us

    def size(self):
        return self.file.tell() if self.file is not None and not self.file.closed else (
            self.path.stat().st_size if self.path.exists() else 0)

    def close(self):
        if self.closed:
            return
        self.closed = True
        errors = []
        if self.container is not None:
            try:
                if self.stream is not None:
                    for packet in self.stream.encode(None):
                        self.container.mux(packet)
            except Exception as exc:
                errors.append(str(exc))
            try:
                self.container.close()
            except Exception as exc:
                errors.append(str(exc))
        if self.file is not None:
            try:
                self.file.close()
            except OSError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('MP4写尾失败：'+'; '.join(errors))

    def __exit__(self, *_):
        self.close()


def record_stream(name, state, capture, directory, profile):
    path = directory/('RGB/rgb.mp4' if name == 'rgb' else 'Depth/depth.mp4')
    frames = missing = warmup_frames = 0
    last_ts = last_seq = first_ts = None
    recent = deque(maxlen=90)
    last_preview = 0.
    try:
        decoder = av.CodecContext.create('mjpeg', 'r') if name == 'rgb' else None
        with path.with_suffix('.timestamps.csv').open('x', newline='') as times_file:
            times = csv.writer(times_file)
            times.writerow(['frame', 'source_sequence', 'source_timestamp_us', 'sdk_host_received_us',
                            'received_monotonic_ns', 'session_pts_us', 'mm_per_unit', 'source_flags'])
            with MP4Writer(path, profile) as writer:
                while not state.stop.is_set():
                    item = capture.read(state.stop)
                    if item is None:
                        break
                    if name == 'rgb':
                        raw, source_ts, seq, flags = item
                        received = time.monotonic_ns()
                        if not raw.startswith(b'\xff\xd8'):
                            raise RuntimeError('RGB不是预期的MJPEG数据')
                        decoded = decoder.decode(av.Packet(raw))
                        if len(decoded) != 1:
                            raise RuntimeError('RGB JPEG未解出唯一图像')
                        frame = decoded[0]
                        scale, sdk_host = '', ''
                        stats = {}
                        jpeg = raw
                    else:
                        source_ts, seq = item['device_timestamp_us'], item['sequence']
                        received, sdk_host = item['received_monotonic_ns'], item['sdk_host_received_us']
                        flags, scale = '', item['mm_per_unit']
                        if not math.isfinite(scale) or scale <= 0:
                            raise RuntimeError('深度比例系数无效')
                        if ((item['width'], item['height']) != (profile['width'], profile['height'])
                                or len(item['raw']) != profile['width']*profile['height']*2):
                            raise RuntimeError('实际深度尺寸/字节布局不符')
                        depth = np.frombuffer(item['raw'], dtype='<u2').reshape(profile['height'], profile['width'])
                        colors, stats = colorize_depth(depth, scale,
                            state.data['preview_near_mm'], state.data['preview_far_mm'])
                        stats['mm_per_unit'] = scale
                        frame = av.VideoFrame.from_ndarray(colors, format='bgr24')
                        jpeg = None
                    if source_ts < 0 or (last_ts is not None and source_ts <= last_ts):
                        raise RuntimeError('来源时间戳重复或倒退')
                    if seq < 0 or (last_seq is not None and seq <= last_seq):
                        raise RuntimeError('来源帧序号重复或倒退')
                    if state.session_start_ns is None:
                        # Consume startup queues while the other source starts.
                        # Neither video begins until both have delivered a
                        # valid frame; this avoids encoding queued startup
                        # depth frames in a burst before RGB is ready.
                        warmup_frames += 1
                        state.update_stream(name, warmup_discarded_frames=warmup_frames)
                        state.ready[name].set()
                        continue
                    pts_us = (received-state.session_start_ns)//1000
                    if pts_us < 0:
                        continue
                    writer.write(frame, pts_us)
                    frames += 1
                    missing += max(0, seq-last_seq-1) if last_seq is not None else 0
                    if first_ts is None:
                        first_ts = source_ts
                    last_ts, last_seq = source_ts, seq
                    recent.append(source_ts)
                    elapsed = (source_ts-first_ts)/1000000
                    fps = (frames-1)/elapsed if elapsed else 0.
                    recent_fps = ((len(recent)-1)*1000000/(recent[-1]-recent[0]) if len(recent)>1 else 0.)
                    times.writerow([frames, seq, source_ts, sdk_host, received, pts_us, scale, flags])
                    state.update_stream(name, frames=frames, elapsed_s=elapsed,
                        measured_fps=fps, recent_fps=recent_fps, bytes_written=writer.size(),
                        estimated_missing_frames=missing, **stats)
                    if time.monotonic()-last_preview >= 1/15:
                        if name == 'depth':
                            ok, encoded = cv2.imencode('.jpg', colors, [cv2.IMWRITE_JPEG_QUALITY, 85])
                            if not ok:
                                raise RuntimeError('深度预览编码失败')
                            jpeg = encoded.tobytes()
                        state.publish(name, frames, jpeg)
                        last_preview = time.monotonic()
                if not frames:
                    raise RuntimeError('本次没有收到'+name+'图像')
    except Exception as exc:
        state.fail(name, exc)
    finally:
        state.update_stream(name, bytes_written=path.stat().st_size if path.exists() else 0)


def make_server(state, port):
    page = (HERE/'rgbd_recorder.html').read_bytes()
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
                elif self.path in ('/rgb.mjpg', '/depth.mjpg'):
                    name = self.path[1:].split('.')[0]
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.connection.settimeout(2)
                    last = 0
                    while True:
                        with state.condition:
                            state.condition.wait_for(lambda: state.latest[name][0] != last or state.stop.is_set(), timeout=1)
                            number, jpeg = state.latest[name]
                            if number == last:
                                if state.stop.is_set():
                                    break
                                continue
                            last = number
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                            str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
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
                        state.update(state='stopping')
                        state.stop.set()
                    self.reply(b'{"ok":true}')
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .1}, daemon=True).start()
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description='同次录制 RGB + 深度伪彩色：两个文件夹、两段MP4、双画面窗口。')
    parser.add_argument('--camera', choices=PORTS, default='wrist', help='默认腕部相机，同时取RGB与深度')
    parser.add_argument('--rgb-camera', choices=PORTS, help='独立指定RGB相机，覆盖--camera')
    parser.add_argument('--depth-camera', choices=PORTS, help='独立指定深度相机，覆盖--camera')
    parser.add_argument('--duration', type=float, default=0, help='秒；0持续至Ctrl+C或网页停止')
    parser.add_argument('--output-dir', type=Path, help='新的会话目录，内建RGB和Depth两个文件夹')
    parser.add_argument('--port', type=int, default=18765)
    parser.add_argument('--near-mm', type=float, default=200)
    parser.add_argument('--far-mm', type=float, default=2000)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--takeover-preview', action='store_true', help='临时暂停所选相机原预览，退出恢复原状态')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration 必须是有限非负数')
    if (not math.isfinite(args.near_mm) or not math.isfinite(args.far_mm)
            or not 0 <= args.near_mm < args.far_mm):
        parser.error('预览范围必须满足 0 <= near-mm < far-mm')
    if not 0 <= args.port <= 65535:
        parser.error('--port 必须在0至65535之间')
    state = server = directory = depth_capture = rgb_capture = None
    leases, threads, old_signals = [], [], {}
    result = 0
    try:
        clients = robot_camera_clients()
        if clients:
            raise RuntimeError('ACT/示范录制正在使用相机，PID：'+', '.join(str(c['pid']) for c in clients))
        av.Codec('libx264', 'w')
        cv2.setNumThreads(1)
        rgb_id = camera_identity(args.rgb_camera or args.camera)
        depth_id = camera_identity(args.depth_camera or args.camera)
        stamp = datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S_%f')
        directory = (args.output_dir or ROOT/'recordings'/'rgbd'/f'rgbd_{stamp}').expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=False)
        for folder in ('RGB', 'Depth'):
            (directory/folder).mkdir()
        state = RecordingState(rgb_id, depth_id, directory, args.near_mm, args.far_mm)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, lambda *_: state.stop.set())
        server = make_server(state, args.port)
        depth_capture = SDKDepthCamera(depth_id['serial'])
        def can_restore():
            return depth_capture.resources_released and (rgb_capture is None or rgb_capture.fd is None)
        with ExitStack() as stack:
            identities = {identity['role']:identity for identity in (rgb_id, depth_id)}
            for identity in identities.values():
                lease = PreviewLease(identity, args.takeover_preview, can_restore)
                leases.append(lease)
                stack.enter_context(lease)
            def recovery_guard():
                if robot_camera_clients():
                    raise RuntimeError('ACT/示范录制正在使用相机，拒绝恢复RGB驱动')
                for lease in leases:
                    lease.check_closed()
            # LibUVC previews can temporarily remove V4L2 nodes. Resolve only
            # after all selected previews have closed and released their SDKs.
            device = rgb_device(rgb_id['serial'], guard=recovery_guard)
            rgb_modes = enumerate_modes(device)
            rgb_profile = choose_mode(rgb_modes)
            state.update_stream('rgb', profile=rgb_profile)
            stack.enter_context(depth_capture)
            depth_profile = choose_profile(depth_capture.profiles, 'fps')
            state.update_stream('depth', profile=depth_profile)
            state.update(rgb_modes=rgb_modes, depth_profiles=depth_capture.profiles,
                         depth_sdk_device=depth_capture.device_info)
            # Depth-only SDK first; RGB uses its separate serial-bound UVC node.
            depth_capture.start(depth_profile)
            rgb_capture = stack.enter_context(UVCCapture(device, rgb_profile))
            url = f'http://127.0.0.1:{server.server_port}/'
            try:
                for name, capture, profile in (('rgb', rgb_capture, rgb_profile), ('depth', depth_capture, depth_profile)):
                    thread = threading.Thread(target=record_stream,
                        args=(name, state, capture, directory, profile), name='record-'+name)
                    thread.start()
                    threads.append(thread)  # Only started threads may be joined.
                ready_deadline = time.monotonic()+8
                while not all(event.is_set() for event in state.ready.values()):
                    if state.stop.wait(.02):
                        break
                    if time.monotonic() >= ready_deadline:
                        raise TimeoutError('RGB或深度在8秒内未准备好，已停止两路')
                if state.stop.is_set():
                    raise RuntimeError(state.status()['error'] or '启动阶段已停止')
                state.session_start_ns = time.monotonic_ns()
                state.update(state='recording', session_start_monotonic_ns=state.session_start_ns)
                print(f'RGB：{rgb_id["label"]} {rgb_id["serial"]} | {rgb_profile["width"]}×{rgb_profile["height"]} @ {rgb_profile["fps"]:g}fps')
                print(f'深度：{depth_id["label"]} {depth_id["serial"]} | {depth_profile["width"]}×{depth_profile["height"]} @ {depth_profile["fps"]:g}fps')
                print(f'保存：{directory}\n双画面预览：{url}\nCtrl+C 或预览页“停止并保存”结束。', flush=True)
                if not args.no_browser:
                    threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()
                last_print = 0.
                while any(thread.is_alive() for thread in threads):
                    elapsed = (time.monotonic_ns()-state.session_start_ns)/1e9
                    if args.duration and elapsed >= args.duration:
                        state.stop.set()
                    for thread in threads:
                        thread.join(timeout=.05)
                    if time.monotonic()-last_print >= 1:
                        s = state.status()
                        print(f'\r录制 {elapsed:6.1f}s | RGB {s["streams"]["rgb"]["frames"]}帧 / '
                              f'{s["streams"]["rgb"]["measured_fps"]:.1f}fps | 深度 '
                              f'{s["streams"]["depth"]["frames"]}帧 / {s["streams"]["depth"]["measured_fps"]:.1f}fps   ',
                              end='', flush=True)
                        last_print = time.monotonic()
            finally:
                state.stop.set()
                for thread in threads:
                    thread.join()
                print(flush=True)
        if not depth_capture.resources_released:
            raise RuntimeError('SDK资源清理失败：'+'; '.join(depth_capture.cleanup_errors))
        restore_errors = [lease.restore_error for lease in leases if lease.restore_error]
        if restore_errors:
            raise RuntimeError('; '.join(restore_errors))
        if state.status()['error']:
            result = 1
        state.update(state='error' if result else 'finished')
    except Exception as exc:
        result = 1
        print('RGB/深度录制失败：'+str(exc), file=sys.stderr, flush=True)
        if state is not None:
            previous = state.status()['error']
            state.update(state='error', error=(previous+'; ' if previous else '')+str(exc))
    finally:
        if state is not None:
            state.stop.set()
            for thread in threads:
                thread.join()
            state.update(ended_at=datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),
                sdk_resources_released=depth_capture.resources_released if depth_capture else True,
                sdk_cleanup_errors=depth_capture.cleanup_errors if depth_capture else [],
                preview_restore_errors=[lease.restore_error for lease in leases if lease.restore_error])
            summary = state.status()
            try:
                with (directory/'recording.json').open('x') as output:
                    json.dump(summary, output, ensure_ascii=False, indent=2)
                    output.write('\n')
            except OSError as exc:
                print('统计写入失败：'+str(exc), file=sys.stderr)
                result = 1
            print(f'结束：{summary["state"]}；{directory}', flush=True)
        if server is not None:
            time.sleep(.8)
            server.shutdown()
            server.server_close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
