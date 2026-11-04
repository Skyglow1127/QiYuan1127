#!/usr/bin/env python3
"""Standalone wrist RGB video; original UVC MJPEG -> MKV, without re-encoding.

This is camera footage, not a LeRobot demonstration/dataset recorder. No robot,
CAN or Orbbec SDK imports. Linux 64-bit V4L2 single-plane mmap ABI.
"""
import argparse
from collections import deque
import csv
from datetime import datetime
import errno
import fcntl
from fractions import Fraction
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mmap
import os
from pathlib import Path
import select
import signal
import struct
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MJPG = struct.unpack('<I', b'MJPG')[0]
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def ioctl_buffer(fd, request, size, fields=()):
    buf = bytearray(size)
    if fields:
        struct.pack_into('<' + 'I' * len(fields), buf, 0, *fields)
    fcntl.ioctl(fd, request, buf, True)
    return buf


def enum_buffer(fd, request, size, fields):
    try:
        return ioctl_buffer(fd, request, size, fields)
    except OSError as exc:
        if exc.errno == errno.EINVAL:
            return None
        raise


def wrist_identity(roles_file):
    roles = json.loads(roles_file.read_text())['cameras']
    camera = roles['wrist']
    serial = camera['serial']
    if (not isinstance(serial, str) or not serial.isalnum()
            or camera.get('identity_verified_from_image') is not True
            or sum(c.get('serial') == serial for c in roles.values()) != 1):
        raise ValueError('腕部相机必须有唯一、已验证的序列号绑定')
    # Use udev serial identity, never assume /dev/video0 is the wrist camera.
    matches = list(Path('/dev/v4l/by-id').glob(f'*_{serial}-video-index0'))
    if len(matches) != 1:
        raise RuntimeError(f'找不到唯一腕部 RGB 设备 {serial}：{matches}')
    return serial, camera.get('model', 'DaBai DC1'), matches[0]


def enumerate_modes(device):
    """ENUM-only ioctls; do not change settings or start streaming."""
    modes = set()
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    try:
        for index in range(128):
            fmt = enum_buffer(fd, 0xc0405602, 64, (index, 1))  # ENUM_FMT
            if fmt is None:
                break
            code = struct.unpack_from('<I', fmt, 44)[0]
            name = struct.pack('<I', code).decode('ascii', errors='replace')
            for size_index in range(256):
                size = enum_buffer(fd, 0xc02c564a, 44, (size_index, code))
                if size is None:
                    break
                kind, width, height = struct.unpack_from('<III', size, 8)
                if kind != 1:
                    continue  # DC1 has discrete profiles; do not invent modes.
                for interval_index in range(256):
                    interval = enum_buffer(fd, 0xc034564b, 52,
                                           (interval_index, code, width, height))
                    if interval is None:
                        break
                    kind, numerator, denominator = struct.unpack_from('<III', interval, 16)
                    if kind == 1 and numerator and denominator:
                        modes.add((name, width, height, Fraction(denominator, numerator)))
    finally:
        os.close(fd)
    return [dict(format=f, width=w, height=h, fps=float(rate))
            for f, w, h, rate in sorted(modes, key=lambda p: (p[0], -p[1]*p[2], -p[3]))]


def choose_mode(modes, width=None, height=None, fps=None):
    supported = [m for m in modes if m['format'] == 'MJPG']
    if not supported:
        raise RuntimeError('该设备没有可直接保存的 MJPEG 档位')
    if any(x is not None for x in (width, height, fps)):
        if not all(x is not None for x in (width, height, fps)):
            raise ValueError('--width、--height、--fps 必须一起指定')
        supported = [m for m in supported if (m['width'], m['height']) == (width, height)
                     and abs(m['fps']-fps) < 1e-6]
        if not supported:
            raise ValueError('指定档位不在相机枚举表中；先运行 --list-formats')
    best = max(supported, key=lambda m: (m['width']*m['height'], m['fps']))
    if width is None and best['fps'] < max(m['fps'] for m in supported):
        raise RuntimeError('最高分辨率与最高帧率不能同时达到，请显式指定三个档位参数')
    return dict(best)


class UVCCapture:
    """Nonblocking mmap capture with finite waits, including unplug/stop paths."""
    def __init__(self, device, profile):
        self.device, self.profile = device, profile
        self.fd, self.maps, self.started = None, [], False

    def __enter__(self):
        if struct.calcsize('P') != 8 or sys.byteorder != 'little':
            raise RuntimeError('当前 V4L2 绑定仅支持 Linux 64 位小端系统')
        self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        try:
            p = self.profile
            fmt = bytearray(208)  # v4l2_format: union starts at 8 on 64-bit Linux
            struct.pack_into('<I', fmt, 0, 1)
            struct.pack_into('<IIII', fmt, 8, p['width'], p['height'], MJPG, 0)
            fcntl.ioctl(self.fd, 0xc0d05605, fmt, True)  # S_FMT
            w, h, code = struct.unpack_from('<III', fmt, 8)
            if (w, h, code) != (p['width'], p['height'], MJPG):
                raise RuntimeError(f'驱动降低了请求档位，拒绝录制：{w}×{h} / {code}')
            rate = Fraction(str(p['fps'])).limit_denominator(100000)
            parm = bytearray(204)  # v4l2_streamparm.capture.timeperframe
            struct.pack_into('<I', parm, 0, 1)
            struct.pack_into('<II', parm, 12, rate.denominator, rate.numerator)
            fcntl.ioctl(self.fd, 0xc0cc5616, parm, True)  # S_PARM
            n, d = struct.unpack_from('<II', parm, 12)
            if not n or not d or abs(d/n-p['fps']) > 1e-6:
                raise RuntimeError(f'驱动未接受请求帧率：timeperframe={n}/{d}')
            requested = ioctl_buffer(self.fd, 0xc0145608, 20, (8, 1, 1))
            count = struct.unpack_from('<I', requested)[0]
            if not count:
                raise RuntimeError('驱动没有分配采集缓冲区')
            for index in range(count):
                buf = self.buffer(index)
                fcntl.ioctl(self.fd, 0xc0585609, buf, True)  # QUERYBUF
                length = struct.unpack_from('<I', buf, 72)[0]
                offset = struct.unpack_from('<I', buf, 64)[0]
                self.maps.append(mmap.mmap(self.fd, length, flags=mmap.MAP_SHARED,
                                          prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=offset))
                fcntl.ioctl(self.fd, 0xc058560f, self.buffer(index), True)  # QBUF
            ioctl_buffer(self.fd, 0x40045612, 4, (1,))  # STREAMON
            self.started = True
            return self
        except BaseException:
            self.close()
            raise

    @staticmethod
    def buffer(index=0):
        buf = bytearray(88)  # v4l2_buffer, 64-bit timeval/pointer alignment
        struct.pack_into('<II', buf, 0, index, 1)
        struct.pack_into('<I', buf, 60, 1)  # V4L2_MEMORY_MMAP
        return buf

    def read(self, stop):
        deadline = time.monotonic() + 3
        while not stop.is_set():
            if time.monotonic() >= deadline:
                raise TimeoutError('腕部相机连续 3 秒没有新帧，已结束录制')
            ready, _, _ = select.select([self.fd], [], [], .1)
            if not ready:
                continue
            buf = self.buffer()
            try:
                fcntl.ioctl(self.fd, 0xc0585611, buf, True)  # DQBUF
            except OSError as exc:
                if exc.errno == errno.EAGAIN:
                    continue
                raise
            index, _, used, flags = struct.unpack_from('<IIII', buf)
            try:
                if flags & 0x40:
                    raise RuntimeError('驱动标记了一帧损坏数据，已结束录制')
                if index >= len(self.maps) or not 0 < used <= len(self.maps[index]):
                    raise RuntimeError('相机返回了无效视频缓冲区')
                raw = self.maps[index][:used]
                seconds, micros = struct.unpack_from('<qq', buf, 24)
                sequence = struct.unpack_from('<I', buf, 56)[0]
                return raw, seconds*1000000+micros, sequence, flags
            finally:
                fcntl.ioctl(self.fd, 0xc058560f, self.buffer(index), True)
        return None

    def close(self):
        errors = []
        if self.fd is not None:
            if self.started:
                try:
                    ioctl_buffer(self.fd, 0x40045613, 4, (1,))  # STREAMOFF
                except OSError as exc:
                    errors.append(str(exc))
            for buffer in self.maps:
                try:
                    buffer.close()
                except OSError as exc:
                    errors.append(str(exc))
            os.close(self.fd)
            self.fd, self.maps, self.started = None, [], False
        if errors:
            print('采集资源清理：' + '; '.join(errors), file=sys.stderr)

    def __exit__(self, *_):
        self.close()


def robot_camera_clients():
    """Conservative check before taking over a serial-bound preview service."""
    names = {'lerobot-record', 'lerobot_record.py', 'lerobot.scripts.lerobot_record',
             'record_piper_official.sh', 'record_piper.py', 'start_recording.sh',
             'rollout_act_piper.py', 'launch_act.py', 'launch_act_piper.py',
             'launch_act_contact_repair.py', 'launch_act_round2.py', 'launch_act_piper_round2.py',
             'run_act_piper.sh', 'run_act_piper_repair.sh', 'run_act_piper_round2.sh'}
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = [x.decode(errors='replace') for x in path.read_bytes().split(b'\0') if x]
            if any(Path(arg).name in names for arg in argv):
                found.append(dict(pid=int(path.parent.name), argv=argv))
        except (OSError, ValueError):
            continue
    return found


class PreviewLease:
    """Only temporarily close the verified wrist preview; retain its prior state."""
    def __init__(self, serial, takeover):
        self.serial, self.takeover, self.restore = serial, takeover, False

    def request(self, path, action=False):
        req = urllib.request.Request('http://127.0.0.1:18760'+path,
              data=b'' if action else None,
              headers={'X-Camera-Control': '1'} if action else {})
        with OPENER.open(req, timeout=15 if action else 2) as response:
            return json.load(response)

    def __enter__(self):
        try:
            status = self.request('/status')
        except urllib.error.HTTPError:
            raise RuntimeError('腕部预览服务状态读取失败，未关闭任何服务')
        except urllib.error.URLError as exc:
            if not isinstance(exc.reason, ConnectionRefusedError):
                raise RuntimeError(f'无法确认腕部预览状态：{exc}') from exc
            return self
        if (status.get('role') != 'wrist' or status.get('requested_serial') != self.serial
                or status.get('device', {}).get('serial', self.serial) != self.serial):
            raise RuntimeError('18760 端口相机身份不匹配，未接管')
        if status.get('desired_open') or not status.get('stopped', True):
            if not self.takeover:
                raise RuntimeError('腕部预览正在占用相机。停止 ACT/示范录制后，加 --takeover-preview 临时接管。')
            clients = robot_camera_clients()
            if clients:
                raise RuntimeError('检测到 ACT/示范录制进程，未中断相机：' +
                                   ', '.join(str(c['pid']) for c in clients))
            # Mark before POST so a lost close response still triggers restoration.
            self.restore = bool(status.get('desired_open'))
            try:
                print('临时暂停腕部预览；退出时恢复原先开启状态。', flush=True)
                result = self.request('/camera/close', True)
                if result.get('ok') is False:
                    raise RuntimeError(str(result))
                closed = self.request('/status')
                if not closed.get('resources_released') or not closed.get('stopped'):
                    raise RuntimeError('腕部预览尚未释放设备')
            except BaseException:
                self.__exit__(None, None, None)
                raise
        elif not status.get('resources_released', False):
            raise RuntimeError('腕部预览尚未完成设备释放，请稍后重试')
        return self

    def __exit__(self, *_):
        if self.restore:
            self.restore = False
            try:
                self.request('/camera/reconnect', True)
                print('已请求恢复原腕部预览（不启动机械臂）。', flush=True)
            except Exception as exc:
                print(f'腕部预览恢复失败：{exc}；请在原预览页点击重新连接。', file=sys.stderr)


class LiveState:
    def __init__(self, serial, model, profile, output):
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.latest = None
        self.latest_number = 0
        self.data = dict(serial=serial, model=model, profile=profile, state='starting',
                         frames=0, elapsed_s=0., measured_fps=0., recent_fps=0.,
                         bytes_written=0, output=str(output), error=None,
                         estimated_missing_frames=0)

    def status(self):
        with self.condition:
            return dict(self.data)

    def update(self, **values):
        with self.condition:
            self.data.update(values)
            self.condition.notify_all()


def preview_server(state, port):
    page = (HERE/'wrist_recorder.html').read_bytes()

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
                    state.stop.set()
                    state.update(state='stopping')
                    self.reply(b'{"ok":true}')
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .1}, daemon=True).start()
    return server


def record(state, capture, output, duration):
    import av
    first_ts = last_ts = last_seq = None
    arrivals = deque(maxlen=90)
    frames = missing = payload_bytes = 0
    digest = hashlib.sha256()
    started = time.monotonic()
    last_print = 0.
    container = None
    # Exclusive creation protects existing recordings, including sidecars.
    with output.open('xb') as video, output.with_suffix('.timestamps.csv').open('x', newline='') as times:
        writer = csv.writer(times)
        writer.writerow(['frame', 'driver_sequence', 'driver_timestamp_us', 'received_monotonic_ns',
                         'v4l2_flags', 'mjpeg_bytes'])
        try:
            while not state.stop.is_set():
                if duration and time.monotonic()-started >= duration:
                    break
                item = capture.read(state.stop)
                if item is None:
                    break
                raw, timestamp, sequence, flags = item
                received = time.monotonic_ns()
                if not raw.startswith(b'\xff\xd8'):
                    raise RuntimeError('相机输出不是 MJPEG，拒绝重编码替代')
                if timestamp <= 0 or (last_ts is not None and timestamp <= last_ts):
                    raise RuntimeError('采集时间戳未严格递增，已保留此前录像')
                step = 1 if last_seq is None else (sequence-last_seq) % (1 << 32)
                if step == 0 or step > (1 << 31):
                    raise RuntimeError('驱动帧序号重复或倒退，已结束录制')
                if first_ts is None:
                    decoded = av.CodecContext.create('mjpeg', 'r').decode(av.Packet(raw))
                    p = state.data['profile']
                    if len(decoded) != 1 or (decoded[0].width, decoded[0].height) != (p['width'], p['height']):
                        raise RuntimeError('首帧实际尺寸与请求尺寸不同，拒绝静默降档')
                    container = av.open(video, mode='w', format='matroska')
                    stream = container.add_stream('mjpeg', rate=Fraction(str(p['fps'])))
                    stream.width, stream.height = p['width'], p['height']
                    stream.pix_fmt = decoded[0].format.name
                    stream.time_base = Fraction(1, 1000000)
                    container.metadata['title'] = 'Wrist RGB original MJPEG '+state.data['serial']
                    first_ts = timestamp
                packet = av.Packet(raw)
                packet.stream, packet.time_base = stream, Fraction(1, 1000000)
                packet.pts = packet.dts = timestamp-first_ts
                packet.duration = round(1000000/state.data['profile']['fps'])
                packet.is_keyframe = True
                container.mux(packet)
                digest.update(raw)
                frames += 1
                payload_bytes += len(raw)
                missing += max(0, step-1)
                last_ts, last_seq = timestamp, sequence
                arrivals.append(timestamp)
                elapsed = (timestamp-first_ts)/1000000
                fps = (frames-1)/elapsed if elapsed > 0 else 0.
                recent = ((len(arrivals)-1)*1000000/(arrivals[-1]-arrivals[0])
                          if len(arrivals) > 1 else 0.)
                writer.writerow([frames, sequence, timestamp, received, flags, len(raw)])
                with state.condition:
                    state.latest, state.latest_number = raw, frames
                    state.data.update(state='recording', frames=frames, elapsed_s=elapsed,
                                      measured_fps=fps, recent_fps=recent, bytes_written=video.tell(),
                                      estimated_missing_frames=missing)
                    state.condition.notify_all()
                if time.monotonic()-last_print >= 1:
                    print(f'\r录制 {elapsed:7.1f}s | {fps:5.1f} fps | {frames} 帧 | '
                          f'{payload_bytes/1048576:.1f} MiB | 驱动缺帧 {missing}   ', end='', flush=True)
                    last_print = time.monotonic()
            if not frames:
                raise RuntimeError('录制结束前没有收到任何视频帧')
        finally:
            if container is not None:
                container.close()
            state.update(source_mjpeg_sha256=digest.hexdigest(), payload_bytes=payload_bytes,
                         bytes_written=video.tell(), wall_duration_s=time.monotonic()-started)
            print(flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='腕部彩色视频：自动最高 MJPEG 分辨率/帧率，浏览器实时预览，原流保存 MKV。')
    parser.add_argument('--list-formats', action='store_true', help='只枚举设备支持档位，不开始采集')
    parser.add_argument('--duration', type=float, default=0, help='录制秒数；默认 0 持续至 Ctrl+C/网页停止')
    parser.add_argument('--output', type=Path, help='保存路径 .mkv；默认项目 recordings/wrist_video/')
    parser.add_argument('--width', type=int)
    parser.add_argument('--height', type=int)
    parser.add_argument('--fps', type=float)
    parser.add_argument('--port', type=int, default=18763, help='本地实时预览端口，默认 18763')
    parser.add_argument('--no-browser', action='store_true', help='不自动打开浏览器，仍提供预览网址')
    parser.add_argument('--takeover-preview', action='store_true', help='无 ACT/示范录制进程时临时暂停腕部预览，结束恢复')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration 必须是有限的非负秒数')
    if not 0 <= args.port <= 65535:
        parser.error('--port 必须介于 0 和 65535')
    server = state = None
    old_signals = {}
    result = 0
    try:
        serial, model, device = wrist_identity(HERE/'camera_roles.json')
        modes = enumerate_modes(device)
        profile = choose_mode(modes, args.width, args.height, args.fps)
        if args.list_formats:
            print(json.dumps(dict(serial=serial, device=str(device), selected=profile, modes=modes),
                             ensure_ascii=False, indent=2))
            return 0
        # Check before any takeover or camera configuration.
        import av  # noqa: F401
        clients = robot_camera_clients()
        if clients:
            raise RuntimeError('独占最高画质录制需要先结束 ACT/示范录制；检测到 PID：'+
                               ', '.join(str(c['pid']) for c in clients))
        stamp = datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S_%f')
        output = (args.output or ROOT/'recordings'/'wrist_video'/f'wrist_{stamp}.mkv').expanduser().resolve()
        if output.suffix.lower() != '.mkv':
            raise ValueError('--output 必须使用 .mkv 后缀（直接封装相机原 MJPEG）')
        for path in (output, output.with_suffix('.json'), output.with_suffix('.timestamps.csv')):
            if path.exists():
                raise FileExistsError(f'拒绝覆盖已有文件：{path}')
        output.parent.mkdir(parents=True, exist_ok=True)
        state = LiveState(serial, model, profile, output)
        state.update(device=str(device), modes=modes, started_at=datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),
                     capture='V4L2 mmap; original MJPEG packets, no re-encoding', audio=False)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, lambda *_: state.stop.set())
        server = preview_server(state, args.port)  # Fail port binding before taking camera.
        url = f'http://127.0.0.1:{server.server_port}/'
        print(f'腕部 {serial} | {profile["width"]}×{profile["height"]} @ {profile["fps"]:g} fps MJPEG')
        print(f'保存：{output}\n预览：{url}\nCtrl+C 或预览页“停止并保存”结束。', flush=True)
        with PreviewLease(serial, args.takeover_preview):
            with UVCCapture(device, profile) as capture:
                if not args.no_browser:
                    threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()
                record(state, capture, output, args.duration)
        state.update(state='finished')
    except Exception as exc:
        result = 1
        print(f'录制失败：{exc}', file=sys.stderr, flush=True)
        if state is not None:
            state.update(state='error', error=str(exc))
    finally:
        if state is not None:
            state.stop.set()
            state.update(ended_at=datetime.now(ZoneInfo('Asia/Singapore')).isoformat())
            summary = state.status()
            try:
                with Path(summary['output']).with_suffix('.json').open('x') as file:
                    json.dump(summary, file, ensure_ascii=False, indent=2)
                    file.write('\n')
            except OSError as exc:
                print(f'录制统计写入失败：{exc}', file=sys.stderr)
                result = 1
            print(f'结束：{summary["state"]}，{summary["frames"]} 帧，'
                  f'实测 {summary["measured_fps"]:.2f} fps；{summary["output"]}', flush=True)
        if server is not None:
            # Give the open page time to display the final status, then release port.
            time.sleep(.8)
            server.shutdown()
            server.server_close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
