#!/usr/bin/env python3
"""Read-only D1 feedback + bounded KR260 capture; never sends robot commands.

Reuse the deployed D2C/PL pipeline. A local monotonic observation window encloses
the remote request, capture and completion, so KR and AIPC wall clocks are never
subtracted. This establishes sampled stationarity, not exposure synchronization.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

CONTROL = Path(__file__).resolve().parents[1]
CAMERA = Path('/home/aup/codex/kr260-camera')
REMOTE = '/home/ubuntu/kr260-camera'
MAX_GAP = 0.6
PRE_SECONDS = 1.5
POST_SECONDS = 1.0
ARM_SPAN_DEG = 0.15
GRIPPER_SPAN = 0.3
CAMERA_PROCESS_PATTERN = r'^([^ ]*/)?python3( -u)? (/[^ ]*/)?(capture_orbbec|run_depth_gate)\.py( |$)'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def command(args, timeout=12):
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return {'returncode': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}


def ssh_options(args):
    return ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=5',
            '-o', 'ServerAliveCountMax=2', '-o', f'ControlPath={args.control_path}']


def check_link(nic):
    if Path('/sys/class/net', nic, 'carrier').read_text().strip() != '1':
        raise ValueError(f'{nic}: no physical carrier; D1 cable/link required')
    state = command(['ip', '-j', '-4', 'address', 'show', 'dev', nic])
    if state['returncode']:
        raise ValueError(state['stderr'])
    addresses = [a['local'] for n in json.loads(state['stdout']) for a in n['addr_info']]
    if not any(a.startswith('192.168.123.') for a in addresses):
        raise ValueError(f'{nic}: no 192.168.123.x address; check saved D1 network profile')
    return addresses


def parse_feedback(line, stamp):
    if '[tx]' in line:
        raise ValueError('Unexpected command transmission from read-only observer')
    if not line.startswith('[rx] '):
        return None
    payload = json.loads(line[5:])
    if payload.get('address') != 2:
        return None
    data = payload.get('data', {})
    if payload.get('funcode') == 1:
        q = [data[f'angle{i}'] for i in range(7)]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in q):
            raise ValueError('Invalid joint feedback')
        return {'t': stamp, 'kind': 'angles', 'q': q}
    if payload.get('funcode') == 3:
        state = [data.get(k) for k in ('power_status', 'enable_status', 'error_status')]
        if any(type(v) is not int for v in state):
            raise ValueError('Incomplete power/enable/error feedback')
        return {'t': stamp, 'kind': 'status', 'state': state}
    return None


def assess_window(events, begin, end):
    """Check every received frame, coverage, gaps, and whole-window excursion."""
    import numpy as np
    if not math.isfinite(begin + end) or end - begin < PRE_SECONDS:
        raise ValueError('Observation window too short')
    chosen = [e for e in events if begin <= e['t'] <= end]
    if any(b['t'] < a['t'] for a, b in zip(chosen, chosen[1:])):
        raise ValueError('Nonmonotonic observation timestamps')
    groups = {k: [e for e in chosen if e['kind'] == k] for k in ('angles', 'status')}
    gaps = {}
    for kind, frames in groups.items():
        if len(frames) < 3:
            raise ValueError(f'Insufficient {kind} feedback')
        stamps = [begin] + [e['t'] for e in frames] + [end]
        gaps[kind] = max(b-a for a, b in zip(stamps, stamps[1:]))
        if gaps[kind] > MAX_GAP:
            raise ValueError(f'{kind} feedback gap > {MAX_GAP}s')
    if any(e['state'] != [1, 1, 0] for e in groups['status']):
        raise ValueError('Power/enable/error state changed or not ready')
    values = np.array([e['q'] for e in groups['angles']], dtype=float)
    if values.ndim != 2 or values.shape[1] != 7 or not np.isfinite(values).all():
        raise ValueError('Invalid angle array')
    span = np.ptp(values, axis=0)
    if np.any(span[:6] > ARM_SPAN_DEG + 1e-5) or span[6] > GRIPPER_SPAN + 1e-5:
        raise ValueError(f'Robot not stationary: full-window spans {span.tolist()}')
    return {'begin_aipc_monotonic_s': begin, 'end_aipc_monotonic_s': end,
            'sampled_stationarity_verified': True, 'max_feedback_gaps_s': gaps,
            'frame_counts': {k: len(v) for k, v in groups.items()},
            'joint_median': np.median(values, axis=0).tolist(), 'joint_spans': span.tolist(),
            'arm_span_limit_deg': ARM_SPAN_DEG, 'gripper_span_limit_sdk': GRIPPER_SPAN,
            'note': 'Host-arrival feedback only; no synchronized exposure or external motion measurement.'}


class Observer:
    def __init__(self, binary, nic, output):
        # stdin is closed; there is no write/command API in this collector.
        self.process = subprocess.Popen([str(binary), '--nic', nic, '--verbose', 'watch', '90'],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.pending = queue.Queue()
        self.events = []
        self.raw = (output/'feedback.jsonl').open('w')
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        for line in self.process.stdout:
            self.pending.put((time.monotonic(), line.rstrip()))

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                stamp, line = self.pending.get(timeout=min(.05, max(.001, deadline-time.monotonic())))
            except queue.Empty:
                if self.process.poll() is not None:
                    raise RuntimeError('D1 read-only observer exited')
                continue
            self.raw.write(json.dumps({'t': stamp, 'line': line}) + '\n')
            self.raw.flush()
            event = parse_feedback(line, stamp)
            if event is not None:
                self.events.append(event)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.thread.join(timeout=2)
        self.process.stdout.close()
        self.raw.close()


def prepare(folder):
    spec = importlib.util.spec_from_file_location('kr_snapshot', CAMERA/'prepare_aligned_snapshot.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.prepare(folder)


def validate_camera_report(report):
    if report.get('alignment_mode') not in ('SDK_D2C_HW', 'SDK_D2C_SW'):
        raise ValueError('SDK D2C alignment required')
    if report.get('accepted_pairs', 0) < 1 or not report.get('fpga_results'):
        raise ValueError('No paired images or no PL processing results')
    if any(p.get('implementation') != 'FPGA_PL_HLS' or p.get('pixels') != 640*480
           for p in report['fpga_results']):
        raise ValueError('Unexpected PL implementation or frame dimensions')
    # The stock converter verifies raw hashes, intrinsics, dimensions and skew.


def capture(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    result = {'robot_motion_commands_sent': False, 'handeye_sample_ready': False,
              'handeye_calibrated': False, 'complete': False}
    observer = remote_process = None
    try:
        check_link(args.nic)
        observer = Observer(CONTROL/'build/arm_control', args.nic, out)
        # Allow DDS discovery, then require a separate full stationary interval.
        observer.pump(2)
        begin = time.monotonic()
        observer.pump(PRE_SECONDS + .2)
        assess_window(observer.events, begin, time.monotonic())
        name = 'handeye-' + uuid.uuid4().hex
        result['remote_directory'] = REMOTE + '/' + name
        # Existing pipeline unchanged. sudo never prompts or receives a password.
        # A remote timeout also bounds a capture if SSH is interrupted.
        remote_command = ('if pgrep -f ' + shlex.quote(CAMERA_PROCESS_PATTERN)
                          + '; then echo "Existing camera/PL process; capture refused"; exit 3; fi; '
                          + 'cd ' + shlex.quote(REMOTE) + ' && ' + shlex.join([
            'flock', '-n', '/tmp/d1-handeye-camera.lock',
            'sudo', '-n', 'timeout', '--signal=INT', '--kill-after=5s', '25s',
            './run_camera.sh', '--mode', 'both', '--align', '--fpga',
            '--seconds', '5', '--output', name]))
        request_begin = time.monotonic()
        with (out/'remote_capture.log').open('w') as log:
            remote_process = subprocess.Popen(['ssh', *ssh_options(args), args.host, remote_command],
                                              stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
            while remote_process.poll() is None:
                observer.pump(.1)
                assess_window(observer.events, begin, time.monotonic())
                if time.monotonic() - request_begin > 40:
                    raise TimeoutError('Remote camera capture timed out')
            if remote_process.returncode:
                raise RuntimeError('KR capture failed; see remote_capture.log (including sudo/UIO permissions)')
        request_end = time.monotonic()
        observer.pump(POST_SECONDS)
        end = time.monotonic()
        result['observation'] = assess_window(observer.events, begin, end)
        result['capture_window'] = {'request_begin_aipc_monotonic_s': request_begin,
                                    'completion_aipc_monotonic_s': request_end,
                                    'cross_host_timestamp_subtraction': False}
        observer.close()
        observer = None
        camera = out/'camera'
        camera.mkdir()
        for filename in ('report.json', 'color.jpg', 'depth_u16le.raw.gz'):
            transfer = command(['scp', *ssh_options(args),
                                f'{args.host}:{REMOTE}/{name}/{filename}', str(camera/filename)], 120)
            if transfer['returncode']:
                raise RuntimeError('Snapshot transfer failed: ' + transfer['stderr'])
        report = json.loads((camera/'report.json').read_text())
        validate_camera_report(report)
        prepare(camera)
        result['camera_serial'] = report['device']['serial']
        result['raw_sha256'] = {str(p.relative_to(out)): digest(p) for p in
                                [out/'feedback.jsonl', camera/'report.json', camera/'color.jpg',
                                 camera/'depth_u16le.raw.gz']}
        result['complete'] = True
        result['note'] = 'Paired raw capture only. Export requires measured board and validated joint model.'
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if remote_process is not None and remote_process.poll() is None:
            remote_process.terminate()
            try:
                remote_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                remote_process.kill()
                remote_process.wait()
        if observer is not None:
            observer.close()
        save(out/'capture.json', result)
    print(json.dumps(result, indent=2))
    return 0 if result['complete'] else 1


def verified_evidence(config, root):
    evidence = config.get('evidence', [])
    if config.get('validated') is not True or not evidence:
        raise ValueError('Measured validation and evidence required; examples are not calibration')
    for entry in evidence:
        if digest(root/entry['path']) != entry['sha256']:
            raise ValueError('Validation evidence checksum mismatch')


def mapped_pose(mapping_path, q):
    import numpy as np
    import xml.etree.ElementTree as ET
    mapping = json.loads(mapping_path.read_text())
    verified_evidence(mapping, mapping_path.parent)
    if mapping.get('base_frame') != 'base_link' or mapping.get('end_frame') != 'Link6':
        raise ValueError('Expected base_link -> rigid Link6, not grasp_tcp')
    urdf = (mapping_path.parent/mapping['urdf']).resolve()
    if digest(urdf) != mapping['urdf_sha256']:
        raise ValueError('URDF differs from validated model')
    entries = mapping['joints']
    if (len(entries) != 6 or {e['sdk_id'] for e in entries} != set(range(6))
            or {e['urdf_joint'] for e in entries} != {f'Joint{i}' for i in range(1, 7)}):
        raise ValueError('Need unique SDK J0..J5 and URDF Joint1..Joint6 mapping')
    values = {}
    tree = ET.parse(urdf).getroot()
    for entry in entries:
        if type(entry['sdk_id']) is not int or type(entry['sign']) is not int or entry['sign'] not in (-1, 1):
            raise ValueError('Invalid SDK ID or sign')
        offset = entry['offset_rad']
        if type(offset) not in (int, float) or not math.isfinite(offset):
            raise ValueError('Measured finite offset required')
        value = math.radians(q[entry['sdk_id']]) * entry['sign'] + offset
        joint = tree.find(f"joint[@name='{entry['urdf_joint']}']")
        limit = joint.find('limit')
        if not math.isfinite(value) or not float(limit.get('lower')) <= value <= float(limit.get('upper')):
            raise ValueError('Mapped angle outside model limits')
        values[entry['urdf_joint']] = value
    # Reuse existing offline FK only after explicit model/mapping evidence gate.
    sys.path.insert(0, str(CONTROL/'ros2_ws/src/d1_pick_place'))
    from d1_pick_place.kinematics import forward
    pose = forward(urdf, values, tip='Link6')
    if not np.isfinite(pose).all():
        raise ValueError('Invalid FK result')
    return pose


def export_samples(args):
    import cv2
    board = json.loads(args.board.read_text())
    verified_evidence(board, args.board.parent)
    cols, rows, square = board['inner_columns'], board['inner_rows'], board['square_size_m']
    if (type(cols) is not int or type(rows) is not int or not 3 <= cols <= 30 or not 3 <= rows <= 30
            or type(square) not in (int, float) or not math.isfinite(square) or not .001 < square < .1
            or board.get('fixed_board_and_rigid_camera_confirmed') is not True):
        raise ValueError('Measured board geometry and fixed mounting required')
    if len(set(p.resolve() for p in args.sessions)) != len(args.sessions):
        raise ValueError('Duplicate session directories')
    if args.output.exists():
        raise ValueError('Output already exists')
    paired = []
    serials = set()
    for session in args.sessions:
        record = json.loads((session/'capture.json').read_text())
        if record.get('complete') is not True:
            raise ValueError('Incomplete capture: ' + str(session))
        expected = {'feedback.jsonl', 'camera/report.json', 'camera/color.jpg', 'camera/depth_u16le.raw.gz'}
        if set(record.get('raw_sha256', {})) != expected:
            raise ValueError('Missing raw evidence hashes')
        for path, sha in record['raw_sha256'].items():
            if digest(session/path) != sha:
                raise ValueError('Capture evidence changed: ' + path)
        events = []
        for line in (session/'feedback.jsonl').read_text().splitlines():
            raw = json.loads(line)
            e = parse_feedback(raw['line'], raw['t'])
            if e:
                events.append(e)
        obs = record['observation']
        begin, end = obs['begin_aipc_monotonic_s'], obs['end_aipc_monotonic_s']
        window = record['capture_window']
        if not (begin+PRE_SECONDS <= window['request_begin_aipc_monotonic_s']
                < window['completion_aipc_monotonic_s'] <= end-POST_SECONDS):
            raise ValueError('Capture not enclosed by stationary observations')
        checked = assess_window(events, begin, end)
        pose = mapped_pose(args.mapping, checked['joint_median'])
        report = json.loads((session/'camera/report.json').read_text())
        validate_camera_report(report)
        serials.add(report['device']['serial'])
        prepare(session/'camera')
        image = cv2.imread(str(session/'camera/snapshot_color.png'))
        found, _ = cv2.findChessboardCornersSB(image, (cols, rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            raise ValueError('Full specified chessboard not found: ' + str(session))
        paired.append((session.resolve()/'camera/snapshot.yml', pose))
    if len(serials) != 1 or serials != {board.get('camera_serial')}:
        raise ValueError('Camera serial mismatch or empty samples')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Mode x reserves the destination; never overwrite a previous sample set.
    with args.output.open('x'):
        pass
    fs = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    fs.write('translation_unit', 'm')
    fs.write('board_columns', cols)
    fs.write('board_rows', rows)
    fs.write('square_size_m', float(square))
    fs.write('base_frame', 'base_link')
    fs.write('end_frame', 'Link6')
    fs.write('mapping_sha256', digest(args.mapping))
    fs.write('board_config_sha256', digest(args.board))
    fs.startWriteStruct('samples', cv2.FileNode_SEQ)
    for snapshot, pose in paired:
        fs.startWriteStruct('', cv2.FileNode_MAP)
        fs.write('snapshot', str(snapshot))
        fs.write('T_base_end', pose)
        fs.endWriteStruct()
    fs.endWriteStruct()
    fs.release()
    print(f'Exported {len(paired)} measured samples; not a solved or validated hand-eye transform.')
    return 0


def doctor(args):
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'robot_motion_commands_sent': False, 'network_modified': False,
              'handeye_calibrated': False}
    for name, cmd in [
            ('addresses', ['ip', '-brief', 'address']), ('routes', ['ip', 'route']),
            ('ppp_service', ['systemctl', 'is-active', 'kr260-ppp-host.service'])]:
        report[name] = command(cmd)
    try:
        report['d1_addresses'] = check_link(args.nic)
        feedback = command([str(CONTROL/'build/arm_control'), '--nic', args.nic, '--verbose', 'watch', '3'])
        (args.output/'d1_feedback.log').write_text(feedback['stdout'] + feedback['stderr'])
        report['d1_observer_returncode'] = feedback['returncode']
        frames = [e for line in feedback['stdout'].splitlines() if (e := parse_feedback(line, 0))]
        angles = [e for e in frames if e['kind'] == 'angles']
        states = [e for e in frames if e['kind'] == 'status']
        report['d1_angle_frames'] = len(angles)
        report['d1_latest_joints'] = angles[-1]['q'] if angles else None
        report['d1_latest_power_enable_error'] = states[-1]['state'] if states else None
        report['d1_drive_ready'] = bool(angles and states and states[-1]['state'] == [1, 1, 0])
    except (ValueError, OSError) as exc:
        report['d1_blocker'] = str(exc)
    remote = (
        'hostname; systemctl is-active kr260-ppp.service; '
        'cat /sys/class/fpga_manager/fpga0/state; '
        'ls /sys/kernel/config/device-tree/overlays; lsusb; '
        'pgrep -af ' + shlex.quote(CAMERA_PROCESS_PATTERN) + '; '
        'sha256sum /home/ubuntu/kr260-camera/capture_orbbec.py '
        '/home/ubuntu/kr260-camera/run_camera.sh /home/ubuntu/kr260-camera/run_depth_gate.py; '
        'sudo -n true')
    report['kr260'] = command(['ssh', *ssh_options(args), args.host, remote])
    report['note'] = 'FPGA operating/USB enumeration do not constitute a new frame-processing test. sudo -n may require interactive authentication.'
    save(args.output/'doctor.json', report)
    print(json.dumps(report, indent=2))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--host', default='ubuntu@192.168.28.2')
    parser.add_argument('--control-path', default=str(CAMERA/'ssh-camera'))
    sub = parser.add_subparsers(dest='mode', required=True)
    for name in ('doctor', 'capture'):
        p = sub.add_parser(name)
        p.add_argument('--output', required=True, type=Path)
    p = sub.add_parser('export')
    p.add_argument('--mapping', required=True, type=Path)
    p.add_argument('--board', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('sessions', nargs='+', type=Path)
    args = parser.parse_args()
    try:
        return {'doctor': doctor, 'capture': capture, 'export': export_samples}[args.mode](args)
    except (ValueError, RuntimeError, OSError, KeyError, subprocess.TimeoutExpired) as exc:
        print(f'Not completed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
