#!/usr/bin/env python3
"""Re-runnable PiPER integration checks against the running Gazebo console.

This moves only the PiPER simulation, changes its gripper, and leaves the arm
open at ready on success. It never starts services, calls real hardware, or
runs the automatic grasp/place sequence. Run after any grasp/place work ends.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
DELTAS = [.06, -.06, .06, .06, -.06, .06]


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def jpeg_size(data):
    """Read the actual JPEG SOF dimensions without a Pillow dependency."""
    if data[:2] != b'\xff\xd8':
        raise AssertionError('Camera endpoint did not return JPEG bytes')
    index = 2
    sof = {0xc0, 0xc1, 0xc2, 0xc3, 0xc5, 0xc6, 0xc7, 0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf}
    while index < len(data):
        if data[index] != 0xff:
            index += 1
            continue
        while index < len(data) and data[index] == 0xff:
            index += 1
        if index >= len(data):
            break
        marker = data[index]
        index += 1
        if marker in {0xd8, 0xd9, 0x01} or 0xd0 <= marker <= 0xd7:
            continue
        if index + 2 > len(data):
            break
        length = int.from_bytes(data[index:index + 2], 'big')
        if length < 2 or index + length > len(data):
            break
        if marker in sof and length >= 8:
            height = int.from_bytes(data[index + 3:index + 5], 'big')
            width = int.from_bytes(data[index + 5:index + 7], 'big')
            return width, height
        if marker == 0xda:
            break
        index += length
    raise AssertionError('JPEG has no readable SOF dimensions')


class Verification:
    def __init__(self, base_url, timeout):
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.robot = json.loads((ROOT / 'config/robot.json').read_text())
        self.scene = json.loads((ROOT / 'config/grasp_scene.json').read_text())
        self.model = json.loads((ROOT / 'generated/model.json').read_text())
        self.arm = self.robot['arm_joints']
        self.fingers = self.robot['finger_joints']
        self.joints = self.arm + self.fingers
        self.results = []
        self.motion_started = False

    def get(self, path):
        with urllib.request.urlopen(self.base_url + path, timeout=5) as response:
            return response.read(), response.headers

    def state(self):
        data, _ = self.get('/api/state')
        return json.loads(data)

    def command(self, data):
        request = urllib.request.Request(self.base_url + '/api/control',
                                        data=json.dumps(data).encode(),
                                        headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=8) as response:
            return json.load(response)

    def reject(self, data, name):
        before = self.state()['targets']
        try:
            self.command(data)
        except urllib.error.HTTPError as error:
            body = json.loads(error.read())
            assert error.code == 400, (name, error.code, body)
        else:
            raise AssertionError(name + ': invalid request was accepted')
        after = self.state()['targets']
        assert before == after, name + ': rejected request changed the targets'
        self.results.append({'check': name, 'passed': True, 'http_status': 400,
                             'error': body.get('error')})

    def settle(self, target, finger_tolerance=.002):
        deadline = time.monotonic() + self.timeout
        stable_since = None
        last = None
        while time.monotonic() < deadline:
            last = self.state()
            if last.get('collision_warning'):
                raise AssertionError(last['collision_warning'])
            ready = last.get('connected') and all(
                name in last['positions'] and math.isfinite(last['positions'][name])
                and abs(last['positions'][name] - value)
                < (finger_tolerance if name in self.fingers else .015)
                for name, value in target.items())
            if ready:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= .4:
                    return last
            else:
                stable_since = None
            time.sleep(.12)
        raise AssertionError({'message': 'Actual feedback did not settle',
                              'expected': target, 'state': last})

    def gripper_target(self, fraction):
        closed = self.robot.get('finger_closed', [0.] * len(self.fingers))
        return {name: opened + (shut - opened) * fraction
                for name, opened, shut in zip(self.fingers, self.robot['finger_open'], closed)}

    def run(self, d1_root):
        piper_manifest = ROOT / 'config/environment.json'
        d1_manifest = d1_root / 'config/environment.json'
        piper_bytes, d1_bytes = piper_manifest.read_bytes(), d1_manifest.read_bytes()
        assert piper_bytes == d1_bytes, 'PiPER and D1 environment manifests differ byte-for-byte'
        self.results.append({'check': 'environment_manifest_byte_identity', 'passed': True,
                             'piper_path': str(piper_manifest), 'd1_path': str(d1_manifest),
                             'sha256': hashlib.sha256(piper_bytes).hexdigest()})

        initial = self.state()
        assert len(self.arm) == 6 and len(self.fingers) == 2
        assert initial.get('mode', '').startswith('Gazebo'), 'Endpoint is not the Gazebo simulation'
        assert initial.get('connected'), 'Gazebo joint feedback is not connected'
        assert all(name in initial['positions'] and math.isfinite(initial['positions'][name])
                   for name in self.joints), 'Expected eight finite PiPER joint positions'
        assert not initial.get('grasp', {}).get('active'), 'Finish automatic grasp/place before testing'
        cube = initial.get('cube', {})
        carried = (len(cube.get('finger_contacts', [])) == 2 and cube.get('pose')
                   and cube['pose']['position'][2] > self.scene['cube']['position'][2] + .04)
        assert not carried, 'Place the held cube before testing; opening now would drop it'
        self.results.append({'check': 'eight_joint_feedback_online', 'passed': True,
                             'joint_names': self.joints, 'age_s': initial.get('age')})

        self.command({'action': 'resume'})
        self.motion_started = True
        self.command({'action': 'ready'})
        ready = dict(zip(self.arm, self.robot['ready']))
        ready.update(self.gripper_target(0.))
        self.settle(ready)

        combined = {name: value + delta for name, value, delta in
                    zip(self.arm, self.robot['ready'], DELTAS)}
        self.command({'values': combined})
        reached = self.settle(combined)
        axis_results = [{'joint': name, 'delta_rad': delta, 'target_rad': combined[name],
                         'actual_rad': reached['positions'][name],
                         'absolute_error_rad': abs(reached['positions'][name] - combined[name])}
                        for name, delta in zip(self.arm, DELTAS)]
        assert all(row['absolute_error_rad'] < .015 for row in axis_results)
        self.results.append({'check': 'six_axis_combined_motion', 'passed': True,
                             'tolerance_rad': .015, 'axes': axis_results})
        self.command({'action': 'ready'})
        self.settle(ready)
        self.results.append({'check': 'return_to_ready', 'passed': True})

        # Use the same verified empty-gripper closing fraction as grasping.
        # The official zero-gap finger meshes meet at 100%; requesting a self-
        # collision would test guard rejection rather than gripper actuation.
        above = dict(zip(self.arm, self.scene['poses']['above']))
        self.command({'values': {**above, 'gripper': 0.}})
        self.settle({**above, **self.gripper_target(0.)})
        fraction = float(self.scene['grip_closed_target'])
        assert 0. < fraction <= 1.
        self.command({'values': {'gripper': fraction}})
        closed = self.settle(self.gripper_target(fraction))
        self.results.append({'check': 'gripper_close_above', 'passed': True,
                             'fraction': fraction, 'targets_m': self.gripper_target(fraction),
                             'actual_m': {name: closed['positions'][name] for name in self.fingers}})
        self.command({'values': {'gripper': 0.}})
        opened = self.settle(self.gripper_target(0.))
        self.results.append({'check': 'gripper_open_above', 'passed': True,
                             'actual_m': {name: opened['positions'][name] for name in self.fingers}})

        first = self.arm[0]
        joint = next(item for item in self.model['joints'] if item['name'] == first)
        limit = joint['limits']['upper']
        self.reject({'values': {first: limit + 1.}}, 'out_of_range_angle_rejected')
        self.reject({'values': {first: float('nan')}}, 'NaN_rejected')

        self.command({'action': 'hold'})
        held = self.state()
        assert held['hold']
        before = held['positions']
        time.sleep(.5)
        after = self.state()['positions']
        drift = max(abs(before[name] - after[name]) for name in self.arm)
        assert drift < .015, ('Arm drift while held', drift)
        self.reject({'action': 'home'}, 'hold_blocks_motion')
        self.command({'action': 'resume'})
        assert not self.state()['hold']
        self.command({'action': 'ready'})
        self.settle(ready)
        self.results.append({'check': 'hold_resume_and_ready', 'passed': True,
                             'maximum_arm_drift_rad': drift})

        self.verify_camera()
        return self.state()

    def verify_camera(self):
        observations = []
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline:
            data, headers = self.get('/api/wrist_camera.jpg?t=' + str(time.monotonic_ns()))
            assert headers.get_content_type() == 'image/jpeg'
            assert jpeg_size(data) == (640, 480), jpeg_size(data)
            frame = int(headers['X-Camera-Frame'])
            age = float(headers['X-Camera-Age'])
            assert math.isfinite(age) and 0. <= age <= 2., age
            observation = {'frame_id': frame, 'age_s': age, 'width': 640, 'height': 480,
                           'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
            if not observations:
                observations.append(observation)
            elif frame > observations[0]['frame_id']:
                observations.append(observation)
                break
            time.sleep(.2)
        assert len(observations) == 2, 'JPEG frames did not advance'
        camera = self.state()['wrist_camera']
        assert camera['connected'] and camera['age'] <= 2.
        camera_config = json.loads((ROOT / 'config/wrist_camera.json').read_text())
        assert camera['topic'] == camera_config['sensor']['topic']
        self.results.append({'check': 'live_640x480_jpeg', 'passed': True,
                             'topic': camera['topic'], 'frames': observations})


def main():
    runtime = json.loads((ROOT / 'config/runtime.json').read_text())
    port = int(os.environ.get('PIPER_PORT', runtime['simulation_port']))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-url', default=f'http://127.0.0.1:{port}')
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--d1-root', type=Path, default=ROOT.parent / 'd1t_sim')
    parser.add_argument('--output', type=Path, default=ROOT / 'verification.json')
    args = parser.parse_args()
    parsed = urllib.parse.urlparse(args.base_url)
    if parsed.scheme != 'http' or parsed.hostname not in {'127.0.0.1', 'localhost', '::1'}:
        parser.error('This simulation test accepts only a local HTTP endpoint')
    verification = Verification(args.base_url, args.timeout)
    report = {'passed': False, 'simulation_only': True, 'base_url': args.base_url,
              'started_at': datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),
              'results': verification.results}
    exit_code = 1
    try:
        report['state'] = verification.run(args.d1_root)
        report['passed'] = True
        exit_code = 0
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        if verification.motion_started:
            try:
                verification.command({'action': 'hold'})
                report['failure_cleanup'] = 'Held the PiPER simulation'
            except Exception as cleanup_error:
                report['failure_cleanup'] = str(cleanup_error)
        try:
            report['state'] = verification.state()
        except Exception as state_error:
            report['state_error'] = str(state_error)
    finally:
        report['finished_at'] = datetime.now(ZoneInfo('Asia/Singapore')).isoformat()
        args.output.write_text(json.dumps(json_safe(report), ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        print(json.dumps({'passed': report['passed'], 'checks': len(verification.results),
                          'output': str(args.output), 'error': report.get('error')}, ensure_ascii=False))
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
