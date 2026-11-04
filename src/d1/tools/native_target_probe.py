#!/usr/bin/env python3
"""Reviewed, bounded native J0 positioning probe; default is offline plan only.

No MoveIt, shared-phase executor, homing, grip, compensation, retry or unload.
Position measurements describe SDK encoder feedback, not physical accuracy.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / 'runtime/Control'
LIMITS = ((-135, 135), (-90, 90), (-90, 90), (-135, 135), (-90, 90), (-135, 135), (0, 65))
JOINT = 0
EXPECTED = [-4.8, -89.9, 88.2, -.6, 11.2, .6, 36.2]
PARAMETERS = dict(joint=JOINT, amplitude_deg=2., hold_s=4., tail_s=2., delay_ms=1000,
                  start_tolerance_deg=.3, feedback_tolerance_deg=.2,
                  feedback_max_age_s=.6, tracking_guard_deg=3.,
                  non_target_guard_deg=.5, minimum_command_distance_deg=.2,
                  max_position_commands=4, max_power_commands=1,
                  endpoint_assessment_deg=.3, task_accuracy_deg=1.,
                  endpoint_stability_span_deg=.3, stability_duration_s=1.)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bindings(binary):
    paths = [Path(__file__).resolve(), Path(binary).resolve(),
             CONTROL/'tools/handeye_joint_step.py', CONTROL/'tools/j4_smooth_test.py',
             CONTROL/'tools/joint_mapping_small_test.py']
    paths += sorted((CONTROL/'ros2_ws/src/d1_pick_place/d1_pick_place').glob('*.py'))
    return {str(path.resolve()): file_hash(path) for path in paths}


def make_review(expected, binary, nic, allow_power_on):
    expected = list(map(float, expected))
    if len(expected) != 7 or not all(math.isfinite(x) for x in expected):
        raise ValueError('Exactly seven finite reviewed start angles required')
    for i, (value, (lo, hi)) in enumerate(zip(expected, LIMITS)):
        tolerance = .2 if i < 6 else 0.
        if not lo-tolerance <= value <= hi+tolerance:
            raise ValueError(f'Reviewed J{i} outside original feedback limits')
    q0 = expected[JOINT]
    if not all(LIMITS[JOINT][0] <= q0+d <= LIMITS[JOINT][1] for d in (2., -2.)):
        raise ValueError('J0 target exceeds original command limits')
    result = dict(schema='d1_native_J0_position_probe_v2', expected_angles=expected,
                  expected_status=dict(power_status=0 if allow_power_on else 1,
                                       enable_status=[0, 1], error_status=0),
                  binary=str(Path(binary).resolve()), nic=nic,
                  allow_power_on=bool(allow_power_on), parameters=dict(PARAMETERS),
                  targets_deg=[q0+2, q0, q0-2, q0],
                  command_shape='native j 0 <angle> 1000; one submission per target; no cover test',
                  camera_condition=dict(global_serial='CC1365302K0', global_port=18761,
                                        wrist_serial='CC1365301EY', wrist_port=18760,
                                        max_board_age_ms=250, max_capture_age_s=.8,
                                        capture_before_power=True, scene_review_required=True),
                  source_shape=dict(group_submission=False, trajectory_cache_used=False,
                                    power_on='one dedicated noninteractive CLI process; fixed power on only; stdin closed',
                                    motion_scope='persistent CLI --joints 0 only',
                                    position_start='CLI bootstrap once if enable=0; normal position if enable=1',
                                    native_delay='requested value; device may transform it',
                                    ack='recv confirms protocol receipt, not arrival',
                                    zero_distance='skip if encoded equal or distance <= reviewed minimum + 1e-5',
                                    failure='stop future commands; no unload or automatic recovery'),
                  measurements='SDK encoder residual only; independent physical accuracy unverified',
                  sha256_bindings=bindings(binary))
    result['review_sha256'] = hashlib.sha256(canonical(result)).hexdigest()
    return result


def verify_review(review):
    content = {k: v for k, v in review.items() if k != 'review_sha256'}
    if hashlib.sha256(canonical(content)).hexdigest() != review.get('review_sha256'):
        raise RuntimeError('Review SHA mismatch')
    if review.get('schema') != 'd1_native_J0_position_probe_v2' or review.get('parameters') != PARAMETERS:
        raise RuntimeError('Unsupported or changed reviewed method')
    if review.get('sha256_bindings') != bindings(review['binary']):
        raise RuntimeError('Code or binary changed after review; no further command')
    expected = make_review(review['expected_angles'], review['binary'], review['nic'], review['allow_power_on'])
    if expected != review:
        raise RuntimeError('Review does not match the fixed native probe shape')


def consume_review(review, approval_text, output, registry=None):
    """Atomically consume this review SHA before any camera or robot operation.

    One registry per probe tool prevents replay by renaming/copying the review.
    A failed run, including camera startup failure, still consumes its review.
    """
    if not isinstance(approval_text, str) or not approval_text.strip():
        raise ValueError('Nonempty approval text required')
    registry = Path(registry) if registry is not None else ROOT/'validation/native_probe_consumed'
    registry.mkdir(parents=True, exist_ok=True)
    marker = registry/(review['review_sha256']+'.json')
    claim = dict(review_sha256=review['review_sha256'], approval_text=approval_text,
                 approval_text_sha256=hashlib.sha256(approval_text.encode()).hexdigest(),
                 output=str(Path(output).resolve()), consumed_wall_time=time.time(),
                 replay_allowed=False)
    try:
        with marker.open('x') as handle:
            json.dump(claim, handle, ensure_ascii=False, indent=2)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
    except FileExistsError as error:
        raise RuntimeError('Review already consumed; replay prohibited') from error
    return marker, claim


def checked_snapshot(snapshot, now, baseline, target=None, *, power_required=True,
                     enable_required=False):
    if snapshot.get('fault'):
        raise RuntimeError(str(snapshot['fault']))
    q = snapshot.get('q')
    if q is None or len(q) != 7:
        raise RuntimeError('Missing complete feedback')
    q = [float(x) for x in q]
    times = snapshot.get('joint_q_time', [snapshot.get('q_time', -math.inf)]*7)
    pending = snapshot.get('joint_feedback_pending', [False]*7)
    if len(times) != 7 or len(pending) != 7:
        raise RuntimeError('Invalid per-axis feedback metadata')
    if any(not math.isfinite(float(t)) or not 0 <= now-float(t) <= .6 for t in times) or any(pending):
        raise RuntimeError('Stale or rejected joint feedback')
    for i, (value, (lo, hi)) in enumerate(zip(q, LIMITS)):
        tolerance = .2 if i < 6 else 0.
        if not math.isfinite(value) or not lo-tolerance-1e-5 <= value <= hi+tolerance+1e-5:
            raise RuntimeError(f'J{i} feedback outside original limits')
    status, stamps = snapshot.get('status', {}), snapshot.get('status_time', {})
    for key in ('power_status', 'enable_status', 'error_status'):
        if type(status.get(key)) is not int or status[key] not in (0, 1):
            raise RuntimeError('Missing/invalid '+key)
        if not 0 <= now-stamps.get(key, -math.inf) <= .6:
            raise RuntimeError('Stale '+key)
    if status['error_status'] != 0 or (power_required and status['power_status'] != 1):
        raise RuntimeError('Power/error state invalid')
    if enable_required and status['enable_status'] != 1:
        raise RuntimeError('Position enable not confirmed')
    if baseline is not None and any(abs(q[i]-baseline[i]) > .5+1e-5 for i in range(7) if i != JOINT):
        raise RuntimeError('Non-target joint drift > 0.5')
    if target is not None and abs(q[JOINT]-target) > 3.+1e-5:
        raise RuntimeError('J0 tracking error > 3 degrees')
    return q


def endpoint_metrics(samples, started, ended, target):
    observed = [s for s in samples if started <= s['t'] <= ended]
    tail = [s['q'][JOINT] for s in observed if s['t'] >= ended-2.]
    median = statistics.median(tail) if tail else None
    def stable_time(tolerance):
        for index, sample in enumerate(observed):
            if ended-sample['t'] >= 1. and all(abs(s['q'][JOINT]-target) <= tolerance+1e-5 for s in observed[index:]):
                return sample['t']-started
        return None
    stable_at = stable_time(.3)
    task_stable_at = stable_time(1.)
    tail_stable = len(tail) >= 10 and max(tail)-min(tail) <= .3+1e-5
    return dict(target_deg=target, observation_s=ended-started, tail_s=2., samples=len(tail),
                encoder_median_deg=median,
                encoder_signed_residual_deg=None if median is None else median-target,
                encoder_peak_to_peak_deg=max(tail)-min(tail) if tail else None,
                stable_to_end_after_s=stable_at,
                task_1deg_stable_to_end_after_s=task_stable_at,
                adequate_samples=len(tail) >= 10,
                tail_stable=tail_stable,
                assessment_0_3deg_pass=bool(tail_stable and stable_at is not None),
                task_1deg_pass=bool(tail_stable and task_stable_at is not None),
                physical_accuracy_verified=False)


def feedback_quality(path):
    """Offline raw-feedback availability and UART diagnostic measurements.

    Host receive intervals are not device sample intervals. No UART diagnostics
    means unavailable integrity evidence, never zero protocol errors.
    """
    path = Path(path)
    times, valid, scans, writes = [], [], [], []
    invalid_frames = malformed = 0
    for line in path.read_text().splitlines():
        try:
            row = json.loads(line)
            stamp = float(row['t'])
            if not math.isfinite(stamp):
                continue
            times.append(stamp)
            text = row.get('line', '')
            if '[rx] ' not in text:
                continue
            message = json.loads(text.split('[rx] ', 1)[1])
            if message.get('address') != 2:
                continue
            data = message.get('data', {})
            if message.get('funcode') == 1:
                angles = [data.get(f'angle{i}') for i in range(7)]
                accepted = all(type(v) in (int, float) and math.isfinite(v)
                               and lo-(.2 if i < 6 else 0)-1e-5 <= v <= hi+(.2 if i < 6 else 0)+1e-5
                               for i, (v, (lo, hi)) in enumerate(zip(angles, LIMITS)))
                if accepted:
                    valid.append(stamp)
                else:
                    invalid_frames += 1
            if message.get('funcode') == 90 and data.get('schema') == 'd1_uart_integrity_v1':
                if data.get('event') == 'feedback_scan':
                    scans.append(data)
                elif data.get('event') in ('command_write', 'command_rejected'):
                    writes.append(dict(host_receive_monotonic_s=stamp, **data))
        except (ValueError, KeyError, TypeError):
            malformed += 1
    intervals = [b-a for a, b in zip(valid, valid[1:])]
    ordered = sorted(intervals)
    p95 = ordered[math.ceil(.95*len(ordered))-1] if ordered else None
    bounds = [min(times), max(times)] if times else [None, None]
    edges = ([bounds[0]]+valid+[bounds[1]]) if valid else []
    stale = [dict(start_monotonic_s=a+.6, end_monotonic_s=b, duration_s=b-a-.6)
             for a, b in zip(edges, edges[1:]) if b-a > .6]
    if not valid and times and bounds[1]-bounds[0] > .6:
        stale = [dict(start_monotonic_s=bounds[0]+.6, end_monotonic_s=bounds[1],
                      duration_s=bounds[1]-bounds[0]-.6)]
    counters = {}
    for key in ('queries_total', 'queries_invalid', 'valid_frames', 'dropped_frames'):
        values = [scan[key] for scan in scans if type(scan.get(key)) is int]
        reset = any(b < a for a, b in zip(values, values[1:]))
        counters[key] = dict(first=values[0] if values else None, last=values[-1] if values else None,
                             delta=values[-1]-values[0] if len(values) >= 2 and not reset else None,
                             reset_detected=reset)
    status_keys = sorted({key for scan in scans for key in scan.get('invalid_by_status', {})})
    invalid_by_status = {}
    for key in status_keys:
        values = [scan.get('invalid_by_status', {}).get(key, 0) for scan in scans]
        if all(type(value) is int for value in values):
            reset = any(b < a for a, b in zip(values, values[1:]))
            invalid_by_status[key] = dict(first=values[0], last=values[-1],
                                          delta=values[-1]-values[0] if len(values)>1 and not reset else None,
                                          reset_detected=reset)
    return dict(source=str(path.resolve()), clock='host raw-message receive monotonic time',
                valid_feedback_count=len(valid), invalid_feedback_count=invalid_frames,
                malformed_log_or_message_count=malformed,
                observation_s=bounds[1]-bounds[0] if times else 0.,
                valid_feedback_rate_hz=(len(valid)-1)/(valid[-1]-valid[0]) if len(valid)>1 and valid[-1]>valid[0] else None,
                interval_p95_s=p95, interval_max_s=max(intervals) if intervals else None,
                stale_threshold_s=.6, stale_intervals=stale, stale_interval_count=len(stale),
                stale_total_s=sum(item['duration_s'] for item in stale),
                uart_integrity_available=bool(scans), uart_scan_diagnostics_count=len(scans),
                uart_counters=counters, protocol_invalid_count=counters['queries_invalid']['delta'],
                uart_invalid_by_status=invalid_by_status, uart_last_scan=scans[-1] if scans else None,
                command_uart_events=writes,
                command_event_semantics='UART write diagnostics do not prove servo adoption or arrival')


def quality_comparison(before, after):
    keys = ('valid_feedback_count', 'invalid_feedback_count', 'valid_feedback_rate_hz',
            'interval_p95_s', 'interval_max_s', 'stale_interval_count', 'stale_total_s',
            'protocol_invalid_count')
    return dict(scope='Feedback availability/integrity only; different joints or observation lengths are not positioning comparisons',
                metrics={key: dict(before=before.get(key), after=after.get(key),
                                   change=after[key]-before[key] if type(before.get(key)) in (int,float)
                                   and type(after.get(key)) in (int,float) else None) for key in keys})


def power_on_argv(binary, nic):
    """The CLI has no power-only flag. A closed-input direct-command process
    gets the global scope required by funcode 6, and has exactly one command.
    The persistent motion CLI retains --joints 0 throughout.
    """
    return [str(binary), '--nic', nic, '--joints', 'all', '--ack', 'recv',
            '--timeout-ms', '1500', '--verbose', 'power', 'on']


class PowerOnSession:
    def __init__(self, binary, nic, output, now=time.monotonic):
        self.now = now
        self.log = Path(output).open('x')
        self.lock = threading.RLock()
        self.finished = False
        self.fault = None
        self.tx = 0
        self.received = 0
        self.rejected = 0
        self.ok = False
        self.sequence = None
        self.started = now()
        self.proc = subprocess.Popen(power_on_argv(binary, nic), stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for raw in self.proc.stdout:
                line = raw.rstrip()
                with self.lock:
                    self.log.write(json.dumps(dict(t=self.now(), kind='output', line=line))+'\n')
                    self.log.flush()
                    if line.startswith('[tx] '):
                        message = json.loads(line[5:])
                        if (message.get('address') != 1 or message.get('funcode') != 6
                                or message.get('data') != {'power': 1}):
                            raise RuntimeError('Unexpected command in power-only session')
                        self.sequence = message['seq']
                        self.tx += 1
                        if self.tx > 1:
                            raise RuntimeError('Power-only session attempted more than one publish')
                    if '[rx] ' in line:
                        message = json.loads(line.split('[rx] ', 1)[1])
                        if (message.get('address') == 3 and message.get('funcode') == 1
                                and message.get('seq') == self.sequence):
                            if message.get('data', {}).get('recv_status') == 1:
                                self.received = 1
                            else:
                                self.rejected = 1
                    if line.startswith('[ok] '):
                        self.ok = True
                    if line.startswith(('[参数错误]', '[未发送]', '[设备拒绝]', '[失败]',
                                        '[接收超时]', '[DDS写入失败]', '[执行/状态未确认]')):
                        if line.startswith(('[参数错误]', '[未发送]', '[设备拒绝]')):
                            self.rejected = 1
                        self.fault = line
        except Exception as error:
            with self.lock:
                self.fault = str(error)
        finally:
            with self.lock:
                self.finished = True

    def state(self):
        with self.lock:
            return dict(finished=self.finished, fault=self.fault, cli_publish_attempts=self.tx,
                        received=self.received, rejected=self.rejected, ok=self.ok,
                        returncode=self.proc.poll())

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.reader.join(timeout=2.)
        self.proc.stdout.close()
        self.log.close()


class Probe:
    def __init__(self, review, transport, cameras, output, cancel=lambda: False,
                 power_session_factory=PowerOnSession):
        self.review, self.transport, self.cameras = review, transport, cameras
        self.cancel = cancel
        self.output = Path(output)
        self.events = (self.output/'probe.jsonl').open('x')
        self.samples = []
        self.last_sequence = None
        self.baseline = None
        self.target = None
        self.started = False
        self.position_commands = 0
        self.power_commands = 0
        self.position_attempts = 0
        self.power_attempts = 0
        self.position_rejected = 0
        self.power_delivery = dict(attempted=0, sent=0, rejected=0, cli_publish_attempts=0,
                                   sent_definition='matching successful device receive ACK', delivery_unknown=False)
        self.power_session_factory = power_session_factory
        self.phase = 'preflight'
        self.holds = []

    def record(self, kind, **data):
        event = dict(t=self.transport.now(), phase=self.phase, kind=kind)
        event.update(data)
        self.events.write(json.dumps(event)+'\n')
        self.events.flush()

    def check(self, *, power_required=True):
        if self.cancel():
            raise RuntimeError('Cancelled; no future command')
        self.cameras.check()
        snapshot = self.transport.snapshot()
        q = checked_snapshot(snapshot, self.transport.now(), self.baseline, self.target,
                             power_required=power_required, enable_required=self.started)
        sequence = snapshot.get('q_seq')
        if sequence != self.last_sequence:
            self.last_sequence = sequence
            times = list(map(float, snapshot.get('joint_q_time', [snapshot['q_time']]*7)))
            observed_at = self.transport.now()
            sample = dict(t=float(snapshot['q_time']), q=q, sequence=sequence, phase=self.phase,
                          observed_at_monotonic_s=observed_at, joint_receive_times_s=times,
                          joint_age_s=[observed_at-stamp for stamp in times],
                          status=snapshot['status'], status_time=snapshot['status_time'])
            self.samples.append(sample)
            self.record('sample', **sample)
        return snapshot, q

    def observe(self, seconds, *, power_required=True):
        deadline = self.transport.now()+seconds
        while True:
            self.check(power_required=power_required)
            remaining = deadline-self.transport.now()
            if remaining <= 0:
                return
            self.transport.wait(min(.02, remaining))

    def write(self, command, *, position=False, power=False):
        if power:
            if command != 'power on' or position:
                raise RuntimeError('Power adapter only permits power on')
            return self.power_on()
        verify_review(self.review)
        snapshot, measured = self.check(power_required=not power)
        if position and self.position_attempts >= self.review['parameters']['max_position_commands']:
            raise RuntimeError('Reviewed command budget exhausted')
        before, sent = self.transport.acks, self.transport.now()
        self.record('native_command', command=command, position=position, power=power,
                    measured_angles=measured, measured_status=snapshot['status'],
                    time_semantics='host CLI submission, not device application')
        with self.transport.lock:
            self.transport.commands += 1
            self.position_attempts += bool(position)
            self.transport.proc.stdin.write(command+'\n')
            self.transport.proc.stdin.flush()
        deadline = sent+1.5
        try:
            while self.transport.acks == before:
                self.check()
                if self.transport.now() >= deadline:
                    raise RuntimeError('Native receipt/bootstrap confirmation timeout')
                self.transport.wait(.01)
        except Exception:
            fault = self.transport.snapshot().get('fault') or ''
            if fault.startswith(('[参数错误]', '[未发送]', '[设备拒绝]')):
                self.position_rejected += 1
            raise
        if position:
            self.started = True
            self.position_commands += 1
        snapshot, _ = self.check()
        for key in ('power_status', 'error_status'):
            if snapshot['status_time'][key] <= sent:
                # CLI verifies power, and initial position bootstrap verifies all
                # statuses. Demand fresh readback here even for later commands.
                deadline = self.transport.now()+.6
                while snapshot['status_time'][key] <= sent:
                    if self.transport.now() >= deadline:
                        raise RuntimeError('No fresh status readback after native command')
                    self.transport.wait(.01)
                    snapshot, _ = self.check()
        self.record('native_receipt', command=command, elapsed_s=self.transport.now()-sent,
                    confirmation='CLI receive/bootstrap confirmation; not arrival')
        return sent

    def power_on(self):
        verify_review(self.review)
        if not self.review.get('allow_power_on') or self.power_attempts >= 1:
            raise RuntimeError('Power on not reviewed or already attempted')
        snapshot, measured = self.check(power_required=False)
        if snapshot['status']['power_status'] != 0 or snapshot['status']['enable_status'] != 0:
            raise RuntimeError('Power-only session requires reviewed power=0 enable=0')
        sent = self.transport.now()
        self.power_attempts += 1
        self.power_delivery['attempted'] = self.power_attempts
        self.record('power_command_attempt', command='power on', measured_angles=measured,
                    measured_status=snapshot['status'],
                    argv=power_on_argv(self.review['binary'], self.review['nic']),
                    note='dedicated closed-input CLI session; motion CLI remains J0-only')
        session = None
        try:
            session = self.power_session_factory(self.review['binary'], self.review['nic'],
                                                  self.output/'power_session.jsonl', self.transport.now)
            deadline = sent+5.
            while True:
                self.check(power_required=False)
                state = session.state()
                if state['fault']:
                    raise RuntimeError(state['fault'])
                if state['finished']:
                    if not state['ok'] or state['received'] != 1 or state['cli_publish_attempts'] != 1:
                        raise RuntimeError('Power-only CLI did not confirm one received command')
                    break
                if self.transport.now() >= deadline:
                    raise RuntimeError('Power-only session timeout; no automatic retry')
                self.transport.wait(.01)
            state_deadline = self.transport.now()+.6
            while True:
                snapshot, _ = self.check(power_required=False)
                if (snapshot['status']['power_status'] == 1
                        and snapshot['status_time']['power_status'] > sent):
                    break
                if self.transport.now() >= state_deadline:
                    raise RuntimeError('No fresh power=1 readback within 0.6 s after confirmation')
                self.transport.wait(.01)
            self.check()
            self.record('power_readback_confirmed', elapsed_s=self.transport.now()-sent)
            return sent
        finally:
            if session is not None:
                session.close()
                state = session.state()
                self.power_commands = state['received']
                self.power_delivery.update(sent=state['received'], rejected=state['rejected'],
                                           cli_publish_attempts=state['cli_publish_attempts'],
                                           delivery_unknown=state['cli_publish_attempts'] > state['received']+state['rejected'])
                self.record('power_delivery', result=self.power_delivery)

    def target_once(self, name, target, delay):
        self.phase = name
        _, q = self.check()
        target = round(float(target), 3)
        if not LIMITS[JOINT][0] <= target <= LIMITS[JOINT][1]:
            raise RuntimeError('Native target outside original command limits')
        if (abs(target-q[JOINT]) <= self.review['parameters']['minimum_command_distance_deg']+1e-5
                or target == round(q[JOINT], 3)):
            self.record('skipped_zero_distance', target_deg=target, measured_deg=q[JOINT])
            return False
        # Validate against the new target before publishing; no temporary blind
        # interval when replacing the old target.
        checked_snapshot(self.transport.snapshot(), self.transport.now(), self.baseline, target,
                         enable_required=self.started)
        self.target = target
        self.write(f'j {JOINT} {target:.3f} {int(delay)}', position=True)
        return True

    def hold(self, name, target, delay=1000, seconds=4.):
        began = self.transport.now()
        sent = self.target_once(name, target, delay)
        self.observe(seconds)
        result = endpoint_metrics(self.samples, began, self.transport.now(), target)
        observed = [sample for sample in self.samples if began <= sample['t'] <= self.transport.now()]
        result['non_target_max_drift_deg'] = max((abs(sample['q'][i]-self.baseline[i])
            for sample in observed for i in range(7) if i != JOINT), default=0.)
        result.update(phase=name, command_sent=sent)
        self.holds.append(result)
        self.record('hold_result', result=result)

    def run(self):
        # Wait for first complete, healthy readback; any known error ends now.
        deadline = self.transport.now()+4.
        while self.transport.snapshot().get('q') is None:
            if self.cancel() or self.transport.snapshot().get('fault') or self.transport.now() >= deadline:
                raise RuntimeError('No initial feedback; no command')
            self.cameras.check()
            self.transport.wait(.02)
        _, q = self.check(power_required=False)
        if any(abs(a-b) > .3+1e-5 for a, b in zip(q, self.review['expected_angles'])):
            raise RuntimeError('Measured start differs from reviewed start > 0.3')
        self.baseline = q
        self.record('measured_start', angles=q, status=self.transport.snapshot()['status'])
        self.observe(3., power_required=False)
        snap, q = self.check(power_required=False)
        expected_status = self.review['expected_status']
        if (snap['status']['power_status'] != expected_status['power_status']
                or snap['status']['error_status'] != expected_status['error_status']
                or snap['status']['enable_status'] not in expected_status['enable_status']):
            raise RuntimeError('Initial software status differs from review')
        if any(abs(a-b) > .3+1e-5 for a, b in zip(q, self.review['expected_angles'])):
            raise RuntimeError('Start drift before power > 0.3')
        if self.review['allow_power_on']:
            self.phase = 'power_on'
            self.write('power on', power=True)
        powered, _ = self.check()
        self.started = powered['status']['enable_status'] == 1
        q0 = self.review['expected_angles'][JOINT]
        for name, target in [('positive', q0+2), ('return_from_positive', q0),
                             ('negative', q0-2), ('return_from_negative', q0)]:
            self.hold(name, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, help='Write offline review; default does not connect hardware')
    parser.add_argument('--expected-angles', type=float, nargs=7, default=EXPECTED)
    parser.add_argument('--binary', type=Path, default=CONTROL/'host_build/arm_control')
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--allow-power-on', action='store_true')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--review', type=Path)
    parser.add_argument('--approval-text')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--before-log', type=Path, help='Optional old transport log for feedback quality comparison only')
    args = parser.parse_args()
    if not args.execute:
        review = make_review(args.expected_angles, args.binary, args.nic, args.allow_power_on)
        if args.plan:
            args.plan.parent.mkdir(parents=True, exist_ok=True)
            with args.plan.open('x') as handle:
                json.dump(review, handle, ensure_ascii=False, indent=2)
                handle.write('\n')
        print(json.dumps(review, ensure_ascii=False, indent=2))
        return 0
    if args.plan or not args.review or not args.output or not args.approval_text or not args.approval_text.strip():
        parser.error('--execute requires --review, --output, nonempty --approval-text; no --plan')
    review = json.loads(args.review.read_text())
    verify_review(review)
    args.output.mkdir(parents=True, exist_ok=False)
    consumed_path, approval = consume_review(review, args.approval_text, args.output)
    (args.output/'review.json').write_text(json.dumps(review, ensure_ascii=False, indent=2)+'\n')
    approval['consumed_marker'] = str(consumed_path)
    (args.output/'approval.json').write_text(json.dumps(approval, ensure_ascii=False, indent=2)+'\n')
    sys.path.insert(0, str(CONTROL/'tools'))
    sys.path.insert(0, str(CONTROL/'ros2_ws/src/d1_pick_place'))
    from handeye_joint_step import Cameras
    from d1_pick_place.sdk_executor import CliTransport
    cancelled = [False]
    def stop(_signum, _frame):
        cancelled[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    cameras = Cameras(args.output/'frames')
    transport = probe = None
    summary = dict(completed=False, physical_accuracy_verified=False,
                   completion_semantics='Experiment sequence completed; does not imply precision passed')
    try:
        deadline = time.monotonic()+5.
        while not cameras.ready:
            if cancelled[0]:
                raise RuntimeError('Cancelled before camera health')
            if cameras.error or time.monotonic() >= deadline:
                cameras.check()
            time.sleep(.02)
        cameras.check()
        verify_review(review)
        transport = CliTransport(review['binary'], review['nic'], args.output/'transport.jsonl',
                                 extra_args=('--joints', str(JOINT)), allow_position_start=True)
        probe = Probe(review, transport, cameras, args.output, cancel=lambda: cancelled[0])
        probe.run()
        summary['completed'] = True
    except (Exception, KeyboardInterrupt) as error:
        summary['error'] = str(error) or type(error).__name__
    finally:
        if probe:
            summary.update(position_commands=probe.position_commands, power_commands=probe.power_commands,
                           position_attempts=probe.position_attempts, position_rejected=probe.position_rejected,
                           position_confirmation_unknown=max(0, probe.position_attempts-probe.position_commands-probe.position_rejected),
                           power_delivery=probe.power_delivery,
                           position_commands_semantics='CLI receive/bootstrap confirmations; stdin attempts counted separately',
                           joint=JOINT, holds=probe.holds,
                           all_holds_task_1deg_pass=len(probe.holds) == 4 and all(h['task_1deg_pass'] for h in probe.holds),
                           all_holds_assessment_0_3deg_pass=len(probe.holds) == 4 and all(h['assessment_0_3deg_pass'] for h in probe.holds),
                           final_angles=None if transport.q is None else list(map(float, transport.q)),
                           final_phase=probe.phase, cancelled=cancelled[0])
            probe.events.close()
        if transport:
            transport.close()
        cameras.close()
        summary['camera_error'] = cameras.error
        raw_log = args.output/'transport.jsonl'
        if raw_log.exists():
            try:
                after = feedback_quality(raw_log)
                summary['feedback_quality_after'] = after
                if args.before_log:
                    before = feedback_quality(args.before_log)
                    summary['feedback_quality_before'] = before
                    summary['feedback_quality_comparison'] = quality_comparison(before, after)
            except Exception as error:
                summary['feedback_quality_error'] = str(error)
        (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
