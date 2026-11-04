#!/usr/bin/env python3
"""Prepare one reviewed J5 recovery: mode5 once, J5=69 then68; default no I/O.

Only 0x151/0101050000000000 and two 0x157 joint5/6 pairs can be sent.
J6 is copied from each fresh feedback packet. No gripper, J1-J4, enable,
disable, reset or zero request exists. Failure sends nothing further and does
not guarantee physical stopping or continued support of the held object.
"""

import argparse
from contextlib import ExitStack
import fcntl
import json
import math
from pathlib import Path
import select
import signal
import socket
import struct
import time

from reset_piper_once import (CONTROL_IDS, MOTOR_IDS, POSE_IDS, STATUS_ID,
    ResetCheckError, ResetMonitor, load_helpers, verify_endpoint)
from preflight_j_hold import FatalCheckError, check_fresh_faults


MODE = bytes.fromhex('0101050000000000')
OWNER_PATH = '/tmp/piper_policy_executor_can0.lock'
REQUIRED_IDS = (*MOTOR_IDS, *POSE_IDS, STATUS_ID)
CONTROL = (*CONTROL_IDS, 0x471)
APPROVAL_FLAGS = ('explicit_execution_approved', 'master_powered_off',
    'arm_and_held_object_stable', 'hands_clear', 'onsite_power_cutoff_ready')


def validate_review(review, *, execute=False):
    if not isinstance(review, dict) or review.get('operation') != 'recover_j5_boundary_once':
        raise ResetCheckError('Explicit recover_j5_boundary_once review required')
    if review.get('expected_motion_ctrl2') != MODE.hex() or review.get('j5_targets_deg') != [69, 68]:
        raise ResetCheckError('Only exact mode5 and J5 targets69 then68 are permitted')
    if not isinstance(review.get('expected_identity'), dict) or not review['expected_identity']:
        raise ResetCheckError('Pinned existing CAN endpoint identity required')
    pose, bounds = review.get('expected_pose_deg_m'), review.get('joint_bounds_deg')
    if (not isinstance(pose, list) or len(pose) != 7
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in pose)
            or not isinstance(bounds, list) or len(bounds) != 6
            or any(not isinstance(pair, (list, tuple)) or len(pair) != 2
                   or any(type(value) not in (int, float) or not math.isfinite(value) for value in pair)
                   or pair[0] >= pair[1] for pair in bounds)):
        raise ResetCheckError('Seven finite reviewed pose values and six finite bounds required')
    if bounds[4][1] != 70 or not bounds[4][0] < 68 or not 70 < pose[4] <= 70.1:
        raise ResetCheckError('Only reviewed J5 starting feedback in(70,70.1] with hard upper70 is permitted')
    for index, (value, (low, high)) in enumerate(zip(pose[:6], bounds, strict=True)):
        if index != 4 and not low <= value <= high:
            raise ResetCheckError('Other reviewed joints must be inside unchanged bounds')
    if not 0 <= pose[6] <= .1:
        raise ResetCheckError('Reviewed gripper width must remain within[0,0.1]m')
    for key in APPROVAL_FLAGS:
        if type(review.get(key)) is not bool or (execute and review[key] is not True):
            raise ResetCheckError('Explicit boolean approval required: ' + key)
    if not isinstance(review.get('approval_note'), str) or not review['approval_note'].strip():
        raise ResetCheckError('Explicit approval note for this held-object recovery required')


class RecoveryMonitor(ResetMonitor):
    def __init__(self, *, started_at):
        super().__init__(started_at=started_at)
        self.active = False
        self.expected_echoes = []
        self.inside_hard_j5 = False

    def ingest(self, can_id, data, *, source_at, now):
        if can_id in CONTROL:
            if not math.isfinite(source_at) or source_at > now:
                raise FatalCheckError('Invalid control-frame source timestamp')
            if self.active:
                match = next((item for item in self.expected_echoes
                    if item['id'] == hex(can_id) and item['data'] == data.hex()
                    and 'rx_observed_at' not in item
                    and source_at >= item['attempt_at'] - .005), None)
                if match is None:
                    raise FatalCheckError('Unexpected control source during recovery')
                # This is independent-socket bus observation, not firmware ACK.
                match['rx_observed_at'] = source_at
            self.last_control_at = max(self.last_control_at, source_at)
        super().ingest(can_id, data, source_at=source_at, now=now)


def checked_sample(monitor, review, *, now, anchor=None, start=False, after=None):
    check_fresh_faults(monitor, now=now)
    sample = monitor.snapshot(now=now)
    times = [monitor.parts[can_id][1] for can_id in REQUIRED_IDS]
    if any(not 0 <= now-stamp < .1 for stamp in times) or max(times)-min(times) > .04:
        raise FatalCheckError('Feedback exceeded100ms age or40ms skew')
    if not all(sample['enabled']) or any(value & 0xbf for value in sample['motor_status_bytes']):
        raise FatalCheckError('All six motors must remain enabled and fault-free')
    status = bytes.fromhex(sample['arm_status_raw'])
    grip = monitor.parts[0x2A8][0]
    if status[:4] != bytes((1, 0, 1, 0)) or int.from_bytes(status[6:], 'big'):
        raise FatalCheckError('Fresh healthy CAN/J controller feedback required')
    if grip[6] & 0x3f or not grip[6] & 0x40 or grip[7] != 0:
        raise FatalCheckError('Held gripper must remain enabled fault-free in width mode')
    pose = sample['pose']
    if not all(math.isfinite(value) for value in pose):
        raise FatalCheckError('Nonfinite recovery feedback')
    reference = review['expected_pose_deg_m'] if anchor is None else anchor
    for index, (value, (low, high)) in enumerate(zip(pose[:6], review['joint_bounds_deg'], strict=True)):
        if index == 4:
            if start:
                if not 70 < value <= 70.1:
                    raise FatalCheckError('Actual starting J5 must be in(70,70.1]')
            else:
                if value <= 70:
                    monitor.inside_hard_j5 = True
                upper = 70. if monitor.inside_hard_j5 else 70.1
                if not max(low, 67.5) <= value <= upper:
                    raise FatalCheckError('J5 left the narrow inward recovery corridor')
        elif not low <= value <= high or abs(value-reference[index]) > .5:
            raise FatalCheckError('Other-axis bounds/deviation exceeded0.5deg')
    if not 0 <= pose[6] <= .1 or abs(pose[6]-reference[6]) > .002:
        raise FatalCheckError('Held gripper width changed more than2mm')
    if after is not None:
        monitor.fresh(REQUIRED_IDS, now=now, after=after)
    return sample


class OnceSender:
    def __init__(self, bus, monitor, *, clock=time.monotonic):
        self.bus, self.monitor, self.clock = bus, monitor, clock
        self.log = []
        self.phase = 0

    def send(self, *, deadline, sample=None, j5=None):
        now = self.clock()
        if now >= deadline:
            raise FatalCheckError('Recovery deadline expired before transmission')
        if self.phase == 0 and j5 is None:
            can_id, data, name = 0x151, MODE, 'mode5'
        elif self.phase in (1, 2) and type(j5) is int and j5 == (69, 68)[self.phase-1] and sample is not None:
            if abs(j5-sample['pose'][4]) > 2.:
                raise FatalCheckError('J5 target delta exceeds2deg')
            # Original feedback bytes preserve J6 exactly at CAN resolution.
            j6_raw = self.monitor.parts[0x2A7][0][4:]
            can_id, data, name = 0x157, struct.pack('>i', j5*1000) + j6_raw, f'j5_{j5}'
        else:
            raise FatalCheckError('Recovery phase repeated or outside fixed allowlist')
        self.phase += 1  # Consume the attempt before send; failures never retry.
        item = {'phase': name, 'id': hex(can_id), 'data': data.hex(),
                'attempt_at': now, 'socket_accepted': False}
        self.log.append(item)
        self.monitor.expected_echoes.append(item)
        packet = struct.pack('=IB3x8s', can_id, 8, data)
        if self.bus.send(packet, socket.MSG_DONTWAIT) != len(packet):
            raise FatalCheckError('Partial/unknown socket send; no retry')
        item.update(socket_accepted=True, completed_at=self.clock())
        return item['completed_at']


def drive_recovery(monitor, sender, review, *, read, clock=time.monotonic, recheck_endpoint=lambda: None):
    """Pure orchestrator with injectable RX/clock/endpoint; no autonomous retry."""
    result = {'operation': 'recover_j5_boundary_once', 'success': False,
        'tx_log': sender.log, 'physical_stop_confirmed': False,
        'mode_bus_echo_is_not_firmware_ack': True, 'automatic_retry': False,
        'no_j1_to_j4_gripper_enable_disable_reset_or_zero': True}
    before = None
    phase = 0
    mode_at = first_at = second_at = stable_at = stable_source = None
    first_stable_at = first_stable_source = None
    preflight_deadline = clock() + 3.
    action_deadline = None
    try:
        while True:
            if clock() >= (preflight_deadline if phase == 0 else action_deadline):
                raise FatalCheckError('Preflight/recovery deadline exceeded; no retry')
            read()
            now = clock()
            if now >= (preflight_deadline if phase == 0 else action_deadline):
                raise FatalCheckError('Preflight/recovery deadline exceeded after RX; no retry')
            try:
                sample = checked_sample(monitor, review, now=now,
                    anchor=None if before is None else before['pose'], start=phase == 0)
            except FatalCheckError:
                raise
            except ResetCheckError:
                if phase:
                    raise
                continue  # Initial RX cohort may not yet be complete; no TX.
            if phase == 0:
                if now-monitor.last_control_at < 1.:
                    continue
                recheck_endpoint()
                read()
                sample = checked_sample(monitor, review, now=clock(), start=True)
                if clock()-monitor.last_control_at < 1.:
                    raise FatalCheckError('Control quiet was lost before mode request')
                if clock() >= preflight_deadline:
                    raise FatalCheckError('Preflight deadline expired before mode request')
                before = result['before'] = sample
                monitor.active = True
                action_deadline = clock() + 2.
                mode_at = sender.send(deadline=action_deadline)
                phase = 1
                continue
            marker = (mode_at, first_at, second_at)[phase-1]
            try:
                monitor.fresh(REQUIRED_IDS, now=now, after=marker)
            except ResetCheckError:
                continue  # Require actual post-command source timestamps.
            if phase == 1 and 'rx_observed_at' in sender.log[0]:
                recheck_endpoint()
                read()
                sample = checked_sample(monitor, review, now=clock(), anchor=before['pose'], after=mode_at)
                result['after_mode'] = sample
                first_at = sender.send(deadline=action_deadline, sample=sample, j5=69)
                phase = 2
            elif phase == 2:
                source = monitor.parts[0x2A7][1]
                if 'rx_observed_at' not in sender.log[1] or abs(sample['pose'][4]-69.) > .2:
                    first_stable_at = first_stable_source = None
                    continue
                if first_stable_at is None:
                    first_stable_at, first_stable_source = now, source
                    continue
                if now-first_stable_at < .1 or source <= first_stable_source or now-first_at < .1:
                    continue
                recheck_endpoint()
                read()
                sample = checked_sample(monitor, review, now=clock(), anchor=before['pose'], after=first_at)
                if abs(sample['pose'][4]-69.) > .2 or clock()-first_at < .1:
                    first_stable_at = first_stable_source = None
                    continue
                result['after_69'] = sample
                result['phase69_stable_duration_s'] = clock()-first_stable_at
                second_at = sender.send(deadline=action_deadline, sample=sample, j5=68)
                phase = 3
            elif phase == 3 and 'rx_observed_at' in sender.log[2]:
                source = monitor.parts[0x2A7][1]
                if abs(sample['pose'][4]-68.) <= .2:
                    if stable_at is None:
                        stable_at, stable_source = now, source
                    elif now-stable_at >= .1 and source > stable_source:
                        result.update(success=True, after=sample,
                                      final_stable_duration_s=now-stable_at)
                        break
                else:
                    stable_at = stable_source = None
    except (Exception, KeyboardInterrupt) as error:
        result['error'] = str(error) or type(error).__name__
    return result


def drain(bus, helpers, mapper, monitor):
    for _ in range(4096):
        try:
            packet, ancillary, flags, _ = bus.recvmsg(16, socket.CMSG_SPACE(helpers.TIMESPEC.size))
        except BlockingIOError:
            return
        if len(packet) != 16:
            raise FatalCheckError('Malformed classic CAN packet')
        can_id, size, data = struct.unpack('=IB3x8s', packet)
        if size > 8:
            raise FatalCheckError('Invalid classic CAN DLC')
        if can_id & 0xe0000000:
            raise FatalCheckError('Unexpected extended/error/RTR CAN frame')
        source_ns, _ = mapper.convert(helpers._kernel_timestamp(ancillary, flags))
        if size != 8 and can_id not in CONTROL:
            continue
        monitor.ingest(can_id, data[:size], source_at=source_ns/1e9, now=time.monotonic())
    raise FatalCheckError('CAN receive queue did not drain')


def run_recovery(review, *, interface='can0', helper_path='/home/ubuntu/kr260-camera/remote_can_source.py'):
    validate_review(review, execute=True)
    result = {'operation': 'recover_j5_boundary_once', 'success': False, 'tx_log': [],
              'physical_stop_confirmed': False, 'automatic_retry': False}
    sender = None
    try:
        helpers = load_helpers(helper_path)
        if struct.calcsize('@l') != 8:
            raise FatalCheckError('64-bit Linux kernel timestamp support required')
        with ExitStack() as stack:
            owner = stack.enter_context(open(OWNER_PATH, 'a'))
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            identity, link = verify_endpoint(helpers, interface, review['expected_identity'])
            mapper = helpers.ClockMapper()
            mapper.sample()
            rx = stack.enter_context(socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW))
            tx = stack.enter_context(socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW))
            rx.setsockopt(socket.SOL_SOCKET, helpers.SO_TIMESTAMPNS_OLD, 1)
            rx.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            tx.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 1)
            tx.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b'')
            for bus in (rx, tx):
                bus.bind((interface,))
                bus.setblocking(False)
            monitor = RecoveryMonitor(started_at=time.monotonic())
            sender = OnceSender(tx, monitor)
            def read():
                select.select([rx], [], [], .005)
                drain(rx, helpers, mapper, monitor)
            result = drive_recovery(monitor, sender, review, read=read,
                recheck_endpoint=lambda: verify_endpoint(helpers, interface, review['expected_identity']))
            result.update(identity=identity, link=link)
    except (Exception, KeyboardInterrupt) as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        if sender is not None:
            result['tx_log'] = sender.log
    return result


def plan(review=None):
    return {'operation': 'recover_j5_boundary_once', 'check_only': True,
        'hardware_opened': False, 'mode': {'id': '0x151', 'data': MODE.hex(), 'attempts': 1},
        'targets': [{'id': '0x157', 'j5_deg': target, 'j6': 'fresh feedback unchanged', 'attempts': 1}
                    for target in (69, 68)],
        'starting_j5_domain_deg': '(70,70.1]', 'max_target_delta_deg': 2.,
        'other_joint_max_deviation_deg': .5, 'gripper_max_deviation_m': .002,
        'preflight_timeout_s': 3., 'control_quiet_s': 1., 'action_timeout_s': 2.,
        'phase69_tolerance_deg': .2, 'min_time_before68_s': .1,
        'phase69_stability_s': .1,
        'phase68_tolerance_deg': .2, 'final_stability_s': .1,
        'owner_path': OWNER_PATH, 'mode_evidence': 'exact new bus frame plus post-mode healthy enabled CAN/J feedback; no independent POS_VEL firmware ACK',
        'failure': 'no further writes; no retry/disable and no physical-stop guarantee',
        'approval_blockers': (list(APPROVAL_FLAGS) if review is None else
                              [key for key in APPROVAL_FLAGS if review.get(key) is not True]),
        'review': review}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-only', action='store_true')
    mode.add_argument('--execute', action='store_true')
    parser.add_argument('--review-json', type=Path)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--rx-helper', default='/home/ubuntu/kr260-camera/remote_can_source.py')
    args = parser.parse_args(argv)
    if args.execute and args.review_json is None:
        parser.error('--execute requires --review-json and explicit approval')
    review = None if args.review_json is None else json.loads(args.review_json.read_text())
    if review is not None:
        validate_review(review, execute=args.execute)
    if not args.execute:
        print(json.dumps(plan(review), ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, terminate)
    result = run_recovery(review, interface=args.interface, helper_path=args.rx_helper)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
