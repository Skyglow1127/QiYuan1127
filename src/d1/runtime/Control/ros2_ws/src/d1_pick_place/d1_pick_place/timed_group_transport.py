"""Timed multi-axis trajectory packets and the line-oriented DDS bridge.

Each point is one seven-value SDK snapshot with a single trajectory time. Only
J0--J5 are commanded; gripper_mode='preserve' makes J6 informational. Neither a
local DDS publish nor accepted/buffered/dispatched state is an arrival signal.
The board owns its steady-clock epoch and its completion criterion.

This module submits all chunks before execution, without per-axis ACK waits.
It never enables, powers, homes, unloads, retries or silently retimes motion.
"""
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np


COMMAND_SCHEMA = 'd1_timed_trajectory_v1'
STATE_SCHEMA = 'd1_trajectory_state_v1'
COMMAND_TOPIC = 'rt/d1_TimedTrajectory'
STATE_TOPIC = 'rt/d1_TrajectoryState'
LIMITS = ((-135., 135.), (-90., 90.), (-90., 90.),
          (-135., 135.), (-90., 90.), (-135., 135.))
STATES = frozenset(('accepted', 'buffered', 'started', 'paused', 'resumed',
                    'dispatched', 'completed', 'cancelled', 'fault'))
TERMINAL_STATES = frozenset(('completed', 'cancelled', 'fault'))
MIN_POINT_INTERVAL_MS = 20
MAX_TRAJECTORY_TIME_MS = 120000
MAX_TRAJECTORY_POINTS = 4096


def _integer(value, name, minimum=0, maximum=2147483647):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be an integer in [{minimum}, {maximum}]')
    return value


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be positive and finite')
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f'{name} must be positive and finite')
    return result


def _identity(value):
    if (not isinstance(value, str) or not value or value != value.strip()
            or len(value.encode('utf-8')) > 128 or any(ord(c) < 32 for c in value)):
        raise ValueError('trajectory_id must be a nonempty identity of at most 128 UTF-8 bytes')
    return value


def checked_points(points, *, previous=None, previous_velocity=None,
                   previous_interval_s=None, velocity_limit_deg_s=5.,
                   acceleration_limit_deg_s2=10.):
    """Validate original timing and six-axis finite differences, across chunks.

    No rounding, resampling or independent-axis timing is introduced. Interval
    velocities use (q[k]-q[k-1])/dt; acceleration uses the separation between
    adjacent velocity midpoints. J6 is finite snapshot data and is not commanded.
    """
    velocity_limit = _positive(velocity_limit_deg_s, 'velocity_limit_deg_s')
    acceleration_limit = _positive(acceleration_limit_deg_s2, 'acceleration_limit_deg_s2')
    if not isinstance(points, (list, tuple)) or not points:
        raise ValueError('A chunk must contain at least one trajectory point')
    result = []
    prior = previous
    last_velocity = previous_velocity
    last_interval = previous_interval_s
    for index, point in enumerate(points):
        if not isinstance(point, dict) or set(point) != {'time_from_start_ms', 'sdk_angles'}:
            raise ValueError('Each point requires exactly time_from_start_ms and sdk_angles')
        stamp = point['time_from_start_ms']
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(float(stamp)) or not 0 <= stamp <= MAX_TRAJECTORY_TIME_MS):
            raise ValueError('time_from_start_ms must be finite numeric milliseconds in [0, 120000]')
        # Preserve the caller's original numeric time (the executor keeps six
        # decimal places). JSON supports floating milliseconds; do not quantize
        # them to integer milliseconds or silently alter the time axis.
        angles = point['sdk_angles']
        if not isinstance(angles, (list, tuple)) or len(angles) != 7:
            raise ValueError('sdk_angles must contain seven SDK values')
        values = []
        for joint, value in enumerate(angles):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))):
                raise ValueError(f'Non-finite or nonnumeric J{joint} target')
            value = float(value)
            if joint < 6 and not LIMITS[joint][0] <= value <= LIMITS[joint][1]:
                raise ValueError(f'J{joint} target outside original command limits')
            values.append(value)
        current = {'time_from_start_ms': stamp, 'sdk_angles': values}
        if prior is not None:
            dt = (stamp-prior['time_from_start_ms'])/1000.
            if stamp-prior['time_from_start_ms'] < MIN_POINT_INTERVAL_MS:
                raise ValueError('Original times must strictly increase with at least 20 ms between points, including across chunks')
            velocity = tuple((values[j]-prior['sdk_angles'][j])/dt for j in range(6))
            if max(abs(v) for v in velocity) > velocity_limit+1e-6:
                raise ValueError(f'Point {index} exceeds the reviewed six-axis velocity limit')
            if last_velocity is not None:
                separation = (last_interval+dt)/2.
                acceleration = max(abs(a-b)/separation for a, b in zip(velocity, last_velocity))
                if acceleration > acceleration_limit+1e-6:
                    raise ValueError(f'Point {index} exceeds the reviewed six-axis acceleration limit')
            last_velocity, last_interval = velocity, dt
        result.append(current)
        prior = current
    return result, prior, last_velocity, last_interval


class TimedGroupTransport:
    """Asynchronous bridge transport, or an injected callable JSON publisher.

    ``output`` is an exclusive-create JSONL log filename. A supplied publisher
    receives one serialized JSON object per call and can feed replies through
    ``ingest_envelope``; otherwise binary is the dedicated d1_timed_bridge CLI.
    ``close`` only closes communication. Cancellation is always explicit.
    """
    def __init__(self, binary=None, nic='enp99s0', output=None, *, publisher=None,
                 domain=None, now=time.monotonic):
        if publisher is None and binary is None:
            raise ValueError('A bridge binary or injected publisher is required')
        if publisher is not None and binary is not None:
            raise ValueError('Choose bridge mode or an injected publisher')
        if publisher is not None and not callable(publisher):
            raise ValueError('publisher must be callable')
        if not isinstance(nic, str) or not nic:
            raise ValueError('A network interface is required')
        if domain is not None:
            _integer(domain, 'domain', 0, 232)
        self.now = now
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self._write_lock = threading.Lock()
        self._publisher = publisher
        self.log = None if output is None else Path(output).open('x')
        self.proc = None
        self.reader = None
        self.closed = False
        self.fault = None
        self.commands = 0
        self.command_attempts = 0
        # State confirmations only; these never count endpoint arrival.
        self.acks = 0
        self._ack_states = set()
        self._state_sequences = {}
        self.chunks = 0
        self.controls = 0
        self.local_published = 0
        self._sequence = 0
        self._packets = {}
        self._trajectories = {}
        self.trajectory_states = {}
        self.state_history = []
        self.q = None
        self.q_time = -math.inf
        self.q_seq = 0
        self.raw_q = None
        self.raw_q_time = -math.inf
        self.joint_q_time = [-math.inf]*7
        self.joint_q_seq = [0]*7
        self.joint_feedback_pending = [False]*7
        self.status = {}
        self.status_time = {}
        self.onboard_diagnostics = None
        self.onboard_diagnostics_time = -math.inf
        self.last_bridge_message_time = -math.inf
        self._record('transport_contract', command_topic=COMMAND_TOPIC, state_topic=STATE_TOPIC,
                     command_schema=COMMAND_SCHEMA, gripper_mode='preserve',
                     epoch_owner='board steady clock, established when chunk 0 is accepted',
                     upload_policy='all chunks before epoch; no execution-time append',
                     arrival_semantics='only firmware completed state; publish/accepted are not arrival')
        if publisher is None:
            argv = [str(binary), '--nic', nic]
            if domain is not None:
                argv += ['--domain', str(domain)]
            try:
                self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE, text=True, bufsize=1)
            except Exception:
                if self.log is not None:
                    self.log.close()
                raise
            self.reader = threading.Thread(target=self._read, daemon=True)
            self.reader.start()
            self._stderr_reader = threading.Thread(target=self._read_stderr, daemon=True)
            self._stderr_reader.start()

    def _record(self, kind, **data):
        with self.lock:
            if self.log is not None and not self.log.closed:
                self.log.write(json.dumps(dict(t=self.now(), kind=kind, **data), ensure_ascii=False)+'\n')
                self.log.flush()

    def _set_fault(self, reason):
        with self.changed:
            if self.fault is None:
                self.fault = str(reason)
                self._record('transport_fault', reason=self.fault)
            self.changed.notify_all()

    def _ensure_open(self, *, allow_fault=False):
        if self.closed:
            raise RuntimeError('Timed trajectory transport is closed')
        if self.fault and not allow_fault:
            raise RuntimeError(self.fault)

    def _next_sequence(self):
        self._sequence = _integer(self._sequence+1, 'message_seq', 1)
        return self._sequence

    def _publish(self, packet, *, allow_fault=False):
        """The caller holds _write_lock; this never waits for a device ACK."""
        text = json.dumps(packet, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
        with self.lock:
            self._ensure_open(allow_fault=allow_fault)
            self._packets[packet['message_seq']] = packet
            self.command_attempts += 1
            self._record('trajectory_submission', packet=packet,
                         time_semantics='host bridge stdin/publisher submission, not device adoption')
        try:
            if self._publisher is not None:
                if self._publisher(text) is False:
                    raise RuntimeError('Injected trajectory publisher rejected the packet')
            else:
                if self.proc.poll() is not None:
                    raise RuntimeError('Timed DDS bridge exited before submission')
                self.proc.stdin.write(text+'\n')
                self.proc.stdin.flush()
        except Exception as error:
            self._set_fault(error)
            raise
        with self.lock:
            # Whole chunk/control submissions, never individual UART commands.
            # local_published separately counts confirmed bridge DDS Writes.
            self.commands += 1
        return packet['message_seq']

    def submit_chunk(self, trajectory_id, chunk_index, points, *, final,
                     start_after_ms=1000, max_lateness_ms=20,
                     max_tracking_error_deg=3., velocity_limit_deg_s=5.,
                     acceleration_limit_deg_s2=10., gripper_mode='preserve'):
        """Submit one ordered chunk of a common seven-value timed trajectory.

        ``start_after_ms`` is the board's lead time, constrained to 100..5000 ms.
        All chunk parameters remain constant for one trajectory. The conservative
        host upload deadline starts at first submission, without assuming that
        host and board monotonic clocks share an epoch.
        """
        identity = _identity(trajectory_id)
        index = _integer(chunk_index, 'chunk_index')
        if type(final) is not bool:
            raise ValueError('final must be a boolean')
        if gripper_mode != 'preserve':
            raise ValueError('This transport only supports gripper_mode=preserve')
        parameters = dict(start_after_ms=_integer(start_after_ms, 'start_after_ms', 100, 5000),
                          max_lateness_ms=_integer(max_lateness_ms, 'max_lateness_ms', 1, 50),
                          max_tracking_error_deg=_positive(max_tracking_error_deg, 'max_tracking_error_deg'),
                          velocity_limit_deg_s=_positive(velocity_limit_deg_s, 'velocity_limit_deg_s'),
                          acceleration_limit_deg_s2=_positive(acceleration_limit_deg_s2, 'acceleration_limit_deg_s2'),
                          gripper_mode='preserve')
        with self._write_lock:
            with self.lock:
                self._ensure_open()
                existing = self._trajectories.get(identity)
                if existing is None:
                    if index != 0:
                        raise ValueError('The first chunk index must be zero')
                    existing = dict(next_chunk_index=0, final=False, parameters=parameters,
                                    previous=None, previous_velocity=None, previous_interval_s=None,
                                    first_submission_monotonic_s=self.now(), point_count=0)
                elif existing['parameters'] != parameters:
                    raise ValueError('Trajectory timing/safety parameters changed between chunks')
                if existing['final'] or index != existing['next_chunk_index']:
                    raise ValueError('Chunks must be ordered without reuse after final')
                if index and self.now() >= existing['first_submission_monotonic_s']+start_after_ms/1000.:
                    raise RuntimeError('Trajectory upload missed its lead time; execution-time append is unsupported')
                values, previous, velocity, interval = checked_points(
                    points, previous=existing['previous'], previous_velocity=existing['previous_velocity'],
                    previous_interval_s=existing['previous_interval_s'],
                    velocity_limit_deg_s=parameters['velocity_limit_deg_s'],
                    acceleration_limit_deg_s2=parameters['acceleration_limit_deg_s2'])
                if existing['point_count']+len(values) > MAX_TRAJECTORY_POINTS:
                    raise ValueError('A trajectory may contain at most 4096 points across all chunks')
                packet = dict(schema=COMMAND_SCHEMA, type='chunk', trajectory_id=identity,
                              message_seq=self._next_sequence(), chunk_index=index,
                              final=final, points=values, **parameters)
                self._trajectories[identity] = dict(existing, next_chunk_index=index+1,
                                                    final=final, previous=previous,
                                                    previous_velocity=velocity, previous_interval_s=interval,
                                                    point_count=existing['point_count']+len(values))
            sequence = self._publish(packet)
            with self.lock:
                self.chunks += 1
            return sequence

    send_chunk = submit_chunk

    def _control(self, trajectory_id, command):
        identity = _identity(trajectory_id)
        if command not in ('cancel', 'pause', 'resume'):
            raise ValueError('Unsupported trajectory control command')
        with self._write_lock:
            with self.lock:
                self._ensure_open(allow_fault=command == 'cancel')
                if identity not in self._trajectories:
                    raise ValueError('Control command requires a trajectory submitted by this transport')
                packet = dict(schema=COMMAND_SCHEMA, type=command, trajectory_id=identity,
                              message_seq=self._next_sequence())
            sequence = self._publish(packet, allow_fault=command == 'cancel')
            with self.lock:
                self.controls += 1
            return sequence

    def cancel(self, trajectory_id):
        return self._control(trajectory_id, 'cancel')

    def pause(self, trajectory_id):
        return self._control(trajectory_id, 'pause')

    def resume(self, trajectory_id):
        return self._control(trajectory_id, 'resume')

    def _read(self):
        try:
            for raw in self.proc.stdout:
                line = raw.rstrip()
                self._record('bridge_output', line=line)
                if not line:
                    continue
                self.ingest_envelope(json.loads(line), record=False)
        except Exception as error:
            self._set_fault('Invalid bridge output: '+str(error))
        finally:
            with self.changed:
                if not self.closed:
                    self._set_fault('Timed DDS bridge stdout ended')
                self.changed.notify_all()

    def _read_stderr(self):
        try:
            for line in self.proc.stderr:
                self._record('bridge_stderr', line=line.rstrip())
        except Exception as error:
            if not self.closed:
                self._set_fault('Timed bridge stderr reader: '+str(error))

    def ingest_envelope(self, envelope, *, record=True):
        """Receive a bridge envelope, also usable by an injected publisher."""
        if not isinstance(envelope, dict):
            raise ValueError('Bridge output must be an object')
        if record:
            self._record('bridge_envelope', envelope=envelope)
        kind = envelope.get('kind')
        stamp_ns = envelope.get('host_monotonic_ns')
        stamp = self.now() if stamp_ns is None else _integer(
            stamp_ns, 'host_monotonic_ns', 0, 9223372036854775807)/1e9
        if stamp > self.now()+.1:
            raise ValueError('Bridge host receive timestamp is in the future')
        message = envelope.get('message')
        with self.changed:
            self.last_bridge_message_time = max(self.last_bridge_message_time, stamp)
            if kind == 'state':
                self._state(message, stamp)
            elif kind == 'feedback':
                self._feedback(message, stamp)
            elif kind == 'published':
                # A bridge publish proves local DDS Write, never motor arrival.
                self.local_published += 1
            elif kind == 'error':
                self._set_fault('DDS bridge error: '+str(message))
            else:
                raise ValueError('Unknown bridge envelope kind: '+str(kind))
            self.changed.notify_all()

    def _state(self, message, stamp):
        if not isinstance(message, dict) or message.get('schema') != STATE_SCHEMA:
            raise ValueError('Invalid trajectory state schema')
        identity = message.get('trajectory_id', message.get('id'))
        if 'trajectory_id' in message and 'id' in message and message['trajectory_id'] != message['id']:
            raise ValueError('Conflicting trajectory state identities')
        if identity not in self._trajectories:
            self._record('unowned_trajectory_state', message=message)
            return
        state = message.get('state')
        if state not in STATES:
            raise ValueError('Unknown board trajectory state')
        seq = _integer(message.get('message_seq'), 'state.message_seq', 0)
        # The board increments message_seq for each emitted state. It is not
        # the request message_seq. Correlate by owned trajectory identity and
        # use the board sequence only to reject repeated/reordered states.
        if seq <= self._state_sequences.get(identity, -1):
            self._record('ignored_trajectory_state', message=message,
                         reason='nonadvancing board state sequence')
            return
        self._state_sequences[identity] = seq
        normalized = dict(message, trajectory_id=identity, received_monotonic_s=stamp)
        self.trajectory_states[identity] = normalized
        self.state_history.append(normalized)
        if len(self.state_history) > 512:
            del self.state_history[:-512]
        acknowledgement = (identity, seq, state)
        if state in ('accepted', 'buffered') and acknowledgement not in self._ack_states:
            self._ack_states.add(acknowledgement)
            self.acks += 1
        if state == 'fault':
            self._set_fault('Board trajectory fault: '+str(message.get('reason', message)))

    def _feedback(self, message, stamp):
        if not isinstance(message, dict) or message.get('address') != 2:
            return
        function, data = message.get('funcode'), message.get('data', {})
        if not isinstance(data, dict):
            raise ValueError('Invalid feedback data')
        if function == 1:
            raw = [data.get(f'angle{joint}') for joint in range(7)]
            self.raw_q = np.asarray([float(value) if type(value) in (int, float) else math.nan for value in raw])
            self.raw_q_time = stamp
            accepted = all(type(q) in (int, float) and math.isfinite(float(q)) for q in raw)
            if accepted:
                accepted = all(lo-.2-1e-5 <= float(raw[j]) <= hi+.2+1e-5
                               for j, (lo, hi) in enumerate(LIMITS))
            if not accepted or stamp <= self.q_time:
                self._record('ignored_angle_feedback', received_monotonic_s=stamp,
                             reason='invalid or nonadvancing raw frame; valid age unchanged')
                return
            self.q = np.asarray(raw, dtype=float)
            self.q_time = stamp
            self.q_seq += 1
            self.joint_q_time = [stamp]*7
            self.joint_q_seq = [self.q_seq]*7
        elif function == 3:
            for key in ('power_status', 'enable_status', 'error_status'):
                value = data.get(key)
                if type(value) is int and value in (0, 1) and stamp > self.status_time.get(key, -math.inf):
                    self.status[key] = value
                    self.status_time[key] = stamp
        elif function == 90 and data.get('schema') == 'd1_uart_integrity_v1':
            self.onboard_diagnostics = dict(data)
            self.onboard_diagnostics_time = stamp
            event = data.get('event')
            if (event in ('command_rejected', 'group_dispatch_failure')
                    or event == 'command_write' and data.get('write_ok') is not True):
                self._set_fault('Onboard command failure: '+str(data))

    def snapshot(self):
        with self.lock:
            return dict(q=None if self.q is None else self.q.copy(), q_time=self.q_time, q_seq=self.q_seq,
                        raw_q=None if self.raw_q is None else self.raw_q.copy(), raw_q_time=self.raw_q_time,
                        status=dict(self.status), status_time=dict(self.status_time), fault=self.fault,
                        joint_q_time=list(self.joint_q_time), joint_q_seq=list(self.joint_q_seq),
                        joint_feedback_pending=list(self.joint_feedback_pending), feedback_pending=False,
                        onboard_diagnostics=None if self.onboard_diagnostics is None else dict(self.onboard_diagnostics),
                        onboard_diagnostics_time=self.onboard_diagnostics_time,
                        trajectory_states={key: dict(value) for key, value in self.trajectory_states.items()},
                        last_bridge_message_time=self.last_bridge_message_time,
                        commands=self.commands, command_attempts=self.command_attempts,
                        acks=self.acks, chunks=self.chunks, controls=self.controls,
                        commands_semantics='successful host submissions of complete chunk/control packets; not per-servo commands',
                        acks_semantics='distinct accepted/buffered board state confirmations; not arrival',
                        local_published=self.local_published,
                        transport='timed_group_v1', hardware_atomic=False)

    def wait(self, seconds):
        if isinstance(seconds, bool) or not math.isfinite(float(seconds)) or seconds < 0:
            raise ValueError('Wait duration must be finite and nonnegative')
        with self.changed:
            self.changed.wait(timeout=float(seconds))

    def wait_for_state(self, trajectory_id, states, timeout_s, cancel=lambda: False):
        identity = _identity(trajectory_id)
        wanted = {states} if isinstance(states, str) else set(states)
        if not wanted or not wanted.issubset(STATES):
            raise ValueError('Invalid requested trajectory states')
        deadline = self.now()+_positive(timeout_s, 'timeout_s')
        with self.changed:
            while True:
                self._ensure_open()
                if cancel():
                    raise RuntimeError('Cancelled while waiting for trajectory state')
                state = self.trajectory_states.get(identity)
                if state is not None and state['state'] in wanted:
                    return dict(state)
                remaining = deadline-self.now()
                if remaining <= 0:
                    raise TimeoutError('Timed out waiting for trajectory state '+str(sorted(wanted)))
                self.changed.wait(timeout=min(.1, remaining))

    def close(self):
        """Close communication only; never generate cancel, hold or power commands."""
        with self.changed:
            self.closed = True
            self.changed.notify_all()
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()
            if self.reader is not None:
                self.reader.join(timeout=2.)
            self._stderr_reader.join(timeout=2.)
            self.proc.stdin.close()
            self.proc.stdout.close()
            self.proc.stderr.close()
        if self.log is not None:
            self.log.close()
