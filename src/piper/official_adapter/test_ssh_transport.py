"""Exercise remote CAN safety gates without sockets, SSH, or real hardware."""
import io
import json
from pathlib import Path
import struct
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from lerobot_robot_piper_shared import devices as d
from lerobot_robot_piper_shared import ssh_transport as ssh
from lerobot_robot_piper_shared.transport import PassiveCAN


IDENTITY = {'interface': 'can0', 'host': {'machine_id': 'test-only-kr260'},
            'device_path': '/test-only/usb', 'ifindex': 8}


def config():
    item = json.loads(d.IO_CONFIG.read_text())
    item['transport'] = {'kind': 'ssh', 'ssh_host': 'test-only-kr260',
                         'remote_script': '/test-only/remote_can_source.py',
                         'expected_identity': IDENTITY, 'max_clock_rtt_ms': 1000}
    return item


class SourceClockTests(unittest.TestCase):
    def calibrated(self, offset=5_000_000_000_000):
        clock = ssh.SourceClock()
        start = 1_000_000_000_000
        clock.update(start, start+20_000_000,
                     start+offset+3_000_000, start+offset+4_000_000)
        return clock, start, offset

    def test_arbitrary_clock_origins_preserve_frame_spacing_and_age_bound(self):
        for offset in (-900_000_000_000, 5_000_000_000_000, 6_000_000_000_000_000):
            with self.subTest(offset=offset):
                clock, start, offset = self.calibrated(offset)
                arrival = start+30_000_000
                first = clock.host_time(start+offset+5_000_000, arrival)
                second = clock.host_time(start+offset+10_000_000, arrival)
                self.assertAlmostEqual(second-first, .005, places=8)
                true_first = (start+5_000_000)/1e9
                self.assertLessEqual(first, true_first)
                self.assertGreaterEqual(arrival/1e9-first, .025)

    def test_ssh_backlog_cannot_make_old_frames_fresh(self):
        clock, start, offset = self.calibrated()
        arrival = start+180_000_000
        mapped = clock.host_time(start+offset+10_000_000, arrival)
        self.assertGreater((arrival/1e9-mapped)*1000, 100)

    def test_missing_expired_and_future_clock_data_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, '未校准'):
            ssh.SourceClock().host_time(1, 1)
        clock, start, offset = self.calibrated()
        with self.assertRaisesRegex(RuntimeError, '过期'):
            clock.host_time(start+offset, start+7_000_000_000)
        with self.assertRaisesRegex(RuntimeError, '未来'):
            clock.host_time(start+offset+1_000_000_000, start+21_000_000)

    def test_bad_rtt_processing_and_clock_discontinuity_are_rejected(self):
        for values in ((10, 10, 20, 20), (10, 20, 20, 31),
                       (10, 20, 31, 20), (0, 100_000_000, 20, 21)):
            with self.subTest(values=values), self.assertRaises(RuntimeError):
                ssh.SourceClock().update(*values)
        clock, start, offset = self.calibrated()
        with self.assertRaisesRegex(RuntimeError, '不连续'):
            clock.update(start+1_000_000_000, start+1_020_000_000,
                         start+offset+2_003_000_000, start+offset+2_004_000_000)


class EndpointProofTests(unittest.TestCase):
    def test_remote_identity_must_match_binding(self):
        hello = {'identity': {**IDENTITY, 'ifindex': 99}, 'link': {'ready': True}}
        with patch.object(ssh, 'remote_probe', return_value=hello):
            with self.assertRaisesRegex(RuntimeError, '身份与绑定不符'):
                d.inspect_endpoint(config())

    def test_current_remote_proof_must_match_identity_mapping_units_and_source(self):
        cfg = config()
        hello = {'identity': IDENTITY, 'link': {'ready': True}}
        with tempfile.TemporaryDirectory(prefix='piper_ssh_proof_test_') as directory:
            path = Path(directory)/'remote_proof.json'
            cfg['mapping_proof'] = str(path)
            with patch.object(ssh, 'remote_probe', return_value=hello):
                endpoint = d.inspect_endpoint(cfg)
                good = {'interface_identity': endpoint['interface_identity'],
                        'mapping': cfg['mapping'], 'joint_unit': 'deg',
                        'gripper_mode': 'width', 'current_source_verified': True}
                for change in ({'interface_identity': {'interface': 'can0', 'ifindex': 14}},
                               {'mapping': {}}, {'joint_unit': 'rad'},
                               {'gripper_mode': 'angle'}, {'current_source_verified': False}):
                    with self.subTest(change=change):
                        path.write_text(json.dumps({**good, **change}))
                        with self.assertRaises(RuntimeError):
                            d.check_endpoint(cfg)
                path.write_text(json.dumps(good))
                self.assertEqual(d.check_endpoint(cfg), endpoint)


class _StopAfterRetry:
    def __init__(self): self.done = False
    def is_set(self): return self.done
    def set(self): self.done = True
    def wait(self, _timeout): self.done = True


class _Stream:
    def __init__(self, descriptor, callback=None):
        self.descriptor, self.callback, self.closed = descriptor, callback, False
    def fileno(self): return self.descriptor
    def write(self, value):
        if self.callback: self.callback(value)
        return len(value)
    def flush(self): pass
    def close(self): self.closed = True


class _FakeSSH:
    """Only emulates stdio; it never launches a process or opens a socket."""
    def __init__(self, frames=None, before_pong=False, identity=None):
        self.frames = frames or [(1, 0)]
        self.before_pong = before_pong
        self.returncode, self.queue, self.reading = None, [], b''
        self.stdin = _Stream(101, self.ping)
        self.stdout, self.stderr = _Stream(102), _Stream(103)
        self.selector = SimpleNamespace(register=lambda *_: None,
            unregister=lambda *_: None, close=lambda: None, select=self.select)
        initial = self.line('hello', identity=IDENTITY if identity is None else identity,
                            link={'ready': True})
        if before_pong:
            initial += self.frame_line([(1, 0)])
        self.queue.append(initial)

    @staticmethod
    def line(kind, **data):
        return (json.dumps({'protocol': ssh.PROTOCOL, 'type': kind, **data})+'\n').encode()

    def frame_line(self, frames):
        source = time.monotonic_ns()+5_000_000_000_000-1_000_000
        packet = struct.pack('=IB3x8s', 0x2a1, 8, bytes([1,0,1,0,1,0,0,0])).hex()
        return self.line('frames', frames=[[seq, source+delta, packet] for seq, delta in frames])

    def ping(self, value):
        token = json.loads(value)['token']
        remote = time.monotonic_ns()+5_000_000_000_000
        self.queue.append(self.line('pong', token=token, remote_receive_ns=remote,
                                    remote_send_ns=remote)+self.frame_line(self.frames))

    def select(self, _timeout):
        self.reading = self.queue.pop(0) if self.queue else b''
        return [(SimpleNamespace(fileobj=self.stdout, data='stdout'), None)]

    def poll(self): return self.returncode
    def terminate(self): self.returncode = -15
    def kill(self): self.returncode = -9
    def wait(self, timeout): return self.returncode


class RemoteSessionTests(unittest.TestCase):
    def run_session(self, process):
        with patch.object(threading.Thread, 'start'):
            bus = PassiveCAN(config())
        bus.stop_event = _StopAfterRetry()
        bus.latest[('old', 0)] = {'receive_monotonic': 1}
        output = io.StringIO()
        bus.attach_raw(output)
        with patch.object(ssh.subprocess, 'Popen', return_value=process) as launch, \
             patch.object(ssh.selectors, 'DefaultSelector', return_value=process.selector), \
             patch.object(ssh.os, 'read', side_effect=lambda *_: process.reading):
            bus.run()
        command = launch.call_args.args[0]
        self.assertIn('StrictHostKeyChecking=yes', command)
        self.assertNotIn('sudo', command)
        self.assertEqual(bus.latest, {})
        self.assertFalse(bus.link.get('ready'))
        self.assertIsNone(bus.remote_process)
        self.assertTrue(all(stream.closed for stream in (process.stdin, process.stdout, process.stderr)))
        return bus, [json.loads(line) for line in output.getvalue().splitlines()]

    def test_disconnect_clears_old_and_new_data_and_closes_process(self):
        bus, rows = self.run_session(_FakeSSH())
        self.assertEqual(bus.generation, 1)
        self.assertIn('连接断开', bus.error)
        self.assertEqual(len(rows), 1)
        self.assertIn('source_monotonic_ns', rows[0])

    def test_sequence_gap_duplicate_and_source_time_rollback_fail_closed(self):
        for frames in ([(2, 0)], [(1, 0), (1, 1)], [(1, 0), (2, -6_000_000)]):
            with self.subTest(frames=frames):
                bus, _ = self.run_session(_FakeSSH(frames))
                self.assertIn('源序号或时间不连续', bus.error)

    def test_frames_before_clock_calibration_are_never_ingested(self):
        bus, rows = self.run_session(_FakeSSH([(2, 0)], before_pong=True))
        self.assertIn('连接断开', bus.error)
        self.assertEqual([row['source_sequence'] for row in rows], [2])

    def test_unexpected_remote_identity_does_not_accept_frames(self):
        bus, rows = self.run_session(_FakeSSH(identity={**IDENTITY, 'ifindex': 99}))
        self.assertIn('身份已改变', bus.error)
        self.assertEqual(rows, [])
        self.assertEqual(bus.generation, 0)


if __name__ == '__main__': unittest.main()
