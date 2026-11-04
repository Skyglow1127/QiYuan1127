"""Synthetic delivery-deadline checks; no SSH, camera, SocketCAN or SDK calls."""

from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from lerobot_robot_piper_executor.backend import PiperBackend
from lerobot_robot_piper_executor.guard import ActionGuard, CheckedCommand, GuardError, GuardLimits, Readiness
from lerobot_robot_piper_executor.sdk_codec import Frame


class FakeClock:
    def __init__(self, value=10.159):
        self.value = value

    def __call__(self):
        return self.value


class FakeRPC:
    def __init__(self, clock, mapping_delay=0., receiver_ttl=.15):
        self.clock, self.mapping_delay, self.receiver_ttl = clock, mapping_delay, receiver_ttl
        self.mapped_deadlines = []
        self.calls = []

    def remote_deadline(self, host_deadline):
        self.mapped_deadlines.append(host_deadline)
        self.clock.value += self.mapping_delay
        return round(host_deadline*1e9)

    def request(self, op, **payload):
        self.calls.append((op, payload))
        # A synthetic receiver checks the real outgoing deadline, rather than
        # merely asserting the formula used by the sender.
        remaining = payload["deadline_ns"]/1e9-self.clock()
        if not 0 < remaining <= self.receiver_ttl+1e-9:
            raise GuardError("Synthetic receiver rejected delivery TTL")
        return {"accepted_sequence": payload["sequence"]}


class DeliveryDeadlineTests(unittest.TestCase):
    def make_backend(self, *, clock=None, ttl=.12, mapping_delay=0.):
        clock = clock or FakeClock()
        rpc = FakeRPC(clock, mapping_delay=mapping_delay)
        backend = PiperBackend(
            {"execute": True, "tx_ttl_s": ttl, "normal_gripper_force_n": 1.},
            {"transport": {"expected_identity": {}}}, client=rpc)
        backend.lease = "SYNTHETIC LEASE"
        return backend, rpc, clock

    def send(self, backend, command, clock):
        with patch("lerobot_robot_piper_executor.backend.time.monotonic", clock), \
             patch("lerobot_robot_piper_executor.backend.encode_target",
                   return_value=(Frame(0x155, b"\0"*8),)):
            backend.send_checked(command)

    def test_cpu_inference_age_and_receiver_delivery_ttl_are_independent(self):
        # These limits are synthetic, not approved hardware limits. A 159ms
        # prediction remains within its original 400ms source-age window.
        limits = GuardLimits(bounds=((-10., 10.),)*6+((-.01, .1),),
            max_step=(1.,)*6+(.002,), max_target_speed=(20.,)*6+(.04,),
            max_tracking_error=(2.,)*6+(.004,), max_observation_age_s=.4,
            max_command_age_s=.4, max_command_gap_s=.75)
        ready = Readiness(**{name: "SYNTHETIC OFFLINE EVIDENCE"
                             for name in Readiness.__dataclass_fields__})
        guard = ActionGuard(limits)
        guard.arm((0.,)*7, sampled_at=10., now=10., readiness=ready)
        target = (.1, 0., 0., 0., 0., 0., .001)
        command = guard.prepare(target, observed_state=(0.,)*7,
                                sampled_at=10., proposal_at=10., now=10.159)
        original = replace(command)
        backend, rpc, clock = self.make_backend()
        self.assertGreater(command.expires_at-clock(), rpc.receiver_ttl)
        self.send(backend, command, clock)
        self.assertEqual(command, original)
        self.assertEqual(command.created_at, 10.159)
        self.assertEqual(command.expires_at, 10.4)
        self.assertAlmostEqual(rpc.mapped_deadlines[0], 10.279)
        self.assertAlmostEqual(rpc.calls[0][1]["timeout_s"], .12)
        self.assertEqual(guard.last_target, (0.,)*7)
        guard.commit(command, now=10.169)
        self.assertEqual(guard.last_target, target)

    def test_original_source_expiry_is_never_extended(self):
        backend, rpc, clock = self.make_backend(clock=FakeClock(10.2), mapping_delay=.01)
        command = CheckedCommand(1, (0.,)*7, 10.19, 10.23)
        self.send(backend, command, clock)
        self.assertAlmostEqual(rpc.mapped_deadlines[0], 10.23)
        self.assertAlmostEqual(rpc.calls[0][1]["timeout_s"], .02)
        self.assertEqual(command.expires_at, 10.23)

    def test_clock_mapping_time_consumes_the_delivery_window(self):
        backend, rpc, clock = self.make_backend(mapping_delay=.03)
        self.send(backend, CheckedCommand(1, (0.,)*7, 10., 10.4), clock)
        self.assertAlmostEqual(rpc.calls[0][1]["timeout_s"], .09)
        self.assertLessEqual(rpc.calls[0][1]["timeout_s"], .25)

    def test_expiry_during_clock_mapping_sends_nothing(self):
        backend, rpc, clock = self.make_backend(mapping_delay=.13)
        with self.assertRaisesRegex(GuardError, "expired"):
            self.send(backend, CheckedCommand(1, (0.,)*7, 10., 10.4), clock)
        self.assertEqual(rpc.calls, [])
        self.assertEqual(backend.sequence, 0)

    def test_expired_or_nonfinite_host_deadline_sends_nothing(self):
        for expiry in (10., math.nan, math.inf):
            with self.subTest(expiry=expiry):
                backend, rpc, clock = self.make_backend()
                with self.assertRaises(GuardError):
                    self.send(backend, CheckedCommand(1, (0.,)*7, 10., expiry), clock)
                self.assertEqual(rpc.mapped_deadlines, [])
                self.assertEqual(rpc.calls, [])

    def test_execution_requires_an_explicit_valid_delivery_ttl(self):
        for ttl in (None, False, True, 0., -.1, .250001, math.inf, math.nan, "0.12"):
            with self.subTest(ttl=ttl):
                with self.assertRaisesRegex(ValueError, "tx_ttl_s"):
                    self.make_backend(ttl=ttl)
        for ttl in (.12, .25):
            self.make_backend(ttl=ttl)

    def test_readonly_needs_no_delivery_ttl_and_cannot_send(self):
        clock = FakeClock()
        rpc = FakeRPC(clock)
        backend = PiperBackend({"execute": False},
            {"transport": {"expected_identity": {}}}, client=rpc)
        with self.assertRaisesRegex(GuardError, "No armed"):
            self.send(backend, CheckedCommand(1, (0.,)*7, 10., 10.4), clock)
        self.assertEqual(rpc.calls, [])

    def test_stale_source_and_out_of_bounds_target_still_fail_before_transport(self):
        limits = GuardLimits(bounds=((-1., 1.),)*7, max_step=(1.,)*7,
            max_target_speed=(10.,)*7, max_tracking_error=(1.,)*7,
            max_observation_age_s=.4, max_command_age_s=.4, max_command_gap_s=.75)
        ready = Readiness(**{name: "SYNTHETIC OFFLINE EVIDENCE"
                             for name in Readiness.__dataclass_fields__})
        for target, now in [((0.,)*7, 10.401), ((2.,)+(0.,)*6, 10.159)]:
            with self.subTest(target=target, now=now):
                guard = ActionGuard(limits)
                guard.arm((0.,)*7, sampled_at=10., now=10., readiness=ready)
                backend, rpc, clock = self.make_backend(clock=FakeClock(now))
                with self.assertRaises(GuardError):
                    command = guard.prepare(target, observed_state=(0.,)*7,
                        sampled_at=10., proposal_at=10., now=now)
                    self.send(backend, command, clock)
                self.assertFalse(guard.armed)
                self.assertEqual(rpc.calls, [])


if __name__ == "__main__":
    unittest.main()
