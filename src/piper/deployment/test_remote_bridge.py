"""No SocketCAN is opened. The bridge state machine uses fake socket/clock."""

from dataclasses import asdict, replace
from pathlib import Path
import struct
import tempfile
import unittest

from lerobot_robot_piper_executor.guard import GuardError, GuardLimits, LocalOwner, Readiness
from lerobot_robot_piper_executor.remote_bridge import ControlEngine, ExecutionPolicy, TX_IDS, readable_link
from lerobot_robot_piper_executor.sdk_codec import encode_target


class FakeClock:
    def __init__(self): self.now = 0.
    def __call__(self): return self.now


class FakeSocket:
    def __init__(self): self.frames, self.fail_at = [], None
    def send_frame(self, can_id, data):
        if self.fail_at is not None and len(self.frames) == self.fail_at:
            self.fail_at = None
            raise BlockingIOError("synthetic socket backpressure")
        self.frames.append((can_id, data))


def policy():
    # Synthetic test values, NOT Piper hardware limits.
    readiness = Readiness(**{name: "OFFLINE SYNTHETIC TEST" for name in Readiness.__dataclass_fields__})
    limits = GuardLimits(bounds=((-10., 10.),)*6+((-.01, .1),), max_step=(1.,)*6+(.002,),
        max_target_speed=(20.,)*6+(.04,), max_tracking_error=(2.,)*6+(.004,),
        max_observation_age_s=.15, max_command_age_s=.15, max_command_gap_s=.25)
    return ExecutionPolicy(execute=True, readiness=readiness, limits=limits,
        expected_motion_ctrl2=(1, 1, 5, 0, 0, 0, 0, 0),
        stop_strategy="fresh_feedback_target_once", stop_gripper_force_n=1., max_gripper_force_n=2.)


def target_frames(joint1=.5):
    return [{"id": f.arbitration_id, "data": f.data.hex()} for f in
        encode_target([joint1, 0., 0., 0., 0., 0., 0.], gripper_force_n=1.)]


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.clock, self.socket = FakeClock(), FakeSocket()
        self.engine = self.new_engine()
        self.prepare(self.engine)

    def new_engine(self, execute_policy=None):
        return ControlEngine(policy=execute_policy or policy(), clock=self.clock,
            send_frame=self.socket.send_frame, owner=LocalOwner(Path(self.folder.name)/"remote-owner"))

    def feedback(self, engine=None, *, fault=0, gripper_status=0x40):
        engine = engine or self.engine
        # Represent a complete, already received CAN cohort at the fake clock.
        # Individual-group skew/fault arrivals are tested separately below.
        for can_id in (0x2A5, 0x2A6, 0x2A7):
            engine.parts[can_id] = (struct.pack(">ii", 0, 0), self.clock())
        engine.parts[0x2A8] = (struct.pack(">ihBB", 0, 0, gripper_status, 0), self.clock())
        engine.ingest(0x2A1, bytes([1, fault, 1, 0, 0, 0, 0, 0]), self.clock())

    def prepare(self, engine):
        self.clock.now = 0.
        engine.ingest(0x151, bytes(policy().expected_motion_ctrl2), self.clock())
        self.clock.now = 2.
        self.feedback(engine)
        engine.set_link({"up": True, "listen_only": False, "bitrate_ok": True})

    def arm(self):
        return self.engine.arm(asdict(self.engine.policy.readiness))["lease"]

    def command(self, lease, frames=None, deadline=None):
        self.clock.now = 2.05
        self.feedback()
        return self.engine.target(lease=lease, sequence=1,
            deadline_ns=int((self.clock()+.1 if deadline is None else deadline)*1e9),
            frames=target_frames() if frames is None else frames)

    def test_rx_default_has_no_tx_or_arm(self):
        engine = self.new_engine(ExecutionPolicy())
        self.prepare(engine)
        self.assertEqual(engine.snapshot()["state"], (0.,)*7)
        with self.assertRaisesRegex(GuardError, "Read-only"):
            engine.arm({})
        with self.assertRaises(GuardError):
            engine.target(lease="", sequence=1, deadline_ns=1, frames=target_frames())
        self.assertEqual(self.socket.frames, [])

    def test_readonly_link_accepts_both_modes_but_execute_arm_requires_normal(self):
        for listen in (True, False):
            self.assertTrue(readable_link({"up": True, "bitrate_ok": True, "listen_only": listen}))
        self.assertFalse(readable_link({"up": False, "bitrate_ok": True, "listen_only": False}))
        self.assertFalse(readable_link({"up": True, "bitrate_ok": False, "listen_only": True}))
        self.engine.set_link({"up": True, "bitrate_ok": True, "listen_only": True})
        with self.assertRaisesRegex(GuardError, "normal-mode"):
            self.arm()
        self.assertEqual(self.socket.frames, [])

    def test_arm_does_not_move_then_accepts_exact_transaction(self):
        lease = self.arm()
        self.assertEqual(self.socket.frames, [])
        result = self.command(lease)
        self.assertEqual(result["accepted_sequence"], 1)
        self.assertEqual([i for i, _ in self.socket.frames], list(TX_IDS))
        self.engine.revoke("test end", hold=False)

    def test_unknown_can_id_rejected_without_transmitting_it(self):
        lease = self.arm()
        frames = target_frames()
        frames[0]["id"] = 0x150
        with self.assertRaisesRegex(GuardError, "CAN ID"):
            self.command(lease, frames)
        self.assertNotIn(0x150, [i for i, _ in self.socket.frames])
        # Only the explicitly reviewed fresh-state hold is allowed on failure.
        self.assertEqual([i for i, _ in self.socket.frames], list(TX_IDS))
        self.assertIsNone(self.engine.lease)

    def test_stale_feedback_prevents_arm_and_target(self):
        self.clock.now = 2.11
        with self.assertRaisesRegex(GuardError, "stale"):
            self.arm()
        self.prepare(self.engine)
        lease = self.arm()
        self.clock.now = 2.11
        with self.assertRaises(GuardError):
            self.engine.target(lease=lease, sequence=1, deadline_ns=2_200_000_000, frames=target_frames())
        self.assertEqual(self.socket.frames, [])
        self.assertIsNone(self.engine.lease)

    def test_single_feedback_group_skew_revokes_without_tx(self):
        self.arm()
        self.clock.now = 2.05
        self.engine.ingest(0x2A5, struct.pack(">ii", 0, 0), self.clock())
        self.assertIsNone(self.engine.lease)
        self.assertEqual(self.socket.frames, [])

    def test_deadline_expiry_holds_latest_fresh_feedback_once(self):
        lease = self.arm()
        with self.assertRaisesRegex(GuardError, "expired"):
            self.command(lease, deadline=2.04)
        self.assertEqual(self.socket.frames[0][1], struct.pack(">ii", 0, 0))
        self.assertEqual(len(self.socket.frames), 4)
        self.engine.tick()
        self.assertEqual(len(self.socket.frames), 4)

    def test_lease_timeout_independent_of_client_holds_once(self):
        self.arm()
        self.clock.now = 2.26
        self.feedback()
        self.engine.tick()
        self.assertIsNone(self.engine.lease)
        self.assertEqual(len(self.socket.frames), 4)
        self.engine.tick()
        self.assertEqual(len(self.socket.frames), 4)

    def test_disconnect_health_gate_and_no_reset_disable(self):
        self.arm()
        self.engine.revoke("SSH EOF", hold=True)
        self.assertEqual([i for i, _ in self.socket.frames], list(TX_IDS))
        self.assertTrue(all(i not in (0x150, 0x151, 0x470) for i, _ in self.socket.frames))
        self.engine = self.new_engine()
        self.prepare(self.engine)
        self.arm()
        self.feedback(fault=1)
        self.engine.revoke("SSH EOF after fault", hold=True)
        self.assertEqual(len(self.socket.frames), 4)

    def test_partial_batch_faults_and_overrides_from_fresh_feedback(self):
        lease = self.arm()
        self.socket.fail_at = 1
        with self.assertRaises(BlockingIOError): self.command(lease)
        self.assertIsNone(self.engine.lease)
        self.assertEqual(self.socket.frames[0][1], struct.pack(">ii", 500, 0))
        self.assertEqual(self.socket.frames[1][1], struct.pack(">ii", 0, 0))
        self.assertEqual(len(self.socket.frames), 5)

    def test_competing_owner_and_foreign_controller(self):
        self.arm()
        other = self.new_engine()
        self.prepare(other)
        with self.assertRaisesRegex(GuardError, "owns"):
            other.arm(asdict(other.policy.readiness))
        self.engine.ingest(0x155, bytes(8), self.clock())
        self.assertIsNone(self.engine.lease)
        self.assertEqual(self.socket.frames, [])
        self.clock.now += 1.1
        self.feedback(other)
        other.set_link({"up": True, "listen_only": False, "bitrate_ok": True})
        other.arm(asdict(other.policy.readiness))
        other.revoke("test end", hold=False)

    def test_mode_evidence_and_stop_review_required(self):
        self.engine.last_motion_ctrl2 = (bytes([1, 1, 5, 0xad, 0, 0, 0, 0]), 0.)
        with self.assertRaisesRegex(GuardError, "MotionCtrl2"): self.arm()
        with self.assertRaisesRegex(GuardError, "unconfirmed"):
            replace(policy(), stop_strategy="unconfirmed").validate_execute()

    def test_angle_feedback_is_not_mislabeled_as_width(self):
        self.engine.parts[0x2A8] = (struct.pack(">ihBB", 1000, 0, 0x40, 1), self.clock())
        snapshot = self.engine.snapshot()
        self.assertIsNone(snapshot["state"][6])
        self.assertEqual(snapshot["status"]["gripper_raw"], 1000)
        self.assertFalse(snapshot["healthy"])
        with self.assertRaisesRegex(GuardError, "gripper"):
            self.arm()


if __name__ == "__main__":
    unittest.main()
