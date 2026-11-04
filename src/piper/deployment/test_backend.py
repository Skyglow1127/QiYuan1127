"""AIPC transport tests without SSH or hardware."""

from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

from lerobot_robot_piper_executor.backend import PiperBackend, RPCClient, ReceiverClock, ssh_command
from lerobot_robot_piper_executor.guard import CheckedCommand, GuardError
from lerobot_robot_piper_executor.remote_bridge import PROTOCOL
from test_remote_bridge import policy


class ClientTests(unittest.TestCase):
    def test_clock_epoch_cannot_reset_while_a_robot_lease_exists(self):
        backend = object.__new__(PiperBackend)
        backend.lease = "synthetic-active-lease"
        with self.assertRaisesRegex(GuardError,"during an execution lease"):
            backend.refresh_unarmed_clock()

    def test_receiver_clock_allows_measured_slew_but_rejects_real_jump(self):
        clock = ReceiverClock()
        clock.update(1_000_000_000,1_001_000_000,2_000_400_000,2_000_500_000)
        # 500ppm offset change across3s remains bounded; source age grows
        # conservatively and cannot be refreshed by this correction.
        clock.update(4_000_000_000,4_001_000_000,4_998_900_000,4_999_000_000)
        self.assertLessEqual(clock.host_time(4_999_000_000,4_001_000_000),4.001)
        clock.update(6_040_000_000,6_041_000_000,7_026_700_000,7_026_800_000)
        # Reject a shift larger than the explicit1% relative-rate bound.
        with self.assertRaises(GuardError):
            clock.update(7_040_000_000,7_041_000_000,8_100_400_000,8_100_500_000)

    def test_rpc_over_local_fake_stdio_process_and_pinned_identity(self):
        identity = {"interface": "SYNTHETIC", "host": "OFFLINE TEST"}
        # A local stdlib JSON peer, not ssh, and no SocketCAN imports/calls.
        script = """
import json,sys,time
p = 'piper-deploy-rpc-v1'
listen=bool(int(sys.argv[1]))
print(json.dumps({'protocol':p,'type':'hello','identity':{'interface':'SYNTHETIC','host':'OFFLINE TEST'},'execute':False,'kernel_timestamps':True,'link':{'ready':listen,'up':True,'listen_only':listen,'bitrate_ok':True}}),flush=True)
for line in sys.stdin:
    r=json.loads(line); t=time.monotonic_ns()
    if r['op']=='ping': result={'remote_receive_ns':t,'remote_send_ns':time.monotonic_ns()}
    else: result={'state':[0,0,0,0,0,0,0],'source_ns':time.monotonic_ns()}
    print(json.dumps({'protocol':p,'type':'reply','id':r['id'],'ok':True,'result':result}),flush=True)
"""
        for listen in (True, False):
            with self.subTest(listen_only=listen):
                client = RPCClient([sys.executable, "-u", "-c", script, str(int(listen))], identity)
                try:
                    client.open()
                    snapshot = client.request("snapshot")
                    self.assertEqual(snapshot["state"], [0]*7)
                    self.assertLessEqual(client.host_source_time(snapshot["source_ns"]), time.monotonic())
                    self.assertFalse(client.hello["execute"])
                finally:
                    client.close()
        wrong = RPCClient([sys.executable, "-u", "-c", script, "1"], {"wrong": "identity"})
        with self.assertRaisesRegex(GuardError, "identity"):
            wrong.open()
        self.assertIsNone(wrong.process)

    def test_deadline_uses_lower_offset_and_never_extends_ttl(self):
        client = RPCClient([], {})
        client.source_clock = ReceiverClock()
        client.source_clock.update(1_000_000_000, 1_020_000_000, 1_205_000_000, 1_206_000_000)
        with patch("lerobot_robot_piper_executor.backend.time.monotonic_ns", return_value=1_200_000_000):
            deadline = client.remote_deadline(2.1)
        self.assertLessEqual(deadline, 2_286_000_000)
        self.assertGreater(deadline, 2_200_000_000)

    def test_default_ssh_has_no_execution_flag_or_interface_mutation(self):
        root = Path(__file__).parent
        config = json.loads((root/"observe_config.json").read_text())
        io = json.loads(Path(config["io_config"]).read_text())
        command = ssh_command(config, io)
        self.assertNotIn("--execution-policy", command[-1])
        self.assertNotIn("ip link", command[-1])
        self.assertIn("StrictHostKeyChecking=yes", command)
        config["execute"] = True
        with self.assertRaises(GuardError): ssh_command(config, io)

    def test_backend_default_never_arms_or_encodes_targets(self):
        class FakeClient:
            def __init__(self): self.calls = []
            def request(self, op, **kwargs): self.calls.append(op)
        fake = FakeClient()
        backend = PiperBackend({"execute": False},
            {"transport": {"expected_identity": {"ifindex": 15}}}, client=fake)
        with self.assertRaisesRegex(GuardError, "Read-only"):
            backend.require_execution_lease(policy().readiness)
        with self.assertRaises(GuardError):
            backend.send_checked(CheckedCommand(1, (0.,)*7, 1., 1.1))
        self.assertEqual(fake.calls, [])


if __name__ == "__main__":
    unittest.main()
