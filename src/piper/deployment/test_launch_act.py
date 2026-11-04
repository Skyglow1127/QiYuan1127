"""Offline checks only: no SSH, CAN or camera connection."""
import json
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import launch_act


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.defaults = launch_act.read(launch_act.DEFAULTS)
        self.legacy_defaults = {key: value for key, value in self.defaults.items()
                               if key not in ('policy_joint_target_margin_deg', 'policy_gripper_target_margin_m')}
        self.legacy_defaults.update(j5_upper_prediction_saturation=True, j5_upper_target_margin_deg=2.)
        self.base = launch_act.read(self.defaults["base_io_config"])
        self.probe = {"identity": dict(self.base["transport"]["expected_identity"]),
            "complete_fresh_feedback": True, "motor_faults": [0]*6,
            "enabled": [False]*6, "arm_status_raw": "0000000000000000",
            "gripper_raw": "000026ac00000000",
            "state": [2.383,-1.995,1.918,-4.365,25.037,2.144,.0099]}

    def test_build_uses_fresh_pose_and_one_new_identity_without_hardware(self):
        self.probe["identity"]["ifindex"] = 99
        with tempfile.TemporaryDirectory() as temp, patch("subprocess.run", side_effect=AssertionError("hardware called")):
            path = Path(temp)
            review = launch_act.build_run(self.defaults, self.base, self.probe, path, "10.42.0.55")
            self.assertEqual(review["initial_target_deg"][1:3], [1., -1.])
            self.assertEqual(review["initial_target_deg"][4], self.probe["state"][4])
            self.assertEqual(review["gripper_hold_force_n"], 1.)
            config = launch_act.read(path/"execute_config.json")
            self.assertEqual(config["expected_ifindex"], 99)
            self.assertEqual(config["tx_ttl_s"], .12)
            self.assertEqual(config["policy_target_rate_fraction"], .9)
            self.assertEqual(config["policy_joint_target_margin_deg"], 2.)
            self.assertEqual(config["policy_gripper_target_margin_m"], .001)
            self.assertNotIn("j5_upper_prediction_saturation", config)
            self.assertNotIn("j5_upper_target_margin_deg", config)
            self.assertNotIn("j5_upper_prediction_tolerance_deg", config)
            self.assertEqual(config["policy_target_trace_path"], str((path/"policy_targets.jsonl").resolve()))
            policy = launch_act.read(path/"act_trial_policy.json")
            self.assertEqual(policy["limits"], config["guard_limits"])
            self.assertEqual(config["guard_limits"]["bounds"][:2], [[-60., 40.], [0., 135.]])
            self.assertEqual(policy["readiness"], launch_act.read(path/"readiness.json"))
            self.assertEqual(policy["expected_motion_ctrl2"], [1,1,5,0,0,0,0,0])
            self.assertEqual(config["expected_motion_ctrl2"], policy["expected_motion_ctrl2"])

    def test_invalid_policy_speed_fraction_rejects_before_writing_run(self):
        for fraction in (True, 0., -.1, .96, float('nan'), float('inf'), '0.9'):
            with self.subTest(fraction=fraction), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'policy_target_rate_fraction'):
                    launch_act.build_run({**self.defaults, 'policy_target_rate_fraction': fraction},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])

    def test_missing_faulted_or_partial_enable_feedback_rejects(self):
        for change in ({"complete_fresh_feedback": False}, {"enabled": [True]*6},
                       {"motor_faults": [1,0,0,0,0,0]}, {"enabled": [True,False,False,False,False,False]}):
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                launch_act.require_start_pose({**self.probe, **change})

    def test_enabled_healthy_arm_builds_takeover_without_recovery_or_enable(self):
        sample = {**self.probe, "enabled": [True]*6, "arm_status_raw": "0100010000000000",
                  "state": [0.,0.,0.,0.,0.,0.,-.0004]}
        for speed in (5, 10):
            with self.subTest(speed=speed), tempfile.TemporaryDirectory() as temp:
                defaults = {**self.defaults, "enabled_takeover_speed_percent": speed}
                review = launch_act.build_run(defaults, self.base, sample, Path(temp), "10.42.0.55")
                self.assertTrue(review["reuse_enabled"])
                self.assertNotIn("initial_target_deg", review)
                self.assertNotIn("gripper_hold_force_n", review)
                self.assertEqual(review["warm_gripper_target_m"], .01)
                self.assertEqual(review["takeover_speed_percent"], speed)
                local = launch_act.read(Path(temp)/"execute_config.json")
                remote = launch_act.read(Path(temp)/"act_trial_policy.json")
                self.assertEqual(local["expected_motion_ctrl2"], [1,1,speed,0,0,0,0,0])
                self.assertEqual(local["expected_motion_ctrl2"], remote["expected_motion_ctrl2"])
                self.assertEqual(local["guard_limits"], remote["limits"])
                self.assertEqual(remote["limits"]["bounds"][4], [-60.,70.])

    def test_invalid_j5_prediction_tolerance_rejects_before_writing_run(self):
        legacy = {key: value for key, value in self.legacy_defaults.items()
                  if key not in ('j5_upper_prediction_saturation', 'j5_upper_target_margin_deg')}
        for tolerance in (True, -0.1, 1.001, float('nan'), float('inf'), '1'):
            with self.subTest(tolerance=tolerance), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'j5_upper_prediction_tolerance_deg'):
                    launch_act.build_run({**legacy, 'j5_upper_prediction_tolerance_deg': tolerance},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])

    def test_invalid_or_ambiguous_j5_prediction_mode_rejects_before_writing_run(self):
        for change in ({'j5_upper_prediction_saturation': value} for value in (1, 0, 'true', None)):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'j5_upper_prediction_saturation'):
                    launch_act.build_run({**self.legacy_defaults, **change},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])
        for enabled in (True, False):
            with self.subTest(enabled=enabled), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'mutually exclusive'):
                    launch_act.build_run({**self.legacy_defaults, 'j5_upper_prediction_saturation': enabled,
                        'j5_upper_prediction_tolerance_deg': 1.},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])

    def test_target_margin_rejects_invalid_or_disabled_mode_before_writing(self):
        for change in ({'j5_upper_target_margin_deg': -.1},
                       {'j5_upper_target_margin_deg': True},
                       {'j5_upper_target_margin_deg': float('nan')},
                       {'j5_upper_target_margin_deg': 5.001},
                       {'j5_upper_prediction_saturation': False}):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'j5_upper_target_margin_deg'):
                    launch_act.build_run({**self.legacy_defaults, **change},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])

    def test_legacy_tolerance_config_does_not_gain_new_target_margin(self):
        defaults = {key: value for key, value in self.legacy_defaults.items()
                    if key not in ('j5_upper_prediction_saturation', 'j5_upper_target_margin_deg')}
        defaults['j5_upper_prediction_tolerance_deg'] = 1.
        with tempfile.TemporaryDirectory() as temp:
            launch_act.build_run(defaults, self.base, self.probe, Path(temp), '10.42.0.55')
            config = launch_act.read(Path(temp)/'execute_config.json')
            self.assertEqual(config['j5_upper_prediction_tolerance_deg'], 1.)
            self.assertNotIn('j5_upper_target_margin_deg', config)
            self.assertNotIn('j5_upper_prediction_saturation', config)
            self.assertEqual(config['guard_limits']['bounds'][4], [-60., 70.])

    def test_invalid_takeover_speed_rejects_before_writing_run(self):
        for speed in (True, 0, 11, 10., '10'):
            with self.subTest(speed=speed), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'enabled_takeover_speed_percent'):
                    launch_act.build_run({**self.defaults, 'enabled_takeover_speed_percent': speed},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])

    def test_joint_margin_rejects_invalid_or_legacy_combination_before_writing(self):
        for change in ({'policy_joint_target_margin_deg': value} for value in
                       (True, None, '2', -.1, 5.001, float('nan'), float('inf'))):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'policy_joint_target_margin_deg'):
                    launch_act.build_run({**self.defaults, **change},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, 'mutually exclusive'):
                launch_act.build_run({**self.defaults, 'j5_upper_prediction_saturation': True},
                    self.base, self.probe, Path(temp), '10.42.0.55')
            self.assertEqual(list(Path(temp).iterdir()), [])

    def test_new_joint_mode_passes_wrapper_gate_without_hardware(self):
        from lerobot_robot_piper_executor.guard import Readiness, GuardLimits
        source = Path(__file__).resolve().parent.parent/'official_adapter/rollout_act_piper.py'
        spec = importlib.util.spec_from_file_location('joint_projection_wrapper_gate_test', source)
        wrapper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(wrapper)
        with tempfile.TemporaryDirectory() as temp, patch('subprocess.run', side_effect=AssertionError('hardware called')):
            path = Path(temp)
            launch_act.build_run(self.defaults, self.base, self.probe, path, '10.42.0.55')
            config = launch_act.read(path/'execute_config.json')
            args = SimpleNamespace(readiness_reviewed=True, readiness=path/'readiness.json')
            _, missing, _ = wrapper.local_gate_status(args, config, Readiness, GuardLimits)
            self.assertEqual(missing, [])
            with self.assertRaisesRegex(ValueError, 'mutually exclusive'):
                wrapper.local_gate_status(args, {**config, 'j5_upper_prediction_saturation': True},
                    Readiness, GuardLimits)

    def test_gripper_margin_rejects_invalid_or_missing_joint_mode_before_writing(self):
        for value in (True, None, '0.001', -.000001, .005001, float('nan'), float('inf')):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as temp:
                with self.assertRaisesRegex(ValueError, 'policy_gripper_target_margin_m'):
                    launch_act.build_run({**self.defaults, 'policy_gripper_target_margin_m': value},
                        self.base, self.probe, Path(temp), '10.42.0.55')
                self.assertEqual(list(Path(temp).iterdir()), [])
        bad = {key: value for key, value in self.defaults.items() if key != 'policy_joint_target_margin_deg'}
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, 'requires joint prediction projection'):
                launch_act.build_run(bad, self.base, self.probe, Path(temp), '10.42.0.55')
            self.assertEqual(list(Path(temp).iterdir()), [])

    def test_wrapper_rejects_invalid_or_missing_joint_mode_for_gripper_projection(self):
        from lerobot_robot_piper_executor.guard import Readiness, GuardLimits
        source = Path(__file__).resolve().parent.parent/'official_adapter/rollout_act_piper.py'
        spec = importlib.util.spec_from_file_location('gripper_projection_wrapper_gate_test', source)
        wrapper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(wrapper)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            launch_act.build_run(self.defaults, self.base, self.probe, path, '10.42.0.55')
            config = launch_act.read(path/'execute_config.json')
            args = SimpleNamespace(readiness_reviewed=True, readiness=path/'readiness.json')
            for value in (True, None, '0.001', -.000001, .005001, float('nan'), float('inf')):
                with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'policy_gripper_target_margin_m'):
                    wrapper.local_gate_status(args, {**config, 'policy_gripper_target_margin_m': value},
                        Readiness, GuardLimits)
            del config['policy_joint_target_margin_deg']
            with self.assertRaisesRegex(ValueError, 'requires joint prediction projection'):
                wrapper.local_gate_status(args, config, Readiness, GuardLimits)

    def test_remote_probe_is_syntax_valid_and_read_only(self):
        compile(launch_act.PROBE, "remote_probe", "exec")
        self.assertNotIn(".send(", launch_act.PROBE)
        self.assertNotIn("ip link set", launch_act.PROBE)


if __name__ == "__main__":
    unittest.main()
