"""Approval and parameter boundaries for the desktop UI (no hardware access)."""
import math
import json
import tempfile
import unittest
from unittest.mock import Mock
from pathlib import Path

from app.gui import DEFAULT_SETTINGS, ENABLE_EXPLANATION, FIELDS, RESULT_NAMES, STAGE_NAMES, GraspApp, device_status_text, execution_blocker, parse_settings, planning_reference_notices, preparation_result, stage_samples


class GuiApprovalTests(unittest.TestCase):
    def setUp(self):
        self.review = {"ready": True, "approval_sha256": "a" * 64, "expires_at": 120.0}

    def allowed(self, review=None, **overrides):
        state = {"used": False, "changed": False, "busy": False, "now": 100.0}
        state.update(overrides)
        return execution_blocker(self.review if review is None else review, **state)

    def test_review_can_be_used_only_while_current_and_unconsumed(self):
        self.assertEqual(self.allowed(), "")
        for flag in ("used", "changed", "busy"):
            with self.subTest(flag=flag):
                self.assertNotEqual(self.allowed(**{flag: True}), "")
        self.assertNotEqual(self.allowed(now=120.0), "")
        self.assertNotEqual(self.allowed(now=121.0), "")

    def test_malformed_or_unready_review_is_not_approval(self):
        for changes in ({"ready": False}, {"blockers": ["设备未连接"]}, {"approval_sha256": ""}, {"expires_at": None}, {"expires_at": "invalid"}, {"expires_at": math.nan}):
            with self.subTest(changes=changes):
                self.assertNotEqual(self.allowed({**self.review, **changes}), "")

    def test_no_fixed_joint_speed_ceiling(self):
        for speed in (0.01, 0.5, 1.0, 20.0, 360.0, 100000.0):
            self.assertEqual(parse_settings({**DEFAULT_SETTINGS, "speed_deg_s": str(speed)})["speed_deg_s"], speed)

    def test_communication_rate_defaults_to_ten_and_keeps_explicit_lower_rate(self):
        from app.backend import DEFAULTS, settings_checked
        from app.fixed_pose import FIXED_MOTION_DEFAULTS
        self.assertEqual(DEFAULT_SETTINGS['rate_hz'],10.)
        self.assertEqual(DEFAULTS['rate_hz'],10.)
        self.assertEqual(FIXED_MOTION_DEFAULTS['rate_hz'],10.)
        self.assertIn('rate_hz',{field[0] for field in FIELDS})
        self.assertEqual(parse_settings({**DEFAULT_SETTINGS,'rate_hz':'5'})['rate_hz'],5.)
        self.assertEqual(settings_checked({'rate_hz':5.})['rate_hz'],5.)
        with self.assertRaises(ValueError):parse_settings({**DEFAULT_SETTINGS,'rate_hz':10.1})

    def test_all_parameters_reject_nonpositive_and_nonfinite_inputs(self):
        for key in DEFAULT_SETTINGS:
            for invalid in ("", "text", 0, -1, math.inf, -math.inf, math.nan):
                with self.subTest(key=key, value=invalid):
                    with self.assertRaises(ValueError):
                        parse_settings({**DEFAULT_SETTINGS, key: invalid})

    def test_chart_uses_supplied_joint_waypoints(self):
        stage = {"duration_s": 2, "sdk_plan": {"sdk": [[1, 2, 3], [4, 5, 6]]}}
        self.assertEqual(stage_samples(stage), [(0.0, [1.0, 2.0, 3.0]), (2.0, [4.0, 5.0, 6.0])])
        self.assertEqual(stage_samples({"duration_s": 2}), [])

    def test_chart_preserves_nonuniform_planned_timing(self):
        stage = {"duration_s": 10, "sdk_plan": {"sdk": [[0], [5], [6]], "waypoint_times_s": [0, 8, 10]}}
        self.assertEqual([t for t, _q in stage_samples(stage)], [0, 8, 10])
        stage["sdk_plan"].pop("waypoint_times_s")
        stage["sdk_plan"].update(ticks=[40, 10], rate=5)
        self.assertEqual([t for t, _q in stage_samples(stage)], [0, 8, 10])


class GuiPreparationFailureTests(unittest.TestCase):
    def test_failure_uses_saved_result_without_a_result_event(self):
        with tempfile.TemporaryDirectory() as directory:
            saved = {"status": "blocked", "total_time_s": 1.8, "error": "saved detail"}
            (Path(directory) / "result.json").write_text(json.dumps(saved), encoding="utf-8")
            result = preparation_result({"ready": False, "run_dir": directory})
        self.assertEqual(result["total_time_s"], 1.8)
        self.assertEqual(result["error"], "saved detail")
        self.assertEqual(result["grasp_result"], "not_attempted")
        self.assertEqual(result["cycle_result"], "not_attempted")

    def test_review_result_precedes_asynchronous_event_and_keeps_all_blockers(self):
        blockers = ["红块下沿被裁掉", "J1=-90.1° 超出现有限位", "控制协议标志 enable_status=0", "夹爪位置不符"]
        review = {"ready": False, "result": {"status": "blocked", "total_time_s": 1.8},
                  "blockers": blockers, "observed_joint_angles": [0, -90.1, 0, 0, 0, 0, 35],
                  "full_log": "/tmp/prepare.log"}
        result = preparation_result(review, {"status": "failed", "total_time_s": 7.2})
        self.assertEqual(result["total_time_s"], 1.8)
        self.assertEqual(result["observed_joint_angles"], review["observed_joint_angles"])
        for reason in blockers:
            self.assertIn(reason, result["error"])
        self.assertIn("/tmp/prepare.log", result["error"])
        self.assertNotIn("grasp_result", review["result"])

    def test_missing_or_damaged_result_still_reports_not_attempted(self):
        with tempfile.TemporaryDirectory() as directory:
            review = {"ready": False, "run_dir": directory, "blockers": ["设备未就绪"], "preparation_time_s": 2.0}
            for text in (None, "{broken", "[]"):
                if text is not None:
                    (Path(directory) / "result.json").write_text(text, encoding="utf-8")
                result = preparation_result(review)
                self.assertEqual(result["status"], "blocked")
                self.assertEqual(result["cycle_result"], "not_attempted")
                self.assertEqual(result["total_time_s"], 2.0)

    def test_ready_review_has_no_failure_result(self):
        self.assertIsNone(preparation_result({"ready": True, "blockers": []}))

    def test_initial_gripper_adjustment_is_an_explained_target(self):
        self.assertEqual(dict((key,label) for key,label,*_rest in FIELDS)['open_target'], '抓取前张开目标')
        self.assertEqual(STAGE_NAMES['prepare_gripper'], '抓取前调整夹爪')
        review={'ready':False,'blockers':['目标未找到'],'notices':['实际 6.50 → 目标 36.00']}
        self.assertEqual(preparation_result(review)['notices'],review['notices'])

    def test_nominal_planning_reference_keeps_raw_readback_visible(self):
        adjustment={'joint':1,'measured_sdk':-90.099998,'nominal_sdk':-90.,'difference_deg':.099998}
        review={'stages':[{'start_reference_adjustments':[adjustment]}, {'start_reference_adjustments':[adjustment]}]}
        notices=planning_reference_notices(review)
        self.assertEqual(len(notices),1)
        self.assertIn('J1 实际反馈 -90.1°，规划参考 -90°',notices[0])
        self.assertIn('0.2° 反馈容差，原始反馈保留；目标限位不变',notices[0])
        self.assertEqual(RESULT_NAMES['not_completed'],'未完成')


class GuiDeviceControlTests(unittest.TestCase):
    def test_status_requires_actual_fresh_acknowledged_values(self):
        labels, fresh = device_status_text(None, now=100)
        self.assertFalse(fresh)
        self.assertEqual(labels["enable_status"], "enable_status: 未知")
        # A successful write without feedback does not imply power or enable.
        labels, fresh = device_status_text({"completed": True, "action": "power_on"}, now=100)
        self.assertFalse(fresh)
        self.assertEqual(labels["power_status"], "power_status: 未知")
        snapshot = {"status": {"power_status": 1, "enable_status": 0}, "status_fresh": True, "snapshot_received_at": 99}
        labels, fresh = device_status_text(snapshot, now=100)
        self.assertTrue(fresh)
        self.assertEqual(labels, {"power_status": "power_status: 1", "enable_status": "enable_status: 0"})

    def test_stale_and_unknown_status_are_distinct(self):
        snapshot = {"status": {"power_status": 1, "enable_status": 0}, "status_fresh": True, "snapshot_received_at": 90}
        labels, fresh = device_status_text(snapshot, now=100)
        self.assertFalse(fresh)
        self.assertIn("1 · 上次", labels["power_status"])
        for stamp in (None, float("nan"), 200):
            labels, fresh = device_status_text({**snapshot, "snapshot_received_at": stamp}, now=100)
            self.assertFalse(fresh)
            self.assertIn("上次", labels["enable_status"])
        labels, fresh = device_status_text({**snapshot, "status": {"enable_status": "missing"}}, now=100)
        self.assertEqual(labels["enable_status"], "enable_status: 未知")

    def app_stub(self):
        app = GraspApp.__new__(GraspApp)
        app.busy = ""
        app.closing = False
        app.review = {"ready": True, "approval_sha256": "a" * 64}
        app.review_used = True
        app.parameters_changed = True
        app.device_snapshot = {"status": {"power_status": 0, "enable_status": 0}}
        for name in ("signature_label", "stage_detail", "state_label", "backend", "result_text", "notebook"):
            setattr(app, name, Mock())
        app._draw_path = Mock()
        app._append_log = Mock()
        app._start_job = Mock()
        app._set_text = Mock()
        return app

    def test_busy_window_does_not_start_parallel_device_work(self):
        for busy, closing in (("prepare", False), ("execute", False), ("device_control", False), ("", True)):
            app = self.app_stub()
            app.busy, app.closing = busy, closing
            app.control_device("power_on")
            app._start_job.assert_not_called()
            app.backend.control_device.assert_not_called()
            self.assertIsNotNone(app.review)

    def test_explicit_click_invalidates_review_and_dispatches_only_requested_action(self):
        for action in ("power_on",):
            app = self.app_stub()
            previous = app.device_snapshot.copy()
            app.control_device(action)
            self.assertIsNone(app.review)
            self.assertFalse(app.review_used)
            self.assertEqual(app.device_snapshot, previous)  # No optimistic protocol state.
            kind, operation = app._start_job.call_args.args
            self.assertEqual(kind, "device_control")
            app.backend.control_device.assert_not_called()  # Queued for worker, never blocking Tk.
            operation()
            app.backend.control_device.assert_called_once_with(action)
            app.backend.execute.assert_not_called()
            app.backend.prepare.assert_not_called()

    def test_enable_button_only_explains_reviewed_position_start_without_device_calls(self):
        app=self.app_stub();review=app.review;snapshot=app.device_snapshot
        app.control_device('enable')
        app._set_text.assert_called_once_with(app.result_text,ENABLE_EXPLANATION)
        app.notebook.select.assert_called_once_with(1)
        app._start_job.assert_not_called()
        app.backend.control_device.assert_not_called()
        app.backend.execute.assert_not_called()
        self.assertIs(app.review,review)
        self.assertIs(app.device_snapshot,snapshot)
        self.assertIn('首个已批准的位置动作',ENABLE_EXPLANATION)
        self.assertIn('不发送阻尼、归零或同位恢复指令',ENABLE_EXPLANATION)

    def test_refresh_keeps_existing_review_and_does_not_prepare_or_execute(self):
        app = self.app_stub()
        review = app.review
        app.control_device("refresh")
        self.assertIs(app.review, review)
        _kind, operation = app._start_job.call_args.args
        operation()
        app.backend.control_device.assert_called_once_with("refresh")
        app.backend.prepare.assert_not_called()
        app.backend.execute.assert_not_called()

    def test_unknown_action_is_not_sent(self):
        app = self.app_stub()
        app.control_device("power_off")
        app._start_job.assert_not_called()
        self.assertIsNotNone(app.review)


if __name__ == "__main__":
    unittest.main()
