"""Chinese desktop UI for the fixed OpenCV grasp procedure.

The displayed preparation is the approval object: execution never refreshes it.
All device work is delegated to Backend on a worker thread.
"""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import queue
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Any

from .execution_scope import PREGRASP, FIXED_PREGRASP, select_execution_stages
from .fixed_pose import FIXED_MOTION_DEFAULTS, fixed_motion_checked


DEFAULT_SETTINGS = {
    "speed_deg_s": 0.5,
    "acceleration_deg_s2": 1.0,
    "gripper_speed_sdk_s": 0.5,
    "rate_hz": 10.0,
    "lift_height_m": 0.10,
    "open_target": 36.0,
    "close_target": 29.6,
}
FIELDS = (
    ("speed_deg_s", "关节速度", "°/s", "0.5"),
    ("acceleration_deg_s2", "关节加速度", "°/s²", "1.0"),
    ("gripper_speed_sdk_s", "夹爪速度", "SDK 单位/s", "0.5"),
    ("rate_hz", "通信频率", "Hz，≤10", "10"),
    ("lift_height_m", "抬升高度", "m", "0.10"),
    ("open_target", "抓取前张开目标", "SDK 单位", "36.0"),
    ("close_target", "夹爪夹紧位置", "SDK 单位", "29.6"),
)
LABELS = {key: label for key, label, _unit, _default in FIELDS}
PALETTE = {
    "bg": "#eef2f5", "paper": "#ffffff", "ink": "#183043",
    "muted": "#607688", "line": "#dce5ec", "navy": "#122c3b",
    "teal": "#087f8c", "teal_active": "#066b76", "teal_pale": "#e6f5f5",
    "red": "#b44040", "red_pale": "#fff0ee", "amber": "#9b640e",
}
JOINT_COLORS = ("#078995", "#496cc4", "#b78027", "#9b56a8", "#d26755", "#508b50", "#748595")
STAGE_NAMES = {"prepare_gripper": "抓取前调整夹爪", "pregrasp": "移动至预抓取点", "approach": "接近抓取点", "close": "闭合夹爪", "lift": "抬升方块", "lower": "放回方块", "open": "张开夹爪", "retreat": "退离目标"}
STAGE_NAMES[FIXED_PREGRASP] = '移动至 Gazebo 固定准备姿态'
PREPARATION_NAMES = {
    "read_only_feedback_discovery_and_stationarity": "读取关节反馈并确认静止",
    "wrist_http_status_and_image": "获取腕部相机状态和图像",
    "saved_image_and_observation_read": "读取保存的图像与反馈",
    "local_opencv_foreground": "本地 OpenCV 红块识别",
    "foreground_http_inference": "前景识别服务",
    "foreground_preprocessed_http_inference": "预处理后前景识别",
    "fixed_plane_silhouette_fit_and_coordinate_transform": "目标定位与坐标转换",
    "board_preprocess_transport_and_compute": "板端预处理传输与计算",
    "npu_task_start_warmup": "启动并预热识别服务",
}
STATUS_NAMES = {"success": "执行完成", "completed": "执行完成", "failed": "执行失败", "cancelled": "已停止后续指令", "canceled": "已停止后续指令", "error": "执行发生错误", "aborted": "执行中止", "demo_completed": "演示完成", "blocked": "准备未通过", "unconfirmed": "结果未确认"}
STATUS_NAMES['pregrasp_completed'] = '已到预抓取点'
RESULT_NAMES = {"success": "成功", "failed": "失败", "failure": "失败", "unknown": "未确认（现有证据不足）", "unverified": "未确认（现有证据不足）", "unconfirmed": "未确认（现有证据不足）", "demo": "演示结果", "not_attempted": "未执行", "not_completed": "未完成", "simulated_success": "模拟成功"}
DISPLAY_ZONE = dt.timezone(dt.timedelta(hours=8))
DEVICE_ACTION_NAMES = {"refresh": "刷新协议状态", "power_on": "协议上电", "enable": "使能说明"}
DEVICE_STATUS_MAX_AGE_S = 3.0
ENABLE_EXPLANATION = (
    "无需单独打开 ENABLE。\n\n"
    "D1 SDK 由位置指令进入位置控制：程序允许 enable_status=0 时准备路径，"
    "在你确认目标、路径和参数后，首个已批准的位置动作启动位置控制，并核对状态与关节反馈。\n\n"
    "本按钮只显示说明，不发送阻尼、归零或同位恢复指令。\n\n"
    "夹爪可以从有效范围内的实际位置开始；“抓取前张开目标”是程序随后调整到的位置，"
    "不是你必须事先调好的初始位置。调整会列入待批准路径。"
)


def device_status_text(snapshot: dict | None, *, now: float | None = None) -> tuple[dict[str, str], bool]:
    """Present acknowledged protocol values, never command intent, as status."""
    snapshot = snapshot if isinstance(snapshot, dict) else {}
    status = snapshot.get("status") if isinstance(snapshot.get("status"), dict) else {}
    stamp = expiry_timestamp(snapshot.get("snapshot_received_at"))
    age = ((time.time() if now is None else now) - stamp) if stamp is not None else None
    known_flags = all(isinstance(status.get(key), (int, float)) and status[key] in (0, 1) for key in ("power_status", "enable_status"))
    fresh = known_flags and bool(snapshot.get("status_fresh")) and age is not None and -1 <= age <= DEVICE_STATUS_MAX_AGE_S
    labels = {}
    for key in ("power_status", "enable_status"):
        value = status.get(key)
        known = isinstance(value, (int, float)) and value in (0, 1)
        labels[key] = f"{key}: {int(value)}" + ("" if fresh else " · 上次") if known else f"{key}: 未知"
    return labels, fresh


def parse_settings(values: dict[str, Any]) -> dict[str, float]:
    """Validate finite positive user parameters, with no artificial speed cap."""
    result = {}
    for key in DEFAULT_SETTINGS:
        try:
            value = float(values[key])
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"{LABELS[key]}需要填写有效数字。") from None
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{LABELS[key]}必须是大于 0 的有限数值。")
        result[key] = value
    if result['rate_hz']>10:
        raise ValueError('通信频率不得超过 10 Hz。')
    return result


def expiry_timestamp(value: Any) -> float | None:
    """Accept epoch seconds or an ISO-8601 expiry returned by the backend."""
    if value is None:
        return None
    try:
        stamp = float(value)
        return stamp if math.isfinite(stamp) else None
    except (TypeError, ValueError):
        try:
            parsed = dt.datetime.fromisoformat(str(value).replace("Z", "+00:00"))
            return parsed.timestamp()
        except (TypeError, ValueError, OverflowError):
            return None


def execution_blocker(review: dict | None, *, used: bool, changed: bool, busy: bool,
                      now: float | None = None) -> str:
    if busy:
        return "程序正在工作"
    if review is None:
        return "请先准备目标与路径"
    if used:
        return "此路径已执行，请重新准备"
    if changed:
        return "参数已修改，请重新准备"
    if not review.get("ready", False) or review.get("blockers"):
        return "准备未通过，请查看原因后重新准备"
    if not review.get("approval_sha256"):
        return "准备结果缺少路径签名，请重新准备"
    expiry = expiry_timestamp(review.get("expires_at"))
    if expiry is None:
        return "准备结果缺少有效期限，请重新准备"
    if expiry <= (time.time() if now is None else now):
        return "此路径已过期，请重新准备"
    return ""


def _numbers(value: Any) -> list[float]:
    if not isinstance(value, (tuple, list)):
        return []
    try:
        result = [float(item) for item in value]
        return result if result and all(math.isfinite(item) for item in result) else []
    except (ValueError, TypeError):
        return []


def stage_samples(stage: dict) -> list[tuple[float, list[float]]]:
    """Read actual SDK waypoints; never invent Cartesian motion for the chart."""
    plan = stage.get("sdk_plan") or {}
    if not isinstance(plan, dict):
        return []
    raw = plan.get("sdk", [])
    if not isinstance(raw, list) or not raw:
        return []
    samples = []
    duration = float(stage.get("duration_s") or plan.get("duration_s") or 0)
    waypoint_times = _numbers(plan.get("waypoint_times_s"))
    ticks = _numbers(plan.get("ticks"))
    if not waypoint_times and len(ticks) == len(raw) - 1 and float(plan.get("rate") or 0) > 0:
        waypoint_times = [0.0]
        for tick_count in ticks:
            waypoint_times.append(waypoint_times[-1] + tick_count / float(plan["rate"]))
    for index, row in enumerate(raw):
        if isinstance(row, dict):
            q = _numbers(row.get("q") or row.get("positions") or row.get("sdk") or row.get("target"))
            stamp = row.get("t_s", row.get("time_s", row.get("t")))
        else:
            q = _numbers(row)
            stamp = None
        if stamp is None and len(waypoint_times) == len(raw):
            stamp = waypoint_times[index]
        if q:
            try:
                stamp = float(stamp) if stamp is not None else duration * index / max(1, len(raw) - 1)
            except (TypeError, ValueError):
                stamp = duration * index / max(1, len(raw) - 1)
            if math.isfinite(stamp):
                samples.append((stamp, q))
    return samples


def endpoint_retry_summary(review: dict | None) -> str:
    """Only describe retry behavior explicitly present in the approved review."""
    if not isinstance(review, dict) or review.get('operation') != FIXED_PREGRASP:
        return ''
    policy = review.get('endpoint_policy')
    if not isinstance(policy, dict) or policy.get('mode') != 'retry_approved_target_v1':
        return ''
    keys = ('retry_interval_s', 'max_seek_s', 'stall_timeout_s', 'min_progress_deg')
    if any(isinstance(policy.get(key), bool) or not isinstance(policy.get(key), (int, float))
           or not math.isfinite(policy[key]) or policy[key] <= 0 for key in keys):
        return ''
    interval, maximum, stall, progress = (policy[key] for key in keys)
    return (f'终点收敛：每 {interval:g} 秒仅重发仍超 1° 的关节原批准目标；'
            f'{stall:g} 秒内偏差未改善 {progress:g}° 或总计 {maximum:g} 秒即停止；J6 和已到位轴不重发。')


def pregrasp_summary(review: dict | None) -> str:
    """Show the exact endpoint and the only stages authorized by the new button."""
    try:
        scope = FIXED_PREGRASP if (review or {}).get('operation') == FIXED_PREGRASP else PREGRASP
        stages = select_execution_stages((review or {}).get('stages', []), scope)
        samples = stage_samples(stages[-1])
        if not samples or len(samples[-1][1]) < 6:
            return ''
        endpoint = samples[-1][1][:6]
        params = (review or {}).get('parameters') or {}
        grip = stages[-1].get('gripper_target', params.get('open_target', '—'))
        summary = (('固定预抓取终点' if scope == FIXED_PREGRASP else '预抓取终点') + ' ' + _format_pose(endpoint, grip) + '\n'
                   f'夹爪张开目标 {grip} SDK；仅调整夹爪 → 移动至预抓取点，到位后停止。')
        retry = endpoint_retry_summary(review)
        return summary + ('\n' + retry if retry else '')
    except (ValueError, TypeError, KeyError):
        return ''


def _short_json(value: Any, limit: int = 1500) -> str:
    if isinstance(value, str):
        text = value
    else:
        text = json.dumps(value, ensure_ascii=False, indent=2, default=str)
    return text if len(text) <= limit else text[:limit] + "…"


def _format_q(values: Any) -> str:
    q = _numbers(values)
    return "[" + ", ".join(f"{value:.2f}" for value in q) + "]" if q else "—"


def _format_pose(values: Any, gripper: Any = None) -> str:
    """Label the six arm angles separately from the SDK gripper position."""
    q = _numbers(values)
    jaw = _numbers([q[6] if len(q) == 7 else gripper])
    return ('J0–J5 / °：' + _format_q(q[:6] if len(q) >= 6 else None)
            + '；J6 / SDK：' + (f'{jaw[0]:.2f}' if jaw else '—'))


def planning_reference_notices(review: dict) -> list[str]:
    notices=[]
    for stage in review.get("stages", []):
        if not isinstance(stage,dict):
            continue
        adjustments=stage.get("start_reference_adjustments",[])
        for item in adjustments if isinstance(adjustments,list) else []:
            if not isinstance(item,dict):
                continue
            values=_numbers([item.get("measured_sdk"),item.get("nominal_sdk")])
            joint=item.get("joint")
            if len(values)!=2 or not (type(joint) is int and 0<=joint<6 or isinstance(joint,str) and joint in {f"J{i}" for i in range(6)}):
                continue
            name=f"J{joint}" if type(joint) is int else joint
            measured,nominal=(f"{value:.3f}".rstrip("0").rstrip(".") for value in values)
            notice=f"{name} 实际反馈 {measured}°，规划参考 {nominal}°（0.2° 反馈容差，原始反馈保留；目标限位不变）"
            if notice not in notices:
                notices.append(notice)
    return notices


def preparation_result(review: dict, event_result: dict | None = None) -> dict | None:
    """Resolve a failed prepare without depending on asynchronous result events."""
    if review.get("ready") and not review.get("blockers"):
        return None
    saved = review.get("result")
    if not isinstance(saved, dict) and review.get("run_dir"):
        try:
            saved = json.loads((Path(review["run_dir"]) / "result.json").read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            saved = None
    result = copy.deepcopy(saved if isinstance(saved, dict) else event_result or {})
    result.setdefault("status", "blocked")
    result.setdefault("run_dir", review.get("run_dir"))
    result.setdefault("total_time_s", review.get("preparation_time_s"))
    # A failed preparation has never reached the execution approval boundary.
    result.update(grasp_result="not_attempted", cycle_result="not_attempted")
    blockers = review.get("blockers") or []
    if blockers:
        result["error"] = "\n".join(str(reason) for reason in blockers)
        if review.get("full_log"):
            result["error"] += "\n完整日志：" + str(review["full_log"])
    result["observed_joint_angles"] = review.get("observed_joint_angles")
    if review.get("notices"):
        result["notices"] = copy.deepcopy(review["notices"])
    return result


class GraspApp:
    def __init__(self, root: tk.Tk, backend: Any, *, mode: str = "live", auto_prepare: bool = True):
        self.root, self.backend, self.mode = root, backend, mode
        self.events: queue.Queue = queue.Queue()
        self.review: dict | None = None
        self.review_used = False
        self.parameters_changed = False
        self.fixed_motion_settings = dict(FIXED_MOTION_DEFAULTS)
        self.busy = ""
        self.closing = False
        self.cancel_requested = False
        self.log_dir: str | None = None
        self.started_at: float | None = None
        self.last_joint_draw = 0.0
        self.latest_backend_result = None
        self.device_snapshot: dict | None = None
        self.device_action = ""
        self.camera_snapshot: dict | None = None
        self.target_photo = None
        self.diagnostic_photo = None
        self.diagnostic_source = None
        self.image_path: str | None = None
        self.vars: dict[str, tk.StringVar] = {}
        self.entries = []
        self.stage_rows: dict[str, str] = {}
        root.title("D1 · 一键视觉抓取" + (" · 演示" if mode == "demo" else ""))
        root.geometry("1260x900+30+30")
        root.minsize(1040, 760)
        root.configure(bg=PALETTE["bg"])
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._styles()
        self._build()
        root.bind('<Configure>', self._layout_for_height, add='+')
        self._refresh_controls()
        root.after(70, self._pump)
        root.after(400, self._tick)
        if auto_prepare:
            root.after(450, self.prepare)

    def emit(self, event: dict) -> None:
        """Thread-safe backend callback; Tk is touched only by the main thread."""
        self.events.put(("backend", event))

    def _styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        self.font = "Noto Sans CJK SC"
        self.root.option_add("*Font", (self.font, 10))
        style.configure("TFrame", background=PALETTE["paper"])
        style.configure("TLabel", background=PALETTE["paper"], foreground=PALETTE["ink"])
        style.configure("TEntry", padding=3, fieldbackground="#f8fafc", bordercolor=PALETTE["line"])
        style.configure("TButton", padding=(10, 7), background="#edf3f7", foreground=PALETTE["ink"], borderwidth=0)
        style.map("TButton", background=[("active", "#dfeaf1")])
        style.configure('CompactAction.TButton', font=(self.font, 9), padding=(5, 7))
        style.configure("Primary.TButton", background=PALETTE["teal"], foreground="white", font=(self.font, 12, "bold"), padding=(10, 12))
        style.map("Primary.TButton", background=[("disabled", "#d5e1e7"), ("active", PALETTE["teal_active"])], foreground=[("disabled", "#7c909d")])
        style.configure("Device.TButton", background="#294958", foreground="#e3eff3", padding=(9, 5), font=(self.font, 9))
        style.map("Device.TButton", background=[("disabled", "#243c48"), ("active", "#365e70")], foreground=[("disabled", "#758f9d")])
        style.configure('Shutdown.Device.TButton',background='#75414a',foreground='white')
        style.map('Shutdown.Device.TButton',background=[('disabled','#243c48'),('active','#8b4a54')],
                  foreground=[('disabled','#758f9d')])
        style.configure("Stop.TButton", background=PALETTE["red_pale"], foreground=PALETTE["red"], padding=(10, 8))
        style.configure("Treeview", background="white", fieldbackground="white", foreground=PALETTE["ink"], rowheight=27, borderwidth=0, font=(self.font, 9))
        style.configure("Treeview.Heading", background="#eef4f7", foreground=PALETTE["muted"], padding=(6, 6), font=(self.font, 9, "bold"))
        style.map("Treeview", background=[("selected", "#d9eef0")], foreground=[("selected", PALETTE["ink"])])
        style.configure("TNotebook", background=PALETTE["paper"], borderwidth=0)
        style.configure("TNotebook.Tab", padding=(14, 6), background="#edf3f7", foreground=PALETTE["muted"])
        style.map("TNotebook.Tab", background=[("selected", "white")], foreground=[("selected", PALETTE["ink"])])

    def _label(self, parent, text="", size=10, color=None, bold=False, **kwargs):
        return tk.Label(parent, text=text, bg=parent.cget("bg") if isinstance(parent, tk.Frame) else PALETTE["paper"], fg=color or PALETTE["ink"], font=(self.font, size, "bold" if bold else "normal"), **kwargs)

    def _card(self, parent, title, subtitle=None):
        card = tk.Frame(parent, bg="white", highlightthickness=1, highlightbackground=PALETTE["line"])
        self._label(card, title, size=12, bold=True, anchor="w").pack(fill="x", padx=17, pady=(13, 0 if subtitle else 11))
        if subtitle:
            self._label(card, subtitle, size=9, color=PALETTE["muted"], anchor="w", justify="left").pack(fill="x", padx=17, pady=(2, 10))
        return card

    def _build(self):
        header = tk.Frame(self.root, bg=PALETTE["navy"], height=122)
        header.pack(fill="x")
        header.pack_propagate(False)
        title_area = tk.Frame(header, bg=PALETTE["navy"])
        title_area.pack(side="left", padx=24, pady=13)
        self._label(title_area, "D1  /  一键视觉抓取", size=21, color="white", bold=True).pack(anchor="w")
        self._label(title_area, "OpenCV  ·  40 mm 红色方块  ·  固定抓放流程", size=10, color="#acc6d4").pack(anchor="w")
        self.fixed_speed_button = ttk.Button(title_area, text='固定预抓取快速参数', style='Device.TButton', command=self.edit_fixed_speed)
        self.fixed_speed_button.pack(anchor='w', pady=(4, 0))
        mode_text = "演示模式 · 不连接设备" if self.mode == "demo" else "实机模式"
        device_area = tk.Frame(header, bg=PALETTE["navy"])
        device_area.pack(side="right", padx=(8, 20), pady=10)
        self.device_labels = {}
        for column, key in enumerate(("power_status", "enable_status")):
            label = self._label(device_area, f"{key}: 未知", size=9, color="#c1d3dd", anchor="w")
            label.grid(row=0, column=column, sticky="w", padx=(0, 16), pady=(0, 6))
            self.device_labels[key] = label
        self._label(device_area, mode_text, size=9, color="#a6e3df", anchor="e").grid(row=0, column=2, columnspan=2, sticky="e", pady=(0, 6))
        self.device_buttons = {}
        for column, (action, label) in enumerate((("power_on", "协议上电"), ("enable", "使能说明"), ("refresh", "刷新状态"))):
            button = ttk.Button(device_area, text=label, style="Device.TButton", command=lambda action=action: self.control_device(action))
            button.grid(row=1, column=column, sticky="ew", padx=(0, 8))
            self.device_buttons[action] = button
        self.camera_button = ttk.Button(device_area,text="一键启动双相机",style="Device.TButton",command=self.start_cameras)
        self.camera_button.grid(row=1,column=3,sticky="ew")
        self.device_buttons['cameras']=self.camera_button
        self.shutdown_button=ttk.Button(device_area,text='关闭双相机 / 下电检查',style='Shutdown.Device.TButton',command=self.shutdown_devices)
        self.shutdown_button.grid(row=2,column=0,columnspan=2,sticky='ew',padx=(0,8),pady=(5,0))
        self.device_buttons['shutdown']=self.shutdown_button
        self.device_buttons['fixed_speed']=self.fixed_speed_button
        self.camera_hint=self._label(device_area,"腕部：未检查  全局：未检查",size=8,color="#acc6d4",anchor="e")
        self.camera_hint.grid(row=2,column=2,columnspan=2,sticky="e",pady=(5,0))

        body = tk.Frame(self.root, bg=PALETTE["bg"])
        body.pack(fill="both", expand=True, padx=16, pady=(14, 8))
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        left = tk.Frame(body, bg=PALETTE["bg"], width=304)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 13))
        left.pack_propagate(False)
        right = tk.Frame(body, bg=PALETTE["bg"])
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        parameters = self._card(left, "01  抓取运动参数", "固定预抓取速度在顶部单独设置。")
        parameters.pack(fill="x", pady=(0, 11))
        fields = tk.Frame(parameters, bg="white")
        fields.pack(fill="x", padx=17)
        fields.columnconfigure(1, weight=1)
        for row, (key, label, unit, default) in enumerate(FIELDS):
            self._label(fields, label, size=9, anchor="w").grid(row=row, column=0, sticky="w", pady=2)
            variable = tk.StringVar(value=default)
            self.vars[key] = variable
            entry = ttk.Entry(fields, textvariable=variable, width=8, justify="right")
            entry.grid(row=row, column=1, sticky="ew", padx=(7, 6), pady=2)
            self._label(fields, unit, size=8, color=PALETTE["muted"], anchor="w").grid(row=row, column=2, sticky="w")
            self.entries.append(entry)
            variable.trace_add("write", lambda *_args, key=key: self._parameters_edited(key))
        self._label(parameters, "速度可调；实际运动受驱动器能力影响。\n关节反馈容差 0.2°；目标限位不变。", size=9, color=PALETTE["muted"], justify="left", anchor="w").pack(fill="x", padx=17, pady=(7, 13))

        actions = self._card(left, "02  准备并执行")
        actions.pack(fill="x", pady=(0, 11))
        first_actions = tk.Frame(actions, bg='white')
        first_actions.pack(fill='x', padx=16, pady=(0, 9))
        first_actions.columnconfigure((0, 1), weight=1, uniform='action')
        self.prepare_button = ttk.Button(first_actions, text="识别 / 准备路径", style='CompactAction.TButton', command=self.prepare)
        self.prepare_button.grid(row=0, column=0, sticky='ew', padx=(0, 3))
        self.pregrasp_button = ttk.Button(first_actions, text="准备固定预抓取", style='CompactAction.TButton', command=self.enter_pregrasp)
        self.pregrasp_button.grid(row=0, column=1, sticky='ew', padx=(3, 0))
        self.execute_button = ttk.Button(actions, text="确认此路径，执行一次抓放", style="Primary.TButton", command=self.execute)
        self.execute_button.pack(fill="x", padx=16, pady=(0, 9))
        self.cancel_button = ttk.Button(actions, text="停止后续指令", style="Stop.TButton", command=self.cancel)
        self.cancel_button.pack(fill="x", padx=16)
        self._label(actions, "固定预抓取：先准备，再确认进入。\n完整抓放：识别后批准右侧全部阶段。\n停止按钮停止后续指令，不是物理急停。", size=9, color=PALETTE["muted"], justify="left", anchor="w").pack(fill="x", padx=17, pady=(10, 13))

        status_card = self._card(left, "03  当前状态")
        status_card.pack(fill="both", expand=True)
        self.state_label = self._label(status_card, "等待准备", size=12, color=PALETTE["teal"], bold=True, anchor="w", justify="left", wraplength=265)
        self.state_label.pack(fill="x", padx=17, pady=(0, 8))
        self.elapsed_label = self._label(status_card, "本次执行  —", color=PALETTE["muted"], anchor="w")
        self.elapsed_label.pack(fill="x", padx=17, pady=(0, 8))
        self.control_hint = self._label(status_card, "", size=9, color=PALETTE["muted"], anchor="nw", justify="left", wraplength=263)
        self.control_hint.pack(fill="x", padx=17, pady=(0, 12))

        target_card = self._card(right, "目标与批准参数", "先检查目标与路径，再选择仅到预抓取点或执行完整抓放。")
        target_card.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        target_row = tk.Frame(target_card, bg="white")
        target_row.pack(fill="x", padx=17, pady=(0, 10))
        self.target_image_label = self._label(target_row, "")
        target_info = tk.Frame(target_row, bg="white")
        target_info.pack(side="left", fill="both", expand=True)
        self.target_label = self._label(target_info, "尚未识别目标", size=10, anchor="w", justify="left", wraplength=820)
        self.target_label.pack(fill="x", pady=(0, 7))
        self.parameter_summary = self._label(target_info, "", size=9, color=PALETTE["muted"], anchor="w", justify="left", wraplength=820)
        self.parameter_summary.pack(fill="x", pady=(0, 6))
        self.pregrasp_target_label = self._label(target_info, "", size=9, color=PALETTE['teal'], anchor='w', justify='left', wraplength=820)
        self.pregrasp_target_label.pack(fill='x', pady=(0, 6))
        self.signature_label = self._label(target_info, "", size=8, color=PALETTE["muted"], anchor="w")
        self.signature_label.pack(fill="x")
        target_info.bind("<Configure>", lambda event: (self.target_label.configure(wraplength=max(300, event.width)), self.parameter_summary.configure(wraplength=max(300, event.width)), self.pregrasp_target_label.configure(wraplength=max(300, event.width))))

        path_card = self._card(right, "完整抓放路径", "SDK 关节位置曲线 · 点击阶段查看起点、终点和执行参数")
        path_card.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        self.path_title, self.path_subtitle = path_card.winfo_children()[:2]
        self.canvas = tk.Canvas(path_card, height=100, bg="white", highlightthickness=0)
        self.canvas.pack(fill="x", padx=12)
        self.canvas.bind("<Configure>", lambda _event: self._draw_path())
        table_frame = tk.Frame(path_card, bg="white")
        table_frame.pack(fill="x", padx=12, pady=(4, 0))
        self.stage_table_frame = table_frame
        self.stages_table = ttk.Treeview(table_frame, columns=("stage", "duration", "speed", "gripper"), show="headings", height=3, selectmode="browse")
        for name, label, width, stretch in (("stage", "阶段", 220, True), ("duration", "时长 / s", 85, False), ("speed", "关节速度 / °/s", 120, False), ("gripper", "夹爪起点 → 目标", 125, False)):
            self.stages_table.heading(name, text=label)
            self.stages_table.column(name, width=width, minwidth=65, stretch=stretch, anchor="w" if name == "stage" else "center")
        self.stages_table.pack(side="left", fill="x", expand=True)
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.stages_table.yview)
        scrollbar.pack(side="right", fill="y")
        self.stages_table.configure(yscrollcommand=scrollbar.set)
        self.stages_table.bind("<<TreeviewSelect>>", self._select_stage)
        self.stage_detail = self._label(path_card, "准备完成后显示完整阶段。", size=8, color=PALETTE["muted"], anchor="w", justify="left", wraplength=840)
        self.stage_detail.pack(fill="x", padx=17, pady=(7, 10))

        bottom = tk.Frame(right, bg="white", highlightthickness=1, highlightbackground=PALETTE["line"])
        bottom.grid(row=2, column=0, sticky="nsew")
        notebook = ttk.Notebook(bottom)
        notebook.pack(fill="both", expand=True, padx=10, pady=(9, 6))
        self.notebook = notebook
        log_frame = ttk.Frame(notebook)
        result_frame = ttk.Frame(notebook)
        detail_frame = ttk.Frame(notebook)
        notebook.add(log_frame, text="执行日志")
        notebook.add(result_frame, text="本次结果")
        notebook.add(detail_frame, text="目标 / 路径详情")
        self.log_text = self._text_panel(log_frame, background="#122c3b", foreground="#d9e6ec")
        self.result_text = self._text_panel(result_frame)
        self.detail_text = self._text_panel(detail_frame)
        self._set_text(self.result_text, "完成一次抓放后，这里显示结果、总耗时、阶段耗时及日志目录。")
        self._set_text(self.detail_text, "准备完成后，可在这里查看本次目标和每阶段关节起止位置。")
        footer = tk.Frame(bottom, bg="white")
        footer.pack(fill="x", padx=13, pady=(0, 10))
        self.joint_label = self._label(footer, "实际关节位置  —", size=8, color=PALETTE["muted"], anchor="w")
        self.joint_label.pack(side="left", fill="x", expand=True)
        self.open_logs_button = ttk.Button(footer, text="打开日志目录", command=self.open_logs)
        self.open_logs_button.pack(side="right")
        self.open_image_button = ttk.Button(footer, text="打开诊断图", command=self.open_image)
        self.open_image_button.pack(side="right", padx=(0, 6))
        # Reserve the footer before allocating remaining height to the log pane.
        notebook.pack_forget()
        footer.pack_configure(side="bottom")
        notebook.pack(fill="both", expand=True, padx=10, pady=(9, 6))
        self._append_log("程序已启动。准备只识别、读取状态并生成路径；点击执行后才开始抓放。")
        if self.mode == "demo":
            self._append_log("当前为演示模式：不连接相机或机械臂，结果不能用于证明实物抓取成功。")

    def _text_panel(self, parent, background="white", foreground=None):
        frame = ttk.Frame(parent)
        frame.pack(fill="both", expand=True)
        text = tk.Text(frame, wrap="word", height=5, borderwidth=0, highlightthickness=0, padx=10, pady=9, bg=background, fg=foreground or PALETTE["ink"], insertbackground=foreground or PALETTE["ink"], font=(self.font, 9), state="disabled")
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        return text

    @staticmethod
    def _set_text(widget, text):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("end", text)
        widget.configure(state="disabled")

    def _append_log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{dt.datetime.now(DISPLAY_ZONE).strftime('%H:%M:%S')}  {message}\n")
        # Keep the UI responsive; the backend retains the complete file logs.
        if int(self.log_text.index("end-1c").split(".")[0]) > 1400:
            self.log_text.delete("1.0", "201.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _parameters_edited(self, key=None, *_args):
        if self.review is not None and not (self.review.get('operation') == FIXED_PREGRASP and key in
                ('speed_deg_s','acceleration_deg_s2','gripper_speed_sdk_s','rate_hz','lift_height_m','close_target')):
            self.parameters_changed = True
            self.state_label.configure(text="参数已修改 · 需要重新准备", fg=PALETTE["amber"])
        self._refresh_controls()

    def _current_settings(self):
        return parse_settings({key: variable.get() for key, variable in self.vars.items()})

    def edit_fixed_speed(self):
        if self.busy or self.closing:
            return
        window = tk.Toplevel(self.root)
        window.title('固定预抓取快速参数')
        window.transient(self.root)
        frame = ttk.Frame(window, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='仅用于 Gazebo 固定准备姿态；与抓取速度分开。\n以下为参考上限；按实际反馈推进，SDK 指令延时 1 ms。').grid(row=0,column=0,columnspan=3,sticky='w',pady=(0,12))
        variables = {}
        fields = [('speed_deg_s','固定姿态关节速度','°/s'),('acceleration_deg_s2','固定姿态关节加速度','°/s²'),
                  ('gripper_speed_sdk_s','固定姿态夹爪速度','SDK/s'),('rate_hz','通信频率','Hz，≤10')]
        for row,(key,label,unit) in enumerate(fields,1):
            ttk.Label(frame,text=label).grid(row=row,column=0,sticky='w',pady=5)
            variables[key]=tk.StringVar(value=str(self.fixed_motion_settings[key]))
            ttk.Entry(frame,textvariable=variables[key],width=12).grid(row=row,column=1,padx=10,pady=5)
            ttk.Label(frame,text=unit).grid(row=row,column=2,sticky='w')
        error=ttk.Label(frame,text='',foreground=PALETTE['red'],wraplength=450)
        error.grid(row=5,column=0,columnspan=3,sticky='w',pady=8)
        def save():
            try:
                values=fixed_motion_checked({key:variable.get() for key,variable in variables.items()})
            except (TypeError,ValueError) as exc:
                error.configure(text=str(exc));return
            if values!=self.fixed_motion_settings:
                self.fixed_motion_settings=values
                if (self.review or {}).get('operation')==FIXED_PREGRASP:
                    self.parameters_changed=True
                self._append_log('已更新固定预抓取快速参数，抓取速度设置保持独立。')
                self._refresh_controls()
            window.destroy()
        ttk.Button(frame,text='保存固定姿态参数',command=save).grid(row=6,column=0,columnspan=3,sticky='ew')

    def _refresh_controls(self):
        blocker = execution_blocker(self.review, used=self.review_used, changed=self.parameters_changed, busy=bool(self.busy))
        fixed = (self.review or {}).get('operation') == FIXED_PREGRASP
        self.execute_button.configure(state="disabled" if blocker or self.closing or fixed else "normal")
        self.pregrasp_button.configure(state='disabled' if self.busy or self.closing else 'normal',
            text='一键进入预抓取' if fixed and not blocker else '准备固定预抓取')
        self.prepare_button.configure(state="disabled" if self.busy or self.closing else "normal")
        self.cancel_button.configure(state="normal" if self.busy and not self.cancel_requested else "disabled")
        self.open_logs_button.configure(state="normal" if self.log_dir else "disabled")
        self.open_image_button.configure(state="normal" if self.image_path else "disabled")
        for entry in self.entries:
            entry.configure(state="disabled" if self.busy or self.closing else "normal")
        for button in self.device_buttons.values():
            button.configure(state="disabled" if self.busy or self.closing else "normal")
        self._refresh_device_status()
        if self.busy == "device_control":
            hint = "正在" + DEVICE_ACTION_NAMES.get(self.device_action, "读取设备状态") + "；等待实际协议反馈。此操作不会开始抓放。"
        elif self.busy == "camera_services":
            hint = "正在检查并启动腕部、全局相机及 SSH 转发，等待新鲜图像。"
        elif self.busy == 'shutdown_devices':
            hint = '正在核对机械臂下电支持，并关闭双相机服务与转发。'
        elif self.busy == "execute":
            hint = "正在按已批准路径执行。日志持续记录实际关节位置、时间与结果。"
        elif self.busy:
            hint = "正在识别目标并计算路径；此步骤不会开始抓放。"
        elif blocker:
            hint = blocker
        else:
            remaining = max(0, int((expiry_timestamp(self.review.get("expires_at")) or 0) - time.time()))
            hint = (f"固定姿态路径已准备，剩余 {remaining} 秒。\n检查六轴固定目标与路径后，点击“一键进入预抓取”。" if fixed else
                    f"路径已准备。有效期剩余 {remaining} 秒。\n检查右侧目标、路径和参数后点击执行。")
        if self.cancel_requested and self.busy:
            hint = "已请求停止后续指令，正在等待执行线程结束。机械臂当前状态以反馈为准。"
        if self.closing:
            hint = "正在停止后续指令并等待日志写入，结束后自动关闭窗口。"
        self.control_hint.configure(text=hint)

    def _refresh_device_status(self):
        labels, fresh = device_status_text(self.device_snapshot)
        status = (self.device_snapshot or {}).get("status") or {}
        if not isinstance(status, dict):
            status = {}
        for key, label in self.device_labels.items():
            color = "#a6e3df" if fresh and status.get(key) == 1 else "#e9be78" if status.get(key) in (0, 1) else "#c1d3dd"
            label.configure(text=labels[key], fg=color)

    def control_device(self, action):
        if self.busy or self.closing or action not in DEVICE_ACTION_NAMES:
            return
        if action == "enable":
            self._set_text(self.result_text, ENABLE_EXPLANATION)
            self.notebook.select(1)
            self._append_log("查看使能说明；未发送设备指令，当前已准备路径保持有效。")
            return
        self.device_action = action
        # A state-changing attempt invalidates the old approval; refresh only reads.
        if action != "refresh":
            self.review = None
            self.review_used = False
            self.parameters_changed = False
            self.signature_label.configure(text="设备状态操作后，请重新准备并检查路径")
            self.stage_detail.configure(text="原路径已失效，请重新准备目标与路径。")
            self._draw_path()
        self.state_label.configure(text=DEVICE_ACTION_NAMES[action] + "中…", fg=PALETTE["teal"])
        self._append_log("用户点击“" + DEVICE_ACTION_NAMES[action] + "”；" + ("只读取协议状态。" if action == "refresh" else "原路径已失效，此操作不会开始抓放。"))
        self._start_job("device_control", lambda: self.backend.control_device(action))

    def start_cameras(self):
        if self.busy or self.closing:
            return
        self.review=None;self.review_used=False;self.parameters_changed=False
        self.camera_snapshot=None
        self.signature_label.configure(text='双相机启动后，请重新准备目标与路径')
        self.stage_detail.configure(text='原路径已失效；相机就绪后重新准备。')
        self._draw_path()
        self.camera_hint.configure(text='腕部：检查中…  全局：检查中…',fg='#e9be78')
        self.state_label.configure(text='正在启动双相机…',fg=PALETTE['teal'])
        self._append_log('用户点击“一键启动双相机”；检查相机服务、SSH 转发及新鲜图像。')
        self._start_job('camera_services',self.backend.start_cameras)

    def _show_camera_services(self,result):
        if not isinstance(result,dict):
            result=dict(completed=False,cameras={},error='相机服务返回了无效结果')
        self.camera_snapshot=copy.deepcopy(result)
        self.log_dir=str(result.get('log_dir') or self.log_dir or '') or None
        cameras=result.get('cameras') or {}
        if not isinstance(cameras,dict):cameras={}
        hints=[];lines=['双相机启动'+('完成' if result.get('completed') is True else '未完成'),'']
        for role,label in [('wrist','腕部相机'),('global','全局相机')]:
            camera=cameras.get(role) or {}
            if not isinstance(camera,dict):camera={}
            ready=camera.get('ready') is True
            state='已就绪' if ready else '未就绪'
            hints.append(('腕部' if role=='wrist' else '全局')+'：'+state)
            fps=camera.get('capture_fps')
            detail=f'，约 {fps:.1f} 帧/秒' if ready and isinstance(fps,(int,float)) and math.isfinite(fps) else ''
            lines.append(label+'：'+state+detail)
            if camera.get('error'):lines.append('原因：'+str(camera['error']))
        completed=result.get('completed') is True and all(isinstance(cameras.get(role),dict)
            and cameras[role].get('ready') is True for role in ('wrist','global'))
        self.camera_hint.configure(text='  '.join(hints),fg='#a6e3df' if completed else '#e9be78')
        self.state_label.configure(text='双相机已就绪 · 请重新准备' if completed else '双相机未就绪 · 查看原因',
                                   fg=PALETTE['teal'] if completed else PALETTE['red'])
        if result.get('synthetic'):lines.insert(0,'演示模式 · 未连接相机')
        if result.get('error'):lines.extend(['','结束原因：'+str(result['error'])])
        lines.extend(['','以上为本次检查结果。相机就绪后点击“识别目标 / 准备路径”。',
                      '本次未发送机械臂指令。','日志目录：'+(self.log_dir or '—')])
        self._set_text(self.result_text,'\n'.join(lines));self.notebook.select(1)
        self._refresh_controls()

    def shutdown_devices(self):
        if self.busy or self.closing:return
        self.review=None;self.review_used=False;self.parameters_changed=False
        self.signature_label.configure(text='设备关闭操作后，原路径已失效')
        self.stage_detail.configure(text='再次启动设备后需重新准备目标与路径。')
        self._draw_path()
        self.state_label.configure(text='正在关闭设备…',fg=PALETTE['amber'])
        self._append_log('用户点击“关闭双相机 / 下电检查”；下电和相机关停分别核对结果。')
        self._start_job('shutdown_devices',self.backend.shutdown_devices)

    def _show_shutdown_devices(self,result):
        if not isinstance(result,dict):result=dict(completed=False,error='未收到有效的设备关闭报告')
        self.log_dir=str(result.get('log_dir') or self.log_dir or '') or None
        power=result.get('power_off') or {};stop=result.get('camera_stop') or {}
        self.device_snapshot=copy.deepcopy(power)  # No optimistic power_status=0.
        self.camera_snapshot=copy.deepcopy(stop)
        power_ok=power.get('completed') is True and power.get('status',{}).get('power_status')==0
        completed=power_ok and stop.get('completed') is True and all(
            stop.get('cameras',{}).get(role,{}).get('stopped') is True for role in ('wrist','global'))
        lines=['设备关闭'+('完成' if completed else '未全部完成'),'']
        lines.append('机械臂下电：'+('协议 power_status=0 已回读' if power_ok else '未确认下电'))
        if power.get('error'):lines.append('原因：'+str(power['error']))
        hints=[]
        for role,label in [('wrist','腕部相机'),('global','全局相机')]:
            item=stop.get('cameras',{}).get(role,{})
            state='已关闭' if item.get('stopped') is True else '未确认关闭'
            hints.append(('腕部' if role=='wrist' else '全局')+'：'+state)
            lines.append(label+'：'+state)
            if item.get('error'):lines.append('原因：'+str(item['error']))
        self.camera_hint.configure(text='  '.join(hints),fg='#a6e3df' if stop.get('completed') else '#e9be78')
        if result.get('error'):lines.extend(['','结束原因：'+str(result['error'])])
        if result.get('synthetic'):lines.insert(0,'演示模式 · 未操作实体设备')
        lines.extend(['','未执行归零、卸力或抓放。','日志目录：'+(self.log_dir or '—')])
        self.state_label.configure(text='设备关闭完成' if completed else '设备未全部关闭 · 查看原因',
                                   fg=PALETTE['teal'] if completed else PALETTE['amber'])
        self._set_text(self.result_text,'\n'.join(lines));self.notebook.select(1);self._refresh_controls()

    def _show_device_control(self, result):
        if not isinstance(result, dict):
            result = {"completed": False, "status": {}, "status_fresh": False, "error": "设备控制返回了无效结果"}
        self.device_snapshot = copy.deepcopy(result)
        self.log_dir = str(result.get("log_dir") or result.get("run_dir") or self.log_dir or "") or None
        action_key = str(result.get("action") or self.device_action)
        action = DEVICE_ACTION_NAMES.get(action_key, "设备状态操作")
        completed = result.get("completed") is True
        error = str(result.get("error") or "")
        self.state_label.configure(text=action + (("完成" if action_key == "refresh" else "完成 · 请重新准备") if completed else "未完成 · 查看原因"), fg=PALETTE["teal"] if completed else PALETTE["red"])
        q = _numbers(result.get("sdk_angles") or result.get("observed_joint_angles") or result.get("q"))
        if q:
            self.joint_label.configure(text="实际关节 SDK  " + _format_q(q))
        labels, fresh = device_status_text(self.device_snapshot)
        lines = [action + ("完成" if completed else "未完成"), "", *labels.values(),
                 "反馈状态：" + ("本次已收到有效协议反馈" if fresh else "未知或仅有上次反馈，请刷新状态")]
        if result.get("message"):
            lines.extend(["", str(result["message"])])
        if error:
            lines.extend(["", "原因：" + error])
        lines.extend(["", "本次只读取协议状态；未执行抓放。" if action_key == "refresh" else "设备状态操作后需要重新准备路径；未执行抓放。", "日志目录：" + (self.log_dir or "—")])
        self._set_text(self.result_text, "\n".join(lines))
        self.notebook.select(1)
        self._append_log(action + ("完成。" if completed else "未完成。") + error)
        self._refresh_controls()

    def _start_job(self, kind, operation):
        self.busy = kind
        self.cancel_requested = False
        self._refresh_controls()

        def worker():
            try:
                self.events.put((kind + "_done", operation()))
            except Exception as exc:
                self.events.put((kind + "_error", f"{type(exc).__name__}: {exc}"))

        threading.Thread(target=worker, name=f"grasp-{kind}", daemon=False).start()

    def prepare(self, *, fixed_pregrasp=False):
        if self.busy or self.closing:
            return
        try:
            settings = self._current_settings()
        except ValueError as exc:
            self.state_label.configure(text="参数需要修正", fg=PALETTE["red"])
            self._append_log(str(exc))
            self.control_hint.configure(text=str(exc))
            return
        self.review = None
        self.review_used = False
        self.parameters_changed = False
        self.latest_backend_result = None
        self.state_label.configure(text="正在准备固定预抓取姿态…" if fixed_pregrasp else "正在准备目标与路径…", fg=PALETTE["teal"])
        self.target_label.configure(text="读取当前关节反馈，规划至 Gazebo ready 固定姿态…" if fixed_pregrasp else "正在识别红色方块并读取当前关节状态…")
        self.parameter_summary.configure(text="")
        self.pregrasp_target_label.configure(text='')
        self.signature_label.configure(text="")
        self.target_image_label.pack_forget()
        self.target_image_label.configure(image="")
        self.target_photo = None
        self.diagnostic_photo = None
        self.diagnostic_source = None
        self.image_path = None
        self.joint_label.configure(text="实际关节位置  正在读取…")
        self._set_path_display(diagnostic=False)
        self.stages_table.delete(*self.stages_table.get_children())
        self.stage_rows.clear()
        self.stage_detail.configure(text="正在计算完整流程。")
        self._draw_path()
        self._append_log("开始准备新路径。")
        if fixed_pregrasp:
            fixed_motion = copy.deepcopy(self.fixed_motion_settings)
            self._start_job('prepare', lambda: self.backend.prepare(settings, fixed_pregrasp=True, fixed_motion=fixed_motion))
        else:
            self._start_job("prepare", lambda: self.backend.prepare(settings))

    def enter_pregrasp(self):
        if self.busy or self.closing:
            return
        blocker = execution_blocker(self.review, used=self.review_used, changed=self.parameters_changed, busy=False)
        if (self.review or {}).get('operation') != FIXED_PREGRASP or blocker:
            self.prepare(fixed_pregrasp=True)
        else:
            self._execute_review(pregrasp_only=True)

    def execute(self):
        self._execute_review(pregrasp_only=False)

    def _execute_review(self, *, pregrasp_only):
        blocker = execution_blocker(self.review, used=self.review_used, changed=self.parameters_changed, busy=bool(self.busy))
        if ((self.review or {}).get('operation') == FIXED_PREGRASP) != pregrasp_only:
            blocker = '请先准备对应的固定姿态或完整抓放路径'
        if pregrasp_only and not blocker and not pregrasp_summary(self.review):
            blocker = '当前路径没有可显示的预抓取目标，请重新准备'
        if blocker or self.closing:
            self._append_log("未开始执行：" + (blocker or "窗口正在关闭"))
            self._refresh_controls()
            return
        # An immutable copy of the already displayed review is sent as approval.
        review = copy.deepcopy(self.review)
        self.review_used = True
        self.started_at = time.monotonic()
        self.state_label.configure(text="正在进入预抓取状态…" if pregrasp_only else "正在执行一次抓放…", fg=PALETTE["teal"])
        self.notebook.select(0)
        if pregrasp_only:
            approval = (f"用户点击‘一键进入预抓取’，明确批准界面已显示的 Gazebo ready 固定六轴目标、夹爪调整及路径和参数；"
                        f"仅执行至固定预抓取姿态并停止。approval_sha256={review['approval_sha256']}")
            self._append_log(f"用户已批准预抓取路径 {review['approval_sha256'][:16]}；仅调整夹爪并移动至预抓取点。")
            self._start_job('execute', lambda: self.backend.execute(review, approval_text=approval, execution_scope=FIXED_PREGRASP))
            return
        approval = f"用户点击‘确认此路径，执行一次抓放’，明确批准界面已显示的目标、完整路径和关键参数。approval_sha256={review['approval_sha256']}"
        self._append_log(f"用户已批准当前路径 {review['approval_sha256'][:16]}；开始一次抓放。")
        self._start_job("execute", lambda: self.backend.execute(review, approval_text=approval))

    def cancel(self):
        if not self.busy or self.cancel_requested:
            return
        self.cancel_requested = True
        self.state_label.configure(text="正在停止后续指令…", fg=PALETTE["amber"])
        self._append_log("已请求停止后续指令。此操作不是物理急停，等待后台确认结束。")
        self._refresh_controls()

        def request_cancel():
            try:
                self.backend.cancel()
            except Exception as exc:
                self.events.put(("cancel_error", str(exc)))
        threading.Thread(target=request_cancel, name="grasp-cancel", daemon=True).start()

    def close(self):
        if self.busy:
            self.closing = True
            self.cancel()
            self._refresh_controls()
        else:
            self.root.destroy()

    def open_logs(self):
        if self.log_dir:
            path = Path(self.log_dir).expanduser().resolve()
            if path.is_dir():
                try:
                    subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError as exc:
                    self._append_log(f"无法打开目录：{exc}；日志位置：{path}")
            else:
                self._append_log(f"日志目录尚不存在：{path}")

    def open_image(self):
        if self.image_path:
            path = Path(self.image_path).expanduser().resolve()
            if path.is_file():
                try:
                    subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError as exc:
                    self._append_log(f"无法打开诊断图：{exc}；图像位置：{path}")

    def _layout_for_height(self, event):
        if event.widget is self.root:
            self._fit_path_chart(event.height)

    def _fit_path_chart(self, height):
        compact = height < 850
        diagnostic = getattr(self, 'path_is_diagnostic', False)
        if compact and not diagnostic:
            self.canvas.pack_forget()
        else:
            if not self.canvas.winfo_manager():
                following = self.stage_table_frame if self.stage_table_frame.winfo_manager() else self.stage_detail
                self.canvas.pack(fill='x', padx=12, before=following)
            self.canvas.configure(height=260 if diagnostic else 100)
        self.stages_table.configure(height=2 if compact else 3)

    def _set_path_display(self, *, diagnostic: bool):
        self.path_is_diagnostic = diagnostic
        fixed = (self.review or {}).get('operation') == FIXED_PREGRASP
        self.path_title.configure(text="准备诊断图" if diagnostic else "固定准备姿态路径" if fixed else "完整抓放路径")
        self.path_subtitle.configure(text="点击图像可打开原图；准备失败原因见下方“本次结果”。" if diagnostic else "SDK 关节位置曲线 · 点击阶段查看起点、终点和执行参数")
        if diagnostic:
            self.stage_table_frame.pack_forget()
        elif not self.stage_table_frame.winfo_manager():
            self.stage_table_frame.pack(fill="x", padx=12, pady=(4, 0), before=self.stage_detail)
        self._fit_path_chart(self.root.winfo_height())

    def _target_summary(self, target):
        if not isinstance(target, dict):
            return _short_json(target, 420)
        transform = target.get("T_base_cube")
        if isinstance(transform, list) and len(transform) == 4:
            try:
                xyz = [float(transform[index][3]) for index in range(3)]
                if all(math.isfinite(value) for value in xyz):
                    return "红色方块 40 mm\n基座坐标：" + "   ".join(f"{axis} = {value:.4f} m" for axis, value in zip("XYZ", xyz))
            except (TypeError, ValueError, IndexError):
                pass
        if target.get("description"):
            return str(target["description"])
        useful = []
        for key, label in (("label", "目标"), ("center_base_m", "基座坐标 / m"), ("target_base_m", "基座坐标 / m"), ("position_base_m", "基座坐标 / m"), ("center_px", "图像中心 / px"), ("confidence", "置信度"), ("size_m", "尺寸 / m")):
            if key in target:
                useful.append(f"{label}：{_short_json(target[key], 100).replace(chr(10), ' ')}")
        if useful:
            return "红色方块 40 mm\n" + "   ·   ".join(useful)
        return "红色方块 40 mm  |  " + _short_json(target, 480).replace("\n", " ")

    def _show_review(self, review):
        self.review = copy.deepcopy(review)
        if isinstance(review.get("device_status"), dict):
            self.device_snapshot = copy.deepcopy(review["device_status"])
        self.target_image_label.pack_forget()
        self.target_image_label.configure(image="")
        self.target_photo = None
        self.diagnostic_photo = None
        self.diagnostic_source = None
        self.image_path = None
        self.log_dir = str(review.get("run_dir") or self.log_dir or "") or None
        fallback_target = "目标定位未通过 · 查看诊断图及准备失败原因" if not review.get("ready") else "未识别到目标"
        self.target_label.configure(text=self._target_summary(review.get("target", fallback_target)))
        params = review.get("parameters") or {}
        self.parameter_summary.configure(text=(f"关节 {params.get('speed_deg_s', '—')} °/s  ·  加速度 {params.get('acceleration_deg_s2', '—')} °/s²  ·  夹爪 {params.get('gripper_speed_sdk_s', '—')} SDK/s\n"
            f"抬升 {params.get('lift_height_m', '—')} m  ·  抓取前张开目标 {params.get('open_target', '—')}  ·  夹紧 {params.get('close_target', '—')} SDK"))
        self.pregrasp_target_label.configure(text=pregrasp_summary(review).split('\n')[0] if review.get('operation') == FIXED_PREGRASP else '')
        if review.get('operation') == FIXED_PREGRASP:
            retry = endpoint_retry_summary(review)
            self.parameter_summary.configure(text=(f"关节 {params.get('speed_deg_s', '—')} °/s  ·  加速度 {params.get('acceleration_deg_s2', '—')} °/s²\n"
                f"夹爪 J6 目标 {params.get('open_target', '—')} SDK  ·  张开速度 {params.get('gripper_speed_sdk_s', '—')} SDK/s；按实际反馈推进，到位结束"
                + ('\n' + retry if retry else '')))
        if review.get("image"):
            try:
                from PIL import Image, ImageTk
                with Image.open(review["image"]) as source:
                    preview = source.copy()
                self.diagnostic_source = preview.copy()
                self.image_path = str(review["image"])
                preview.thumbnail((180, 110), Image.Resampling.LANCZOS)
                self.target_photo = ImageTk.PhotoImage(preview, master=self.root)
                self.target_image_label.configure(image=self.target_photo)
                if review.get("stages"):
                    self.target_image_label.pack(side="right", padx=(12, 0), before=self.target_image_label.master.winfo_children()[1])
            except (ImportError, OSError, ValueError) as exc:
                self._append_log(f"识别叠图预览不可用：{exc}")
        signature = str(review.get("approval_sha256", ""))
        expiry = expiry_timestamp(review.get("expires_at"))
        expires = dt.datetime.fromtimestamp(expiry, DISPLAY_ZONE).strftime("%H:%M:%S") if expiry is not None else "无有效期限"
        self.signature_label.configure(text=f"路径签名 {signature[:20]}    ·    有效至 {expires}" if signature else "尚未生成可批准路径")
        self.stages_table.delete(*self.stages_table.get_children())
        self.stage_rows.clear()
        detail_lines = ["本次目标", _short_json(review.get("target"), 10000), "", "已准备的关键参数", _short_json(params, 10000), "", "完整路径阶段"]
        actual_q = _numbers(review.get("observed_joint_angles"))
        if actual_q:
            self.joint_label.configure(text="实际关节 SDK  " + _format_q(actual_q))
            detail_lines[0:0] = ["准备时实际关节反馈 / SDK", _format_q(actual_q), ""]
        notices = list(dict.fromkeys([*[str(notice) for notice in review.get("notices", [])],*planning_reference_notices(review)]))
        if notices:
            detail_lines[0:0] = ["本次执行说明", *notices, ""]
            for notice in notices:
                self._append_log(notice)
        for index, stage in enumerate(review.get("stages") or []):
            original_name = str(stage.get("name") or f"阶段 {index + 1}")
            name = STAGE_NAMES.get(original_name, original_name)
            duration = stage.get("duration_s", "—")
            duration_text = f"{duration:.2f}" if isinstance(duration, (int, float)) else str(duration)
            gripper = stage.get("gripper_plan") or {}
            grip_target = gripper.get("target_sdk_units", gripper.get("target", gripper.get("target_sdk", gripper.get("to", stage.get("gripper_target", "—"))))) if isinstance(gripper, dict) else gripper
            speed = stage.get("speed_deg_s", params.get("speed_deg_s", "—")) if stage.get("sdk_plan") else "—"
            samples = stage_samples(stage)
            start_q = samples[0][1] if samples else (stage.get("expected_angles") or stage.get("start_sdk") or stage.get("q_start"))
            end_q = samples[-1][1] if samples else (stage.get("end_sdk") or stage.get("q_end"))
            if gripper and not end_q and _numbers(start_q):
                end_q = list(start_q)
                if len(end_q) == 7 and isinstance(grip_target, (int, float)):
                    end_q[6] = grip_target
            grip_display = grip_target
            if gripper and len(_numbers(start_q)) == 7 and isinstance(grip_target, (int, float)):
                grip_display = f"{float(start_q[6]):.2f} → {grip_target:.2f}"
            row = self.stages_table.insert("", "end", values=(f"{index + 1:02d}  {name}", duration_text, speed, grip_display))
            expected_q = _numbers(stage.get('expected_angles'))
            start = _format_pose(start_q, expected_q[6] if len(expected_q) == 7 else None)
            end = _format_pose(end_q, grip_target)
            details = f"{name} · {duration_text} s\n起点 {start}\n终点 {end}\n关节速度：{speed} °/s；夹爪：{_short_json(gripper, 500).replace(chr(10), ' ')}"
            for field, label in (("start_tcp", "起点 TCP"), ("target_tcp", "终点 TCP")):
                matrix = stage.get(field)
                if isinstance(matrix, list) and len(matrix) == 4:
                    details += f"\n{label} / m：" + _format_q([matrix[i][3] for i in range(3)])
            self.stage_rows[row] = details
            detail_lines.append(details + "\n")
        blockers = review.get("blockers") or []
        if blockers:
            detail_lines.extend(["准备未通过原因", *[f"{index + 1}. {reason}" for index, reason in enumerate(blockers)]])
        detail_lines.extend(["", "路径签名", signature, "", "日志目录", self.log_dir or "—"])
        self._set_text(self.detail_text, "\n".join(detail_lines))
        rows = self.stages_table.get_children()
        if rows:
            self.stages_table.selection_set(rows[0])
            self._select_stage()
        else:
            self.stage_detail.configure(text="准备未通过，尚未执行抓放。" if not review.get("ready") else "尚无可执行路径。")
        self._set_path_display(diagnostic=not rows and self.diagnostic_source is not None)
        self._draw_path()
        if review.get("ready") and not blockers:
            fixed = review.get('operation') == FIXED_PREGRASP
            self.state_label.configure(text="固定姿态路径已准备 · 待确认" if fixed else "路径已准备 · 等待批准", fg=PALETTE["teal"])
            self._append_log("固定姿态路径已准备；查看六轴目标与路径后，点击“一键进入预抓取”，到位结束。" if fixed else "准备完成。请查看目标、全部阶段和参数，然后点击执行。")
        else:
            self.state_label.configure(text="准备未通过", fg=PALETTE["red"])
            self._append_log("准备未通过：" + "\n".join(str(reason) for reason in blockers or ["请查看路径详情"]))
            self._show_result(preparation_result(review, self.latest_backend_result))
        self._refresh_controls()

    def _select_stage(self, _event=None):
        selection = self.stages_table.selection()
        if selection:
            details = self.stage_rows.get(selection[0], "")
            self.stage_detail.configure(text="\n".join(details.splitlines()[1:3]))

    def _draw_path(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(500, canvas.winfo_width()), max(110, canvas.winfo_height())
        points = []
        boundaries = []
        offset = 0.0
        for stage in (self.review or {}).get("stages", []):
            samples = stage_samples(stage)
            points.extend((offset + stamp, q) for stamp, q in samples)
            duration = float(stage.get("duration_s") or 0)
            if not duration and samples:
                duration = samples[-1][0]
            offset += duration
            boundaries.append(offset)
        if not points:
            if self.diagnostic_source is not None:
                from PIL import Image, ImageTk
                preview = self.diagnostic_source.copy()
                preview.thumbnail((max(1, width - 20), max(1, height - 12)), Image.Resampling.LANCZOS)
                self.diagnostic_photo = ImageTk.PhotoImage(preview, master=self.root)
                canvas.create_image(width / 2, height / 2, image=self.diagnostic_photo, tags="diagnostic")
                canvas.tag_bind("diagnostic", "<Button-1>", lambda _event: self.open_image())
                return
            canvas.create_text(width / 2, height / 2, text="准备后显示实际计划关节轨迹", fill=PALETTE["muted"], font=(self.font, 10))
            return
        left, top, right, bottom = 48, 21, width - 15, height - 25
        values = [value for _stamp, q in points for value in q[:6]]
        low, high = min(values), max(values)
        margin = max(1.0, (high - low) * 0.08)
        low, high = low - margin, high + margin
        total = max(offset, max(stamp for stamp, _q in points), 0.01)
        x = lambda stamp: left + stamp / total * (right - left)
        y = lambda value: bottom - (value - low) / (high - low) * (bottom - top)
        for index in range(3):
            value = low + (high - low) * index / 2
            canvas.create_line(left, y(value), right, y(value), fill="#e6edf2")
            canvas.create_text(left - 7, y(value), text=f"{value:.1f}", anchor="e", fill=PALETTE["muted"], font=(self.font, 8))
        for stamp in boundaries[:-1]:
            canvas.create_line(x(stamp), top, x(stamp), bottom, fill="#e6edf2", dash=(3, 4))
        joint_count = min(6, max(len(q) for _stamp, q in points))
        for joint in range(joint_count):
            coords = [coordinate for stamp, q in points if len(q) > joint for coordinate in (x(stamp), y(q[joint]))]
            if len(coords) >= 4:
                canvas.create_line(*coords, fill=JOINT_COLORS[joint], width=1.7)
            legend_x = left + joint * 74
            canvas.create_line(legend_x, 8, legend_x + 14, 8, fill=JOINT_COLORS[joint], width=2)
            canvas.create_text(legend_x + 19, 8, text=f"J{joint}", anchor="w", fill=PALETTE["muted"], font=(self.font, 8))
        canvas.create_text(left, height - 7, text="0 s", anchor="w", fill=PALETTE["muted"], font=(self.font, 8))
        canvas.create_text(right, height - 7, text=f"{total:.1f} s", anchor="e", fill=PALETTE["muted"], font=(self.font, 8))

    def _show_result(self, result):
        self.log_dir = str(result.get("run_dir") or self.log_dir or "") or None
        status = str(result.get("status", "unknown"))
        grasp = result.get("grasp_result", "未确认")
        total = result.get("total_time_s")
        total_text = f"{total:.2f} 秒" if isinstance(total, (float, int)) else str(total or "—")
        state_text = STATUS_NAMES.get(status, status)
        if status == 'pregrasp_completed' and result.get('execution_scope') == FIXED_PREGRASP:
            state_text = '已到固定预抓取姿态'
        grasp_text = RESULT_NAMES.get(grasp, grasp) if isinstance(grasp, str) else _short_json(grasp)
        cycle = result.get("cycle_result", "not_attempted" if grasp == "not_attempted" else "unknown")
        cycle_text = RESULT_NAMES.get(cycle, cycle) if isinstance(cycle, str) else _short_json(cycle)
        self.state_label.configure(text=state_text, fg=PALETTE["teal"] if status in ("success", "completed", "demo_completed", 'pregrasp_completed') else PALETTE["amber"])
        self.elapsed_label.configure(text=f"本次执行  {total_text}")
        lines = [f"流程状态：{state_text}  ·  总耗时：{total_text}", f"抓取结果：{grasp_text}  ·  完整抓放结果：{cycle_text}"]
        if self.mode == "demo":
            lines.insert(0, "演示模式 · 未操作实体设备")
        if result.get("error"):
            lines.append("结束原因：" + str(result["error"]))
        if result.get('execution_scope') in (PREGRASP, FIXED_PREGRASP):
            lines.append('执行范围：夹爪调整 → 移动至 Gazebo 固定准备姿态。' if result.get('execution_scope') == FIXED_PREGRASP else '执行范围：夹爪调整 → 移动至预抓取点。')
            if result.get('pregrasp_reached'):
                lines.append('结束说明：' + str(result.get('completion_note') or '已到预抓取点并停止。'))
                if _numbers(result.get('final_angles')):
                    lines.append('到位关节反馈 / SDK：' + _format_q(result['final_angles']))
        notices = result.get("notices") or (result.get("preparation_diagnostics") or {}).get("notices")
        if isinstance(notices, list) and notices:
            lines.extend(["", "执行说明", *map(str, notices)])
        if _numbers(result.get("observed_joint_angles")):
            lines.extend(["", "准备时实际关节 SDK：" + _format_q(result["observed_joint_angles"])])
        lines.extend(["", f"日志目录：{self.log_dir or '—'}"])
        times = result.get("stage_times", {})
        times_text = "\n".join(f"{STAGE_NAMES.get(name, name)}：{value:.3f} 秒" for name, value in times.items()) if isinstance(times, dict) and all(isinstance(value, (int, float)) for value in times.values()) else _short_json(times, 16000)
        preparation_times = result.get("preparation_timings") or []
        if isinstance(preparation_times, list):
            timing_lines = []
            for item in preparation_times:
                if not isinstance(item, dict) or not isinstance(item.get("duration_s"), (int, float)):
                    continue
                name = str(item.get("name", "准备步骤"))
                outcome = item.get("outcome")
                outcome_text={"failed":"未通过","cancelled":"已取消","canceled":"已取消"}.get(outcome,outcome)
                suffix = "" if outcome in (None, "completed") else f"（{outcome_text}）"
                timing_lines.append(f"{PREPARATION_NAMES.get(name, name)}：{item['duration_s']:.3f} 秒{suffix}")
            if timing_lines:
                lines.extend(["", "准备分项耗时", *timing_lines])
        lines.extend(["", "预抓取阶段耗时" if result.get('execution_scope') in (PREGRASP, FIXED_PREGRASP) else "抓放阶段耗时", times_text or ("准备未通过，未开始任何抓放阶段。" if grasp == "not_attempted" else "无阶段执行记录")])
        self._set_text(self.result_text, "\n".join(lines))
        self._append_log(f"{state_text}；抓取结果：{grasp_text}；总耗时：{total_text}。")
        self.notebook.select(1)
        self._refresh_controls()

    def _handle_backend_event(self, event):
        if not isinstance(event, dict):
            self._append_log(str(event))
            return
        kind = event.get("kind", "log")
        message = event.get("message")
        if event.get("run_dir"):
            self.log_dir = str(event["run_dir"])
        if kind == "device_status":
            if isinstance(event.get("data"), dict):
                self.device_snapshot = copy.deepcopy(event["data"])
                self._refresh_device_status()
        elif kind == "joint":
            now = time.monotonic()
            if now - self.last_joint_draw >= 0.15:
                q = event.get("q", event.get("data", {}).get("q") if isinstance(event.get("data"), dict) else [])
                warning=sorted(set(event.get('stale_feedback_joints',[])+event.get('pending_feedback_joints',[])))
                suffix=(' · 反馈待恢复 '+','.join(f'J{i}' for i in warning)) if warning else ''
                self.joint_label.configure(text=("有效关节 SDK  " if event.get('feedback_value_source') else "实际关节 SDK  ")
                                           +_format_q(q)+suffix,
                                           fg=PALETTE['amber'] if warning else PALETTE['ink'])
                self.last_joint_draw = now
        elif kind == "state":
            if message and not self.cancel_requested:
                self.state_label.configure(text=STAGE_NAMES.get(str(message), STATUS_NAMES.get(str(message), str(message))))
            if message:
                self._append_log(STAGE_NAMES.get(str(message), STATUS_NAMES.get(str(message), str(message))))
        elif kind == 'camera_status':
            # The job's return value updates the UI after the helper has closed.
            if isinstance(event.get('data'),dict):self.camera_snapshot=copy.deepcopy(event['data'])
        elif kind == "result":
            # execute() returning is the authoritative completion, after files flush.
            if isinstance(event.get("data"), dict):
                self.latest_backend_result = event["data"]
            if message:
                self._append_log(str(message))
        else:
            self._append_log(str(message) if message is not None else _short_json(event.get("data", event), 2000))

    def _pump(self):
        try:
            for _ in range(200):
                kind, payload = self.events.get_nowait()
                if kind == "backend":
                    self._handle_backend_event(payload)
                elif kind in ("prepare_done", "execute_done", "device_control_done", "camera_services_done", 'shutdown_devices_done'):
                    self.busy = ""
                    if kind == "prepare_done":
                        self._show_review(payload)
                    elif kind == "device_control_done":
                        self._show_device_control(payload)
                    elif kind == "camera_services_done":
                        self._show_camera_services(payload)
                    elif kind == 'shutdown_devices_done':
                        self._show_shutdown_devices(payload)
                    else:
                        self.started_at = None
                        self._show_result(payload)
                    if self.closing:
                        self.root.destroy()
                        return
                elif kind.endswith("_error"):
                    if kind != "cancel_error":
                        self.busy = ""
                        self.started_at = None
                    self.state_label.configure(text="操作未完成 · 查看日志", fg=PALETTE["red"])
                    self._append_log(payload)
                    if kind == "device_control_error":
                        self.device_snapshot = {"status": {}, "status_fresh": False}
                    if kind == 'camera_services_error':
                        self._show_camera_services(dict(completed=False,cameras={},error=payload))
                    if kind == 'shutdown_devices_error':
                        self._show_shutdown_devices(dict(completed=False,error=payload))
                    if kind in ("execute_error", "device_control_error"):
                        self._set_text(self.result_text, ("设备状态操作发生错误" if kind == "device_control_error" else "执行发生错误") + "\n" + payload + "\n\n日志目录：" + (self.log_dir or "—"))
                        self.notebook.select(1)
                    self._refresh_controls()
                    if self.closing and not self.busy:
                        self.root.destroy()
                        return
        except queue.Empty:
            pass
        self.root.after(70, self._pump)

    def _tick(self):
        if self.started_at is not None:
            self.elapsed_label.configure(text=f"本次执行  {time.monotonic() - self.started_at:.1f} 秒")
        self._refresh_controls()
        self.root.after(400, self._tick)


def main(argv=None):
    parser = argparse.ArgumentParser(description="D1 OpenCV 一键视觉抓取桌面程序")
    parser.add_argument("--demo", action="store_true", help="仅演示界面和日志，不连接硬件")
    parser.add_argument("--no-auto-prepare", action="store_true", help="打开界面后不自动准备")
    args = parser.parse_args(argv)
    from app.backend import Backend
    root = tk.Tk()
    app_ref = {}

    def emit(event):
        if "app" in app_ref:
            app_ref["app"].emit(event)

    mode = "demo" if args.demo else "live"
    backend = Backend(mode=mode, emit=emit)
    app_ref["app"] = GraspApp(root, backend, mode=mode, auto_prepare=not args.no_auto_prepare)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
