#!/usr/bin/env python3
"""Launcher for this PiPER and the saved official ACT checkpoint.

Default is read-only --check. --run declares that the master is off, the empty
follower is securely supported, hands are outside the workspace and an operator
can cut power. No automatic reset, homing, calibration or dependency install.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import urllib.request
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
WORKSPACE = PROJECT.parents[1]
DEFAULTS = HERE / "act_launch_defaults.json"
PROBE = r'''
import importlib.util,json,socket,struct,time
spec=importlib.util.spec_from_file_location('rx','/home/ubuntu/kr260-camera/remote_can_source.py')
rx=importlib.util.module_from_spec(spec);spec.loader.exec_module(rx)
identity=rx.interface_identity('can0');link=rx.link_status('can0')
result={'identity':identity,'link':link,'commands_sent':0}
parts={};count=0
if link.get('up') and link.get('bitrate_ok'):
 s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
 s.bind(('can0',));s.settimeout(.05);end=time.monotonic()+.45
 while time.monotonic()<end:
  try:p=s.recv(16)
  except socket.timeout:continue
  i,n,d=struct.unpack('=IB3x8s',p);count+=1
  if n==8:parts[i]=(d,time.monotonic())
 s.close()
now=time.monotonic();required=(0x2a1,0x2a5,0x2a6,0x2a7,0x2a8,*range(0x261,0x267))
result['frames_received']=count
result['complete_fresh_feedback']=all(i in parts and now-parts[i][1]<.2 for i in required)
if result['complete_fresh_feedback']:
 result['state']=[v*.001 for i in (0x2a5,0x2a6,0x2a7) for v in struct.unpack('>ii',parts[i][0])]+[int.from_bytes(parts[0x2a8][0][:4],'big',signed=True)*1e-6]
 result['enabled']=[bool(parts[i][0][5]&0x40) for i in range(0x261,0x267)]
 result['motor_faults']=[parts[i][0][5]&0xbf for i in range(0x261,0x267)]
 result['arm_status_raw']=parts[0x2a1][0].hex()
 result['gripper_raw']=parts[0x2a8][0].hex()
print(json.dumps(result))
'''


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n")


def endpoint(base, host):
    return ["ssh", "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", "ConnectTimeout=5", "-o", "HostName="+host, base["transport"]["ssh_host"]]


def probe(base, host):
    proc = subprocess.run(endpoint(base, host)+["python3 -"], input=PROBE,
                          capture_output=True, text=True, timeout=12, check=False)
    if proc.returncode:
        raise RuntimeError("KR260 检查失败："+proc.stderr.strip()[-500:])
    result = json.loads(proc.stdout)
    expected = dict(base["transport"]["expected_identity"])
    observed = dict(result["identity"])
    # ifindex is recreated on boot. All physical identity fields stay pinned.
    expected.pop("ifindex", None);observed.pop("ifindex", None)
    if observed != expected:
        raise RuntimeError("CAN 板卡/USB/机器身份变化，请人工核对；不会自动接受新硬件")
    return result


def camera_checks(base):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    checks = {}
    for role, config in base["cameras"].items():
        try:
            with opener.open(config["url"]+"/status", timeout=2) as response:
                status = json.load(response)
            age = status.get("streams", {}).get("color", {}).get("age_ms")
            valid = (status.get("role") == role
                     and status.get("device", {}).get("serial") == config["serial"]
                     and not status.get("stopped") and not status.get("error")
                     and isinstance(age, (int, float)) and 0 <= age <= 150)
            checks[role] = {"ready": valid, "serial": config["serial"], "age_ms": age,
                            "error": status.get("error")}
        except Exception as error:
            checks[role] = {"ready": False, "error": str(error)}
    return checks


def require_start_pose(result):
    if not result.get("complete_fresh_feedback"):
        raise RuntimeError("没有完整新鲜的从臂反馈，请检查从臂电源、急停和 CAN 线")
    if any(result["motor_faults"]):
        raise RuntimeError("六轴反馈包含故障，拒绝使能")
    status = bytes.fromhex(result["arm_status_raw"])
    grip = bytes.fromhex(result["gripper_raw"])
    if status[1] or int.from_bytes(status[6:8], "big") or grip[6] & 0x3f or grip[7] != 0:
        raise RuntimeError("控制器/夹爪故障或夹爪不在宽度模式")
    enabled = result["enabled"]
    if len(enabled) != 6 or (any(enabled) and not all(enabled)):
        raise RuntimeError("六轴使能状态不一致，不能直接接管")
    if all(enabled) and (status[0] != 1 or status[2] != 1 or status[3] != 0):
        raise RuntimeError("已使能从臂必须处于正常 CAN/J 状态才能接管")
    if not any(enabled) and status[0] not in (0, 1):
        raise RuntimeError(f"当前控制模式不能接管(ctrl_mode={status[0]})")


def build_run(defaults, base, result, directory, host):
    require_start_pose(result)
    work_bounds = defaults["policy"]["limits"]["bounds"]
    controller_bounds = defaults["verified_controller_joint_bounds_deg"]
    if len(work_bounds) != 7 or len(controller_bounds) != 6:
        raise ValueError("Six verified controller ranges and seven operating ranges required")
    for index, ((low, high), (controller_low, controller_high)) in enumerate(
            zip(work_bounds[:6], controller_bounds, strict=True), 1):
        if (type(low) not in (int, float) or type(high) not in (int, float)
                or not controller_low <= low < high <= controller_high):
            raise ValueError(f"joint_{index} operating range exceeds verified controller limits")
    rate_fraction = defaults["policy_target_rate_fraction"]
    if (isinstance(rate_fraction, bool) or not isinstance(rate_fraction, (int, float))
            or not 0 < rate_fraction <= .95):
        raise ValueError("policy_target_rate_fraction must be in (0, 0.95]")
    joint_margin = defaults.get("policy_joint_target_margin_deg")
    joint_bounds = None
    if "policy_joint_target_margin_deg" in defaults:
        if any(key in defaults for key in ("j5_upper_prediction_saturation",
                "j5_upper_prediction_tolerance_deg", "j5_upper_target_margin_deg")):
            raise ValueError("policy_joint_target_margin_deg and legacy J5 settings are mutually exclusive")
        if type(joint_margin) not in (int, float) or not 0 <= joint_margin <= 5:
            raise ValueError("policy_joint_target_margin_deg must be a finite JSON number in [0, 5]")
        joint_bounds = [(lo+joint_margin, hi-joint_margin)
                        for lo, hi in defaults["policy"]["limits"]["bounds"][:6]]
        if any(lo > hi for lo, hi in joint_bounds):
            raise ValueError("Joint target margin leaves no feasible target interval")
    gripper_margin = defaults.get("policy_gripper_target_margin_m")
    if "policy_gripper_target_margin_m" in defaults:
        if joint_bounds is None:
            raise ValueError("policy_gripper_target_margin_m requires joint prediction projection")
        if type(gripper_margin) not in (int, float) or not 0 <= gripper_margin <= .005:
            raise ValueError("policy_gripper_target_margin_m must be a finite JSON number in [0, 0.005]")
        low, high = work_bounds[6]
        if low+gripper_margin > high-gripper_margin:
            raise ValueError("Gripper target margin leaves no feasible target interval")
    smoothing_alpha = defaults.get("policy_joint_smoothing_alpha")
    if "policy_joint_smoothing_alpha" in defaults:
        if type(smoothing_alpha) not in (int,float) or not .5 <= smoothing_alpha <= 1.:
            raise ValueError("policy_joint_smoothing_alpha must be a finite JSON number in [0.5, 1]")
        if joint_bounds is None:
            raise ValueError("policy_joint_smoothing_alpha requires joint prediction projection")
    if "policy_release_chunk_extension" in defaults:
        from act_release_continuation import validate_release_continuation
        validate_release_continuation(defaults['policy_release_chunk_extension'], bounds=work_bounds)
        if joint_bounds is None or gripper_margin is None:
            raise ValueError('policy_release_chunk_extension requires joint and gripper prediction projection')
    j5_saturation = defaults.get("j5_upper_prediction_saturation", False)
    if type(j5_saturation) is not bool:
        raise ValueError("j5_upper_prediction_saturation must be a JSON boolean")
    if "j5_upper_prediction_saturation" in defaults and "j5_upper_prediction_tolerance_deg" in defaults:
        raise ValueError("j5_upper_prediction_saturation and j5_upper_prediction_tolerance_deg are mutually exclusive")
    j5_tolerance = defaults.get("j5_upper_prediction_tolerance_deg")
    if "j5_upper_prediction_tolerance_deg" in defaults and (
            isinstance(j5_tolerance, bool) or not isinstance(j5_tolerance, (int, float))
            or not 0 <= j5_tolerance <= 1.):
        raise ValueError("j5_upper_prediction_tolerance_deg must be in [0, 1]")
    j5_margin = defaults.get("j5_upper_target_margin_deg", 0.)
    if "j5_upper_target_margin_deg" in defaults:
        if not j5_saturation:
            raise ValueError("j5_upper_target_margin_deg requires J5 prediction saturation")
        if type(j5_margin) not in (int, float) or not 0 <= j5_margin <= 5.:
            raise ValueError("j5_upper_target_margin_deg must be a finite JSON number in [0, 5]")
        low, high = defaults["policy"]["limits"]["bounds"][4]
        if min(70., high)-j5_margin < low:
            raise ValueError("j5_upper_target_margin_deg leaves no valid J5 target interval")
    sys.path.insert(0, str(HERE))
    from preflight_j_hold import validate_review
    q = result["state"]
    reuse_enabled = all(result["enabled"])
    takeover_speed = defaults["enabled_takeover_speed_percent"]
    if type(takeover_speed) is not int or not 1 <= takeover_speed <= 10:
        raise ValueError("enabled_takeover_speed_percent must be an integer from1to10")
    # Cold recovery keeps its original low-speed path; only an already enabled
    # healthy arm uses the reviewed faster takeover for this run.
    speed_percent = takeover_speed if reuse_enabled else 5
    target = q[:6] if reuse_enabled else [q[0], 1.0, -1.0, *q[3:6]]
    review = {"expected_identity": result["identity"],
              "joint_bounds_deg": defaults["verified_controller_joint_bounds_deg"],
              "expected_pose_deg_m": q, "initial_target_deg": target,
              "start_joint_tolerance_deg": 1., "start_gripper_tolerance_m": .002,
              "max_deviation_deg": 4.5, "support_and_disable_on_fault_reviewed": True,
              "gripper_hold_force_n": 1.,
              "approval_note": "Operator invoked --run for the saved ACT task, affirming master off, supported empty follower, hands clear and onsite power cutoff. Explicit J2=1/J3=-1 recovery <=4deg, J5percent and current-width gripper1N."}
    if reuse_enabled:
        review.pop("initial_target_deg")
        review.pop("gripper_hold_force_n")
        review["reuse_enabled"] = True
        review["takeover_speed_percent"] = speed_percent
        review["max_deviation_deg"] = 2.
        review["warm_gripper_target_m"] = .01 if q[6] < 0 else q[6]
        review["approval_note"] = f"User requests more frequent ACT feedback and improved arm/gripper timing from the powered enabled arm. Preserve six current joint targets, reviewed CAN/J{speed_percent}%, empty gripper1N; open to10mm only if closed feedback is slightly negative. No reset, disable or enable requests."
    validate_review(review)
    io = json.loads(json.dumps(base))
    io["transport"]["ssh_hostname"] = host
    io["transport"]["expected_identity"] = result["identity"]
    save(directory/"io_config.json", io)
    save(directory/"j_hold_review.json", review)
    refs = {
        "leader_isolation": "Operator --run declaration: master remains powered off.",
        "firmware_and_j_mode": str(directory/"startup_result.json")+" must pass before arm; receiver observes actual low-speed151.",
        "hardware_and_fixture_limits": "Pinned PiPER controller limits from controller_limits.json; conservative trial ranges, operator hands-clear declaration; no collision-check guarantee.",
        "signed_gripper_mapping": "Unchanged signed micrometre raw width, bounded0..0.1m; empty current-width hold and1N; no zero/offset change.",
        "verified_stop_strategy": "Supported current-target hold preflight must pass; healthy-feedback stop, no guaranteed hold on feedback/power loss; onsite power cutoff required.",
        "robot_side_lease_and_watchdog": "Receiver identity/clock checks, independent owner/sequence/TTL/lease; offline-tested; source age and send TTL are separate.",
        "approved_goal_path_parameters": f"Operator --run requests the saved red-cube-to-left-black-target ACT task, {speed_percent}-percent J,1N, bounded duration; no home/reset/calibration; gate rejection ends trial."}
    refs["approved_goal_path_parameters"] += " Official relative-target cap:2deg per joint and3mm gripper; actual feedback bounds remain strict."
    refs["approved_goal_path_parameters"] += f" Policy targets are additionally limited to{rate_fraction*100:g}percent of the existing target-speed limits on the actual CAN grid; hard receiver limits remain unchanged."
    if joint_bounds is not None:
        refs["approved_goal_path_parameters"] += f" Finite ACT six-joint predictions project into the operating bounds with{joint_margin:g}deg inward margin at both ends. Final feedback/rate/CAN-grid intersection enforces those target bounds. Actual feedback keeps its original strict bounds."
        if gripper_margin is not None:
            refs["approved_goal_path_parameters"] += f" Finite ACT gripper proposals also project into the unchanged gripper working range with{gripper_margin:g}m inward target margin at both ends; no negative target, zero calibration or force change."
        else:
            refs["approved_goal_path_parameters"] += " Gripper input keeps its original strict bounds."
    elif j5_saturation:
        refs["approved_goal_path_parameters"] += f" Finite ACT J5 upper predictions saturate at min(training70deg,guard_upper)-{j5_margin:g}deg. The final feedback/rate/CAN-grid intersection also enforces this interior target upper bound; all actual feedback and other bounds remain hard checks."
    elif j5_tolerance is not None:
        refs["approved_goal_path_parameters"] += f" Only J5 upper raw predictions within{j5_tolerance:g}deg above the configured upper bound may project back to that bound; other/larger violations and actual feedback remain hard errors."
    if smoothing_alpha is not None:
        refs["approved_goal_path_parameters"] += f" Six-joint feedback-capped proposals blend{100*smoothing_alpha:g}percent new target with the last committed target before the original rate/bounds/CAN-grid intersection; gripper and source deadlines are not filtered."
    if "policy_release_chunk_extension" in defaults:
        refs['approved_goal_path_parameters'] += ' In the recorded placement joint window, an acknowledged ACT opening proposal may continue up to20 following actions from the same original prediction if actual opening is incomplete. Hold the last committed joint targets until actual jaws reach60mm and settle150ms; abort if unconfirmed within2s. No forced gripper target, new source timestamp or changed guard limit; joint window is not table-contact confirmation.'
    if reuse_enabled:
        refs["signed_gripper_mapping"] += " Enabled takeover only: closed feedback in[-1mm,0) opens to10mm at1N; no signed offset or zero calibration."
        refs["verified_stop_strategy"] = "Enabled current-pose hold checked; failure never resets or disables motors. Healthy-feedback stop retains current target; physical hold is not guaranteed on feedback/power loss."
    policy = json.loads(json.dumps(defaults["policy"]))
    policy["expected_motion_ctrl2"] = [1, 1, speed_percent, 0, 0, 0, 0, 0]
    policy["readiness"] = refs
    save(directory/"readiness.json", refs)
    save(directory/"act_trial_policy.json", policy)
    remote = defaults["remote_directory"]
    config = {"execute": True, "io_config": str(directory/"io_config.json"),
              "expected_ifindex": result["identity"]["ifindex"],
              "remote_bridge": remote+"/remote_bridge.py",
              "remote_rx_helper": "/home/ubuntu/kr260-camera/remote_can_source.py",
              "remote_execution_policy": remote+"/act_trial_policy.json",
              "expected_motion_ctrl2": policy["expected_motion_ctrl2"],
              "guard_limits": policy["limits"], "normal_gripper_force_n": 1.,
              "tx_ttl_s": defaults["tx_ttl_s"], "rpc_timeout_s": 1.}
    config["max_relative_target"] = {**{f"joint_{i}.deg": 2. for i in range(1,7)}, "gripper.m": .003}
    config["policy_target_rate_fraction"] = rate_fraction
    if smoothing_alpha is not None:
        config["policy_joint_smoothing_alpha"] = smoothing_alpha
    if "policy_release_chunk_extension" in defaults:
        config['policy_release_chunk_extension'] = defaults['policy_release_chunk_extension']
    if joint_bounds is not None:
        config["policy_joint_target_margin_deg"] = joint_margin
        if gripper_margin is not None:
            config["policy_gripper_target_margin_m"] = gripper_margin
    if "j5_upper_prediction_saturation" in defaults:
        config["j5_upper_prediction_saturation"] = j5_saturation
        if "j5_upper_target_margin_deg" in defaults:
            config["j5_upper_target_margin_deg"] = j5_margin
    elif j5_tolerance is not None:
        config["j5_upper_prediction_tolerance_deg"] = j5_tolerance
    config["policy_target_trace_path"] = str((directory/"policy_targets.jsonl").resolve())
    save(directory/"execute_config.json", config)
    command = endpoint(base, host)+[shlex.join(["python3", "-S", remote+"/preflight_j_hold.py",
               "--prepare-j-hold-supported-reviewed", "--review-json", remote+"/j_hold_review.json"])]
    save(directory/"startup_command.json", command)
    return review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="默认：只检查；不使能、不发运动目标")
    mode.add_argument("--run", action="store_true", help="现场条件已确认，准备后运行 ACT；主臂必须断电、从臂稳固承托且空载、手退出、可现场断电")
    parser.add_argument("--duration", type=float, default=30, help="ACT最长运行秒数，默认30，上限60")
    parser.add_argument("--n-action-steps", type=int,
                        help="每次ACT预测后执行的动作数；默认读取当前入口的配置")
    parser.add_argument("--host", help="KR260地址，默认10.42.0.55")
    args = parser.parse_args()
    if not 0 < args.duration <= 60:
        parser.error("duration 必须在 (0,60] 秒")
    defaults = read(DEFAULTS);base = read(defaults["base_io_config"])
    n_action_steps = args.n_action_steps if args.n_action_steps is not None else defaults["n_action_steps"]
    if type(n_action_steps) is not int or not 1 <= n_action_steps <= 100:
        parser.error("n-action-steps 必须是1到100的整数")
    host = args.host or defaults["host"]
    if not re.fullmatch(r"[A-Za-z0-9.:-]+", host):
        parser.error("无效KR260地址")
    checkpoint = Path(defaults["checkpoint"])
    weights = checkpoint/"model.safetensors"
    if not weights.is_file():
        raise RuntimeError("保存的 ACT 权重缺失")
    with weights.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest != defaults["model_sha256"]:
        raise RuntimeError("保存的 ACT 权重缺失或校验不符")
    if args.run:
        print("启动条件：主臂断电；从臂空载且稳固承托；手退出；现场可断电。", flush=True)
        print(f"计划：已使能时保持当前六轴姿态直接接管，J速度比例{defaults['enabled_takeover_speed_percent']}%、夹爪1N；冷启动恢复及该次运行保持5%。", flush=True)
        print(f"ACT每次执行{n_action_steps}步后重新预测，20Hz目标节奏下约{n_action_steps/20:g}秒，另加实际推理耗时。", flush=True)
        if "policy_joint_smoothing_alpha" in defaults:
            print(f"六关节轻度平滑：新目标权重{defaults['policy_joint_smoothing_alpha']:g}；夹爪不滤波，原保护继续检查。", flush=True)
        if "policy_release_chunk_extension" in defaults:
            print('放置松爪衔接：模型已要求松爪且实际尚未张开时，可沿用同一次预测的后续最多20步；普通阶段仍按上述步数重新预测。',flush=True)
            print('回撤等待：实际夹爪张开到60mm并稳定150ms后放行；最长等2秒，未确认则停止；此反馈不代表已接触桌面。',flush=True)
        if "policy_joint_target_margin_deg" in defaults:
            margin = defaults["policy_joint_target_margin_deg"]
            bounds = [[lo+margin, hi-margin] for lo, hi in defaults["policy"]["limits"]["bounds"][:6]]
            print(f"六关节策略目标范围deg：{bounds}；实际反馈仍检查原硬范围。", flush=True)
        if "policy_gripper_target_margin_m" in defaults:
            margin = defaults["policy_gripper_target_margin_m"]
            low, high = defaults["policy"]["limits"]["bounds"][6]
            print(f"夹爪有限策略预测映射到{(low+margin)*1000:g}～{(high-margin)*1000:g}mm目标范围；原实际反馈范围、限速和1N力度不变。", flush=True)
        if defaults.get("j5_upper_prediction_saturation") is True:
            hard_upper = defaults["policy"]["limits"]["bounds"][4][1]
            target_upper = min(70., hard_upper)-defaults.get("j5_upper_target_margin_deg", 0.)
            print(f"J5策略目标上限{target_upper:g}°；实际反馈硬上限{hard_upper:g}°，不归一化或放宽实际反馈。", flush=True)
    result = probe(base, host)
    if args.run and (not result["link"].get("up") or result["link"].get("listen_only")
                     or not result["link"].get("bitrate_ok")):
        print("准备CAN网卡正常1Mbps；如提示sudo密码，请在终端输入，不会保存。", flush=True)
        command = endpoint(base, host)
        command[1] = "-tt"
        subprocess.run(command+["sudo -v && sudo -n ip link set can0 down && sudo -n ip link set can0 type can bitrate 1000000 listen-only off && sudo -n ip link set can0 up"], check=True)
        result = probe(base, host)
    if args.run:
        subprocess.run([sys.executable, "-B", str(PROJECT/"start_cameras.py")], check=True)
    cameras = camera_checks(base)
    print(json.dumps({"checkpoint": str(checkpoint), "CAN_frames": result.get("frames_received", 0),
        "fresh_feedback": result.get("complete_fresh_feedback", False), "state": result.get("state"),
        "enabled": result.get("enabled"), "cameras": cameras}, ensure_ascii=False, indent=2), flush=True)
    require_start_pose(result)
    if not all(value["ready"] for value in cameras.values()):
        raise RuntimeError("双相机未就绪；--run会尝试恢复原相机服务，仍失败请检查USB连接")
    directory = WORKSPACE/"outputs/act_runs"/datetime.now(ZoneInfo("Asia/Singapore")).strftime("%Y%m%d_%H%M%S_%f")
    directory.mkdir(parents=True)
    save(directory/"read_only_probe.json", result)
    # Refresh after camera startup so the reviewed pose is not an old startup sample.
    if args.run:
        result = probe(base, host)
        save(directory/"read_only_probe.json", result)
    review = build_run(defaults, base, result, directory, host)
    print("本次关节保持目标 deg："+str(review.get("initial_target_deg", review["expected_pose_deg_m"][:6])), flush=True)
    print("本次文件："+str(directory), flush=True)
    if not args.run:
        print("CHECK通过：未配置CAN、未使能、未发运动目标。下一步用 --run。", flush=True)
        return 0
    print("逐帧诊断："+str(directory/"policy_targets.jsonl")+"（预测、反馈与夹爪时序；ACK不代表实际到位）", flush=True)
    # Copy only independent hardware adapters/configs; never modify the RX helper.
    subprocess.run(endpoint(base, host)+["mkdir -p "+shlex.quote(defaults["remote_directory"])], check=True)
    scp = ["scp", "-q", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "HostName="+host]
    files = [HERE/name for name in ("remote_bridge.py", "guard.py")]
    files = [HERE/"lerobot_robot_piper_executor"/p.name for p in files]
    files += [HERE/"reset_piper_once.py", HERE/"preflight_j_hold.py", directory/"j_hold_review.json", directory/"act_trial_policy.json"]
    subprocess.run(scp+[str(p) for p in files]+[base["transport"]["ssh_host"]+":"+defaults["remote_directory"]+"/"], check=True)
    argv = [sys.executable, "-u", "-B", str(PROJECT/"official_adapter/rollout_act_piper.py"),
        "--execute", "--checkpoint", str(checkpoint), "--deployment-config", str(directory/"execute_config.json"),
        "--readiness", str(directory/"readiness.json"), "--readiness-reviewed", "--duration", str(args.duration),
        "--n-action-steps", str(n_action_steps),
        "--threads", str(defaults["inference_threads"]),
        "--warmup-passes", str(defaults["warmup_passes"]),
        "--startup-command-json", str(directory/"startup_command.json"), "--startup-result", str(directory/"startup_result.json")]
    with (directory/"rollout.log").open("w") as log, subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as proc:
        try:
            for line in proc.stdout:
                log.write(line);log.flush();print(line, end="", flush=True)
            return proc.wait()
        except KeyboardInterrupt:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.terminate();proc.wait(timeout=3)
            return 130


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as error:
        print("ACT启动已中止："+str(error), file=sys.stderr)
        raise SystemExit(2)
