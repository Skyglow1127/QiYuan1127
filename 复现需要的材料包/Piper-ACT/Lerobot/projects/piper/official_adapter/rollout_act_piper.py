#!/usr/bin/env python3
"""Check an ACT rollout plan; explicit reviewed gates are required for execution.

The control loop, inference engine and teardown belong to official LeRobot
0.6.1. No Robot is constructed by the default/check-only branch. Execution can
run one explicitly supplied, reviewed startup preflight after RX is connected;
the launcher itself adds no reset, calibration, home or return-to-start motion.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import fields
import hashlib
import importlib.metadata
import json
import logging
import math
import os
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
DEPLOYMENT = HERE.parent / "deployment"
DEFAULT_CHECKPOINT = Path(
    "/home/aup/Lerobot/models/ACT/rad_place_v5_r05/020000/pretrained_model"
)
CANONICAL_KEYS = tuple([f"joint_{i}.deg" for i in range(1, 7)] + ["gripper.m"])
ROLLOUT_KEYS = tuple([f"joint_{i}.pos" for i in range(1, 7)] + ["gripper.pos"])
TASK = "Pick up the red cube and place it inside the black square on the left side of the table."
LOG = logging.getLogger("piper_act_rollout")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true", help="Validate and print a plan; no hardware construction.")
    mode.add_argument("--execute", action="store_true", help="Run only after all explicit review/config gates pass.")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--deployment-config", type=Path, default=DEPLOYMENT / "observe_config.json")
    parser.add_argument("--readiness", type=Path,
                        help="Reviewed JSON containing the seven Readiness evidence-reference strings.")
    parser.add_argument("--readiness-reviewed", action="store_true",
                        help="Explicitly attest that the supplied Readiness evidence and this plan were reviewed.")
    startup = parser.add_mutually_exclusive_group()
    startup.add_argument("--startup-command-json", type=Path,
                         help="Trusted local JSON argv array for the single reviewed preflight SSH command.")
    startup.add_argument("--existing-mode-evidence", type=Path,
                         help="Reviewed local JSON evidence that this receiver already observed the required mode.")
    parser.add_argument("--startup-result", type=Path,
                        help="Save the actual preflight JSON and launcher acceptance before arming.")
    parser.add_argument("--duration", type=float, default=5.0, help="Finite rollout limit in seconds; first plan defaults to 5.")
    parser.add_argument("--n-action-steps", type=int,
                        help="Actions consumed from each ACT prediction; default: saved checkpoint setting.")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--warmup-passes", type=int, default=3,
                        help="Offline ACT predictions before any hardware connection; queues are reset afterwards.")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive; infinite rollout is unavailable here")
    if args.threads < 1:
        parser.error("--threads must be positive")
    if args.warmup_passes < 1:
        parser.error("--warmup-passes must be positive")
    if args.n_action_steps is not None and args.n_action_steps < 1:
        parser.error("--n-action-steps must be positive")
    return args


def read_json(path):
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_startup_command(path):
    argv = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(argv, list) or not argv
            or not all(isinstance(value, str) and value and "\0" not in value for value in argv)):
        raise ValueError("Startup command JSON must be a nonempty string argv array.")
    payload = " ".join(argv[1:])
    if (Path(argv[0]).name != "ssh" or "preflight_j_hold.py" not in payload
            or "--prepare-j-hold-supported-reviewed" not in payload or "--review-json" not in payload):
        raise ValueError("Startup argv must explicitly name the reviewed preflight SSH invocation.")
    return argv


def startup_mode_bytes(mode):
    if (not isinstance(mode, (list, tuple, bytes)) or len(mode) != 8
            or any(type(value) is not int or not 0 <= value <= 255 for value in mode)
            or tuple(mode[:2]) != (1, 1) or not 1 <= mode[2] <= 10
            or tuple(mode[3:]) != (0, 0, 0, 0, 0)):
        raise ValueError("Expected an explicit CAN/J/POS_VEL mode with speed1..10 and no installation change")
    return bytes(mode)


def validate_startup_result(result, *, expected_motion_ctrl2=(1, 1, 5, 0, 0, 0, 0, 0)):
    if not isinstance(result, dict) or result.get("success") is not True:
        raise RuntimeError("Startup preflight did not report success=true.")
    if result.get("holds_current_pose_enabled") is not True:
        raise RuntimeError("Startup preflight did not confirm the enabled hold.")
    enable_events = [item for item in result.get("tx_log", []) if item.get("phase") == "enable"]
    expected_mode = startup_mode_bytes(expected_motion_ctrl2)
    if result.get("requested_mode") != expected_mode.hex():
        raise RuntimeError("Startup requested_mode does not match this run's reviewed speed/mode.")
    mode_events = [item for item in result.get("tx_log", []) if item.get("phase") == "mode"]
    if len(mode_events) != 1 or mode_events[0].get("data") != expected_mode.hex():
        raise RuntimeError("Startup mode request does not match this run's reviewed speed/mode.")
    reuse = result.get("reused_enabled") is True
    if reuse:
        tx = result.get("tx_log", [])
        if (enable_events or any(item.get("id") in ("0x150", "0x471") for item in tx)
                or any(item.get("socket_accepted") is not True for item in tx)):
            raise RuntimeError("Enabled takeover must not reset, enable or disable motors.")
        last_enable = float(mode_events[0]["completed_at"])
    elif (len(enable_events) != 6
            or any(item.get("socket_accepted") is not True for item in enable_events)):
        raise RuntimeError("Startup preflight did not confirm all six enable sends.")
    else:
        last_enable = max(float(item["completed_at"]) for item in enable_events)
    if not math.isfinite(last_enable):
        raise RuntimeError("Startup enable timestamp is invalid.")
    for key in ("enabled", "after"):
        sample = result.get(key, {})
        enabled = sample.get("enabled", [])
        status_bytes = sample.get("motor_status_bytes", [])
        sources = sample.get("motor_source_monotonic", [])
        ages = sample.get("motor_ages_s", [])
        if len(enabled) != 6 or any(value is not True for value in enabled):
            raise RuntimeError(f"Startup {key} has not confirmed six enabled axes.")
        if (len(status_bytes) != 6
                or any(type(value) is not int or not 0 <= value <= 255 or value & 0xbf
                       for value in status_bytes)):
            raise RuntimeError(f"Startup {key} reports motor fault flags.")
        if (len(sources) != 6 or len(ages) != 6
                or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                       or value <= last_enable for value in sources)
                or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                       or not 0 <= value < .2 for value in ages)):
            raise RuntimeError(f"Startup {key} lacks fresh post-enable feedback.")
        status = bytes.fromhex(sample.get("arm_status_raw", ""))
        if (len(status) != 8 or status[0] != 1 or status[1] != 0
                or status[2] != 1 or int.from_bytes(status[6:8], "big") != 0):
            raise RuntimeError(f"Startup {key} is not healthy CAN/J controller feedback.")


def run_startup_once(argv, result_path, *, run=subprocess.run,
                     expected_motion_ctrl2=(1, 1, 5, 0, 0, 0, 0, 0)):
    """One bounded invocation; persist failures as well, and never retry."""
    result_path = result_path.expanduser().resolve()
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result = {}
    audit = {"argv": argv, "accepted_by_launcher": False, "attempts": 1}
    try:
        completed = run(argv, shell=False, stdin=subprocess.DEVNULL, capture_output=True,
                        text=True, timeout=30, check=False)
        audit["return_code"] = completed.returncode
        audit["stderr"] = completed.stderr
        result = json.loads(completed.stdout)
        if not isinstance(result, dict):
            raise RuntimeError("Startup stdout is not a JSON object.")
        if completed.returncode != 0:
            detail = result.get("error") or completed.stderr.strip()[-500:] or "No failure detail reported"
            raise RuntimeError(f"Startup command exited with status {completed.returncode}: {detail}. Result: {result_path}")
        validate_startup_result(result, expected_motion_ctrl2=expected_motion_ctrl2)
        audit["accepted_by_launcher"] = True
        return result
    except BaseException as exc:
        audit["error"] = str(exc) or type(exc).__name__
        if not isinstance(result, dict):
            result = {"success": False, "raw_result": result}
        raise
    finally:
        result["launcher"] = audit
        result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                               encoding="utf-8")


def local_gate_status(args, deployment, Readiness, GuardLimits):
    """Validate evidence/config without instantiating a backend or Robot."""
    missing = []
    readiness = None
    if type(deployment.get("execute")) is not bool:
        raise ValueError("Deployment execute must be an explicit JSON boolean.")
    if deployment["execute"] is not True:
        missing.append("deployment config execute must be true")
    if not args.readiness_reviewed:
        missing.append("--readiness-reviewed must explicitly acknowledge review")
    if args.readiness is None:
        missing.append("--readiness must name the reviewed evidence JSON")
    else:
        evidence = read_json(args.readiness.expanduser().resolve())
        permitted = {item.name for item in fields(Readiness)}
        if set(evidence) - permitted or not all(isinstance(value, str) for value in evidence.values()):
            raise ValueError("Readiness must contain only the seven documented string fields.")
        readiness = Readiness(**evidence)
        try:
            readiness.require_complete()
        except Exception as exc:
            missing.append(str(exc))
    limits = deployment.get("guard_limits")
    if limits is None:
        missing.append("reviewed guard_limits are absent")
    else:
        GuardLimits(**limits)
    rate_fraction = deployment.get("policy_target_rate_fraction")
    if rate_fraction is not None:
        if (isinstance(rate_fraction, bool) or not isinstance(rate_fraction, (int, float))
                or not math.isfinite(rate_fraction) or not 0 < rate_fraction <= .95):
            raise ValueError("policy_target_rate_fraction must be in (0, 0.95]")
        relative_caps = deployment.get("max_relative_target")
        if (not isinstance(relative_caps, dict) or set(relative_caps) != set(CANONICAL_KEYS)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0 for value in relative_caps.values())):
            raise ValueError("Policy target-rate shaping requires all seven relative target caps")
    if "policy_joint_target_margin_deg" in deployment:
        margin = deployment["policy_joint_target_margin_deg"]
        if any(key in deployment for key in ("j5_upper_prediction_saturation",
                "j5_upper_prediction_tolerance_deg", "j5_upper_target_margin_deg")):
            raise ValueError("policy_joint_target_margin_deg and legacy J5 settings are mutually exclusive")
        if type(margin) not in (int, float) or not 0 <= margin <= 5:
            raise ValueError("policy_joint_target_margin_deg must be a finite JSON number in [0, 5]")
        if limits is None or any(lo+margin > hi-margin for lo, hi in limits["bounds"][:6]):
            raise ValueError("Joint target margin leaves no feasible target interval")
        if rate_fraction is None or deployment.get("max_relative_target") is None:
            raise ValueError("Joint prediction projection requires feedback-cap and target-rate shaping")
    if "policy_gripper_target_margin_m" in deployment:
        margin = deployment["policy_gripper_target_margin_m"]
        if "policy_joint_target_margin_deg" not in deployment:
            raise ValueError("policy_gripper_target_margin_m requires joint prediction projection")
        if type(margin) not in (int, float) or not 0 <= margin <= .005:
            raise ValueError("policy_gripper_target_margin_m must be a finite JSON number in [0, 0.005]")
        if limits is None or limits["bounds"][6][0]+margin > limits["bounds"][6][1]-margin:
            raise ValueError("Gripper target margin leaves no feasible target interval")
    if "policy_joint_smoothing_alpha" in deployment:
        alpha = deployment["policy_joint_smoothing_alpha"]
        if type(alpha) not in (int,float) or not .5 <= alpha <= 1.:
            raise ValueError("policy_joint_smoothing_alpha must be a finite JSON number in [0.5, 1]")
        if ("policy_joint_target_margin_deg" not in deployment or rate_fraction is None
                or deployment.get("max_relative_target") is None):
            raise ValueError("Joint smoothing requires joint prediction projection, feedback-cap and target-rate shaping")
    if "policy_release_chunk_extension" in deployment:
        from act_release_continuation import validate_release_continuation
        if (limits is None or 'policy_joint_target_margin_deg' not in deployment
                or 'policy_gripper_target_margin_m' not in deployment
                or rate_fraction is None or deployment.get('max_relative_target') is None):
            raise ValueError('policy_release_chunk_extension requires joint/gripper projection, feedback-cap and target-rate shaping')
        validate_release_continuation(deployment['policy_release_chunk_extension'], bounds=limits['bounds'])
    saturation = deployment.get("j5_upper_prediction_saturation", False)
    if type(saturation) is not bool:
        raise ValueError("j5_upper_prediction_saturation must be a JSON boolean")
    if "j5_upper_prediction_saturation" in deployment and "j5_upper_prediction_tolerance_deg" in deployment:
        raise ValueError("j5_upper_prediction_saturation and j5_upper_prediction_tolerance_deg are mutually exclusive")
    tolerance = deployment.get("j5_upper_prediction_tolerance_deg")
    if "j5_upper_prediction_tolerance_deg" in deployment and (isinstance(tolerance, bool)
            or not isinstance(tolerance, (int, float)) or not math.isfinite(tolerance)
            or not 0 <= tolerance <= 1.):
        raise ValueError("j5_upper_prediction_tolerance_deg must be in [0, 1]")
    if "j5_upper_target_margin_deg" in deployment:
        margin = deployment["j5_upper_target_margin_deg"]
        if not saturation:
            raise ValueError("j5_upper_target_margin_deg requires J5 prediction saturation")
        if type(margin) not in (int, float) or not 0 <= margin <= 5.:
            raise ValueError("j5_upper_target_margin_deg must be a finite JSON number in [0, 5]")
        if limits is None or min(70., limits["bounds"][4][1])-margin < limits["bounds"][4][0]:
            raise ValueError("j5_upper_target_margin_deg leaves no valid J5 target interval")
        if rate_fraction is None or deployment.get("max_relative_target") is None:
            raise ValueError("J5 target margin requires feedback-cap and target-rate shaping")
    if not isinstance(deployment.get("remote_execution_policy"), str) or not deployment["remote_execution_policy"].strip():
        missing.append("reviewed receiver-side remote_execution_policy path is absent")
    force = deployment.get("normal_gripper_force_n")
    if isinstance(force, bool) or not isinstance(force, (int, float)) or not math.isfinite(force) or force <= 0:
        missing.append("reviewed positive normal_gripper_force_n is absent")
    if "io_config" not in deployment:
        raise ValueError("Deployment config has no io_config path.")
    io_path = Path(deployment["io_config"]).expanduser()
    if not io_path.is_absolute() or not io_path.is_file():
        raise ValueError("Deployment io_config must be an existing absolute local path.")
    # Inspect configuration only. The receiver identity and current state are
    # verified later by the independent unarmed backend, never inferred here.
    io = read_json(io_path)
    return readiness, missing, io


@contextmanager
def official_build_injection(context_module, checked_policy, created_robots):
    """Inject only strict loading and construction tracking, restoring both hooks.

    build_rollout_context does not expose a strict loader argument and otherwise
    performs a permissive load. The already checked policy is reused exactly
    once. Tracking constructed Robots provides cleanup if context assembly
    fails after its internal connect(). No strategy/control loop is replaced.
    """
    previous_loader = context_module._load_pretrained_policy
    previous_factory = context_module.make_robot_from_config

    def strict_loader(config):
        if config is not checked_policy.config:
            raise RuntimeError("Official context requested an unchecked policy configuration.")
        return checked_policy

    def tracked_factory(config):
        robot = previous_factory(config)
        created_robots.append(robot)
        return robot

    context_module._load_pretrained_policy = strict_loader
    context_module.make_robot_from_config = tracked_factory
    try:
        yield
    finally:
        context_module._load_pretrained_policy = previous_loader
        context_module.make_robot_from_config = previous_factory


def execute_official(cfg, checked_policy, readiness, *, startup_argv=None,
                     startup_result=None, existing_mode_evidence=None,
                     expected_motion_ctrl2=(1, 1, 5, 0, 0, 0, 0, 0)):
    import lerobot.rollout.context as context_module
    from lerobot.rollout import build_rollout_context, create_strategy
    from lerobot.utils.process import ProcessSignalHandler
    from act_chunk_timing import track_act_chunks

    if startup_argv is None and existing_mode_evidence is None:
        raise RuntimeError("Execution requires explicit startup or existing receiver mode evidence.")

    signal_handler = ProcessSignalHandler(use_threads=True, display_pid=False)
    shutdown = signal_handler.shutdown_event
    created_robots = []
    ctx = None
    strategy = create_strategy(cfg.strategy)
    failed = False
    cleanup_errors = []
    try:
        with official_build_injection(context_module, checked_policy, created_robots):
            ctx = build_rollout_context(cfg, shutdown)
        robot = ctx.hardware.robot_wrapper.inner
        if robot.guard is not None and robot.guard.armed:
            raise RuntimeError("Robot unexpectedly armed during context construction.")
        # Official base setup only prepares/resets the inference engine. Keep
        # the adapter unarmed throughout setup so it cannot consume lease TTL.
        strategy.setup(ctx)
        if shutdown.is_set():
            raise RuntimeError("Shutdown requested before arming.")
        if startup_argv is not None:
            if startup_result is None:
                raise RuntimeError("An explicit startup result path is required.")
            # The RX bridge is already running and can observe this preflight's
            # single 0x151. No receiver timestamp or mode evidence is fabricated.
            run_startup_once(startup_argv, startup_result,
                             expected_motion_ctrl2=expected_motion_ctrl2)
            if shutdown.wait(1.05):
                raise RuntimeError("Shutdown requested during post-startup control quiet.")
        # This is the only arm site. It anchors to actual current feedback and
        # emits no auto-home/enable/reset command. All future targets are still
        # checked by the independent execution adapter and receiver.
        robot.backend.refresh_unarmed_clock()
        robot.arm_from_current_state(readiness)
        with track_act_chunks(checked_policy, robot, fps=cfg.fps,
                release_extension=robot.backend.config.get('policy_release_chunk_extension')):
            strategy.run(ctx)
    except BaseException:
        failed = True
        raise
    finally:
        shutdown.set()
        for robot in created_robots:
            try:
                if robot.is_connected:
                    robot.disarm("ACT rollout finished or aborted")
            except BaseException as exc:
                cleanup_errors.append(f"disarm: {exc}")
                LOG.exception("Could not invalidate execution lease")
        if ctx is not None:
            try:
                strategy.teardown(ctx)
            except BaseException as exc:
                cleanup_errors.append(f"official teardown: {exc}")
                LOG.exception("Official teardown failed")
        for robot in created_robots:
            try:
                if robot.is_connected:
                    robot.disconnect()
            except BaseException as exc:
                cleanup_errors.append(f"disconnect: {exc}")
                LOG.exception("Could not complete adapter disconnect")
        if cleanup_errors and not failed:
            raise RuntimeError("Rollout cleanup failed: " + "; ".join(cleanup_errors))


def main():
    args = arguments()
    checkpoint = args.checkpoint.expanduser().resolve()
    deployment_path = args.deployment_config.expanduser().resolve()
    # These imports register configuration/classes only. No Robot constructor,
    # camera, SSH process, backend open, or CAN operation is used in check-only.
    for directory in (HERE, DEPLOYMENT, Path('/home/aup/piper SDK/pyAgxArm')):
        sys.path.insert(0, str(directory))
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    os.environ["HF_HUB_DISABLE_TELEMETRY"] = "1"
    os.environ["OMP_NUM_THREADS"] = str(args.threads)
    os.environ["MKL_NUM_THREADS"] = str(args.threads)
    os.environ["OPENBLAS_NUM_THREADS"] = "1"

    import lerobot_robot_piper_executor.robot as executor_module
    from lerobot_robot_piper_executor.guard import Readiness, GuardLimits
    from lerobot_robot_piper_executor.robot import PiperPolicyExecutorConfig

    if tuple(executor_module.ACTION_KEYS) != CANONICAL_KEYS or tuple(executor_module.ROLLOUT_KEYS) != ROLLOUT_KEYS:
        raise RuntimeError("Executor state/action key order does not match the trained policy.")
    deployment = read_json(deployment_path)
    expected_startup_mode = startup_mode_bytes(
        deployment.get("expected_motion_ctrl2", (1, 1, 5, 0, 0, 0, 0, 0)))
    readiness, blockers, io = local_gate_status(args, deployment, Readiness, GuardLimits)
    startup_argv = None
    if args.startup_command_json is not None:
        startup_argv = read_startup_command(args.startup_command_json.expanduser().resolve())
        if args.startup_result is None:
            blockers.append("--startup-result must save the actual startup result")
    elif args.existing_mode_evidence is not None:
        read_json(args.existing_mode_evidence.expanduser().resolve())
    else:
        blockers.append("explicit --startup-command-json or --existing-mode-evidence is required")
    if args.execute and blockers:
        raise RuntimeError("Execution gates are closed: " + "; ".join(blockers))

    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies import make_pre_post_processors
    from lerobot.rollout import BaseStrategyConfig, RolloutConfig, SyncInferenceConfig
    from preview_act_offline import ensure_checkpoint_files

    if importlib.metadata.version("lerobot") != "0.6.1":
        raise RuntimeError("This launcher requires the reviewed official LeRobot 0.6.1 interface.")
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    ensure_checkpoint_files(checkpoint)
    policy_overrides = ["--device=cpu", "--use_amp=false", "--push_to_hub=false",
                        "--pretrained_backbone_weights=null"]
    if args.n_action_steps is not None:
        policy_overrides.append(f"--n_action_steps={args.n_action_steps}")
    policy_cfg = PreTrainedConfig.from_pretrained(
        checkpoint, local_files_only=True,
        cli_overrides=policy_overrides,
    )
    policy_cfg.pretrained_path = checkpoint
    if policy_cfg.type != "act" or policy_cfg.use_peft or policy_cfg.n_obs_steps != 1:
        raise ValueError("Expected a regular, single-observation ACT checkpoint.")
    expected_shapes = {"observation.state": (7,),
                       "observation.images.global": (3, 480, 640),
                       "observation.images.wrist": (3, 480, 640)}
    if {key: feature.shape for key, feature in policy_cfg.input_features.items()} != expected_shapes:
        raise ValueError("Checkpoint input features do not match the recorded PiPER state/cameras.")
    if policy_cfg.output_features["action"].shape != (7,):
        raise ValueError("Checkpoint output must contain exactly seven actions.")
    # Overriding backbone initialization is sufficient to avoid torchvision's
    # ImageNet download. Strict local loading then restores trained weights.
    checked_policy = ACTPolicy.from_pretrained(
        checkpoint, config=policy_cfg, local_files_only=True, strict=True,
    )
    checked_policy.eval().requires_grad_(False)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg, pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": "cpu"},
                                "normalizer_processor": {"device": "cpu"}},
        postprocessor_overrides={"device_processor": {"device": "cpu"},
                                 "unnormalizer_processor": {"device": "cpu"}},
    )
    from act_warmup import warm_up_act
    warmup_summary = warm_up_act(checked_policy, preprocessor, postprocessor,
                                passes=args.warmup_passes)
    from lerobot_robot_piper_executor.sdk_codec import prepare_target_codec
    codec_preparation = prepare_target_codec()
    robot_cfg = PiperPolicyExecutorConfig(id="piper_act_executor", deployment_config=deployment_path)
    rollout_cfg = RolloutConfig(
        robot=robot_cfg, policy=policy_cfg, strategy=BaseStrategyConfig(),
        inference=SyncInferenceConfig(), dataset=None, teleop=None,
        fps=20.0, duration=args.duration, device="cpu", task=TASK,
        return_to_initial_position=False, interpolation_multiplier=1,
        use_torch_compile=False, display_data=False, play_sounds=False,
    )
    plan = {
        "mode": "execute" if args.execute else "check-only",
        "checkpoint": str(checkpoint),
        "checkpoint_config_sha256": file_sha256(checkpoint / "config.json"),
        "strict_weights_loaded": True, "checkpoint_modified": False,
        "deployment_config": str(deployment_path),
        "deployment_config_sha256": file_sha256(deployment_path),
        "deployment_execute": deployment["execute"],
        "robot_type": robot_cfg.type, "robot_constructed_during_check": False,
        "robot_connection": "unarmed; no automatic mode/enable/calibration",
        "task": TASK, "duration_s": args.duration, "target_fps": 20.0,
        "device": "cpu", "use_amp": False, "threads": args.threads,
        "warmup": warmup_summary,
        "target_codec_preparation": codec_preparation,
        "chunk_size": policy_cfg.chunk_size, "n_action_steps": policy_cfg.n_action_steps,
        "temporal_ensemble_coeff": policy_cfg.temporal_ensemble_coeff,
        "chunk_timing": "original observation + action index / fps; one-use scheduled target deadline",
        "command_pacing": "at least 1/fps between successful policy command creation times; original deadlines and rate limits still apply",
        "policy_target_rate_fraction": deployment.get("policy_target_rate_fraction"),
        "policy_joint_smoothing_alpha": deployment.get("policy_joint_smoothing_alpha"),
        "policy_release_chunk_extension": deployment.get("policy_release_chunk_extension"),
        "j5_upper_prediction_tolerance_deg": deployment.get("j5_upper_prediction_tolerance_deg"),
        "j5_upper_prediction_saturation": deployment.get("j5_upper_prediction_saturation", False),
        "policy_joint_target_margin_deg": deployment.get("policy_joint_target_margin_deg"),
        "policy_gripper_target_margin_m": deployment.get("policy_gripper_target_margin_m"),
        "policy_gripper_target_bounds_m": (
            [deployment["guard_limits"]["bounds"][6][0]+deployment["policy_gripper_target_margin_m"],
             deployment["guard_limits"]["bounds"][6][1]-deployment["policy_gripper_target_margin_m"]]
            if "policy_gripper_target_margin_m" in deployment else None),
        "policy_joint_target_bounds_deg": (
            [[lo+deployment["policy_joint_target_margin_deg"], hi-deployment["policy_joint_target_margin_deg"]]
             for lo, hi in deployment["guard_limits"]["bounds"][:6]]
            if "policy_joint_target_margin_deg" in deployment else None),
        "j5_upper_target_margin_deg": deployment.get("j5_upper_target_margin_deg", 0.),
        "j5_policy_target_upper_deg": (
            min(70., deployment["guard_limits"]["bounds"][4][1])-deployment.get("j5_upper_target_margin_deg", 0.)
            if deployment.get("j5_upper_prediction_saturation") is True else None),
        "strategy": "official base", "inference": "official sync",
        "return_to_initial_position": False, "automatic_reset_home": False,
        "implicit_enable": False,
        "startup_command_json": str(args.startup_command_json.expanduser().resolve()) if args.startup_command_json else None,
        "startup_argv": startup_argv,
        "startup_result": str(args.startup_result.expanduser().resolve()) if args.startup_result else None,
        "startup_expected_motion_ctrl2": list(expected_startup_mode),
        "startup_execution": "once after unarmed RX/context/setup; require success/fresh six-axis enable/healthy hold; then 1.05s quiet",
        "existing_mode_evidence": str(args.existing_mode_evidence.expanduser().resolve()) if args.existing_mode_evidence else None,
        "feature_key_aliases": dict(zip(ROLLOUT_KEYS, CANONICAL_KEYS, strict=True)),
        "feature_values": "six joint angles in deg; gripper width in m; aliases do not rescale",
        "readiness": str(args.readiness.expanduser().resolve()) if args.readiness else None,
        "readiness_reviewed": args.readiness_reviewed,
        "execution_gates_open": not blockers, "execution_gate_blockers": blockers,
        "transport_interface": io.get("interface"),
        "limits": deployment.get("guard_limits"),
        "limitations": [
            "Existing CPU prediction measurements are about 150-170 ms per chunk; cached actions reuse that prediction. Actual control cadence must be measured.",
            "ACT action count and temporal ensemble follow the loaded checkpoint unless explicitly overridden.",
            "Offline warmup is reset before hardware connection; live predictions still use real observations and the original deadlines.",
            "Observation/command age, lease TTL, step/tracking bounds may reject slow inference or the first target; this launcher never relaxes them.",
            "Readiness evidence must cover the actual approved goal, path and all parameters in this plan; config checks do not establish physical readiness.",
            "Check-only opens no SSH, cameras, robot or CAN; it does not verify receiver deployment or physical isolation.",
        ],
    }
    print(json.dumps(plan, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    if not args.execute:
        print("CHECK ONLY: no Robot constructed, connected, armed or commanded.", flush=True)
        return 0

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s", force=True)
    execute_official(rollout_cfg, checked_policy, readiness,
                     startup_argv=startup_argv, startup_result=args.startup_result,
                     existing_mode_evidence=args.existing_mode_evidence,
                     expected_motion_ctrl2=expected_startup_mode)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ACT rollout stopped before/within guarded execution: {type(error).__name__}: {error}", file=sys.stderr)
        raise SystemExit(1)
