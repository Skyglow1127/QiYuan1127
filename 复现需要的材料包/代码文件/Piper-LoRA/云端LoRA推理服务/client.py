#!/usr/bin/env python3
"""PI05 cloud inference client and strictly read-only Piper prediction preview.

There is deliberately no execute option. A prediction is never sent to a robot.
Source timestamps remain on this computer; server time is never a freshness clock.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import io
import json
import math
from pathlib import Path
import signal
import socket
import sys
import time
import uuid

import numpy as np

PROTOCOL = "pi05-cloud-grasp/v1"
MODEL_ID = "pi05-piper-red-cube-020000"
BASE_SHA256 = "0eb11ca9587678c1d2ef8cf32807c29f8ce53a2bfdfc1aa4a4c96f16fca59b0f"
ADAPTER_SHA256 = "8e064a68c5991f1b1321e726d901968c516ec728971aa4b9eb1fc6d4213f14a1"
ACTION_KEYS = tuple(f"joint_{i}.deg" for i in range(1, 7)) + ("gripper.m",)
TASK = "Pick up the red cube and place it inside the black square on the left side of the table."
DEPLOYMENT = Path("/home/aup/Lerobot/projects/piper/deployment")
ADAPTER = Path("/home/aup/Lerobot/projects/piper/official_adapter")
DEFAULTS = Path(__file__).resolve().parent / "pi05_execution_defaults.json"
OBSERVE_CONFIG = Path(__file__).resolve().parent / "pi05_observe_config.json"


def as_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def state_vector(value):
    array = as_numpy(value)
    if array.shape == (1, 7):
        array = array[0]
    if array.shape != (7,) or not np.issubdtype(array.dtype, np.number) or not np.isfinite(array).all():
        raise ValueError("Expected seven finite state values, in degree/metre absolute units")
    return array.astype(np.float32, copy=False)


def compact_image(value):
    """Canonical LeRobot centered letterbox followed by round-to-uint8.

    This is the same explicit quantized transport used in the cloud speed test.
    There is no JPEG encoding, channel swap, normalization or interpolation change.
    """
    import torch
    from lerobot.policies.common.vla_utils import resize_with_pad_torch
    array = as_numpy(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3:
        raise ValueError("Expected one RGB image in HWC or CHW order")
    if array.shape[0] == 3:
        pass
    elif array.shape[-1] == 3:
        array = np.transpose(array, (2, 0, 1))
    else:
        raise ValueError("Expected exactly three RGB channels")
    if array.dtype == np.uint8:
        image = torch.from_numpy(np.ascontiguousarray(array)).to(torch.float32) / 255.0
    elif np.issubdtype(array.dtype, np.floating) and np.isfinite(array).all():
        if array.min() < 0 or array.max() > 1:
            raise ValueError("Floating RGB input must be unnormalized in [0,1]")
        image = torch.from_numpy(np.ascontiguousarray(array, dtype=np.float32))
    else:
        raise ValueError("RGB input must be uint8 or finite floating [0,1]")
    resized = resize_with_pad_torch(image.unsqueeze(0).permute(0, 2, 3, 1), 224, 224)
    # Clamping here is solely the bounded image quantizer, never an action clamp.
    return torch.round(resized[0].permute(2, 0, 1) * 255).clamp(0, 255).to(torch.uint8).numpy()


def encode_observation(state, image_global, image_wrist, task, request_id=None):
    """Return (NPZ body bytes, sha256 hex, preparation timing seconds)."""
    began = time.monotonic()
    request_id = str(uuid.uuid4()) if request_id is None else str(uuid.UUID(str(request_id)))
    if not isinstance(task, str) or not task.strip() or len(task) > 4096:
        raise ValueError("Expected a nonempty task string with at most 4096 characters")
    fields = {
        "protocol_version": np.asarray(PROTOCOL), "request_id": np.asarray(request_id),
        "task": np.asarray(task), "observation.state": state_vector(state),
        "observation.images.global": compact_image(image_global),
        "observation.images.wrist": compact_image(image_wrist),
    }
    transformed = time.monotonic()
    stream = io.BytesIO()
    np.savez_compressed(stream, **fields)
    payload = stream.getvalue()
    encoded = time.monotonic()
    digest = hashlib.sha256(payload).hexdigest()
    ended = time.monotonic()
    return payload, digest, {"resize_quantize": transformed-began, "npz_encode": encoded-transformed,
                             "sha256": ended-encoded, "encoding_total": ended-began}


class InferenceClient:
    """Persistent localhost-only HTTP, no retry/replay and bounded request timeout."""
    def __init__(self, host="127.0.0.1", port=18805, timeout_s=3.0):
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("Use a localhost SSH tunnel; direct public HTTP is disabled")
        if not 1 <= int(port) <= 65535 or not math.isfinite(timeout_s) or not 0 < timeout_s <= 30:
            raise ValueError("Invalid port or timeout (maximum 30 seconds)")
        self.host, self.port, self.timeout_s = host, int(port), timeout_s
        self.connection = self.health = None

    def open(self):
        if self.connection is not None:
            return self.health
        self.connection = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout_s)
        try:
            self.connection.connect()
            self.connection.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connection.request("GET", "/health", headers={"Connection": "keep-alive"})
            response = self.connection.getresponse()
            raw = response.read(1_000_001)
            if len(raw) > 1_000_000:
                raise RuntimeError("Oversized cloud health response")
            health = json.loads(raw)
            metadata = health.get("metadata", {})
            if (response.status != 200 or health.get("ready") is not True
                    or health.get("warmed_up") is not True or health.get("protocol_version") != PROTOCOL
                    or metadata.get("model_id") != MODEL_ID or metadata.get("base_sha256") != BASE_SHA256
                    or metadata.get("adapter_sha256") != ADAPTER_SHA256):
                raise RuntimeError("Cloud readiness/model identity mismatch: " + str(health))
            self.health = health
            return health
        except BaseException:
            self.close()
            raise

    def infer(self, state, image_global, image_wrist, task):
        """Return raw absolute actions[50,7], response, request identity and timings."""
        self.open()
        request_id = str(uuid.uuid4())
        began = time.monotonic()
        payload, digest, preparation = encode_observation(state, image_global, image_wrist, task, request_id)
        post_start = time.monotonic()
        try:
            self.connection.request("POST", "/infer", body=payload, headers={
                "Content-Type": "application/x-npz", "Content-Length": str(len(payload)),
                "X-Request-ID": request_id, "X-Input-SHA256": digest, "Connection": "keep-alive"})
            response = self.connection.getresponse()
            raw = response.read(2_000_001)
            received = time.monotonic()
            if len(raw) > 2_000_000:
                raise RuntimeError("Oversized cloud prediction response")
            result = json.loads(raw)
            if response.status != 200:
                raise RuntimeError("Cloud inference rejected request: " + str(result))
            if (result.get("protocol_version") != PROTOCOL or result.get("request_id") != request_id
                    or result.get("input_sha256") != digest or result.get("model_id") != MODEL_ID
                    or result.get("action_shape") != [50, 7]
                    or result.get("units") != ["degree"]*6+["metre"]
                    or result.get("representation") != "absolute joint targets and gripper width"):
                raise RuntimeError("Prediction protocol/source identity/units mismatch")
            actions = np.asarray(result["actions"], dtype=np.float32)
            if actions.shape != (50, 7) or not np.isfinite(actions).all():
                raise RuntimeError("Expected exactly finite actions[50,7]; no action clipping is performed")
            server_timing = result["server_timing_s"]
            for key in ("decode", "preprocess", "model", "postprocess", "total"):
                value = server_timing[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    raise RuntimeError("Invalid server timing")
            validated = time.monotonic()
            return {"actions": actions, "response": result, "request_id": request_id,
                    "input_sha256": digest, "request_bytes": len(payload), "response_bytes": len(raw),
                    "request_started_monotonic": began, "received_monotonic": received,
                    "validated_monotonic": validated,
                    "timing_s": {**preparation, "request": received-post_start,
                                  "model": server_timing["model"], "decode_validate": validated-received,
                                  "input_ready_to_validated_actions": validated-began}}
        except BaseException:
            self.close()  # A failed request is never retried or silently replayed.
            raise

    def close(self):
        connection, self.connection = self.connection, None
        if connection is not None:
            connection.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()


class PassiveSource:
    """Existing backend observation path, forced execute=false; no Robot constructed."""
    def __init__(self, config_path=OBSERVE_CONFIG):
        self.config_path, self.backend, self.snapshot = Path(config_path), None, None

    def open(self):
        config = json.loads(self.config_path.read_text())
        if config.get("execute") is not False:
            raise ValueError("Preview requires a configuration with explicit execute=false")
        for path in (str(ADAPTER), str(DEPLOYMENT), "/home/aup/piper SDK/pyAgxArm"):
            if path not in sys.path:
                sys.path.insert(0, path)
        from lerobot_robot_piper_executor.backend import PiperBackend
        for attempt in range(2):
            self.backend = PiperBackend.from_file(self.config_path)
            if self.backend.execution_enabled is not False:
                raise RuntimeError("Read-only backend required")
            original_request = self.backend.client.request

            def observe_reply(op, **kwargs):
                result = original_request(op, **kwargs)
                if op == "snapshot":
                    self.snapshot = result
                return result

            # Only record the actual snapshot already used by observation().
            self.backend.client.request = observe_reply
            try:
                self.backend.open_read_only()
                return self
            except Exception as error:
                self.backend.close()
                self.backend = None
                # One reconnect is limited to an EOF during this unarmed,
                # RX-only opening. Identity/clock/fault checks and observations
                # are never bypassed, and no action request can be retried here.
                if attempt or "Deployment SSH disconnected" not in str(error):
                    raise
                print("只读接收连接提前关闭，重新建立一次未武装连接；未发送动作。", flush=True)
                time.sleep(.25)

    def capture(self):
        acquisition_start = time.monotonic()
        self.snapshot = None
        observation = self.backend.observation()
        captured = time.monotonic()
        if self.snapshot is None:
            raise RuntimeError("Missing original observation snapshot metadata")
        state = np.asarray([observation.values[key] for key in ACTION_KEYS], dtype=np.float32)
        camera_metadata = {role: dict(camera.last_metadata) for role, camera in self.backend.cameras.items()}
        camera_times = [value["estimated_capture_monotonic"] for value in camera_metadata.values()]
        # This mapping remains conservative and derives from the same snapshot.
        state_at = self.backend.client.host_source_time(self.snapshot["source_ns"])
        group_ages = [float(value) for value in self.snapshot["group_ages_s"]]
        if len(group_ages) != 5 or not all(math.isfinite(age) and age >= 0 for age in group_ages):
            raise RuntimeError("Invalid feedback source group ages")
        if not np.array_equal(state, state_vector(self.snapshot["state"])):
            raise RuntimeError("Observation and its recorded feedback snapshot differ")
        sources = [observation.sampled_at, state_at, *camera_times]
        if not all(math.isfinite(value) and value <= captured for value in sources):
            raise RuntimeError("Invalid/future observation source timestamps")
        return {"kind": "live", "state": state, "global": observation.values["global"],
                "wrist": observation.values["wrist"], "task": TASK,
                "sampled_at": min(sources), "input_ready_monotonic": captured,
                "source_metadata": {"backend_sampled_at": observation.sampled_at, "state_at": state_at,
                    "camera_metadata": camera_metadata, "camera_pair_skew_s": max(camera_times)-min(camera_times),
                    "state_group_skew_s": max(group_ages)-min(group_ages),
                    "state_group_ages_at_receiver_s": group_ages, "snapshot": self.snapshot,
                    "receiver_identity": dict(self.backend.client.hello["identity"]),
                    "receiver_link": dict(self.backend.client.hello["link"]),
                    "receiver_link_metadata_scope": "read-only hello at connection; not a new link observation",
                    "acquisition_start_monotonic": acquisition_start, "input_ready_monotonic": captured}}

    def close(self):
        backend, self.backend = self.backend, None
        if backend is not None:
            backend.close()


def load_recorded(path):
    with np.load(path, allow_pickle=False) as archive:
        result = {"kind": "recorded", "state": state_vector(archive["observation.state"]),
                  "global": archive["observation.images.global"].copy(),
                  "wrist": archive["observation.images.wrist"].copy(), "task": str(archive["task"].item()),
                  "sampled_at": None, "source_metadata": {"file": str(Path(path).resolve()),
                      "capture_time_unknown": True, "freshness_not_attestable": True}}
        for key in ("episode_index", "frame_index"):
            if key in archive:
                result["source_metadata"][key] = int(archive[key])
    return result


def evaluate_prediction(observation, result, defaults=None):
    """Assess action0 against input feedback, without arming or emitting anything.

    Rate/tracking are diagnostic comparisons against the input state, which is
    explicitly not new feedback after inference. This is never an execution gate.
    """
    defaults = json.loads(DEFAULTS.read_text()) if defaults is None else defaults
    limits = defaults["policy"]["limits"]
    state, action = state_vector(observation["state"]), result["actions"][0]
    now = result["validated_monotonic"]
    reasons, acquisition_reasons = [], []
    sampled = observation.get("sampled_at")
    age = None if sampled is None else now-sampled
    if sampled is None:
        reasons.append("recorded capture timestamp unknown: freshness cannot be established")
    elif not math.isfinite(age) or age < 0:
        reasons.append("invalid/future observation source timestamp")
    else:
        if age > limits["max_observation_age_s"]:
            reasons.append("source observation older than max_observation_age_s")
        # Both deadlines derive from the original source, never receive time.
        if age > limits["max_command_age_s"]:
            reasons.append("source-derived proposal older than max_command_age_s")
    metadata = observation["source_metadata"]
    if observation["kind"] == "live":
        ready = metadata["input_ready_monotonic"]
        camera_ages = {role: ready-item["estimated_capture_monotonic"]
                       for role, item in metadata["camera_metadata"].items()}
        state_age_at_input = ready-metadata["state_at"]
        if any(not 0 <= value <= .15 for value in camera_ages.values()):
            acquisition_reasons.append("camera source age at input exceeds 150ms or is future")
        if metadata["camera_pair_skew_s"] > .06:
            acquisition_reasons.append("camera pair skew exceeds 60ms")
        if not 0 <= state_age_at_input <= defaults["policy"]["max_feedback_age_s"]:
            acquisition_reasons.append("feedback source age at input exceeds 100ms or is future")
        if metadata["state_group_skew_s"] > defaults["policy"]["max_feedback_skew_s"]:
            acquisition_reasons.append("feedback group skew exceeds 40ms")
        if metadata["snapshot"].get("healthy") is not True:
            reasons.extend("receiver health: " + str(reason) for reason in
                           (metadata["snapshot"].get("health_reasons") or ["health unknown"]))
        reasons.extend(acquisition_reasons)
    dimensions = []
    for i, key in enumerate(ACTION_KEYS):
        lo, hi = limits["bounds"][i]
        delta = float(action[i]-state[i])
        faults = []
        if not lo <= float(state[i]) <= hi:
            faults.append("input feedback outside reviewed bounds")
        if not lo <= float(action[i]) <= hi:
            faults.append("target outside reviewed bounds")
        if abs(delta) > limits["max_step"][i]:
            faults.append("target step exceeds reviewed max_step")
        if abs(delta) > limits["max_tracking_error"][i]:
            faults.append("target differs from input feedback more than max_tracking_error")
        reasons.extend(key+": "+fault for fault in faults)
        dimensions.append({"key": key, "unit": "degree" if i < 6 else "metre", "input_state": float(state[i]),
            "action0": float(action[i]), "delta_from_input_state": delta, "bounds": [lo, hi],
            "max_step": limits["max_step"][i], "max_target_speed": limits["max_target_speed"][i],
            "min_rate_limited_transition_time_s": abs(delta)/limits["max_target_speed"][i], "reasons": faults})
    return {"execute_available": False, "execution_ready": False, "robot_commands_sent": 0,
            "preview_numeric_and_source_checks_pass": not reasons, "rejection_reasons": reasons,
            "source_to_validated_action_s": age, "source_to_response_received_s": None if sampled is None else result["received_monotonic"]-sampled,
            "source_expires_at_monotonic": None if sampled is None else sampled+min(
                limits["max_observation_age_s"], limits["max_command_age_s"]),
            "acquisition_rejection_reasons": acquisition_reasons,
            "full_execution_evidence": "not assessed: approval, collision path, leases and target-rate history",
            "tracking_comparison": "input-time feedback only; this is not fresh feedback at action receipt",
            "action0": dimensions, "chunk_min": result["actions"].min(axis=0).tolist(),
            "chunk_max": result["actions"].max(axis=0).tolist(), "actions_clamped": False,
            "chunk_execution": "disabled; only action0 is displayed, no cached action is executed"}


def image_hwc_u8(value):
    array = as_numpy(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.shape[0] == 3:
        array = array.transpose(1, 2, 0)
    if array.dtype != np.uint8:
        array = np.round(array*255).clip(0, 255).astype(np.uint8)
    return np.ascontiguousarray(array)


def save_pair(observation, path):
    from PIL import Image, ImageDraw
    images = [Image.fromarray(image_hwc_u8(observation[role])) for role in ("global", "wrist")]
    height = max(image.height for image in images)
    canvas = Image.new("RGB", (sum(image.width for image in images), height+30), "white")
    draw = ImageDraw.Draw(canvas)
    x = 0
    for role, image in zip(("global", "wrist"), images, strict=True):
        draw.text((x+5, 6), role+" | "+observation["kind"]+" input | NO COMMANDS SENT", fill="black")
        canvas.paste(image, (x, 30))
        x += image.width
    canvas.save(path)


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--recorded", type=Path, help="Recorded NPZ smoke test; source freshness is always unknown")
    source.add_argument("--live", action="store_true", help="Read only current feedback and two cameras")
    parser.add_argument("--observe-config", type=Path, default=OBSERVE_CONFIG)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18805)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true", help="Recorded NPZ encoding only; no cloud or hardware")
    args = parser.parse_args()
    if not 1 <= args.count <= 100 or not 1 <= args.threads <= 32:
        parser.error("Use count 1..100 and threads 1..32")
    if args.prepare_only and args.live:
        parser.error("--prepare-only supports recorded input only")
    if args.output.exists():
        parser.error("Use a new output directory to preserve previous evidence")
    args.output.mkdir(parents=True)
    import torch
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    # Load canonical resize module before any live source timestamp exists.
    from lerobot.policies.common.vla_utils import resize_with_pad_torch  # noqa: F401
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    client, live = InferenceClient(args.host, args.port, args.timeout_s), None
    report = {"created_utc": datetime.now(timezone.utc).isoformat(), "purpose": "pi05_cloud_read_only_preview",
              "robot_commands_sent": 0, "execute_available": False, "status": "initializing", "requests": []}
    try:
        if args.prepare_only:
            observation = load_recorded(args.recorded)
            payload, digest, timing = encode_observation(observation["state"], observation["global"],
                                                         observation["wrist"], observation["task"])
            (args.output/"prepared_request.npz").write_bytes(payload)
            report.update(status="prepared_only", request_bytes=len(payload), input_sha256=digest,
                          timing_s=timing, source_metadata=observation["source_metadata"])
            save_pair(observation, args.output/"input_camera_pair.png")
        else:
            report["server_health"] = client.open()
            if args.live:
                live = PassiveSource(args.observe_config)
                live.open()
            else:
                recorded = load_recorded(args.recorded)
            for index in range(args.count):
                observation = live.capture() if live is not None else recorded
                # Capture/resize/encode/request happen before disk writes for honest age accounting.
                result = client.infer(observation["state"], observation["global"], observation["wrist"], observation["task"])
                assessment = evaluate_prediction(observation, result)
                prefix = f"request_{index:03d}"
                item = {key: value for key, value in result.items() if key != "actions"}
                item.update(source_kind=observation["kind"], source_metadata=observation["source_metadata"],
                            guard_assessment=assessment, camera_pair_image=prefix+"_camera_pair.png")
                report["requests"].append(item)
                save_pair(observation, args.output/(prefix+"_camera_pair.png"))
                (args.output/(prefix+".json")).write_text(json.dumps(item, indent=2, ensure_ascii=False,
                                                                    default=json_default, allow_nan=False)+"\n")
                print(json.dumps({"request": index, "NO_COMMANDS_SENT": True,
                    "timing_ms": {key: round(value*1000, 2) for key, value in result["timing_s"].items()},
                    "source_to_action_ms": None if assessment["source_to_validated_action_s"] is None else round(assessment["source_to_validated_action_s"]*1000, 2),
                    "action0": result["actions"][0].tolist(), "guard_rejections": assessment["rejection_reasons"]}, ensure_ascii=False), flush=True)
            report["status"] = "preview_complete"
    except BaseException as error:
        report.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                      error=type(error).__name__+": "+str(error))
        raise
    finally:
        try:
            client.close()
            if live is not None:
                live.close()
        finally:
            (args.output/"report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                                            default=json_default, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
