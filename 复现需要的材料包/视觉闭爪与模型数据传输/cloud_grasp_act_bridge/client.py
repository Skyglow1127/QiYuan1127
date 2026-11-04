#!/usr/bin/env python3
"""PI05 new cloud GPU inference client and strictly read-only Piper prediction preview.

There is deliberately no execute option. A prediction is never sent to a robot.
Source timestamps remain on this computer; server time is never a freshness clock.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import importlib.util
import io
import json
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time
import uuid
from functools import lru_cache

import numpy as np

PROTOCOL = "pi05-expert-inference/v1"
MODEL_SHA256 = "583ff741239b111053dc969fe5ba69eb70475e888c7ece92b04a78d3319386cf"
MODEL_BYTES = 9354050752
ACTION_KEYS = tuple(f"joint_{i}.deg" for i in range(1, 7)) + ("gripper.m",)
TASK = "Pick up the red cube and place it inside the black square on the left side of the table."
DEPLOYMENT = Path("/home/aup/Lerobot/projects/piper/deployment")
ADAPTER = Path("/home/aup/Lerobot/projects/piper/official_adapter")
DEFAULTS = Path(__file__).resolve().parent / "pi05_execution_defaults.json"
OBSERVE_CONFIG = Path(__file__).resolve().parent / "pi05_observe_config.json"
INFERENCE_CONFIG = Path(__file__).resolve().parent / "inference_config.json"
RAW_CODEC = "raw-npz-v1"
PNG_CODEC = "png-rgb-u8-v1"


@lru_cache(maxsize=2)
def wire_codec_module(filename):
    """Use the reviewed service's exact lossless encoder/decoder implementation."""
    path = Path(filename).resolve(strict=True)
    spec = importlib.util.spec_from_file_location("pi05_reviewed_wire_codec", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def service_settings():
    interface = json.loads(INFERENCE_CONFIG.read_text())
    runtime_path = Path(interface["runtime_config"])
    if not runtime_path.is_absolute() or not Path(interface["remote_config"]).is_absolute():
        raise RuntimeError("Service configuration paths must be absolute")
    payload = runtime_path.read_bytes()
    return interface, json.loads(payload), hashlib.sha256(payload).hexdigest()


def validate_health(health, status=200):
    """Validate the exact full B service without constructing network/robot IO."""
    interface, settings, config_sha256 = service_settings()
    metadata = health.get("metadata", {})
    proof = metadata.get("full_loading", {})
    if (status != 200 or health.get("ready") is not True
            or health.get("warmed_up") is not True
            or health.get("protocol_version") != PROTOCOL
            or health.get("model_sha256") != MODEL_SHA256
            or metadata.get("model_sha256") != MODEL_SHA256
            or interface.get("expected_model_sha256") != MODEL_SHA256
            or settings.get("expected_model_sha256") != MODEL_SHA256
            or metadata.get("model_format") != "full-native-PI05"
            or metadata.get("peft_present") is not False
            or metadata.get("adapter_tensors") != 0
            or metadata.get("chunk_size") != 50
            or metadata.get("num_inference_steps") != 10
            or metadata.get("batch_size") != 1
            or metadata.get("model_verification", {}).get("bytes") != MODEL_BYTES
            or metadata.get("warmup_predictions", 0) < 1
            or metadata.get("model_id") != settings["model_id"]
            or metadata.get("pipeline_files_sha256") != settings["expected_pipeline_sha256"]
            or metadata.get("tokenizer_files_sha256") != settings["expected_tokenizer_sha256"]
            or metadata.get("compile_model") != settings["compile_inference"]
            or metadata.get("compile_target") != (settings["compile_target"] if settings["compile_inference"] else None)
            or metadata.get("compile_mode") != ("reduce-overhead" if settings["compile_inference"] else None)
            or metadata.get("float32_matmul_precision") != settings["float32_matmul_precision"]
            or metadata.get("device_api") != settings["device"]
            or metadata.get("config_sha256") != config_sha256
            or metadata.get("config_path") != interface["remote_config"]
            or metadata.get("torch_hip") is not None or not metadata.get("torch_cuda")
            or proof.get("native_complete_checkpoint") is not True
            or proof.get("source_tensors") != 813 or proof.get("destination_tensors") != 813
            or proof.get("missing_keys") != [] or proof.get("unexpected_keys") != []):
        raise RuntimeError("LeRobot cloud B readiness/model/config identity mismatch")
    if "production_image_codec" in settings:
        if (settings["production_image_codec"] != PNG_CODEC
                or metadata.get("production_image_codec") != PNG_CODEC
                or metadata.get("wire_image_codecs") != [RAW_CODEC, PNG_CODEC]):
            raise RuntimeError("Lossless image transport identity mismatch")
    return health


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


def prepare_wire_observation(state, image_global, image_wrist, task, request_id=None,
                             *, image_codec=RAW_CODEC, codec_path=None):
    """Return (NPZ body bytes, sha256 hex, preparation timing seconds)."""
    began = time.monotonic()
    request_id = str(uuid.uuid4()) if request_id is None else str(uuid.UUID(str(request_id)))
    if not isinstance(task, str) or not task.strip() or len(task) > 4096:
        raise ValueError("Expected a nonempty task string with at most 4096 characters")
    fields = {
        "protocol_version": np.asarray(PROTOCOL),
        "task": np.asarray(task), "observation.state": state_vector(state),
        "observation.images.global": compact_image(image_global),
        "observation.images.wrist": compact_image(image_wrist),
    }
    transformed = time.monotonic()
    semantic_sha256 = None
    if image_codec == RAW_CODEC:
        stream = io.BytesIO()
        np.savez_compressed(stream, **fields)
        payload = stream.getvalue()
    elif image_codec == PNG_CODEC and codec_path is not None:
        codec = wire_codec_module(str(codec_path))
        semantic_sha256 = codec.decoded_fields_sha256(fields)
        payload = codec.encode_payload(fields, image_codec)
    else:
        raise ValueError("Unsupported lossless image codec or missing reviewed codec path")
    encoded = time.monotonic()
    digest = hashlib.sha256(payload).hexdigest()
    ended = time.monotonic()
    return payload, digest, {"resize_quantize": transformed-began, "npz_encode": encoded-transformed,
                             "sha256": ended-encoded, "encoding_total": ended-began}, semantic_sha256


def encode_observation(state, image_global, image_wrist, task, request_id=None):
    """Backward-compatible canonical raw NPZ encoder used by existing fixtures."""
    return prepare_wire_observation(state, image_global, image_wrist, task, request_id)[:3]


class InferenceClient:
    """Persistent localhost-only HTTP, no retry/replay and bounded request timeout."""
    def __init__(self, host="127.0.0.1", port=None, timeout_s=3.0):
        if port is None:
            port = json.loads(INFERENCE_CONFIG.read_text())["local_port"]
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("Use the dedicated localhost SSH tunnel to the new cloud model; public HTTP is disabled")
        if not 1 <= int(port) <= 65535 or not math.isfinite(timeout_s) or not 0 < timeout_s <= 30:
            raise ValueError("Invalid port or timeout (maximum 30 seconds)")
        self.host, self.port, self.timeout_s = host, int(port), timeout_s
        self.connection = self.health = None
        self.prediction_busy = threading.Event()
        self.connection_warmth = self.last_connection_warmth = None
        self.warmth_statistics = self.warmth_report_path = None
        self.warmth_settings = json.loads(INFERENCE_CONFIG.read_text()).get("connection_warmth", {})
        if self.warmth_settings.get("enabled") is True and self.warmth_settings.get("endpoint") != "/ping":
            raise ValueError("Connection warmth only supports the independent GET /ping endpoint")
        _, settings, _ = service_settings()
        self.image_codec = settings.get("production_image_codec", RAW_CODEC)
        self.codec_path = Path(json.loads(INFERENCE_CONFIG.read_text())["runtime_config"]).parent / "wire_codec.py"

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
                raise RuntimeError("Oversized local inference health response")
            health = json.loads(raw)
            self.health = validate_health(health, response.status)
            if self.warmth_settings.get("enabled") is True:
                from connection_warmth import ConnectionWarmth
                self.connection_warmth = ConnectionWarmth(
                    self.host, self.port, self.health["metadata"], self.prediction_busy,
                    interval_s=self.warmth_settings["interval_s"],
                    timeout_s=self.warmth_settings["timeout_s"]).start()
            return health
        except BaseException:
            self.close()
            raise

    def infer(self, state, image_global, image_wrist, task):
        # The separate GET-only connection keeps the SSH transport active while
        # the official FIFO is consumed. It never supplies an observation.
        self.prediction_busy.set()
        try:
            return self._infer_once(state, image_global, image_wrist, task)
        finally:
            self.prediction_busy.clear()

    def _infer_once(self, state, image_global, image_wrist, task):
        """Return raw absolute actions[50,7], response, request identity and timings."""
        self.open()
        request_id = str(uuid.uuid4())
        began = time.monotonic()
        payload, digest, preparation, semantic_sha256 = prepare_wire_observation(
            state, image_global, image_wrist, task, request_id,
            image_codec=self.image_codec, codec_path=self.codec_path)
        post_start = time.monotonic()
        try:
            self.connection.request("POST", "/infer", body=payload, headers={
                "Content-Type": "application/x-npz", "Content-Length": str(len(payload)),
                "X-Image-Codec": self.image_codec,
                "X-Request-ID": request_id, "X-Input-SHA256": digest, "Connection": "keep-alive"})
            written = time.monotonic()
            response = self.connection.getresponse()
            headers_received = time.monotonic()
            raw = response.read(2_000_001)
            received = time.monotonic()
            if len(raw) > 2_000_000:
                raise RuntimeError("Oversized local inference prediction response")
            result = json.loads(raw)
            if response.status != 200:
                raise RuntimeError("Local inference rejected request: " + str(result))
            if (result.get("protocol_version") != PROTOCOL or result.get("request_id") != request_id
                    or result.get("input_sha256") != digest or result.get("model_sha256") != MODEL_SHA256
                    or result.get("startup_id") != self.health["metadata"]["startup_id"]
                    or result.get("model_id") != self.health["metadata"]["model_id"]
                    or result.get("action_shape") != [50, 7]
                    or result.get("units") != ["degree"]*6+["metre"]
                    or result.get("representation") != "absolute joint targets and gripper width"):
                raise RuntimeError("Prediction protocol/source identity/units mismatch")
            if self.image_codec == PNG_CODEC and (
                    result.get("image_codec") != self.image_codec
                    or result.get("decoded_fields_sha256") != semantic_sha256):
                raise RuntimeError("Server decoded different image/state bytes; refusing prediction")
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
                                  "request_write": written-post_start,
                                  "wait_response_headers": headers_received-written,
                                  "response_read": received-headers_received,
                                  "server_decode": server_timing["decode"],
                                  "server_preprocess": server_timing["preprocess"],
                                  "server_postprocess": server_timing["postprocess"],
                                  "server_total": server_timing["total"],
                                  "model": server_timing["model"], "decode_validate": validated-received,
                                  "input_ready_to_validated_actions": validated-began}}
        except BaseException:
            self.close()  # A failed request is never retried or silently replayed.
            raise

    def close(self):
        warmth, self.connection_warmth = self.connection_warmth, None
        if warmth is not None:
            self.last_connection_warmth = warmth
            self.warmth_statistics = warmth.close(
                join_timeout=0.0 if self.prediction_busy.is_set() else None)
        connection, self.connection = self.connection, None
        if connection is not None:
            connection.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
        if self.last_connection_warmth is not None:
            self.warmth_statistics = self.last_connection_warmth.statistics()
        if self.warmth_report_path is not None:
            from control_logging import write_statistics
            write_statistics(self.warmth_report_path, self.warmth_statistics)


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
        from camera_recovery import VLACameraRecoveryBackend as PiperBackend
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


def warmup_recorded(inference_client):
    """Three discarded predictions before live acquisition; never queue/live evidence.

    Reuse the existing canonical real-recording NPZ of the validated service.
    Compilation can specialize on real inputs after the server's zero-input
    warmup. Only these startup predictions get a30s timeout; the existing
    connection stays open and its original live timeout is always restored.
    The ordinary infer() path validates response/model/shape as usual. There is
    no robot, source-capture, policy-selection or executable-target interface.
    """
    interface, _, _ = service_settings()
    path = Path(interface["runtime_config"]).parent / "canonical_input.npz"
    observation = load_recorded(path)
    if observation["kind"] != "recorded" or observation["task"] != TASK:
        raise ValueError("Warmup requires the existing recorded canonical task input")
    began = time.monotonic()
    original_timeout = inference_client.timeout_s
    original_connection = inference_client.connection
    original_connection_timeout = original_connection.timeout if original_connection is not None else original_timeout
    original_socket = original_connection.sock if original_connection is not None else None
    original_socket_timeout = original_socket.gettimeout() if original_socket is not None else original_timeout
    predictions = []
    try:
        inference_client.timeout_s =30.
        if original_connection is not None:
            original_connection.timeout =30.
            if original_socket is not None:
                original_socket.settimeout(30.)
        for index in range(3):
            prediction_started = time.monotonic()
            result = inference_client.infer(observation["state"], observation["global"],
                                            observation["wrist"], observation["task"])
            predictions.append({"index": index, "request_id": result["request_id"],
                                "timing_s": dict(result["timing_s"]),
                                "elapsed_s": time.monotonic()-prediction_started})
    finally:
        inference_client.timeout_s = original_timeout
        if original_connection is not None:
            original_connection.timeout = original_connection_timeout
        connection = inference_client.connection
        if connection is not None:
            connection.timeout = original_connection_timeout if connection is original_connection else original_timeout
            if connection.sock is not None:
                connection.sock.settimeout(original_socket_timeout if connection.sock is original_socket else original_timeout)
    # Return no actions and no live source/proposal timestamp. The warmup never
    # participates in evaluate_prediction(), the policy FIFO or live evidence.
    return {"purpose": "discarded_recorded_remote_warmup_before_live_acquisition",
            "source_kind": "recorded", "input_file": str(path.resolve()),
            "recorded_input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "capture_time_unknown": True, "live_evidence": False,
            "actions_discarded": True, "queued_actions": 0,
            "robot_commands_sent": 0, "prediction_count": len(predictions),
            "discarded_predictions": predictions, "startup_timeout_s":30.,
            "restored_inference_timeout_s": original_timeout,
            "warmup_elapsed_s": time.monotonic()-began}


def evaluate_prediction(observation, result, defaults=None):
    """Diagnose a new chunk's first action and hard bounds, with no actuation.

    Each raw model chunk remains unchanged. Only a hypothetical first command is
    evaluated through the four existing ACT hardware-adapter fields. Raw bounds
    are diagnostics, while actual feedback and adapted targets stay strict.
    Each request is an independent fresh chunk. Later queued actions have no
    measured future feedback, rate/commit history or collision-path evidence.
    """
    from nativequeuepolicy import QUEUE_CONTRACT, RUNTIME_ACTION_STEPS
    from newconditioning import (CONDITIONING_ID, profile_sha256, condition_first_command)
    defaults = json.loads(DEFAULTS.read_text()) if defaults is None else defaults
    limits = defaults["policy"]["limits"]
    actions = np.asarray(result["actions"])
    if actions.shape != (50, 7) or not np.isfinite(actions).all():
        raise ValueError("Expected exactly finite actions[50,7]; no repair or clipping")
    state, action = state_vector(observation["state"]), actions[0]
    gated_action, visual_assessment, visual_elapsed = action, None, 0.
    if defaults.get('vla_visual_gripper_gate') is not None:
        from visual_gripper_gate import GateState, red_geometry, shape_gripper
        visual_started = time.monotonic()
        geometry = red_geometry(observation.get('wrist'), defaults['vla_visual_gripper_gate'])
        frame_id = observation['source_metadata'].get('camera_metadata', {}).get('wrist', {}).get('index')
        gated_action, phase, visual_assessment = shape_gripper(action, state, state,
            GateState(), geometry, frame_id, defaults['vla_visual_gripper_gate'])
        visual_elapsed = time.monotonic()-visual_started
        visual_assessment = {**visual_assessment, 'geometry': geometry,
            'config_sha256': profile_sha256(defaults['vla_visual_gripper_gate']),
            'raw_prediction': action.tolist(), 'candidate': list(gated_action),
            'phase_commit_performed': False, 'fresh_phase_per_independent_preview': True,
            'processing_s': visual_elapsed}
    now = result["validated_monotonic"]+visual_elapsed
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
        # action0's slot equals the original capture time. Later slots retain
        # that same origin plus index/fps; receipt never refreshes the source.
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
        # Raw proposals are diagnostics. Actual input feedback remains strict.
        reasons.extend(key+": "+fault for fault in faults if fault == "input feedback outside reviewed bounds")
        dimensions.append({"key": key, "unit": "degree" if i < 6 else "metre", "input_state": float(state[i]),
            "action0": float(action[i]), "delta_from_input_state": delta, "bounds": [lo, hi],
            "max_step": limits["max_step"][i], "max_target_speed": limits["max_target_speed"][i],
            "min_rate_limited_transition_time_s": abs(delta)/limits["max_target_speed"][i], "reasons": faults})
    conditioning = None
    try:
        conditioning = condition_first_command(gated_action, state, limits=limits,
            profile=defaults["command_conditioning"], sampled_at=sampled,
            validated_at=now, source_kind=observation["kind"],
            rate_anchor_at=metadata.get("input_ready_monotonic") if observation["kind"] == "live" else None)
    except (ValueError, RuntimeError) as error:
        reasons.append("ACT hardware-adapted first command: "+str(error))
    first_action_reasons = list(reasons)
    violations = [{"action_index": index, "key": key, "target": float(target),
                   "bounds": list(limits["bounds"][axis])}
                  for index, targets in enumerate(actions)
                  for axis, (key, target) in enumerate(zip(ACTION_KEYS, targets, strict=True))
                  if not limits["bounds"][axis][0] <= float(target) <= limits["bounds"][axis][1]]
    prefix_violations = [item for item in violations if item["action_index"] < RUNTIME_ACTION_STEPS]
    return {"execute_available": False, "execution_ready": False, "robot_commands_sent": 0,
            "preview_numeric_and_source_checks_pass": not reasons, "rejection_reasons": reasons,
            "first_action_static_checks_pass": not first_action_reasons,
            "first_action_rejection_reasons": first_action_reasons,
            "queue_contract": QUEUE_CONTRACT,
            "preview_scope": "independent fresh chunks; existing ACT hardware-adapted first-command numerical feasibility only; raw prefix/full bounds diagnostic, no queue replay or hypothetical future feedback",
            "command_conditioning_id": CONDITIONING_ID,
            "command_conditioning_sha256": profile_sha256(defaults["command_conditioning"]),
            "first_command_conditioning": conditioning,
            "vla_visual_gripper_gate": visual_assessment,
            "raw_prediction_rejections_diagnostic_only": [key["key"]+": "+reason for key in dimensions for reason in key["reasons"] if reason != "input feedback outside reviewed bounds"],
            "raw_prefix_bounds_are_diagnostic_only": True,
            "chunk_original_source_age_at_validated_s": age,
            "first_action_slot_lateness_s": age,
            "consumed_prefix_bounds_pass": not prefix_violations,
            "consumed_prefix_bound_violations": prefix_violations,
            "full_chunk_bounds": {"all_50_within_hard_bounds": not violations, "violations": violations,
                                  "indices_to_be_consumed": list(range(RUNTIME_ACTION_STEPS)),
                                  "remaining_indices_are_diagnostic_only": list(range(RUNTIME_ACTION_STEPS, 50))},
            "future_queue_feedback_or_commit_history_verified": False,
            "fresh_feedback_after_inference_verified": False,
            "source_to_validated_action_s": age, "source_to_response_received_s": None if sampled is None else result["received_monotonic"]-sampled,
            "source_expires_at_monotonic": None if sampled is None else sampled+min(
                limits["max_observation_age_s"], limits["max_command_age_s"]),
            "acquisition_rejection_reasons": acquisition_reasons,
            "full_execution_evidence": "not assessed: approval, collision path, fresh post-inference feedback, independent receiver lease, full queued-command step/rate/tracking history",
            "tracking_comparison": "input-time feedback only; this is not fresh feedback at action receipt",
            "action0": dimensions, "chunk_min": result["actions"].min(axis=0).tolist(),
            "chunk_max": result["actions"].max(axis=0).tolist(), "actions_clamped": False, "model_outputs_modified_by_preview": False,
            "chunk_execution": "no actions emitted by preview; execution uses official FIFO for the first20 actions under explicit user-authorized runtime reduction(checkpoint saved10), then predicts a new chunk"}


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
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--interval-s", type=float, default=0.0,
                        help="Read-only idle before each subsequent capture; use1s to match runtime20 queue consumption")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true", help="Recorded NPZ encoding only; no cloud or hardware")
    args = parser.parse_args()
    if not 1 <= args.count <= 100 or not 1 <= args.threads <= 32:
        parser.error("Use count 1..100 and threads 1..32")
    if not math.isfinite(args.interval_s) or not 0 <= args.interval_s <= 5:
        parser.error("Use interval-s within0..5")
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
    from nativequeuepolicy import QUEUE_CONTRACT
    interface, settings, runtime_sha256 = service_settings()
    report = {"created_utc": datetime.now(timezone.utc).isoformat(), "purpose": "pi05_expert_newcloud_act_bridge_read_only_preview",
              "robot_commands_sent": 0, "execute_available": False, "status": "initializing", "requests": [],
              "idle_before_subsequent_capture_s": args.interval_s,
              "queue_contract": QUEUE_CONTRACT,
              "preview_scope": "independent fresh chunks; no actual queue execution, future feedback or command-history proof",
              "inference_backend": "new cloud CUDA GPU server over dedicated localhost SSH tunnel",
              "runtime_config_sha256": runtime_sha256,
              "inference_config_sha256": hashlib.sha256((Path(__file__).resolve().parent / "inference_config.json").read_bytes()).hexdigest()}
    from newconditioning import profile_sha256
    report['visual_gripper_gate_sha256'] = profile_sha256(json.loads(DEFAULTS.read_text())['vla_visual_gripper_gate'])
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
                report["recorded_remote_warmup"] = warmup_recorded(client)
                print(json.dumps({"recorded_remote_warmup": report["recorded_remote_warmup"]},
                                 ensure_ascii=False, allow_nan=False), flush=True)
                live = PassiveSource(args.observe_config)
                live.open()
            else:
                recorded = load_recorded(args.recorded)
            for index in range(args.count):
                if index and args.interval_s:
                    time.sleep(args.interval_s)
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
                    "action0": result["actions"][0].tolist(),
                    "adapted_first_target": None if assessment["first_command_conditioning"] is None else assessment["first_command_conditioning"]["quantized_target"],
                    "computed_first_wait_ms": None if assessment["first_command_conditioning"] is None else round(assessment["first_command_conditioning"]["computed_first_command_wait_s"]*1000,2),
                    "guard_rejections": assessment["rejection_reasons"]}, ensure_ascii=False), flush=True)
            report["status"] = "preview_complete"
    except BaseException as error:
        report.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                      error=type(error).__name__+": "+str(error))
        raise
    finally:
        try:
            client.close()
            report["connection_warmth"] = getattr(client, "warmth_statistics", None)
            if live is not None:
                live.close()
        finally:
            (args.output/"report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                                            default=json_default, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
