#!/usr/bin/env python3
"""Start/status/stop this local ROCm PI05 service; never touches cloud or robot I/O."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
PYTHON = Path("/home/aup/Lerobot/envs/pi05-rocm-7.14.1/bin/python")
PROTOCOL = "pi05-expert-inference/v1"
PORT = 18807
MIN_AVAILABLE = 1280 * 1024**2
STATE_PATH = HERE / "runtime/local_service_state.json"


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_state(state):
    STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    temporary = STATE_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(STATE_PATH)


def read_state():
    if not STATE_PATH.exists():
        return None
    value = json.loads(STATE_PATH.read_text())
    if not isinstance(value, dict):
        raise RuntimeError("Local service state is not a JSON object")
    return value


@contextmanager
def service_lock():
    STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with (STATE_PATH.parent / "local_service.lock").open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def read_config(path):
    path = Path(path).resolve(strict=True)
    config = json.loads(path.read_text())
    if path.parent != HERE or Path(config["deployment_dir"]).resolve() != HERE:
        raise RuntimeError("Local service config and deployment_dir must belong to this directory")
    if config.get("protocol_version") != PROTOCOL or type(config.get("port")) is not int \
            or config["port"] != PORT or config.get("device") != "cuda":
        raise RuntimeError("Require this PI05 protocol, local ROCm cuda device and port 18807")
    if type(config.get("compile_inference", False)) is not bool:
        raise RuntimeError("compile_inference must be an explicit boolean")
    expected = config.get("expected_model_sha256", "")
    if not isinstance(expected, str) or len(expected) != 64 \
            or any(char not in "0123456789abcdef" for char in expected):
        raise RuntimeError("Config must pin the complete model SHA256")
    return path, config, digest(path)


def process_info(pid):
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        fields = raw[raw.rfind(")") + 2:].split()
        argv = [value.decode() for value in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if value]
        return {"start_ticks": fields[19], "state": fields[0], "argv": argv}
    except (OSError, UnicodeError, IndexError):
        return None


def expected_argv(state):
    return [str(PYTHON), "-u", str(HERE / "server.py"), "--config", state["config_path"],
            "--startup-id", state["startup_id"]]


def pid_matches(state):
    if not state or type(state.get("pid")) is not int or state["pid"] <= 1:
        return False
    try:
        if str(uuid.UUID(state["startup_id"])) != state["startup_id"]:
            return False
        info = process_info(state["pid"])
        return bool(info and info["state"] != "Z" and info["argv"] == expected_argv(state)
                    and info["start_ticks"] == state["process_start_ticks"])
    except (ValueError, TypeError, KeyError):
        return False


def available_memory():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    raise RuntimeError("Cannot read MemAvailable; no model was started")


def health():
    connection = http.client.HTTPConnection("127.0.0.1", PORT, timeout=.2)
    try:
        connection.request("GET", "/health")
        response = connection.getresponse()
        document = json.loads(response.read())
        return document if response.status == 200 else None
    except (OSError, ValueError, http.client.HTTPException):
        return None
    finally:
        connection.close()


def validate_health(document, config, config_sha256, state):
    if not document or document.get("ready") is not True or document.get("warmed_up") is not True \
            or document.get("protocol_version") != PROTOCOL:
        raise RuntimeError("Local endpoint is not ready with the expected protocol")
    metadata = document.get("metadata", {})
    expected = {"startup_id": state["startup_id"], "pid": state["pid"], "port": PORT,
        "config_sha256": config_sha256, "config_path": state["config_path"],
        "model_sha256": config["expected_model_sha256"], "model_id": config["model_id"],
        "compile_model": config.get("compile_inference", False),
        "float32_matmul_precision": config.get("float32_matmul_precision", "high"),
        "batch_size": 1, "chunk_size": 50, "num_inference_steps": 10}
    if expected["compile_model"]:
        expected["compile_target"] = config.get("compile_target", "denoise_step")
    for name, value in expected.items():
        if metadata.get(name) != value:
            raise RuntimeError(f"Local endpoint identity/config mismatch: {name}")
    if document.get("model_sha256") != config["expected_model_sha256"]:
        raise RuntimeError("Local endpoint top-level model SHA256 differs")
    return metadata


def port_in_use():
    try:
        with socket.create_connection(("127.0.0.1", PORT), timeout=.2):
            return True
    except ConnectionRefusedError:
        return False
    except OSError as error:
        raise RuntimeError(f"Cannot establish whether local port {PORT} is free: {error}") from error


def stop_owned(state):
    """Signal only the recorded exact process; a reused PID is never signalled."""
    if not pid_matches(state):
        if state and process_info(state.get("pid")) is not None:
            raise RuntimeError("Recorded PID identity changed; refusing to signal it")
        return {"stopped": True, "already_stopped": True}
    pidfd = None
    try:
        if hasattr(os, "pidfd_open") and hasattr(signal, "pidfd_send_signal"):
            pidfd = os.pidfd_open(state["pid"])
            if not pid_matches(state):
                raise RuntimeError("Process identity changed before stop; no signal sent")
            signal.pidfd_send_signal(pidfd, signal.SIGTERM)
        else:
            if not pid_matches(state):
                raise RuntimeError("Process identity changed before stop; no signal sent")
            os.kill(state["pid"], signal.SIGTERM)
    except ProcessLookupError:
        pass
    finally:
        if pidfd is not None:
            os.close(pidfd)
    deadline = time.monotonic() + 10
    while pid_matches(state) and time.monotonic() < deadline:
        time.sleep(.2)
    if pid_matches(state):
        raise RuntimeError("Owned server did not stop after SIGTERM; no other process was signalled")
    return {"stopped": True, "already_stopped": False}


def child_environment():
    environment = dict(os.environ)
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "TORCH_ROCM_AOTRITON_ENABLE_EXPERIMENTAL"):
        environment.pop(key, None)
    environment.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
        HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_DATASETS_OFFLINE="1",
        HF_HUB_DISABLE_TELEMETRY="1", TOKENIZERS_PARALLELISM="false",
        HF_HOME="/home/aup/Lerobot/cache/pi05_rocm/huggingface",
        TORCH_HOME="/home/aup/Lerobot/cache/pi05_rocm/torch",
        OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="8", MKL_NUM_THREADS="8",
        TORCHINDUCTOR_COMPILE_THREADS="4", MAX_JOBS="4")
    return environment


def status(config_path=None):
    state = read_state()
    active = pid_matches(state)
    endpoint = health()
    result = {"active": active, "ready": False, "state": state, "robot_io": False,
              "url": f"http://127.0.0.1:{PORT}"}
    if not active:
        result["status"] = "unmanaged_endpoint" if endpoint else "stopped"
        return result
    try:
        path, config, current_hash = read_config(config_path or state["config_path"])
        if str(path) != state["config_path"] or current_hash != state["config_sha256"]:
            raise RuntimeError("Current config differs from the recorded startup config")
        metadata = validate_health(endpoint, config, current_hash, state)
        result.update(ready=True, status="ready", model_id=metadata["model_id"],
                      compile_model=metadata["compile_model"], device=metadata.get("device"))
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        result.update(status="starting_or_unhealthy", identity_error=str(error))
    return result


def start(config_path, timeout):
    path, config, config_hash = read_config(config_path or HERE / "local_config.json")
    with service_lock():
        current = status(path)
        if current["active"]:
            if current["ready"]:
                return {**current, "reused_existing_process": True}
            raise RuntimeError("An owned local server is starting or has a different configuration; "
                               "inspect status and explicitly stop it before changing configuration")
        if port_in_use():
            raise RuntimeError("Local port 18807 is occupied by an unmanaged service; nothing was stopped")
        memory = available_memory()
        if memory < MIN_AVAILABLE:
            raise RuntimeError(f"MemAvailable is below 1.25 GiB before startup: {memory} bytes")
        if not PYTHON.is_file() or not (HERE / "server.py").is_file():
            raise RuntimeError("Missing fixed ROCm Python or this deployment's server.py")
        startup_id = str(uuid.uuid4())
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        log_path = STATE_PATH.parent / f"local_server_{stamp}.log"
        state = {"startup_id": startup_id, "config_path": str(path), "config_sha256": config_hash,
                 "log": str(log_path), "started_utc": now(), "phase": "starting",
                 "port": PORT, "robot_io": False, "cloud_operations": False,
                 "minimum_mem_available_bytes": memory}
        with log_path.open("ab", buffering=0) as log:
            child = subprocess.Popen(expected_argv(state), stdin=subprocess.DEVNULL, stdout=log,
                stderr=subprocess.STDOUT, cwd=HERE, env=child_environment(), start_new_session=True)
        info = process_info(child.pid)
        if info is None:
            state.update(pid=child.pid, phase="failed", failed_utc=now(),
                         error="Server exited before process identity could be recorded")
            atomic_state(state)
            raise RuntimeError(f"Local server exited before identity could be recorded; inspect {log_path}")
        state.update(pid=child.pid, process_start_ticks=info["start_ticks"])
        atomic_state(state)
    # Release the lifecycle lock while warming, so an explicit stop can proceed.
    print(f"Starting local PI05 model; PID {child.pid}; log: {log_path}", flush=True)
    began = time.monotonic()
    next_progress = 0.0
    try:
        while time.monotonic() - began < timeout:
            poll_begin = time.monotonic()
            memory = available_memory()
            state["minimum_mem_available_bytes"] = min(state["minimum_mem_available_bytes"], memory)
            if memory < MIN_AVAILABLE:
                raise RuntimeError(f"MemAvailable fell below 1.25 GiB during startup: {memory} bytes")
            code = child.poll()
            if code is not None:
                raise RuntimeError(f"Local server exited with status {code}; inspect {log_path}")
            if not pid_matches(state):
                raise RuntimeError("New local server process identity changed")
            endpoint = health()
            if endpoint is not None:
                validate_health(endpoint, config, config_hash, state)
                if digest(path) != config_hash:
                    raise RuntimeError("Local config changed during startup")
                state.update(phase="ready", ready_utc=now(), startup_seconds=time.monotonic() - began)
                with service_lock():
                    current = read_state()
                    if not pid_matches(state) or not current or current.get("startup_id") != startup_id \
                            or current.get("phase") == "stopped":
                        raise RuntimeError("Local server was stopped or replaced during startup")
                    atomic_state(state)
                return {"active": True, "ready": True, "reused_existing_process": False,
                        "state": state, "url": f"http://127.0.0.1:{PORT}", "robot_io": False}
            elapsed = time.monotonic() - began
            if elapsed >= next_progress:
                print(f"Loading/warming local model: {elapsed:.0f}s; "
                      f"MemAvailable {memory / 1024**3:.2f} GiB; log: {log_path}", flush=True)
                next_progress = elapsed + 10
            time.sleep(max(0, .5 - (time.monotonic() - poll_begin)))
        raise TimeoutError(f"Local model was not ready within {timeout:g} seconds")
    except BaseException as error:
        state.update(phase="failed", failed_utc=now(), error=f"{type(error).__name__}: {error}")
        try:
            state["startup_cleanup"] = stop_owned(state)
        except RuntimeError as cleanup:
            state["startup_cleanup_error"] = str(cleanup)
        with service_lock():
            current = read_state()
            if current and current.get("startup_id") == startup_id:
                atomic_state(state)
        raise


def stop(config_path=None):
    with service_lock():
        state = read_state()
        if not state:
            return {"stopped": True, "already_stopped": True, "robot_io": False}
        if config_path and str(Path(config_path).resolve()) != state.get("config_path"):
            raise RuntimeError("Requested stop config differs from the owned server; nothing was stopped")
        result = stop_owned(state)
        state.update(phase="stopped", stopped_utc=now(), stop_result=result)
        atomic_state(state)
        return {**result, "state": state, "robot_io": False, "cloud_operations": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "status", "stop"))
    parser.add_argument("--config", type=Path,
                        help="Start defaults to local_config.json; status/stop use the owned config")
    parser.add_argument("--timeout", type=float, default=600, help="Startup wait, 0 < seconds <= 600")
    args = parser.parse_args()
    if not 0 < args.timeout <= 600:
        parser.error("Use a startup timeout greater than 0 and at most 600 seconds")
    try:
        result = start(args.config, args.timeout) if args.action == "start" \
            else status(args.config) if args.action == "status" else stop(args.config)
        print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError) as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
