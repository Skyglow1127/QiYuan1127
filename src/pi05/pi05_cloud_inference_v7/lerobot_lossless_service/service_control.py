#!/usr/bin/env python3
"""Manage one independent cloud B service and its localhost SSH forward.

Credentials are entered only through getpass/SSH's private terminal. No keys,
password files, robot commands, instance shutdown or unrelated process signals.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import errno
import fcntl
import getpass
import hashlib
import http.client
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import signal
import socket
import stat
import subprocess
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
PROTOCOL = "pi05-expert-inference/v1"
MODEL_SHA256 = "583ff741239b111053dc969fe5ba69eb70475e888c7ece92b04a78d3319386cf"
MODEL_BYTES = 9354050752
FILES = ("server.py", "runtime.py", "pi05_low_memory.py", "service_control.py", "wire_codec.py")
SESSION = HERE / "runtime/cloud_session.json"


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def load_json(path):
    return json.loads(path.read_text()) if path.exists() else None


def read_config(path):
    config = json.loads(path.read_bytes())
    if config.get("protocol_version") != PROTOCOL or config.get("expected_model_sha256") != MODEL_SHA256 \
            or config.get("expected_model_bytes") != MODEL_BYTES:
        raise ValueError("Require the pinned complete B best-004000 model and expert protocol")
    for key in ("deployment_dir", "remote_python", "model", "tokenizer", "control_socket"):
        if not isinstance(config.get(key), str) or not Path(config[key]).is_absolute() \
                or any(char in config[key] for char in ("\0", "\n", "\r")):
            raise ValueError(f"{key} must be an absolute path without control characters")
    if config.get("remote_dir", config["deployment_dir"]) != config["deployment_dir"]:
        raise ValueError("remote_dir and deployment_dir must agree")
    for key in ("port", "local_port"):
        if type(config.get(key)) is not int or not 1024 <= config[key] <= 65535:
            raise ValueError(f"{key} must be a nonprivileged integer TCP port")
    if config["local_port"] == 18807:
        raise ValueError("Port 18807 is reserved for the existing local model service")
    ssh = config.get("ssh", {})
    if not isinstance(ssh.get("host"), str) or not ssh["host"] or not isinstance(ssh.get("user"), str) \
            or not ssh["user"] or type(ssh.get("port")) is not int or not 1 <= ssh["port"] <= 65535:
        raise ValueError("Require ssh.host/user/port")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.:-]*", ssh["host"]) \
            or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_-]*", ssh["user"]):
        raise ValueError("Invalid SSH host/user")
    if type(config.get("compile_inference", False)) is not bool:
        raise ValueError("compile_inference must be an explicit boolean")
    if config.get("supported_image_codecs") != ["raw-npz-v1", "png-rgb-u8-v1"] \
            or config.get("production_image_codec") != "png-rgb-u8-v1":
        raise ValueError("Require both reviewed lossless codecs and production PNG")
    return config


def health(port, timeout=1):
    # http.client never uses HTTP_PROXY/HTTPS_PROXY environment variables.
    connection = http.client.HTTPConnection("127.0.0.1", int(port), timeout=timeout)
    try:
        connection.request("GET", "/health")
        response = connection.getresponse()
        raw = response.read(2_000_001)
        if response.status != 200 or len(raw) > 2_000_000:
            return None
        return json.loads(raw)
    except (OSError, ValueError, http.client.HTTPException):
        return None
    finally:
        connection.close()


def validate_health(document, config, config_hash, state):
    if not document or document.get("ready") is not True or document.get("warmed_up") is not True \
            or document.get("protocol_version") != PROTOCOL or document.get("model_sha256") != MODEL_SHA256:
        raise RuntimeError("Endpoint is not the expected ready complete-model B service")
    metadata = document.get("metadata", {})
    expected = {"pid": state["pid"], "startup_id": state["startup_id"], "model_sha256": MODEL_SHA256,
        "model_id": config["model_id"], "config_sha256": config_hash,
        "config_path": str(Path(config["deployment_dir"]) / "cloud_config.json"),
        "port": config["port"], "model_format": "full-native-PI05", "peft_present": False,
        "batch_size": 1, "chunk_size": 50, "num_inference_steps": 10,
        "compile_model": config.get("compile_inference", False),
        "float32_matmul_precision": config.get("float32_matmul_precision", "high"),
        "wire_image_codecs": config["supported_image_codecs"],
        "production_image_codec": config["production_image_codec"],
        "action_shape": [50, 7], "units": ["degree"] * 6 + ["metre"],
        "representation": "absolute joint targets and gripper width"}
    if expected["compile_model"]:
        expected["compile_mode"] = "reduce-overhead"
        expected["compile_target"] = config.get("compile_target", "denoise_step")
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise RuntimeError(f"Service identity/configuration mismatch: {key}")
    loading = metadata.get("full_loading", {})
    if loading.get("native_complete_checkpoint") is not True or loading.get("source_tensors") != 813 \
            or loading.get("destination_tensors") != 813 or loading.get("missing_keys") != [] \
            or loading.get("unexpected_keys") != [] or loading.get("shape_mismatches") != {}:
        raise RuntimeError("Service did not strictly load all 813 native full-policy tensors")
    return metadata


def process_info(pid):
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        fields = raw[raw.rfind(")") + 2:].split()
        argv = [part.decode() for part in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if part]
        return {"argv": argv, "state": fields[0], "start_ticks": fields[19]}
    except (OSError, ValueError, UnicodeError, IndexError):
        return None


def expected_argv(state, config):
    return [config["remote_python"], "-u", str(Path(config["deployment_dir"]) / "server.py"),
            "--config", state["config_path"], "--startup-id", state["startup_id"]]


def pid_matches(state, config):
    if not state or type(state.get("pid")) is not int or state["pid"] <= 1:
        return False
    try:
        info = process_info(state["pid"])
        return bool(info and info["state"] != "Z" and info["argv"] == expected_argv(state, config)
                    and info["start_ticks"] == state["process_start_ticks"]
                    and str(uuid.UUID(state["startup_id"])) == state["startup_id"])
    except (KeyError, ValueError, TypeError):
        return False


def remote_state_path(config):
    return Path(config["deployment_dir"]) / "control-state.json"


@contextmanager
def remote_lock(config):
    root = Path(config["deployment_dir"])
    root.mkdir(parents=True, exist_ok=True)
    with (root / "control.lock").open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def remote_status(config, config_path):
    state = load_json(remote_state_path(config))
    active = pid_matches(state, config)
    endpoint = health(config["port"])
    result = {"active": active, "ready": False, "state": state, "health": endpoint,
              "status": "starting" if active else "unmanaged_endpoint" if endpoint else "stopped",
              "robot_io": False, "cloud_instance_powered_on": True}
    if active:
        current_hash = digest(config_path)
        try:
            if state.get("config_sha256") != current_hash or state.get("config_path") != str(config_path):
                raise RuntimeError("Remote config differs from the managed startup bytes")
            if endpoint is not None:
                validate_health(endpoint, config, current_hash, state)
                result.update(ready=True, status="ready")
        except RuntimeError as error:
            result["identity_error"] = str(error)
    return result


def remote_start(config, config_path):
    with remote_lock(config):
        current = remote_status(config, config_path)
        if current["active"]:
            if current.get("identity_error"):
                raise RuntimeError(current["identity_error"])
            return {**current, "reused_existing_process": True}
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", config["port"]))
            except OSError as error:
                raise RuntimeError("Remote port is occupied; no unmanaged process was stopped") from error
        root = Path(config["deployment_dir"])
        for name in FILES:
            if not (root / name).is_file():
                raise RuntimeError(f"Missing remote deployment file: {name}")
        state = {"startup_id": str(uuid.uuid4()), "config_path": str(config_path),
                 "config_sha256": digest(config_path), "started_utc": now(), "phase": "starting",
                 "port": config["port"], "robot_io": False}
        environment = dict(os.environ)
        for key in ("PYTHONPATH", "LD_LIBRARY_PATH"):
            environment.pop(key, None)
        environment.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
            HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_DATASETS_OFFLINE="1",
            HF_HUB_DISABLE_TELEMETRY="1", TOKENIZERS_PARALLELISM="false",
            OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="8", MKL_NUM_THREADS="8",
            TORCHINDUCTOR_COMPILE_THREADS="4", MAX_JOBS="4")
        if config.get("cuda_visible_devices"):
            environment["CUDA_VISIBLE_DEVICES"] = str(config["cuda_visible_devices"])
        with (root / "server.log").open("ab", buffering=0) as log:
            child = subprocess.Popen(expected_argv(state, config), stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, env=environment, cwd=root, start_new_session=True)
        info = process_info(child.pid)
        if info is None:
            raise RuntimeError("Server exited before identity could be recorded; inspect server.log")
        state.update(pid=child.pid, process_start_ticks=info["start_ticks"])
        atomic_json(remote_state_path(config), state)
        return {"active": True, "ready": False, "status": "starting", "state": state,
                "reused_existing_process": False, "robot_io": False, "cloud_instance_powered_on": True}


def remote_stop(config, config_path):
    with remote_lock(config):
        state = load_json(remote_state_path(config))
        if not pid_matches(state, config):
            return {"stopped": True, "already_stopped": True, "unmanaged_processes_touched": False,
                    "robot_io": False, "cloud_instance_powered_on": True}
        if state.get("config_sha256") != digest(config_path):
            raise RuntimeError("Owned startup configuration changed; refusing an ambiguous stop")
        pidfd = None
        try:
            if hasattr(os, "pidfd_open") and hasattr(signal, "pidfd_send_signal"):
                pidfd = os.pidfd_open(state["pid"])
                if not pid_matches(state, config):
                    raise RuntimeError("Process identity changed before stop")
                signal.pidfd_send_signal(pidfd, signal.SIGTERM)
            else:
                if not pid_matches(state, config):
                    raise RuntimeError("Process identity changed before stop")
                os.kill(state["pid"], signal.SIGTERM)
        except ProcessLookupError:
            pass
        finally:
            if pidfd is not None:
                os.close(pidfd)
        deadline = time.monotonic() + 10
        while pid_matches(state, config) and time.monotonic() < deadline:
            time.sleep(.2)
        if pid_matches(state, config):
            raise RuntimeError("Owned service did not stop after SIGTERM; no other process was signalled")
        state.update(phase="stopped", stopped_utc=now())
        atomic_json(remote_state_path(config), state)
        return {"stopped": True, "pid": state["pid"], "startup_id": state["startup_id"],
                "unmanaged_processes_touched": False, "robot_io": False, "cloud_instance_powered_on": True}


class SSH:
    def __init__(self, config, master=None):
        self.config, self.spec = config, config["ssh"]
        self.external = master is not None
        self.socket = Path(master or config["control_socket"]).resolve()
        if len(str(self.socket).encode()) >= 100:
            raise ValueError("SSH control socket must be shorter than 100 bytes; use a /tmp path")
        self.destination = f"{self.spec['user']}@{self.spec['host']}"
        self.endpoint = {key: self.spec[key] for key in ("host", "user", "port")}
        self.created, self.forward_added = False, False
        self.connected_identity = None
        self.base = ["ssh", "-p", str(self.spec["port"]), "-S", str(self.socket), "-T",
                     "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=15",
                     "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
                     "-o", "ExitOnForwardFailure=yes"]
        if self.spec.get("bind_address"):
            self.base += ["-b", self.spec["bind_address"]]
        if self.spec.get("host_key_alias"):
            self.base += ["-o", "HostKeyAlias=" + self.spec["host_key_alias"]]
        if self.spec.get("known_hosts_file"):
            self.base += ["-o", "UserKnownHostsFile=" + self.spec["known_hosts_file"]]

    def master_identity(self):
        result = subprocess.run(self.base + ["-O", "check", self.destination],
                                capture_output=True, text=True, timeout=20)
        if result.returncode:
            return None
        match = re.search(r"pid=(\d+)", result.stdout + result.stderr)
        info = process_info(int(match.group(1))) if match else None
        if not match or not info or info["state"] == "Z":
            raise RuntimeError("Cannot pin existing SSH master's PID/start time; no operation attempted")
        return {"pid": int(match.group(1)), "start_ticks": info["start_ticks"],
                "argv_sha256": hashlib.sha256(json.dumps(info["argv"]).encode()).hexdigest()}

    def check(self):
        return self.master_identity() is not None

    def require_master(self):
        if not self.connected_identity or self.master_identity() != self.connected_identity:
            raise RuntimeError("Authenticated SSH master disappeared or was replaced; no operation attempted")

    def connect(self):
        identity = self.master_identity()
        if identity:
            self.connected_identity = identity
            return
        if self.external:
            raise RuntimeError("The supplied external SSH master is not running; no new login attempted")
        if self.socket.exists() or self.socket.is_symlink():
            info = self.socket.lstat()
            if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
                raise RuntimeError("Refusing to replace a non-owned/non-socket control path")
            self.socket.unlink()
        self.socket.parent.mkdir(parents=True, exist_ok=True)
        command = self.base + ["-M", "-f", "-N", "-o", "ControlPersist=yes",
                               "-o", "NumberOfPasswordPrompts=1", self.destination]
        import termios
        master_fd, slave_fd = os.openpty()

        def controlling_terminal():
            os.setsid()
            fcntl.ioctl(slave_fd, termios.TIOCSCTTY, 0)

        child = subprocess.Popen(command, stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
                                 preexec_fn=controlling_terminal)
        os.close(slave_fd)
        selector = selectors.DefaultSelector(); selector.register(master_fd, selectors.EVENT_READ)
        transcript, supplied = b"", False
        deadline = time.monotonic() + 60
        try:
            while time.monotonic() < deadline:
                for _key, _mask in selector.select(timeout=.2):
                    try:
                        block = os.read(master_fd, 4096)
                    except OSError as error:
                        if error.errno != errno.EIO:
                            raise
                        block = b""
                    transcript = (transcript + block)[-8192:]
                    if b"password:" in transcript.lower() and not supplied:
                        password = getpass.getpass("新云端 SSH 密码（不会保存）: ")
                        try:
                            os.write(master_fd, password.encode() + b"\n")
                        finally:
                            del password
                        transcript = b""; supplied = True; deadline = time.monotonic() + 30
                if child.poll() is not None:
                    break
            if child.poll() is None:
                raise RuntimeError("SSH connection/authentication timed out")
            identity = self.master_identity() if child.returncode == 0 else None
            if child.returncode != 0 or not identity:
                reason = "认证失败" if b"permission denied" in transcript.lower() else \
                    "主机密钥校验失败" if b"host key verification failed" in transcript.lower() else "连接或认证失败"
                raise RuntimeError(f"SSH {reason}；核对地址、端口、已验证的 host key 和密码。密码未保存")
            self.created = True
            self.connected_identity = identity
        finally:
            selector.close(); os.close(master_fd); transcript = b""
            if child.poll() is None:
                child.terminate(); child.wait(timeout=5)

    def command(self, argv, timeout=45):
        self.require_master()
        result = subprocess.run(self.base + ["-o", "ControlMaster=no", "-o", "BatchMode=yes",
            "-o", "ProxyCommand=false", self.destination, shlex.join([str(value) for value in argv])],
            capture_output=True, text=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError("SSH command failed: " + (result.stderr.strip() or result.stdout.strip()))
        return result.stdout.strip()

    def remote(self, action):
        root = Path(self.config["deployment_dir"])
        return json.loads(self.command([self.config["remote_python"], root / "service_control.py", action,
                                       "--config", root / "cloud_config.json"]))

    def installed(self):
        program = "from pathlib import Path; import sys; print(int(Path(sys.argv[1]).is_file()))"
        return self.command([self.config["remote_python"], "-c", program,
                             Path(self.config["deployment_dir"]) / "service_control.py"]) == "1"

    def prepare(self, config_path):
        root = Path(self.config["deployment_dir"])
        program = ("import json,pathlib,sys; p=pathlib.Path(sys.argv[1]); "
            "s=json.loads(p.read_text()) if p.exists() else {}; pid=s.get('pid'); "
            "live=bool(pid and pathlib.Path(f'/proc/{pid}/cmdline').exists() "
            "and pathlib.Path(f'/proc/{pid}/cmdline').read_bytes()); "
            "sys.exit('Deployment PID is alive; explicitly stop before prepare.') if live else None")
        self.command([self.config["remote_python"], "-c", program, root / "control-state.json"])
        self.command(["mkdir", "-p", "--", root])
        for source in [HERE / name for name in FILES] + [config_path]:
            self.require_master()
            name = "cloud_config.json" if source == config_path else source.name
            command = ["scp", "-q", "-P", str(self.spec["port"]), "-o", "ControlPath=" + str(self.socket),
                "-o", "ControlMaster=no", "-o", "BatchMode=yes", "-o", "ProxyCommand=false",
                "-o", "StrictHostKeyChecking=yes"]
            if self.spec.get("host_key_alias"):
                command += ["-o", "HostKeyAlias=" + self.spec["host_key_alias"]]
            if self.spec.get("known_hosts_file"):
                command += ["-o", "UserKnownHostsFile=" + self.spec["known_hosts_file"]]
            result = subprocess.run(command + [str(source), self.destination + ":" + str(root / name)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise RuntimeError("SCP deployment failed: " + result.stderr.strip())

    def forward_spec(self):
        return f"127.0.0.1:{self.config['local_port']}:127.0.0.1:{self.config['port']}"

    def session_matches(self, session):
        return bool(session and session.get("endpoint") == self.endpoint
                    and session.get("control_socket") == str(self.socket)
                    and session.get("master_identity") == self.connected_identity
                    and session.get("forward_spec") == self.forward_spec())

    def ensure_forward(self, state, config_hash):
        endpoint = health(self.config["local_port"])
        if endpoint:
            validate_health(endpoint, self.config, config_hash, state)
            return
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", self.config["local_port"]))
            except OSError as error:
                session = load_json(SESSION)
                if self.external or self.session_matches(session):
                    # An existing external/master-owned cold tunnel is allowed,
                    # but exact endpoint/config/UUID health is still mandatory.
                    return
                raise RuntimeError("Local port is occupied; no process/tunnel was replaced") from error
        self.require_master()
        result = subprocess.run(self.base + ["-O", "forward", "-L", self.forward_spec(), self.destination],
                                capture_output=True, text=True, timeout=20)
        if result.returncode:
            raise RuntimeError("SSH forward failed: " + result.stderr.strip())
        self.forward_added = True

    def cancel_forward(self, owned=False):
        if self.forward_added or owned:
            self.require_master()
            result = subprocess.run(self.base + ["-O", "cancel", "-L", self.forward_spec(), self.destination],
                                    capture_output=True, text=True, timeout=20)
            if result.returncode:
                raise RuntimeError("Could not cancel this deployment's recorded forward: " + result.stderr.strip())
            self.forward_added = False
            return True
        return False

    def close_created_master(self):
        if self.created and self.master_identity() == self.connected_identity:
            subprocess.run(self.base + ["-O", "exit", self.destination], capture_output=True, timeout=20)


def local_action(args, config):
    ssh = SSH(config, args.master)
    created_server = False
    keep_connection = False
    try:
        ssh.connect()
        if args.action == "prepare":
            ssh.prepare(args.config)
            return {"prepared": True, "deployment_dir": config["deployment_dir"], "model_files_uploaded": False}
        installed = ssh.installed()
        if args.action == "status":
            if not installed:
                return {"ready": False, "status": "not_prepared", "robot_io": False}
            result = ssh.remote("_remote-status")
            document = health(config["local_port"])
            result["local_url"] = f"http://127.0.0.1:{config['local_port']}"
            result["tunnel_ready"] = False
            if document and result.get("state"):
                validate_health(document, config, digest(args.config), result["state"])
                result["tunnel_ready"] = True
            return result
        if args.action == "stop":
            result = ssh.remote("_remote-stop") if installed else {"stopped": True, "already_stopped": True}
            session = load_json(SESSION)
            owned_forward = ssh.session_matches(session) and session.get("forward_owned", False)
            result["tunnel_closed"] = ssh.cancel_forward(owned=owned_forward)
            result["external_master_retained"] = not ssh.created
            atomic_json(SESSION, {"stopped": True, "endpoint": ssh.endpoint, "control_socket": str(ssh.socket),
                                 "master_identity": ssh.connected_identity,
                                 "forward_spec": ssh.forward_spec(), "stopped_utc": now()})
            return result
        existing = ssh.remote("_remote-status") if installed else {"active": False}
        if not existing.get("active"):
            ssh.prepare(args.config)
        result = ssh.remote("_remote-start")
        state = result["state"]
        created_server = not result.get("reused_existing_process", False)
        config_hash = digest(args.config)
        if state.get("config_sha256") != config_hash:
            raise RuntimeError("Remote startup config SHA differs from the local deployment config")
        ssh.ensure_forward(state, config_hash)
        previous = load_json(SESSION)
        atomic_json(SESSION, {"endpoint": ssh.endpoint, "control_socket": str(ssh.socket),
            "master_identity": ssh.connected_identity,
            "master_created_by_control": ssh.created or bool(ssh.session_matches(previous) and previous.get("master_created_by_control")),
            "forward_spec": ssh.forward_spec(), "forward_owned": ssh.forward_added or bool(ssh.session_matches(previous) and previous.get("forward_owned")),
            "startup_id": state["startup_id"], "remote_state": state, "started_utc": now()})
        deadline = time.monotonic() + float(config.get("startup_timeout_seconds", 900))
        next_progress = 0.0
        while time.monotonic() < deadline:
            document = health(config["local_port"])
            if document:
                validate_health(document, config, config_hash, state)
                if digest(args.config) != config_hash:
                    raise RuntimeError("Local deployment config changed during startup")
                keep_connection = True
                return {"ready": True, "warmed_up": True, "local_url": f"http://127.0.0.1:{config['local_port']}",
                    "state": state, "health": document, "robot_io": False,
                    "reused_existing_process": result.get("reused_existing_process", False),
                    "cloud_instance_powered_on": True}
            current = ssh.remote("_remote-status")
            if not current.get("active"):
                raise RuntimeError("Cloud model exited during startup; inspect remote server.log")
            if current.get("identity_error"):
                raise RuntimeError(current["identity_error"])
            if time.monotonic() >= next_progress:
                print("Cloud model is loading/warming; waiting for exact health identity...", file=sys.stderr, flush=True)
                next_progress = time.monotonic() + 10
            time.sleep(2)
        raise RuntimeError("Cloud startup timed out; inspect remote server.log")
    except BaseException:
        if created_server:
            try:
                ssh.remote("_remote-stop")
            except Exception as error:
                print("Could not confirm cleanup of newly started owned server: " + str(error), file=sys.stderr)
        raise
    finally:
        if not keep_connection:
            try:
                ssh.cancel_forward()
            finally:
                ssh.close_created_master()


def speedtest(args, config):
    status_args = argparse.Namespace(action="status", config=args.config, master=args.master)
    current = local_action(status_args, config)
    if not current.get("ready") or not current.get("tunnel_ready"):
        raise RuntimeError("The exact new cloud service/tunnel is not ready; run start.sh first")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or HERE / "results" / ("speed_" + stamp)
    python = config.get("local_client_python", "/home/aup/Lerobot/envs/piper-cpu/bin/python")
    command = [python, "-B", str(HERE / "client.py"), "--config", str(args.config),
        "--url", f"http://127.0.0.1:{config['local_port']}", "--input", str(HERE / "canonical_input.npz"),
        "--output", str(output), "--warmup", str(args.warmup), "--repeats", str(args.repeats),
        "--codec", args.codec, "--idle-s", str(args.idle_s)]
    result = subprocess.run(command)
    if result.returncode:
        raise RuntimeError(f"Read-only speed test exited {result.returncode}; inspect its partial report")
    report_path = output if Path(output).suffix.lower() == ".json" else Path(output) / "report.json"
    report = json.loads(Path(report_path).read_text())
    return {"speedtest_passed": report.get("success") is True, "report": str(report_path),
            "summary_seconds": report.get("summary_seconds"), "robot_io": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "start", "status", "stop", "speedtest", "_remote-start", "_remote-status", "_remote-stop"))
    parser.add_argument("--config", type=Path, default=HERE / "cloud_config.json")
    parser.add_argument("--master", type=Path, help="Reuse this authenticated master; never close it or prompt for a new login")
    parser.add_argument("--output", type=Path, help="New speed-test output directory or JSON report")
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--codec", choices=("raw", "png", "ab"), default="png")
    parser.add_argument("--idle-s", type=float, default=1.0)
    args = parser.parse_args()
    if not 0 <= args.warmup <= 100 or not 1 <= args.repeats <= 1000 or not 0 <= args.idle_s <= 60:
        parser.error("Use warmup 0..100 and repeats 1..1000")
    try:
        args.config = args.config.resolve(strict=True)
        config = read_config(args.config)
        if args.action.startswith("_remote-"):
            if HERE != Path(config["deployment_dir"]).resolve() or args.config != HERE / "cloud_config.json":
                raise RuntimeError("Remote lifecycle commands require this exact independent deployment directory/config")
            function = {"_remote-start": remote_start, "_remote-status": remote_status, "_remote-stop": remote_stop}[args.action]
            result = function(config, args.config)
        elif args.action == "speedtest":
            result = speedtest(args, config)
        else:
            result = local_action(args, config)
        print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
