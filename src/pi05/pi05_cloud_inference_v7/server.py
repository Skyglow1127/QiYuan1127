#!/usr/bin/env python3
"""Localhost-only complete PI05 expert model B service; no robot access."""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import socket
import threading
import time
import traceback
import uuid
import zipfile

from runtime import CAMERAS, PROTOCOL, REPRESENTATION, UNITS, Runtime

INPUT_KEYS = {"protocol_version", "task", "observation.state", *CAMERAS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--startup-id", required=True)
    args = parser.parse_args()
    startup_id = str(uuid.UUID(args.startup_id))
    config_path = args.config.resolve(strict=True)
    config_bytes = config_path.read_bytes()
    settings = json.loads(config_bytes)
    if settings["protocol_version"] != PROTOCOL:
        raise RuntimeError("Unsupported configured full-policy protocol")
    deployment = Path(settings["deployment_dir"]).resolve()
    if Path(__file__).resolve().parent != deployment:
        raise RuntimeError("Run the service from its own configured independent deployment")
    port = settings["port"]
    if type(port) is not int or not 1024 <= port <= 65535:
        raise RuntimeError("Expected a nonprivileged localhost TCP port")
    # CUDA graph capture/replay owns thread-local state. Construction, startup
    # warmup and every subsequent prediction must share one persistent thread.
    worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pi05-inference")

    def initialize_runtime():
        runtime = Runtime(settings)
        runtime.warmup()
        return runtime, {**runtime.metadata, "inference_worker_thread_id": threading.get_ident()}

    runtime, runtime_metadata = worker.submit(initialize_runtime).result()
    if config_path.read_bytes() != config_bytes:
        raise RuntimeError("Service configuration changed during model loading/warmup")
    import numpy as np

    metadata = {**runtime_metadata, "startup_id": startup_id, "pid": os.getpid(),
                "protocol_version": PROTOCOL, "port": port,
                "config_path": str(config_path), "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
                "ready_utc": datetime.now(timezone.utc).isoformat(),
                "input_image_format": "RGB CHW uint8 [3,224,224], official resize_with_pad_torch",
                "freshness_clock": "caller local monotonic; server time is not an observation timestamp"}
    temporary = deployment / "server_metadata.tmp"
    temporary.write_text(json.dumps(metadata, indent=2) + "\n")
    temporary.replace(deployment / "server_metadata.json")
    inference_lock, log_lock = threading.Lock(), threading.Lock()
    seen_ids, seen_order = set(), deque()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.settimeout(30.0)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def reply(self, status, body):
            payload = json.dumps(body, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            keep = status == 200
            self.send_header("Connection", "keep-alive" if keep else "close")
            self.end_headers()
            self.wfile.write(payload)
            self.close_connection = not keep

        def do_GET(self):
            if self.path == "/health":
                self.reply(200, {"ready": True, "warmed_up": True, "protocol_version": PROTOCOL,
                                 "model_sha256": metadata["model_sha256"], "metadata": metadata})
            else:
                self.reply(404, {"error": "unknown endpoint"})

        def do_POST(self):
            began = time.perf_counter()
            request_id, acquired = None, False
            try:
                if self.path != "/infer":
                    self.reply(404, {"error": "unknown endpoint"})
                    return
                if self.headers.get("Content-Type", "").split(";")[0] != "application/x-npz":
                    raise ValueError("Content-Type must be application/x-npz")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 512 * 1024:
                    raise ValueError("Expected nonempty compact NPZ below 512 KiB")
                expected_hash = self.headers.get("X-Input-SHA256", "")
                if len(expected_hash) != 64 or any(char not in "0123456789abcdef" for char in expected_hash):
                    raise ValueError("Expected lowercase hexadecimal X-Input-SHA256")
                request_id = self.headers.get("X-Request-ID", "")
                if str(uuid.UUID(request_id)) != request_id:
                    raise ValueError("Expected canonical UUID X-Request-ID")
                payload = self.rfile.read(length)
                if len(payload) != length:
                    raise ValueError("Truncated NPZ request")
                digest = hashlib.sha256(payload).hexdigest()
                if digest != expected_hash:
                    raise ValueError("Input SHA256 mismatch")
                with zipfile.ZipFile(io.BytesIO(payload)) as archive:
                    names = set(archive.namelist())
                    mandatory = {name + ".npy" for name in INPUT_KEYS}
                    if names not in (mandatory, mandatory | {"noise.npy"}) \
                            or len(archive.infolist()) != len(names) \
                            or sum(item.file_size for item in archive.infolist()) > 400 * 1024:
                        raise ValueError("Unexpected NPZ fields, duplicates or decompressed size")
                with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
                    scalar = {}
                    for name in ("protocol_version", "task"):
                        value = archive[name]
                        if value.shape != () or value.dtype.kind != "U":
                            raise ValueError(f"Expected scalar Unicode string: {name}")
                        scalar[name] = str(value.item())
                    if scalar["protocol_version"] != PROTOCOL or scalar["task"] != settings["task"]:
                        raise ValueError("Protocol/task mismatch")
                    state = archive["observation.state"]
                    if state.dtype != np.float32 or state.shape != (7,) or not np.isfinite(state).all():
                        raise ValueError("State must be finite float32[7] in degree/metre units")
                    sample = {"task": scalar["task"], "observation.state": state.copy()}
                    for key in CAMERAS:
                        image = archive[key]
                        if image.dtype != np.uint8 or image.shape != (3, 224, 224):
                            raise ValueError(f"Expected RGB uint8[3,224,224]: {key}")
                        sample[key] = image.copy()
                    noise = None
                    if "noise" in archive.files:
                        noise = archive["noise"]
                        if noise.dtype != np.float32 or noise.shape != (1, 50, 32) or not np.isfinite(noise).all():
                            raise ValueError("Expected finite float32 noise[1,50,32]")
                        noise = noise.copy()
                decoded = time.perf_counter()
                acquired = inference_lock.acquire(blocking=False)
                if not acquired:
                    self.reply(429, {"error": "inference busy; stale observations are not queued",
                                     "request_id": request_id, "protocol_version": PROTOCOL})
                    return
                if request_id in seen_ids:
                    self.reply(409, {"error": "duplicate UUID", "request_id": request_id})
                    return
                seen_ids.add(request_id); seen_order.append(request_id)
                if len(seen_order) > 1024:
                    seen_ids.remove(seen_order.popleft())
                # inference_lock above rejects concurrent requests before they
                # could enter the executor; this worker never queues observers.
                actions, timing = worker.submit(runtime.predict, sample, noise=noise).result()
                timing = {**timing, "decode": decoded - began, "total": time.perf_counter() - began}
                response = {"protocol_version": PROTOCOL, "request_id": request_id,
                            "input_sha256": digest, "model_id": settings["model_id"],
                            "model_sha256": metadata["model_sha256"], "startup_id": startup_id,
                            "actions": actions, "action_shape": [50, 7], "units": UNITS,
                            "representation": REPRESENTATION, "server_timing_s": timing,
                            "robot_io": False}
                with log_lock, (deployment / "requests.jsonl").open("a") as stream:
                    stream.write(json.dumps({key: value for key, value in response.items() if key != "actions"}) + "\n")
                self.reply(200, response)
            except (ValueError, KeyError, zipfile.BadZipFile, EOFError) as error:
                self.reply(400, {"error": f"{type(error).__name__}: {error}",
                                 "request_id": request_id, "protocol_version": PROTOCOL})
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                self.close_connection = True
            except Exception as error:
                traceback.print_exc()
                print(f"Inference failed: {type(error).__name__}: {error}", flush=True)
                self.reply(500, {"error": f"{type(error).__name__}: {error}",
                                 "request_id": request_id, "protocol_version": PROTOCOL})
            finally:
                if acquired:
                    inference_lock.release()

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    print(json.dumps({"ready": True, "protocol_version": PROTOCOL, "model_sha256": metadata["model_sha256"],
                      "startup_id": startup_id, "port": port}), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        worker.shutdown(wait=True, cancel_futures=True)


if __name__ == "__main__":
    main()
