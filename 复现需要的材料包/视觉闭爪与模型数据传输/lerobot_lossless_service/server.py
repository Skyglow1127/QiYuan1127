#!/usr/bin/env python3
"""Localhost-only complete PI05 expert model B service; no robot access."""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
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
from wire_codec import (CODEC_HEADER, RAW_CODEC, SUPPORTED_CODECS, decode_payload_profiled,
                        decoded_fields_sha256)

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
    if settings.get("supported_image_codecs") != list(SUPPORTED_CODECS) \
            or settings.get("production_image_codec") not in SUPPORTED_CODECS:
        raise RuntimeError("Configured image codecs differ from this service")
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
    metadata = {**runtime_metadata, "startup_id": startup_id, "pid": os.getpid(),
                "protocol_version": PROTOCOL, "port": port,
                "config_path": str(config_path), "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
                "ready_utc": datetime.now(timezone.utc).isoformat(),
                "input_image_format": "RGB CHW uint8 [3,224,224], official resize_with_pad_torch",
                "wire_image_codecs": list(SUPPORTED_CODECS),
                "production_image_codec": settings["production_image_codec"],
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
            elif self.path == "/ping":
                self.reply(200, {"ready": True, "startup_id": startup_id,
                                 "model_id": settings["model_id"], "wire_image_codecs": list(SUPPORTED_CODECS)})
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
                codec = self.headers.get(CODEC_HEADER, RAW_CODEC)
                if codec not in SUPPORTED_CODECS:
                    raise ValueError("Unsupported X-Image-Codec")
                body_began = time.perf_counter()
                payload = self.rfile.read(length)
                body_end = time.perf_counter()
                if len(payload) != length:
                    raise ValueError("Truncated NPZ request")
                digest = hashlib.sha256(payload).hexdigest()
                sha_end = time.perf_counter()
                if digest != expected_hash:
                    raise ValueError("Input SHA256 mismatch")
                fields, decode_timing = decode_payload_profiled(payload, codec)
                if str(fields["protocol_version"].item()) != PROTOCOL \
                        or str(fields["task"].item()) != settings["task"]:
                    raise ValueError("Protocol/task mismatch")
                sample = {"task": str(fields["task"].item()),
                          "observation.state": fields["observation.state"],
                          **{key: fields[key] for key in CAMERAS}}
                noise = fields.get("noise")
                semantic_began = time.perf_counter()
                semantic_digest = decoded_fields_sha256(fields)
                semantic_end = time.perf_counter()
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
                submitted = time.perf_counter()

                def predict_timed():
                    worker_started = time.perf_counter()
                    predicted_actions, pipeline_timing = runtime.predict(sample, noise=noise)
                    return predicted_actions, {**pipeline_timing,
                        "worker_queue": worker_started - submitted,
                        "worker_execution": time.perf_counter() - worker_started}

                actions, timing = worker.submit(predict_timed).result()
                timing = {**timing, **decode_timing, "header_validate": body_began - began,
                          "body_read": body_end - body_began, "input_sha": sha_end - body_end,
                          "decoded_sha": semantic_end - semantic_began,
                          "decode": decoded - began, "dispatch": submitted - decoded,
                          "total": time.perf_counter() - began}
                response = {"protocol_version": PROTOCOL, "request_id": request_id,
                            "input_sha256": digest, "image_codec": codec,
                            "decoded_fields_sha256": semantic_digest, "model_id": settings["model_id"],
                            "model_sha256": metadata["model_sha256"], "startup_id": startup_id,
                            "actions": actions, "action_shape": [50, 7], "units": UNITS,
                            "representation": REPRESENTATION, "server_timing_s": timing,
                            "robot_io": False}
                log_began = time.perf_counter()
                with log_lock, (deployment / "requests.jsonl").open("a") as stream:
                    stream.write(json.dumps({key: value for key, value in response.items() if key != "actions"}) + "\n")
                timing["request_log"] = time.perf_counter() - log_began
                timing["handler_to_reply"] = time.perf_counter() - began
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
