#!/usr/bin/env python3
"""Read-only HTTP benchmark of the full PI05 expert policy over an existing tunnel.

The input bytes and fixed noise are shared with benchmark.py local. This program
does not create SSH connections, control cloud services or connect to a robot.
"""
from __future__ import annotations

import argparse
import http.client
import json
from pathlib import Path
import re
import time
import traceback
from urllib.parse import urlsplit
import uuid

from benchmark import (BEST_SHA256, PROTOCOL, REPRESENTATION, UNITS, load_input, now, output_paths,
                       summarize, validate_actions, validate_timing, write_report)


def expected_identity(args):
    if args.expected_model_sha256:
        digest = args.expected_model_sha256
    elif args.config:
        settings = json.loads(Path(args.config).read_text())
        digest = settings.get("expected_model_sha256")
        if not digest:
            raise ValueError("Config is missing expected_model_sha256")
    else:
        digest = BEST_SHA256
    if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("Expected model SHA256 must contain exactly 64 lowercase hex digits")
    return digest


def benchmark(args):
    report_path, actions_path = output_paths(args.output)
    report = {"created_utc": now(), "mode": "cloud_http", "success": False,
              "robot_io": False, "protocol_version": PROTOCOL,
              "batch_size": 1, "chunk_size": 50, "flow_steps": 10,
              "url": args.url, "warmup_count": args.warmup, "measured_count": args.repeats,
              "rows": [], "actions_file": str(actions_path)}
    connection = None
    try:
        _, _, payload, provenance = load_input(args.input)
        report.update(provenance)
        expected = expected_identity(args)
        report["expected_model_sha256"] = expected
        url = urlsplit(args.url)
        if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password \
                or url.query or url.fragment:
            raise ValueError("Use an HTTP(S) service URL without credentials/query/fragment")
        connection_type = http.client.HTTPSConnection if url.scheme == "https" else http.client.HTTPConnection
        connection = connection_type(url.hostname, url.port, timeout=args.timeout)
        prefix = url.path.rstrip("/")
        started = time.perf_counter()
        connection.connect()
        report["connection_seconds"] = time.perf_counter() - started
        started = time.perf_counter()
        connection.request("GET", prefix + "/health")
        response = connection.getresponse()
        raw = response.read()
        report["health_request_seconds"] = time.perf_counter() - started
        health = json.loads(raw)
        if response.status != 200 or health.get("ready") is not True \
                or health.get("protocol_version") != PROTOCOL or health.get("model_sha256") != expected:
            raise RuntimeError(f"Server health/model identity mismatch: {health}")
        report["server_health"] = health
        for index in range(args.warmup + args.repeats):
            request_id = str(uuid.uuid4())
            ready = time.perf_counter()
            connection.request("POST", prefix + "/infer", body=payload,
                headers={"Content-Type": "application/x-npz", "Content-Length": str(len(payload)),
                         "X-Request-ID": request_id, "X-Input-SHA256": provenance["input_sha256"]})
            written = time.perf_counter()
            response = connection.getresponse()
            headers_received = time.perf_counter()
            raw = response.read()
            received = time.perf_counter()
            result = json.loads(raw)
            if response.status != 200:
                raise RuntimeError(f"Inference HTTP {response.status}: {result}")
            for key, wanted in (("protocol_version", PROTOCOL), ("request_id", request_id),
                                ("input_sha256", provenance["input_sha256"]), ("model_sha256", expected)):
                if result.get(key) != wanted:
                    raise RuntimeError(f"Inference response {key} mismatch")
            actions = validate_actions(result.get("actions"))
            if result.get("action_shape") != [50, 7]:
                raise ValueError("Inference response action_shape differs")
            if result.get("units") != UNITS or result.get("representation") != REPRESENTATION:
                raise ValueError("Inference response units/absolute-action semantics differ")
            timing = validate_timing(result.get("server_timing_s", result.get("timing_s")))
            validated = time.perf_counter()
            row = {"index": index, "phase": "warmup" if index < args.warmup else "measured",
                   "first_request": index == 0, "request_id": request_id,
                   "request_bytes": len(payload), "response_bytes": len(raw),
                   "model_seconds": timing["model"], "server_pipeline_seconds": sum(timing.values()),
                   "roundtrip_seconds": received - ready,
                   "input_ready_to_validated_actions_seconds": validated - ready,
                   "request_write_seconds": written - ready,
                   "wait_response_headers_seconds": headers_received - written,
                   "response_body_seconds": received - headers_received,
                   "decode_validate_seconds": validated - received,
                   "server_result": result}
            report["rows"].append(row)
            if index == 0:
                report["first_request"] = row
            write_report(report_path, report)
            print(json.dumps({key: value for key, value in row.items() if key != "server_result"}), flush=True)
        report["summary_seconds"] = summarize(report["rows"],
            ("model_seconds", "server_pipeline_seconds", "roundtrip_seconds",
             "input_ready_to_validated_actions_seconds", "decode_validate_seconds"))
        write_report(actions_path, {"model_metadata": health.get("metadata"), "model_sha256": expected,
            "input_sha256": provenance["input_sha256"], "noise_sha256": provenance["noise_sha256"],
            "units": UNITS, "representation": REPRESENTATION,
            "actions": report["rows"][-1]["server_result"]["actions"], "robot_io": False})
        report.update(success=True,
            timing_scope="Canonical NPZ bytes already in memory -> actual HTTP upload over one persistent connection -> server decode/preprocess/model/postprocess -> response body -> client JSON decode/action/identity validation. Input preparation, disk read, model loading, camera/feedback/control are excluded.",
            summary_note="Measured rows exclude requested warmup rows. First request is recorded separately and may already use a server-warmed model. p95 is descriptive for this sample count; roundtrip is not one-way network delay.")
    except BaseException as error:
        report["error"] = {"type": type(error).__name__, "message": str(error),
                           "traceback": traceback.format_exc()}
        raise
    finally:
        if connection is not None:
            connection.close()
        write_report(report_path, report)
        print("Report:", report_path, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True,
                        help="New directory, or new JSON report path")
    parser.add_argument("--config", type=Path, help="Read expected_model_sha256 from this config")
    parser.add_argument("--expected-model-sha256", help="Default: verified B best checkpoint SHA256")
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=180)
    args = parser.parse_args()
    if not 0 <= args.warmup <= 100 or not 1 <= args.repeats <= 1000 or args.timeout <= 0:
        parser.error("Use warmup 0..100, repeats 1..1000, and a positive timeout")
    benchmark(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
