#!/usr/bin/env python3
"""Recorded-input, lossless-codec HTTP comparison. Never connects to a robot."""
from __future__ import annotations

import argparse
import hashlib
import http.client
import json
from pathlib import Path
import re
import time
import traceback
from urllib.parse import urlsplit
import uuid

from benchmark import (BEST_SHA256, PROTOCOL, REPRESENTATION, UNITS, now, output_paths,
                       summarize, validate_actions, validate_timing, write_report)
from wire_codec import (CODEC_HEADER, PNG_CODEC, RAW_CODEC, decode_payload,
                        decoded_fields_sha256, encode_payload, field_byte_hashes)


def expected_identity(args):
    settings = json.loads(args.config.read_text()) if args.config else {}
    digest = args.expected_model_sha256 or settings.get("expected_model_sha256", BEST_SHA256)
    if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("Expected exactly 64 lowercase SHA256 hex digits")
    return digest


def codec_order(codec, count):
    codecs = [RAW_CODEC, PNG_CODEC] if codec == "ab" else [{"raw": RAW_CODEC, "png": PNG_CODEC}[codec]]
    for index in range(count):
        # Alternate order to reduce monotonic time/order bias in the same service.
        for selected in (codecs if index % 2 == 0 else list(reversed(codecs))):
            yield index, selected


def benchmark(args):
    import numpy as np

    report_path, actions_path = output_paths(args.output)
    report = {"created_utc": now(), "mode": "cloud_http_lossless_comparison", "success": False,
              "robot_io": False, "protocol_version": PROTOCOL, "batch_size": 1,
              "chunk_size": 50, "flow_steps": 10, "url": args.url, "codec": args.codec,
              "idle_seconds_before_each_measured_request": args.idle_s,
              "warmup_per_codec": args.warmup, "measured_per_codec": args.repeats,
              "rows": [], "actions_file": str(actions_path)}
    connection = None
    try:
        original = args.input.read_bytes()
        fields = decode_payload(original, RAW_CODEC)
        if "noise" not in fields:
            raise ValueError("A/B timing requires canonical fixed noise")
        semantic_sha = decoded_fields_sha256(fields)
        report.update(input=str(args.input.resolve()), input_sha256=hashlib.sha256(original).hexdigest(),
                      canonical_bytes=len(original), decoded_fields_sha256=semantic_sha,
                      decoded_field_hashes=field_byte_hashes(fields), codec_roundtrip={})
        for codec in (RAW_CODEC, PNG_CODEC):
            began = time.perf_counter()
            encoded = encode_payload(fields, codec)
            encoded_at = time.perf_counter()
            restored = decode_payload(encoded, codec)
            if decoded_fields_sha256(restored) != semantic_sha:
                raise RuntimeError("Lossless codec roundtrip changed a field byte")
            report["codec_roundtrip"][codec] = {"bytes": len(encoded),
                "wire_sha256": hashlib.sha256(encoded).hexdigest(),
                "encode_seconds": encoded_at - began,
                "decode_check_seconds": time.perf_counter() - encoded_at,
                "decoded_fields_sha256": semantic_sha}
        expected = expected_identity(args)
        report["expected_model_sha256"] = expected
        url = urlsplit(args.url)
        if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password \
                or url.query or url.fragment:
            raise ValueError("Expected HTTP(S) URL without credentials/query/fragment")
        cls = http.client.HTTPSConnection if url.scheme == "https" else http.client.HTTPConnection
        connection = cls(url.hostname, url.port, timeout=args.timeout)
        prefix = url.path.rstrip("/")
        began = time.perf_counter()
        connection.connect()
        report["connection_seconds"] = time.perf_counter() - began
        connection.request("GET", prefix + "/health")
        response = connection.getresponse()
        health = json.loads(response.read())
        meta = health.get("metadata", {})
        if response.status != 200 or health.get("ready") is not True \
                or health.get("model_sha256") != expected or health.get("protocol_version") != PROTOCOL \
                or meta.get("wire_image_codecs") != [RAW_CODEC, PNG_CODEC] \
                or meta.get("float32_matmul_precision") != "highest" \
                or meta.get("compile_model") is not True \
                or meta.get("compile_target") != "denoise_step" \
                or meta.get("compile_mode") != "reduce-overhead" \
                or meta.get("num_inference_steps") != 10:
            raise RuntimeError("Service model/pipeline/codec health mismatch")
        report["server_health"] = health
        baseline_actions = None
        for phase, count in (("warmup", args.warmup), ("measured", args.repeats)):
            for index, codec in codec_order(args.codec, count):
                if phase == "measured" and args.idle_s:
                    time.sleep(args.idle_s)
                request_id = str(uuid.uuid4())
                ready = time.perf_counter()
                payload = encode_payload(fields, codec)
                encoded = time.perf_counter()
                wire_sha = hashlib.sha256(payload).hexdigest()
                request_began = time.perf_counter()
                connection.request("POST", prefix + "/infer", body=payload,
                    headers={"Content-Type": "application/x-npz", "Content-Length": str(len(payload)),
                             CODEC_HEADER: codec, "X-Request-ID": request_id, "X-Input-SHA256": wire_sha})
                written = time.perf_counter()
                response = connection.getresponse()
                headers_received = time.perf_counter()
                raw = response.read()
                received = time.perf_counter()
                result = json.loads(raw)
                if response.status != 200:
                    raise RuntimeError(f"HTTP {response.status}: {result}")
                for key, wanted in (("protocol_version", PROTOCOL), ("request_id", request_id),
                                    ("input_sha256", wire_sha), ("model_sha256", expected),
                                    ("startup_id", meta["startup_id"]), ("image_codec", codec),
                                    ("decoded_fields_sha256", semantic_sha)):
                    if result.get(key) != wanted:
                        raise RuntimeError(f"Response {key} mismatch")
                actions = validate_actions(result.get("actions"))
                if result.get("action_shape") != [50, 7] or result.get("units") != UNITS \
                        or result.get("representation") != REPRESENTATION:
                    raise ValueError("Absolute action shape/units/representation mismatch")
                if baseline_actions is None:
                    baseline_actions = np.asarray(actions, dtype=np.float64)
                error = float(np.max(np.abs(np.asarray(actions) - baseline_actions)))
                if error != 0.0:
                    raise RuntimeError(f"Fixed-noise action outputs changed: max_abs_error={error}")
                pipeline = validate_timing(result.get("server_timing_s"))
                server = result["server_timing_s"]
                validated = time.perf_counter()
                row = {"index": index, "phase": phase, "image_codec": codec,
                       "request_id": request_id, "request_bytes": len(payload), "response_bytes": len(raw),
                       "encode_seconds": encoded - ready, "input_sha_seconds": request_began - encoded,
                       "request_write_seconds": written - request_began,
                       "wait_response_headers_seconds": headers_received - written,
                       "response_body_seconds": received - headers_received,
                       "decode_validate_seconds": validated - received,
                       "roundtrip_seconds": received - request_began,
                       "input_ready_to_validated_actions_seconds": validated - ready,
                       "model_seconds": pipeline["model"], "server_pipeline_seconds": sum(pipeline.values()),
                       "server_body_read_seconds": server["body_read"],
                       "server_npz_decode_seconds": server["npz_decode"],
                       "server_image_decode_seconds": server["image_decode"],
                       "server_worker_queue_seconds": server["worker_queue"],
                       "actions_max_abs_error_from_first": error,
                       "server_result": result}
                report["rows"].append(row)
                write_report(report_path, report)
                print(json.dumps({k: v for k, v in row.items() if k != "server_result"}), flush=True)
        timing_keys = ("encode_seconds", "model_seconds", "server_body_read_seconds",
                       "server_npz_decode_seconds", "server_image_decode_seconds", "server_worker_queue_seconds",
                       "roundtrip_seconds", "input_ready_to_validated_actions_seconds")
        report["summary_seconds"] = {codec: summarize([r for r in report["rows"] if r["image_codec"] == codec],
                                                     timing_keys)
                                     for codec in sorted({r["image_codec"] for r in report["rows"]})}
        report.update(success=True, fixed_noise_action_exact_equality=True,
            timing_scope="Decoded recorded RGB/state/fixed-noise fields in memory -> lossless encode/hash -> persistent HTTP -> server decode/same model -> JSON/action/identity validation. Includes encode; excludes idle, disk loading, capture/feedback/control. Per-codec warmups excluded from summaries.",
            summary_note="Alternating A/B order on one service and connection. p95 is descriptive, not a deadline guarantee. Every decoded field and action element must exactly match the first fixed-noise request.")
        write_report(actions_path, {"model_metadata": meta, "decoded_fields_sha256": semantic_sha,
            "actions": baseline_actions.tolist(), "units": UNITS, "representation": REPRESENTATION,
            "robot_io": False})
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
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--expected-model-sha256")
    parser.add_argument("--codec", choices=("raw", "png", "ab"), default="png")
    parser.add_argument("--idle-s", type=float, default=1.0)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if not 0 <= args.warmup <= 100 or not 1 <= args.repeats <= 1000 \
            or not 0 <= args.idle_s <= 60 or args.timeout <= 0:
        parser.error("Use warmup 0..100, repeats 1..1000, idle 0..60s and positive timeout")
    benchmark(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
