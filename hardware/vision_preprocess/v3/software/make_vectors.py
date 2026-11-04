#!/usr/bin/env python3
"""Freeze Pi05 reference vectors once; optionally evaluate actual RTL/PL RGB files."""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
import numpy as np

try:
    from .geometry import geometry, validate_pl
    from .pi05_adapter import official_float_reference, quality_metrics, reference_identity
except ImportError:
    from geometry import geometry, validate_pl
    from pi05_adapter import official_float_reference, quality_metrics, reference_identity


INTEGER_ACCEPTANCE = {"max_channel_error": 2, "canvas_mae": 0.5, "padding_exact": True}
FLOAT_ACCEPTANCE = {"max_error_byte_equivalent": 2.5, "mae_byte_equivalent": 1.0,
                    "padding_exact": True}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _hex(path, image):
    values = image.astype(np.uint32)
    pixels = values[..., 0] | (values[..., 1] << 8) | (values[..., 2] << 16)
    with path.open("w") as stream:
        for row in pixels:
            stream.write("".join(f"{int(pixel):08x}\n" for pixel in row))


def generate(folder, samples=None, source_path=None):
    import cv2
    import torch

    torch.set_num_threads(1)
    cv2.setNumThreads(1)
    folder = Path(folder)
    if folder.exists() and any(folder.iterdir()):
        raise FileExistsError("Use a new/empty vector directory; preserve frozen reference evidence")
    folder.mkdir(parents=True, exist_ok=True)
    if samples is None:
        samples = Path(__file__).resolve().parents[2] / "delivery/kr260-vpr2-20261028-v1/samples"
    samples = Path(samples)
    random = np.random.default_rng(20261031)
    records, rows = [], []

    def add(image, width=224, height=224, origin="deterministic_synthetic"):
        image = np.ascontiguousarray(image)
        g = geometry(image.shape[1], image.shape[0], width, height)
        validate_pl(g)
        reference, _ = official_float_reference(image, g, source_path)
        quantized = np.rint(reference * np.float32(255.0)).clip(0, 255).astype(np.uint8)
        name = f"case_{len(rows):02d}"
        paths = {"input": folder / f"{name}.input.rgb",
                 "expected": folder / f"{name}.expected.rgb",
                 "float": folder / f"{name}.expected.float32.rgb"}
        image.tofile(paths["input"])
        quantized.tofile(paths["expected"])
        reference.astype("<f4", copy=False).tofile(paths["float"])
        _hex(folder / f"{name}.input.hex", image)
        _hex(folder / f"{name}.expected.hex", quantized)
        rows.append([name, g.iw, g.ih, g.rw, g.rh, g.width, g.height, g.pad])
        records.append({"name": name, "geometry": g.metadata(), "origin": origin,
                        "input_sha256": digest(paths["input"]),
                        "expected_sha256": digest(paths["expected"]),
                        "float_sha256": digest(paths["float"]),
                        "quantized_reference_error": quality_metrics(quantized, reference, g)})

    def noise(width, height):
        return random.integers(0, 256, (height, width, 3), dtype=np.uint8)

    def gradient(width, height):
        x = np.linspace(0, 255, width, dtype=np.uint8)
        y = np.linspace(0, 255, height, dtype=np.uint8)
        result = np.empty((height, width, 3), np.uint8)
        result[..., 0] = x
        result[..., 1] = y[:, None]
        result[..., 2] = ((x[None, :].astype(np.uint16) + y[:, None]) // 2).astype(np.uint8)
        return result

    def checker(width, height):
        y, x = np.indices((height, width))
        return np.stack(((x % 2) * 255, (y % 2) * 255, ((x + y) % 2) * 255), axis=2).astype(np.uint8)

    add(gradient(640, 480))
    add(noise(641, 479))  # int(479 / (641/224))=167, differs from rounded letterbox.
    add(noise(33, 25))
    add(gradient(1920, 1080), 640, 640)  # Compiled maximum; 224 would exceed 4x limit.
    add(noise(896, 672))  # Exact 4x downscale boundary.
    add(checker(16, 16))
    add(noise(642, 481))
    add(noise(640, 480), 416, 416)
    add(checker(640, 480))
    add(checker(641, 479), 320, 320)
    for role in ("global", "wrist"):
        source = samples / f"{role}.jpg"
        decoded = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if decoded is None:
            raise RuntimeError(f"Required recorded JPEG could not be decoded: {source}")
        add(cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB), origin={
            "kind": "recorded_camera_image", "path": str(source.resolve()), "sha256": digest(source)})
    add(gradient(33, 53))
    add(noise(224, 224))  # Model skips resize at the target shape.

    with (folder / "cases.txt").open("w") as stream:
        for row in rows:
            stream.write(" ".join(map(str, row)) + "\n")
    manifest = {
        "schema": "vpr3_pi05_vectors_v1", "case_count": len(rows), "cases": rows, "records": records,
        "official_reference": reference_identity(source_path), "opencv_version": cv2.__version__,
        "contract": "Pi05 0.6.1: int-truncated resize, bilinear align_corners=False, centered black border",
        "float_format": "little-endian float32 HWC RGB [0,1], before Pi05 *2-1",
        "uint8_reference": "np.rint(float_reference * float32(255)).clip(0,255).astype(uint8)",
        "hex_format": "one 32-bit RGBX word per line, R in low byte, upper byte 0",
        "actual_suffix": ".actual.rgb", "source_kind": "synthetic_and_recorded_images",
        "input_limit": [1920, 1080], "canvas_limit": [640, 640], "max_downscale_per_axis": 4,
        "integer_acceptance": INTEGER_ACCEPTANCE, "float_acceptance": FLOAT_ACCEPTANCE,
        "float_error_policy": "Both integer and official float pixel gates are required; no model action equivalence claim",
        "model_action_regression": "NOT_RUN",
    }
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"TEST_PASS VPR3_PI05_REFERENCE cases={len(rows)} folder={folder}")
    return manifest


def evaluate(folder, actual_folder=None):
    folder = Path(folder)
    actual_folder = Path(actual_folder) if actual_folder else folder
    manifest = json.loads((folder / "manifest.json").read_text())
    names = [record["name"] for record in manifest["records"]]
    if (len(names) != manifest["case_count"] or len(set(names)) != len(names) or
            len(manifest["cases"]) != len(names)):
        raise RuntimeError("Incomplete or duplicate frozen reference matrix")
    results = []
    for record, row in zip(manifest["records"], manifest["cases"], strict=True):
        name = record["name"]
        dimensions = record["geometry"]
        g = geometry(dimensions["iw"], dimensions["ih"], dimensions["width"], dimensions["height"])
        if dimensions != g.metadata() or row != [name, g.iw, g.ih, g.rw, g.rh, g.width, g.height, g.pad]:
            raise RuntimeError(f"Frozen geometry/manifest disagreement: {name}")
        for suffix, sha_key in (("input.rgb", "input_sha256"), ("expected.rgb", "expected_sha256"),
                                ("expected.float32.rgb", "float_sha256")):
            if digest(folder / f"{name}.{suffix}") != record[sha_key]:
                raise RuntimeError(f"Frozen reference hash mismatch: {name}.{suffix}")
        shape = (g.height, g.width, 3)
        actual = np.fromfile(actual_folder / f"{name}.actual.rgb", np.uint8).reshape(shape)
        expected = np.fromfile(folder / f"{name}.expected.rgb", np.uint8).reshape(shape)
        reference = np.fromfile(folder / f"{name}.expected.float32.rgb", "<f4").reshape(shape)
        delta = np.abs(actual.astype(np.int16) - expected.astype(np.int16))
        metrics = quality_metrics(actual, reference, g)
        integer_pass = bool(delta.max() <= INTEGER_ACCEPTANCE["max_channel_error"] and
                            delta.mean() <= INTEGER_ACCEPTANCE["canvas_mae"] and metrics["padding_exact"])
        float_pass = bool(metrics["max_error_byte_equivalent"] <= FLOAT_ACCEPTANCE["max_error_byte_equivalent"] and
                          metrics["mae_byte_equivalent"] <= FLOAT_ACCEPTANCE["mae_byte_equivalent"] and
                          metrics["padding_exact"])
        results.append({"name": name, "actual_sha256": digest(actual_folder / f"{name}.actual.rgb"),
                        "quantized_max_error": int(delta.max()), "quantized_mae": float(delta.mean()),
                        "integer_acceptance_passed": integer_pass, "float_acceptance_passed": float_pass,
                        "official_float_error": metrics})
    passed = (len(results) == manifest["case_count"] and
              all(r["integer_acceptance_passed"] and r["float_acceptance_passed"] for r in results))
    return {"status": "PIXEL_COMPARISON_PASS" if passed else "PIXEL_COMPARISON_FAIL",
            "cases": results, "case_count": len(results), "model_action_regression": "NOT_RUN",
            "integer_acceptance": INTEGER_ACCEPTANCE, "float_acceptance": FLOAT_ACCEPTANCE,
            "reference_manifest": str((folder / "manifest.json").resolve()),
            "official_reference": manifest["official_reference"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    parser.add_argument("--samples", type=Path)
    parser.add_argument("--reference-source", type=Path)
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--actual-folder", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.evaluate:
        result = evaluate(args.folder, args.actual_folder)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if result["status"] == "PIXEL_COMPARISON_PASS" else 1)
    generate(args.folder, args.samples, args.reference_source)
