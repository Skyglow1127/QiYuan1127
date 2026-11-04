#!/usr/bin/env python3
"""Offline reference/API tests: synthetic images only; no devices or model weights."""
import argparse
import json
import os
from pathlib import Path
import tempfile
import unittest

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
import numpy as np
import torch

try:
    from .geometry import geometry, validate_pl
    from .pi05_adapter import (Pi05Camera, Pi05ImageAdapter, official_float_reference,
                               quality_metrics, quantized_cpu, reference_identity)
    from .make_vectors import digest, evaluate
except ImportError:
    from geometry import geometry, validate_pl
    from pi05_adapter import (Pi05Camera, Pi05ImageAdapter, official_float_reference,
                              quality_metrics, quantized_cpu, reference_identity)
    from make_vectors import digest, evaluate

torch.set_num_threads(1)
torch.set_num_interop_threads(1)


class Source:
    fps, width, height = 30, 640, 480
    is_connected = False
    last_metadata = {"index": 17, "serial": "synthetic_only"}

    def __init__(self):
        self.frame = np.random.default_rng(3).integers(0, 256, (480, 640, 3), np.uint8)
        self.calls = []

    def connect(self, warmup=True):
        self.calls.append(("connect", warmup))
        self.is_connected = True

    def disconnect(self):
        self.calls.append(("disconnect",))
        self.is_connected = False

    def read(self):
        if not self.is_connected:
            raise RuntimeError("Source not connected")
        return self.frame

    def async_read(self, timeout_ms=200):
        self.calls.append(("async_read", timeout_ms))
        return self.read()

    def read_latest(self, max_age_ms=500):
        self.calls.append(("read_latest", max_age_ms))
        return self.read()


class Pi05Tests(unittest.TestCase):
    def test_dimensions_use_truncation_and_black_centered_border(self):
        g = geometry(641, 479)
        self.assertEqual((g.rw, g.rh, g.top, g.bottom, g.pad), (224, 167, 28, 29, 0))
        self.assertEqual((geometry(33, 53).rw, geometry(33, 53).rh), (139, 224))
        self.assertEqual(geometry(640, 480).rh, 168)
        with self.assertRaises(ValueError):
            geometry(640, 480, pad=114)

    def test_compiled_limits_and_fourfold_boundary(self):
        for dims in ((16, 16, 224, 224), (896, 672, 224, 224), (1920, 1080, 640, 640)):
            validate_pl(geometry(*dims))
        for dims in ((1280, 720, 224, 224), (1920, 1080, 224, 224), (15, 16, 224, 224),
                     (640, 480, 641, 641), (1921, 1080, 640, 640)):
            with self.assertRaises(ValueError):
                validate_pl(geometry(*dims))

    def test_reference_matches_direct_torch_float_and_padding(self):
        image = np.random.default_rng(5).integers(0, 256, (479, 641, 3), np.uint8)
        g = geometry(641, 479)
        actual, _ = official_float_reference(image, g)
        tensor = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        content = torch.nn.functional.interpolate(tensor, size=(167, 224), mode="bilinear", align_corners=False)
        expected = torch.nn.functional.pad(content, (0, 0, 28, 29), value=0.0)[0].permute(1, 2, 0).numpy()
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(len(reference_identity()["function_sha256"]), 64)
        quantized, _ = quantized_cpu(image, g)
        quality = quality_metrics(quantized, actual, g)
        self.assertTrue(quality["padding_exact"])
        self.assertLessEqual(quality["max_error_byte_equivalent"], 0.5001)
        self.assertEqual(quality["model_action_regression"], "NOT_RUN")

    def test_target_shape_preserves_rgb_and_avoids_double_normalization(self):
        image = np.zeros((224, 224, 3), np.uint8)
        image[..., 0], image[..., 1], image[..., 2] = 255, 128, 0
        adapter = Pi05ImageAdapter()
        output = adapter.process(image)
        np.testing.assert_array_equal(output, image)
        self.assertFalse(adapter.last_metadata["normalization_applied"])
        normalized_once = output.astype(np.float32) / 255.0 * 2.0 - 1.0
        np.testing.assert_allclose(normalized_once[0, 0], [1.0, 128 / 255 * 2 - 1, -1.0], atol=1e-7)
        with self.assertRaises(ValueError):
            adapter.process(image.astype(np.float32) / 255)

    def test_raw_and_image_mapping_keep_original_frames_and_keys(self):
        source = Source()
        adapter = Pi05ImageAdapter(backend="raw")
        output = adapter.process_mapping({"observation.images.custom_wrist": source.frame})
        self.assertEqual(list(output), ["observation.images.custom_wrist"])
        self.assertIs(output["observation.images.custom_wrist"], source.frame)

    def test_explicit_pl_transport_and_failure_do_not_fallback(self):
        with self.assertRaises(ValueError):
            Pi05ImageAdapter(backend="pl")

        class Offline:
            def run(self, image, g):
                raise OSError("injected DMA failure")

        adapter = Pi05ImageAdapter(backend="pl", transport=Offline())
        with self.assertRaisesRegex(OSError, "DMA failure"):
            adapter.process(Source().frame)

        class Malformed:
            def run(self, image, g):
                return np.zeros((224, 224, 4), np.uint8), {}

        with self.assertRaises(RuntimeError):
            Pi05ImageAdapter(backend="pl", transport=Malformed()).process(Source().frame)

    def test_pl_receives_original_rgb_and_exact_geometry(self):
        class Spy:
            def run(self, image, g):
                self.geometry, self.image = g, image
                return np.zeros((g.height, g.width, 3), np.uint8), {"dma_ns": 123}

        transport, source = Spy(), Source()
        result = Pi05ImageAdapter(backend="pl", transport=transport).process(source.frame)
        self.assertEqual(result.shape, (224, 224, 3))
        self.assertIs(transport.image, source.frame)
        self.assertEqual(transport.geometry, geometry(640, 480))

    def test_official_camera_api_delegates_freshness_and_preserves_raw_access(self):
        from lerobot.cameras.camera import Camera

        source = Source()
        camera = Pi05Camera(source)
        self.assertIsInstance(camera, Camera)
        camera.connect(warmup=False)
        self.assertEqual((camera.width, camera.height, camera.fps), (224, 224, 30))
        self.assertEqual(camera.async_read(timeout_ms=83).shape, (224, 224, 3))
        self.assertEqual(camera.read_latest(max_age_ms=41).shape, (224, 224, 3))
        self.assertIs(camera.read_raw(), source.frame)
        self.assertIn(("async_read", 83), source.calls)
        self.assertIn(("read_latest", 41), source.calls)
        self.assertEqual(camera.last_metadata["source"]["index"], 17)
        camera.disconnect()
        self.assertFalse(source.is_connected)

    def test_wrapper_does_not_disconnect_borrowed_connection(self):
        source = Source()
        source.connect()
        camera = Pi05Camera(source, backend="raw")
        camera.connect()
        self.assertEqual((camera.width, camera.height), (640, 480))
        camera.disconnect()
        self.assertTrue(source.is_connected)
        source.disconnect()

    def test_float_quality_gate_independent_of_integer_gate(self):
        # A discrepant float reference must fail even if quantized bytes match.
        # This deliberately inconsistent fixture tests the gate, not the generator.
        with tempfile.TemporaryDirectory(prefix="vpr3_quality_") as directory:
            folder = Path(directory)
            g = geometry(16, 16, 16, 16)
            actual = np.zeros((16, 16, 3), np.uint8)
            for suffix in ("input.rgb", "expected.rgb", "actual.rgb"):
                actual.tofile(folder / f"test.{suffix}")
            reference = np.full((16, 16, 3), 10 / 255, dtype="<f4")
            reference.tofile(folder / "test.expected.float32.rgb")
            record = {"name": "test", "geometry": g.metadata(),
                      "input_sha256": digest(folder / "test.input.rgb"),
                      "expected_sha256": digest(folder / "test.expected.rgb"),
                      "float_sha256": digest(folder / "test.expected.float32.rgb")}
            manifest = {"case_count": 1, "records": [record], "official_reference": reference_identity(),
                        "cases": [["test", 16, 16, 16, 16, 16, 16, 0]]}
            (folder / "manifest.json").write_text(json.dumps(manifest))
            result = evaluate(folder)
            self.assertEqual(result["status"], "PIXEL_COMPARISON_FAIL")
            self.assertTrue(result["cases"][0]["integer_acceptance_passed"])
            self.assertFalse(result["cases"][0]["float_acceptance_passed"])
            # Matching float/byte values satisfy both gates.
            np.zeros_like(reference).tofile(folder / "test.expected.float32.rgb")
            record["float_sha256"] = digest(folder / "test.expected.float32.rgb")
            (folder / "manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(evaluate(folder)["status"], "PIXEL_COMPARISON_PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Pi05Tests))
    record = {"status": "SOFTWARE_API_PASS" if result.wasSuccessful() else "SOFTWARE_API_FAIL",
              "tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
              "reference": reference_identity(), "scope": "offline synthetic frames, CPU only, no model weights or devices",
              "model_action_regression": "NOT_RUN", "hardware_transport_test": "MOCK_ONLY"}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(record, indent=2) + "\n")
    if result.wasSuccessful():
        print(f"TEST_PASS VPR3_PI05_ADAPTER tests={result.testsRun}")
    raise SystemExit(0 if result.wasSuccessful() else 1)
