"""Offline exact-pixel and malformed wire-boundary regressions; no robot/GPU."""
import io
import copy
import json
from pathlib import Path
import unittest
import warnings
import zipfile
from unittest.mock import patch

import numpy as np
from PIL import Image

from wire_codec import (IMAGE_KEYS, MAX_PAYLOAD_BYTES, PNG_CODEC, RAW_CODEC,
                        decode_payload, decode_payload_profiled, decoded_fields_sha256,
                        encode_payload)


class WireCodecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fields = decode_payload(Path(__file__).with_name("canonical_input.npz").read_bytes(), RAW_CODEC)

    def fresh(self):
        return {key: value.copy() for key, value in self.fields.items()}

    def pack(self, fields):
        stream = io.BytesIO()
        np.savez_compressed(stream, **fields)
        return stream.getvalue()

    def assert_same(self, original, decoded):
        self.assertEqual(set(original), set(decoded))
        for key in original:
            self.assertEqual(original[key].dtype, decoded[key].dtype, key)
            self.assertEqual(original[key].shape, decoded[key].shape, key)
            self.assertEqual(original[key].tobytes(order="C"), decoded[key].tobytes(order="C"), key)
        self.assertEqual(decoded_fields_sha256(original), decoded_fields_sha256(decoded))

    def test_recorded_pixels_state_and_fixed_noise_exact(self):
        for codec in (RAW_CODEC, PNG_CODEC):
            with self.subTest(codec=codec):
                self.assert_same(self.fields, decode_payload(encode_payload(self.fields, codec), codec))

    def test_channel_order_and_noncontiguous_input(self):
        fields = self.fresh()
        image = np.empty((224, 224, 3), dtype=np.uint8)
        image[..., 0] = 233
        image[..., 1] = np.arange(224, dtype=np.uint8)[None, :]
        image[..., 2] = np.arange(224, dtype=np.uint8)[:, None]
        for key in IMAGE_KEYS:
            fields[key] = image.transpose(2, 0, 1)[:, :, ::-1]
        self.assertFalse(fields[IMAGE_KEYS[0]].flags.c_contiguous)
        self.assert_same(fields, decode_payload(encode_payload(fields, PNG_CODEC), PNG_CODEC))

    def test_optional_noise_absent(self):
        fields = self.fresh()
        fields.pop("noise")
        for codec in (RAW_CODEC, PNG_CODEC):
            self.assert_same(fields, decode_payload(encode_payload(fields, codec), codec))

    def test_unknown_codec_and_wrong_header_rejected(self):
        for codec in ("jpeg", "", None):
            with self.assertRaises(ValueError):
                encode_payload(self.fields, codec)
            with self.assertRaises(ValueError):
                decode_payload(b"anything", codec)
        for actual, header in ((RAW_CODEC, PNG_CODEC), (PNG_CODEC, RAW_CODEC)):
            with self.assertRaises(ValueError):
                decode_payload(encode_payload(self.fields, actual), header)

    def test_duplicate_and_extra_zip_fields_rejected(self):
        payload = encode_payload(self.fields, RAW_CODEC)
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            data = {key: archive.read(key) for key in archive.namelist()}
        stream = io.BytesIO()
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                for key, value in data.items():
                    archive.writestr(key, value)
                archive.writestr("task.npy", data["task.npy"])
        with self.assertRaises(ValueError):
            decode_payload(stream.getvalue(), RAW_CODEC)
        fields = self.fresh()
        fields["extra"] = np.zeros(1)
        with self.assertRaises(ValueError):
            decode_payload(self.pack(fields), RAW_CODEC)

    def test_compact_decompressed_bound(self):
        fields = self.fresh()
        fields[IMAGE_KEYS[0]] = np.zeros(500 * 1024, dtype=np.uint8)
        with self.assertRaises(ValueError):
            decode_payload(self.pack(fields), PNG_CODEC)
        for payload in (b"", b"x" * (MAX_PAYLOAD_BYTES + 1)):
            with self.assertRaises(ValueError):
                decode_payload(payload, RAW_CODEC)

    def test_bad_dtype_nonfinite_and_image_shape_rejected(self):
        for key, invalid in (("noise", np.full((1, 50, 32), np.nan, np.float32)),
                             ("observation.state", np.zeros(7, np.float64)),
                             (IMAGE_KEYS[0], np.zeros((224, 224, 3), np.uint8))):
            fields = self.fresh()
            fields[key] = invalid
            with self.assertRaises(ValueError):
                encode_payload(fields, PNG_CODEC)
            with self.assertRaises(ValueError):
                decode_payload(self.pack(fields), RAW_CODEC)

    def test_png_format_dimensions_mode_and_corruption_rejected(self):
        with np.load(io.BytesIO(encode_payload(self.fields, PNG_CODEC)), allow_pickle=False) as archive:
            wire = {key: archive[key].copy() for key in archive.files}
        samples = []
        for format_, mode, size in (("JPEG", "RGB", (224, 224)), ("PNG", "RGBA", (224, 224)),
                                    ("PNG", "RGB", (223, 224))):
            stream = io.BytesIO()
            Image.new(mode, size).save(stream, format=format_)
            samples.append(stream.getvalue())
        samples.append(b"not a PNG")
        for sample in samples:
            wire[IMAGE_KEYS[0]] = np.frombuffer(sample, dtype=np.uint8)
            with self.assertRaises(ValueError):
                decode_payload(self.pack(wire), PNG_CODEC)

    def test_digest_includes_metadata_and_exact_bytes(self):
        digest = decoded_fields_sha256(self.fields)
        for key in ("observation.state", "noise", IMAGE_KEYS[0], "task"):
            fields = self.fresh()
            if key == "task":
                fields[key] = np.array(str(fields[key].item()) + ".")
            elif key in IMAGE_KEYS:
                fields[key].flat[0] ^= 1
            else:
                fields[key].flat[0] += np.float32(.125)
            self.assertNotEqual(decoded_fields_sha256(fields), digest)

    def test_profile_reports_separate_nonnegative_decode_phases(self):
        for codec in (RAW_CODEC, PNG_CODEC):
            restored, timing = decode_payload_profiled(encode_payload(self.fields, codec), codec)
            self.assert_same(self.fields, restored)
            self.assertEqual(set(timing), {"npz_decode", "image_decode", "field_validate"})
            self.assertTrue(all(value >= 0 for value in timing.values()))

    def test_runtime_and_model_configuration_unchanged(self):
        here = Path(__file__).resolve().parent
        previous = here.with_name("lerobot_fast_service")
        for name in ("runtime.py", "pi05_low_memory.py", "canonical_input.npz", "canonical_input.npz.json"):
            self.assertEqual((here / name).read_bytes(), (previous / name).read_bytes(), name)
        old = json.loads((previous / "cloud_config.json").read_text())
        new = json.loads((here / "cloud_config.json").read_text())
        allowed = {"deployment_dir", "hash_cache_dir", "port", "local_port",
                   "supported_image_codecs", "production_image_codec"}
        self.assertEqual({k: v for k, v in old.items() if k not in allowed},
                         {k: v for k, v in new.items() if k not in allowed})

    def test_health_pins_include_codec_and_compile_precision(self):
        import service_control

        config = service_control.read_config(Path(__file__).with_name("cloud_config.json"))
        metadata = {"pid": 7, "startup_id": "test", "model_sha256": service_control.MODEL_SHA256,
            "model_id": config["model_id"], "config_sha256": "hash",
            "config_path": config["deployment_dir"] + "/cloud_config.json", "port": config["port"],
            "model_format": "full-native-PI05", "peft_present": False, "batch_size": 1,
            "chunk_size": 50, "num_inference_steps": 10, "compile_model": True,
            "compile_mode": "reduce-overhead", "compile_target": "denoise_step",
            "float32_matmul_precision": "highest", "wire_image_codecs": [RAW_CODEC, PNG_CODEC],
            "production_image_codec": PNG_CODEC, "action_shape": [50, 7],
            "units": ["degree"] * 6 + ["metre"], "representation": "absolute joint targets and gripper width",
            "full_loading": {"native_complete_checkpoint": True, "source_tensors": 813,
                "destination_tensors": 813, "missing_keys": [], "unexpected_keys": [], "shape_mismatches": {}}}
        health = {"ready": True, "warmed_up": True, "protocol_version": service_control.PROTOCOL,
                  "model_sha256": service_control.MODEL_SHA256, "metadata": metadata}
        state = {"pid": 7, "startup_id": "test"}
        service_control.validate_health(health, config, "hash", state)
        for key, wrong in (("wire_image_codecs", [PNG_CODEC, RAW_CODEC]),
                           ("production_image_codec", RAW_CODEC), ("compile_mode", "default"),
                           ("float32_matmul_precision", "high"), ("num_inference_steps", 5)):
            invalid = copy.deepcopy(health)
            invalid["metadata"][key] = wrong
            with self.assertRaises(RuntimeError):
                service_control.validate_health(invalid, config, "hash", state)

    def test_benchmark_defaults_and_balanced_alternating_order(self):
        import client

        expected = [(0, RAW_CODEC), (0, PNG_CODEC), (1, PNG_CODEC), (1, RAW_CODEC)]
        self.assertEqual(list(client.codec_order("ab", 2)), expected)
        with patch("sys.argv", ["client.py", "--url", "http://127.0.0.1:18814", "--input", "recorded.npz",
                                "--output", "unused"]), patch.object(client, "benchmark") as benchmark:
            client.main()
        args = benchmark.call_args.args[0]
        self.assertEqual((args.codec, args.idle_s, args.repeats, args.warmup), ("png", 1.0, 20, 3))


if __name__ == "__main__":
    unittest.main(verbosity=2)
