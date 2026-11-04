"""Lossless, bounded PI05 wire formats; no model, network or robot imports.

Both codecs return the original RGB CHW uint8 images, state and optional noise.
PNG is only a transport encoding: resizing/quantization happens before this API.
"""
from __future__ import annotations

import hashlib
import io
import json
import time
import zipfile

import numpy as np

RAW_CODEC = "raw-npz-v1"
PNG_CODEC = "png-rgb-u8-v1"
SUPPORTED_CODECS = (RAW_CODEC, PNG_CODEC)
CODEC_HEADER = "X-Image-Codec"
IMAGE_KEYS = ("observation.images.global", "observation.images.wrist")
REQUIRED_KEYS = {"protocol_version", "task", "observation.state", *IMAGE_KEYS}
MAX_PAYLOAD_BYTES = 512 * 1024
MAX_ARCHIVE_BYTES = 400 * 1024


def _check_codec(codec):
    if codec not in SUPPORTED_CODECS:
        raise ValueError(f"Unsupported image codec: {codec!r}")


def _validate(fields):
    keys = set(fields)
    if keys not in (REQUIRED_KEYS, REQUIRED_KEYS | {"noise"}):
        raise ValueError("Unexpected input fields")
    for key in ("protocol_version", "task"):
        value = fields[key]
        if not isinstance(value, np.ndarray) or value.shape != () or value.dtype.kind != "U":
            raise ValueError(f"Expected scalar Unicode string: {key}")
    for key, shape in (("observation.state", (7,)), ("noise", (1, 50, 32))):
        if key not in fields:
            continue
        value = fields[key]
        if not isinstance(value, np.ndarray) or value.dtype != np.float32 \
                or value.shape != shape or not np.isfinite(value).all():
            raise ValueError(f"Expected finite float32{shape}: {key}")
    for key in IMAGE_KEYS:
        value = fields[key]
        if not isinstance(value, np.ndarray) or value.dtype != np.uint8 or value.shape != (3, 224, 224):
            raise ValueError(f"Expected RGB uint8[3,224,224]: {key}")


def decoded_fields_sha256(fields):
    """Hash sorted field names, dtypes, shapes and exact C-order decoded bytes."""
    _validate(fields)
    digest = hashlib.sha256()
    for key in sorted(fields):
        value = fields[key]
        descriptor = json.dumps([key, value.dtype.str, list(value.shape)],
                                ensure_ascii=True, separators=(",", ":")).encode()
        digest.update(len(descriptor).to_bytes(8, "big"))
        digest.update(descriptor)
        raw = value.tobytes(order="C")
        digest.update(len(raw).to_bytes(8, "big"))
        digest.update(raw)
    return digest.hexdigest()


def field_byte_hashes(fields):
    _validate(fields)
    return {key: {"dtype": fields[key].dtype.str, "shape": list(fields[key].shape),
                  "sha256": hashlib.sha256(fields[key].tobytes(order="C")).hexdigest()}
            for key in sorted(fields)}


def encode_payload(fields: dict, codec: str) -> bytes:
    _check_codec(codec)
    _validate(fields)
    wire = dict(fields)
    if codec == PNG_CODEC:
        import cv2

        for key in IMAGE_KEYS:
            rgb = np.ascontiguousarray(fields[key].transpose(1, 2, 0))
            # OpenCV encodes BGR; decoding below explicitly restores RGB.
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, encoded = cv2.imencode(".png", bgr, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                raise ValueError(f"PNG encoding failed: {key}")
            wire[key] = encoded.reshape(-1)
    stream = io.BytesIO()
    np.savez_compressed(stream, **wire)
    payload = stream.getvalue()
    if not 0 < len(payload) <= MAX_PAYLOAD_BYTES:
        raise ValueError("Encoded payload exceeds compact wire limit")
    return payload


def decode_payload_profiled(payload: bytes, codec: str):
    _check_codec(codec)
    if not 0 < len(payload) <= MAX_PAYLOAD_BYTES:
        raise ValueError("Expected compact nonempty payload")
    began = time.perf_counter()
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        entries = archive.infolist()
        names = {entry.filename for entry in entries}
        mandatory = {key + ".npy" for key in REQUIRED_KEYS}
        if names not in (mandatory, mandatory | {"noise.npy"}) or len(entries) != len(names) \
                or sum(entry.file_size for entry in entries) > MAX_ARCHIVE_BYTES:
            raise ValueError("Unexpected NPZ fields, duplicates or decompressed size")
    with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
        fields = {key: archive[key].copy() for key in archive.files}
    npz_end = time.perf_counter()
    if codec == PNG_CODEC:
        from PIL import Image

        for key in IMAGE_KEYS:
            encoded = fields[key]
            if encoded.dtype != np.uint8 or encoded.ndim != 1 or not encoded.size:
                raise ValueError(f"Expected one-dimensional PNG uint8 bytes: {key}")
            try:
                with Image.open(io.BytesIO(encoded.tobytes())) as image:
                    if image.format != "PNG" or image.mode != "RGB" or image.size != (224, 224) \
                            or getattr(image, "n_frames", 1) != 1:
                        raise ValueError(f"Expected one RGB PNG[224,224]: {key}")
                    rgb = np.asarray(image.convert("RGB"), dtype=np.uint8)
                    fields[key] = np.ascontiguousarray(rgb.transpose(2, 0, 1))
            except OSError as error:
                raise ValueError(f"Invalid PNG: {key}") from error
    image_end = time.perf_counter()
    _validate(fields)
    return fields, {"npz_decode": npz_end - began, "image_decode": image_end - npz_end,
                    "field_validate": time.perf_counter() - image_end}


def decode_payload(payload: bytes, codec: str) -> dict:
    return decode_payload_profiled(payload, codec)[0]
