"""Opt-in Camera adapter for official LeRobot; no policy or Robot modification.

Both processed backends return HWC uint8 RGB. LeRobot performs its ordinary
uint8 -> [0,1] conversion and Pi05 then applies [-1,1] normalization once. The
camera uint8 contract introduces quantization versus Pi05's native float resize.
Keep source/read_raw available for the unmodified baseline and original data.
"""
import ast
from functools import lru_cache
import hashlib
from pathlib import Path
from time import perf_counter_ns, process_time_ns

import numpy as np

try:
    from .geometry import geometry, validate_image, validate_pl
except ImportError:
    from geometry import geometry, validate_image, validate_pl


DEFAULT_REFERENCE_SOURCE = Path(
    "/home/aup/Lerobot/envs/pi05-rocm-7.14.1/lib/python3.12/site-packages/"
    "lerobot/policies/common/vla_utils.py"
)


@lru_cache(maxsize=4)
def _official_function(source_path=None):
    """Load the installed pure function verbatim, without importing any model."""
    import torch
    import torch.nn.functional as functional

    source = Path(source_path) if source_path is not None else DEFAULT_REFERENCE_SOURCE
    text = source.read_text()
    tree = ast.parse(text, filename=str(source))
    matches = [node for node in tree.body if isinstance(node, ast.FunctionDef)
               and node.name == "resize_with_pad_torch"]
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one official resize_with_pad_torch function")
    node = matches[0]
    # Compile the original AST only. Other imports/functions in vla_utils are not run.
    namespace = {"torch": torch, "F": functional}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
    function_text = ast.get_source_segment(text, node)
    metadata = {"source_path": str(source.resolve()),
                "source_sha256": hashlib.sha256(text.encode()).hexdigest(),
                "function_sha256": hashlib.sha256(function_text.encode()).hexdigest(),
                "function": "resize_with_pad_torch", "torch_version": torch.__version__}
    return namespace["resize_with_pad_torch"], metadata


def reference_identity(source_path=None):
    return dict(_official_function(source_path)[1])


def official_float_reference(image, g, source_path=None):
    """Exact installed resize function; HWC float32 [0,1], before model *2-1."""
    import torch

    validate_image(image, g)
    if g != geometry(g.iw, g.ih, g.width, g.height):
        raise ValueError("Geometry does not match the installed Pi05 convention")
    resize, _ = _official_function(source_path)
    cpu_begin, begin = process_time_ns(), perf_counter_ns()
    with torch.inference_mode():
        input_tensor = torch.from_numpy(np.ascontiguousarray(image)).to(torch.float32).div(255.0)
        tensor = input_tensor.unsqueeze(0)
        # This is also the condition in Pi05._preprocess_images.
        if (g.ih, g.iw) != (g.height, g.width):
            tensor = resize(tensor, g.height, g.width)
        out = tensor.squeeze(0).contiguous().numpy().copy()
    end = perf_counter_ns()
    return out, {"preprocess_ns": end - begin, "cpu_time_ns": process_time_ns() - cpu_begin,
                 "reference": "installed_official_torch_float32_0_1"}


def quantized_cpu(image, g, source_path=None):
    """Official float resize followed by explicit uint8 camera-boundary rounding."""
    cpu_begin, begin = process_time_ns(), perf_counter_ns()
    reference, stages = official_float_reference(image, g, source_path)
    before_quantize = perf_counter_ns()
    result = np.rint(reference * np.float32(255.0)).clip(0, 255).astype(np.uint8)
    end = perf_counter_ns()
    return result, {**stages, "quantize_ns": end - before_quantize,
                    "preprocess_ns": end - begin, "cpu_time_ns": process_time_ns() - cpu_begin,
                    "reference": "official_float_resize_then_uint8_quantization"}


def quality_metrics(actual, reference, g):
    """Float-reference errors in byte equivalents and Pi05 normalized [-1,1]."""
    if actual.dtype != np.uint8 or actual.shape != (g.height, g.width, 3):
        raise ValueError("Actual output must be compact RGB888 canvas")
    if reference.dtype != np.float32 or reference.shape != actual.shape:
        raise ValueError("Reference must be HWC float32 [0,1] of identical shape")
    if not np.isfinite(reference).all() or reference.min() < 0 or reference.max() > 1:
        raise ValueError("Invalid float reference range")
    difference = np.abs(actual.astype(np.float32) / np.float32(255.0) - reference)
    border = np.ones(actual.shape[:2], dtype=bool)
    border[g.top:g.top + g.rh, g.left:g.left + g.rw] = False
    content = difference[g.top:g.top + g.rh, g.left:g.left + g.rw]
    return {"max_error_byte_equivalent": float(difference.max() * 255),
            "mae_byte_equivalent": float(difference.mean(dtype=np.float64) * 255),
            "content_mae_byte_equivalent": float(content.mean(dtype=np.float64) * 255),
            "normalized_max_error": float(difference.max() * 2),
            "normalized_mae": float(difference.mean(dtype=np.float64) * 2),
            "padding_exact": bool(np.all(actual[border] == 0)),
            "model_action_regression": "NOT_RUN"}


class Pi05ImageAdapter:
    """Pure image boundary; PL transport must be provided explicitly by caller."""
    def __init__(self, backend="software", transport=None, width=224, height=224,
                 source_path=None):
        if backend not in ("software", "pl", "raw"):
            raise ValueError("backend must be software, pl or raw")
        if backend == "pl" and transport is None:
            raise ValueError("PL backend requires an explicit transport; no automatic fallback")
        if backend != "pl" and transport is not None:
            raise ValueError("A transport is only used by the explicit PL backend")
        geometry(16, 16, width, height)  # Early target dimension/type validation.
        self.backend, self.transport = backend, transport
        self.width, self.height, self.source_path = width, height, source_path
        self.last_metadata = None

    def process(self, image):
        if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("Expected HWC RGB image")
        g = geometry(image.shape[1], image.shape[0], self.width, self.height)
        validate_image(image, g)
        begin = perf_counter_ns()
        if self.backend == "raw":
            output, stages = image, {"preprocess_ns": 0}
        elif self.backend == "software":
            output, stages = quantized_cpu(image, g, self.source_path)
        else:
            validate_pl(g)
            output, stages = self.transport.run(np.ascontiguousarray(image), g)
        expected_shape = image.shape if self.backend == "raw" else (self.height, self.width, 3)
        if (not isinstance(output, np.ndarray) or output.dtype != np.uint8 or
                output.shape != expected_shape or not output.flags.c_contiguous):
            raise RuntimeError("Backend did not return compact HWC RGB888 with expected dimensions")
        self.last_metadata = {"backend": self.backend, "geometry": g.metadata(),
                              "stages": stages, "adapter_wall_ns": perf_counter_ns() - begin,
                              "pixel_domain": "RGB_uint8_0_255", "normalization_applied": False,
                              "quantized_camera_boundary": self.backend != "raw"}
        return output

    def process_mapping(self, images):
        """Preserve every caller-defined image key; no camera role inference."""
        return {key: self.process(value) for key, value in images.items()}


def camera_class():
    """Lazy Camera dependency keeps vector/reference use independent of LeRobot imports."""
    from lerobot.cameras.camera import Camera
    from lerobot.cameras.configs import CameraConfig

    class Pi05Camera(Camera):
        """Inject an existing RGB Camera; owns no transport and discovers no devices."""
        def __init__(self, source, backend="software", transport=None, width=224, height=224,
                     source_path=None):
            self.source = source
            self.adapter = Pi05ImageAdapter(backend, transport, width, height, source_path)
            raw = backend == "raw"
            config = CameraConfig(fps=source.fps, width=source.width if raw else width,
                                  height=source.height if raw else height)
            super().__init__(config)
            self.last_metadata = None
            self._owns_source_connection = False

        @property
        def is_connected(self):
            return self.source.is_connected

        @staticmethod
        def find_cameras():
            return []  # The caller supplies its existing camera; do not probe USB or HTTP.

        def connect(self, warmup=True):
            if not self.source.is_connected:
                self.source.connect(warmup=warmup)
                self._owns_source_connection = True

        def _process(self, frame):
            output = self.adapter.process(frame)
            self.last_metadata = {"source": getattr(self.source, "last_metadata", None),
                                  "preprocessing": self.adapter.last_metadata}
            return output

        def read(self):
            return self._process(self.source.read())

        def async_read(self, timeout_ms=200):
            return self._process(self.source.async_read(timeout_ms=timeout_ms))

        def read_latest(self, max_age_ms=500):
            return self._process(self.source.read_latest(max_age_ms=max_age_ms))

        def read_raw(self):
            """Original image path for dataset capture or native float-model baseline."""
            return self.source.read()

        def disconnect(self):
            if self._owns_source_connection and self.source.is_connected:
                self.source.disconnect()
            self._owns_source_connection = False

    return Pi05Camera


def __getattr__(name):
    # ``from pi05_adapter import Pi05Camera`` yields a real official Camera subclass.
    if name == "Pi05Camera":
        result = camera_class()
        globals()[name] = result
        return result
    raise AttributeError(name)
