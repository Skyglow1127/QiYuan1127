"""Pi05 0.6.1 geometry and an explicitly separate OpenCV uint8 baseline.

Image input/output is compact HWC RGB888. This module neither normalizes model
tensors nor chooses camera roles. PL limits describe the reused Vitis kernel.
"""
from dataclasses import asdict, dataclass
from time import perf_counter_ns, process_time_ns

import numpy as np


@dataclass(frozen=True)
class Geometry:
    iw: int
    ih: int
    width: int
    height: int
    rw: int
    rh: int
    left: int
    top: int
    right: int
    bottom: int
    gain_x: float
    gain_y: float
    pad: int = 0
    rounding: str = "floor"
    interpolation: str = "bilinear_half_pixel"

    def metadata(self):
        return asdict(self)


def geometry(iw, ih, width=224, height=224, pad=0, rounding="floor"):
    """Match resize_with_pad_torch's ratio, int truncation and centered border."""
    for label, value in (("iw", iw), ("ih", ih), ("width", width), ("height", height)):
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 1:
            raise ValueError(f"{label} must be a positive integer")
    if pad != 0 or rounding != "floor":
        raise ValueError("Pi05 requires black padding (0) and int-truncated dimensions")
    iw, ih, width, height = map(int, (iw, ih, width, height))
    ratio = max(iw / width, ih / height)
    rw, rh = int(iw / ratio), int(ih / ratio)
    if min(rw, rh) < 1:
        raise ValueError("Resized content would have a zero axis")
    dw, dh = width - rw, height - rh
    return Geometry(iw, ih, width, height, rw, rh, dw // 2, dh // 2,
                    dw - dw // 2, dh - dh // 2, rw / iw, rh / ih)


def validate_image(image, g):
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
        raise ValueError("Input must be uint8 RGB, before any model normalization")
    if image.shape != (g.ih, g.iw, 3):
        raise ValueError(f"Expected RGB shape {(g.ih, g.iw, 3)}, got {image.shape}")


def validate_pl(g):
    if not (16 <= g.iw <= 1920 and 16 <= g.ih <= 1080 and
            16 <= g.width <= 640 and 16 <= g.height <= 640):
        raise ValueError("PL input axes 16..1920 x 16..1080; canvas axes 16..640")
    if min(g.rw, g.rh) < 2 or g.iw > 4 * g.rw or g.ih > 4 * g.rh:
        raise ValueError("PL content axes >=2; maximum downscale is 4 per axis")
    if g != geometry(g.iw, g.ih, g.width, g.height):
        raise ValueError("PL geometry must use the Pi05 black-border contract")


def cpu(image, g):
    """OpenCV CPU baseline; not the official PyTorch floating-point reference."""
    import cv2

    validate_image(image, g)
    cpu_begin, begin = process_time_ns(), perf_counter_ns()
    resized = (image if (g.rw, g.rh) == (g.iw, g.ih) else
               cv2.resize(image, (g.rw, g.rh), interpolation=cv2.INTER_LINEAR))
    middle = perf_counter_ns()
    out = cv2.copyMakeBorder(resized, g.top, g.bottom, g.left, g.right,
                            cv2.BORDER_CONSTANT, value=(0, 0, 0))
    end = perf_counter_ns()
    return out, {"resize_ns": middle - begin, "border_ns": end - middle,
                 "preprocess_ns": end - begin, "cpu_time_ns": process_time_ns() - cpu_begin,
                 "reference": "opencv_uint8_not_official_torch_float"}
