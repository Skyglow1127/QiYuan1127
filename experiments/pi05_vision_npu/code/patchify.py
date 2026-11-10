"""Exact im2col for original non-overlapping Conv2d(kernel=14,stride=14).

No interpolation, normalization, learned operations, or numerical arithmetic.
Only a byte-preserving permutation; the real patch weights are applied on NPU.
"""
import numpy as np
def patchify(pixels):
    assert pixels.shape==(2,3,224,224) and pixels.dtype==np.float32
    return np.ascontiguousarray(pixels.reshape(2,3,16,14,16,14).transpose(0,2,4,1,3,5).reshape(2,256,588))
