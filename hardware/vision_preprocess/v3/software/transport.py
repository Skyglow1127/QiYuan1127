"""VPR3 coherent DMA. RGBX input, compact RGB output; no CPU fallback."""
import ctypes
import fcntl
import mmap
import os
import struct
import sys
from pathlib import Path
from time import perf_counter_ns, process_time_ns

import numpy as np
if __package__:
    from .geometry import validate_pl
else:
    from geometry import validate_pl

BYTES = 1920 * 1080 * 4
REQUEST = struct.Struct('<14IQ')
IOCTL = (3 << 30) | (REQUEST.size << 16) | (ord('V') << 8) | 3


class PL:
    def __init__(self, device='/dev/vpr3dma'):
        if sys.byteorder != 'little':
            raise RuntimeError('RGBX mapping requires a little-endian CPU')
        self.lib = ctypes.CDLL(str(Path(__file__).with_name('libvpr3_pixels.so')))
        self.lib.vpr3_pack.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        self.lib.vpr3_pack.restype = None
        self.fd = os.open(device, os.O_RDWR | os.O_CLOEXEC)
        try:
            self.tx = mmap.mmap(self.fd, BYTES, offset=0)
            self.rx = mmap.mmap(self.fd, BYTES, offset=BYTES)
            self.txptr = ctypes.addressof(ctypes.c_char.from_buffer(self.tx))
            self.rxptr = ctypes.addressof(ctypes.c_char.from_buffer(self.rx))
        except BaseException:
            if hasattr(self, 'rx'):
                self.rx.close()
            if hasattr(self, 'tx'):
                self.tx.close()
            os.close(self.fd)
            raise
        self.closed = False

    def close(self):
        if not self.closed:
            self.rx.close()
            self.tx.close()
            os.close(self.fd)
            self.closed = True

    def _transfer(self, iw, ih, ow, oh, bypass, content=0, pad=0):
        request = bytearray(REQUEST.pack(iw, ih, ow, oh, bypass,
                                        *([0] * 7), content, pad, 0))
        fcntl.ioctl(self.fd, IOCTL, request, True)
        fields = REQUEST.unpack(request)
        expected_bytes = ow * oh * (4 if bypass else 3)
        # Driver checks these as well; reject stale or incompatible userspace ABI.
        if (fields[5] & 3) != 2 or fields[6] or fields[7] != iw * ih or fields[8] != ow * oh:
            raise RuntimeError('VPR3 status/count contract failed: ' + repr(fields))
        if fields[9] & 0x770 or fields[10] & 0x770 or fields[11] != expected_bytes:
            raise RuntimeError('VPR3 DMA status/byte count contract failed: ' + repr(fields))
        return fields

    @staticmethod
    def _status(fields):
        return {'dma_reset_submit_and_wait_ns': fields[-1],
                'core_status': fields[5], 'core_errors': fields[6],
                'input_count': fields[7], 'output_count': fields[8],
                'tx_status': fields[9], 'rx_status': fields[10],
                'rx_bytes': fields[11]}

    def bypass(self, words):
        if words.dtype != np.uint32 or words.ndim != 2 or not words.flags.c_contiguous:
            raise ValueError('Contiguous uint32 words required')
        ih, iw = words.shape
        if not (16 <= iw <= 1920 and 16 <= ih <= 1080):
            raise ValueError('Bypass dimensions outside compiled input range')
        cpu_begin = process_time_ns()
        begin = perf_counter_ns()
        ctypes.memmove(self.txptr, words.ctypes.data, words.nbytes)
        copied = perf_counter_ns()
        fields = self._transfer(iw, ih, iw, ih, 1)
        completed = perf_counter_ns()
        out = np.empty_like(words)
        ctypes.memmove(out.ctypes.data, self.rxptr, out.nbytes)
        end = perf_counter_ns()
        return out, {'copy_to_coherent_ns': copied - begin,
                     'ioctl_wall_ns': completed - copied,
                     'copy_from_coherent_ns': end - completed,
                     'preprocess_ns': end - begin,
                     'cpu_time_ns': process_time_ns() - cpu_begin,
                     'input_bytes': words.nbytes, 'output_bytes': out.nbytes,
                     **self._status(fields)}

    def run(self, image, g):
        validate_pl(g)
        if image.dtype != np.uint8 or image.shape != (g.ih, g.iw, 3) or not image.flags.c_contiguous:
            raise ValueError('Contiguous RGB888 required')
        cpu_begin = process_time_ns()
        begin = perf_counter_ns()
        self.lib.vpr3_pack(image.ctypes.data, self.txptr, g.iw * g.ih)
        packed = perf_counter_ns()
        fields = self._transfer(g.iw, g.ih, g.width, g.height, 0,
                                (g.rh << 16) | g.rw, g.pad)
        completed = perf_counter_ns()
        out = np.empty((g.height, g.width, 3), np.uint8)
        # One contiguous copy gives the caller ownership across later DMA writes.
        ctypes.memmove(out.ctypes.data, self.rxptr, out.nbytes)
        end = perf_counter_ns()
        return out, {'cpu_time_ns': process_time_ns() - cpu_begin,
                     'pack_into_coherent_buffer_ns': packed - begin,
                     'ioctl_wall_ns': completed - packed,
                     'copy_from_coherent_ns': end - completed,
                     'preprocess_ns': end - begin,
                     'input_bytes': image.nbytes,
                     'dma_input_bytes': g.iw * g.ih * 4,
                     'output_bytes': out.nbytes, 'output_format': 'rgb888_packed',
                     **self._status(fields)}
