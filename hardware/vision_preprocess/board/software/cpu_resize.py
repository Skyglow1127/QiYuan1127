"""Owned-output native CPU baseline; includes mapping computation and allocation."""
import ctypes
from pathlib import Path
import numpy as np

class NativeCPU:
    def __init__(self,path=None):
        self.lib=ctypes.CDLL(str(path or Path(__file__).with_name('libvpr_cpu.so')))
        self.fn=self.lib.vpr_cpu_resize
        self.fn.argtypes=[ctypes.c_void_p,ctypes.c_void_p]+[ctypes.c_int]*5
        self.fn.restype=ctypes.c_int
    def resize(self,image,width,height):
        if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2] not in (3,4):raise ValueError('uint8 RGB/RGBX required')
        if not image.flags.c_contiguous:raise ValueError('contiguous input required')
        ih,iw,c=image.shape
        if not(1<=width<=iw<=1920 and 1<=height<=ih<=1080):raise ValueError('invalid dimensions')
        out=np.empty((height,width,c),dtype=np.uint8)
        if self.fn(image.ctypes.data,out.ctypes.data,iw,ih,width,height,c):raise RuntimeError('native resize failed')
        return out
