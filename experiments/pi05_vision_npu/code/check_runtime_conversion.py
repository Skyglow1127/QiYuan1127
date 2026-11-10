"""Test the installed runtime conversion symbols independently of the NPU."""
import ctypes as c
import numpy as np
import ml_dtypes
from common import ROOT,write_json
lib=c.CDLL('libflexmlrt.so')
result={}
for n in (63,64,65,1152*256):
    x=np.linspace(-3,3,n,dtype=np.float32)
    expected=x.astype(ml_dtypes.bfloat16)
    b=np.full(n,0x7fc0,dtype=np.uint16)
    f=getattr(lib,'_ZN8flexmlrt3Qdq19convert_f32_to_bf16EPKfPtmi')
    f.argtypes=[c.c_void_p,c.c_void_p,c.c_size_t,c.c_int];f.restype=None
    f(x.ctypes.data,b.ctypes.data,n,0)
    y=np.full(n,np.nan,dtype=np.float32)
    g=getattr(lib,'_ZN8flexmlrt3Qdq19convert_bf16_to_f32EPKtPfm')
    g.argtypes=[c.c_void_p,c.c_void_p,c.c_size_t];g.restype=None
    g(expected.ctypes.data,y.ctypes.data,n)
    result[str(n)]={'f32_to_bf16_mismatches':int(np.sum(b!=expected.view(np.uint16))),
      'bf16_to_f32_mismatches':int(np.sum(y!=expected.astype(np.float32))),
      'output_nan_count':int(np.isnan(y).sum())}
write_json(ROOT/'results/runtime_conversion_check.json',result);print(result)
