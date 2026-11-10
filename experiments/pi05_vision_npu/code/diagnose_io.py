"""Distinguish ORT output allocation from numerical conversion; actual patch weights."""
import argparse
from pathlib import Path
import numpy as np
import onnxruntime as ort
from common import ROOT, stats, write_json

p=argparse.ArgumentParser();p.add_argument('--native',action='store_true');a=p.parse_args()
out=ROOT/'results'/('repair_native_io' if a.native else 'repair_bound_io');out.mkdir(exist_ok=True)
model=ROOT/'models/stem_unvectorized_conv.onnx';cache=ROOT/'cache/stem_unvectorized_conv'
x=np.load(ROOT/'results/reference/pixels.npy');shape=(2,1152,16,16)
etype=1
if a.native:
    import onnx,ml_dtypes
    m=onnx.load(str(ROOT/'models/diagnostic_native_bf16.onnx'))
    conv=m.graph.node[1];conv.input[0]='pixels';conv.output[0]='result'
    del m.graph.node[:];m.graph.node.append(conv)
    m.graph.input[0].type.tensor_type.elem_type=16
    m.graph.output[0].type.tensor_type.elem_type=16
    model=ROOT/'models/repair_native_io.onnx';onnx.save(m,str(model))
    cache=ROOT/'cache/repair_native_io'
    x=x.astype(ml_dtypes.bfloat16);etype=16
opts=ort.SessionOptions();opts.intra_op_num_threads=4
opts.add_session_config_entry('session.disable_cpu_ep_fallback','1')
opts.enable_profiling=True;opts.profile_file_prefix=str(out/'ort')
config=ROOT/'code'/('vaiml_config.json' if a.native else 'vaiml_unvectorized.json')
s=ort.InferenceSession(str(model),opts,providers=[('VitisAIExecutionProvider',dict(config_file=str(config),cache_dir=str(cache),cache_key='probe',enable_cache_file_io_in_mem='0'))],enable_fallback=False)
s.disable_fallback()
y=np.full(shape,np.nan,dtype=x.dtype)
io=s.io_binding();io.bind_input('pixels','cpu',0,etype,x.shape,x.ctypes.data)
io.bind_output(s.get_outputs()[0].name,'cpu',0,etype,y.shape,y.ctypes.data)
s.run_with_iobinding(io);io.synchronize_outputs()
np.save(out/'output.npy',y.astype(np.float32))
ref=np.load(ROOT/'results/stage_diagnostic/gpu_stages.npz')['patch_conv']
r=dict(nan_fraction=float(np.mean(np.isnan(y))),zero_fraction=float(np.mean(y==0)),comparison=stats(ref,y.astype(np.float32)),profile=s.end_profiling())
write_json(out/'comparison.json',r);print(r,flush=True)
