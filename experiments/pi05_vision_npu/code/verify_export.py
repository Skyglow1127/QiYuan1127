"""One real dual-camera FP32 ONNX/GPU equivalence check before BF16 compilation."""
import time
import argparse
from pathlib import Path
import numpy as np
import onnxruntime as ort
from common import ROOT, stats, write_json

ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,default=ROOT/'models/vision_batch1.onnx')
ap.add_argument('--name',default='export_equivalence');args=ap.parse_args()
opt=ort.SessionOptions(); opt.intra_op_num_threads=8; opt.inter_op_num_threads=1
t=time.perf_counter()
s=ort.InferenceSession(str(args.model),opt,providers=['CPUExecutionProvider'])
load=time.perf_counter()-t
x=np.load(ROOT/'results/reference/pixels.npy')
input_name=s.get_inputs()[0].name
if input_name=='patches':
    from patchify import patchify
    x=patchify(x)
device_batch=s.get_inputs()[0].shape[0]
t=time.perf_counter()
y=np.concatenate([s.run(None,{input_name:x[i:i+device_batch]})[0] for i in range(0,len(x),device_batch)])
run=time.perf_counter()-t
np.save(ROOT/f'results/reference/{args.name}_fp32_features.npy',y)
r=stats(np.load(ROOT/'results/reference/gpu_features.npy'),y)
r.update(load_seconds=load,run_seconds=run,backend='CPUExecutionProvider',purpose='export equivalence only; not NPU evidence')
write_json(ROOT/f'results/reference/{args.name}.json',r)
print(r,flush=True)
assert r['all_finite'] and r['cosine'] > .99999 and r['max_abs'] < .005, r
