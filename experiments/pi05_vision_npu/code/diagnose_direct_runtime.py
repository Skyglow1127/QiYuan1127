"""Run actual patch Conv through the SDK runtime without ONNX Runtime EP."""
import time
import onnx
import numpy as np
import flexml
from common import ROOT,stats,write_json
out=ROOT/'results/repair_direct_runtime';out.mkdir(exist_ok=True)
m=onnx.load(str(ROOT/'models/stem_unvectorized_conv.onnx'))
t=time.perf_counter()
model=flexml.compile(m,None,output_dir=str(ROOT/'cache/repair_direct_runtime'),device='stx',optimize_level=1,
  preferred_data_storage='unvectorized',enable_f32_to_bf16_conversion=True)
compiled=time.perf_counter()-t
x=np.load(ROOT/'results/reference/pixels.npy');ref=np.load(ROOT/'results/stage_diagnostic/gpu_stages.npz')['patch_conv']
t=time.perf_counter();y=model(x);elapsed=time.perf_counter()-t
np.save(out/'output.npy',y)
r=dict(compile_seconds=compiled,run_seconds=elapsed,zero_fraction=float(np.mean(y==0)),comparison=stats(ref,y))
write_json(out/'comparison.json',r);print(r,flush=True)
