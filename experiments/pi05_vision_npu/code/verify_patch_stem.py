"""Confirm Conv14/stride14 vs exact real-weight GEMM with single-output probes."""
import json
import os
from pathlib import Path
import time
import argparse
import numpy as np
import onnx
from onnx import helper as h,TensorProto as T
import onnxruntime as ort
from common import ROOT,stats,write_json
from patchify import patchify
from reused_npu_audit import CompilerEvidenceWatcher,audit_session

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--name',default='stem_diagnostic')
    ap.add_argument('--config',type=Path,default=ROOT/'code/vaiml_config.json')
    ap.add_argument('--batch',type=int,default=2)
    ap.add_argument('--dtype',choices=['float32','float16'],default='float32')
    ap.add_argument('--only-conv',action='store_true');args=ap.parse_args()
    out=ROOT/'results'/args.name;out.mkdir(exist_ok=True)
    pixels=np.load(ROOT/'results/reference/pixels.npy')
    with np.load(ROOT/'results/stage_diagnostic/gpu_stages.npz') as ref:expected=ref['patch_conv'].copy()
    results={}
    candidates=[('conv','vision.onnx')] if args.only_conv else [('conv','vision.onnx'),('gemm','vision_patchify.onnx')]
    for name,source in candidates:
        src=onnx.load(str(ROOT/'models'/source),load_external_data=False)
        nodes=list(src.graph.node[:(1 if name=='conv' else 2)])
        used={x for n in nodes for x in n.input}
        weights=[x for x in src.graph.initializer if x.name in used]
        src.graph.input[0].type.tensor_type.shape.dim[0].dim_value=args.batch
        dtype=T.FLOAT
        if args.dtype=='float16':
            weights=[onnx.numpy_helper.from_array(onnx.numpy_helper.to_array(v,base_dir=str(ROOT/'models')).astype(np.float16),v.name) for v in weights]
            dtype=T.FLOAT16;src.graph.input[0].type.tensor_type.elem_type=dtype
        shape=[args.batch,1152,16,16] if name=='conv' else [args.batch,256,1152]
        graph=h.make_graph(nodes,'real_patch_'+name,[src.graph.input[0]],
            [h.make_tensor_value_info(nodes[-1].output[0],dtype,shape)],weights)
        model=h.make_model(graph,opset_imports=[h.make_opsetid('',17)]);model.ir_version=9
        path=ROOT/f'models/{args.name}_{name}.onnx';onnx.save_model(model,str(path))
        opt=ort.SessionOptions();opt.intra_op_num_threads=8;opt.log_severity_level=2
        feed={src.graph.input[0].name:(pixels[:args.batch] if name=='conv' else patchify(pixels)[:args.batch])}
        if args.dtype=='float16': feed={k:v.astype(np.float16) for k,v in feed.items()}
        target=(expected if name=='conv' else expected.reshape(2,1152,256).transpose(0,2,1))[:args.batch]
        cpu=ort.InferenceSession(str(path),opt,providers=['CPUExecutionProvider']).run(None,feed)[0]
        ck=ROOT/f'cache/{args.name}_{name}'; report=ck/'probe/vitisai_ep_report.json';report.parent.mkdir(parents=True,exist_ok=True)
        os.environ['XLNX_ONNX_EP_REPORT_FILE']=str(report)
        opt.enable_profiling=True;opt.profile_file_prefix=str(out/f'ort_{name}')
        opt.add_session_config_entry('session.disable_cpu_ep_fallback','1')
        options={'config_file':str(args.config),'cache_dir':str(ck),'cache_key':'probe','enable_cache_file_io_in_mem':'0'}
        watcher=CompilerEvidenceWatcher(report.parent);watcher.start();t=time.perf_counter()
        try:s=ort.InferenceSession(str(path),opt,providers=[('VitisAIExecutionProvider',options)],enable_fallback=False)
        finally:watcher.stop()
        seconds=time.perf_counter()-t
        y=s.run(None,feed)[0];np.save(out/f'{name}_npu.npy',y)
        audit=audit_session(s,report,Path(s.end_profiling()),report.parent,report.parent/'compiler_evidence',requested_options={'target':'VAIML',**options})
        write_json(out/f'{name}_audit.json',audit)
        results[name]=dict(cpu_vs_gpu=stats(target,cpu),npu_vs_gpu=stats(target,y),zero_fraction=float(np.mean(y==0)),create_seconds=seconds,npu_only_proven=audit['npu_only_proven'])
        write_json(out/'comparison.json',results);print(name,json.dumps(results[name]),flush=True)
if __name__=='__main__':main()
