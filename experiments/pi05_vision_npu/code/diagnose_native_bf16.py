"""Check native BF16 constants/Conv with explicit FP32 boundary Cast nodes."""
import os
from pathlib import Path
import json
import time
import numpy as np
import ml_dtypes
import onnx
from onnx import helper as h,TensorProto as T
import onnxruntime as ort
from common import ROOT,stats,write_json
from reused_npu_audit import CompilerEvidenceWatcher,audit_session

def main():
    out=ROOT/'results/stem_native_bf16';out.mkdir(exist_ok=True)
    src=onnx.load(str(ROOT/'models/vision.onnx'),load_external_data=False)
    conv=src.graph.node[0]; used=set(conv.input)
    weights=[]
    for v in src.graph.initializer:
        if v.name not in used: continue
        a=onnx.numpy_helper.to_array(v,base_dir=str(ROOT/'models')).astype(ml_dtypes.bfloat16)
        weights.append(onnx.numpy_helper.from_array(a,v.name))
    conv.input[0]='pixels_bf16';conv.output[0]='conv_bf16'
    nodes=[h.make_node('Cast',['pixels'],['pixels_bf16'],to=T.BFLOAT16),conv,
           h.make_node('Cast',['conv_bf16'],['result'],to=T.FLOAT)]
    graph=h.make_graph(nodes,'real_patch_native_bf16',[src.graph.input[0]],
        [h.make_tensor_value_info('result',T.FLOAT,[2,1152,16,16])],weights)
    model=h.make_model(graph,opset_imports=[h.make_opsetid('',22)]);model.ir_version=9
    path=ROOT/'models/diagnostic_native_bf16.onnx';onnx.save_model(model,str(path));onnx.checker.check_model(str(path))
    ck=ROOT/'cache/stem_native_bf16';report=ck/'probe/vitisai_ep_report.json';report.parent.mkdir(parents=True,exist_ok=True)
    os.environ['XLNX_ONNX_EP_REPORT_FILE']=str(report)
    opt=ort.SessionOptions();opt.intra_op_num_threads=8;opt.enable_profiling=True
    opt.profile_file_prefix=str(out/'ort_native_bf16');opt.add_session_config_entry('session.disable_cpu_ep_fallback','1')
    options={'config_file':str(ROOT/'code/vaiml_config.json'),'cache_dir':str(ck),'cache_key':'probe','enable_cache_file_io_in_mem':'0'}
    w=CompilerEvidenceWatcher(report.parent);w.start();t=time.perf_counter()
    try:s=ort.InferenceSession(str(path),opt,providers=[('VitisAIExecutionProvider',options)],enable_fallback=False)
    finally:w.stop()
    seconds=time.perf_counter()-t
    y=s.run(None,{'pixels':np.load(ROOT/'results/reference/pixels.npy')})[0]
    np.save(out/'npu.npy',y)
    with np.load(ROOT/'results/stage_diagnostic/gpu_stages.npz') as ref:r=stats(ref['patch_conv'],y)
    audit=audit_session(s,report,Path(s.end_profiling()),report.parent,report.parent/'compiler_evidence',requested_options={'target':'VAIML',**options})
    write_json(out/'execution_audit.json',audit)
    r.update(create_seconds=seconds,zero_fraction=float(np.mean(y==0)),npu_only_proven=audit['npu_only_proven'])
    write_json(out/'comparison.json',r);print(json.dumps(r,indent=2),flush=True)
if __name__=='__main__':main()
