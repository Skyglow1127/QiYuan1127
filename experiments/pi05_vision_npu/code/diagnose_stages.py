"""One first-block probe to localize the real-weight VAIML precision failure."""
import argparse
import json
import os
from pathlib import Path
import time
import numpy as np
from common import ROOT, BASE, stats, write_json

OUT=ROOT/'results/stage_diagnostic'
NAMES=['patch_conv','position_add','layer0_ln1','layer0_scaled_scores','layer0_softmax',
       'layer0_attention_residual','layer0_ln2','layer0_fc1_bias','layer0_gelu',
       'layer0_fc2_bias','layer0_mlp_residual']

def build():
    import onnx
    from onnx import helper as h,TensorProto as T
    m=onnx.load(str(ROOT/'models/vision.onnx'),load_external_data=False)
    nodes=[]; mapping={}
    for node in m.graph.node:
        nodes.append(node)
        if node.name in NAMES: mapping[node.name]=node.output[0]
        if node.name=='layer0_mlp_residual': break
    used={x for node in nodes for x in node.input}
    initializers=[v for v in m.graph.initializer if v.name in used]
    shapes={name:[2,256,1152] for name in NAMES}
    shapes.update(patch_conv=[2,1152,16,16],layer0_scaled_scores=[2,16,256,256],
                  layer0_softmax=[2,16,256,256],layer0_fc1_bias=[2,256,4304],layer0_gelu=[2,256,4304])
    graph=h.make_graph(nodes,'real_first_siglip_block',[m.graph.input[0]],
        [h.make_tensor_value_info(mapping[name],T.FLOAT,shapes[name]) for name in NAMES],initializers)
    model=h.make_model(graph,opset_imports=[h.make_opsetid('',17)]); model.ir_version=9
    path=ROOT/'models/diagnostic_first_block.onnx'
    onnx.save_model(model,str(path)); onnx.checker.check_model(str(path),full_check=True)
    write_json(OUT/'mapping.json',mapping); print('Probe graph saved',len(nodes),len(initializers))

def reference():
    import torch
    from torch import nn
    from accelerate import init_empty_weights
    from accelerate.utils import set_module_tensor_to_device
    from safetensors import safe_open
    from transformers import SiglipVisionConfig
    from transformers.models.siglip.modeling_siglip import SiglipVisionEmbeddings,SiglipEncoderLayer
    torch.set_num_threads(8)
    c=SiglipVisionConfig.from_dict(json.loads((ROOT/'results/reference/vision_config.json').read_text()))
    c._attn_implementation='eager'
    with init_empty_weights():
        model=nn.Module(); model.embeddings=SiglipVisionEmbeddings(c); model.layer=SiglipEncoderLayer(c)
    prefix='paligemma_with_expert.paligemma.model.vision_tower.vision_model.'
    with safe_open(str(BASE/'model.safetensors'),framework='pt') as sf:
        for name,_ in model.named_parameters():
            source=prefix+name if name.startswith('embeddings.') else prefix+name.replace('layer.','encoder.layers.0.',1)
            set_module_tensor_to_device(model,name,'cuda',value=sf.get_tensor(source).to('cuda'))
    set_module_tensor_to_device(model,'embeddings.position_ids','cuda',value=torch.arange(256,device='cuda').expand(1,-1))
    model.eval().requires_grad_(False); result={}
    with torch.inference_mode():
        x=torch.from_numpy(np.load(ROOT/'results/reference/pixels.npy')).to('cuda')
        p=model.embeddings.patch_embedding(x);result['patch_conv']=p
        x=p.flatten(2).transpose(1,2)+model.embeddings.position_embedding(model.embeddings.position_ids)
        result['position_add']=x; layer=model.layer
        n=layer.layer_norm1(x);result['layer0_ln1']=n
        q=layer.self_attn.q_proj(n).reshape(2,256,16,72).transpose(1,2)
        k=layer.self_attn.k_proj(n).reshape(2,256,16,72).transpose(1,2)
        v=layer.self_attn.v_proj(n).reshape(2,256,16,72).transpose(1,2)
        score=(q@k.transpose(-1,-2))*72**-.5;result['layer0_scaled_scores']=score
        a=torch.softmax(score,dim=-1);result['layer0_softmax']=a
        a=(a@v).transpose(1,2).reshape(2,256,1152)
        x=x+layer.self_attn.out_proj(a);result['layer0_attention_residual']=x
        n=layer.layer_norm2(x);result['layer0_ln2']=n
        n=layer.mlp.fc1(n);result['layer0_fc1_bias']=n
        n=layer.mlp.activation_fn(n);result['layer0_gelu']=n
        n=layer.mlp.fc2(n);result['layer0_fc2_bias']=n
        result['layer0_mlp_residual']=x+n
    np.savez(OUT/'gpu_stages.npz',**{k:v.cpu().numpy() for k,v in result.items()})
    print('GPU stage reference saved')

def npu():
    import onnxruntime as ort
    from reused_npu_audit import CompilerEvidenceWatcher,audit_session
    ck=ROOT/'cache/stage_diagnostic';ck.mkdir(parents=True,exist_ok=True)
    os.environ['XLNX_ONNX_EP_REPORT_FILE']=str(ck/'vitisai_ep_report.json')
    opt=ort.SessionOptions();opt.intra_op_num_threads=8; opt.enable_profiling=True
    opt.profile_file_prefix=str(OUT/'ort_probe');opt.add_session_config_entry('session.disable_cpu_ep_fallback','1')
    options={'config_file':str(ROOT/'code/vaiml_config.json'),'cache_dir':str(ck),'cache_key':'first_block','enable_cache_file_io_in_mem':'0'}
    # Cache key dir is where the compiler watcher must scan.
    report=ck/'first_block/vitisai_ep_report.json';report.parent.mkdir(parents=True,exist_ok=True)
    os.environ['XLNX_ONNX_EP_REPORT_FILE']=str(report)
    watch=CompilerEvidenceWatcher(report.parent); watch.start();t=time.perf_counter()
    try:s=ort.InferenceSession(str(ROOT/'models/diagnostic_first_block.onnx'),opt,providers=[('VitisAIExecutionProvider',options)],enable_fallback=False)
    finally:watch.stop()
    load=time.perf_counter()-t
    outputs=s.run(None,{'pixels':np.load(ROOT/'results/reference/pixels.npy')})
    result=dict(zip(NAMES,outputs)); np.savez(OUT/'npu_stages.npz',**result)
    with np.load(OUT/'gpu_stages.npz') as ref:comparison={k:stats(ref[k],v) for k,v in result.items()}
    profile=Path(s.end_profiling())
    audit=audit_session(s,report,profile,report.parent,report.parent/'compiler_evidence',requested_options={'target':'VAIML',**options})
    write_json(OUT/'execution_audit.json',audit)
    write_json(OUT/'comparison.json',dict(load_seconds=load,stages=comparison))
    print(json.dumps(comparison,indent=2),flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['build','reference','npu']);a=ap.parse_args()
    OUT.mkdir(exist_ok=True)
    globals()[a.mode]()
