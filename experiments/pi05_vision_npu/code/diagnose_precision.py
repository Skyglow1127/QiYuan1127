"""Bounded BF16 diagnostic using only the real vision weights on the ROCm GPU."""
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from accelerate import init_empty_weights
from accelerate.utils import set_module_tensor_to_device
from safetensors import safe_open
from transformers import SiglipVisionModel, SiglipVisionConfig
from common import ROOT, BASE, write_json, stats

def main():
    out=ROOT/'results/precision_diagnostic'; out.mkdir(exist_ok=True)
    torch.set_num_threads(8)
    cfg=SiglipVisionConfig.from_dict(json.loads((ROOT/'results/reference/vision_config.json').read_text()))
    cfg._attn_implementation='eager'
    with init_empty_weights():
        model=nn.Module()
        model.vision_tower=SiglipVisionModel(cfg)
        model.multi_modal_projector=nn.Module()
        model.multi_modal_projector.linear=nn.Linear(1152,2048)
    prefix='paligemma_with_expert.paligemma.model.'
    with safe_open(str(BASE/'model.safetensors'),framework='pt') as sf:
        for name,_ in model.named_parameters():
            v=sf.get_tensor(prefix+name).to(device='cuda',dtype=torch.bfloat16)
            set_module_tensor_to_device(model,name,'cuda',value=v,dtype=torch.bfloat16)
    set_module_tensor_to_device(model,'vision_tower.vision_model.embeddings.position_ids','cuda',
        value=torch.arange(256,device='cuda').expand(1,-1))
    model.eval().requires_grad_(False)
    pixels=torch.from_numpy(np.load(ROOT/'results/reference/pixels.npy')).to('cuda',dtype=torch.bfloat16)
    with torch.inference_mode():
        y=model.multi_modal_projector.linear(model.vision_tower(pixels).last_hidden_state)
    a=y.float().cpu().numpy()
    np.save(out/'gpu_bf16_features.npy',a)
    fp32=np.load(ROOT/'results/reference/gpu_features.npy')
    npu=np.load(ROOT/'results/npu_first_compile/npu_features.npy')
    report=dict(gpu_bf16_vs_gpu_fp32=stats(fp32,a),npu_vs_gpu_bf16=stats(a,npu),
        note='Pure GPU BF16 reference isolates floating precision from AMD VAIML execution; eager attention uses FP32 softmax accumulation.')
    write_json(out/'precision.json',report); print(json.dumps(report,indent=2))

if __name__=='__main__': main()
