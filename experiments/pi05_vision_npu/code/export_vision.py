"""Export real PI05 SigLIP+projector to a static, external-weight ONNX graph.

Operations mirror the installed SiglipVisionTransformer and PaliGemma projector.
No Torch/ONNX exporter dependencies and no random or substitute parameters.
The FP32 CPU ONNX/reference comparison is a separate required validation.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import onnx
from onnx import helper as h, TensorProto as T
from safetensors import safe_open
from common import ROOT, BASE, write_json

class Exporter:
    def __init__(self, output):
        self.output = output
        self.data = output.with_suffix('.data')
        self.stream = self.data.open('wb')
        self.nodes, self.initializers, self.weights = [], [], []
        self.used = set()
        self.source = safe_open(str(BASE / 'model.safetensors'), framework='np')
        self.v = 'paligemma_with_expert.paligemma.model.vision_tower.vision_model.'
        self.p = 'paligemma_with_expert.paligemma.model.multi_modal_projector.linear.'
        self.n = 0
    def name(self, stem):
        self.n += 1
        return f'{stem}_{self.n}'
    def node(self, op, inputs, stem, output=None, **attrs):
        out = output or self.name(stem)
        self.nodes.append(h.make_node(op,inputs,[out],name=stem,**attrs))
        return out
    def const(self, value, stem, dtype=np.float32):
        name = self.name(stem)
        self.initializers.append(onnx.numpy_helper.from_array(np.asarray(value,dtype=dtype),name))
        return name
    def weight(self, key, transpose=False, prepend_batch=False, flatten_kernel=False):
        arr = self.source.get_tensor(key)
        assert arr.dtype == np.float32, (key, arr.dtype)
        original_shape = list(arr.shape)
        if flatten_kernel: arr = arr.reshape(arr.shape[0],-1)
        if transpose: arr = arr.T
        if prepend_batch: arr = arr[None]
        arr = np.ascontiguousarray(arr)
        padding = (-self.stream.tell()) % 64
        self.stream.write(bytes(padding))
        offset = self.stream.tell()
        raw = arr.tobytes()
        self.stream.write(raw)
        name = self.name(key.replace('.','_'))
        tensor = T(name=name,data_type=T.FLOAT,dims=list(arr.shape),data_location=T.EXTERNAL)
        for k,v in [('location',self.data.name),('offset',str(offset)),('length',str(len(raw)))]:
            item = tensor.external_data.add(); item.key=k; item.value=v
        self.initializers.append(tensor)
        self.used.add(key)
        self.weights.append(dict(source_key=key,onnx_name=name,source_shape=original_shape,
            onnx_shape=list(arr.shape),transpose=transpose,flatten_kernel=flatten_kernel,sha256=hashlib.sha256(raw).hexdigest()))
        return name
    def reshape(self,x,shape,stem):
        return self.node('Reshape',[x,self.const(shape,stem+'_shape',np.int64)],stem)
    def linear(self,x,key,stem):
        y=self.node('MatMul',[x,self.weight(key+'weight',transpose=True)],stem+'_matmul')
        return self.node('Add',[y,self.weight(key+'bias')],stem+'_bias')
    def ln(self,x,key,stem):
        return self.node('LayerNormalization',[x,self.weight(key+'weight'),self.weight(key+'bias')],
                         stem,axis=-1,epsilon=1e-6,stash_type=1)
    def gelu(self,x,stem):
        c=self.node('Pow',[x,self.const(3,stem+'_three')],stem+'_cube')
        c=self.node('Mul',[c,self.const(.044715,stem+'_coef')],stem+'_cubic')
        c=self.node('Add',[x,c],stem+'_poly')
        c=self.node('Mul',[c,self.const(math.sqrt(2/math.pi),stem+'_sqrt2pi')],stem+'_argument')
        c=self.node('Tanh',[c],stem+'_tanh')
        c=self.node('Add',[c,self.const(1,stem+'_one')],stem+'_plus1')
        c=self.node('Mul',[c,self.const(.5,stem+'_half')],stem+'_scale')
        return self.node('Mul',[x,c],stem)
    def build(self,c,batch,patchify_input=False):
        d,heads,patch=c['hidden_size'],c['num_attention_heads'],c['patch_size']
        side=c['image_size']; tokens=(side//patch)**2; hd=d//heads
        assert c['hidden_act']=='gelu_pytorch_tanh' and c['layer_norm_eps']==1e-6
        if patchify_input:
            x=self.node('MatMul',['patches',self.weight(self.v+'embeddings.patch_embedding.weight',
                flatten_kernel=True,transpose=True)],'patch_matmul')
            x=self.node('Add',[x,self.weight(self.v+'embeddings.patch_embedding.bias')],'patch_bias')
        else:
            x=self.node('Conv',['pixels',self.weight(self.v+'embeddings.patch_embedding.weight'),
                               self.weight(self.v+'embeddings.patch_embedding.bias')],
                               'patch_conv',kernel_shape=[patch,patch],strides=[patch,patch])
            x=self.reshape(x,[batch,d,tokens],'patch_flatten')
            x=self.node('Transpose',[x],'patch_tokens',perm=[0,2,1])
        x=self.node('Add',[x,self.weight(self.v+'embeddings.position_embedding.weight',prepend_batch=True)],'position_add')
        for i in range(c['num_hidden_layers']):
            key=self.v+f'encoder.layers.{i}.'; s=f'layer{i}'
            n=self.ln(x,key+'layer_norm1.',s+'_ln1')
            q=self.linear(n,key+'self_attn.q_proj.',s+'_q')
            k=self.linear(n,key+'self_attn.k_proj.',s+'_k')
            v=self.linear(n,key+'self_attn.v_proj.',s+'_v')
            q=self.node('Transpose',[self.reshape(q,[batch,tokens,heads,hd],s+'_qr')],s+'_qt',perm=[0,2,1,3])
            k=self.node('Transpose',[self.reshape(k,[batch,tokens,heads,hd],s+'_kr')],s+'_kt',perm=[0,2,3,1])
            v=self.node('Transpose',[self.reshape(v,[batch,tokens,heads,hd],s+'_vr')],s+'_vt',perm=[0,2,1,3])
            scores=self.node('MatMul',[q,k],s+'_qk')
            scores=self.node('Mul',[scores,self.const(hd**-.5,s+'_scale')],s+'_scaled_scores')
            a=self.node('Softmax',[scores],s+'_softmax',axis=-1)
            a=self.node('MatMul',[a,v],s+'_av')
            a=self.node('Transpose',[a],s+'_at',perm=[0,2,1,3])
            a=self.reshape(a,[batch,tokens,d],s+'_ar')
            a=self.linear(a,key+'self_attn.out_proj.',s+'_out')
            x=self.node('Add',[x,a],s+'_attention_residual')
            n=self.ln(x,key+'layer_norm2.',s+'_ln2')
            n=self.linear(n,key+'mlp.fc1.',s+'_fc1')
            n=self.gelu(n,s+'_gelu')
            n=self.linear(n,key+'mlp.fc2.',s+'_fc2')
            x=self.node('Add',[x,n],s+'_mlp_residual')
        x=self.ln(x,self.v+'post_layernorm.','post_layernorm')
        x=self.linear(x,self.p,'projector')
        self.node('Identity',[x],'features_output',output='features')
        self.stream.close()
        input_info=h.make_tensor_value_info('patches',T.FLOAT,[batch,tokens,3*patch*patch]) if patchify_input else h.make_tensor_value_info('pixels',T.FLOAT,[batch,3,side,side])
        graph=h.make_graph(self.nodes,'real_pi05_siglip_and_projector',
            [input_info],
            [h.make_tensor_value_info('features',T.FLOAT,[batch,tokens,2048])],self.initializers)
        model=h.make_model(graph,opset_imports=[h.make_opsetid('',17)],producer_name='pi05_real_weight_export')
        model.ir_version=9
        onnx.save_model(model,str(self.output))
        onnx.checker.check_model(str(self.output),full_check=True)
        wanted={k for k in self.source.keys() if k.startswith(self.v) or k.startswith(self.p)}
        assert self.used==wanted, sorted(wanted-self.used)
        return dict(source=str(BASE/'model.safetensors'),random_weights=False,config=c,patchify_input=patchify_input,
                    source_tensors=len(self.used),batch=batch,onnx=str(self.output),
                    external_data=str(self.data),external_bytes=self.data.stat().st_size,
                    node_count=len(self.nodes),operations=dict(Counter(n.op_type for n in self.nodes)),
                    tensor_mapping=self.weights,checker='passed')

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--batch',type=int,default=1)
    ap.add_argument('--output',type=Path,default=ROOT/'models/vision_batch1.onnx')
    ap.add_argument('--patchify-input',action='store_true')
    args=ap.parse_args(); args.output.parent.mkdir(parents=True,exist_ok=True)
    c=json.loads((ROOT/'results/reference/vision_config.json').read_text())
    started=time.perf_counter()
    report=Exporter(args.output).build(c,args.batch,args.patchify_input)
    report['export_seconds']=time.perf_counter()-started
    write_json(args.output.with_suffix('.manifest.json'),report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('tensor_mapping','config')},indent=2))
