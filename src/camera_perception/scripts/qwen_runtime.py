"""Reusable local Qwen3.5 CPU runtime; no network inference and no robot control."""
import os
from pathlib import Path
import time
from grasp_semantics import SCENE_PROMPT

os.environ.setdefault('HF_HUB_DISABLE_TELEMETRY','1')
os.environ.setdefault('TOKENIZERS_PARALLELISM','false')


class QwenRuntime:
    def __init__(self,model_path,threads=4,dtype='bfloat16',max_edge=640):
        if threads<1 or threads>24 or dtype not in ('float32','bfloat16') or not 256<=max_edge<=1280:
            raise ValueError('Invalid CPU inference configuration')
        import torch
        from transformers import AutoModelForImageTextToText,AutoProcessor
        self.torch=torch;self.path=Path(model_path).resolve();self.threads=threads;self.dtype=dtype;self.max_edge=max_edge
        torch.set_num_threads(threads)
        try:torch.set_num_interop_threads(1)
        except RuntimeError:pass
        started=time.perf_counter()
        self.processor=AutoProcessor.from_pretrained(str(self.path),local_files_only=True)
        self.model=AutoModelForImageTextToText.from_pretrained(str(self.path),local_files_only=True,
            dtype=getattr(torch,dtype),attn_implementation='sdpa').eval()
        self.load_ms=1000*(time.perf_counter()-started)

    def infer(self,image,prompt,max_tokens=160):
        if not isinstance(prompt,str) or not prompt.strip() or len(prompt)>8000 or not 16<=max_tokens<=512:
            raise ValueError('Invalid bounded inference request')
        view=image.convert('RGB').copy();view.thumbnail((self.max_edge,self.max_edge))
        messages=[{'role':'user','content':[{'type':'image','image':view},{'type':'text','text':prompt}]}]
        started=time.perf_counter()
        inputs=self.processor.apply_chat_template(messages,tokenize=True,add_generation_prompt=True,
            return_dict=True,return_tensors='pt',enable_thinking=False)
        # Preserve integer token IDs, grid sizes and masks while matching vision float dtype.
        inputs={k:v.to(getattr(self.torch,self.dtype)) if self.torch.is_tensor(v) and v.is_floating_point() else v
                for k,v in inputs.items()}
        prepared=time.perf_counter()
        with self.torch.inference_mode():
            result=self.model.generate(**inputs,max_new_tokens=max_tokens,do_sample=False,max_time=90)
        tokens=result[0,inputs['input_ids'].shape[-1]:]
        raw=self.processor.decode(tokens,skip_special_tokens=True).strip()
        timing={'model_load_ms':self.load_ms,'preprocess_ms':1000*(prepared-started),
            'grounding_ms':1000*(time.perf_counter()-started),'generated_tokens':len(tokens),
            'input_tokens':inputs['input_ids'].shape[-1],'input_image_size':list(view.size),
            'runtime':'transformers CPU '+self.dtype,'threads':self.threads,'max_edge':self.max_edge,
            'generation_hit_token_limit':len(tokens)>=max_tokens,
            'generation_hit_time_limit':time.perf_counter()-prepared>=90}
        return raw,timing


def target_prompt(target):
    return ('Locate the single object described below. If missing or ambiguous, return []. '
            'Otherwise return ONLY a JSON list with one object: '
            '[{"bbox_2d":[x1,y1,x2,y2],"label":"target"}]. '
            'Coordinates are normalized to 0..1000 relative to the entire image. '
            'Use a tight axis-aligned bounding box. Do not explain. Target: '+target)


INVENTORY_PROMPT=SCENE_PROMPT
