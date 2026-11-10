"""Local Qwen -> validated TaskSpec -> stage routing. Saved RGB only; no robot."""
import argparse
import json
from pathlib import Path
from PIL import Image
from scripts.qwen_llama import create_llama_runtime
from .routing import TASKSPEC_PROMPT,validate_task_spec,route
from .prepare import sha

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--image',type=Path,required=True)
    p.add_argument('--instruction',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--device',choices=['auto','cpu','vulkan'],default='auto')
    p.add_argument('--model',type=Path,default=ROOT/'models/qwen3.5-2b-gguf/model-q8_0.gguf')
    a=p.parse_args()
    if not a.instruction.strip() or len(a.instruction)>1000:raise ValueError('invalid task instruction')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    with Image.open(a.image) as im:image=im.convert('RGB')
    prompt=TASKSPEC_PROMPT+'\nRequested task: '+a.instruction+'\nThis is an offline saved image. Output only the requested JSON, no Markdown.'
    with create_llama_runtime(a.model,threads=4,device=a.device,strict_json=False,log_path=out/'qwen.log') as runtime:
        text,timing=runtime.infer(image,prompt,max_tokens=160)
    (out/'raw_response.txt').write_text(text+'\n')
    if timing.get('generation_hit_token_limit'):raise ValueError('incomplete Qwen response')
    spec=validate_task_spec(text)
    report=dict(status='offline_task_spec_validated',task_spec=spec,
        routes=[route(spec,stage) for stage in ('approach','close','lift','lower','open','retreat')],
        image=str(a.image.resolve()),image_sha256=sha(a.image),model=str(a.model.resolve()),
        instruction=a.instruction,timing=timing,robot_commands=0,grasp_executed=False,
        qwen_finetuned=False,target_geometry_validated=False)
    (out/'task_spec.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'task_spec':spec,'runtime':timing['runtime'],'robot_commands':0},ensure_ascii=False))
if __name__=='__main__':main()
