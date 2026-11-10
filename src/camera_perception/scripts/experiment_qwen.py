"""Reproducible bounded Qwen experiments retaining prompts, outputs and provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import resource
import time

from PIL import Image
from qwen_runtime import QwenRuntime, target_prompt, INVENTORY_PROMPT


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases', type=Path, required=True)
    p.add_argument('--model', type=Path, default=Path('models/Qwen3.5-0.8B'))
    p.add_argument('--dtype', choices=['float32','bfloat16'], default='bfloat16')
    p.add_argument('--threads', type=int, default=4)
    p.add_argument('--max-edge', type=int, default=640)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--backend', choices=['transformers','llamacpp'], default='transformers')
    p.add_argument('--strict-json', action='store_true')
    p.add_argument('--mmproj', type=Path)
    a = p.parse_args()
    cases = json.loads(a.cases.read_text())
    if not isinstance(cases,list) or not 1<=len(cases)<=500:
        p.error('Expected 1..500 cases')
    ids = [case.get('id','') for case in cases]
    if len(set(ids))!=len(ids) or any(not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',x) for x in ids):
        p.error('Case IDs must be unique simple filenames')
    a.output.mkdir(parents=True, exist_ok=False)
    if a.backend == 'llamacpp':
        from qwen_llama import LlamaRuntime
        engine = LlamaRuntime(a.model, a.threads, a.max_edge, mmproj=a.mmproj,
                              log_path=a.output/'runtime.log', strict_json=a.strict_json)
    else:
        engine = QwenRuntime(a.model, a.threads, a.dtype, a.max_edge)
    results = []
    try:
        for case in cases:
            print('CASE', case['id'], flush=True)
            started = time.perf_counter()
            row = {'case':case, 'model':str(a.model.resolve()), 'simulated':False}
            try:
                image_path = Path(case['image']).resolve()
                with Image.open(image_path) as source: image = source.convert('RGB')
                row['image_sha256'] = hashlib.sha256(image_path.read_bytes()).hexdigest()
                if case.get('crop'): image = image.crop(case['crop'])
                if case.get('resize'): image = image.resize(tuple(case['resize']))
                prompt = case.get('prompt') or (INVENTORY_PROMPT if case.get('task')=='inventory' else target_prompt(case['target']))
                row['prompt'] = prompt
                if hasattr(engine,'object_limit'): engine.object_limit = 1 if case.get('target') else 6
                raw, timing = engine.infer(image, prompt, case.get('max_tokens',160))
                row.update(raw_response=raw, timing=timing)
                print(raw, flush=True)
            except Exception as error:
                row['error'] = str(error)
                print('ERROR', error, flush=True)
            row['elapsed_s'] = time.perf_counter()-started
            row['peak_rss_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            row['peak_rss_scope'] = 'Python process only; excludes llama.cpp child when used'
            results.append(row)
            (a.output/(case['id']+'.json')).write_text(json.dumps(row, indent=2, ensure_ascii=False)+'\n')
            (a.output/'results.json').write_text(json.dumps(results, indent=2, ensure_ascii=False)+'\n')
    finally:
        if hasattr(engine,'close'): engine.close()
    print('DONE', a.output, flush=True)
    if any('error' in row for row in results): raise SystemExit(1)


if __name__ == '__main__': main()
