"""Watch a saved RGB-D snapshot without blocking the camera producer.

Only the newest observation is analyzed when a job finishes; intermediate frames
are not queued. This watches files, not a live robot pose. A result never becomes
a motion command even when it still matches the watched snapshot.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import time

import cv2

from qwen_scene import ROOT, SamRefiner, analyze_scene, save_json


def fingerprint(path):
    path = Path(path).resolve()
    if not path.exists(): return None
    metadata = path.read_bytes()
    f = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not f.isOpened(): return None
    try:
        names = [f.getNode(key).string() for key in ('color','depth')]
        if not all(names): return None
        images = [path.parent/name for name in names]
        hashes = [hashlib.sha256(metadata).hexdigest()] + [hashlib.sha256(p.read_bytes()).hexdigest() for p in images]
        if path.read_bytes() != metadata: return None
        return hashes
    finally: f.release()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--config', type=Path, default=ROOT/'configs/kr260_tabletop.yml')
    p.add_argument('--model', type=Path, default=ROOT/'models/qwen3.5-2b-gguf/model-q8_0.gguf')
    p.add_argument('--seconds', type=float, default=120)
    p.add_argument('--threads', type=int, default=4)
    p.add_argument('--device', choices=['auto','cpu','vulkan'], default='auto')
    p.add_argument('--max-model-jobs', type=int, default=12, help='Reload owned Qwen process after this many observations to bound retained memory')
    p.add_argument('--target')
    a = p.parse_args()
    if a.output.exists() or not 1<=a.seconds<=7200 or not 1<=a.threads<=24 or not 1<=a.max_model_jobs<=128:
        p.error('Output must be new; seconds 1..7200; threads 1..24; max-model-jobs 1..128')
    a.output.mkdir(parents=True)
    pointer = a.output/'latest_result.json'
    baseline = {'offline_replay': True, 'motion_command_ready': False,
                'scope': 'saved observation only; matching a file does not prove current camera or robot state'}
    state = dict(baseline, status='initializing', current_snapshot_matches=False)
    save_json(pointer, state)
    from qwen_llama import create_llama_runtime
    last = None; jobs = 0
    try:
        with ExitStack() as runtime_stack:
            runtime = runtime_stack.enter_context(create_llama_runtime(a.model, a.threads, device=a.device, log_path=a.output/'runtime.log'))
            sam_path = ROOT/'models/sam2.1_t.pt'
            refiner = SamRefiner(sam_path, a.threads)
            deadline = time.monotonic()+a.seconds
            while time.monotonic()<deadline:
                try: current = fingerprint(a.replay)
                except (OSError, cv2.error): current = None
                if current is None or current == last:
                    time.sleep(.25)
                    continue
                if jobs and jobs%a.max_model_jobs==0:
                    state.update(status='reloading_model',current_snapshot_matches=False)
                    save_json(pointer,state)
                    runtime_stack.close()
                    runtime = runtime_stack.enter_context(create_llama_runtime(a.model,a.threads,device=a.device,
                        log_path=a.output/f'runtime_{jobs//a.max_model_jobs:03d}.log'))
                last = current; jobs += 1
                directory = a.output/f'job_{jobs:04d}'
                state = dict(baseline, status='processing', current_snapshot_matches=False,
                             source_fingerprint=current, result_directory=str(directory.resolve()),
                             model_generation=(jobs-1)//a.max_model_jobs)
                save_json(pointer, state)
                try:
                    report = analyze_scene(a.replay, a.config, directory, runtime, sam_path,
                                           target=a.target, threads=a.threads, sam_refiner=refiner)
                    after = fingerprint(a.replay)
                    # Compare the files actually frozen by analyze_scene, including the
                    # original source hashes, with the original observation we requested.
                    captured = list(report['provenance']['original_sha256'].values())
                    matches = current == after == captured
                    state.update(status='completed' if matches else 'superseded', current_snapshot_matches=matches,
                                 result_status=report['status'], completed_unix=time.time())
                except Exception as error:
                    state.update(status='failed', error=str(error), current_snapshot_matches=False)
                save_json(pointer, state)
                with (a.output/'events.jsonl').open('a') as log: log.write(json.dumps(state,ensure_ascii=False)+'\n')
                print(state['status'], directory, flush=True)
    except Exception as error:
        state.update(status='failed',error=str(error),current_snapshot_matches=False)
        raise
    finally:
        state.update(last_job_status=state['status'],status='stopped', current_snapshot_matches=False, jobs_completed_or_failed=jobs)
        save_json(pointer, state)


if __name__ == '__main__': main()
