"""One-command local Qwen/SAM RGB-D inspection, optionally acquiring a new KR260 capture."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def latest_recording():
    recordings = []
    for summary in (ROOT/'output/ppp_checks').glob('*/summary.json'):
        try:
            if json.loads(summary.read_text())['status'] != 'completed': continue
            pairs = sorted(summary.parent.glob('replay_*/snapshot.yml'))
            if pairs: recordings.append((summary.stat().st_mtime, pairs[-1]))
        except (OSError, ValueError, KeyError): continue
    direct = ROOT/'output/last_session.yml'
    if direct.is_file(): recordings.append((direct.stat().st_mtime, direct))
    if not recordings: raise FileNotFoundError('No completed recording. Use --capture or --replay FILE.')
    return max(recordings, key=lambda item:item[0])[1]


def main():
    started = time.perf_counter()
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group()
    source.add_argument('--capture', action='store_true', help='Capture 20 seconds on KR260 over the configured USB PPP')
    source.add_argument('--replay', type=Path, help='Saved RGB-D snapshot; otherwise select the latest completed local recording')
    p.add_argument('--target', help='Optional natural-language target, e.g. 红色方块')
    p.add_argument('--config', type=Path, default=ROOT/'configs/kr260_tabletop.yml')
    p.add_argument('--output', type=Path)
    p.add_argument('--profile', choices=['balanced','review','small','reference'], default='balanced')
    p.add_argument('--device', choices=['auto','cpu','vulkan'], default='auto')
    a = p.parse_args()
    output = a.output or ROOT/'output/qwen_grasp'/datetime.now().strftime('%Y%m%d_%H%M%S_')/uuid.uuid4().hex[:8]
    if output.exists(): p.error('Output must be a new directory')
    if a.capture:
        # Capture has bounded timeouts and its own durable logs. No robot command is sent.
        process = subprocess.Popen([sys.executable, str(ROOT/'scripts/check_kr260.py'), '--seconds','20','--snapshots','3',
                        '--config',str(a.config.resolve())], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        summary = None
        for line in process.stdout:
            print(line, end='', flush=True)
            if line.startswith('SUMMARY '): summary = Path(line[8:].strip())
        if process.wait() != 0: raise RuntimeError('New KR260 capture failed; no old observation was substituted')
        if summary is None or json.loads(summary.read_text())['status'] != 'completed':
            raise RuntimeError('Capture did not produce a completed summary')
        replay = sorted(summary.parent.glob('replay_*/snapshot.yml'))[-1]
    else:
        replay = a.replay.resolve() if a.replay else latest_recording()
    print('RECORDED RGB-D:', replay, flush=True)
    print('Qwen/SAM analysis is offline; output is not a robot motion command.', flush=True)
    command = [sys.executable, str(ROOT/'scripts/qwen_scene.py'), '--replay',str(replay),
               '--config',str(a.config.resolve()), '--output',str(output.resolve()), '--device',a.device]
    if a.profile == 'balanced':
        command += ['--backend','llamacpp','--model',str(ROOT/'models/qwen3.5-2b-gguf/model-q8_0.gguf')]
    elif a.profile == 'review':
        command += ['--backend','llamacpp','--model',str(ROOT/'models/qwen3.5-4b-gguf/model-q5_k_m.gguf')]
    else:
        command += ['--backend','transformers','--model',str(ROOT/'models'/('Qwen3.5-0.8B' if a.profile=='small' else 'Qwen3.5-2B'))]
    if a.target: command += ['--target',a.target]
    subprocess.run(command, check=True)
    actual = json.loads((output/'scene_result.json').read_text())['qwen_timing']['runtime']
    (output/'run_report.json').write_text(json.dumps({'profile':a.profile,'requested_device':a.device,'actual_runtime':actual,'replay':str(replay.resolve()),
        'new_capture_requested':a.capture,'total_wall_s':time.perf_counter()-started,
        'motion_command_ready':False},indent=2)+'\n')
    print('RESULT',output.resolve()/'scene_result.json',flush=True)
    print('PREVIEW',output.resolve()/'scene_annotated.png',flush=True)


if __name__ == '__main__': main()
