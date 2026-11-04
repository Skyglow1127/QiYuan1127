#!/usr/bin/env python3
"""Wait for this one cloud job and collect its separate result; never deploy.

Uses the existing interactive SSH master socket; contains no password. The
bounded watcher can exit without affecting the cloud tmux training process.
"""
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time

HERE = Path(__file__).resolve().parent
HOST = os.environ['ACT_CLOUD_HOST']
PORT = os.environ.get('ACT_CLOUD_PORT', '22')
SOCKET = os.environ.get('ACT_CLOUD_SOCKET', '/tmp/piper-act-cloud-v6/ssh.sock')
SSH = ['ssh','-S',SOCKET,'-T','-p',PORT,'-o','BatchMode=yes',
       '-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=8',HOST]
SCP = ['scp','-q','-P',PORT,'-o','ControlPath='+SOCKET,
       '-o','BatchMode=yes','-o','StrictHostKeyChecking=yes']


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def snapshot(plan):
    code = '''from pathlib import Path
import json,re
job=Path(JOB_DIR)
status=json.loads((job/'status.json').read_text()) if (job/'status.json').exists() else {'state':'pending'}
log=job/'train.log'
lines=log.read_text(errors='replace').splitlines()[-100:] if log.exists() else []
metrics=[line for line in lines if re.search(r'step:|loss:',line)]
print(json.dumps({'status':status,'latest_metric':metrics[-1] if metrics else None,'last_log_lines':lines[-8:]}))
'''.replace('JOB_DIR',repr(plan['cloud_job_dir']))
    proc = subprocess.run(SSH+['/root/autodl-tmp/lerobot_act/.venv/bin/python','-'],
        input=code,text=True,capture_output=True,timeout=30)
    if proc.returncode:
        raise RuntimeError(proc.stderr.strip()[-1500:])
    return json.loads(proc.stdout)


def collect(plan, state):
    cloud = state['status']
    expected = plan['output_dir']+'/checkpoints/010000/pretrained_model'
    if (cloud['final_checkpoint'] != expected or cloud['final_step'] != 10000
            or cloud['parent_files_unchanged'] is not True
            or cloud['parent_sha256'] != plan['parent_weights_sha256']):
        raise RuntimeError('Unexpected final checkpoint or changed cloud parent; refusing collection')
    target = HERE/'result'
    if target.exists():
        if (not (target/'collection_manifest.json').exists()
                or sha(target/'pretrained_model/model.safetensors') != cloud['final_model_sha256']):
            raise FileExistsError('An unverified result already exists; no overwrite')
        return str(target/'pretrained_model')
    with tempfile.TemporaryDirectory(prefix='.round2_download_',dir=HERE) as temporary:
        temp = Path(temporary)
        subprocess.run(SCP+['-r',HOST+':'+expected,str(temp)],check=True,timeout=300)
        model = temp/'pretrained_model'
        assert sha(model/'model.safetensors') == cloud['final_model_sha256']
        config = json.loads((model/'config.json').read_text())
        assert config['type'] == 'act' and config['chunk_size'] == 100
        assert config['input_features']['observation.state']['shape'] == [7]
        assert config['output_features']['action']['shape'] == [7]
        for camera in ('global','wrist'):
            assert config['input_features']['observation.images.'+camera]['shape'] == [3,480,640]
        assert cloud['final_model_tensors_finite'] == 234
        parent_sha = sha(Path(plan['parent_local_checkpoint'])/'model.safetensors')
        assert parent_sha == plan['parent_weights_sha256']
        (temp/'collection_manifest.json').write_text(json.dumps({
            'cloud_status':cloud,'files_sha256':{p.name:sha(p) for p in model.iterdir() if p.is_file()},
            'original_local_model_sha256_verified':parent_sha,'automatic_deploy':False},indent=2)+'\n')
        # Directory rename publishes only a complete, verified independent result.
        shutil.move(str(temp),str(target))
    for name in ('status.json','preflight.json','train.log'):
        try:
            subprocess.run(SCP+[HOST+':'+plan['cloud_job_dir']+'/'+name,str(HERE/('cloud_final_'+name))],
                check=True,timeout=90)
        except (subprocess.SubprocessError,OSError) as exc:
            (HERE/('cloud_final_'+name+'.download_error')).write_text(str(exc)+'\n')
    return str(target/'pretrained_model')


def main():
    plan = json.loads((HERE/'plan.json').read_text())
    with (HERE/'collector.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        deadline = time.monotonic()+3*3600
        failures = 0
        while time.monotonic() < deadline:
            try:
                state = snapshot(plan)
                state['observed_at'] = datetime.now(timezone.utc).isoformat()
                state['local_collector_state'] = 'watching'
                failures = 0
                if state['status']['state'] == 'complete':
                    state['local_result'] = collect(plan,state)
                    state['local_collector_state'] = 'downloaded_and_verified'
                elif state['status']['state'] == 'failed':
                    state['local_collector_state'] = 'training_failed'
                (HERE/'latest_cloud_status.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
                print(json.dumps({'cloud_state':state['status']['state'],
                    'collector':state['local_collector_state'],'metric':state['latest_metric']},ensure_ascii=False),flush=True)
                if state['local_collector_state'] != 'watching':
                    return
            except Exception as exc:
                failures += 1
                state = {'local_collector_state':'connection_or_collection_error','error':str(exc),
                    'observed_at':datetime.now(timezone.utc).isoformat(),'consecutive_failures':failures,
                    'cloud_training_stopped_by_collector':False}
                (HERE/'latest_cloud_status.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
                print(json.dumps(state,ensure_ascii=False),flush=True)
                if failures >= 3:
                    raise
            time.sleep(45)
        (HERE/'latest_cloud_status.json').write_text(json.dumps({
            'local_collector_state':'watch_timeout','cloud_training_stopped_by_collector':False},indent=2)+'\n')


if __name__ == '__main__':
    main()
