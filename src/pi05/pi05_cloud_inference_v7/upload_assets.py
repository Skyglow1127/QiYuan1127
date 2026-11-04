"""Resumable upload of the existing complete B policy; no credentials stored."""
from pathlib import Path
import json
import shlex
import subprocess
import time

HERE=Path(__file__).resolve().parent
OUT=HERE.parent
SSH=['ssh','-S','/tmp/pi05-newcloud-v7.sock','-p','18561','-o','BatchMode=yes','-o','ProxyCommand=false','-o','ConnectTimeout=8','-T']
TARGET='root@192.0.2.10'
GROUPS=[('model',OUT/'piper_pi05_expert_cloud_backup_v6/pi05_expert_only_v6_r09/best_model','/root/autodl-tmp/pi05_models/expert_b8_best004000'),('tokenizer',OUT/'piper_pi05_cloud_backup_v5/models/paligemma_tokenizer','/root/autodl-tmp/pi05_models/paligemma_tokenizer')]

def main():
    state={'started_unix':time.time(),'groups':[],'finished':False}
    path=HERE/'upload_status.json'
    for label,source,destination in GROUPS:
        files=sorted(p.name for p in source.iterdir() if p.is_file())
        listing=HERE/(label+'_upload.files')
        listing.write_bytes(b''.join(n.encode()+b'\0' for n in files))
        command=['rsync','-rtz','--compress-level=1','--partial','--partial-dir=.rsync-partial','--protect-args','--from0','--files-from='+str(listing),'--info=progress2','--rsh='+shlex.join(SSH),str(source)+'/',TARGET+':'+destination+'/']
        state.update(stage=label,source=str(source),destination=destination,bytes=sum((source/n).stat().st_size for n in files))
        path.write_text(json.dumps(state,indent=2)+'\n')
        with (HERE/'upload_progress.log').open('ab') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode:
            state.update(error_code=result.returncode);path.write_text(json.dumps(state,indent=2)+'\n')
            raise SystemExit(result.returncode)
        state['groups'].append({'label':label,'files':files,'bytes':state['bytes'],'rsync_complete':True})
    state.update(stage='complete',finished=True,finished_unix=time.time())
    path.write_text(json.dumps(state,indent=2)+'\n')
    print(json.dumps(state))

if __name__=='__main__':main()
