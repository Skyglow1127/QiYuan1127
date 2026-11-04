"""Read only: query this cloud experiment through its existing SSH master."""
from pathlib import Path
import json
import subprocess
import sys

HERE = Path(__file__).resolve().parent
MASTER = HERE.parent / "pi05_cloud_grasp/runtime/cloud.sock"
IDENT = "pi05_expert_only_v6_r09"
REMOTE = r'''
from pathlib import Path
import json, subprocess
root=Path('/root/autodl-tmp/piper_pi05/experiments/pi05_expert_only_v6_r09')
state=json.loads((root/'job_status.json').read_text())
scratch=Path(state['scratch_root'])
audit=scratch/('train' if state['stage'] in ['formal_training','complete'] else 'smoke')/'audit'
steps=audit/'actual_steps.jsonl'
if steps.exists():
    rows=steps.read_text().splitlines()
    if rows:state['latest_optimizer_step']=json.loads(rows[-1])
log=Path(state.get('child_log',''))
if log.is_file():state['log_tail']=log.read_text(errors='replace').replace('\r','\n').splitlines()[-8:]
gpu=subprocess.run(['nvidia-smi','--query-gpu=uuid,utilization.gpu,memory.used,memory.total','--format=csv,noheader'],capture_output=True,text=True)
state['gpu']=gpu.stdout.strip()
for field in ['supervisor_pid','child_pid']:
    pid=state.get(field)
    state[field+'_exists']=bool(pid and Path('/proc',str(pid)).exists())
print(json.dumps(state,ensure_ascii=False,indent=2))
'''

def main():
    cmd = ["ssh", "-o", "ControlPath=" + str(MASTER), "-o", "ProxyCommand=false",
           "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-p", "29596",
           "root@116.172.66.186", "/root/autodl-tmp/piper_pi05/.venv/bin/python -"]
    result = subprocess.run(cmd, input=REMOTE, text=True, capture_output=True, timeout=45)
    if result.returncode:
        print("无法通过现有 SSH 会话读取状态：" + result.stderr.strip(), file=sys.stderr)
        return result.returncode
    state = json.loads(result.stdout)
    if state.get("experiment_id") != IDENT:
        raise RuntimeError("Unexpected experiment identity")
    temporary = HERE / "latest_cloud_status.json.tmp"
    temporary.write_text(json.dumps(state,ensure_ascii=False,indent=2)+"\n")
    temporary.replace(HERE / "latest_cloud_status.json")
    print(json.dumps(state,ensure_ascii=False,indent=2))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
