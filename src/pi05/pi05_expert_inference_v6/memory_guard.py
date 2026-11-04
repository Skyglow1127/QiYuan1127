"""Launch one owned offline process with a system-memory and time limit."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

def available():
    for row in Path('/proc/meminfo').read_text().splitlines():
        if row.startswith('MemAvailable:'):
            return int(row.split()[1])*1024
    raise RuntimeError('No MemAvailable')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log',required=True,type=Path)
    p.add_argument('--receipt',required=True,type=Path)
    p.add_argument('--timeout',type=float,default=600)
    p.add_argument('--minimum-gib',type=float,default=1.25)
    p.add_argument('command',nargs=argparse.REMAINDER)
    a=p.parse_args();cmd=a.command[1:] if a.command[:1]==['--'] else a.command
    if not cmd:raise ValueError('Missing command')
    if a.log.exists() or a.receipt.exists():raise FileExistsError('Choose fresh log and receipt paths')
    a.log.parent.mkdir(parents=True,exist_ok=True)
    start=time.monotonic();minimum=available();reason=None
    with a.log.open('x') as log:
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL,start_new_session=True)
        print(json.dumps({'child_pid':child.pid,'log':str(a.log)},ensure_ascii=False),flush=True)
        while child.poll() is None:
            minimum=min(minimum,available())
            if minimum<a.minimum_gib*2**30:reason='system_memory_floor'
            if time.monotonic()-start>a.timeout:reason='time_limit'
            if reason:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()
                break
            time.sleep(.5)
        code=child.wait()
    result={'exit_code':code,'stop_reason':reason,'elapsed_seconds':time.monotonic()-start,
            'minimum_available_memory_bytes':minimum,'command':cmd,'child_pid':child.pid}
    a.receipt.parent.mkdir(parents=True,exist_ok=True)
    a.receipt.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if code==0 and reason is None else 1

if __name__=='__main__':raise SystemExit(main())
