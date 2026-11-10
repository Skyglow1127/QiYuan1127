#!/usr/bin/env bash
set -eu
cd /home/aup/NPU/manual_demo_app
exec python3 - <<'PY'
import json, pathlib, subprocess, sys, time, urllib.error, urllib.request, webbrowser
root=pathlib.Path('/home/aup/NPU/manual_demo_app')
url='http://127.0.0.1:18771'
def ready():
    try:
        with urllib.request.urlopen(url+'/api/status',timeout=2) as response:
            status=json.load(response)
        if status.get('schema')!='d1-workstation-v1':
            raise RuntimeError('端口18771被其他程序占用，未尝试终止该程序。')
        return True
    except (urllib.error.URLError,TimeoutError): return False
if not ready():
    with (root/'app.log').open('a') as log:
        process=subprocess.Popen([sys.executable,'-u',str(root/'app.py')],cwd=root,
            stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for _ in range(40):
        if ready(): break
        if process.poll() is not None: raise RuntimeError('启动失败，请查看 '+str(root/'app.log'))
        time.sleep(.2)
    else: raise RuntimeError('等待工作台启动超时。')
print(url)
webbrowser.open(url)
PY
