#!/usr/bin/env python3
"""Start/reuse two serial-bound local RGB/depth previews; no robot API calls."""
import json
import os
from pathlib import Path
import socket
import signal
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
PREVIEW=ROOT/'camera_preview'
LOGS=ROOT.parents[1]/'logs/cameras'
CAMERAS=[{'role':'wrist','port':18760,'serial':'CC1365301EY'},
         {'role':'global','port':18761,'serial':'CC1365302B2'}]
OPENER=urllib.request.build_opener(urllib.request.ProxyHandler({}))


def probe(camera):
    base=f'http://127.0.0.1:{camera["port"]}'
    with OPENER.open(base+'/status',timeout=2) as response: status=json.load(response)
    if status.get('role')!=camera['role'] or status.get('device',{}).get('serial')!=camera['serial']:
        raise RuntimeError('相机身份与预期绑定不符')
    if status.get('error') or status.get('stopped'): raise RuntimeError(str(status.get('error') or 'camera stopped'))
    frames=[]
    for number in range(2):
        if number: time.sleep(.15)
        start=time.monotonic()
        with OPENER.open(base+'/color.jpg',timeout=2) as response:
            raw=response.read()
            index=int(response.headers['X-Frame-Index'])
            age=float(response.headers['X-Board-Age-Ms'])+(time.monotonic()-start)*1000
        image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape!=(480,640,3) or age>250:
            raise RuntimeError('图像尺寸不符或采集帧已过期')
        frames.append(index)
        if number: (LOGS/(camera['role']+'.jpg')).write_bytes(raw)
    if frames[1]<=frames[0]: raise RuntimeError('相机帧没有更新')
    for kind in ['color','depth']:
        if status.get('streams',{}).get(kind,{}).get('age_ms',1e9)>250:
            raise RuntimeError(kind+'数据未就绪')
    return {'ready':True,'serial':camera['serial'],'url':base,'frame_indices':frames,'status':status}


def start(camera):
    try: return probe(camera)
    except Exception: pass
    # Recover only a stopped service created by this launcher, after checking
    # its current PID command line. Never terminate an unrelated port owner.
    process_file=LOGS/(camera['role']+'_process.json')
    try:
        saved=json.loads(process_file.read_text())
        command=Path(f'/proc/{saved["pid"]}/cmdline').read_bytes().split(b'\0')
        actual=[item.decode() for item in command if item]
        if actual==saved['argv'] and str(PREVIEW/'camera_server.py') in actual:
            with OPENER.open(f'http://127.0.0.1:{camera["port"]}/status',timeout=2) as response:
                status=json.load(response)
            if (status.get('role')==camera['role'] and
                status.get('device',{}).get('serial')==camera['serial'] and
                (status.get('stopped') or status.get('error'))):
                os.kill(saved['pid'],signal.SIGTERM)
                deadline=time.monotonic()+3
                while time.monotonic()<deadline and Path(f'/proc/{saved["pid"]}').exists():
                    time.sleep(.1)
    except (OSError,ValueError,KeyError): pass
    try:
        with socket.create_connection(('127.0.0.1',camera['port']),timeout=.3):
            raise RuntimeError(f'端口{camera["port"]}被未通过验证的服务占用，未终止该服务')
    except ConnectionRefusedError: pass
    env=dict(os.environ,LD_LIBRARY_PATH=str(PREVIEW/'lib'))
    argv=[sys.executable,'-u',str(PREVIEW/'camera_server.py'),
          '--sdk-dir',str(PREVIEW/'lib'),'--page',str(PREVIEW/'preview.html'),
          '--roles-file',str(PREVIEW/'camera_roles.json'),
          '--role',camera['role'],'--port',str(camera['port'])]
    logfile=LOGS/(camera['role']+'_server.log')
    with logfile.open('ab',buffering=0) as output:
        proc=subprocess.Popen(argv,cwd=PREVIEW,env=env,stdin=subprocess.DEVNULL,
            stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    (LOGS/(camera['role']+'_process.json')).write_text(json.dumps({'pid':proc.pid,'argv':argv},indent=2)+'\n')
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        if proc.poll() is not None: raise RuntimeError(f'相机服务退出，日志：{logfile}')
        try: return probe(camera)
        except Exception as error: last_error=error
        time.sleep(.3)
    raise RuntimeError(f'{last_error}；日志：{logfile}')


def main():
    LOGS.mkdir(parents=True,exist_ok=True)
    report={'completed':False,'connection':'local USB on AIPC','robot_commands_sent':0,'cameras':{}}
    for camera in CAMERAS:
        try: result=start(camera)
        except Exception as error: result={'ready':False,'error':str(error)}
        report['cameras'][camera['role']]=result
        print(json.dumps({camera['role']:result},ensure_ascii=False),flush=True)
    report['completed']=all(c['ready'] for c in report['cameras'].values())
    report['checked_at_unix']=time.time()
    (LOGS/'restored.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return 0 if report['completed'] else 1


if __name__=='__main__': raise SystemExit(main())
