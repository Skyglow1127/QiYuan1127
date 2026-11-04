"""Start/reuse the two existing KR260 camera services and loopback SSH forwards.

No robot SDK, motion, power, enable, calibration or inference operation.
Camera servers and SSH forwards survive the GUI and this short-lived helper.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time
import urllib.request

import cv2
import numpy as np

CAMERAS = (
    dict(role='wrist', label='腕部相机', port=18760, serial='CC1365301EY'),
    dict(role='global', label='全局相机', port=18761, serial='CC1365302K0'),
)
HOST = 'ubuntu@10.42.0.55'
KEY = Path.home()/'.ssh/id_ed25519_kr260'
SERVER = '/home/ubuntu/kr260-camera/kr260_live_server.py'
READY_TIMEOUT_S = 8.
STOP_TIMEOUT_S = 5.

# Runs only the archived camera server. Reuse healthy servers. Recovery only
# terminates a python camera process with the exact script, role and port.
REMOTE_COMMON = r'''
import json, os, pathlib, signal, socket, subprocess, sys, time, urllib.request
role, port, serial, server = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
def healthy():
    try:
        with op.open('http://127.0.0.1:%d/status'%port, timeout=1) as f: s=json.load(f)
        return (s.get('device',{}).get('serial')==serial and s.get('role')==role
                and not s.get('error') and s.get('stopped') is False
                and 0<=s.get('streams',{}).get('color',{}).get('age_ms',1e9)<250)
    except Exception: return False
def occupied():
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=.3): return True
    except OSError: return False
def matching():
    found=[]
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            a=(p/'cmdline').read_bytes().decode().strip('\0').split('\0')
            if (p.stat().st_uid==os.getuid() and a and 'python' in pathlib.Path(a[0]).name
                    and server in a and '--role' in a and '--port' in a
                    and a[a.index('--role')+1]==role and a[a.index('--port')+1]==str(port)):
                found.append(int(p.name))
        except (OSError,ValueError,IndexError,UnicodeError): pass
    return found
'''
REMOTE_ENSURE = REMOTE_COMMON + r'''
action='reused'
if not healthy():
    if not pathlib.Path(server).is_file(): raise RuntimeError('相机服务文件不存在：'+server)
    pids=matching()
    if occupied() and not pids: raise RuntimeError('远端端口已被其他服务占用，未终止未知进程：'+str(port))
    for pid in pids:
        try: os.kill(pid,signal.SIGTERM)
        except ProcessLookupError: pass
    deadline=time.monotonic()+5
    while (matching() or occupied()) and time.monotonic()<deadline: time.sleep(.1)
    if matching() or occupied(): raise RuntimeError('原相机服务未正常退出，未重复启动：'+role)
    env=dict(os.environ,LD_LIBRARY_PATH='/home/ubuntu/kr260-camera/lib')
    logfile='/tmp/d1-%s-camera-service.log'%role
    with open(logfile,'ab',buffering=0) as log:
        proc=subprocess.Popen(['flock','-n','/tmp/d1-%s-camera.lock'%role,
            'python3','-u',server,'--role',role,'--port',str(port)],
            cwd='/tmp',env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    action='restarted' if pids else 'started'
    deadline=time.monotonic()+12
    while not healthy():
        if proc.poll() is not None: raise RuntimeError('相机服务启动退出；请查看 '+logfile)
        if time.monotonic()>deadline: raise RuntimeError('相机服务未就绪；请查看 '+logfile)
        time.sleep(.2)
print(json.dumps(dict(action=action,role=role,port=port,pids=matching())))
'''
REMOTE_STOP = REMOTE_COMMON + r'''
pids=matching()
if occupied() and not pids:
    raise RuntimeError('远端端口被未知服务占用，未终止该进程：'+str(port))
for pid in pids:
    try: os.kill(pid,signal.SIGTERM)
    except ProcessLookupError: pass
deadline=time.monotonic()+6
while (matching() or occupied()) and time.monotonic()<deadline: time.sleep(.1)
if matching() or occupied(): raise RuntimeError('相机服务尚未正常退出：'+role)
print(json.dumps(dict(stopped=True,action='stopped' if pids else 'already_stopped',role=role,port=port,terminated_pids=pids)))
'''


def ssh_args():
    return ['ssh','-i',str(KEY),'-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
            '-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3']


class CameraServices:
    def __init__(self,folder):
        self.folder=Path(folder)
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def probe(self,camera):
        """Validate camera identity and two advancing, decodable fresh RGB frames."""
        url=f"http://127.0.0.1:{camera['port']}"
        with self.opener.open(url+'/status',timeout=1) as f: status=json.load(f)
        if (status.get('service')!='kr260-live-v1' or status.get('role')!=camera['role']
                or status.get('device',{}).get('serial')!=camera['serial']):
            raise RuntimeError(f"{camera['label']}身份不符，未使用端口 {camera['port']} 上的未知服务")
        if status.get('error') or status.get('stopped') is not False:
            raise RuntimeError(f"{camera['label']}服务异常：{status.get('error') or '采集已停止'}")
        indices=[];ages=[]
        for number in range(2):
            if number: time.sleep(.15)
            began=time.monotonic()
            with self.opener.open(url+'/color.jpg',timeout=1) as f:
                raw=f.read();index=int(f.headers['X-Frame-Index']);age=float(f.headers['X-Board-Age-Ms'])
            age+=(time.monotonic()-began)*1000
            image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape!=(480,640,3):
                raise RuntimeError(f"{camera['label']}未返回有效的 640×480 图像")
            if not 0<=age<=250 or (indices and index<=indices[-1]):
                raise RuntimeError(f"{camera['label']}图像过期或帧号未更新")
            indices.append(index);ages.append(age)
        return dict(ready=True,checked_at=time.time(),serial=camera['serial'],url=url,
                    frame_indices=indices,maximum_frame_age_ms=max(ages),
                    capture_fps=status.get('streams',{}).get('color',{}).get('capture_fps'),status=status)

    def port_open(self,port):
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.3): return True
        except OSError: return False

    def ensure_remote(self,camera):
        return self.remote_request(camera,REMOTE_ENSURE,'启动','_remote.log')

    def remote_request(self,camera,script,action,suffix):
        command='python3 -c '+shlex.quote(script)+' '+shlex.join([
            camera['role'],str(camera['port']),camera['serial'],SERVER])
        argv=ssh_args()+[HOST,command]
        try:
            done=subprocess.run(argv,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,text=True,timeout=26)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError('连接 KR260 或'+action+'相机超时；请检查板卡电源与网络') from error
        (self.folder/(camera['role']+suffix)).write_text(done.stdout+done.stderr)
        if done.returncode:
            detail=(done.stderr.strip() or done.stdout.strip())[-1200:]
            raise RuntimeError('无法'+action+' KR260 相机服务；请检查板卡电源、网络及 SSH 登录。详情：'+detail)
        try: return json.loads(done.stdout.strip().splitlines()[-1])
        except (ValueError,IndexError) as error: raise RuntimeError('KR260 未返回有效的相机启动报告') from error

    def stop_remote(self,camera):
        return self.remote_request(camera,REMOTE_STOP,'关闭','_remote_stop.log')

    def matching_forwards(self,camera):
        expected=f"127.0.0.1:{camera['port']}:127.0.0.1:{camera['port']}"
        found=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:
                args=(entry/'cmdline').read_bytes().decode().strip('\0').split('\0')
                forwards=[args[i+1] for i,arg in enumerate(args[:-1]) if arg=='-L']
                if (entry.stat().st_uid==os.getuid() and args and Path(args[0]).name=='ssh'
                        and HOST in args and forwards==[expected]
                        and ('-N' in args or SERVER in args[-1])):
                    found.append(int(entry.name))
            except (OSError,UnicodeError):pass
        return found

    def stop_forward(self,camera):
        pids=self.matching_forwards(camera)
        for pid in pids:
            try: os.kill(pid,signal.SIGTERM)
            except ProcessLookupError:pass
        deadline=time.monotonic()+STOP_TIMEOUT_S
        while (self.matching_forwards(camera) or self.port_open(camera['port'])) and time.monotonic()<deadline:
            time.sleep(.1)
        if self.matching_forwards(camera) or self.port_open(camera['port']):
            raise RuntimeError(f"本机相机端口 {camera['port']} 仍被占用；未强制终止未知进程")
        return dict(stopped=True,terminated_pids=pids)

    def stop(self):
        self.folder.mkdir(parents=True,exist_ok=True)
        result=dict(completed=False,cameras={},robot_commands_sent=0)
        for camera in CAMERAS:
            remote=None
            try:
                if not KEY.is_file():raise RuntimeError('缺少 KR260 SSH 密钥，无法确认远端相机关闭')
                remote=self.stop_remote(camera)
                if remote.get('stopped') is not True:raise RuntimeError('远端未确认相机服务已关闭')
                forward=self.stop_forward(camera)
                outcome=dict(stopped=True,ready=False,remote=remote,forward=forward,checked_at=time.time())
            except Exception as error:
                outcome=dict(stopped=False,ready=False,error=str(error),remote=remote,checked_at=time.time())
            result['cameras'][camera['role']]=dict(outcome,label=camera['label'])
        result['completed']=all(c.get('stopped') is True for c in result['cameras'].values())
        result['checked_at']=time.time()
        if not result['completed']:
            result['error']='\n'.join(c['label']+'：'+c['error'] for c in result['cameras'].values() if not c['stopped'])
        return result

    def start_forward(self,camera):
        port=camera['port']
        argv=ssh_args()+['-o','ExitOnForwardFailure=yes','-N','-L',
                        f'127.0.0.1:{port}:127.0.0.1:{port}',HOST]
        with (self.folder/(camera['role']+'_forward.log')).open('ab') as log:
            proc=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (self.folder/(camera['role']+'_forward.json')).write_text(json.dumps(dict(pid=proc.pid,argv=argv),indent=2))
        return proc

    def ensure(self,camera):
        try:
            return dict(self.probe(camera),action='reused',forward_action='reused')
        except Exception as error:
            initial_error=str(error)
        if not KEY.is_file(): raise RuntimeError('缺少已有 KR260 SSH 密钥：'+str(KEY))
        remote=self.ensure_remote(camera)
        proc=None
        if not self.port_open(camera['port']): proc=self.start_forward(camera)
        deadline=time.monotonic()+READY_TIMEOUT_S
        while True:
            try:
                return dict(self.probe(camera),action=remote['action'],remote=remote,
                            forward_action='started' if proc is not None else 'reused',initial_error=initial_error)
            except Exception as error:
                if proc is not None and proc.poll() is not None:
                    raise RuntimeError('本机 SSH 转发启动失败；请查看 '+str(self.folder/(camera['role']+'_forward.log'))) from error
                if time.monotonic()>=deadline:
                    # Do not stop a pre-existing service or unknown listener.
                    if proc is not None:
                        proc.terminate()
                        try: proc.wait(timeout=2)
                        except subprocess.TimeoutExpired: proc.kill();proc.wait()
                    raise RuntimeError(f"{error}；请检查本机端口 {camera['port']} 与 SSH 转发（未终止已有服务）") from error
                time.sleep(.2)

    def start(self):
        self.folder.mkdir(parents=True,exist_ok=True)
        result=dict(completed=False,cameras={},robot_commands_sent=0,checked_at=None)
        for camera in CAMERAS:
            try: outcome=self.ensure(camera)
            except Exception as error: outcome=dict(ready=False,error=str(error),checked_at=time.time())
            result['cameras'][camera['role']]=dict(outcome,label=camera['label'])
        result['completed']=all(camera.get('ready') is True for camera in result['cameras'].values())
        result['checked_at']=time.time()
        if not result['completed']:
            result['error']='\n'.join(c['label']+'：'+c['error'] for c in result['cameras'].values() if not c['ready'])
        return result


def main():
    parser=argparse.ArgumentParser(description='仅启动并验证双相机服务及本机转发，不操作机械臂')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--action',choices=['start','stop'],default='start')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    # Serialize button requests even if this helper is launched independently.
    lock_path=Path(__file__).resolve().parents[1]/'logs/camera_services/camera_start.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    with lock_path.open('a+') as lock:
        try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            result=dict(completed=False,error='另一次相机服务操作正在进行，请等待完成',cameras={},robot_commands_sent=0)
        else:
            manager=CameraServices(args.output)
            result=manager.start() if args.action=='start' else manager.stop()
    (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['completed'] else 1


if __name__=='__main__': raise SystemExit(main())
