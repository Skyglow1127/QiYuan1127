#!/usr/bin/env python3
"""Local demonstration workstation. Device access is through a read-only recorder.

Replay plans can be inspected here. Physical replay is deliberately unavailable
while the installed D1 control/holding and gripper interfaces remain unverified.
No endpoint powers, enables, releases, zeros, or commands an arm position.
"""
import argparse
from collections import deque
import csv
from datetime import datetime
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, urlopen

HERE = Path(__file__).resolve().parent
RECORDER = HERE.parent/'manual_demo_recorder'
OUTPUT_ROOT = Path('/media/aup/UbuntuData/抓取实验/手动示教')
CHILD_URL = 'http://127.0.0.1:18772'
ARTIFACTS = {
    'joint_timeline.csv': 'joint_timeline.csv',
    'demonstration_timeline.csv': 'analysis/demonstration_timeline.csv',
    'demonstration_timeline.png': 'analysis/demonstration_timeline.png',
    'analysis_review.json': 'analysis/analysis_review.json',
    'replay_preview.html': 'analysis/replay_preview.html',
}
BLOCKERS = [
    {'code':'controller_hold_not_verified','message':'D1 在当前位置安全恢复保持的接口尚未通过验证。'},
    {'code':'gripper_timing_not_verified','message':'现有执行器只支持六个臂关节，夹爪时序和动态控制尚未验证。'},
    {'code':'physical_path_not_reviewed','message':'尚未核对从当前姿态到演示起点的路径，以及演示路径与当前桌面环境。'},
]
SEGMENT_VELOCITY_METHOD = {
    'id': 'formal_segment_receipt_time_v1',
    'basis': 'DDS host receipt timestamps; estimated from measured positions, not motor velocity.',
    'raw': 'Adjacent positions within the selected segment divided by their positive receipt-time interval.',
    'smoothed': 'Endpoint slope over the latest <=0.4 seconds inside the selected segment.',
    'reset': 'Reset at the formal start, invalid/nonmonotonic samples, and recorded or >0.5 second gaps.',
    'first_sample_velocity': None,
    'preparation_samples_used': False,
    'position_clipping_or_unwrapping': False,
    'units': ['deg/s'] * 6 + ['sdk_raw/s'],
}

def read_json(path):
    return json.loads(Path(path).read_text())

def save_json(path, value):
    temporary = Path(str(path)+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)

def json_request(url, data=None, token=None):
    headers = {} if token is None else {'X-Recorder-Token':token}
    if data is not None:
        data = json.dumps(data, ensure_ascii=False).encode()
        headers['Content-Type'] = 'application/json'
    with urlopen(Request(url, data=data, headers=headers), timeout=5) as response:
        return json.load(response)

def safe_session(root, name):
    if not isinstance(name, str) or not re.fullmatch(r'demo_\d{8}_\d{6}',name):
        raise ValueError('无效的记录编号')
    path = (root/name).resolve()
    if path.parent != root.resolve() or not path.is_dir():
        raise ValueError('记录不存在或不属于本程序目录')
    return path

def pid_is_recorder(pid):
    try:
        parts = Path(f'/proc/{int(pid)}/cmdline').read_bytes().split(b'\0')
        return str(RECORDER/'recorder.py').encode() in parts and b'18772' in parts
    except (OSError, ValueError, TypeError):
        return False

def load_samples(path):
    return [json.loads(line) for line in (path/'joint_samples.jsonl').read_text().splitlines() if line.strip()]

def segment_rows(path):
    events = [json.loads(line) for line in (path/'events.jsonl').read_text().splitlines() if line.strip()]
    starts = [row['elapsed_s'] for row in events if row.get('label') in ('开始演示','demonstration_started','manual_demo_started')]
    rows = load_samples(path)
    start = starts[0] if starts else (rows[0]['elapsed_s'] if rows else 0.)
    selected = [row for row in rows if row['elapsed_s'] >= start]
    # Derive new values on copies: a stored smoothing window may begin before
    # the start marker, even when its final sample is inside the formal segment.
    derived, previous, window = [], None, deque()
    def finite(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    for original in selected:
        row = dict(original)
        stamp, q = row.get('dds_received_monotonic_s'), row.get('q')
        valid = finite(stamp) and isinstance(q, list) and len(q) == 7 and all(finite(value) for value in q)
        row['velocity'] = [None] * 7
        row['velocity_smoothed'] = [None] * 7
        row['segment_dt_s'] = None
        row['velocity_estimation_method'] = SEGMENT_VELOCITY_METHOD['id']
        if not valid:
            previous = None; window.clear(); derived.append(row)
            continue
        dt = stamp - previous['dds_received_monotonic_s'] if previous is not None else None
        reset = dt is None or dt <= 0 or dt > .5 or bool(row.get('gap_before'))
        if reset:
            window.clear()
        else:
            row['segment_dt_s'] = dt
            raw = [(q[i] - previous['q'][i]) / dt for i in range(7)]
            if all(finite(value) for value in raw):
                row['velocity'] = raw
        window.append((stamp, q))
        while len(window) > 1 and stamp - window[0][0] > .4:
            window.popleft()
        if len(window) > 1:
            first_stamp, first_q = window[0]
            smoothed = [(q[i] - first_q[i]) / (stamp - first_stamp) for i in range(7)]
            if all(finite(value) for value in smoothed):
                row['velocity_smoothed'] = smoothed
        previous = row
        derived.append(row)
    return start, derived

def export_segment(path):
    """Derived exports only; source timestamps and positions remain untouched."""
    start, rows = segment_rows(path)
    if not rows:
        raise ValueError('本次没有有效关节样本；保留原始日志，不能生成演示轨迹')
    summary = read_json(path/'summary.json')
    out = path/'analysis'
    out.mkdir(exist_ok=True)
    derived_by_sample = {str(row['sample']): row for row in rows}
    with (path/'joint_timeline.csv').open() as source, (out/'demonstration_timeline.csv').open('w',newline='') as target:
        reader = csv.DictReader(source)
        writer = csv.DictWriter(target,fieldnames=['episode_elapsed_s','segment_dt_s','velocity_estimation_method']+reader.fieldnames)
        writer.writeheader()
        for row in reader:
            if float(row['elapsed_s']) >= start:
                derived = derived_by_sample[row['sample']]
                # Keep original position/time columns, including the originally
                # recorded dt_s; segment_dt_s separately describes this estimate.
                for i, unit in enumerate(['deg'] * 6 + ['sdk_raw']):
                    row[f'J{i}_velocity_{unit}_per_s'] = derived['velocity'][i]
                    row[f'J{i}_velocity_smoothed_{unit}_per_s'] = derived['velocity_smoothed'][i]
                writer.writerow({'episode_elapsed_s':float(row['elapsed_s'])-start,
                                 'segment_dt_s':derived['segment_dt_s'],
                                 'velocity_estimation_method':SEGMENT_VELOCITY_METHOD['id'],**row})
    result = dict(start_marker_elapsed_s=start,stop_elapsed_s=summary['elapsed_s'],
                  duration_s=max(0.,summary['elapsed_s']-start),sample_count=len(rows),
                  first_q=rows[0]['q'],last_q=rows[-1]['q'],positions_modified=False,
                  original_timestamps_preserved=True,replay_executed=False,
                  grasp_success_verified=False,velocity_estimation=SEGMENT_VELOCITY_METHOD)
    save_json(out/'demonstration_segment.json',result)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(7,2,figsize=(12,14),sharex=True)
        ts = [row['elapsed_s']-start for row in rows]
        for i in range(7):
            axes[i,0].plot(ts,[r['q'][i] for r in rows],color='#1767b8',linewidth=1.3)
            axes[i,1].plot(ts,[r['velocity_smoothed'][i] for r in rows],color='#aa6500',linewidth=1.2)
            unit = 'deg' if i<6 else 'raw'
            axes[i,0].set_ylabel(f'J{i} {unit}')
            axes[i,1].set_ylabel(f'J{i} {unit}/s')
            for axis in axes[i]: axis.grid(alpha=.22)
        axes[0,0].set_title('Measured positions')
        axes[0,1].set_title('Estimated, smoothed velocity')
        for axis in axes[-1]: axis.set_xlabel('Seconds after start marker')
        fig.suptitle(f'D1 demonstration: {result["duration_s"]:.2f} s, {len(rows)} samples. No physical replay.')
        fig.tight_layout(rect=[0,0,1,.97])
        fig.savefig(out/'demonstration_timeline.png',dpi=130)
        plt.close(fig)
    except ImportError:
        pass
    return result

class Workstation:
    def __init__(self, root=OUTPUT_ROOT):
        self.root = Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.lock = threading.RLock()
        self.planning_lock = threading.Lock()
        self.token = secrets.token_urlsafe(32)
        self.state = 'idle'
        self.error = None
        self.active = None
        self.selected = None
        self.process = None
        self.child_pid = None
        self.child_token = None
        self.plan_cache = {}
        self.session_cache = {}
        self.worker = None
        self.shutting_down = False
        self.recover_recorder()

    def recover_recorder(self):
        try:
            active = read_json(RECORDER/'active_session.json')
            if active.get('url') != CHILD_URL or not pid_is_recorder(active.get('pid')): return
            status = json_request(CHILD_URL+'/api/status')
            path = safe_session(self.root,Path(status['path']).name)
            if status['state'] in ('starting','recording'):
                self.active = self.selected = path.name
                self.child_pid = active['pid']
                self.child_token = active['csrf_token']
                self.state = 'starting'
                self.worker = threading.Thread(target=self._resume_worker,daemon=True)
                self.worker.start()
            elif status['state'] in ('stopped','error'):
                os.kill(active['pid'], signal.SIGTERM)
        except (OSError, ValueError, KeyError):
            return

    def _resume_worker(self):
        try:
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                with self.lock:
                    if self.state!='starting': return
                status=json_request(CHILD_URL+'/api/status')
                if status.get('ready') and status.get('fresh'):
                    path=safe_session(self.root,self.active)
                    events=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines() if line.strip()]
                    with self.lock:
                        if self.state!='starting': return
                        if not any(event.get('label')=='开始演示' for event in events):
                            json_request(CHILD_URL+'/api/mark',{'label':'开始演示'},self.child_token)
                        self.state='recording'
                    return
                time.sleep(.15)
            raise RuntimeError('恢复记录器后未收到足够的新鲜反馈')
        except Exception as exc:
            self._end_child()
            with self.lock: self.state='error';self.error=str(exc)

    def sessions(self):
        items = []
        for path in sorted(self.root.glob('demo_*'),reverse=True):
            try:
                safe_session(self.root,path.name)
                meta = read_json(path/'metadata.json')
                ended = path/'summary.json'
                summary = read_json(ended) if ended.exists() else {}
                segment = path/'analysis/demonstration_segment.json'
                formal = read_json(segment) if segment.exists() else {}
                items.append(dict(id=path.name,path=str(path),created_at=meta.get('created_at'),
                                  duration_s=formal.get('duration_s',summary.get('elapsed_s')),
                                  sample_count=formal.get('sample_count',summary.get('sample_count')),
                                  interface_test=(path/'interface_test.json').exists(),
                                  ended=summary.get('state') in ('stopped','error')))
            except (OSError,ValueError,KeyError): continue
        return items

    def status(self):
        with self.lock:
            state, active, error = self.state,self.active,self.error
        record = None
        if state in ('starting','recording','stopping'):
            try:
                record = json_request(CHILD_URL+'/api/status')
                record.pop('csrf_token',None)
                record['tail']=record.get('tail',[])[-300:]
                if state=='recording' and record.get('state') in ('error','stopped'):
                    with self.lock:
                        self.state='error';self.error=record.get('error') or '记录器意外停止；已有日志已保留'
                        state,error=self.state,self.error
            except Exception as exc:
                if state=='recording': error='无法读取记录器状态：'+str(exc)
        sessions = self.sessions()
        preferred=next((item['id'] for item in sessions if not item['interface_test']),None)
        selected = self.selected or preferred or (sessions[0]['id'] if sessions else None)
        return dict(schema='d1-workstation-v1',state=state,record=record,sessions=sessions,active_session_id=active,
                    selected_session_id=selected,csrf_token=self.token,error=error,
                    hardware_replay_available=False)

    def session(self, name):
        path = safe_session(self.root,name)
        summary_path = path/'summary.json'
        if not summary_path.exists():
            return dict(id=name,summary=None,segment=None,tail=[],blockers=BLOCKERS,
                        analysis_ready=False,preview_url=None,plot_url=None)
        version = summary_path.stat().st_mtime_ns
        if name in self.session_cache and self.session_cache[name][0] == version:
            return self.session_cache[name][1]
        summary = read_json(summary_path)
        start, rows = segment_rows(path)
        segment_file = path/'analysis/demonstration_segment.json'
        segment = read_json(segment_file) if segment_file.exists() else dict(duration_s=summary['elapsed_s']-start,sample_count=len(rows))
        result = dict(id=name,summary=summary,segment=segment,
                      tail=[dict(t=r['elapsed_s']-start,q=r['q'],v=r.get('velocity_smoothed')) for r in rows],
                      velocity_estimation=SEGMENT_VELOCITY_METHOD,
                      blockers=BLOCKERS,analysis_ready=(path/'analysis/analysis_review.json').exists(),
                      preview_url=f'/preview?session_id={name}',
                      plot_url=f'/api/artifact?session_id={name}&name=demonstration_timeline.png',
                      robot_commands_sent_by_app=0)
        self.session_cache[name]=(version,result)
        return result

    def start(self):
        with self.lock:
            if self.shutting_down: raise ValueError('程序正在退出')
            if self.state in ('starting','recording','stopping'): raise ValueError('已有一轮正在记录或保存')
            if self.child_pid and pid_is_recorder(self.child_pid): raise ValueError('先结束当前记录器，不能重复启动')
            self.state='starting';self.error=None;self.active=None
            self.worker=threading.Thread(target=self._start_worker,daemon=True)
            self.worker.start()
        return {'ok':True,'state':'starting'}

    def _start_worker(self):
        try:
            with self.lock:
                if self.state!='starting' or self.shutting_down: return
            with (HERE/'recorder_child.log').open('a') as log:
                self.process=subprocess.Popen([sys.executable,'-u',str(RECORDER/'recorder.py'),
                    '--port','18772','--output-root',str(self.root)],stdin=subprocess.DEVNULL,
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self.child_pid=self.process.pid
            deadline=time.monotonic()+15
            marked=False
            while time.monotonic()<deadline:
                if self.process.poll() is not None: raise RuntimeError('记录器启动失败，请查看 recorder_child.log；没有发出机械臂控制指令')
                try:
                    current=read_json(RECORDER/'active_session.json')
                    if current.get('pid') != self.child_pid: raise ValueError('等待本轮记录器启动')
                    status=json_request(CHILD_URL+'/api/status')
                    self.child_token=current['csrf_token']
                    name=Path(status['path']).name
                    safe_session(self.root,name)
                    with self.lock:
                        self.active=self.selected=name
                        if self.state!='starting': return
                        if not marked and status.get('sample_count',0)>0:
                            json_request(CHILD_URL+'/api/mark',{'label':'开始演示'},self.child_token)
                            marked=True
                        if status.get('ready') and status.get('fresh'):
                            self.state='recording'
                            return
                    if status.get('state')=='error': raise RuntimeError(status.get('error','读取反馈失败'))
                except (OSError,ValueError,KeyError): pass
                time.sleep(.15)
            raise RuntimeError('15 秒内未收到足够的新鲜关节反馈')
        except Exception as exc:
            self._end_child()
            with self.lock: self.state='error';self.error=str(exc)

    def stop(self):
        with self.lock:
            if self.state=='stopping': return {'ok':True,'state':'stopping'}
            if self.state not in ('starting','recording','error'): return {'ok':True,'state':self.state}
            old_worker=self.worker
            self.state='stopping'
            self.worker=threading.Thread(target=self._stop_worker,args=(old_worker,),daemon=True)
            self.worker.start()
        return {'ok':True,'state':'stopping'}

    def _end_child(self):
        if not self.child_pid or not pid_is_recorder(self.child_pid): return
        if self.child_token:
            try: json_request(CHILD_URL+'/api/stop',{'reason':'user_pressed_app_end'},self.child_token)
            except Exception: pass
        if pid_is_recorder(self.child_pid): os.kill(self.child_pid,signal.SIGTERM)
        if self.process:
            try: self.process.wait(timeout=6)
            except subprocess.TimeoutExpired: self.process.kill();self.process.wait(timeout=2)
        else:
            deadline=time.monotonic()+6
            while pid_is_recorder(self.child_pid) and time.monotonic()<deadline: time.sleep(.1)

    def _stop_worker(self,old_worker):
        try:
            if old_worker and old_worker.is_alive(): old_worker.join(timeout=17)
            self._end_child()
            if self.active:
                path=safe_session(self.root,self.active)
                if not (path/'summary.json').exists(): raise RuntimeError('未找到结束摘要，请检查原始日志')
                run=subprocess.run([sys.executable,str(RECORDER/'analyze_demo.py'),'--session',str(path)],
                    capture_output=True,text=True,timeout=60)
                (path/'analysis_command.log').write_text(run.stdout+run.stderr)
                if run.returncode: raise RuntimeError('日志已保存，但分析失败：'+(run.stderr or run.stdout)[-500:])
                export_segment(path)
                self.session_cache.pop(self.active,None)
            with self.lock: self.state='idle';self.error=None;self.child_pid=None;self.child_token=None
        except Exception as exc:
            with self.lock: self.state='error';self.error=str(exc)

    def prepare(self, name, speed):
        with self.lock:
            if self.state in ('starting','recording','stopping'): raise ValueError('请先结束并保存当前记录')
        with self.planning_lock:
            from planner import prepare_session
            path=safe_session(self.root,name)
            plan=prepare_session(path,speed_deg_s=speed)
            (path/'analysis').mkdir(exist_ok=True)
            save_json(path/'analysis/replay_plan.json',plan)
            with self.lock: self.plan_cache[name]=plan;self.selected=name
            source_duration=plan['source_marker_duration']
            if source_duration is None: source_duration=plan['source_duration']
            return dict(plan,sha256=plan['plan_sha256'],points=len(plan.get('original_points',[])),
                        duration_s=plan['duration'],source_duration_s=source_duration,
                        blockers=BLOCKERS)

    def replay(self, name, digest, approved):
        # A click cannot bypass the currently unverified physical-control path.
        safe_session(self.root,name)
        with self.lock: plan=self.plan_cache.get(name)
        if not plan or not secrets.compare_digest(str(digest),str(plan['plan_sha256'])):
            return {'ok':False,'error':'请先生成并审核当前会话的回放计划','blockers':BLOCKERS},409
        if approved is not True:
            return {'ok':False,'error':'尚未确认具体轨迹和参数','blockers':BLOCKERS},409
        return {'ok':False,'error':'当前实物复现接口未通过验证，已阻止发送运动指令',
                'blockers':BLOCKERS,'robot_commands_sent':0},409

    def close(self):
        self.shutting_down=True
        if self.state in ('starting','recording','error') and self.child_pid:
            self.stop()
        if self.worker and self.worker.is_alive(): self.worker.join(timeout=22)
        self._end_child()

def make_handler(app, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def reply(self,data,code=200,ctype='application/json; charset=utf-8'):
            body=data if isinstance(data,bytes) else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type',ctype)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('X-Frame-Options','SAMEORIGIN')
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass
        def do_GET(self):
            url=urlsplit(self.path);q=parse_qs(url.query)
            try:
                if url.path=='/': self.reply((HERE/'index.html').read_bytes(),ctype='text/html; charset=utf-8')
                elif url.path=='/api/status': self.reply(app.status())
                elif url.path=='/api/session': self.reply(app.session(q.get('id',[''])[0]))
                elif url.path=='/preview':
                    path=safe_session(app.root,q.get('session_id',[''])[0])
                    existing=path/'analysis/replay_preview.html'
                    template=HERE/'preview_template.html'
                    if existing.exists(): self.reply(existing.read_bytes(),ctype='text/html; charset=utf-8')
                    elif template.exists(): self.reply(template.read_bytes(),ctype='text/html; charset=utf-8')
                    else: self.reply({'error':'请在工作台查看所选会话轨迹曲线'},404)
                elif url.path=='/api/artifact':
                    path=safe_session(app.root,q.get('session_id',[''])[0])
                    name=q.get('name',[''])[0]
                    if name not in ARTIFACTS: raise ValueError('不支持的文件类型')
                    artifact=(path/ARTIFACTS[name]).resolve()
                    if not artifact.is_relative_to(path): raise ValueError('文件路径无效')
                    if not artifact.is_file(): self.reply({'error':'该文件尚未生成'},404);return
                    self.reply(artifact.read_bytes(),ctype=mimetypes.guess_type(artifact.name)[0] or 'application/octet-stream')
                else: self.reply({'error':'not found'},404)
            except (ValueError,KeyError,OSError) as exc: self.reply({'error':str(exc)},400)
        def do_POST(self):
            origin=self.headers.get('Origin')
            if origin and origin not in (f'http://127.0.0.1:{port}',f'http://localhost:{port}'):
                self.reply({'ok':False,'error':'拒绝跨站请求'},403);return
            if not secrets.compare_digest(self.headers.get('X-App-Token',''),app.token):
                self.reply({'ok':False,'error':'请刷新页面后重试'},403);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<=length<=16384: raise ValueError('请求长度无效')
                data=json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(data,dict): raise ValueError('请求必须是JSON对象')
                path=urlsplit(self.path).path
                if path=='/api/start': self.reply(app.start(),202)
                elif path=='/api/stop': self.reply(app.stop(),202)
                elif path=='/api/prepare': self.reply({'ok':True,'plan':app.prepare(data.get('session_id'),data.get('speed_deg_s',.5))})
                elif path=='/api/replay':
                    result,code=app.replay(data.get('session_id'),data.get('plan_sha256'),data.get('approval'))
                    self.reply(result,code)
                else: self.reply({'ok':False,'error':'not found'},404)
            except (ValueError,KeyError,TypeError,OSError) as exc: self.reply({'ok':False,'error':str(exc)},400)
            except Exception as exc: self.reply({'ok':False,'error':str(exc)},500)
    return Handler

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=18771)
    parser.add_argument('--output-root',type=Path,default=OUTPUT_ROOT)
    args=parser.parse_args()
    lock=(HERE/'app.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    app=Workstation(args.output_root)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(app,args.port))
    save_json(HERE/'app_process.json',{'pid':os.getpid(),'url':f'http://127.0.0.1:{args.port}','started_at':datetime.now().astimezone().isoformat()})
    for sig in (signal.SIGTERM,signal.SIGINT):
        signal.signal(sig,lambda *_: threading.Thread(target=server.shutdown,daemon=True).start())
    print(f'D1_WORKSTATION http://127.0.0.1:{args.port}',flush=True)
    try: server.serve_forever(poll_interval=.2)
    finally: app.close();server.server_close()

if __name__=='__main__': main()
