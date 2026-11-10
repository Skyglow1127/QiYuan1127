#!/usr/bin/env python3
"""Local launcher: no HTTP server is needed to start or stop the simulator."""
import json
import os
import queue
import socket
import subprocess
import threading
import time
import tkinter as tk
import urllib.request
import webbrowser
from pathlib import Path
from tkinter import ttk

ROOT = Path(__file__).resolve().parents[1]
PORT = int(os.environ.get('D1T_PORT') or json.loads((ROOT / 'config/runtime.json').read_text()).get('simulation_port', 8767))
if not 1 <= PORT <= 65535:
    raise ValueError('D1T_PORT 必须是 1–65535 的端口号。')
URL = f'http://127.0.0.1:{PORT}/'
REAL_URL = 'http://127.0.0.1:8766/'


def real_feedback():
    try:
        with urllib.request.urlopen(REAL_URL + 'api/state', timeout=0.5) as response:
            return json.load(response)
    except (OSError, ValueError):
        return None


def is_simulation_state(state):
    return (isinstance(state, dict) and isinstance(state.get('connected'), bool)
            and isinstance(state.get('positions'), dict)
            and isinstance(state.get('targets'), dict)
            and 'arm_Joint1' in state['targets']
            and isinstance(state.get('mode'), str) and state['mode'].startswith('Gazebo')
            and isinstance(state.get('sim_time'), (int, float)))


def feedback():
    try:
        with urllib.request.urlopen(URL + 'api/state', timeout=1) as response:
            state = json.load(response)
        return state if is_simulation_state(state) else None
    except (OSError, ValueError):
        return None


def port_open():
    try:
        with socket.create_connection(('127.0.0.1', PORT), timeout=0.5):
            return True
    except OSError:
        return False


def services():
    result = subprocess.run(
        ['systemctl', '--user', 'is-active', 'd1t-sim.service', 'd1t-gui.service'],
        capture_output=True, text=True, timeout=5)
    return result.stdout.splitlines()


class Console:
    def __init__(self, window):
        self.window = window
        self.busy = False
        self.opening = False
        self.real_busy = False
        self.events = queue.Queue()
        self.closed = threading.Event()
        window.title('D1-T 从臂 · 仿真控制台')
        window.geometry('620x570')
        window.minsize(590, 550)
        window.configure(bg='#f3f5f8')
        style = ttk.Style(window)
        style.theme_use('clam')
        style.configure('TFrame', background='#f3f5f8')
        style.configure('TLabel', background='#f3f5f8', font=('Sans', 11))
        style.configure('TButton', font=('Sans', 12), padding=12)
        frame = ttk.Frame(window, padding=24)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='D1-T 从臂仿真', font=('Sans', 19, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='照片工作台 · D1-T · KR260 · Ryzen AI 9 HX 370').pack(anchor='w', pady=(8, 18))
        self.status = tk.StringVar(value='正在检查运行状态…')
        self.status_label = ttk.Label(frame, textvariable=self.status, font=('Sans', 12, 'bold'))
        self.status_label.pack(anchor='w')
        self.details = tk.StringVar(value='')
        ttk.Label(frame, textvariable=self.details).pack(anchor='w', pady=(5, 14))
        row = ttk.Frame(frame)
        row.pack(fill='x')
        self.start_button = ttk.Button(row, text='▶ 启动仿真', command=lambda: self.action('start'))
        self.start_button.pack(side='left', fill='x', expand=True, padx=(0, 8))
        self.stop_button = ttk.Button(row, text='■ 关闭仿真', command=lambda: self.action('stop'))
        self.stop_button.pack(side='left', fill='x', expand=True)
        self.open_button = ttk.Button(frame, text='打开机械臂控制页面', command=self.open_page)
        self.open_button.pack(fill='x', pady=10)
        address_row = ttk.Frame(frame)
        address_row.pack(fill='x', pady=(0, 8))
        self.address = tk.StringVar(value=URL)
        ttk.Entry(address_row, textvariable=self.address, state='readonly',
                  font=('Sans', 11)).pack(side='left', fill='x', expand=True)
        ttk.Button(address_row, text='复制网址', command=self.copy_url).pack(side='left', padx=(8, 0))
        ttk.Separator(frame).pack(fill='x', pady=6)
        self.real_status = tk.StringVar(value='实机监视：未连接')
        ttk.Label(frame, textvariable=self.real_status).pack(anchor='w', pady=(4, 6))
        real_row = ttk.Frame(frame)
        real_row.pack(fill='x')
        self.real_open = ttk.Button(real_row, text='连接并查看实机（只读）', command=lambda: self.real_action('start'))
        self.real_open.pack(side='left', fill='x', expand=True, padx=(0, 8))
        self.real_close = ttk.Button(real_row, text='断开实机监视', command=lambda: self.real_action('stop'))
        self.real_close.pack(side='left')
        self.message = tk.StringVar(value='点击“启动仿真”，就绪后会自动打开控制页面。')
        ttk.Label(frame, textvariable=self.message, wraplength=490).pack(anchor='w', pady=6)
        ttk.Label(frame, text='关闭此窗口只收起面板；停止仿真请点“关闭仿真”。',
                  font=('Sans', 10), foreground='#5b6472').pack(anchor='w', side='bottom')
        window.protocol('WM_DELETE_WINDOW', self.close)
        threading.Thread(target=self.poll, daemon=True).start()
        window.after(100, self.drain)

    def close(self):
        self.closed.set()
        self.window.destroy()

    def open_page(self):
        if self.opening:
            return
        self.opening = True
        self.message.set('正在请求浏览器新窗口打开控制页面…')
        threading.Thread(target=self.launch_browser, daemon=True).start()

    def copy_url(self):
        self.window.clipboard_clear()
        self.window.clipboard_append(URL)
        self.message.set('网址已复制，粘贴到浏览器地址栏后按回车即可。')

    def launch_browser(self):
        try:
            if feedback() is None:
                raise RuntimeError('控制服务尚未就绪，请先点击“启动仿真”。')
            # Explicitly request a new window instead of an easily missed background tab.
            if not webbrowser.open(URL, new=1, autoraise=True):
                raise RuntimeError('未能调用浏览器，请复制下方网址手动打开。')
            self.events.put(('browser', '已请求浏览器打开新窗口；若未看到，请复制下方网址到浏览器打开。'))
        except Exception as exc:
            self.events.put(('browser', '打开页面未完成：' + str(exc)))

    def real_action(self, action):
        if self.real_busy:
            return
        self.real_busy = True
        self.real_open.state(['disabled'])
        self.real_close.state(['disabled'])
        self.message.set('正在连接实机反馈…' if action == 'start' else '正在断开实机监视…')
        threading.Thread(target=self.perform_real, args=(action,), daemon=True).start()

    def perform_real(self, action):
        try:
            script = ROOT / ('run_real.sh' if action == 'start' else 'stop_real.sh')
            result = subprocess.run(['/bin/bash', str(script)], capture_output=True, text=True, timeout=90)
            if result.returncode:
                raise RuntimeError((result.stderr or result.stdout).strip()[-300:])
            if action == 'start':
                deadline = time.monotonic() + 15
                while real_feedback() is None:
                    if time.monotonic() > deadline:
                        raise RuntimeError('实机监视页启动超时。')
                    time.sleep(.3)
                opened = webbrowser.open(REAL_URL, new=1, autoraise=True)
                message = '实机只读页：' + REAL_URL
                if not opened:
                    message += '（浏览器未打开，请手动访问）'
            else:
                message = '实机监视已断开；机械臂供电和使能状态未改变。'
            self.events.put(('real_done', message))
        except Exception as exc:
            self.events.put(('real_done', '实机监视操作未完成：' + str(exc)))

    def poll(self):
        while not self.closed.is_set():
            try:
                self.events.put(('state', (feedback(), services(), port_open())))
                self.events.put(('real_state', real_feedback()))
            except Exception as exc:
                self.events.put(('error', str(exc)))
            self.closed.wait(2)

    def action(self, action):
        if self.busy:
            return
        self.busy = True
        for button in (self.start_button, self.stop_button, self.open_button):
            button.state(['disabled'])
        self.message.set('正在启动，请等待机械臂反馈…' if action == 'start' else '正在关闭 Gazebo 和控制端口…')
        threading.Thread(target=self.perform, args=(action,), daemon=True).start()

    def perform(self, action):
        try:
            result = subprocess.run(['/bin/bash', str(ROOT / ('run.sh' if action == 'start' else 'stop.sh'))],
                                    capture_output=True, text=True, timeout=45)
            if result.returncode:
                raise RuntimeError((result.stderr or result.stdout).strip()[-500:])
            deadline = time.monotonic() + (75 if action == 'start' else 10)
            while time.monotonic() < deadline:
                if action == 'start':
                    state = feedback()
                    if state and state.get('connected'):
                        self.events.put(('done', 'start'))
                        return
                elif feedback() is None and all(s != 'active' for s in services()):
                    self.events.put(('done', 'stop'))
                    return
                time.sleep(0.5)
            raise RuntimeError('等待启动超时，请查看 work/run 中的日志。' if action == 'start'
                               else f'关闭后仍检测到本仿真服务或 {PORT} 端口上的仿真反馈。')
        except Exception as exc:
            self.events.put(('failed', str(exc)))

    def drain(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == 'state':
                    state, units, listening = data
                    ready = bool(state and state.get('connected'))
                    if ready:
                        text, color = '● 仿真已就绪', '#137333'
                    elif 'active' in units or state:
                        text, color = '● 启动中 / 等待机械臂连接', '#985800'
                    elif listening:
                        text, color = f'● 端口 {PORT} 被其他服务占用', '#b3261e'
                    else:
                        text, color = '● 仿真已关闭', '#5b6472'
                    self.status.set(text)
                    self.status_label.configure(foreground=color)
                    gui = len(units) > 1 and units[1] == 'active'
                    self.details.set(f'{PORT} 端口：{"已开启" if listening else "已关闭"}    Gazebo 窗口：{"运行中" if gui else "已关闭"}')
                    if not self.busy:
                        self.open_button.state(['!disabled'])
                elif kind in ('done', 'failed'):
                    self.busy = False
                    self.start_button.state(['!disabled'])
                    self.stop_button.state(['!disabled'])
                    if kind == 'failed':
                        self.message.set('操作未完成：' + data)
                    elif data == 'start':
                        self.open_button.state(['!disabled'])
                        self.message.set('已启动。可在控制页面操作关节、抓取和放下方块。')
                        self.open_page()
                    else:
                        self.open_button.state(['!disabled'])
                        self.message.set('已关闭本项目仿真服务。可随时重新启动。')
                elif kind == 'browser':
                    self.opening = False
                    self.message.set(data)
                elif kind == 'real_state':
                    self.real_status.set('实机监视：反馈在线 · 只读' if data and data.get('connected') else
                                         '实机监视：等待新反馈' if data else '实机监视：未连接')
                elif kind == 'real_done':
                    self.real_busy = False
                    self.real_open.state(['!disabled'])
                    self.real_close.state(['!disabled'])
                    self.message.set(data)
                elif kind == 'error' and not self.busy:
                    self.message.set('状态检查失败：' + data)
        except queue.Empty:
            pass
        self.window.after(100, self.drain)


if __name__ == '__main__':
    window = tk.Tk()
    console = Console(window)
    # Lifecycle smoke mode exercises the same button callbacks without opening a browser.
    import sys
    if '--open-page' in sys.argv:
        window.after(300, console.open_page)
    if '--smoke-test' in sys.argv:
        console.open_page = lambda: None
        def smoke_start():
            console.action('start')
            window.after(200, wait_start)
        def wait_start():
            if console.busy:
                window.after(200, wait_start)
            elif console.message.get().startswith('已启动'):
                console.action('stop')
                window.after(200, wait_stop)
            else:
                print(console.message.get(), flush=True)
                console.close()
                raise SystemExit(1)
        def wait_stop():
            if console.busy:
                window.after(200, wait_stop)
            else:
                ok = console.message.get().startswith('已关闭仿真')
                print('PASS: 启动、仿真反馈连接、仿真服务关闭' if ok else console.message.get(), flush=True)
                console.close()
                if not ok:
                    raise SystemExit(1)
        window.after(300, smoke_start)
    window.mainloop()
