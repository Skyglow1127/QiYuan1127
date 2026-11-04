#!/usr/bin/env python3
"""Run an offline benchmark and record system memory and process status."""
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

root = Path(__file__).resolve().parent
results_root = Path(os.environ.get(
    'PI05_BENCHMARK_RESULTS_ROOT',
    '/media/aup/UbuntuData/技术文档/嵌入式比赛技术文档/π0.5本机测速结果',
))
results_root.mkdir(parents=True, exist_ok=True)
out = results_root / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
out.mkdir(exist_ok=False)
log = out / 'benchmark.log'
argv = [sys.executable, '-u', str(root / 'benchmark.py'), '--output', str(out), '--warmup', '3', '--repeats', '10']
started = time.monotonic()
minimum = 2**63
reason = None
requested_stop = None


def request_stop(signum, _frame):
    global requested_stop
    requested_stop = f'received signal {signum}'


for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(signum, request_stop)


print('结果保存到：', out, flush=True)
print('开始加载模型；加载完成后会显示每轮测速进度。', flush=True)
with log.open('w') as stream:
    process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, start_new_session=True,
                               text=True, errors='replace', bufsize=1)

    def copy_output():
        assert process.stdout is not None
        for line in process.stdout:
            stream.write(line)
            stream.flush()
            try:
                print(line, end='', flush=True)
            except BrokenPipeError:
                pass

    reader = threading.Thread(target=copy_output, daemon=True)
    reader.start()
    while process.poll() is None:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemAvailable:'):
                available = int(line.split()[1]) * 1024
                break
        minimum = min(minimum, available)
        if requested_stop:
            reason = requested_stop
        elif available < int(1.5 * 1024**3):
            reason = 'available system memory below 1.5 GiB'
        elif time.monotonic() - started > 600:
            reason = 'benchmark exceeded 600 seconds'
        if reason:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            break
        time.sleep(0.2)
    code = process.wait()
    reader.join(timeout=5)
summary = {'return_code': code, 'stop_reason': reason, 'elapsed_seconds': time.monotonic() - started,
           'minimum_available_system_memory_bytes': minimum, 'log_path': str(log), 'argv': argv}
(out / 'supervisor.json').write_text(json.dumps(summary, indent=2) + '\n')
success = code == 0 and reason is None and (out / 'report.json').is_file() and (out / '技术文档可用数据.md').is_file()
if success:
    print('\n测速完成。可以直接查看技术文档用数据：', flush=True)
    print(out / '技术文档可用数据.md', flush=True)
    if (out / '技术文档可用数据.md').is_file():
        print('\n' + (out / '技术文档可用数据.md').read_text(), flush=True)
else:
    readable_reason = {
        'available system memory below 1.5 GiB': '电脑可用内存不足，请关闭不需要的程序后重新运行。',
        'benchmark exceeded 600 seconds': '测速超过 10 分钟。',
    }.get(reason, reason or (f'进程退出码 {code}' if code else '结果文件不完整'))
    print('\n测速没有完成。原因：', readable_reason, flush=True)
    print('日志：', log, flush=True)
sys.exit(0 if success else 1)
