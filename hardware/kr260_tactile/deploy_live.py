#!/usr/bin/env python3
"""Deploy this verified pressure package on KR260, then read 100 real samples.

Run on the board with sudo. Only the stock slot-0 image may be replaced.
An already loaded matching pressure image is verified and granted access only.
No passwords, direct register writes, rollback guesses, or simulated samples.
"""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import stat
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
STOCK = 'k26-starter-kits'
TARGET = 'kr260-fsr'
TARGET_USER = 'ubuntu'
DT_ROOT = Path('/sys/firmware/devicetree/base')
OVERLAY_ROOT = Path('/sys/kernel/config/device-tree/overlays')


def parse_listapps(output):
    """Accept only known AMD table layouts; unknown output fails closed.

    Primary format references: AMD Kria application running-on-board docs and
    KD240 app_deployment docs (the latter adds a Pid column). The installed
    dfx-mgr 2024.2 table uses slot->handle; retain both values explicitly.
    """
    rows, header = [], None
    for raw in output.splitlines():
        line = raw.strip()
        if not line or re.fullmatch(r'Socket [0-9]+ closed by client', line):
            continue
        fields = line.split()
        if fields[0] == 'Accelerator':
            if header is not None:
                raise ValueError('重复的 xmutil 表头')
            expected = ['Accelerator', 'Accel_type', 'Base', 'Base_type',
                        '#slots(PL+AIE)', 'Active_slot']
            normalized = [f for f in fields if f != 'Pid']
            normalized = ['#slots(PL+AIE)' if f == '#slots(RPU+PL+AIE)' else
                          'Active_slot' if f == 'slot->handle' else f for f in normalized]
            if normalized != expected or fields.count('Pid') > 1:
                raise ValueError('不支持的 xmutil 表头：' + line)
            if 'Pid' in fields and fields.index('Pid') != 3:
                raise ValueError('无法识别 Pid 列位置')
            header = fields
            continue
        if header is None or len(fields) != len(header):
            raise ValueError('无法严格解析 xmutil 行：' + line)
        row = dict(zip(header, fields))
        if not re.fullmatch(r'[A-Za-z0-9_.+-]+', row['Accelerator']):
            raise ValueError('无效应用名')
        slot_key = 'Active_slot' if 'Active_slot' in row else 'slot->handle'
        value = row[slot_key]
        if value == '-1':
            slots = []
            handles = []
        else:
            if not re.fullmatch(r'[0-9]+(?:->[0-9]+)?(?:,[0-9]+(?:->[0-9]+)?)*,?', value):
                raise ValueError('无法识别 slot：' + value)
            slots = []
            handles = []
            for item in value.rstrip(',').split(','):
                pair = item.split('->')
                if slot_key == 'Active_slot' and len(pair) != 1:
                    raise ValueError('Active_slot 列不应包含 handle')
                if slot_key == 'slot->handle' and len(pair) != 2:
                    raise ValueError('slot->handle 列缺少映射')
                slots.append(int(pair[0]))
                handles.append(int(pair[-1]))
            if len(set(slots)) != len(slots):
                raise ValueError('重复 active slot')
        slot_count_key = next(k for k in row if k.startswith('#slots'))
        if not re.fullmatch(r'\([0-9]+\+[0-9]+(?:\+[0-9]+)?\)', row[slot_count_key]):
            raise ValueError('无法识别 slots 数量')
        row['active_slots'] = slots
        row['active_handles'] = handles
        rows.append(row)
    if header is None or not rows:
        raise ValueError('xmutil 未返回非空可识别列表')
    if len({row['Accelerator'] for row in rows}) != len(rows):
        raise ValueError('存在重复应用名')
    return rows


def require_application(rows, active_name, require_target=True):
    by_name = {row['Accelerator']: row for row in rows}
    required = {active_name, TARGET} if require_target else {active_name}
    for name in required:
        row = by_name.get(name)
        if not row or row['Accel_type'] != 'XRT_FLAT' or row['Base_type'] != 'XRT_FLAT' or row['Base'] != name:
            raise ValueError('缺少预期的独立 XRT_FLAT 应用：' + name)
        # AMD dfx-mgr: no_id means PID/UID metadata is absent, not a mismatch.
        # The installed 2024.2 library contains id_ok/no_id/id_err; retain model,
        # overlay, package-hash and live-DT checks as the compatibility evidence.
        # https://github.com/Xilinx/dfx-mgr#command-to-list-the-packages
        if 'Pid' in row and row['Pid'] not in ('ok', 'id_ok', 'no_id'):
            raise ValueError('应用 Pid 不匹配或无法识别：' + name + '=' + row['Pid'])
        count = next(row[k] for k in row if k.startswith('#slots'))
        if count not in ('(0+0)', '(0+0+0)'):
            raise ValueError('应用不是已约定的平坦设计：' + name)
    active = [(row['Accelerator'], row['active_slots']) for row in rows if row['active_slots']]
    if active != [(active_name, [0])]:
        raise ValueError('当前应用不是唯一预期 slot 0：' + repr(active))
    return by_name[active_name]


def dt_text(path):
    return path.read_bytes().rstrip(b'\0\n').decode('utf-8')


def board_state():
    if not OVERLAY_ROOT.is_dir():
        raise ValueError('不存在 configfs overlay 目录')
    overlays = []
    for path in sorted(OVERLAY_ROOT.iterdir()):
        if not path.is_dir():
            raise ValueError('overlay 目录含未知条目：' + str(path))
        overlays.append({'name': path.name, 'status': dt_text(path / 'status'),
                         'path': dt_text(path / 'path')})
    return {'model': dt_text(DT_ROOT / 'model'), 'overlays': overlays,
            'firmware': dt_text(DT_ROOT / 'fpga-region/firmware-name'),
            'manager_state': Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()}


def classify_board(state):
    if 'KR260' not in state['model'] or state['manager_state'] != 'operating':
        raise ValueError('板卡不是 KR260 或 FPGA 未 operating')
    if len(state['overlays']) != 1 or state['overlays'][0]['status'] != 'applied':
        raise ValueError('只允许唯一且 applied 的已知 overlay')
    overlay = state['overlays'][0]['name']
    if overlay == STOCK + '_image_1' and state['firmware'] == STOCK + '.bit.bin':
        return STOCK
    if overlay == TARGET + '_image_1' and state['firmware'] == TARGET + '.bit.bin':
        return TARGET
    raise ValueError('不是允许替换的 stock overlay，也不是本压力固件：' + repr(state))


def find_gpio_device():
    """Called only after install_board.sh --grant-access passes full DT checks."""
    expected = (DT_ROOT / 'axi/gpio@a0000000').resolve(strict=True)
    matches = []
    for chip in Path('/sys/bus/gpio/devices').glob('gpiochip*'):
        if not re.fullmatch(r'gpiochip[0-9]+', chip.name):
            continue
        if (chip / 'of_node').resolve() != expected:
            continue
        dev = Path('/dev') / chip.name
        info = dev.lstat()
        if not stat.S_ISCHR(info.st_mode):
            raise ValueError('匹配的 GPIO 路径不是字符设备')
        major, minor = map(int, (chip / 'dev').read_text().strip().split(':'))
        if info.st_rdev != os.makedev(major, minor):
            raise ValueError('GPIO sysfs 与字符设备编号不对应')
        matches.append(str(dev))
    if len(matches) != 1:
        raise ValueError('没有唯一匹配的已核验压力 GPIO：' + repr(matches))
    return matches[0]


def summarize_samples(output, chip):
    rows = [json.loads(line) for line in output.splitlines() if line.strip()]
    if len(rows) != 100:
        raise ValueError(f'需要 100 条真实样本，实际 {len(rows)} 条')
    last_t = None
    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict) or row.get('source') != 'real' or row.get('chip') != chip or row.get('offsets') != [0, 1]:
            raise ValueError('样本来源或 GPIO 对应关系不匹配')
        if type(row.get('seq')) is not int or row['seq'] != index:
            raise ValueError('真实样本序号不连续')
        if any(type(row.get(k)) is not int or row[k] not in (0, 1) for k in ('do1', 'do2')):
            raise ValueError('真实样本存在非 0/1 电平')
        now = row.get('t_s')
        if type(now) not in (float, int) or not math.isfinite(now) or now < 0 or (last_t is not None and now <= last_t):
            raise ValueError('真实样本时间无效或不递增')
        last_t = now
    changes = [sum(a[k] != b[k] for a, b in zip(rows, rows[1:])) for k in ('do1', 'do2')]
    elapsed = rows[-1]['t_s'] - rows[0]['t_s']
    return {'count': len(rows), 'source': 'real', 'chip': chip, 'offsets': [0, 1],
            'requested_hz': 100, 'observed_hz': 99 / elapsed,
            'first_sample': rows[0], 'last_sample': rows[-1],
            'unique_levels': {k: sorted({row[k] for row in rows}) for k in ('do1', 'do2')},
            'observed_level_changes': dict(zip(('do1', 'do2'), changes)),
            'dynamic_levels_observed': bool(sum(changes)),
            'physical_press_correlation': 'NOT_VERIFIED: no observed physical press/release ground truth',
            'measurement': 'digital threshold states; no calibrated force or pressure',
            'interpretation': ('Observed real digital level changes; physical pressure correlation remains unverified.'
                               if sum(changes) else '100 real GPIO samples were constant; no pressure transition was observed.')}


def flush_file(stream):
    stream.flush()
    os.fsync(stream.fileno())


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class Deployment:
    def __init__(self):
        self.evidence = ROOT / 'evidence'
        if self.evidence.is_symlink():
            raise ValueError('evidence 不得为符号链接')
        self.evidence.mkdir(mode=0o755, exist_ok=True)
        sync_directory(ROOT)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '-' + str(os.getpid())
        self.folder = self.evidence / ('deployment_' + stamp)
        self.folder.mkdir(mode=0o755)
        sync_directory(self.evidence)
        self.state = {'schema_version': 1, 'status': 'IN_PROGRESS',
                      'started_utc': datetime.now(timezone.utc).isoformat(),
                      'evidence_directory': str(self.folder), 'steps': [],
                      'fpga_reloaded': False, 'acquisition': None,
                      'rollback': 'No automatic rollback; preserve exact command logs and inspect current state.'}
        self.save()

    def write(self, name, data):
        path = self.folder / name
        with path.open('x', encoding='utf-8') as stream:
            os.fchmod(stream.fileno(), 0o644)
            stream.write(data)
            flush_file(stream)
        sync_directory(path.parent)
        return path

    def save(self):
        data = json.dumps(self.state, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        for path in (self.folder / 'result.json', self.evidence / 'deployment_latest.json'):
            if path.is_symlink():
                raise ValueError('拒绝覆盖符号链接：' + str(path))
            temp = path.with_name(path.name + '.' + str(os.getpid()) + '.tmp')
            with temp.open('x', encoding='utf-8') as stream:
                os.fchmod(stream.fileno(), 0o644)
                stream.write(data)
                flush_file(stream)
            temp.replace(path)
            sync_directory(path.parent)

    def run(self, label, command, timeout=60, separate_stderr=False):
        print('执行：' + label, flush=True)
        suffix = '.jsonl' if label == 'real_samples' else '.log'
        stdout_path = self.folder / (label + suffix)
        stderr_path = self.folder / (label + '.stderr.log') if separate_stderr else None
        step = {'label': label, 'command': command, 'status': 'STARTING',
                'started_utc': datetime.now(timezone.utc).isoformat(),
                'stdout_path': str(stdout_path)}
        if stderr_path:
            step['stderr_path'] = str(stderr_path)
        self.state['steps'].append(step)
        self.save()
        started = time.monotonic()
        process = None
        with ExitStack() as stack:
            # Direct file descriptors preserve output even if the board hangs
            # before command completion; no subprocess output waits in pipes.
            stdout = stack.enter_context(stdout_path.open('xb', buffering=0))
            stderr = stack.enter_context(stderr_path.open('xb', buffering=0)) if stderr_path else None
            streams = [stdout] + ([stderr] if stderr else [])
            for stream in streams:
                os.fchmod(stream.fileno(), 0o644)
                flush_file(stream)
            sync_directory(self.folder)

            def checkpoint():
                for stream in streams:
                    flush_file(stream)
                step['elapsed_s'] = time.monotonic() - started
                step['last_fsynced_utc'] = datetime.now(timezone.utc).isoformat()
                step['stdout_bytes'] = os.fstat(stdout.fileno()).st_size
                if stderr:
                    step['stderr_bytes'] = os.fstat(stderr.fileno()).st_size
                self.save()

            def terminate_group():
                if process is None:
                    return
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                # A process blocked in a kernel driver may not exit promptly.
                # Preserve that uncertainty instead of waiting without a bound.
                try:
                    step['returncode'] = process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    step['termination_pending'] = True

            try:
                process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                           stdout=stdout, stderr=stderr or subprocess.STDOUT,
                                           start_new_session=True,
                                           env={**os.environ, 'LC_ALL': 'C',
                                                'DEBIAN_FRONTEND': 'noninteractive',
                                                'PYTHONUNBUFFERED': '1'})
                step.update(status='RUNNING', pid=process.pid)
                checkpoint()
                while True:
                    remaining = timeout - (time.monotonic() - started)
                    if remaining <= 0:
                        step['error'] = f'command timed out after {timeout}s'
                        terminate_group()
                        break
                    try:
                        step['returncode'] = process.wait(timeout=min(0.5, remaining))
                        break
                    except subprocess.TimeoutExpired:
                        checkpoint()
            except OSError as exc:
                step['error'] = str(exc)
                terminate_group()
                (stderr or stdout).write(('Deployment command error: ' + str(exc) + '\n').encode())
            except BaseException as exc:
                step.update(status='INTERRUPTED', error=f'{type(exc).__name__}: {exc}')
                terminate_group()
                checkpoint()
                raise
            step['status'] = 'COMPLETED' if step.get('returncode') == 0 and not step.get('error') else 'FAILED'
            checkpoint()
        out = stdout_path.read_text(encoding='utf-8', errors='replace')
        err = stderr_path.read_text(encoding='utf-8', errors='replace') if stderr_path else ''
        if step['status'] != 'COMPLETED':
            raise RuntimeError(f'{label} 失败；日志：{step["stdout_path"]}；{err[-1000:] or out[-1000:]}')
        return out

    def execute(self):
        if os.geteuid() != 0 or platform.machine() != 'aarch64':
            raise ValueError('请在 KR260 上使用 sudo python3 deploy_live.py')
        for executable in ('bash', 'xmutil', 'runuser', 'python3'):
            if shutil.which(executable) is None:
                raise ValueError('缺少命令：' + executable)
        initial = board_state()
        self.state['board_before'] = initial
        current = classify_board(initial)
        self.run('verify_package', ['bash', str(ROOT / 'install_board.sh'), '--verify-package'])
        manifest_path = ROOT / 'firmware/output/manifest.json'
        self.state['firmware_manifest_sha256'] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        self.state['firmware_revision'] = json.loads(manifest_path.read_text())['revision']
        self.save()
        initial_apps = parse_listapps(self.run('listapps_before', ['xmutil', 'listapps'], timeout=20))
        require_application(initial_apps, current, require_target=current == TARGET)
        self.state['apps_before'] = initial_apps
        self.save()
        if current == STOCK:
            self.run('install', ['bash', str(ROOT / 'install_board.sh'), '--install'], timeout=1800)
            rows = parse_listapps(self.run('listapps_preload', ['xmutil', 'listapps'], timeout=20))
            require_application(rows, STOCK, require_target=True)
            # Recheck live state immediately before the only allowed replacement.
            if classify_board(board_state()) != STOCK:
                raise ValueError('安装期间 FPGA 状态变化；停止卸载')
            # Persist apt/firmware installation, its own logs, and our current
            # evidence before any FPGA operation could make Linux unresponsive.
            self.state['pre_unload_sync'] = 'IN_PROGRESS'
            self.save()
            os.sync()
            self.state['pre_unload_sync'] = 'COMPLETED'
            self.state['pre_unload_synced_utc'] = datetime.now(timezone.utc).isoformat()
            self.save()
            self.run('unload_stock_slot0', ['xmutil', 'unloadapp', '0'], timeout=60)
            unloaded = parse_listapps(self.run('listapps_unloaded', ['xmutil', 'listapps'], timeout=20))
            if any(row['active_slots'] for row in unloaded):
                raise ValueError('卸载后仍有活动 slot；停止加载')
            if list(OVERLAY_ROOT.iterdir()):
                raise ValueError('卸载后仍有 overlay；停止加载')
            self.run('load_pressure', ['xmutil', 'loadapp', TARGET], timeout=120)
            self.state['fpga_reloaded'] = True
        else:
            self.state['existing_target_reused'] = True
        final = board_state()
        if classify_board(final) != TARGET:
            raise ValueError('加载后 FPGA/overlay 并非目标固件')
        rows = parse_listapps(self.run('listapps_after', ['xmutil', 'listapps'], timeout=20))
        require_application(rows, TARGET)
        self.state['board_after'] = final
        self.state['apps_after'] = rows
        # This existing script checks hashes, system, DT attributes, character
        # device identity and two named input lines before touching permissions.
        self.run('grant_access', ['bash', str(ROOT / 'install_board.sh'), '--grant-access', TARGET_USER])
        chip = find_gpio_device()
        self.state['verified_chip'] = chip
        self.save()
        self.run('ubuntu_identity', ['runuser', '-u', TARGET_USER, '--', 'id'])
        output = self.run('real_samples', ['runuser', '-u', TARGET_USER, '--', 'python3', '-u',
                         str(ROOT / 'board_reader.py'), '--source', 'real', '--chip', chip,
                         '--offsets', '0', '1', '--rate-hz', '100', '--samples', '100'],
                          timeout=30, separate_stderr=True)
        self.state['acquisition'] = summarize_samples(output, chip)
        self.state['sample_sha256'] = hashlib.sha256(output.encode('utf-8')).hexdigest()
        self.state['status'] = 'REAL_GPIO_ACQUISITION_PASS'
        self.state['completed_utc'] = datetime.now(timezone.utc).isoformat()
        self.save()
        print(json.dumps(self.state['acquisition'], ensure_ascii=False, indent=2), flush=True)
        print('部署与真实 GPIO 读取通过；结果：' + str(self.evidence / 'deployment_latest.json'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    os.umask(0o022)
    job = None
    try:
        job = Deployment()
        job.execute()
        return 0
    except BaseException as exc:
        if job is not None:
            job.state['status'] = 'FAILED'
            job.state['error'] = f'{type(exc).__name__}: {exc}'
            job.state['failed_utc'] = datetime.now(timezone.utc).isoformat()
            try:
                job.state['board_at_failure'] = board_state()
            except Exception as state_error:
                job.state['board_at_failure_error'] = str(state_error)
            job.save()
        print('部署未完成：' + str(exc), file=sys.stderr, flush=True)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == '__main__':
    sys.exit(main())
