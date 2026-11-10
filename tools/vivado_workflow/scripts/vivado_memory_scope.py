"""Place only the newly started Vivado process tree in a bounded user scope."""
from pathlib import Path
import os
import subprocess
import time


def protect_vivado(pid, memory_max_gib=5):
    # 3 GiB is reserved for the explicitly approved, empty GUI language trial.
    # Existing KR260 build callers retain the 5 GiB default.
    # VPR2 is a larger HLS/PS/DMA design using the normal 8/6 GiB gates.
    if memory_max_gib not in (3, 5, 8):
        raise ValueError('Expected 3 GiB GUI trial, 5 GiB small KR260, or 8 GiB VPR2 scope')
    pid = int(pid)
    if pid <= 1 or Path(f'/proc/{pid}').stat().st_uid != os.getuid():
        raise RuntimeError('Refusing to manage an unrelated process')
    parents = {}
    for proc in Path('/proc').iterdir():
        if proc.name.isdigit():
            try:
                status = (proc/'status').read_text()
                ppid = int(next(line for line in status.splitlines() if line.startswith('PPid:')).split()[1])
                parents[int(proc.name)] = ppid
            except (OSError, StopIteration):
                continue
    selected = {pid}
    while True:
        children = {p for p, parent in parents.items() if parent in selected}
        if children <= selected:
            break
        selected.update(children)
    unit = f'ross-kr260-{pid}.scope'
    command = ['busctl', '--user', 'call', 'org.freedesktop.systemd1',
               '/org/freedesktop/systemd1', 'org.freedesktop.systemd1.Manager',
               'StartTransientUnit', 'ssa(sv)a(sa(sv))', unit, 'fail', '4',
               'Description', 's', 'Ross KR260 Vivado and build children',
               'PIDs', 'au', str(len(selected)), *map(str, sorted(selected)),
               'MemoryMax', 't', str(memory_max_gib * 1024**3),
               'MemorySwapMax', 't', str(1024**3), '0']
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError('Cannot isolate new Vivado process: ' + result.stderr.strip())
    for _ in range(30):
        group = Path(f'/proc/{pid}/cgroup').read_text().strip().split('::', 1)[1]
        if group.endswith('/' + unit):
            base = Path('/sys/fs/cgroup') / group.lstrip('/')
            if int((base/'memory.max').read_text()) != memory_max_gib * 1024**3:
                raise RuntimeError('Memory limit readback mismatch')
            if int((base/'memory.swap.max').read_text()) != 1024**3:
                raise RuntimeError('Swap limit readback mismatch')
            return {'unit': unit, 'cgroup': str(base), 'memory_max_gib': memory_max_gib,
                    'swap_max_gib': 1, 'initial_pids': sorted(selected)}
        time.sleep(0.1)
    raise RuntimeError('Vivado memory scope did not become active; compilation not started')
