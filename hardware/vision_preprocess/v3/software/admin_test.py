#!/usr/bin/env python3
"""KR260 root test entry: verify package, temporarily load VPR3, restore prior app.

This script never stops existing pressure, camera, robot or other applications.
An active PL user is a conflict. Run interactively with sudo; no password handling.
"""
import argparse
import hashlib
import importlib.util
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from xmutil_table import parse_listapps

APP = 'kr260-vision-v3'
HERE = Path(__file__).resolve().parents[1]
COMMAND_LOG = None


def command(args, timeout=120):
    line = '+ ' + ' '.join(map(str, args))
    print(line, flush=True)
    if COMMAND_LOG:
        with COMMAND_LOG.open('a') as log:
            log.write(line + '\n')
            log.flush()
            offset = log.tell()
            result = subprocess.run(list(map(str, args)), stdout=log,
                                    stderr=subprocess.STDOUT, text=True, timeout=timeout)
        with COMMAND_LOG.open() as log:
            log.seek(offset)
            output = log.read()
        print(output, end='', flush=True)
    else:
        result = subprocess.run(list(map(str, args)), capture_output=True, text=True, timeout=timeout)
        output = result.stdout
        print(output, end='', flush=True)
        if result.stderr:
            print(result.stderr, file=sys.stderr, end='', flush=True)
    if result.returncode:
        raise RuntimeError(f'command failed rc={result.returncode}: {args[0]}')
    return output


def active_app(allow_empty=False):
    rows = parse_listapps(command(['xmutil', 'listapps'], 20))
    active = [row for row in rows if row['active_slots']]
    if not active and allow_empty:
        return None
    if len(active) != 1 or active[0]['active_slots'] != [0]:
        raise RuntimeError('Expected exactly one active slot-0 app')
    return active[0]['Accelerator']


def verify_package():
    manifest = json.loads((HERE / 'package_manifest.json').read_text())
    if (manifest['part'] != 'xck26-sfvc784-2LV-c' or manifest['clock_hz'] != 100000000
            or manifest.get('abi_tag') != '0x56505233' or manifest.get('packed_rgb888') is not True):
        raise RuntimeError('Unexpected VPR3 design/ABI')
    required = {'driver/vpr3_dma.c', 'driver/Makefile', 'software/admin_test.py',
                'software/board_test.py', 'software/service.py', 'software/transport.py',
                'software/native_pixels.c', 'software/xmutil_table.py', 'software/geometry.py',
                'firmware/kr260-vision-v3.bit.bin', 'firmware/kr260-vision-v3.dtbo',
                'firmware/shell.json', 'vectors/manifest.json'}
    if not required <= manifest['files'].keys():
        raise RuntimeError('Package manifest does not cover all required execution files')
    for name, expected in manifest['files'].items():
        path = HERE / name
        if (path.is_symlink() or not path.is_file() or HERE not in path.resolve().parents
                or '..' in Path(name).parts):
            raise RuntimeError('Unsafe package file ' + name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Hash mismatch ' + name)
    return manifest


def inspect_pl_users():
    """Read-only preflight; no signals are sent to existing processes."""
    conflicts = []
    device_prefixes = ('/dev/mem', '/dev/uio', '/dev/vpr', '/dev/udmabuf', '/dev/xlnx')
    for process in Path('/proc').iterdir():
        if not process.name.isdecimal() or int(process.name) == os.getpid():
            continue
        try:
            argv = (process / 'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
            reasons = []
            if (b'--verify-pl' in argv and b'--source' in argv
                    and argv[argv.index(b'--source') + 1:argv.index(b'--source') + 2] == [b'real']):
                reasons.append('active pressure reader requesting real PL')
            for descriptor in (process / 'fd').iterdir():
                try:
                    target = os.readlink(descriptor)
                except FileNotFoundError:
                    continue
                if target.startswith(device_prefixes):
                    reasons.append('open ' + target)
            maps = (process / 'maps').read_text()
            for line in maps.splitlines():
                if any(prefix in line for prefix in device_prefixes):
                    reasons.append('mapped PL/MMIO device')
                    break
            if reasons:
                conflicts.append({'pid': int(process.name), 'reasons': sorted(set(reasons))})
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as error:
            raise RuntimeError('Cannot verify active PL users: ' + str(error)) from error
    return conflicts


def require_idle_pl(record):
    conflicts = inspect_pl_users()
    record['conflicting_pl_users'] = conflicts
    if conflicts:
        raise RuntimeError('Active PL/MMIO users; no process was stopped and firmware remains unchanged: '
                           + json.dumps(conflicts))


def install_firmware():
    destination = Path('/lib/firmware/xilinx') / APP
    names = ('kr260-vision-v3.bit.bin', 'kr260-vision-v3.dtbo', 'shell.json')
    if destination.exists():
        for name in names:
            if (destination / name).read_bytes() != (HERE / 'firmware' / name).read_bytes():
                raise RuntimeError('Different existing firmware: ' + str(destination))
        return
    staging = destination.with_name('.' + APP + '.staging-' + str(os.getpid()))
    staging.mkdir(mode=0o755)
    for name in names:
        shutil.copy2(HERE / 'firmware' / name, staging / name)
    staging.rename(destination)


def benchmark_server(args, samples, result_dir, record):
    (HERE / 'benchmark_stop').unlink(missing_ok=True)
    ready = HERE / 'benchmark_ready.json'
    ready.unlink(missing_ok=True)
    server_args = [sys.executable, str(HERE / 'software/service.py'), '--enable-pl',
                   '--port', str(args.port), '--wrist-url', args.wrist_url,
                   '--global-url', args.global_url]
    if args.saved_images:
        server_args += ['--samples', str(samples)]
    with (result_dir / 'benchmark_server.log').open('w') as log:
        server = subprocess.Popen(server_args, stdout=log, stderr=subprocess.STDOUT)
        try:
            for _ in range(50):
                if server.poll() is not None:
                    raise RuntimeError('Benchmark server failed')
                if 'VPR3_SERVICE_READY' in (result_dir / 'benchmark_server.log').read_text():
                    break
                time.sleep(.1)
            else:
                raise RuntimeError('Benchmark server did not become ready')
            ready.write_text(json.dumps({'schema': 'vpr3-ready-v1', 'pid': server.pid,
                'port': args.port, 'duration_s': args.serve_seconds,
                'result_directory': str(result_dir), 'source_kind': record['source_kind']}))
            print('Benchmark ready on board loopback port %d for %d seconds'
                  % (args.port, args.serve_seconds), flush=True)
            until = time.monotonic() + args.serve_seconds
            while time.monotonic() < until:
                if (HERE / 'benchmark_stop').exists():
                    break
                if server.poll() is not None:
                    raise RuntimeError('Benchmark server exited early')
                time.sleep(.25)
        finally:
            if server.poll() is None:
                server.terminate()  # Only the child created by this function.
                server.wait(timeout=10)
            ready.unlink(missing_ok=True)


def main():
    global COMMAND_LOG
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-deps', action='store_true', help='Install small build/OpenCV dependencies only')
    parser.add_argument('--repeats', type=int, default=20)
    parser.add_argument('--saved-images', action='store_true', help='Use recorded JPEGs; never imply live camera acquisition')
    parser.add_argument('--serve-seconds', type=int, default=0)
    parser.add_argument('--port', type=int, default=18768)
    parser.add_argument('--wrist-url', default='http://127.0.0.1:18760/color.jpg')
    parser.add_argument('--global-url', default='http://127.0.0.1:18761/color.jpg')
    args = parser.parse_args()
    if not 0 <= args.serve_seconds <= 600 or not 1 <= args.repeats <= 1000 or not 1024 <= args.port <= 65535:
        parser.error('serve-seconds 0..600; repeats 1..1000; port 1024..65535')
    if os.geteuid() != 0:
        raise RuntimeError('Run with sudo in your board terminal; no passwords are handled here')
    if platform.machine() != 'aarch64':
        raise RuntimeError('KR260 aarch64 only')
    model = Path('/sys/firmware/devicetree/base/model').read_bytes().rstrip(b'\0').decode()
    if 'KR260' not in model:
        raise RuntimeError('Board model mismatch: ' + model)
    manifest = verify_package()
    result_dir = HERE / ('results_' + time.strftime('%Y%m%d_%H%M%S', time.gmtime()))
    result_dir.mkdir()
    COMMAND_LOG = result_dir / 'commands.log'
    record = {'schema': 'vpr3-deployment-v1', 'hardware': 'NOT_TESTED', 'model': model,
              'kernel': platform.release(), 'target_app': APP,
              'source_kind': 'recorded_image' if args.saved_images else 'live_camera',
              'manifest_sha256': hashlib.sha256((HERE / 'package_manifest.json').read_bytes()).hexdigest(),
              'restored_previous_app': False, 'firmware_changed': False,
              'model_action_regression': 'NOT_RUN'}
    previous = None
    changed = driver_attempted = False
    interrupted = False

    def interrupt(signum, frame):
        nonlocal interrupted
        if not interrupted:
            interrupted = True
            raise InterruptedError('Signal %d: restoring prior application' % signum)

    original_handlers = {sig: signal.signal(sig, interrupt) for sig in
                         (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        previous = active_app()
        record['previous_app'] = previous
        if previous not in ('kr260-fsr', 'k26-starter-kits'):
            raise RuntimeError('Refusing to replace unrecognized active app: ' + previous)
        require_idle_pl(record)
        if Path('/sys/module/vpr3_dma').exists():
            raise RuntimeError('VPR3 module already loaded; no existing module will be removed')
        if args.install_deps:
            command(['apt-get', 'update'], 180)
            command(['apt-get', 'install', '-y', '--no-install-recommends', 'build-essential',
                     'linux-headers-' + platform.release(), 'python3-numpy', 'python3-opencv'], 600)
        if importlib.util.find_spec('cv2') is None:
            vendor = Path('/home/ubuntu/grasp-accel-20261023/vendor')
            if not (vendor / 'cv2').is_dir():
                raise RuntimeError('Existing board OpenCV runtime missing')
            os.environ['PYTHONPATH'] = str(vendor) + os.pathsep + os.environ.get('PYTHONPATH', '')
        command([sys.executable, '-c', 'import numpy,cv2; print(numpy.__version__,cv2.__version__)'], 10)
        command(['make', '-C', HERE / 'driver', '-j1'], 240)
        command(['gcc', '-O3', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                 HERE / 'software/native_pixels.c', '-o', HERE / 'software/libvpr3_pixels.so'], 60)
        samples = HERE / ('samples' if args.saved_images else 'samples_live')
        if args.saved_images:
            for role in ('wrist', 'global'):
                relative = 'samples/' + role + '.jpg'
                if relative not in manifest['files']:
                    raise RuntimeError('Recorded sample not covered by package manifest: ' + relative)
        else:
            samples.mkdir(exist_ok=True)
            for role, url in (('wrist', args.wrist_url), ('global', args.global_url)):
                with urllib.request.urlopen(url, timeout=5) as response:
                    jpeg = response.read()
                (samples / (role + '.jpg')).write_bytes(jpeg)
        # Validate all frozen float inputs before changing the active FPGA.
        command([sys.executable, '-c',
                 'import sys; from pathlib import Path; sys.path.insert(0,sys.argv[1]); '
                 'from board_test import fixture_cases; m,c=fixture_cases(Path(sys.argv[2])); '
                 'print("OFFICIAL_FIXTURES_VALID",len(c))', HERE / 'software', HERE / 'vectors'], 30)
        install_firmware()
        # Recheck immediately before changing firmware; never stop another app.
        if active_app() != previous:
            raise RuntimeError('Active application changed during preparation')
        require_idle_pl(record)
        # Restore even if unload/load succeeded but its command was interrupted.
        changed = True
        record['firmware_change_attempted'] = True
        command(['xmutil', 'unloadapp'], 30)
        record['firmware_changed'] = True
        command(['xmutil', 'loadapp', APP], 45)
        if active_app() != APP:
            raise RuntimeError('New application did not become active')
        driver_attempted = True
        command(['insmod', HERE / 'driver/vpr3_dma.ko'], 15)
        if not Path('/dev/vpr3dma').exists():
            raise RuntimeError('VPR3 driver failed to bind; inspect kernel log')
        test = [sys.executable, HERE / 'software/board_test.py', '--vectors', HERE / 'vectors',
                '--output', result_dir / 'tests', '--repeats', str(args.repeats)]
        for path in sorted(samples.glob('*.jpg')):
            test += ['--jpeg', path]
        command(test, 900)
        evidence = json.loads((result_dir / 'tests/hardware_results.json').read_text())
        if evidence['status'] != 'VPR3_HARDWARE_PASS':
            raise RuntimeError('Test result did not pass')
        record['hardware'] = 'HARDWARE_PASS'
        if args.serve_seconds:
            benchmark_server(args, samples, result_dir, record)
    except BaseException as error:
        record['error'] = str(error)
        raise
    finally:
        # A second terminal signal must not interrupt the bounded restoration.
        interrupted = True
        try:
            if changed:
                conflicts = inspect_pl_users()
                record['pre_restore_conflicting_pl_users'] = conflicts
                if conflicts:
                    raise RuntimeError('PL/MMIO users remain active; driver and firmware will not be unloaded: '
                                       + json.dumps(conflicts))
            if driver_attempted and Path('/sys/module/vpr3_dma').exists():
                # .owner pins the module until every fd and mmap reference closes.
                # rmmod is never forced; failure prevents subsequent FPGA unload.
                command(['rmmod', 'vpr3_dma'], 15)
            if changed:
                current = active_app(allow_empty=True)
                if current not in (None, APP, previous):
                    raise RuntimeError('Another application became active during the test: ' + current)
                if current == APP:
                    command(['xmutil', 'unloadapp'], 30)
                if current != previous:
                    command(['xmutil', 'loadapp', previous], 45)
                record['restored_previous_app'] = active_app() == previous
                if not record['restored_previous_app']:
                    raise RuntimeError('Previous app restoration failed')
            else:
                record['restoration'] = 'NOT_REQUIRED_FIRMWARE_UNCHANGED'
        except BaseException as error:
            record['restore_error'] = str(error)
            print('RESTORE FAILED: ' + str(error), file=sys.stderr)
        (result_dir / 'deployment_result.json').write_text(json.dumps(record, indent=2) + '\n')
        if os.environ.get('SUDO_UID'):
            uid, gid = int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID'])
            for path in [result_dir, *result_dir.rglob('*')]:
                os.chown(path, uid, gid)
        for sig, handler in original_handlers.items():
            signal.signal(sig, handler)
        print('Results: ' + str(result_dir), flush=True)
    if changed and not record['restored_previous_app']:
        raise RuntimeError('Prior application was not restored')


if __name__ == '__main__':
    main()
