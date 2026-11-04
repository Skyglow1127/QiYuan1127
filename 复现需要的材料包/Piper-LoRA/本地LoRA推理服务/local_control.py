#!/usr/bin/env python3
"""Prepare, start, inspect or stop the separate localhost PI05 inference process.

Only the PID with the recorded command and startup UUID may be stopped.
This controller never opens a robot, camera or CAN interface.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.error
import urllib.request
import uuid

HERE = Path(__file__).resolve().parent
PROTOCOL = 'pi05-local-grasp/v1'


def read_config(path):
    path = Path(path).expanduser().resolve()
    config = json.loads(path.read_text())
    if config['protocol_version'] != PROTOCOL:
        raise ValueError('Unsupported configured protocol.')
    if isinstance(config['local_port'], bool) or not isinstance(config['local_port'], int) or not 1024 <= config['local_port'] <= 65535:
        raise ValueError('local_port must be a nonprivileged TCP port.')
    for field in ('python', 'base', 'adapter', 'tokenizer'):
        location = Path(config[field]).expanduser()
        if not location.is_absolute():
            raise ValueError(f'{field} must be absolute.')
        config[field] = str(location)
    runtime = Path(config.get('runtime_dir', 'runtime')).expanduser()
    config['runtime_dir'] = str((path.parent / runtime).resolve())
    config['config_path'] = str(path)
    timeout = config.get('startup_timeout_seconds', 180)
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not 0 < timeout <= 3600:
        raise ValueError('startup_timeout_seconds must be in (0,3600].')
    return config


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def health(port, timeout=2):
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(f'http://127.0.0.1:{port}/health', timeout=timeout) as response:
            return json.load(response)
    except (OSError, ValueError, urllib.error.URLError):
        return None


def validate_health(document, config, startup_id=None):
    if not isinstance(document, dict) or document.get('protocol_version') != PROTOCOL:
        raise RuntimeError('Endpoint is not the expected PI05 protocol.')
    metadata = document.get('metadata', {})
    if not document.get('ready') or not document.get('warmed_up'):
        raise RuntimeError('Model is not ready and warmed up.')
    expected = {'model_id': config['model_id'],
                'base_sha256': config['expected_base_sha256'],
                'adapter_sha256': config['expected_adapter_sha256']}
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise RuntimeError(f'Health model identity mismatch: {key}')
    if startup_id and metadata.get('startup_id') != startup_id:
        raise RuntimeError('Health startup UUID differs from the managed PID.')
    return metadata


def state_path(config):
    return Path(config['runtime_dir']) / 'control-state.json'


def read_state(config):
    try:
        value = json.loads(state_path(config).read_text())
        return value if isinstance(value, dict) else None
    except (FileNotFoundError, ValueError):
        return None


def pid_matches(state, config):
    if not state or type(state.get('pid')) is not int:
        return False
    try:
        argv = Path(f"/proc/{state['pid']}/cmdline").read_bytes().split(b'\0')
        actual = [value.decode() for value in argv if value]
        expected = [str(HERE / 'server.py'), '--config', config['config_path'],
                    '--startup-id', state['startup_id']]
        return len(actual) == len(expected) + 1 and actual[1:] == expected
    except (OSError, KeyError, UnicodeError):
        return False


def local_status(config):
    state = read_state(config)
    matches = pid_matches(state, config)
    endpoint = health(config['local_port'])
    valid, error = False, None
    if matches and endpoint:
        try:
            validate_health(endpoint, config, state['startup_id'])
            valid = True
        except RuntimeError as issue:
            error = str(issue)
    return {'active': matches, 'ready': valid,
            'status': 'ready' if valid else 'starting' if matches else 'stopped',
            'state': state, 'health': endpoint, 'identity_error': error,
            'robot_io': False}


@contextmanager
def process_lock(config):
    import fcntl
    runtime = Path(config['runtime_dir'])
    runtime.mkdir(parents=True, mode=0o700, exist_ok=True)
    with (runtime / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def prepare(config):
    for path in (HERE / 'server.py', HERE / 'pi05_low_memory.py',
                 Path(config['python']), Path(config['base']) / 'model.safetensors',
                 Path(config['adapter']) / 'adapter_model.safetensors',
                 Path(config['adapter']) / 'config.json',
                 Path(config['tokenizer']) / 'tokenizer_config.json'):
        if not path.is_file():
            raise RuntimeError(f'Required local file is missing: {path}')
    if not os.access(config['python'], os.X_OK):
        raise RuntimeError('Configured Python interpreter is not executable.')
    return {'prepared': True, 'robot_io': False, 'runtime_dir': config['runtime_dir']}


def local_start(config):
    with process_lock(config):
        current = local_status(config)
        if current['active']:
            if current['identity_error']:
                raise RuntimeError(current['identity_error'])
            current['reused_existing_process'] = True
            return current
        prepare(config)
        try:
            with socket.create_connection(('127.0.0.1', config['local_port']), timeout=1):
                raise RuntimeError('Local port is occupied by an unmanaged process; nothing was stopped.')
        except ConnectionRefusedError:
            pass
        runtime = Path(config['runtime_dir'])
        startup_id = str(uuid.uuid4())
        command = [config['python'], str(HERE / 'server.py'), '--config',
                   config['config_path'], '--startup-id', startup_id]
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        environment.update({'PYTHONNOUSERSITE': '1', 'HF_HUB_OFFLINE': '1',
                            'TRANSFORMERS_OFFLINE': '1', 'HF_DATASETS_OFFLINE': '1',
                            'HF_HUB_DISABLE_TELEMETRY': '1', 'TOKENIZERS_PARALLELISM': 'false'})
        with (runtime / 'server.log').open('ab', buffering=0) as log:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, env=environment,
                                       cwd=HERE, start_new_session=True)
        state = {'pid': process.pid, 'startup_id': startup_id, 'command': command,
                 'started_utc': datetime.now(timezone.utc).isoformat(),
                 'runtime_dir': str(runtime), 'port': config['local_port']}
        atomic_json(state_path(config), state)
        return {'active': True, 'ready': False, 'status': 'starting', 'state': state,
                'reused_existing_process': False, 'robot_io': False}


def local_stop(config, expected_state=None):
    with process_lock(config):
        state = read_state(config)
        if expected_state is not None and (not state or any(
                state.get(key) != expected_state.get(key) for key in ('pid', 'startup_id'))):
            return {'stopped': False, 'managed_process_replaced': True,
                    'unmanaged_processes_touched': False, 'robot_io': False}
        if not pid_matches(state, config):
            return {'stopped': True, 'already_stopped': True,
                    'unmanaged_processes_touched': False, 'robot_io': False}
        os.kill(state['pid'], signal.SIGTERM)
        deadline = time.monotonic() + 10
        while pid_matches(state, config) and time.monotonic() < deadline:
            time.sleep(0.2)
        if pid_matches(state, config):
            raise RuntimeError('Managed process did not stop after SIGTERM; no other process was signalled.')
        state['stopped_utc'] = datetime.now(timezone.utc).isoformat()
        atomic_json(state_path(config), state)
        return {'stopped': True, 'pid': state['pid'], 'startup_id': state['startup_id'],
                'unmanaged_processes_touched': False, 'robot_io': False}


def start_and_wait(config):
    started = local_start(config)
    deadline = time.monotonic() + config.get('startup_timeout_seconds', 180)
    try:
        while time.monotonic() < deadline:
            status = local_status(config)
            if not status['state'] or any(status['state'].get(key) != started['state'].get(key)
                    for key in ('pid', 'startup_id')):
                raise RuntimeError('Managed startup was replaced; the new process was left untouched.')
            if status['identity_error']:
                raise RuntimeError(status['identity_error'])
            if not status['active']:
                raise RuntimeError('Local model exited during startup. Inspect runtime/server.log.')
            if status['ready']:
                status.update({'local_url': f"http://127.0.0.1:{config['local_port']}",
                               'reused_existing_process': started['reused_existing_process']})
                return status
            time.sleep(0.5)
        raise RuntimeError('Model startup timed out; inspect runtime/server.log.')
    except BaseException:
        if not started['reused_existing_process']:
            local_stop(config, expected_state=started['state'])
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'start', 'status', 'stop'))
    parser.add_argument('--config', type=Path, default=HERE / 'local_config.json')
    args = parser.parse_args()
    config = read_config(args.config)
    result = {'prepare': prepare, 'start': start_and_wait,
              'status': local_status, 'stop': local_stop}[args.action](config)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        raise SystemExit(f'Local inference control failed: {error}')
