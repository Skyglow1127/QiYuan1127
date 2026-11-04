#!/usr/bin/env python3
"""Deploy/start/status/stop the localhost-only PI05 service over verified SSH.

Normal start prompts for the existing SSH password in the terminal. Passwords
are never saved. No keys are generated and no cloud instance is shut down.
Only this deployment's PID (verified by startup UUID and full argv) is stopped.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import getpass
import errno
import json
import os
from pathlib import Path
import shlex
import signal
import selectors
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
import uuid

HERE = Path(__file__).resolve().parent
PROTOCOL = 'pi05-cloud-grasp/v1'
FILES = ('server.py', 'pi05_low_memory.py', 'cloud_control.py', 'cloud_config.json')


def read_config(path):
    config = json.loads(path.read_text())
    if config['protocol_version'] != PROTOCOL:
        raise ValueError('Unsupported configured protocol.')
    for field in ('remote_dir', 'remote_python', 'base', 'adapter', 'tokenizer'):
        if not Path(config[field]).is_absolute():
            raise ValueError(f'{field} must be absolute.')
    for field in ('local_port', 'remote_port'):
        if not 1024 <= int(config[field]) <= 65535:
            raise ValueError(f'{field} must be a nonprivileged TCP port.')
    return config


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def health(port, timeout=2):
    try:
        # Ignore HTTP proxy environment variables for a localhost-only tunnel.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(f'http://127.0.0.1:{port}/health', timeout=timeout) as response:
            return json.load(response)
    except (OSError, ValueError, urllib.error.URLError):
        return None


def validate_health(document, config, startup_id=None):
    if not document or document.get('protocol_version') != PROTOCOL:
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


def remote_state_path(config):
    return Path(config['remote_dir']) / 'control-state.json'


def pid_matches(state, config):
    if not state or not isinstance(state.get('pid'), int):
        return False
    try:
        argv = Path(f"/proc/{state['pid']}/cmdline").read_bytes().split(b'\0')
        actual = [value.decode() for value in argv if value]
        expected = [str(Path(config['remote_dir']) / 'server.py'), '--config',
                    str(Path(config['remote_dir']) / 'cloud_config.json'),
                    '--startup-id', state['startup_id']]
        return len(actual) == len(expected) + 1 and actual[1:] == expected
    except (OSError, KeyError, UnicodeError):
        return False


def remote_status(config):
    path = remote_state_path(config)
    try:
        state = json.loads(path.read_text())
    except FileNotFoundError:
        state = None
    matches = pid_matches(state, config)
    endpoint = health(config['remote_port'])
    valid = False
    error = None
    if matches and endpoint:
        try:
            validate_health(endpoint, config, state['startup_id'])
            valid = True
        except RuntimeError as issue:
            error = str(issue)
    return {'active': matches, 'ready': valid,
            'status': 'ready' if valid else 'starting' if matches else 'stopped',
            'state': state, 'health': endpoint, 'identity_error': error,
            'cloud_instance_powered_on': True}


@contextmanager
def remote_lock(config):
    import fcntl
    root = Path(config['remote_dir'])
    root.mkdir(parents=True, exist_ok=True)
    with (root / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def remote_start(config):
    with remote_lock(config):
        current = remote_status(config)
        if current['active']:
            if current['identity_error']:
                raise RuntimeError(current['identity_error'])
            current['reused_existing_process'] = True
            return current
        try:
            with socket.create_connection(('127.0.0.1', int(config['remote_port'])), timeout=1):
                raise RuntimeError('Remote port is occupied by an unmanaged process; nothing was stopped.')
        except ConnectionRefusedError:
            pass
        root = Path(config['remote_dir'])
        for name in FILES:
            if not (root / name).is_file():
                raise RuntimeError(f'Deployment file is missing: {name}')
        startup_id = str(uuid.uuid4())
        command = [config['remote_python'], str(root / 'server.py'), '--config',
                   str(root / 'cloud_config.json'), '--startup-id', startup_id]
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        environment.update({'PYTHONNOUSERSITE': '1', 'HF_HUB_OFFLINE': '1',
                            'TRANSFORMERS_OFFLINE': '1', 'HF_DATASETS_OFFLINE': '1',
                            'HF_HUB_DISABLE_TELEMETRY': '1', 'TOKENIZERS_PARALLELISM': 'false'})
        with (root / 'server.log').open('ab', buffering=0) as log:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, env=environment,
                                       cwd=root, start_new_session=True)
        state = {'pid': process.pid, 'startup_id': startup_id, 'command': command,
                 'started_utc': datetime.now(timezone.utc).isoformat(),
                 'remote_dir': str(root), 'port': int(config['remote_port'])}
        atomic_json(remote_state_path(config), state)
        return {'active': True, 'ready': False, 'status': 'starting', 'state': state,
                'reused_existing_process': False, 'cloud_instance_powered_on': True}


def remote_stop(config):
    with remote_lock(config):
        current = remote_status(config)
        state = current['state']
        if not current['active']:
            return {'stopped': True, 'already_stopped': True,
                    'cloud_instance_powered_on': True, 'unmanaged_processes_touched': False}
        # pid_matches checks both deployment path and the unique startup UUID.
        if not pid_matches(state, config):
            raise RuntimeError('Managed process identity changed; refusing to signal it.')
        os.kill(state['pid'], signal.SIGTERM)
        deadline = time.monotonic() + 10
        while pid_matches(state, config) and time.monotonic() < deadline:
            time.sleep(0.2)
        if pid_matches(state, config):
            raise RuntimeError('Managed process did not stop after SIGTERM; no other process was signalled.')
        state['stopped_utc'] = datetime.now(timezone.utc).isoformat()
        atomic_json(remote_state_path(config), state)
        return {'stopped': True, 'pid': state['pid'], 'startup_id': state['startup_id'],
                'cloud_instance_powered_on': True, 'unmanaged_processes_touched': False}


class SSH:
    def __init__(self, config, master=None):
        self.config = config
        self.spec = config['ssh']
        self.runtime = HERE / 'runtime'
        self.runtime.mkdir(mode=0o700, exist_ok=True)
        os.chmod(self.runtime, 0o700)
        self.external = master is not None
        self.socket = Path(master).resolve() if master else self.runtime / 'cloud.sock'
        if len(str(self.socket).encode()) >= 100:
            raise ValueError('SSH control socket path is too long; use --master with a shorter path.')
        self.created = False
        self.forward_added = False
        self.destination = f"{self.spec['user']}@{self.spec['host']}"
        self.base = ['ssh', '-b', self.spec['bind_address'], '-p', str(self.spec['port']),
                     '-o', f"HostKeyAlias={self.spec['host_key_alias']}",
                     '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=15',
                     '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3',
                     '-o', 'ExitOnForwardFailure=yes', '-S', str(self.socket)]

    def check(self):
        result = subprocess.run(self.base + ['-O', 'check', self.destination],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return result.returncode == 0

    def connect(self):
        if self.check():
            return
        if self.external:
            raise RuntimeError('The supplied external SSH master is not running.')
        if self.socket.exists():
            self.socket.unlink()
        command = self.base + ['-M', '-f', '-N', '-o', 'ControlPersist=yes',
                               '-o', 'NumberOfPasswordPrompts=1', self.destination]
        # Standard-library PTY keeps SSH's credential input off argv, files and
        # environment variables. No pexpect/sshpass dependency or saved secret.
        import fcntl
        import termios
        master_fd, slave_fd = os.openpty()

        def make_controlling_terminal():
            os.setsid()
            fcntl.ioctl(slave_fd, termios.TIOCSCTTY, 0)

        child = subprocess.Popen(command, stdin=slave_fd, stdout=slave_fd,
                                 stderr=slave_fd, preexec_fn=make_controlling_terminal)
        os.close(slave_fd)
        selector = selectors.DefaultSelector()
        selector.register(master_fd, selectors.EVENT_READ)
        transcript = b''
        supplied = False
        deadline = time.monotonic() + 45
        try:
            while time.monotonic() < deadline:
                for _, _ in selector.select(timeout=0.2):
                    try:
                        block = os.read(master_fd, 4096)
                    except OSError as issue:
                        if issue.errno != errno.EIO:
                            raise
                        block = b''
                    transcript = (transcript + block)[-8192:]
                    if b'password:' in transcript.lower() and not supplied:
                        credential = getpass.getpass('云端 SSH 密码（不会保存）: ')
                        try:
                            os.write(master_fd, credential.encode() + b'\n')
                        finally:
                            del credential
                        transcript = b''
                        supplied = True
                        deadline = time.monotonic() + 30
                if child.poll() is not None:
                    break
            if child.poll() is None:
                raise RuntimeError('SSH connection or authentication timed out.')
            if child.returncode != 0 or not self.check():
                raise RuntimeError('SSH did not authenticate. Check the password, physical bind address and verified host key.')
            self.created = True
        finally:
            selector.close()
            os.close(master_fd)
            transcript = b''
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=5)

    def command(self, argv, timeout=30):
        result = subprocess.run(self.base + ['-o', 'BatchMode=yes', self.destination,
                                            shlex.join([str(value) for value in argv])],
                                capture_output=True, text=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError(f'SSH remote command failed: {result.stderr.strip() or result.stdout.strip()}')
        return result.stdout.strip()

    def remote(self, action):
        root = Path(self.config['remote_dir'])
        result = self.command([self.config['remote_python'], root / 'cloud_control.py',
                               action, '--config', root / 'cloud_config.json'], timeout=30)
        return json.loads(result)

    def remote_installed(self):
        root = Path(self.config['remote_dir'])
        result = self.command(['test', '-f', root / 'cloud_control.py'])
        return result == ''

    def prepare(self, config_path):
        root = Path(self.config['remote_dir'])
        # Do not overwrite a deployment while its managed process is alive.
        program = (
            'import json,pathlib,sys; p=pathlib.Path(sys.argv[1]); '
            's=json.loads(p.read_text()) if p.exists() else {}; '
            'pid=s.get("pid"); '
            'live=bool(pid and pathlib.Path(f"/proc/{pid}/cmdline").exists() '
            'and pathlib.Path(f"/proc/{pid}/cmdline").read_bytes()); '
            'sys.exit("Deployment process is alive; stop it before prepare.") if live else None')
        self.command([self.config['remote_python'], '-c', program, root / 'control-state.json'])
        self.command(['mkdir', '-p', '--', root])
        sources = [HERE / name for name in FILES if name != 'cloud_config.json']
        # Custom config is always installed under the stable remote basename.
        for source in sources + [config_path]:
            target_name = 'cloud_config.json' if source == config_path else source.name
            scp = ['scp', '-q', '-P', str(self.spec['port']), '-o', f'ControlPath={self.socket}',
                   '-o', 'ControlMaster=no', '-o', 'BatchMode=yes',
                   '-o', f"HostKeyAlias={self.spec['host_key_alias']}",
                   '-o', 'StrictHostKeyChecking=yes', str(source),
                   f'{self.destination}:{shlex.quote(str(root / target_name))}']
            result = subprocess.run(scp, capture_output=True, text=True, timeout=30)
            if result.returncode:
                raise RuntimeError(f'SCP upload failed: {result.stderr.strip()}')

    def ensure_forward(self, expected_startup_id):
        local = health(self.config['local_port'])
        if local:
            validate_health(local, self.config, expected_startup_id)
            return False
        # Fail on any occupied local port, even if it does not speak HTTP.
        with socket.socket() as probe:
            try:
                probe.bind(('127.0.0.1', int(self.config['local_port'])))
            except OSError as issue:
                if self.external:
                    # An explicitly supplied master may already own its -L
                    # listener before the cold model starts. Never replace it;
                    # start still requires health + exact startup UUID below.
                    return False
                raise RuntimeError('Local port is occupied; no process or tunnel was stopped.') from issue
        result = subprocess.run(self.base + ['-O', 'forward', '-L', self.forward_spec(),
                                            self.destination], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(f'Could not create SSH tunnel: {result.stderr.strip()}')
        self.forward_added = True
        return True

    def forward_spec(self):
        return f"127.0.0.1:{self.config['local_port']}:127.0.0.1:{self.config['remote_port']}"

    def close_added_forward(self):
        if self.forward_added and self.check():
            subprocess.run(self.base + ['-O', 'cancel', '-L', self.forward_spec(), self.destination],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.forward_added = False

    def close(self):
        if not self.external and self.check():
            subprocess.run(self.base + ['-O', 'exit', self.destination],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def local_action(args, config):
    ssh = SSH(config, args.master)
    keep_connection = False
    started_new_process = False
    try:
        ssh.connect()
        if args.action == 'prepare':
            ssh.prepare(args.config)
            return {'prepared': True, 'remote_dir': config['remote_dir'],
                    'robot_io': False, 'cloud_instance_powered_on': True}
        if args.action == 'status':
            try:
                return ssh.remote('_remote-status')
            except RuntimeError as issue:
                return {'ready': False, 'error': str(issue), 'cloud_instance_powered_on': True}
        if args.action == 'stop':
            result = ssh.remote('_remote-stop')
            session_path = ssh.runtime / 'session.json'
            if session_path.exists():
                session = json.loads(session_path.read_text())
                if session.get('control_path') == str(ssh.socket) and session.get('forward_added'):
                    ssh.forward_added = True
            managed_forward = ssh.forward_added
            if ssh.external:
                ssh.close_added_forward()
            else:
                ssh.close()
            atomic_json(session_path, {'stopped': True, 'control_path': str(ssh.socket),
                                       'cloud_instance_powered_on': True})
            result['tunnel_closed'] = not ssh.external or managed_forward
            result['external_master_retained'] = ssh.external
            return result
        # start is idempotent: reuse this exact healthy process, otherwise deploy.
        try:
            existing = ssh.remote('_remote-status')
        except (RuntimeError, ValueError):
            existing = {'active': False}
        if not existing.get('active'):
            ssh.prepare(args.config)
        start = ssh.remote('_remote-start')
        started_new_process = not start.get('reused_existing_process', False)
        startup_id = start['state']['startup_id']
        ssh.ensure_forward(startup_id)
        deadline = time.monotonic() + float(config.get('startup_timeout_seconds', 180))
        while time.monotonic() < deadline:
            document = health(config['local_port'])
            if document:
                validate_health(document, config, startup_id)
                session_path = ssh.runtime / 'session.json'
                previous = json.loads(session_path.read_text()) if session_path.exists() else {}
                forward_owned = ssh.forward_added or (
                    previous.get('control_path') == str(ssh.socket) and previous.get('forward_added', False))
                atomic_json(session_path, {'control_path': str(ssh.socket),
                                          'external_master': ssh.external,
                                          'forward_added': forward_owned,
                                          'startup_id': startup_id,
                                          'local_url': f"http://127.0.0.1:{config['local_port']}",
                                          'started_utc': datetime.now(timezone.utc).isoformat()})
                keep_connection = True
                return {'ready': True, 'warmed_up': True,
                        'local_url': f"http://127.0.0.1:{config['local_port']}",
                        'state': start['state'], 'health': document, 'robot_io': False,
                        'reused_existing_process': start.get('reused_existing_process', False),
                        'cloud_instance_powered_on': True}
            status = ssh.remote('_remote-status')
            if not status.get('active'):
                raise RuntimeError('Cloud model exited during startup. Inspect remote server.log; no robot was controlled.')
            time.sleep(2)
        raise RuntimeError('Model startup timed out; inspect remote server.log.')
    except Exception:
        if started_new_process:
            try:
                ssh.remote('_remote-stop')
            except Exception as issue:
                print(f'Cleanup could not confirm model stop: {issue}', file=sys.stderr)
        raise
    finally:
        if not keep_connection:
            ssh.close_added_forward()
            if ssh.created:
                ssh.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'start', 'status', 'stop',
                                            '_remote-start', '_remote-status', '_remote-stop'))
    parser.add_argument('--config', type=Path, default=HERE / 'cloud_config.json')
    parser.add_argument('--master', type=Path,
                        help='Reuse an already authenticated SSH control master without closing it.')
    args = parser.parse_args()
    args.config = args.config.resolve()
    try:
        config = read_config(args.config)
        if args.action.startswith('_remote-'):
            if HERE != Path(config['remote_dir']).resolve():
                raise RuntimeError('Remote lifecycle commands only run in the configured cloud deployment directory.')
            result = {'_remote-start': remote_start, '_remote-status': remote_status,
                      '_remote-stop': remote_stop}[args.action](config)
        else:
            result = local_action(args, config)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
