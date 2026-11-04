#!/usr/bin/env python3
"""One-shot local watcher + verified B backup, using an existing SSH master.

Never starts/retries/stops training, controls hardware, removes remote files,
or changes model A. Run this foreground, or detach with ordinary nohup.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import sys
import time


def now():
    return datetime.now(timezone.utc).isoformat()


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def safe_relative(name):
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or any(part in ("..", "") for part in path.parts) or "\x00" in name:
        raise RuntimeError(f"Unsafe relative backup file: {name!r}")
    return path


REMOTE_QUERY = r'''
import hashlib,json,os
from pathlib import Path
CONFIG=__CONFIG__
root=Path('/root/autodl-tmp/piper_pi05/experiments')/CONFIG['experiment_id']
scratch=Path('/root/pi05_expert_runs')/CONFIG['experiment_id']
status_path=root/'job_status.json'
status=json.loads(status_path.read_text())
if status.get('experiment_id')!=CONFIG['experiment_id']:
    raise RuntimeError('Remote experiment identity mismatch')
if status.get('data_root')!=str(root) or status.get('scratch_root')!=str(scratch):
    raise RuntimeError('Unexpected remote experiment paths')
result={'status':status}
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024**2),b''):h.update(block)
    return h.hexdigest()
if CONFIG['manifest']:
    if status.get('stage') not in ('complete','failed'):
        raise RuntimeError('Metadata backup requires a terminal experiment status')
    sources={'metadata':root}
    for phase in ('smoke','train'):
        candidate=scratch/phase/'audit'
        if candidate.is_dir():sources[phase+'_audit']=candidate
    manifests={}
    for label,source in sources.items():
        files={}
        for current,dirs,names in os.walk(source,followlinks=False):
            directory=Path(current)
            dirs[:]=[name for name in dirs if not (directory/name).is_symlink()
                     and name not in ('best_model','best_model.staging','best_model.previous','checkpoints')]
            for name in names:
                path=directory/name
                if path.is_symlink() or not path.is_file():continue
                # Status is returned as an explicit read-only JSON snapshot.
                if path==status_path:continue
                if path.stat().st_size>256*1024**2:
                    raise RuntimeError('Unexpected large metadata file: '+str(path))
                files[str(path.relative_to(source))]={'bytes':path.stat().st_size,'sha256':digest(path)}
        manifests[label]={'remote_root':str(source),'files':files}
    result['metadata_manifests']=manifests
    if status.get('stage')=='complete':
        expected=scratch/'train/checkpoints/020000'
        if not status.get('training_complete') or not status.get('model_A_unchanged') or status.get('final_checkpoint')!=str(expected):
            raise RuntimeError('Remote completion identity/integrity checks did not pass')
        receipt_path=root/'formal_audit/checkpoint_020000.json'
        final_receipt=json.loads(receipt_path.read_text())
        if final_receipt.get('step')!=20000 or not final_receipt.get('exact_cpu_tensor_reload_verified') or not final_receipt.get('complete_frozen_vlm_sha256_unchanged'):
            raise RuntimeError('Final checkpoint receipt did not pass required audits')
        best_path=root/'best_model_receipt.json'
        best=json.loads(best_path.read_text())
        if best.get('model_dir')!=str(root/'best_model') or not best.get('sha256_verified'):
            raise RuntimeError('Best-model receipt identity mismatch')
        best_checkpoint=root/'formal_audit'/('checkpoint_%06d.json'%best['step'])
        best_receipt=json.loads(best_checkpoint.read_text())
        best_files={name[len('pretrained_model/'):]:value
                    for name,value in best_receipt['artifacts'].items()
                    if name.startswith('pretrained_model/')}
        if best_files['model.safetensors']['sha256']!=best['source_model_sha256']:
            raise RuntimeError('Best-model and checkpoint receipt SHA mismatch')
        result['models']={
            'final_checkpoint':{'remote_root':str(expected),'files':final_receipt['artifacts'],
                'receipt_relative':'formal_audit/checkpoint_020000.json','receipt_sha256':digest(receipt_path)},
            'best_model':{'remote_root':str(root/'best_model'),'files':best_files,
                'receipt_relative':str(best_checkpoint.relative_to(root)),
                'receipt_sha256':digest(best_checkpoint),'selected_step':best['step'],
                'best_receipt_sha256':digest(best_path)}}
print(json.dumps(result))
'''


class Watcher:
    def __init__(self, args):
        self.args = args
        self.root = Path(args.backup_dir).resolve()
        protected = Path(__file__).resolve().parent.parent / "piper_pi05_cloud_backup_v5"
        if self.root == protected or self.root.is_relative_to(protected):
            raise RuntimeError("Refusing to write inside original model A backup")
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = (self.root / "watcher.lock").open("a")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        identity = self.root / "backup_identity.json"
        if identity.exists():
            if json.loads(identity.read_text()).get("experiment_id") != args.experiment_id:
                raise RuntimeError("Backup directory belongs to another experiment")
        elif any(p.name not in ("watcher.lock", "watcher.log") for p in self.root.iterdir()):
            raise RuntimeError("Existing backup directory lacks this experiment's identity marker")
        else:
            save(identity, {"experiment_id": args.experiment_id, "created_utc": now(),
                            "model": "B expert-only", "original_model_A_modified": False})
        self.target = f"{args.user}@{args.host}"
        # ProxyCommand=false makes a vanished control master fail closed rather
        # than initiating an independent network/authentication connection.
        self.ssh = ["ssh", "-S", str(Path(args.socket).resolve()), "-p", str(args.port),
                    "-o", "ControlMaster=no", "-o", "BatchMode=yes", "-o", "ProxyCommand=false",
                    "-o", "ConnectTimeout=8", "-o", "LogLevel=ERROR", "-T"]
        self.state = {"experiment_id": args.experiment_id, "watcher_pid": os.getpid(),
                      "started_utc": now(), "robot_operations": False, "remote_mutations": False,
                      "backup_root": str(self.root), "poll_count": 0}

    def status(self, stage, **details):
        self.state.update(stage=stage, updated_utc=now(), **details)
        save(self.root / "watcher_status.json", self.state)
        print(json.dumps({"at": now(), "stage": stage, **details}, ensure_ascii=False), flush=True)

    def master_check(self):
        check = subprocess.run(self.ssh + ["-O", "check", self.target], capture_output=True, text=True, timeout=15)
        if check.returncode:
            raise RuntimeError("Existing SSH master unavailable; no new login attempted: " + check.stderr.strip())

    def query(self, manifest=False):
        self.master_check()
        config = {"experiment_id": self.args.experiment_id, "manifest": manifest}
        program = REMOTE_QUERY.replace("__CONFIG__", repr(config))
        command = shlex.join([self.args.remote_python, "-c", program])
        result = subprocess.run(self.ssh + [self.target, command], capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise RuntimeError("Remote read-only status query failed: " + result.stderr[-4000:])
        return json.loads(result.stdout)

    def rsync(self, label, manifest):
        if label not in ("metadata", "smoke_audit", "train_audit", "final_checkpoint", "best_model"):
            raise RuntimeError("Unexpected backup group")
        self.master_check()
        destination = self.root / label
        if destination.is_symlink() or not destination.resolve().is_relative_to(self.root):
            raise RuntimeError("Backup destination resolves outside this isolated B backup")
        destination.mkdir(exist_ok=True)
        filelist = self.root / ("." + label + ".files")
        files = manifest["files"]
        filelist.write_bytes(b"".join(str(safe_relative(name)).encode() + b"\x00" for name in sorted(files)))
        # All subprocesses use argv lists. rsync itself parses --rsh; shlex.join
        # quotes every SSH argument, including control socket paths with spaces.
        command = ["rsync", "-rt", "--protect-args", "--partial", "--partial-dir=.rsync-partial",
                   "--from0", "--files-from=" + str(filelist), "--rsh=" + shlex.join(self.ssh),
                   self.target + ":" + manifest["remote_root"].rstrip("/") + "/", str(destination) + "/"]
        with (self.root / "rsync.log").open("a") as log:
            log.write(f"\n{now()} copying {label}\n"); log.flush()
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=self.args.transfer_timeout)
        if result.returncode:
            raise RuntimeError(f"rsync {label} exited {result.returncode}; inspect rsync.log")
        verified = []
        for name, expected in files.items():
            target = destination.joinpath(*safe_relative(name).parts)
            if not target.is_file() or target.is_symlink() or target.stat().st_size != expected["bytes"] \
                    or sha(target) != expected["sha256"]:
                raise RuntimeError(f"Local copied file SHA/size mismatch: {label}/{name}")
            verified.append(name)
        return {"files_verified": len(verified), "bytes_verified": sum(value["bytes"] for value in files.values())}

    def backup(self, query):
        stage = query["status"]["stage"]
        save(self.root / "remote_terminal_snapshot.json", query)
        self.status("backing_up_failure_logs" if stage == "failed" else "backing_up_complete_run",
                    remote_stage=stage)
        verification = {"experiment_id": self.args.experiment_id, "created_utc": now(),
                        "remote_stage": stage, "original_model_A_modified": False, "groups": {}}
        for label, manifest in query["metadata_manifests"].items():
            verification["groups"][label] = self.rsync(label, manifest)
        if stage == "complete":
            models = query["models"]
            required = sum(value["bytes"] for entry in models.values() for value in entry["files"].values())
            already_present = sum(path.stat().st_size for label in models
                                  for path in (self.root / label).rglob("*") if path.is_file())
            if shutil.disk_usage(self.root).free < max(0, required - already_present) + 1024**3:
                raise RuntimeError("Insufficient local space for final + best + 1 GiB reserve")
            for label, manifest in models.items():
                receipt = self.root / "metadata" / manifest["receipt_relative"]
                if not receipt.is_file() or sha(receipt) != manifest["receipt_sha256"]:
                    raise RuntimeError(f"Remote checkpoint receipt SHA mismatch: {label}")
                if label == "best_model" and sha(self.root / "metadata/best_model_receipt.json") != manifest["best_receipt_sha256"]:
                    raise RuntimeError("Best-model selection receipt SHA mismatch")
                verification["groups"][label] = self.rsync(label, manifest)
        verification["complete_model_backup"] = stage == "complete"
        verification["all_selected_files_sha256_verified"] = True
        save(self.root / "verification.json", verification)
        self.status("complete_verified" if stage == "complete" else "failed_logs_preserved",
                    complete_model_backup=(stage == "complete"), verification_file="verification.json",
                    remote_error=query["status"].get("error"))
        return 0 if stage == "complete" else 2

    def run(self):
        deadline = time.monotonic() + self.args.max_hours * 3600
        self.status("waiting_for_cloud_training")
        while time.monotonic() < deadline:
            self.state["poll_count"] += 1
            try:
                snapshot = self.query()
            except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
                self.status("waiting_for_existing_ssh_master", last_read_error=str(error))
            else:
                remote = snapshot["status"]
                save(self.root / "latest_remote_status.json", remote)
                if remote.get("stage") in ("complete", "failed"):
                    # One terminal backup attempt. No training retry or remote cleanup.
                    return self.backup(self.query(manifest=True))
                self.status("waiting_for_cloud_training", remote_stage=remote.get("stage"),
                            remote_child_pid=remote.get("child_pid"))
            time.sleep(self.args.poll_seconds)
        self.status("watch_timeout", complete_model_backup=False)
        return 4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--backup-dir", required=True)
    parser.add_argument("--host", default="116.172.66.186")
    parser.add_argument("--port", type=int, default=29596)
    parser.add_argument("--user", default="root")
    parser.add_argument("--remote-python", default="/root/autodl-tmp/piper_pi05/.venv/bin/python")
    parser.add_argument("--poll-seconds", type=int, default=180)
    parser.add_argument("--max-hours", type=float, default=48)
    parser.add_argument("--transfer-timeout", type=int, default=8 * 3600)
    args = parser.parse_args()
    if not re.fullmatch(r"pi05_expert_only_v6_[0-9]{6}", args.experiment_id):
        parser.error("Unexpected B experiment ID")
    if not re.fullmatch(r"[A-Za-z0-9.-]+", args.host) or not re.fullmatch(r"[A-Za-z0-9_-]+", args.user):
        parser.error("Invalid SSH host/user")
    if not 1 <= args.port <= 65535 or args.poll_seconds < 30 or args.max_hours <= 0:
        parser.error("Invalid port/poll/deadline")
    if not args.remote_python.startswith("/"):
        parser.error("Remote Python must be an absolute executable path")
    watcher = Watcher(args)
    try:
        return watcher.run()
    except BaseException as error:
        watcher.status("backup_or_watcher_failed", error=type(error).__name__ + ": " + str(error),
                       complete_model_backup=False, automatic_training_retry=False)
        raise


if __name__ == "__main__":
    sys.exit(main())
