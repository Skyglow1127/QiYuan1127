#!/usr/bin/env python3
"""Explicit rollback plus a 30-action window and light joint smoothing.

Original model/default files and the final round2 model are preserved.
Physical task performance still requires an operator-run trial.
"""
import hashlib
from pathlib import Path
import subprocess
import sys

import launch_act
from act_release_continuation import RELEASE_CONTINUATION

HERE = Path(__file__).resolve().parent
PROFILE = HERE/'act_launch_contact_repair.json'
BASELINE = HERE/'baselines/task_success_v6'


def verify_profile(profile_path=PROFILE):
    profile = launch_act.read(profile_path)
    expected = launch_act.read(BASELINE/'act_launch_defaults.json')
    expected.update(n_action_steps=30,policy_joint_smoothing_alpha=.7,
        policy_release_chunk_extension=RELEASE_CONTINUATION)
    if profile != expected:
        raise ValueError('Contact repair must retain the parent model, action count30, smoothing0.7 and the reviewed release continuation')
    preserved = launch_act.read(BASELINE/'model_backup_manifest.json')
    if Path(profile['checkpoint']).resolve() != Path(preserved['original_model']).resolve():
        raise ValueError('Repair must use the preserved successful parent ACT')
    for name,digest in preserved['sha256'].items():
        with (Path(profile['checkpoint'])/name).open('rb') as stream:
            if hashlib.file_digest(stream,'sha256').hexdigest() != digest:
                raise ValueError('Original model integrity failure:'+name)
    return profile


def main():
    profile = verify_profile()
    print('抓放修复入口：使用已完成整段任务的原ACT权重；续训最终模型保留作对照。',flush=True)
    print('默认连续执行30步后重新预测，六关节新目标权重0.7；夹爪不滤波、限位/限速/时效保护保持。',flush=True)
    print('放置阶段：已要求松爪但实际尚未张开时，最多继续同一次预测的后续20步，避免换块立刻重新闭爪。',flush=True)
    print('回撤前等待实际开口达到60mm并稳定150ms；最长2秒，未确认则停止。',flush=True)
    previous = launch_act.DEFAULTS
    try:
        launch_act.DEFAULTS = PROFILE
        return launch_act.main()
    finally:
        launch_act.DEFAULTS = previous


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError,OSError,ValueError,KeyError,subprocess.SubprocessError) as error:
        print('抓放修复入口已中止：'+str(error),file=sys.stderr)
        raise SystemExit(2)
