#!/usr/bin/env python3
"""Explicit round2 entry; use the existing guarded launcher with a separate profile.

The original defaults and original model are unchanged; this profile retains
the baseline control settings without the opt-in contact-repair smoothing.
All normal --check/--run arguments and hardware checks are retained.
"""
import hashlib
from pathlib import Path
import subprocess
import sys

import launch_act

HERE = Path(__file__).resolve().parent
PROFILE = HERE/'act_launch_round2.json'
BASELINE = HERE/'baselines/task_success_v6/act_launch_defaults.json'
RESULT = HERE.parent/'training/act_round2_v6/result'


def verify_profile(profile_path=PROFILE, baseline_path=BASELINE, result_path=RESULT):
    profile = launch_act.read(profile_path)
    baseline = launch_act.read(baseline_path)
    result_path = Path(result_path)
    manifest = launch_act.read(result_path/'collection_manifest.json')
    status = manifest['cloud_status']
    if (status['state'] != 'complete' or status['exit_code'] != 0
            or status['final_step'] != 10000 or status['parent_files_unchanged'] is not True):
        raise ValueError('Second-round training is incomplete or parent preservation failed')
    expected = dict(baseline)
    expected['checkpoint'] = str((result_path/'pretrained_model').resolve())
    expected['model_sha256'] = status['final_model_sha256']
    if profile != expected:
        raise ValueError('Round2 profile may change only the checkpoint path and weight SHA256')
    if profile['model_sha256'] == baseline['model_sha256']:
        raise ValueError('Round2 entry must select the new trained model')
    checkpoint = Path(profile['checkpoint'])
    files = manifest['files_sha256']
    required = {'model.safetensors', 'config.json', 'train_config.json',
                'policy_preprocessor.json', 'policy_postprocessor.json',
                'policy_preprocessor_step_3_normalizer_processor.safetensors',
                'policy_postprocessor_step_0_unnormalizer_processor.safetensors'}
    if set(files) != required or files['model.safetensors'] != profile['model_sha256']:
        raise ValueError('Incomplete second-round model manifest')
    for name, expected_digest in files.items():
        with (checkpoint/name).open('rb') as stream:
            if hashlib.file_digest(stream,'sha256').hexdigest() != expected_digest:
                raise ValueError('Second-round model integrity failure: '+name)
    return profile


def main():
    profile = verify_profile()
    print('调用独立第二轮ACT：'+profile['checkpoint'], flush=True)
    print('默认沿用成功参数：20步、已使能10%/冷启动5%、夹爪1N及原双端保护。', flush=True)
    previous = launch_act.DEFAULTS
    try:
        # Select only for this process. Never rewrite the shared defaults.
        launch_act.DEFAULTS = PROFILE
        return launch_act.main()
    finally:
        launch_act.DEFAULTS = previous


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print('第二轮ACT启动已中止：'+str(error), file=sys.stderr)
        raise SystemExit(2)
