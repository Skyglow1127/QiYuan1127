#!/usr/bin/env python3
"""Materialize a relocatable local copy. No network, services or robot operations."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_PI05 = '/home/aup/Documents/Codex/v5/wo-y/outputs'
ORIGINAL_D1 = '/media/aup/UbuntuData/ACT训练/11_D1轨迹执行改进'
TEXT = {'.py','.sh','.json','.yaml','.yml','.toml','.md','.txt','.xml','.cpp',
        '.c','.h','.hpp','.sv','.v','.tcl','.xdc','.dts','.cfg','.ini','.html',
        '.css','.js','.csv','.rpt','.urdf','.xacro'}

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT / '.local/workspace')
    p.add_argument('--assets', type=Path, help='Local JSON with existing asset directories; never uploaded.')
    args = p.parse_args()
    out = args.output.resolve()
    if out.exists():
        p.error(f'Output already exists; use a new output directory: {out}')
    assets = json.loads(args.assets.read_text()) if args.assets else {}
    for name, value in assets.items():
        if not Path(value).is_dir():
            p.error(f'Asset directory does not exist: {name}')
    out.mkdir(parents=True)
    layout = {
        'src/piper': out/'lerobot/projects/piper',
        'src/lerobot_workspace': out/'lerobot',
        'src/pi05': out/'pi05',
        'src/d1': out/'d1',
        'hardware/robot_rt_50mhz': out/'hardware/robot_rt_50mhz',
        'hardware/vision_preprocess': out/'hardware/vision_workspace/vision_preprocess',
    }
    for src, dst in layout.items():
        shutil.copytree(ROOT/src, dst, dirs_exist_ok=True)
    # Longer substitutions first; preserve numerical control and model parameters.
    mappings = {
        '/home/aup/Lerobot/envs/piper-cpu/bin/python': sys.executable,
        '/home/aup/Lerobot/projects/piper/robot_rt': str(out/'hardware/robot_rt_50mhz'),
        '/media/aup/UbuntuData/GITHUB/fpga-part/50MHz版本_20261104/工程源码': str(out/'hardware/robot_rt_50mhz'),
        '/media/aup/UbuntuData/GITHUB/fpga-part/50MHz版本_20261104': str(out/'hardware/robot_rt_50mhz'),
        ORIGINAL_D1: str(out/'d1'),
        ORIGINAL_PI05: str(out/'pi05'),
        '/home/aup/Lerobot': str(out/'lerobot'),
        '/home/aup/文档/ChatGPT/VIVADO': str(out/'hardware/vision_workspace'),
    }
    # A local Python environment takes precedence; do not copy a virtualenv.
    changed=[]
    pattern=re.compile('|'.join(re.escape(x) for x in sorted(mappings,key=len,reverse=True)))
    for f in sorted(out.rglob('*')):
        if not f.is_file() or (f.suffix not in TEXT and f.name not in {'Makefile','CMakeLists.txt'}):
            continue
        try: old=f.read_text()
        except UnicodeError: continue
        new=pattern.sub(lambda match:mappings[match.group(0)],old)
        if new!=old:
            f.write_text(new);changed.append(str(f.relative_to(out)))
    destinations={
        'act_020000': out/'lerobot/models/ACT/rad_place_v5_r05/020000/pretrained_model',
        'pi05_best004000': out/'pi05/piper_pi05_expert_cloud_backup_v6/pi05_expert_only_v6_r09/best_model',
        'pi05_final020000': out/'pi05/piper_pi05_expert_cloud_backup_v6/pi05_expert_only_v6_r09/final_checkpoint/pretrained_model',
        'pi05_base': out/'lerobot/models/PI05/pi05_base',
        'dataset': out/'lerobot/datasets/rad_place_merged_v5',
        'tokenizer': out/'pi05/piper_pi05_cloud_backup_v5/models/paligemma_tokenizer',
    }
    for name,dst in destinations.items():
        if name in {'dataset','pi05_base','tokenizer'}:
            if name in assets:
                dst.parent.mkdir(parents=True,exist_ok=True)
                dst.symlink_to(Path(assets[name]).resolve(),target_is_directory=True)
            continue
        shutil.copytree(ROOT/'models'/name,dst,dirs_exist_ok=True)
        if name in assets:
            weight=Path(assets[name]).resolve()/'model.safetensors'
            if not weight.is_file():p.error(f'Missing model.safetensors for {name}')
            (dst/'model.safetensors').symlink_to(weight)
    paths=[str(out/'lerobot/projects/piper/deployment'),str(out/'lerobot/projects/piper/official_adapter')]
    import shlex
    activation = '#!/usr/bin/env bash\n'
    activation += 'export PYTHONPATH='+shlex.quote(':'.join(paths))+'${PYTHONPATH:+:$PYTHONPATH}\n'
    activation += 'export PATH='+shlex.quote(str(Path(sys.executable).parent))+':"$PATH"\n'
    (out/'lerobot/activate.sh').write_text(activation)
    (out/'lerobot/lerobot_env.sh').write_text(activation)
    (out/'setup_report.json').write_text(json.dumps({
        'path_mappings':mappings,'changed_files':changed,'linked_assets':list(assets),
        'hardware_operations':False,'note':'Historical hashes remain historical; regenerate deployment reviews after configuring devices.'
    },ensure_ascii=False,indent=2)+'\n')
    print(out)
    print('Prepared local copies only. Review device and cloud settings before deployment.')

if __name__=='__main__':main()
