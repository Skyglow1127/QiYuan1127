"""Reproduce the optional RT-DETR / Qwen experiment assets without altering defaults."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
RT_COMMIT='df2a96c8c3a098567a1149bf0248da6efe6d495d'
RT_URL='https://github.com/lyuwenyu/storage/releases/download/v0.2/rtdetrv2_r18vd_120e_coco_rerun_48.1.pth'
RT_SHA='2ace52184b620204004509b72752ac7bfe64aadaf7fc1d076b18df8ab5a5c77e'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--with-qwen',action='store_true',help='Also download the ~1.7GB local Qwen weights')
    p.add_argument('--image',type=Path,default=ROOT/'output/benchmark_fixture/last_session_color.png',
                   help='Local image used for PyTorch/ONNX parity checks')
    a=p.parse_args()
    python=ROOT/'.venv-models/bin/python'
    if not python.exists():
        p.error('Run scripts/setup_models.sh first')
    subprocess.run(['python3','-m','pip','--python',str(python),'install','-r',str(ROOT/'scripts/experimental-requirements.txt')],check=True)
    source=ROOT/'experiments/RT-DETR'
    if not source.exists():
        source.parent.mkdir(exist_ok=True)
        subprocess.run(['git','clone','https://github.com/lyuwenyu/RT-DETR.git',str(source)],check=True)
        subprocess.run(['git','checkout','--detach',RT_COMMIT],cwd=source,check=True)
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if actual!=RT_COMMIT or subprocess.check_output(['git','status','--porcelain'],cwd=source,text=True).strip():
        raise RuntimeError('Existing RT-DETR checkout differs from tested revision; preserve it and choose a fresh experiment directory')
    checkpoint=ROOT/'models/rtdetrv2_r18.pth'
    if not checkpoint.exists():
        temporary=checkpoint.with_suffix('.download')
        urllib.request.urlretrieve(RT_URL,temporary)
        if hashlib.sha256(temporary.read_bytes()).hexdigest()!=RT_SHA:
            raise RuntimeError('Downloaded checkpoint SHA256 mismatch')
        temporary.replace(checkpoint)
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=RT_SHA:
        raise RuntimeError('Existing checkpoint SHA256 mismatch')
    subprocess.run([str(python),str(ROOT/'scripts/prepare_rtdetr.py'),'--image',str(a.image.resolve())],check=True)
    if a.with_qwen:
        manifest=json.loads((ROOT/'models/qwen3.5-0.8b.json').read_text())
        # A subprocess ensures the newly installed packages come from the isolated runtime.
        code='''from huggingface_hub import snapshot_download
import sys
snapshot_download("Qwen/Qwen3.5-0.8B",revision=sys.argv[1],local_dir=sys.argv[2],
allow_patterns=["*.json","*.safetensors","*.txt","*.jinja","README.md","LICENSE*"])
'''
        directory=ROOT/'models/Qwen3.5-0.8B'
        subprocess.run([str(python),'-c',code,manifest['revision'],str(directory)],check=True)
        for name,digest in manifest['files_sha256'].items():
            if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Qwen asset SHA256 mismatch: '+name)
    print('Optional assets ready. The default detection_mode was not changed.')

if __name__=='__main__':
    main()
