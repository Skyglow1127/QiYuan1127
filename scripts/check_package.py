#!/usr/bin/env python3
"""Check publishable files without importing project code or connecting to devices."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
DENIED_SUFFIXES={'.pt','.pth','.onnx','.gguf','.ckpt','.parquet','.mp4','.bag','.mcap','.bit','.xsa','.dcp'}
BOARD_ARTIFACTS={
 'hardware/fpga_project_20261104/上板文件/robot_rt.bit',
 'hardware/fpga_project_20261104/上板文件/robot_rt.bit.bin',
 'hardware/fpga_project_20261104/上板文件/robot_rt.xsa',
 'hardware/fpga_project_20261104/板端配置/robot_rt.dtbo',
}
SECRET=re.compile(rb'github_pat_[A-Za-z0-9_]{20,}|gh[pousr]_[A-Za-z0-9]{20,}|hf_[A-Za-z0-9]{20,}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----')

def main():
 try:
  paths=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT).decode().split('\0')
  files=[ROOT/p for p in sorted(set(paths)) if p and (ROOT/p).is_file()]
 except subprocess.CalledProcessError:
  files=[p for p in ROOT.rglob('*') if p.is_file() and not any(x in {'.git','.local','.venv','__pycache__','.pytest_cache'} for x in p.relative_to(ROOT).parts)]
 errors=[];total=0
 for f in files:
  rel=str(f.relative_to(ROOT));data=f.read_bytes();total+=len(data)
  if f.is_symlink():errors.append(f'Symlink in publication: {rel}')
  if len(data)>10_000_000:errors.append(f'Large file should be external: {rel}')
  if (f.suffix in DENIED_SUFFIXES or f.name=='model.safetensors') and rel not in BOARD_ARTIFACTS:errors.append(f'Excluded asset present: {rel}')
  if SECRET.search(data):errors.append(f'Potential embedded credential: {rel}')
  try:
   if f.suffix=='.py':ast.parse(data,filename=rel)
   elif f.suffix=='.json':json.loads(data)
  except (SyntaxError,ValueError,UnicodeError) as e:errors.append(f'Parse error {rel}: {e}')
 manifest=ROOT/'docs/source_manifest.json'
 if manifest.exists():
  for item in json.loads(manifest.read_text())['files']:
   f=ROOT/item['path']
   if not f.is_file():errors.append(f'Missing recorded file: {item["path"]}')
   elif hashlib.sha256(f.read_bytes()).hexdigest()!=item['sha256']:errors.append(f'Snapshot digest changed: {item["path"]}')
 board=ROOT/'hardware/fpga_project_20261104/上板文件'
 if (board/'artifacts.sha256.json').exists():
  for name,sha in json.loads((board/'artifacts.sha256.json').read_text())['files'].items():
   f=board/name
   if not f.is_file():errors.append(f'Missing board artifact: {name}')
   elif hashlib.sha256(f.read_bytes()).hexdigest()!=sha:errors.append(f'Board artifact digest changed: {name}')
 print(json.dumps({'publishable_files':len(files),'bytes':total,'errors':errors},ensure_ascii=False,indent=2))
 return bool(errors)
if __name__=='__main__':sys.exit(main())
