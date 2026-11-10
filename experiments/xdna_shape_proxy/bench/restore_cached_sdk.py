#!/usr/bin/env python3
"""Restore this machine's previously installed SDK from intact uv archives, offline.
No license acceptance, driver change, downloads, or modification of the source cache.
Requires an empty Python 3.12 venv created by `python3 -m venv <target>`.
"""
from pathlib import Path
import argparse,email,json,os,shutil,subprocess
p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,required=True);p.add_argument('--venv',type=Path,required=True);p.add_argument('--receipt',type=Path,required=True);a=p.parse_args()
target=a.venv.resolve()/ 'lib/python3.12/site-packages'
if not (a.venv/'pyvenv.cfg').is_file(): raise SystemExit('Create a Python3.12 venv first')
items=[]
for f in sorted(a.cache.glob('archive-v0/*/*.dist-info/METADATA')):
 m=email.message_from_string(f.read_text()); items.append((m['Name'],m['Version'],f.parent.parent))
# The original SDK installer reinstalls the AMD ORT wheel after VOE.
items.sort(key=lambda v:(v[0]=='onnxruntime-vitisai',v[0]))
links=0;collisions=[]
def link(src,dst):
 global links
 dst=Path(dst)
 if dst.exists() or dst.is_symlink():
  if dst.is_file() and os.path.samefile(src,dst): return str(dst)
  dst.unlink();collisions.append(str(dst.relative_to(target)))
 os.link(src,dst);links+=1;return str(dst)
for name,version,root in items:
 for source in root.iterdir():
  if source.name.endswith('.data'):
   for sub in ('purelib','platlib'):
    if (source/sub).is_dir(): shutil.copytree(source/sub,target,dirs_exist_ok=True,copy_function=link)
   continue
  dst=target/source.name
  if source.is_dir(): shutil.copytree(source,dst,dirs_exist_ok=True,copy_function=link)
  else: link(source,dst)
a.receipt.parent.mkdir(parents=True,exist_ok=True)
a.receipt.write_text(json.dumps({'method':'offline_hardlink_restore','cache':str(a.cache.resolve()),'venv':str(a.venv.resolve()),'file_links':links,'overwrites':collisions,'distributions':[{'name':n,'version':v,'archive':str(r)} for n,v,r in items]},indent=2)+'\n')
print(json.dumps({'distributions':len(items),'file_links':links,'overwrites':len(collisions),'receipt':str(a.receipt)}))
