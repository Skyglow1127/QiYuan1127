#!/usr/bin/env python3
"""Package source, portable project entry, measured evidence and reports."""
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1]
dest=ROOT/'output/sfls_kr260_simulation.zip'
dest.parent.mkdir(exist_ok=True)
files=set()
for folder in ['rtl','model','scripts','docs','reports']:
    files.update(p for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p))
files.update((ROOT/'sim').glob('*.sv'))
files.update(ROOT/x for x in ['README.md','requirements.txt','.gitignore','build/vivado/sfls_kr260.xpr'])
for test in ['divider','faults','small','stride1','stride64','extended1','extended64',
             'e2e_small','e2e_full','e2e_decimated','e2e_precision','e2e_precision_offset',
             'postroute','project_arithmetic']:
    for name in ['simulate.log','results.csv']:
        p=ROOT/'build'/test/name
        if p.exists():files.add(p)
for p in (ROOT/'sim/vectors').glob('*metadata.json'):files.add(p)
with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in sorted(files):z.write(p,Path('sfls_kr260_simulation')/p.relative_to(ROOT))
with zipfile.ZipFile(dest) as z:assert z.testzip() is None
print(dest, len(files),'files',dest.stat().st_size,'bytes')
