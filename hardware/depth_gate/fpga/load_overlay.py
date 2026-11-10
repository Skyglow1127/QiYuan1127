#!/usr/bin/env python3
import hashlib,json,os,shutil,subprocess
from pathlib import Path
if os.geteuid()!=0:raise SystemExit('Run with sudo')
root=Path(__file__).resolve().parent
meta=json.loads((root/'bitstream-manifest.json').read_text())
bit=root/'kr260-depth-gate.bit.bin'
if hashlib.sha256(bit.read_bytes()).hexdigest()!=meta['sha256']:raise SystemExit('Bitstream SHA256 mismatch')
base=Path('/sys/kernel/config/device-tree/overlays')
if list(base.iterdir()):raise SystemExit('Existing overlay(s) present; refusing to replace them')
backup=Path('/root/kr260-depth-gate-before');backup.mkdir(exist_ok=True)
(backup/'fpga-state.txt').write_text(Path('/sys/class/fpga_manager/fpga0/state').read_text())
(backup/'clk-summary.txt').write_bytes(Path('/sys/kernel/debug/clk/clk_summary').read_bytes())
shutil.copyfile(bit,'/lib/firmware/kr260-depth-gate.bit.bin')
os.chmod('/lib/firmware/kr260-depth-gate.bit.bin',0o644)
node=base/'depth-gate';node.mkdir()
try:
 (node/'dtbo').write_bytes((root/'depth_gate.dtbo').read_bytes())
 print('overlay_status', (node/'status').read_text().strip())
 print('fpga_state',Path('/sys/class/fpga_manager/fpga0/state').read_text().strip())
except Exception:
 try:node.rmdir()
 except OSError:pass
 raise
