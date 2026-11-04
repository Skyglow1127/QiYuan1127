#!/usr/bin/python3
"""Load the verified DESC_ONLY image and run the two authorized board tests."""
from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,tempfile,time
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parent/('actual_tests_'+time.strftime('%Y%m%d_%H%M%S'))
APP=Path('/lib/firmware/xilinx/robot_rt')
OVERLAYS=Path('/sys/kernel/config/device-tree/overlays')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def require(ok,msg):
 if not ok:raise RuntimeError(msg)
def run(name,args,timeout=60):
 print(name,flush=True)
 log=OUT/(name+'.log')
 with log.open('w') as handle:
  handle.flush();os.fsync(handle.fileno());os.sync()
  r=subprocess.run(args,cwd=ROOT,text=True,stdout=handle,stderr=subprocess.STDOUT,timeout=timeout)
  handle.flush();os.fsync(handle.fileno())
 output=log.read_text()
 require(r.returncode==0,name+' failed; see '+str(OUT/(name+'.log')))
 return output

def wait_for(predicate,message):
 end=time.monotonic()+10
 while time.monotonic()<end:
  if predicate():return
  time.sleep(.2)
 raise RuntimeError(message)

def bound(driver,node):
 base=Path('/sys/bus/platform/drivers')/driver
 return any(p.is_symlink() and (p/'of_node').exists() and (p/'of_node').resolve()==Path('/sys/firmware/devicetree/base/fpga-region')/node for p in base.iterdir())

require(os.geteuid()==0,'Please run with sudo')
require(not OUT.exists(),'A test result directory already exists; do not repeat this run blindly')
OUT.mkdir()
status={'load_test_passed':False,'stationary_test_passed':False,'robot_or_CAN_access':False}
try:
 manifest=json.loads((ROOT/'stage.sha256.json').read_text())
 for name,want in manifest.items():require(sha(ROOT/name)==want,'Staged file changed: '+name)
 apps=run('01_apps_before',['xmutil','listapps'])
 active=[l.split()[0] for l in apps.splitlines() if re.search(r'\b0->\d+',l)]
 require(active in (['k26-starter-kits'],['robot_rt']),'Unexpected active FPGA application')
 run('02_unload_old',['xmutil','unloadapp','0'])
 wait_for(lambda:not list(OVERLAYS.iterdir()),'Old overlay was not removed')
 APP.mkdir(parents=True,exist_ok=True)
 shutil.copyfile(ROOT/'build/robot_rt.bit.bin',APP/'robot_rt.bit.bin')
 shutil.copyfile(ROOT/'shell.json',APP/'shell.json')
 shutil.copyfile(ROOT/'robot_rt.dtbo',APP/'robot_rt.dtbo');os.sync()
 run('03_load_new',['xmutil','loadapp','robot_rt'],timeout=90)
 wait_for(lambda:any((p/'status').read_text().strip()=='applied' for p in OVERLAYS.iterdir()),'New overlay was not applied')
 apps=run('04_apps_after',['xmutil','listapps'])
 require([l.split()[0] for l in apps.splitlines() if re.search(r'\b0->\d+',l)]==['robot_rt'],'robot_rt is not the active DFX application')
 require(Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()=='operating','FPGA manager is not operating')
 require(bound('xilinx_fclk','clocking0') and bound('afi-fpga','afi0'),'FCLK/AFI platform drivers did not bind')
 clocks=Path('/sys/kernel/debug/clk/clk_summary').read_text();(OUT/'clocks_after.txt').write_text(clocks)
 pl0=next(l.split() for l in clocks.splitlines() if l.split() and l.split()[0]=='pl0_ref')
 rate=int(pl0[4]);require(int(pl0[1])>0 and int(pl0[2])>0,'PL0 clock is disabled')
 require(abs(rate-49999500)<=1000 and rate<=50000000,'Unexpected PL0 clock rate')
 status['linux_pl0_clock_hz']=rate;status['compiled_nominal_clock_hz']=49999500
 run('05_loaded_identity',['bash',str(ROOT/'sw/load.sh'),'--check','--build',str(ROOT/'build'),'--read-status'])
 status['load_test_passed']=True
 (OUT/'status.json').write_text(json.dumps(status,indent=2)+'\n');os.sync()
 data=json.loads((ROOT/'stationary.json').read_text())
 data['session_nonce']=time.time_ns()
 test_input=OUT/'stationary_input.json'
 test_input.write_text(json.dumps(data,indent=2)+'\n');os.sync()
 manifest_sha=sha(ROOT/'build/artifacts.sha256.json')
 run('06_stationary',['/usr/bin/python3',str(ROOT/'trace_stationary.py'),'--run','--build',str(ROOT/'build'),'--input',str(test_input),'--lead-cycles','40000000','--window-cycles','49999999','--output-dir',str(OUT/'stationary'),'--platform-manifest-sha256',manifest_sha,'--device-mmio-mapping'])
 result=json.loads((OUT/'stationary/result.json').read_text())
 require(result.get('hardware_access') is True,'Stationary test was not executed on hardware')
 status['stationary_test_passed']=True
 status['stationary_result']=result
except BaseException as e:
 status['error']=str(e)
finally:
 status['finished_at_unix']=time.time()
 (OUT/'status.json').write_text(json.dumps(status,indent=2)+'\n');os.sync()
 r=subprocess.run(['dmesg','-T'],capture_output=True,text=True,timeout=10)
 (OUT/'kernel_tail.txt').write_text('\n'.join(r.stdout.splitlines()[-120:])+'\n')
 print(json.dumps(status,indent=2),flush=True)
raise SystemExit(0 if status['stationary_test_passed'] else 1)
