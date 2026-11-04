#!/usr/bin/python3
"""Recorded-image ACT trajectory replay on DESC_ONLY FPGA, no CAN transport."""
from pathlib import Path
import json,sys,time,struct,hashlib,os,traceback
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from sw.board_desc_smoke import NativeMMIO,Session,check_build
from sw.pl_bank import build_bank
from sim.tick_reference import DDSConfig

def save(path,value):
 path.write_text(json.dumps(value,indent=2)+'\n')
def expected_descriptor(data,artifact,count):
 if count==0:return bytes(96)
 normal=len(data['normal_jerks']);total=normal+len(data['brake_jerks'])
 wire=artifact.report['CAN_integer_samples'][min(count,total)-1]
 tick=data['first_tick_index']+count-1
 cycle=data['origin_cycle']+DDSConfig(data['f_num'],data['f_den'],data['clock_hz']).tick_cycle(tick)
 flags=0 if count<=normal else 1 if count<=total else 2
 raw=bytearray(struct.pack('<QQQII',count,cycle,tick,data['config_epoch'],flags))
 for i,ident in enumerate((0x155,0x156,0x157,0x159)):
  payload=struct.pack('>ii',*wire[i*2:i*2+2]) if i<3 else struct.pack('>iHBB',wire[6],data['gripper_force_mn'],1,0)
  raw.extend(struct.pack('<HBB8sI',ident,8,0,payload,0))
 return bytes(raw)
def check_snapshot(snap,data,artifact):
 w=snap['words'];count=w['0x510']|(w['0x514']<<32)
 raw=struct.pack('<24I',*(w[f'0x{a:03x}'] for a in range(0x600,0x660,4)))
 assert raw==expected_descriptor(data,artifact,count),f'Descriptor mismatch at emit {count}: {raw.hex()}'
 assert w['0x018']==0 and w['0x01c']==0,'FPGA fault/reject'
 return count

def main():
 out=ROOT.parent/('act_replay_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
 log=(out/'progress.log').open('w',buffering=1)
 def mark(s):
  print(s,flush=True);log.write(s+'\n');log.flush();os.fsync(log.fileno())
 status={'pass':False,'hardware_access':False,'scope':'ACT inference on recorded RGB -> trajectory -> real FPGA descriptor replay; not live camera or robot'}
 io=None;session=None
 try:
  data=json.loads((ROOT/'act_bank_template.json').read_text());data['session_nonce']=time.time_ns()
  initial=build_bank(data);build=check_build(ROOT/'build')
  assert data['clock_hz']==build['addresses']['clock_hz']
  save(out/'input.json',data);os.sync()
  io=NativeMMIO(build,out,build['manifest_sha256'],True);session=Session(io,timeout_s=3,sleep=time.sleep)
  status['hardware_access']=True
  assert session.status()==dict(STATE=0,STATUS=0,FAULT_CODE=0,REJECT_CODE=0),'FPGA must be idle'
  mark('01_config')
  session.writes([(a,v) for a,v in initial.register_writes if not 0x400<=a<0x44c])
  session.cmd('CONFIG_COMMIT',0,256)
  origin=session.u64(0xa0);now=session.cycle();c=data['clock_hz'];dds=DDSConfig(data['f_num'],data['f_den'],c)
  earliest=(now+c*2-origin)*dds.f_num//dds.threshold+1
  # Preserve the exact DDS interval sequence used for host integer planning.
  first=1+((earliest-1+274)//275)*275
  data.update(origin_cycle=origin,first_tick_index=first,observation_cycle=now,produced_cycle=now,validation_at_cycle=now,seed_cycle=now,normal_deadline_cycle=now+data['source_max_age_cycles'])
  mark('02_build_replay_bank')
  artifact=build_bank(data)
  assert artifact.report['CAN_integer_samples']==initial.report['CAN_integer_samples'],'DDS phase changed numerical trajectory'
  save(out/'replay_input.json',data);save(out/'expected.json',artifact.report)
  (out/'bank.bin').write_bytes(artifact.image)
  session.writes([(a,v) for a,v in artifact.register_writes if 0x400<=a<0x44c])
  used=256*(len(data['normal_jerks'])+len(data['brake_jerks']))
  for start,end in ((256,256+used),(0,256)):
   for a in range(start,end,4):io.write32('ram',a,struct.unpack_from('<I',artifact.image,a)[0])
  io.barrier()
  for a in range(0,256+used,4):assert io.read32('ram',a)==struct.unpack_from('<I',artifact.image,a)[0]
  mark('03_commit_bank')
  session.mailbox(0x80,data['bank_sequence']);session.cmd('COMMIT_BANK0',0,258)
  session.mailbox(0x98,1);session.cmd('KICK',0,258)
  assert session.cycle()<origin+dds.tick_cycle(first),'Replay lead time exhausted'
  session.mailbox(0x90,first)
  mark('04_arm')
  session.cmd('ARM',1,259);session.cmd('STOP',1,259)
  mark('05_compare_outputs')
  total=len(data['normal_jerks'])+len(data['brake_jerks']);seen={};start=time.monotonic()
  while time.monotonic()-start<8:
   snap=session.snapshot('sample');n=check_snapshot(snap,data,artifact)
   if n not in seen:seen[n]=snap
   if n>=total+2:break
   time.sleep(.001)
  assert max(seen)>=total+2,'Replay did not reach HOLD'
  assert any(0<n<=len(data['normal_jerks']) for n in seen),'No normal ACT output captured'
  assert any(len(data['normal_jerks'])<n<=total for n in seen),'No braking output captured'
  assert any(initial.report['CAN_integer_samples'][i]!=initial.report['CAN_integer_samples'][0] for i in range(total)),'Trajectory must change'
  save(out/'captures.json',list(seen.values()))
  status.update(pass_=True,normal_records=len(data['normal_jerks']),brake_records=len(data['brake_jerks']),sampled_emit_sequences=sorted(seen),checked_snapshots=len(seen),final_emit_count=max(seen),build_manifest_sha256=build['manifest_sha256'],input_sha256=hashlib.sha256((ROOT/'act_bank_template.json').read_bytes()).hexdigest())
  status['pass']=status.pop('pass_')
 except BaseException as e:
  status['error']=str(e);traceback.print_exc()
 finally:
  if session is not None:
   try:mark('06_disarm');session.cmd('DISARM',0,0)
   except BaseException as e:status['pass']=False;status['cleanup_error']=str(e)
   save(out/'receipts.json',session.receipts)
  if io is not None:io.close()
  save(out/'result.json',status);os.sync();print(json.dumps(status,indent=2),flush=True)
 return 0 if status['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
