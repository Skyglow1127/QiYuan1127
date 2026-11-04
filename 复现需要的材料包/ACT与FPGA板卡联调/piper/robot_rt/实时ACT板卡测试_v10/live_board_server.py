#!/usr/bin/python3
from pathlib import Path
import sys,json,time,os,socket,struct,threading,subprocess,base64,traceback
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
from sw.board_desc_smoke import NativeMMIO,Session,check_build
OUT=ROOT.parent/('live_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
build=check_build(ROOT/'build');io=NativeMMIO(build,OUT,build['manifest_sha256'],True);session=Session(io,3,time.sleep)
state_lock=threading.Lock();frames={};done=threading.Event();configured=False;kick=0
subprocess.run(['ip','link','set','can0','type','can','bitrate','1000000'],check=True) if '"operstate":"DOWN"' in subprocess.check_output(['ip','-j','link','show','can0'],text=True).replace(' ','') else None
subprocess.run(['ip','link','set','can0','up'],check=True)
bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.setsockopt(socket.SOL_SOCKET,35,1);bus.bind(('can0',));bus.settimeout(.2)
def receive():
 while not done.is_set():
  try:
   packet,anc,flags,addr=bus.recvmsg(16,64)
   ident,size,payload=struct.unpack('=IB3x8s',packet)
   if ident not in (0x2a5,0x2a6,0x2a7,0x2a8) or size!=8:continue
   stamps=[v for level,typ,v in anc if level==socket.SOL_SOCKET and typ==35]
   if not stamps:continue
   sec,ns=struct.unpack('=qq',stamps[0]);stamp=sec*10**9+ns+time.monotonic_ns()-time.time_ns()
   with state_lock:frames[ident]=(payload,stamp)
  except socket.timeout:pass
threading.Thread(target=receive,daemon=True).start()
def feedback():
 with state_lock:copy=dict(frames)
 if len(copy)!=4:return {'ready':False,'seen_ids':list(copy)}
 times=[v[1] for v in copy.values()];now=time.monotonic_ns()
 q=[]
 for ident in (0x2a5,0x2a6,0x2a7):q.extend(v/1000 for v in struct.unpack('>ii',copy[ident][0]))
 q.append(struct.unpack('>i',copy[0x2a8][0][:4])[0]/1e6)
 return dict(ready=(now-min(times)<100000000 and max(times)-min(times)<40000000),q=q,age_ns=now-min(times),source_ns=min(times),skew_ns=max(times)-min(times))
def request(r):
 global configured,kick
 op=r['op']
 if op=='query':
  c=session.cycle();response=dict(cycle=c,remote_ns=time.monotonic_ns(),status=session.status(),feedback=feedback(),origin=session.u64(0xa0))
  if configured:
   kick+=1;session.mailbox(0x98,kick);session.w(16,256);io.barrier()
  if r.get('capture'):response['snapshot']=session.snapshot('live')
  return response
 if op=='config':
  assert session.status()==dict(STATE=0,STATUS=0,FAULT_CODE=0,REJECT_CODE=0)
  writes=[(int(a),int(v)) for a,v in r['writes'] if not 0x400<=a<0x44c]
  session.writes(writes);session.cmd('CONFIG_COMMIT',0,256);configured=True
  return dict(origin=session.u64(0xa0),cycle=session.cycle())
 if op=='submit':
  bank=r['bank'];assert bank in (0,1)
  status=session.status();assert ((status['STATUS']>>(4*bank))&3)==0,'bank is not free'
  if r['initial']:session.writes([(a,v) for a,v in r['writes'] if 0x400<=a<0x44c])
  raw=base64.b64decode(r['image']);assert len(raw)==65536
  used=r['records']*256+256;assert 256<used<=65536
  for start,end in ((256,used),(0,256)):
   for a in range(start,end,4):io.write32('ram',bank*65536+a,struct.unpack_from('<I',raw,a)[0])
  io.barrier()
  for a in range(0,used,4):assert io.read32('ram',bank*65536+a)==struct.unpack_from('<I',raw,a)[0]
  before=session.cycle();assert before<r['first_cycle'],'bank arrived too late'
  session.mailbox(0x80+8*bank,r['sequence']);session.w(16,32 if bank==0 else 64);io.barrier()
  deadline=time.monotonic()+3
  while True:
   s=session.status();assert s['FAULT_CODE']==0 and s['REJECT_CODE']==0,str(s)
   if ((s['STATUS']>>(4*bank))&3)==2:break
   assert time.monotonic()<deadline,'bank validation timeout';time.sleep(.001)
  return dict(admitted_cycle=session.cycle(),status=s)
 if op=='arm':
  kick+=1;session.mailbox(0x98,kick);session.cmd('KICK',0,session.r(20));session.mailbox(0x90,r['first']);session.cmd('ARM',1,259)
  return {'armed':True}
 if op=='stop':
  session.w(16,4);io.barrier();return session.status()
 if op=='close':
  session.cmd('DISARM',0,0);configured=False;done.set();return {'closed':True}
 raise ValueError('unknown operation')
assert session.r(0)==0x5254504c and session.u64(0xa8)==49999500,'Loaded FPGA identity/clock mismatch'
listener=socket.socket();listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);listener.bind(('127.0.0.1',18880));listener.listen(1);listener.settimeout(600)
print('LIVE_SERVICE_READY',flush=True)
try:
 conn,_=listener.accept();conn.settimeout(120)
 with conn,conn.makefile('rwb') as stream:
  while not done.is_set():
   line=stream.readline(1000000)
   if not line:break
   try:result={'ok':True,'result':request(json.loads(line))}
   except BaseException as e:traceback.print_exc();result={'ok':False,'error':str(e)}
   stream.write((json.dumps(result,separators=(',',':'))+'\n').encode());stream.flush()
finally:
 done.set()
 if configured:
  try:session.cmd('DISARM',0,0)
  except Exception:traceback.print_exc()
 (OUT/'receipts.json').write_text(json.dumps(session.receipts,indent=2)+'\n');os.sync();io.close();bus.close();listener.close()
