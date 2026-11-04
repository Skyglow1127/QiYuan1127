from pathlib import Path
import sys,time,json,socket,threading,struct,base64,math,copy,traceback
from concurrent.futures import ThreadPoolExecutor
ROOT=Path(__file__).resolve().parents[1];PROJECT=ROOT.parent
sys.path[:0]=[str(ROOT),str(ROOT/'measure'),str(PROJECT/'official_adapter')]
from act_runtime import load_runtime
from lerobot_robot_piper_shared.devices import PiperHttpCamera,PiperHttpCameraConfig
from sw.rt_adapter.integer_planner import IntegerPlanner,physical,physical_jerk
from sw.rt_adapter.trajectory import MotionBounds,C2Tracker
from sim.player_reference import State,PlayerFault,encode_q32
from sim.tick_reference import DDSConfig
from sw.pl_bank import build_bank
C=49999500;DDS=DDSConfig(275,2,C)
OUT=Path(__file__).resolve().parent/('run_'+time.strftime('%Y%m%d_%H%M%S'))

def advance_target(planner,target,dt):
 proposal=C2Tracker(physical(planner.state.q),planner.bounds,1.,velocity=physical(planner.state.v),acceleration=physical(planner.state.a)).advance(target,dt/C)
 raw=proposal['j']
 for factor in (1.,.999,.99,.9,.5,.1,0.):
  j=tuple(encode_q32(float(x)*factor) for x in raw)
  try:nxt,wire=planner.checked_step(planner.state,j,dt,planner.wire)
  except PlayerFault:continue
  planner.state,planner.wire=nxt,wire;return physical_jerk(j),factor
 raise PlayerFault('No integer-feasible jerk proposal')

def plan(state,actions,limits,first,n=137):
 p=IntegerPlanner(state,MotionBounds.from_dict(limits),clock_hz=C,response_time_s=1.)
 normal=[];scaled=0
 for i in range(n):
  tick=first+i;dt=DDS.tick_cycle(tick)-(DDS.tick_cycle(tick-1) if tick>1 else 0)
  target=actions[min(int(i/137.5*20),len(actions)-1)]
  j,f=advance_target(p,target,dt);normal.append(j);scaled+=f<1
 end=p.state
 brake=p.stop([DDS.tick_cycle(first+n+i)-DDS.tick_cycle(first+n+i-1) for i in range(254-n)])
 return normal,brake,end,scaled

class RPC:
 def __init__(self):
  self.sock=socket.create_connection(('127.0.0.1',18880),timeout=15);self.stream=self.sock.makefile('rwb');self.lock=threading.Lock()
 def call(self,op,**kwargs):
  with self.lock:
   t0=time.monotonic();self.stream.write((json.dumps(dict(op=op,**kwargs),separators=(',',':'))+'\n').encode());self.stream.flush();line=self.stream.readline();t1=time.monotonic()
  if not line:raise RuntimeError('Board connection closed')
  reply=json.loads(line)
  if not reply['ok']:raise RuntimeError(reply['error'])
  return reply['result'],t0,t1

def make_template(state,limits):
 return dict(design_only=True,clock_hz=C,f_num=275,f_den=2,origin_cycle=0,first_tick_index=1,session_nonce=time.time_ns(),config_epoch=1,bank_sequence=1,previous_sequence=0,observation_cycle=0,produced_cycle=0,validation_at_cycle=0,normal_deadline_cycle=C*5,map_error_cycles=0,max_map_error_cycles=1000000,lease_cycles=C*10,source_max_age_cycles=C*5,stop_timeout_cycles=C*10,seed_cycle=0,seed_sequence=1,gripper_explicit_reviewed=True,gripper_status_code=1,gripper_set_zero=0,gripper_force_mn=1000,limits=limits,initial_state=dict(q=state,v=[0]*7,a=[0]*7),normal_jerks=[[0]*7]*2,brake_jerks=[[0]*7]*2,source_kind='live_camera_ACT_descriptor_test')

def verify_capture(snap,banks,global_first):
 w=snap['words'];n=w['0x510']|(w['0x514']<<32)
 assert w['0x018']==0 and w['0x01c']==0,'FPGA fault/reject'
 if not n:return None
 raw=struct.pack('<24I',*(w[f'0x{a:03x}'] for a in range(0x600,0x660,4)))
 tick=global_first+n-1
 selected=next((b for b in reversed(banks) if b['data']['first_tick_index']<=tick),None)
 assert selected is not None,'Unknown output tick'
 d=selected['data'];a=selected['artifact'];i=tick-d['first_tick_index'];normal=len(d['normal_jerks']);total=normal+len(d['brake_jerks']);wire=a['CAN_integer_samples'][min(i,total-1)]
 flags=0 if i<normal else 1 if i<total else 2
 expected=bytearray(struct.pack('<QQQII',n,d['origin_cycle']+DDS.tick_cycle(tick),tick,d['config_epoch'],flags))
 for k,ident in enumerate((0x155,0x156,0x157,0x159)):
  payload=struct.pack('>ii',*wire[k*2:k*2+2]) if k<3 else struct.pack('>iHBB',wire[6],1000,1,0)
  expected.extend(struct.pack('<HBB8sI',ident,8,0,payload,0))
 assert raw==bytes(expected),f'output mismatch at {n}, bank {d["bank_sequence"]}'
 return dict(emit=n,tick=tick,bank=d['bank_sequence'],flags=flags)

def main():
 OUT.mkdir();report={'pass':False,'scope':'live RGB and read-only joint feedback -> ACT -> continuous two-bank FPGA descriptors','robot_commands_sent':0};rpc=None;cameras={};stop=threading.Event();samples=[];errors=[];banks=[];thread=None
 def save(name,data):(OUT/name).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
 try:
  runtime=load_runtime();torch=runtime['torch'];record=runtime['records'][0]
  with torch.inference_mode():
   for _ in range(3):runtime['policy'].predict_action_chunk(runtime['builder'].build(record['images'],record['state']))
  rpc=RPC()
  feedback=rpc.call('query')[0]['feedback']
  if not feedback['ready']:raise RuntimeError('Live joint feedback unavailable: '+str(feedback))
  b=runtime['profile']['policy']['limits']['bounds'];limits=dict(lower=[x[0] for x in b],upper=[x[1] for x in b],velocity=[45]*6+[.05],acceleration=[160]*6+[.5],jerk=[2000]*6+[5],dq_max=[6]*6+[.008])
  template=make_template(feedback['q'],limits);a=build_bank(template);config=rpc.call('config',writes=a.register_writes)[0];origin=config['origin']
  for role,port,serial in [('global',18761,'CC1365302B2'),('wrist',18760,'CC1365301EY')]:
   cam=PiperHttpCamera(PiperHttpCameraConfig(role=role,url=f'http://127.0.0.1:{port}',serial=serial));cam.connect();cameras[role]=cam
  state=State.from_physical(**template['initial_state']);nextfirst=None;global_first=None
  def capture():
   while not stop.wait(.02):
    try:samples.append(rpc.call('query',capture=True)[0]['snapshot'])
    except Exception as e:errors.append(str(e));stop.set()
  with ThreadPoolExecutor(max_workers=2) as pool:
   for index in range(8):
    # Wait for the physical bank being reused to become FREE.
    slot=index%2
    while True:
     q=rpc.call('query')[0]
     assert q['status']['FAULT_CODE']==0 and q['status']['REJECT_CODE']==0,str(q['status'])
     if ((q['status']['STATUS']>>(slot*4))&3)==0:break
     if errors:raise RuntimeError(errors)
     time.sleep(.02)
    syncs=[rpc.call('query') for _ in range(3)];sync,t0,t1=min(syncs,key=lambda item:item[2]-item[1]);anchor=(t0+t1)/2
    def cycle(t):return int(sync['cycle']+(t-anchor)*C)
    futures={role:pool.submit(cam.async_read) for role,cam in cameras.items()};images={role:f.result() for role,f in futures.items()}
    times=[cam.last_metadata['estimated_capture_monotonic'] for cam in cameras.values()];observed=min(times)
    assert max(times)-observed<=.06 and time.monotonic()-observed<=.15,'Live camera age/skew'
    q=rpc.call('query')[0];feedback=q['feedback'];assert feedback['ready'],'Joint feedback stale'
    inference_start=time.monotonic()
    with torch.inference_mode():
     raw_actions=runtime['postprocessor'].normalized_to_physical(runtime['policy'].predict_action_chunk(runtime['builder'].build(images,feedback['q'])))
     projected=runtime['postprocessor'].project(raw_actions,clip=True);actions=projected.values[0].cpu().numpy()
    produced=time.monotonic();first=nextfirst
    if first is None:first=max(1,((cycle(produced)+C-origin)*275)//(C*2)+1);global_first=first
    planning_start=time.monotonic();normal,brake,end,scaled=plan(state,actions,limits,first)
    data=copy.deepcopy(template);error=math.ceil(((t1-t0)/2+.001+abs(time.monotonic()-anchor)*.0001)*C)
    assert error<=data['max_map_error_cycles'],'Clock mapping uncertainty too large'
    data.update(origin_cycle=origin,first_tick_index=first,bank_sequence=index+1,previous_sequence=index,observation_cycle=cycle(observed),produced_cycle=cycle(produced),validation_at_cycle=cycle(time.monotonic()),normal_deadline_cycle=cycle(observed)+C*5-error,map_error_cycles=error,seed_cycle=cycle(observed),initial_state={k:physical(getattr(state,k)).tolist() for k in ('q','v','a')},normal_jerks=normal,brake_jerks=brake)
    artifact=build_bank(data)
    entry=dict(data=data,artifact=artifact.report,source=dict(camera_times=times,metadata={role:cam.last_metadata for role,cam in cameras.items()},joint_feedback=feedback,actions=actions.tolist(),raw_actions=raw_actions[0].cpu().tolist(),projected_values=int(projected.clipped_mask.sum()),inference_s=produced-inference_start,planning_s=time.monotonic()-planning_start,scaled_jerks=scaled,clock_anchor_host=anchor,clock_anchor_pl=sync['cycle'],clock_rtt_s=t1-t0))
    result=rpc.call('submit',bank=slot,initial=index==0,writes=artifact.register_writes,image=base64.b64encode(artifact.image).decode(),records=len(normal)+len(brake),sequence=index+1,first_cycle=origin+DDS.tick_cycle(first))[0]
    entry['admission']=result;banks.append(entry);save(f'bank_{index+1:02}.json',entry)
    if index==0:
     rpc.call('arm',first=first);thread=threading.Thread(target=capture,daemon=True);thread.start()
    state=end;nextfirst=first+len(normal)
    print(json.dumps(dict(bank=index+1,admitted=True,inference_ms=round((produced-inference_start)*1000,2),planning_ms=round(entry['source']['planning_s']*1000,2))),flush=True)
   last=banks[-1]['data'];lastfirst=origin+DDS.tick_cycle(last['first_tick_index'])
   while rpc.call('query')[0]['cycle']<lastfirst:time.sleep(.01)
   rpc.call('stop')
   finaltick=last['first_tick_index']+len(last['normal_jerks'])+len(last['brake_jerks'])+2
   while rpc.call('query')[0]['cycle']<origin+DDS.tick_cycle(finaltick):
    if errors:raise RuntimeError(errors)
    time.sleep(.02)
  stop.set();thread.join();samples.append(rpc.call('query',capture=True)[0]['snapshot'])
  checked=[v for s in samples if (v:=verify_capture(s,banks,global_first)) is not None]
  assert not errors,errors
  assert {v['bank'] for v in checked}==set(range(1,9)),'Missing bank output'
  final=samples[-1]['words'];assert final['0x528']==0 and final['0x52c']==0,'Unexpected starvation at bank seam'
  assert any(v['flags']==2 for v in checked),'Final HOLD not reached'
  report.update(pass_=True,banks=8,bank_switches=7,verified_snapshots=len(checked),distinct_outputs=len({v['emit'] for v in checked}),final_emit_count=max(v['emit'] for v in checked),starve_count=0,model_hashes=runtime['hashes']['checkpoint_files']);report['pass']=report.pop('pass_')
  save('verified_outputs.json',checked)
 except BaseException as e:report['error']=str(e);traceback.print_exc()
 finally:
  stop.set()
  if thread:thread.join(timeout=10)
  if rpc:
   try:rpc.call('close')
   except Exception as e:report['close_error']=str(e);report['pass']=False
  for cam in cameras.values():cam.disconnect()
  save('captures.json',samples);save('result.json',report);print(json.dumps(report,indent=2),flush=True)
 return 0 if report['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
