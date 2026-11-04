from pathlib import Path
import sys,json,time,hashlib
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'measure')]
from act_runtime import load_runtime
from sw.rt_adapter.integer_planner import IntegerPlanner,physical
from sw.rt_adapter.trajectory import MotionBounds
from sim.player_reference import State
from sim.tick_reference import DDSConfig
from sw.pl_bank import build_bank
out=Path(__file__).resolve().parent
runtime=load_runtime(); record=runtime['records'][0]
with runtime['torch'].inference_mode():
 t=time.monotonic()
 raw=runtime['policy'].predict_action_chunk(runtime['builder'].build(record['images'],record['state']))
 actions=runtime['postprocessor'].normalized_to_physical(raw)[0].cpu().numpy()
 elapsed=time.monotonic()-t
profile=runtime['profile']; b=profile['policy']['limits']['bounds']
limits=dict(lower=[x[0] for x in b],upper=[x[1] for x in b],velocity=[45]*6+[.05],acceleration=[160]*6+[.5],jerk=[2000]*6+[5],dq_max=[6]*6+[.008])
initial=dict(q=record['state'].tolist(),v=[0]*7,a=[0]*7)
clock=49999500;dds=DDSConfig(275,2,clock_hz=clock);first=1
planner=IntegerPlanner(State.from_physical(**initial),MotionBounds.from_dict(limits),clock_hz=clock,response_time_s=.4)
normal=[];trace=[]
for i in range(10):
 tick=first+i;dt=dds.tick_cycle(tick)-(dds.tick_cycle(tick-1) if tick>1 else 0)
 slot=min(int(i/137.5*20),99);target=actions[slot]
 normal.append(planner.propose(target,dt));trace.append(dict(slot=slot,target=target.tolist(),planned_q=physical(planner.state.q).tolist()))
tick=first+len(normal)
brake=planner.stop([dds.tick_cycle(tick+i)-dds.tick_cycle(tick+i-1) for i in range(160)])
data=dict(design_only=True,clock_hz=clock,f_num=275,f_den=2,origin_cycle=0,first_tick_index=first,session_nonce=time.time_ns(),config_epoch=1,bank_sequence=1,previous_sequence=0,observation_cycle=0,produced_cycle=0,validation_at_cycle=0,normal_deadline_cycle=clock*10,map_error_cycles=0,max_map_error_cycles=0,lease_cycles=clock*10,source_max_age_cycles=clock*10,stop_timeout_cycles=clock*10,seed_cycle=0,seed_sequence=1,gripper_explicit_reviewed=True,gripper_status_code=1,gripper_set_zero=0,gripper_force_mn=1000,limits=limits,initial_state=initial,normal_jerks=normal,brake_jerks=brake,source_kind='recorded_ACT_trajectory_replay')
artifact=build_bank(data)
for name,value in [('act_bank_template.json',data),('act_predictions.json',dict(actions=actions.tolist(),trace=trace,input_path=record['path'],input_sha256=record['sha256'],model_files=runtime['hashes']['checkpoint_files'],inference_s=elapsed,scope='Fresh ACT inference on recorded images; timed board replay, not live camera clock synchronization')),('expected_template.json',artifact.report)]:
 (out/name).write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(dict(normal=len(normal),brake=len(brake),inference_s=elapsed,initial=initial['q'],final=physical(planner.state.q).tolist())),flush=True)
