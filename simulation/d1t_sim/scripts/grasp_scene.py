"""Gazebo payload feedback and verified physical pick / replace sequence.
No attachment joints, kinematic following, or pose writes during grasping.
"""
import json, pathlib, threading, time, subprocess, re
import numpy as np
from scipy.spatial.transform import Rotation
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
ROOT=pathlib.Path(__file__).resolve().parents[1]
CONFIG=json.loads((ROOT/'config/grasp_scene.json').read_text())

class Payload:
 def __init__(self):
  self.lock=threading.RLock(); self.pose=None;self.last=0.;self.touch={};self.contacts=[]
  self.gz=GzNode()
  self.gz.subscribe(Pose_V,'/model/red_cube/pose',self.on_pose)
  self.gz.subscribe(Contacts,'/red_cube/contacts',self.on_contacts)
 def on_pose(self,msg):
  for p in msg.pose:
   if p.name=='red_cube':
    with self.lock:
     self.pose={'position':[p.position.x,p.position.y,p.position.z],'quaternion':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]};self.last=time.monotonic()
 def on_contacts(self,msg):
  with self.lock:
   self.contacts=[]
   for c in msg.contact:
    pair=[c.collision1.name,c.collision2.name];self.contacts.append(pair)
    for finger in ('arm_Link7_1','arm_Link7_2'):
     if any(finger in n for n in pair):self.touch[finger]=time.monotonic()
 def state(self):
  with self.lock:
   now=time.monotonic()
   return {'connected':self.pose is not None and now-self.last<2,'pose':self.pose,'finger_contacts':[n for n,t in self.touch.items() if now-t<.4], 'contacts':list(self.contacts),'size':CONFIG['cube']['size'],'mass':CONFIG['cube']['mass']}
 def update_guard(self,guard,actual=None):
  s=self.state()
  guard.carried=None
  if s['pose']:
   p=s['pose'];R=Rotation.from_quat(p['quaternion']).as_matrix();guard.set_box('red_cube',CONFIG['cube']['size'],p['position'],R)
   if actual and len(s['finger_contacts'])==2:
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=p['position'];guard.carried=np.linalg.inv(guard.transforms(actual)['arm_Link6'])@T
 def reset(self):
  # Only called by the explicit reset action, after the arm has retreated.
  x,y,z=CONFIG['cube']['position']
  request=f'name: "red_cube" position: {{x: {x} y: {y} z: {z+.001}}} orientation: {{w: 1}}'
  # Isolate the transport request from Python subscriber callback threads.
  # In-process requests can time out while their command still takes effect.
  response=subprocess.run(['gz','service','-s','/world/d1t_lab/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','4000','--req',request],capture_output=True,text=True,timeout=8)
  if response.returncode or not re.search(r'data:\s*true',response.stdout):raise ValueError('方块复位失败：Gazebo 未确认请求')
  with self.lock:self.touch={}

class GraspSequence:
 def __init__(self,control):
  self.c=control;self.active=False;self.phase='待抓取';self.result=None;self.steps=[];self.index=0;self.entered=0.;self.settled=None;self.start_z=CONFIG['cube']['position'][2];self.last_success=None
 def state(self):return {'active':self.active,'phase':self.phase,'result':self.result,'last_success':self.last_success}
 def cancel(self,reason='已中止自动动作'):
  self.active=False;self.phase=reason;self.steps=[]
 def pose_target(self,name,grip):
  v=dict(zip([f'arm_Joint{i}' for i in range(1,7)],CONFIG['poses'][name]));v.update(arm_Joint7_1=.03*(1-grip),arm_Joint7_2=-.03*(1-grip));return v
 def start(self,action):
  c=self.c;s=c.payload.state()
  if self.active:raise ValueError('自动动作进行中，请先保持当前位置')
  if c.hold:raise ValueError('请先恢复控制')
  if not s['connected']:raise ValueError('未收到方块的 Gazebo 状态')
  pos=np.array(s['pose']['position']);expected=np.array(CONFIG['cube']['position']);grip=CONFIG['grip_closed_target']
  if action=='grasp':
   if np.linalg.norm(pos-expected)>.008:raise ValueError('方块不在指定地面位置，请先复位方块')
   self.start_z=float(pos[2]);self.steps=[('移动到方块上方','above',0.,'move'),('下降对准方块','pick',0.,'move'),('夹紧并确认双指接触','pick',grip,'close'),('抬起并验证保持','above',grip,'lift')]
  elif action=='place':
   if len(s['finger_contacts'])!=2 or pos[2]<expected[2]+.04:raise ValueError('尚未夹起方块，无法放回')
   self.steps=[('将方块放回地面','pick',grip,'move'),('松开方块','pick',0.,'open'),('退出抓取位置','above',0.,'retreat')]
  elif action=='reset_cube':
   if len(s['finger_contacts'])==2 and pos[2]>expected[2]+.04:raise ValueError('请先放回方块，再复位')
   self.steps=[('张开夹爪',None,0.,'open'),('退到方块上方','above',0.,'move'),('复位方块','above',0.,'reset')]
  else:raise ValueError('未知抓取动作')
  self.active=True;self.result=None;self.index=0
  try:self.enter()
  except Exception:self.active=False;raise
 def enter(self):
  label,pose,grip,kind=self.steps[self.index];self.phase=label
  target=self.pose_target(pose,grip) if pose else dict(self.c.actual)
  target.update(arm_Joint7_1=.03*(1-grip),arm_Joint7_2=-.03*(1-grip))
  self.c.set_target(target);self.entered=self.c.sim_time;self.settled=None;self.reset_started=False;self.reset_result=None
 def reset_worker(self):
  try:self.c.payload.reset();self.reset_result='ok'
  except Exception as e:self.reset_result=str(e)
 def fail(self,reason):
  self.cancel('抓取停止：'+reason);self.result='failed';self.c.freeze();self.c.collision_warning=self.phase
 def advance(self):
  self.index+=1
  if self.index==len(self.steps):
   self.active=False;kind=self.steps[-1][3];self.result='success';self.phase={'lift':'抓取成功 · 已抬起保持','retreat':'已放回 · 可再次抓取','reset':'方块已复位'}.get(kind,'已完成');return
  try:self.enter()
  except ValueError as e:self.fail(str(e))
 def tick(self):
  if not self.active:return
  c=self.c;s=c.payload.state()
  if not s['connected'] or time.monotonic()-c.last>2:return self.fail('仿真反馈中断')
  dt=c.sim_time-self.entered;kind=self.steps[self.index][3]
  if dt>18:return self.fail('未在限定时间内完成 '+self.phase)
  err=max(abs(c.actual.get(f'arm_Joint{i}',99)-c.target[f'arm_Joint{i}']) for i in range(1,7))
  commanded=max(abs(c.command[j]-c.target[j]) for j in c.target)<1e-5
  reached=err<.018 and commanded
  if kind in ('open','retreat','reset'):reached=reached and max(abs(c.actual.get(j,99)-c.target[j]) for j in ('arm_Joint7_1','arm_Joint7_2'))<.002
  if kind=='close':
   reached=reached and len(s['finger_contacts'])==2 and dt>1.5
   if dt>6 and len(s['finger_contacts'])!=2:return self.fail('没有检测到双指夹持')
  if kind=='lift':
   height=s['pose']['position'][2]-self.start_z
   reached=reached and height>=CONFIG['lift_min_m'] and len(s['finger_contacts'])==2
   if reached:self.last_success={'lift_m':height,'cube_position':s['pose']['position'],'both_fingers':True,'sim_time':c.sim_time}
  if not reached:self.settled=None;return
  if self.settled is None:self.settled=c.sim_time
  duration=1.2 if kind in ('lift','close') else .5
  if c.sim_time-self.settled<duration:return
  if kind=='reset':
   if not self.reset_started:
    self.reset_started=True;threading.Thread(target=self.reset_worker,daemon=True).start();return
   if self.reset_result is None:return
   if self.reset_result!='ok':return self.fail(self.reset_result)
   if np.linalg.norm(np.array(s['pose']['position'])-CONFIG['cube']['position'])>.003:return
  if kind=='retreat':
   if np.linalg.norm(np.array(s['pose']['position'])-CONFIG['cube']['position'])>.01:return self.fail('方块未稳定放回地面')
  self.advance()
