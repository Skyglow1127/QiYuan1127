"""Gazebo payload feedback and verified physical pick / replace sequence.
No attachment joints, kinematic following, or pose writes during grasping.
"""
import json, pathlib, threading, time, subprocess, re
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
ROOT=pathlib.Path(__file__).resolve().parents[1]
CONFIG=json.loads((ROOT/'config/grasp_scene.json').read_text())
ROBOT=json.loads((ROOT/'config/robot.json').read_text())
ARM_JOINTS=list(ROBOT['arm_joints'])
FINGER_JOINTS=list(ROBOT['finger_joints'])
FINGER_LINKS=list(ROBOT['finger_links'])
def gripper_targets(amount):
 opened=ROBOT['finger_open'];closed=ROBOT.get('finger_closed',[0.]*len(FINGER_JOINTS))
 if len(opened)!=len(FINGER_JOINTS) or len(closed)!=len(FINGER_JOINTS):raise ValueError('robot.json 夹爪范围数量不一致')
 return {name:float(a+(b-a)*amount) for name,a,b in zip(FINGER_JOINTS,opened,closed)}
def has_finger_contacts(state):
 return bool(FINGER_LINKS) and all(link in state['finger_contacts'] for link in FINGER_LINKS)

class Payload:
 def __init__(self):
  self.lock=threading.RLock(); self.pose=None;self.last=0.;self.touch={};self.contacts=[]
  self.gz=GzNode()
  self.gz.subscribe(Pose_V,ROBOT.get('payload_pose_topic','/model/'+CONFIG['cube']['name']+'/pose'),self.on_pose)
  self.gz.subscribe(Contacts,ROBOT.get('payload_contacts_topic','/'+CONFIG['cube']['name']+'/contacts'),self.on_contacts)
 def on_pose(self,msg):
  for p in msg.pose:
   if p.name==CONFIG['cube']['name']:
    with self.lock:
     self.pose={'position':[p.position.x,p.position.y,p.position.z],'quaternion':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]};self.last=time.monotonic()
 def on_contacts(self,msg):
  with self.lock:
   self.contacts=[]
   for c in msg.contact:
    pair=[c.collision1.name,c.collision2.name];self.contacts.append(pair)
    for finger in FINGER_LINKS:
     if any(finger in re.split(r'::|/',n) for n in pair):self.touch[finger]=time.monotonic()
 def state(self):
  with self.lock:
   now=time.monotonic()
   return {'connected':self.pose is not None and now-self.last<2,'pose':self.pose,'finger_contacts':[n for n,t in self.touch.items() if now-t<.4], 'contacts':list(self.contacts),'size':CONFIG['cube']['size'],'mass':CONFIG['cube']['mass']}
 def update_guard(self,guard,actual=None):
  s=self.state()
  guard.carried=None
  if s['pose']:
   p=s['pose'];R=Rotation.from_quat(p['quaternion']).as_matrix();guard.set_box('red_cube',CONFIG['cube']['size'],p['position'],R)
   if actual and has_finger_contacts(s):
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=p['position'];guard.carried=np.linalg.inv(guard.transforms(actual)[ROBOT['wrist_link']])@T
 def reset(self):
  # Only called by the explicit reset action, after the arm has retreated.
  x,y,z=CONFIG['cube']['position']
  request=f'name: "{CONFIG["cube"]["name"]}" position: {{x: {x} y: {y} z: {z+.001}}} orientation: {{w: 1}}'
  # Isolate the transport request from Python subscriber callback threads.
  # In-process requests can time out while their command still takes effect.
  response=subprocess.run(['gz','service','-s','/world/'+ROBOT['world']+'/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','4000','--req',request],capture_output=True,text=True,timeout=8)
  if response.returncode or not re.search(r'data:\s*true',response.stdout):raise ValueError('方块复位失败：Gazebo 未确认请求')
  with self.lock:self.touch={}

class GraspSequence:
 def __init__(self,control):
  self.c=control;self.active=False;self.phase='待抓取';self.result=None;self.steps=[];self.index=0;self.entered=0.;self.settled=None;self.start_z=CONFIG['cube']['position'][2];self.last_success=None;self.dynamic_poses={};self.placement=None
 def state(self):return {'active':self.active,'phase':self.phase,'result':self.result,'last_success':self.last_success,'placement':self.placement}
 def cancel(self,reason='已中止自动动作'):
  self.active=False;self.phase=reason;self.steps=[]
 def pose_target(self,name,grip):
  pose=self.dynamic_poses.get(name,CONFIG['poses'].get(name))
  if len(pose)!=len(ARM_JOINTS):raise ValueError('抓取姿态的关节数与 robot.json 不一致')
  v=dict(zip(ARM_JOINTS,pose));v.update(gripper_targets(grip));return v
 def observed_place_pose(self,s):
  # Solve robot joints from observed payload tilt/slip. Never write payload pose.
  guard=self.c.guard;actual=self.c.actual
  observed=np.eye(4);observed[:3,:3]=Rotation.from_quat(s['pose']['quaternion']).as_matrix();observed[:3,3]=s['pose']['position']
  relative=np.linalg.inv(guard.transforms(actual)[ROBOT['wrist_link']])@observed
  desired=guard.transforms(self.pose_target('pick',CONFIG['grip_closed_target']))[ROBOT['wrist_link']].copy()
  cube_rotation=desired[:3,:3]@relative[:3,:3]
  half_height=float((np.abs(cube_rotation)@(np.array(CONFIG['cube']['size'])/2))[2])
  center=np.array(CONFIG['cube']['position'],dtype=float);center[2]=CONFIG.get('support_plane_z',0.)+half_height+.006
  desired[:3,3]=center-desired[:3,:3]@relative[:3,3]
  limits={j['name']:j['limits'] for j in guard.joints if j['name'] in ARM_JOINTS}
  lo=np.array([limits[j]['lower'] for j in ARM_JOINTS])+1e-7;hi=np.array([limits[j]['upper'] for j in ARM_JOINTS])-1e-7
  def residual(q):
   T=guard.transforms({**actual,**dict(zip(ARM_JOINTS,q))})[ROBOT['wrist_link']]
   return np.r_[3*(T[:3,3]-desired[:3,3]),Rotation.from_matrix(desired[:3,:3].T@T[:3,:3]).as_rotvec()]
  result=least_squares(residual,np.clip(CONFIG['poses']['pick'],lo,hi),bounds=(lo,hi),max_nfev=120)
  if np.linalg.norm(residual(result.x))>1e-5:raise ValueError('当前夹持姿态无法安全求解放置位置')
  self.placement={'observed_cube_pose':s['pose'],'release_center_m':center.tolist(),'release_bottom_clearance_m':.006,'joint_target':dict(zip(ARM_JOINTS,result.x.tolist()))}
  return result.x.tolist()
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
   if not has_finger_contacts(s) or pos[2]<expected[2]+.04:raise ValueError('尚未夹起方块，无法放回')
   self.dynamic_poses['observed_place']=self.observed_place_pose(s)
   self.steps=[('按实际夹持姿态放回工作台','observed_place',grip,'move'),('松开方块','observed_place',0.,'open'),('退出抓取位置','above',0.,'retreat')]
  elif action=='reset_cube':
   if has_finger_contacts(s) and pos[2]>expected[2]+.04:raise ValueError('请先放回方块，再复位')
   self.steps=[('张开夹爪',None,0.,'open'),('退到方块上方','above',0.,'move'),('复位方块','above',0.,'reset')]
  else:raise ValueError('未知抓取动作')
  self.active=True;self.result=None;self.index=0
  try:self.enter()
  except Exception:self.active=False;raise
 def enter(self):
  label,pose,grip,kind=self.steps[self.index];self.phase=label
  target=self.pose_target(pose,grip) if pose else dict(self.c.actual)
  target.update(gripper_targets(grip))
  self.c.set_target(target);self.entered=self.c.sim_time;self.settled=None;self.reset_started=False;self.reset_result=None
 def reset_worker(self):
  try:self.c.payload.reset();self.reset_result='ok'
  except Exception as e:self.reset_result=str(e)
 def fail(self,reason):
  self.cancel('抓取停止：'+reason);self.result='failed';self.c.freeze();self.c.collision_warning=self.phase
 def advance(self):
  self.index+=1
  if self.index==len(self.steps):
   self.active=False;kind=self.steps[-1][3];self.result='success';self.phase={'lift':'抓取成功 · 已抬起保持','retreat':'已放回工作台 · 可复位后再抓取','reset':'方块已复位'}.get(kind,'已完成');return
  try:self.enter()
  except ValueError as e:self.fail(str(e))
 def tick(self):
  if not self.active:return
  c=self.c;s=c.payload.state()
  if not s['connected'] or time.monotonic()-c.last>2:return self.fail('仿真反馈中断')
  dt=c.sim_time-self.entered;kind=self.steps[self.index][3]
  if dt>18:return self.fail('未在限定时间内完成 '+self.phase)
  err=max(abs(c.actual.get(j,99)-c.target[j]) for j in ARM_JOINTS)
  commanded=max(abs(c.command[j]-c.target[j]) for j in c.target)<1e-5
  reached=err<.018 and commanded
  if kind in ('open','retreat','reset'):reached=reached and max(abs(c.actual.get(j,99)-c.target[j]) for j in FINGER_JOINTS)<.002
  if kind=='close':
   reached=reached and has_finger_contacts(s) and dt>1.5
   if dt>6 and not has_finger_contacts(s):return self.fail('没有检测到双指夹持')
  if kind=='lift':
   height=s['pose']['position'][2]-self.start_z
   reached=reached and height>=CONFIG['lift_min_m'] and has_finger_contacts(s)
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
   delta=np.array(s['pose']['position'])-CONFIG['cube']['position']
   on_table=any(any('floor' in re.split(r'::|/',name) for name in pair) for pair in s['contacts'])
   if not on_table or s['finger_contacts'] or abs(delta[2])>CONFIG.get('place_z_tolerance_m',.003) or np.linalg.norm(delta[:2])>CONFIG.get('place_xy_tolerance_m',.05):return self.fail('方块未稳定放回工作台目标区域')
  self.advance()
