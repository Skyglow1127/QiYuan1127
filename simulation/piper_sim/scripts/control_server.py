#!/usr/bin/env python3
"""Local-only operator console. Rendered positions always come from Gazebo /joint_states."""
import json, math, pathlib, threading, time, http.server, functools, argparse, signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from collision_guard import CollisionGuard
from grasp_scene import Payload, GraspSequence, CONFIG, ROBOT, ARM_JOINTS, FINGER_JOINTS, gripper_targets
from wrist_camera_stream import WristCameraStream
ROOT=pathlib.Path(__file__).resolve().parents[1]
MODEL=json.loads((ROOT/'generated/model.json').read_text())
JOINTS={j['name']:j for j in MODEL['joints'] if j['type']!='fixed'}
NAMESPACE=ROBOT['namespace'].rstrip('/')
def joint_speed(name):
 return float(ROBOT.get('joint_speeds',{}).get(name,ROBOT.get('finger_speed',.025) if name in FINGER_JOINTS else ROBOT.get('arm_speed',.45)))
class Control(Node):
 def __init__(self):
  super().__init__(NAMESPACE.strip('/').replace('/','_')+'_operator_console');self.lock=threading.RLock();self.actual={};self.last=0.;self.sim_time=0.;self.hold=False
  if set(ARM_JOINTS+FINGER_JOINTS)-JOINTS.keys():raise ValueError('robot.json 关节未出现在生成的 URDF 中')
  self.guard=CollisionGuard(margin=.002);self.collision_warning=''
  self.target={j:0. for j in JOINTS};self.target.update(gripper_targets(0.));self.command=dict(self.target)
  camera_config=ROOT/'config/wrist_camera.json'
  WristCameraStream.TOPIC=json.loads(camera_config.read_text()).get('sensor',{}).get('topic',NAMESPACE+'/wrist_camera/image') if camera_config.exists() else NAMESPACE+'/wrist_camera/image'
  self.payload=Payload();self.wrist_camera=WristCameraStream();self.sequence=GraspSequence(self);self.last_tick_sim=0.
  self.pub={j:self.create_publisher(Float64,NAMESPACE+'/'+j+'/cmd_pos',10) for j in JOINTS}
  self.create_subscription(JointState,ROBOT.get('joint_state_topic','/joint_states'),self.feedback,10)
  self.create_subscription(Clock,ROBOT.get('clock_topic','/clock'),self.clock,10)
  self.create_timer(.02,self.tick)
 def clock(self,msg):self.sim_time=msg.clock.sec+msg.clock.nanosec*1e-9
 def feedback(self,msg):
  with self.lock:
   self.actual={n:float(v) for n,v in zip(msg.name,msg.position) if n in JOINTS};self.last=time.monotonic()
 def tick(self):
  with self.lock:
   self.payload.update_guard(self.guard,self.actual)
   self.sequence.tick()
   dt=max(0.,min(.04,self.sim_time-self.last_tick_sim));self.last_tick_sim=self.sim_time
   candidate=dict(self.command)
   for j in JOINTS:
    speed=joint_speed(j)
    delta=self.target[j]-self.command[j];candidate[j]+=max(-speed*dt,min(speed*dt,delta))
   hit=self.guard.check(candidate)
   if hit:
    self.collision_warning='已停止：'+ ' ↔ '.join(hit[0]['pair'])+' 接近碰撞'
    self.target=dict(self.command);self.hold=True
    if self.sequence.active:self.sequence.cancel(self.collision_warning)
   else:self.command=candidate
   for j in JOINTS:self.pub[j].publish(Float64(data=self.command[j]))
 def state(self):
  with self.lock:return {'connected':self.feedback_ready(),'age':None if not self.last else time.monotonic()-self.last,'positions':dict(self.actual),'targets':dict(self.target),'hold':self.hold,'sim_time':self.sim_time,'mode':'Gazebo Harmonic / PID position control','collision_protection':True,'collision_margin_m':self.guard.margin,'collision_warning':self.collision_warning,'cube':self.payload.state(),'grasp':self.sequence.state(),'scene':CONFIG,'wrist_camera':self.wrist_camera.state()}
 def feedback_ready(self):
  return all(j in self.actual for j in JOINTS) and time.monotonic()-self.last<2
 def freeze(self):
  for j in ARM_JOINTS:self.target[j]=self.command[j]=self.actual.get(j,self.command[j])
  # Retain finger squeeze to avoid dropping a held payload when stopping the arm.
  self.hold=True
 def set_target(self,proposed):
  proposed={**self.target,**proposed}
  for j,v in list(proposed.items()):
   lim=JOINTS[j].get('limits') or {}
   # Gazebo may report a joint resting on a limit a few ulps beyond it.
   # Normalize internal measured snapshots; user requests stay strictly checked.
   if 'lower' in lim and lim['lower']-1e-7<=v<lim['lower']:v=lim['lower']
   if 'upper' in lim and lim['upper']<v<=lim['upper']+1e-7:v=lim['upper']
   proposed[j]=v
   if not math.isfinite(v) or ('lower' in lim and v<lim['lower']) or ('upper' in lim and v>lim['upper']):raise ValueError('目标超出官方 URDF 限位: '+j)
  self.payload.update_guard(self.guard,self.actual)
  for start in (self.command,self.actual):
   hit=self.guard.check_path(start,proposed)
   if hit:
    self.collision_warning='拒绝运动：'+' ↔ '.join(hit['pair'])+' 路径碰撞或间距不足 2 mm'
    raise ValueError(self.collision_warning)
  self.collision_warning='';self.target=proposed
 def request(self,data):
  with self.lock:
   action=data.get('action','joints')
   if not self.feedback_ready():raise ValueError('Gazebo 完整关节反馈未连接')
   if action=='hold':
    self.sequence.cancel();self.freeze();return
   if action=='resume':self.hold=False;self.collision_warning='';return
   if self.hold:raise ValueError('已保持当前位置，请先恢复控制')
   if action in ('grasp','place','reset_cube'):self.sequence.start(action);return
   if self.sequence.active:raise ValueError('自动动作进行中，请先保持当前位置')
   if action=='home':values={j:0. for j in ARM_JOINTS};values['gripper']=0.
   elif action=='ready':
    if len(ROBOT['ready'])!=len(ARM_JOINTS):raise ValueError('准备姿态的关节数与 robot.json 不一致')
    values=dict(zip(ARM_JOINTS,ROBOT['ready']));values['gripper']=0.
   elif action=='joints':values=data.get('values',{})
   else:raise ValueError('未知动作')
   if not isinstance(values,dict):raise ValueError('values 必须是对象')
   checked={}
   for j,v in values.items():
    if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v):raise ValueError('无效数值')
    if j=='gripper':
     if not 0<=v<=1:raise ValueError('夹爪闭合量应为 0–1')
     checked.update(gripper_targets(v))
    else:
     if j not in ARM_JOINTS:raise ValueError('未知关节或独立夹爪命令')
     lim=JOINTS[j].get('limits') or {}
     if ('lower' in lim and v<lim['lower']) or ('upper' in lim and v>lim['upper']):raise ValueError('超出官方 URDF 限位')
     checked[j]=float(v)
   proposed=dict(self.target);proposed.update(checked)
   self.set_target(proposed)
class Handler(http.server.SimpleHTTPRequestHandler):
 def __init__(self,*a,**kw):super().__init__(*a,directory=str(ROOT),**kw)
 def log_message(self,*a):pass
 def reply(self,data,status=200):
  b=json.dumps(data,ensure_ascii=False).encode();self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
 def do_GET(self):
  path=self.path.split('?',1)[0]
  if path=='/api/state':self.reply(node.state())
  elif path=='/api/wrist_camera.jpg':
   jpeg,state=node.wrist_camera.snapshot()
   if jpeg is None:return self.reply({'error':state['error'],'wrist_camera':state},503)
   self.send_response(200);self.send_header('Content-Type','image/jpeg');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(jpeg)));self.send_header('X-Camera-Frame',str(state['frame_id']));self.send_header('X-Camera-Age',f"{state['age']:.6f}");self.end_headers();self.wfile.write(jpeg)
  else:
   if self.path=='/':self.path='/web/index.html'
   super().do_GET()
 def do_POST(self):
  if self.path!='/api/control':return self.reply({'error':'not found'},404)
  if self.headers.get('Origin') not in (None,f'http://127.0.0.1:{port}',f'http://localhost:{port}'):return self.reply({'error':'origin rejected'},403)
  try:
   n=int(self.headers.get('Content-Length','0'))
   if n<1 or n>4096:raise ValueError('请求大小无效')
   data=json.loads(self.rfile.read(n))
   if not isinstance(data,dict):raise ValueError('请求必须为对象')
   node.request(data);self.reply({'ok':True,**node.state()})
  except (ValueError,TypeError) as e:self.reply({'error':str(e)},400)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=json.loads((ROOT/'config/runtime.json').read_text())['simulation_port']);port=p.parse_args().port
 rclpy.init();node=Control();threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
 signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
 print(f"{ROBOT.get('name',NAMESPACE)} simulation console: http://127.0.0.1:{port}",flush=True)
 try:http.server.ThreadingHTTPServer(('127.0.0.1',port),Handler).serve_forever()
 finally:node.wrist_camera.close();node.destroy_node();rclpy.shutdown()
