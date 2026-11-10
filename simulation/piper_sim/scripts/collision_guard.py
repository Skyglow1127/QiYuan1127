"""FCL checks on original CAD triangles; same FK and geometry as URDF.
Adjacent joint bodies are intentionally excluded, matching DART joint filtering.
No global planning: validate the actual rate-limited command path before moving.
"""
import json,math,pathlib,itertools,numpy as np,fcl
ROOT=pathlib.Path(__file__).resolve().parents[1]
ROBOT=json.loads((ROOT/'config/robot.json').read_text())
def rot(axis,angle):
 a=np.asarray(axis,dtype=float);a/=np.linalg.norm(a);x,y,z=a;c=math.cos(angle);s=math.sin(angle);C=1-c
 return np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,y*y*C+c,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
def origin(xyz,rpy):
 T=np.eye(4);T[:3,3]=xyz
 T[:3,:3]=rot([0,0,1],rpy[2])@rot([0,1,0],rpy[1])@rot([1,0,0],rpy[0]);return T
class CollisionGuard:
 def __init__(self,margin=.002):
  self.margin=margin;self.model=json.loads((ROOT/'generated/model.json').read_text());self.objects={};self.corners={};self.attachments={};self.joints=self.model['joints'];self.fixed={j['name']:origin(j['xyz'],j['rpy']) for j in self.joints}
  self.excluded={frozenset((j['parent'],j['child'])) for j in self.joints}
  self.excluded.update(frozenset(pair) for pair in ROBOT.get('collision_exclusions',[]))
  for l in self.model['links']:
   mesh=l.get('collision_mesh',l.get('mesh'))
   if not mesh:continue
   p=ROOT/mesh.lstrip('/');records=np.fromfile(p,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]),offset=84)
   vertices=records['vertices'].reshape(-1,3).astype(float);faces=np.arange(len(vertices)).reshape(-1,3)
   self.corners[l['name']]=np.array(list(itertools.product(*zip(vertices.min(axis=0),vertices.max(axis=0)))))
   geom=fcl.BVHModel();geom.beginModel(len(vertices),len(faces));geom.addSubModel(vertices,faces);geom.endModel();self.objects[l['name']]=fcl.CollisionObject(geom)
  camera_path=ROOT/'config/wrist_camera.json'
  if camera_path.exists():
   camera=json.loads(camera_path.read_text());parts=[]
   for item in camera['colliders']:
    name=item['name'];parts.append(name)
    if item.get('geometry')=='mesh':
     p=ROOT/item['mesh'].lstrip('/');records=np.fromfile(p,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]),offset=84)
     vertices=records['vertices'].reshape(-1,3).astype(float)*np.asarray(item.get('scale',[1,1,1]));faces=np.arange(len(vertices)).reshape(-1,3)
     geom=fcl.BVHModel();geom.beginModel(len(vertices),len(faces));geom.addSubModel(vertices,faces);geom.endModel();self.objects[name]=fcl.CollisionObject(geom)
     self.corners[name]=np.array(list(itertools.product(*zip(vertices.min(axis=0),vertices.max(axis=0)))))
    else:
     size=np.asarray(item['size']);self.objects[name]=fcl.CollisionObject(fcl.Box(*size))
     self.corners[name]=np.array(list(itertools.product(*zip(-size/2,size/2))))
    self.attachments[name]=(camera['link'],origin(item['position'],item.get('rpy',[0,0,0])))
    # The upstream wrist joint assembly intentionally overlaps the fixed mount.
    # Keep the exact allowed bodies configurable for a different robot.
    upstream=next((j['parent'] for j in self.joints if j['child']==camera['parent']),camera['parent'])
    self.excluded.update(frozenset((name,link)) for link in ROBOT.get('camera_excluded_links',[camera['parent'],upstream]))
   self.excluded.update(frozenset((a,b)) for a in parts for b in parts if a!=b)
  self.env={};self.env_bounds={}
  environment_path=ROOT/'config/environment.json'
  environment=json.loads(environment_path.read_text()) if environment_path.exists() else {}
  support=environment.get('support_surface',{'size':[10.,10.,.1],'position':[0.,0.,-.05]})
  self.set_box('floor',support['size'],support['position'],origin([0,0,0],support.get('rpy',[0,0,0]))[:3,:3])
  for item in environment.get('colliders',[]):
   if item['name'] in ('floor','mount','red_cube'):raise ValueError('Environment collider name is reserved: '+item['name'])
   self.set_box(item['name'],item['size'],item['position'],origin([0,0,0],item.get('rpy',[0,0,0]))[:3,:3])
  scene=json.loads((ROOT/'config/grasp_scene.json').read_text())
  self.carried=None;self.cube_size=scene['cube']['size']
  for key in ('cube','mount'):
   item=scene[key];self.set_box(item['name'],item['size'],item['position'])
  self.pairs=[(a,b) for i,a in enumerate(self.objects) for b in list(self.objects)[i+1:] if frozenset((a,b)) not in self.excluded]
 def set_box(self,name,size,position,rotation=None):
  R=np.eye(3) if rotation is None else np.asarray(rotation);p=np.asarray(position);s=np.asarray(size)
  if name not in self.env:self.env[name]=fcl.CollisionObject(fcl.Box(*s))
  self.env[name].setTransform(fcl.Transform(R,p));ext=np.abs(R)@(s/2);self.env_bounds[name]=(p-ext,p+ext)
 def transforms(self,q):
  # URDF document order is not required to be topological. Seed every actual
  # root and resolve parents first, using the official joint axes and origins.
  children={j['child'] for j in self.joints}
  frames={l['name']:np.eye(4) for l in self.model['links'] if l['name'] not in children}
  pending=list(self.joints)
  while pending:
   progressed=False
   for j in pending[:]:
    if j['parent'] not in frames:continue
    M=np.eye(4);v=q.get(j['name'],0.)
    if j['type'] in ('revolute','continuous'):M[:3,:3]=rot(j['axis'],v)
    elif j['type']=='prismatic':M[:3,3]=np.array(j['axis'])*v
    frames[j['child']]=frames[j['parent']]@self.fixed[j['name']]@M
    pending.remove(j);progressed=True
   if not progressed:raise ValueError('URDF 关节树存在未连接父节点或循环')
  return frames
 def check(self,q,margin=None,all_hits=False):
  margin=self.margin if margin is None else margin;frames=self.transforms(q)
  if self.carried is not None:
   # Prediction only: a held object's observed relative transform. This never
   # sets the Gazebo object pose or adds an attachment in the physics engine.
   T=frames[ROBOT['wrist_link']]@self.carried;self.set_box('red_cube',self.cube_size,T[:3,3],T[:3,:3])
  bounds=dict(self.env_bounds)
  for n,o in self.objects.items():
   if n in self.attachments:
    parent,offset=self.attachments[n];T=frames[parent]@offset
   else:T=frames[n]
   o.setTransform(fcl.Transform(T[:3,:3],T[:3,3]));v=self.corners[n]@T[:3,:3].T+T[:3,3];bounds[n]=(v.min(axis=0),v.max(axis=0))
  hits=[];req=fcl.CollisionRequest(num_max_contacts=1)
  pairs=[(a,b,self.objects[a],self.objects[b]) for a,b in self.pairs]
  # Fingertip contact with the payload is intentional. All other robot bodies
  # still avoid the cube; the floor is checked for every finger.
  # Only the fixed base is permitted to meet its mounting plane.
  pairs += [(a,b,o,e) for a,o in self.objects.items() for b,e in self.env.items() if not(a==ROBOT['base_link'] and b=='mount') and not(b=='red_cube' and a in ROBOT['finger_links'])]
  for a,b,o,e in pairs:
   alo,ahi=bounds[a];blo,bhi=bounds[b]
   if np.linalg.norm(np.maximum(0,np.maximum(alo-bhi,blo-ahi)))>margin:continue
   res=fcl.CollisionResult();fcl.collide(o,e,req,res)
   if res.is_collision:distance=-1.
   elif margin>0:distance=fcl.distance(o,e,fcl.DistanceRequest(),fcl.DistanceResult())
   else:continue
   if distance<margin:
    hits.append({'pair':[a,b],'distance_m':distance})
    if not all_hits:return hits
  if self.carried is not None:
   # Allow resting contact, but reject >1 mm predicted payload penetration.
   payload=fcl.CollisionObject(fcl.Box(*(np.array(self.cube_size)-.002)),self.env['red_cube'].getTransform())
   for name,obj in self.env.items():
    if name=='red_cube':continue
    res=fcl.CollisionResult();fcl.collide(payload,obj,req,res)
    if res.is_collision:
     hits.append({'pair':['red_cube',name],'distance_m':-1.})
     if not all_hits:return hits
  return hits
 def check_path(self,start,target):
  # Follow the SAME per-joint rate limiter as Control.tick, with samples <=0.01 rad / 0.00056 m.
  moving={j['name']:float(ROBOT.get('joint_speeds',{}).get(j['name'],ROBOT.get('finger_speed',.025) if j['name'] in ROBOT['finger_joints'] else ROBOT.get('arm_speed',.45))) for j in self.joints if j['type']!='fixed'}
  duration=max([abs(target.get(n,0.)-start.get(n,0.))/v for n,v in moving.items()]+[0.])
  count=max(1,math.ceil(duration/.02))
  for k in range(count+1):
   t=duration*k/count;q={n:start.get(n,0.)+np.clip(target.get(n,0.)-start.get(n,0.),-v*t,v*t) for n,v in moving.items()}
   hit=self.check(q)
   if hit:return {'fraction':k/count,**hit[0]}
  return None
