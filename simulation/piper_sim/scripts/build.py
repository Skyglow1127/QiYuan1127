#!/usr/bin/env python3
"""Official PiPER geometry / inertial values, with Harmonic PID adapters."""
import json,pathlib,subprocess,xml.etree.ElementTree as E
import numpy as np,trimesh,yaml
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1];GEN=ROOT/'generated';GEN.mkdir(exist_ok=True)
ROBOT=json.loads((ROOT/'config/robot.json').read_text());NS=ROBOT['namespace']
CAMERA=json.loads((ROOT/'config/wrist_camera.json').read_text())
def child(parent,tag,value=None,**attrs):
 e=E.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
 if value is not None:e.text=str(value)
 return e
def vector(e,attr,default='0 0 0'):
 return [float(x) for x in (e.get(attr,default) if e is not None else default).split()]
subprocess.run(['python3',str(ROOT/'scripts/build_wrist_camera.py')],check=True)
subprocess.run(['xacro',str(ROOT/'urdf/piper.urdf.xacro'),'-o',str(GEN/'piper.urdf')],check=True)
r=E.parse(GEN/'piper.urdf').getroot()
for mesh in r.findall('.//mesh'):
 filename=mesh.get('filename','')
 if '/upstream/agx_arm_urdf/' in filename:
  mesh.set('filename',str(ROOT/'upstream/agx_arm_urdf'/filename.split('/upstream/agx_arm_urdf/',1)[1]))
r.find("joint[@name='world_to_base_link']/origin").set('xyz','0 0 0.02')
model={'name':ROBOT['name'],'links':[],'joints':[]}
# Small contact patches make the flat gripping surface robust in DART/FCL;
# visual CAD and inertial data remain official. These are simulation settings.
for pad in ROBOT.get('contact_pads',[]):
 link=r.find("link[@name='%s']"%pad['link'])
 collision=child(link,'collision',name='simulation_grip_pad')
 child(collision,'origin',xyz=' '.join(map(str,pad['xyz'])),rpy='0 0 0')
 child(child(collision,'geometry'),'box',size=' '.join(map(str,pad['size'])))
# Native visuals retain the official DAE materials. Web/FCL use binary STL in
# link coordinates; visual and collision meshes are recorded independently.
for link in r.findall('link'):
 name=link.get('name');entry={'name':name,'mesh':None,'collision_mesh':None,'color':[.76,.78,.80,1]}
 if name in ('link1','link4','link5','link6','gripper_base'):entry['color']=[.16,.18,.20,1]
 if name.startswith('gripper_link'):entry['color']=[.24,.26,.28,1]
 if name!=CAMERA['link']:
  for key,tag in [('mesh','visual'),('collision_mesh','collision')]:
   meshes=[]
   for element in link.findall(tag):
    m=element.find('geometry/mesh')
    if m is None:
     box=element.find('geometry/box')
     if box is None:continue
     mesh=trimesh.creation.box(extents=vector(box,'size'))
    else:
     source=pathlib.Path(m.get('filename').removeprefix('file://'))
     if key=='mesh':
      c=link.find('collision/geometry/mesh')
      if c is not None:source=pathlib.Path(c.get('filename').removeprefix('file://'))
     mesh=trimesh.load(source,force='mesh',process=False);mesh.apply_scale(vector(m,'scale','1 1 1'))
    o=element.find('origin');T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',vector(o,'rpy')).as_matrix();T[:3,3]=vector(o,'xyz');mesh.apply_transform(T);meshes.append(mesh)
   if meshes:
    out=ROOT/'meshes'/f'{name}_{tag}.STL';trimesh.util.concatenate(meshes).export(out);entry[key]='/meshes/'+out.name
 model['links'].append(entry)
 if name not in ('world',CAMERA['link']) and not name.endswith('_optical_link'):
  gazebo=child(r,'gazebo',reference=name);child(gazebo,'selfCollide','true');child(gazebo,'mu1',.8);child(gazebo,'mu2',.8)
  if name==ROBOT['base_link']:child(child(child(child(gazebo,'collision'),'surface'),'contact'),'collide_bitmask',2)
servo=ROBOT['servo']
for j in r.findall('joint'):
 name=j.get('name');kind=j.get('type');o=j.find('origin');a=j.find('axis');lim=j.find('limit')
 model['joints'].append({'name':name,'type':kind,'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'xyz':vector(o,'xyz'),'rpy':vector(o,'rpy'),'axis':vector(a,'xyz','0 0 1'),'limits':None if lim is None else {k:float(v) for k,v in lim.attrib.items()}})
 if kind=='fixed':continue
 child(j,'dynamics',damping=.04,friction=0)
 if name in ROBOT['arm_joints']:
  i=ROBOT['arm_joints'].index(name);p,integ,d,effort=(servo[k][i] for k in ['arm_p','arm_i','arm_d','arm_effort'])
 else:p,integ,d,effort=(servo[k] for k in ['finger_p','finger_i','finger_d','finger_effort'])
 plugin=child(child(r,'gazebo'),'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
 for k,v in [('joint_name',name),('topic',f'{NS}/{name}/cmd_pos'),('p_gain',p),('i_gain',integ),('d_gain',d),('i_max',2),('i_min',-2),('cmd_max',effort),('cmd_min',-effort)]:child(plugin,k,v)
publisher=child(child(r,'gazebo'),'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher');child(publisher,'topic',f'{NS}/joint_states')
E.indent(r,space='  ');E.ElementTree(r).write(GEN/'piper.urdf',encoding='utf-8',xml_declaration=True)
(GEN/'model.json').write_text(json.dumps(model,indent=2))
bridges=[{'ros_topic_name':'/joint_states','gz_topic_name':f'{NS}/joint_states','ros_type_name':'sensor_msgs/msg/JointState','gz_type_name':'gz.msgs.Model','direction':'GZ_TO_ROS'},{'ros_topic_name':'/clock','gz_topic_name':'/clock','ros_type_name':'rosgraph_msgs/msg/Clock','gz_type_name':'gz.msgs.Clock','direction':'GZ_TO_ROS'}]
for j in model['joints']:
 if j['type']=='fixed':continue
 topic=f"{NS}/{j['name']}/cmd_pos";bridges.append({'ros_topic_name':topic,'gz_topic_name':topic,'ros_type_name':'std_msgs/msg/Float64','gz_type_name':'gz.msgs.Double','direction':'ROS_TO_GZ'})
(GEN/'bridge.yaml').write_text(yaml.safe_dump(bridges));(GEN/'robot_state.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'use_sim_time':True,'robot_description':(GEN/'piper.urdf').read_text()}}}))
scene=json.loads((ROOT/'config/grasp_scene.json').read_text());cube=scene['cube'];model_dir=ROOT/'models/red_cube'
subprocess.run(['xacro',str(ROOT/'urdf/red_cube.sdf.xacro'),f"edge:={cube['size'][0]}",f"mass:={cube['mass']}",'xyz:='+' '.join(map(str,cube['position'])),'-o',str(model_dir/'model.sdf')],check=True)
subprocess.run(['python3',str(ROOT/'scripts/build_environment.py')],check=True)
environment=json.loads((ROOT/'config/environment.json').read_text());support=environment['support_surface'];tree=E.parse(ROOT/'worlds/lab.sdf');world=tree.getroot().find('world');floor=world.find("model[@name='floor']")
geometry=floor.find('link/collision/geometry');geometry.clear();child(child(geometry,'box'),'size',' '.join(map(str,support['size'])))
floor.find('pose').text=' '.join(map(str,support['position']+support.get('rpy',[0,0,0])))
E.indent(tree,space='  ');tree.write(ROOT/'worlds/lab.sdf',encoding='utf-8',xml_declaration=True)
print('Built official PiPER:',GEN/'piper.urdf')
