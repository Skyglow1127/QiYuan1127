#!/usr/bin/env python3
import pathlib,subprocess,xml.etree.ElementTree as E,json,yaml
root=pathlib.Path(__file__).resolve().parents[1];gen=root/'generated';gen.mkdir(exist_ok=True)
subprocess.run(['python3',str(root/'scripts/build_wrist_camera.py')],check=True)
subprocess.run(['xacro',str(root/'urdf/d1t.urdf.xacro'),f'root:={root}',f'mesh_dir:={root}/meshes','-o',str(gen/'d1t.urdf')],check=True)
r=E.parse(gen/'d1t.urdf').getroot(); model={'links':[],'joints':[]}
for link in r.findall('link'):
 mesh=link.find('visual/geometry/mesh')
 # Camera visuals and collision boxes share their own configuration; avoid
 # adding its first mesh a second time through the arm CAD display / guard.
 if link.get('name')=='arm_wrist_camera_link':mesh=None
 model['links'].append({'name':link.get('name'),'mesh':None if mesh is None else '/meshes/'+pathlib.Path(mesh.get('filename')).name})
for j in r.findall('joint'):
 o=j.find('origin');a=j.find('axis');lim=j.find('limit')
 model['joints'].append({'name':j.get('name'),'type':j.get('type'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'xyz':[float(x) for x in o.get('xyz','0 0 0').split()],'rpy':[float(x) for x in o.get('rpy','0 0 0').split()],'axis':[float(x) for x in (a.get('xyz') if a is not None else '0 0 1').split()],'limits':None if lim is None else {k:float(v) for k,v in lim.attrib.items()}})
(gen/'model.json').write_text(json.dumps(model,indent=2))
bridges=[{'ros_topic_name':'/joint_states','gz_topic_name':'/d1t/joint_states','ros_type_name':'sensor_msgs/msg/JointState','gz_type_name':'gz.msgs.Model','direction':'GZ_TO_ROS'}, {'ros_topic_name':'/clock','gz_topic_name':'/clock','ros_type_name':'rosgraph_msgs/msg/Clock','gz_type_name':'gz.msgs.Clock','direction':'GZ_TO_ROS'}]
for j in model['joints']:
 if j['type']=='fixed':continue
 t='/d1t/'+j['name']+'/cmd_pos';bridges.append({'ros_topic_name':t,'gz_topic_name':t,'ros_type_name':'std_msgs/msg/Float64','gz_type_name':'gz.msgs.Double','direction':'ROS_TO_GZ'})
(gen/'bridge.yaml').write_text(yaml.safe_dump(bridges))
print('Built',gen/'d1t.urdf')

(gen/"robot_state.yaml").write_text(yaml.safe_dump({"robot_state_publisher":{"ros__parameters":{"use_sim_time":True,"robot_description":(gen/"d1t.urdf").read_text()}}}))

# Generate the physical red cube from its parameterized Xacro source.
scene=json.loads((root/'config/grasp_scene.json').read_text()); cube=scene['cube']
model_dir=root/'models/red_cube';model_dir.mkdir(parents=True,exist_ok=True)
subprocess.run(['xacro',str(root/'urdf/red_cube.sdf.xacro'),f"edge:={cube['size'][0]}",f"mass:={cube['mass']}","xyz:="+" ".join(map(str,cube['position'])),'-o',str(model_dir/'model.sdf')],check=True)
(model_dir/'model.config').write_text('<model><name>red_cube</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>40 mm red cube, physical friction grasp</description></model>')

# The photo-based workspace shares one editable geometry manifest with the
# browser and the FCL path guard. Keep the original support contact identity.
environment_path=root/'config/environment.json'
if environment_path.exists():
 subprocess.run(['python3',str(root/'scripts/build_environment.py')],check=True)
 environment=json.loads(environment_path.read_text())
 support=environment['support_surface']
 world_path=root/'worlds/lab.sdf';tree=E.parse(world_path);world=tree.getroot().find('world')
 floor=world.find("model[@name='floor']")
 floor.find('link/collision/geometry').clear()
 box=E.SubElement(floor.find('link/collision/geometry'),'box')
 E.SubElement(box,'size').text=' '.join(map(str,support['size']))
 pose=floor.find('pose')
 if pose is None:pose=E.SubElement(floor,'pose')
 pose.text=' '.join(map(str,support['position']+support.get('rpy',[0,0,0])))
 # The visible desktop is generated with the other photo environment objects.
 for visual in floor.findall('link/visual'):floor.find('link').remove(visual)
 if not any(i.findtext('uri')=='model://photo_workspace' for i in world.findall('include')):
  include=E.SubElement(world,'include');E.SubElement(include,'uri').text='model://photo_workspace'
 E.indent(tree,space='  ');tree.write(world_path,encoding='utf-8',xml_declaration=True)
