#!/usr/bin/env python3
"""Adapt a COPY of vendor CAD URDF. Does not change D1_SDK or vendor files."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as E
import yaml
ws = Path(__file__).resolve().parents[1]
src = ws.parent / 'models/vendor/d1_description'
pkg = ws / 'src/d1_pick_place'
robot = E.parse(src / 'urdf/d1_description.urdf').getroot()
for mesh in robot.findall('.//mesh'):
    mesh.set('filename', mesh.get('filename').replace('package://d1_description/', 'package://d1_pick_place/'))
for joint in robot.findall('joint'):
    # CAD export has zero velocities: these are conservative software planning
    # caps, NOT manufacturer ratings or a measured hardware limit.
    joint.find('limit').set('velocity', '0.2' if joint.get('type') == 'revolute' else '0.01')
E.SubElement(robot.find("joint[@name='Joint7_2']"), 'mimic', joint='Joint7_1', multiplier='-1', offset='0')
E.SubElement(robot, 'link', name='grasp_tcp')
joint = E.SubElement(robot, 'joint', name='nominal_tcp', type='fixed')
E.SubElement(joint, 'parent', link='Link6')
E.SubElement(joint, 'child', link='grasp_tcp')
# CAD-derived nominal grasp centre; explicitly NOT calibrated for hardware.
E.SubElement(joint, 'origin', xyz='-0.00562 0 0.11', rpy='0 0 1.5707963267948966')
E.indent(robot)
E.ElementTree(robot).write(pkg / 'config/d1.urdf', encoding='unicode', xml_declaration=True)
srdf = E.Element('robot', name='d1_description')
g = E.SubElement(srdf, 'group', name='arm')
E.SubElement(g, 'chain', base_link='base_link', tip_link='grasp_tcp')
g = E.SubElement(srdf, 'group', name='gripper')
for name in ['Joint7_1', 'Joint7_2']:
    E.SubElement(g, 'joint', name=name)
E.SubElement(srdf, 'end_effector', name='hand', parent_link='Link6', group='gripper', parent_group='arm')
for j in robot.findall('joint'):
    E.SubElement(srdf, 'disable_collisions', link1=j.find('parent').get('link'),
                 link2=j.find('child').get('link'), reason='Adjacent')
E.indent(srdf)
E.ElementTree(srdf).write(pkg / 'config/d1.srdf', encoding='unicode', xml_declaration=True)
limits = {}
for j in robot.findall('joint'):
    if j.get('type') == 'fixed': continue
    limits[j.get('name')] = {'has_velocity_limits': True, 'max_velocity': float(j.find('limit').get('velocity')),
                            'has_acceleration_limits': True,
                            'max_acceleration': .3 if j.get('type') == 'revolute' else .02}
(pkg / 'config/joint_limits.yaml').write_text(yaml.safe_dump({'joint_limits': limits}))
for f in (src / 'meshes').glob('*.STL'):
    shutil.copy2(f, pkg / 'meshes' / f.name)
print('Generated model copy, conservative planner limits, adjacent-pair SRDF and nominal TCP')
