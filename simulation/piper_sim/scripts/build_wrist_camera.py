#!/usr/bin/env python3
"""Generate the fixed wrist camera visuals, collision envelopes and RGB sensor.

Dimensions, mount and optical parameters remain explicit estimates in JSON.
The fixed camera geometry merges into the existing wrist rigid body; no
unmeasured payload inertia is added to the calibrated-by-source CAD values.
"""
import json
import pathlib
import xml.etree.ElementTree as E
from import_wrist_camera_mesh import import_mesh

ROOT = pathlib.Path(__file__).resolve().parents[1]
XACRO = 'http://www.ros.org/wiki/xacro'
E.register_namespace('xacro', XACRO)

def values(v):
    return ' '.join(str(x) for x in v)

def geometry(parent, shape):
    geo = E.SubElement(parent, 'geometry')
    kind = shape.get('geometry', 'box')
    if kind == 'mesh':
        E.SubElement(geo,'mesh',{'filename':str(ROOT/shape['mesh'].lstrip('/')),'scale':values(shape.get('scale',[1,1,1]))})
        return
    attrs = {'size': values(shape['size'])} if kind == 'box' else {'radius': str(shape['radius'])}
    if kind == 'cylinder':
        attrs['length'] = str(shape['length'])
    E.SubElement(geo, kind, attrs)

def main():
    c = json.loads((ROOT/'config/wrist_camera.json').read_text())
    import_mesh(c)
    root = E.Element('robot')
    macro = E.SubElement(root, '{'+XACRO+'}macro', {'name':'photo_wrist_camera','params':''})
    link = E.SubElement(macro, 'link', {'name':c['link']})
    for shape in c['shapes']:
        visual = E.SubElement(link, 'visual', {'name':shape['name']})
        E.SubElement(visual, 'origin', {'xyz':values(shape['position']), 'rpy':values(shape.get('rpy',[0,0,0]))})
        geometry(visual,shape)
        mat = E.SubElement(visual,'material',{'name':shape['name']+'_finish'})
        color = shape['color']
        E.SubElement(mat,'color',{'rgba':values([int(color[i:i+2],16)/255 for i in (1,3,5)]+[1])})
    for shape in c['colliders']:
        collider = E.SubElement(link,'collision',{'name':shape['name']})
        E.SubElement(collider,'origin',{'xyz':values(shape['position']),'rpy':values(shape.get('rpy',[0,0,0]))})
        geometry(collider,shape)
    joint = E.SubElement(macro,'joint',{'name':'piper_wrist_camera_mount','type':'fixed'})
    E.SubElement(joint,'parent',{'link':c['parent']})
    E.SubElement(joint,'child',{'link':c['link']})
    E.SubElement(joint,'origin',{'xyz':values(c['mount']['xyz']),'rpy':values(c['mount']['rpy'])})
    optical = 'piper_wrist_camera_optical_link'
    E.SubElement(macro,'link',{'name':optical})
    joint = E.SubElement(macro,'joint',{'name':'piper_wrist_camera_optical_joint','type':'fixed'})
    E.SubElement(joint,'parent',{'link':c['link']})
    E.SubElement(joint,'child',{'link':optical})
    settings = c['sensor']
    E.SubElement(joint,'origin',{'xyz':values(settings['optical_xyz']),'rpy':values(settings['optical_rpy'])})
    gazebo = E.SubElement(macro,'gazebo',{'reference':optical})
    sensor = E.SubElement(gazebo,'sensor',{'name':settings['name'],'type':'camera'})
    for name,value in [('always_on','true'),('update_rate',settings['update_rate']),('visualize','false'),('topic',settings['topic'])]:
        E.SubElement(sensor,name).text=str(value)
    camera = E.SubElement(sensor,'camera',{'name':'wrist_camera'})
    E.SubElement(camera,'horizontal_fov').text=str(settings['horizontal_fov'])
    image = E.SubElement(camera,'image')
    for name,value in [('width',settings['width']),('height',settings['height']),('format','R8G8B8')]:
        E.SubElement(image,name).text=str(value)
    clip = E.SubElement(camera,'clip')
    E.SubElement(clip,'near').text=str(settings['near'])
    E.SubElement(clip,'far').text=str(settings['far'])
    E.indent(root,space='  ')
    output=ROOT/'generated/wrist_camera.xacro'
    output.parent.mkdir(exist_ok=True)
    E.ElementTree(root).write(output,encoding='utf-8',xml_declaration=True)
    print('Built',output)

if __name__ == '__main__':
    main()
