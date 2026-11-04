"""URDF forward kinematics for model audit and offline fixtures only."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import transform


def forward(urdf, values, tip='grasp_tcp'):
    root = ET.parse(urdf).getroot()
    poses = {'base_link': np.eye(4)}
    remaining = list(root.findall('joint'))
    while remaining:
        progress = False
        for j in remaining[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in poses:
                continue
            origin = j.find('origin')
            xyz = np.array(list(map(float, origin.get('xyz', '0 0 0').split())))
            rpy = np.array(list(map(float, origin.get('rpy', '0 0 0').split())))
            motion = np.eye(4)
            if j.get('type') != 'fixed':
                axis = np.array(list(map(float, j.find('axis').get('xyz').split())))
                q = values.get(j.get('name'), 0.)
                mimic = j.find('mimic')
                if mimic is not None:
                    q = values.get(mimic.get('joint'), 0.) * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
                if j.get('type') in ['revolute', 'continuous']:
                    motion[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
                elif j.get('type') == 'prismatic':
                    motion[:3, 3] = axis * q
            poses[child] = poses[parent] @ transform(Rotation.from_euler('xyz', rpy).as_matrix(), xyz) @ motion
            remaining.remove(j)
            progress = True
        if not progress:
            raise ValueError('Invalid URDF tree')
    return poses[tip]
