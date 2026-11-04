#!/usr/bin/env python3
"""Read vendor URDF/STL; export nominal dimensions, never calibration values."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def triangles_mm(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    dtype = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    if len(data) != 84 + count * 50:
        raise ValueError(f'Not a supported binary STL: {path}')
    return np.frombuffer(data, dtype=dtype, offset=84, count=count)['v'].astype(float) * 1000


def circular_contours(v):
    """Fit closed circular boundaries on XY planes; do not classify thread/bore."""
    flat = v[np.ptp(v[:, :, 2], axis=1) < 1e-4]
    planes = np.round(flat[:, 0, 2], 3)
    results = []
    for z in np.unique(planes):
        edges = Counter()
        for triangle in flat[planes == z]:
            points = [tuple(p) for p in np.round(triangle[:, :2], 4)]
            for i, j in [(0, 1), (1, 2), (2, 0)]:
                edges[tuple(sorted((points[i], points[j])))] += 1
        adjacent = defaultdict(set)
        for (a, b), count in edges.items():
            if count == 1:
                adjacent[a].add(b)
                adjacent[b].add(a)
        seen = set()
        for point in adjacent:
            if point in seen:
                continue
            pending, component = [point], []
            while pending:
                p = pending.pop()
                if p in seen:
                    continue
                seen.add(p)
                component.append(p)
                pending.extend(adjacent[p] - seen)
            if len(component) < 8 or any(len(adjacent[p]) != 2 for p in component):
                continue
            xy = np.asarray(component)
            design = np.column_stack([2 * xy, np.ones(len(xy))])
            fit = np.linalg.lstsq(design, np.sum(xy * xy, axis=1), rcond=None)[0]
            radii = np.linalg.norm(xy - fit[:2], axis=1)
            error = np.max(np.abs(radii - radii.mean()))
            if error < .03 and radii.mean() < 15:
                results.append(dict(center_mm=[*fit[:2].tolist(), float(z)],
                                    radius_mm=float(radii.mean()),
                                    max_radial_fit_residual_mm=float(error),
                                    vertices=len(component)))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    control = Path(__file__).resolve().parents[1]
    vendor = control / 'models/vendor/d1_description'
    meshes = {}
    for path in sorted((vendor / 'meshes').glob('*.STL')):
        v = triangles_mm(path)
        lo, hi = v.min(axis=(0, 1)), v.max(axis=(0, 1))
        meshes[path.stem] = dict(source=str(path), local_min_mm=lo.tolist(),
                                 local_max_mm=hi.tolist(), envelope_xyz_mm=(hi-lo).tolist())
        if path.stem in ('base_link', 'Link6'):
            meshes[path.stem]['xy_circular_contours'] = circular_contours(v)
    root = ET.parse(vendor / 'urdf/d1_description.urdf').getroot()
    joints = []
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        joints.append(dict(name=joint.get('name'), type=joint.get('type'),
                           parent=joint.find('parent').get('link'),
                           child=joint.find('child').get('link'),
                           origin_xyz_mm=[float(x)*1000 for x in origin.get('xyz').split()],
                           origin_rpy_rad=[float(x) for x in origin.get('rpy').split()],
                           axis=[float(x) for x in joint.find('axis').get('xyz').split()]))
    result = dict(model='Unitree CDN D1-550 gripper URDF/STL',
                  status='nominal_CAD_only_not_physical_calibration',
                  units='mm; joint rpy in rad',
                  cautions=['Mesh envelopes are in each link local frame, not assembled overall dimensions.',
                            'Circular contours may be holes, bosses, or screw surfaces; no thread specification inferred.',
                            'STL fit residual is not measurement accuracy or manufacturing tolerance.',
                            'Physical feature correspondence, joint offsets and custom fixtures are unvalidated.'],
                  meshes=meshes, joints=joints)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'nominal_dimensions.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    lines = ['D1-T 名义尺寸提取表（单位 mm，旋转为 rad）',
             '来源：本地保存的宇树 CDN D1-550 gripper 模型；仅名义 CAD，不是实测标定。', '',
             '连杆局部 XYZ 方向包围盒：']
    for name, mesh in meshes.items():
        lines.append(f"{name}: " + ' × '.join(f'{v:.3f}' for v in mesh['envelope_xyz_mm']))
    lines += ['', '关节原点：父坐标系平移 XYZ；RPY；局部旋转轴。不能将平移分量直接当成连杆总长。']
    for joint in joints:
        lines.append(f"{joint['name']} {joint['parent']} -> {joint['child']}: "
                     f"XYZ={joint['origin_xyz_mm']}; RPY={joint['origin_rpy_rad']}; axis={joint['axis']}")
    (args.output / '全部连杆与关节名义尺寸.txt').write_text('\n'.join(lines)+'\n')
    print(args.output / 'nominal_dimensions.json')
    print(args.output / '全部连杆与关节名义尺寸.txt')


if __name__ == '__main__':
    main()
