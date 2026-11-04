#!/usr/bin/env python3
"""Offline metric pose from explicitly annotated vertices of a known cube.

No camera/robot connection. Pixel coordinates and image hash are retained so
manual correspondence is never confused with automatic live tracking.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/d1_pick_place'))
from d1_pick_place.kinematics import forward
from d1_pick_place.sdk_executor import Mapping, NAMES


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observation', type=Path, required=True)
    p.add_argument('--vertices', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    o = json.loads(args.observation.read_text())
    annotation = json.loads(args.vertices.read_text())
    c = json.loads(args.candidate.read_text())
    im = cv2.imread(o['image'])
    if hashlib.sha256(Path(o['image']).read_bytes()).hexdigest() != o['image_sha256']:
        raise ValueError('Observed image changed')
    if annotation['image_sha256'] != o['image_sha256']:
        raise ValueError('Vertex annotations belong to a different image')
    pixels = np.array(annotation['pixels'], float)
    size = float(annotation['cube_size_m'])
    # +X: right, +Y: back, +Z: up. This handedness matters for two visible faces.
    obj = np.array([[-1,1,1],[1,1,1],[1,-1,1],[-1,-1,1],[1,-1,-1],[-1,-1,-1]], float) * size / 2
    if 'object_points_m' in annotation:
        obj = np.array(annotation['object_points_m'], float)
        if obj.shape not in [(4,3), (6,3)] or not np.isfinite(obj).all() or not np.allclose(abs(obj), size/2) or len(np.unique(obj, axis=0)) != len(obj):
            raise ValueError('Explicit correspondences must be four or six distinct cube vertices')
    if pixels.shape != (len(obj),2) or not np.isfinite(pixels).all() or not .01 <= size <= .1:
        raise ValueError('Malformed annotation')
    k, d = np.array(c['camera_matrix']), np.array(c['distortion'])
    ok, r, t = cv2.solvePnP(obj, pixels, k, d, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError('Cube PnP failed')
    r, t = cv2.solvePnPRefineLM(obj, pixels, k, d, r, t)
    T = np.eye(4); T[:3,:3] = cv2.Rodrigues(r)[0]; T[:3,3] = t.ravel()
    projection = cv2.projectPoints(obj, r, t, k, d)[0].reshape(-1,2)
    errors = np.linalg.norm(projection-pixels, axis=1)
    if (obj @ T[:3,:3].T + T[:3,3])[:,2].min() <= 0 or errors.max() > 3:
        raise ValueError('Cube fit inconsistent with annotated image')
    mapping = Mapping.from_candidate(args.candidate)
    values = dict(zip(NAMES, mapping.to_model(o['sdk_angles'])))
    B = forward(args.urdf, values, 'Link6') @ np.array(c['T_end_camera'])
    cube = B @ T
    table = cube.copy(); table[:3,3] -= size/2 * cube[:3,2]
    report = dict(schema='d1-first-cube-pose-v1', hardware_executed=False,
        human_grasp_approval_received=False, captured_at=o['captured_at'],
        source_observation=str(args.observation.resolve()),
        source_vertices=str(args.vertices.resolve()),
        candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
        cube_size_m=size, annotated_pixels=pixels.tolist(),
        object_points_m=obj.tolist(),
        annotation_method=f'assistant visually annotated {len(obj)} vertices; no live pose tracker',
        image_sha256=o['image_sha256'], sdk_start=o['sdk_angles'],
        T_camera_cube=T.tolist(), T_base_camera=B.tolist(), T_base_cube=cube.tolist(),
        T_base_table=table.tolist(), T_base_nominal_tcp=forward(args.urdf,values).tolist(),
        reprojection_rms_px=float(np.sqrt(np.mean(errors**2))),
        vertex_errors_px=errors.tolist(),
        table_normal_base=cube[:3,2].tolist(),
        limitations=['40 mm size supplied by user; flat supporting surface assumed',
                     'Uses local empirical hand-eye mapping and nominal CAD TCP',
                     'Current observation must be refreshed before approved execution',
                     'No aligned depth, force measurement or collision-free path implied'])
    args.output.write_text(json.dumps(report, indent=2))
    for i, (seen, fitted) in enumerate(zip(pixels, projection)):
        a=tuple(np.rint(seen).astype(int)); b=tuple(np.rint(fitted).astype(int))
        cv2.circle(im,a,3,(0,255,255),-1);cv2.circle(im,b,5,(0,255,0),1)
        cv2.putText(im,str(i), (a[0]+5,a[1]),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
    cv2.putText(im,'40 mm cube / offline pose candidate',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    cv2.imwrite(str(args.output.with_suffix('.jpg')),im)
    print(json.dumps(dict(base_cube_center_m=cube[:3,3].tolist(),
                         reprojection_rms_px=report['reprojection_rms_px'],
                         hardware_executed=False)))


if __name__ == '__main__':
    main()
