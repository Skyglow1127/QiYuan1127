#!/usr/bin/env python3
"""Read-only planar CAD reference diagnostic. Never validates joint mapping.

The supplied correspondences must identify actual CAD surface points, not an
occluded hole's imagined centre or an unknown-height screw head. Outputs remain
diagnostic candidates even if reprojection residuals are small.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import urllib.request

import cv2
import numpy as np


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def decode_snapshot(data, serial):
    if data.get('schema') != 'kr260-color-reference-v1':
        raise ValueError('Reference snapshot schema required')
    if data.get('device', {}).get('serial') != serial:
        raise ValueError('Unexpected camera serial')
    if not 0 <= float(data['board_frame_age_ms']) <= 250:
        raise ValueError('Stale image')
    raw = base64.b64decode(data['image_base64'], validate=True)
    if hashlib.sha256(raw).hexdigest() != data['image_sha256']:
        raise ValueError('Image hash mismatch')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    cal = data['calibration']
    if image is None or image.shape != (cal['height'], cal['width'], 3):
        raise ValueError('Image/intrinsics dimensions differ')
    if (data['frame']['width'],data['frame']['height']) != (cal['width'],cal['height']):
        raise ValueError('Frame/intrinsics dimensions differ')
    if cal.get('mirrored') is not False or cal.get('distortion_order') != 'k1,k2,p1,p2,k3,k4,k5,k6':
        raise ValueError('Unsupported distortion/mirroring convention')
    k, dist = np.asarray(cal['camera_matrix'],float), np.asarray(cal['distortion'],float)
    if k.shape != (3,3) or dist.shape != (8,) or not np.isfinite(k).all() or not np.isfinite(dist).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('Invalid intrinsics')
    return raw, image, k, dist


def planar_candidates(points_mm, pixels, k, dist):
    obj, uv = np.asarray(points_mm, float)/1000, np.asarray(pixels, float)
    if obj.ndim != 2 or obj.shape[1] != 3 or uv.shape != (len(obj),2) or len(obj)<4:
        raise ValueError('At least four identified point correspondences required')
    if not np.isfinite(obj).all() or not np.isfinite(uv).all():
        raise ValueError('Non-finite points')
    if len(np.unique(obj,axis=0)) != len(obj) or len(np.unique(uv,axis=0)) != len(uv):
        raise ValueError('Duplicate points')
    sv=np.linalg.svd(obj-obj.mean(axis=0),compute_uv=False)
    if sv[1] < .001 or sv[2] > 1e-6 or np.ptp(obj[:,2]) > 1e-6:
        raise ValueError('Noncollinear XY-planar reference required')
    if cv2.contourArea(cv2.convexHull(uv.astype(np.float32))) < 100:
        raise ValueError('Projected reference too small or degenerate')
    # IPPE uses a local z=0 plane; convert translation back to original link.
    z0 = float(obj[0,2]); flat=obj.copy();flat[:,2]-=z0
    solved=cv2.solvePnPGeneric(flat,uv,k,dist,flags=cv2.SOLVEPNP_IPPE)
    candidates=[]
    for rv,tv in zip(solved[1],solved[2]):
        rot=cv2.Rodrigues(rv)[0];t=tv.ravel()-rot@np.array([0,0,z0])
        depth=(obj@rot.T+t)[:,2]
        if np.any(depth<=0):continue
        projected=cv2.projectPoints(obj,rv,t,k,dist)[0].reshape(-1,2)
        rms=float(np.sqrt(np.mean(np.sum((projected-uv)**2,axis=1))))
        T=np.eye(4);T[:3,:3]=rot;T[:3,3]=t
        candidates.append(dict(T_camera_link=T.tolist(),reprojection_rms_px=rms))
    return sorted(candidates,key=lambda c:c['reprojection_rms_px'])


def estimate(snapshot, geometry, annotations):
    _, image, k, dist=decode_snapshot(snapshot,geometry['camera_serial'])
    if annotations.get('image_sha256') != snapshot['image_sha256']:
        raise ValueError('Annotations do not belong to this exact image')
    report=dict(schema='d1-mechanical-reference-diagnostic-v1',validated=False,
                hardware_motion_sent=False, operational_mapping_written=False,
                camera_serial=geometry['camera_serial'],image_sha256=snapshot['image_sha256'],references={})
    for name, reference in geometry['references'].items():
        row=dict(status='missing_correspondences',candidates=[],physical_correspondence_verified=False)
        annotation=annotations.get('references',{}).get(name,{})
        entries=annotation.get('points',[])
        identified=[e for e in entries if e.get('visible') is True and e.get('uv') is not None]
        row['visible_identified_points']=len(identified)
        if reference.get('physical_correspondence_verified') is not True:
            row['status']='CAD_to_physical_correspondence_unverified'
        elif len(identified)>=4:
            lookup={p['id']:p['xyz_mm'] for p in reference['points']}
            if len({p['id'] for p in identified})!=len(identified):raise ValueError('Repeated feature ID')
            xyz=[lookup[p['id']] for p in identified];uv=[p['uv'] for p in identified]
            if any(not (0<=u<image.shape[1] and 0<=v<image.shape[0]) for u,v in uv):
                raise ValueError('Pixel outside image')
            row['candidates']=planar_candidates(xyz,uv,k,dist)
            row['physical_correspondence_verified']=True
            row['status']='diagnostic_candidates_only' if row['candidates'] else 'no_positive_depth_solution'
        report['references'][name]=row
    # Deliberately never choose a planar ambiguity or output calibrated base->end.
    report['note']='Low reprojection error alone does not validate reference identity, intrinsics, screw-head height, or joint zeros.'
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
    a=sub.add_parser('capture');a.add_argument('--url',default='http://127.0.0.1:18761')
    a.add_argument('--serial',default='CC1365302K0');a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('estimate');a.add_argument('--snapshot',type=Path,required=True)
    a.add_argument('--geometry',type=Path,required=True);a.add_argument('--annotations',type=Path,required=True)
    a.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.mode=='capture':
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(args.url.rstrip('/')+'/reference-snapshot',timeout=4) as response:data=json.load(response)
        raw,_,_,_=decode_snapshot(data,args.serial)
        args.output.mkdir(parents=True,exist_ok=False)
        save(args.output/'snapshot.json',data);(args.output/'color.jpg').write_bytes(raw)
        save(args.output/'annotations.json',dict(image_sha256=data['image_sha256'],references={}))
        print(args.output)
    else:
        report=estimate(json.loads(args.snapshot.read_text()),json.loads(args.geometry.read_text()),json.loads(args.annotations.read_text()))
        save(args.output,report);print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
