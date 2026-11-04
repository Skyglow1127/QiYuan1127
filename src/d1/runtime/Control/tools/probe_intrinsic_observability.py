#!/usr/bin/env python3
"""Fit candidate intrinsics offline; diagnose sensitivity, never install them.

Train/test split is by robot pose, not adjacent video frames. Held-out PnP
reprojection is camera-model transfer only, not an independent angle reference.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import diagnose_handeye_geometry as geometry
import kr260_handeye_live as live
import kr260_handeye as helper


def pose_errors(obj, images, k, d):
    errors, rotations = [], []
    for image in images:
        r, t, rv = geometry.solve(obj, image, k, d, 'iterative')
        residual = cv2.projectPoints(obj, rv, t, k, d)[0].reshape(-1, 2)-image
        errors.append(float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))))
        rotations.append(r)
    return errors, rotations


def fit(obj, images, k, d, model, train, scale):
    k = k.copy(); k[0,0] *= scale; k[1,1] *= scale
    flags = cv2.CALIB_USE_INTRINSIC_GUESS
    if model != 'brown9':
        flags |= (cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
                  cv2.CALIB_FIX_TANGENT_DIST)
    if model in ('focal1', 'focal2'):
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if model == 'focal1':
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
    rms, k, d, _, _, sd, _, _ = cv2.calibrateCameraExtended(
        [obj.astype(np.float32) for _ in train],
        [images[i].astype(np.float32) for i in train], (640,480), k, d[:5].copy(),
        flags=flags, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 300, 1e-10))
    errors, rotations = pose_errors(obj, images, k, d)
    heldout = [i for i in range(len(images)) if i not in train]
    return dict(model=model, initial_focal_scale=scale, train=train, heldout=heldout,
                fit_rms_px=float(rms), camera_matrix=k.tolist(), distortion=d.ravel().tolist(),
                intrinsics_linearized_stddev=sd.ravel().tolist(), per_pose_rms_px=errors,
                heldout_mean_rms_px=float(np.mean([errors[i] for i in heldout])) if heldout else None,
                first_three_relative_angles_deg=[geometry.angle(a,b) for a,b in zip(rotations[:3],rotations[1:4])])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('sessions',type=Path,nargs=6)
    a=p.parse_args()
    if a.output.exists(): raise ValueError('Fresh output required')
    board=json.loads(live.DEFAULT_BOARD.read_text());_,obj=live.geometry(board)
    captures=[geometry.load_capture(folder,board) for folder in a.sessions]
    k=captures[0]['frames'][0]['k'];d=captures[0]['frames'][0]['d']
    if any(not np.array_equal(f['k'],k) or not np.array_equal(f['d'],d) for c in captures for f in c['frames']):
        raise ValueError('Mixed source intrinsics')
    images=[np.mean([f['sb'] for f in c['frames']],axis=0) for c in captures]
    baseline_errors,rotations=pose_errors(obj,images,k,d)
    normal_vectors=np.array([r[:,2] for r in rotations])
    normal_span=max(float(np.rad2deg(np.arccos(np.clip(np.dot(x,y),-1,1)))) for x in normal_vectors for y in normal_vectors)
    # Zhang's two homography constraints per view. Column scaling removes
    # arbitrary magnitudes; near-null directions still show weak excitation.
    hs=[cv2.findHomography(obj[:,:2],cv2.undistortPoints(im.reshape(-1,1,2),k,d,P=k).reshape(-1,2),0)[0] for im in images]
    def vij(h,i,j):
        x,y=h[:,i],h[:,j]
        return np.array([x[0]*y[0],x[0]*y[1]+x[1]*y[0],x[1]*y[1],
                         x[2]*y[0]+x[0]*y[2],x[2]*y[1]+x[1]*y[2],x[2]*y[2]])
    v=np.vstack([w for h in hs for w in (vij(h,0,1),vij(h,0,0)-vij(h,1,1))])
    s=np.linalg.svd(v/np.maximum(np.linalg.norm(v,axis=0),1e-15),compute_uv=False)
    runs=[]
    for model in ('focal1','focal2','intrinsic4','brown9'):
        for scale in (.8,1.,1.2):
            runs.append(fit(obj,images,k,d,model,[0,1,2,3],scale))
    folds=[fit(obj,images,k,d,'intrinsic4',[i for i in range(6) if i!=j],1.) for j in range(6)]
    report=dict(schema='intrinsic-identifiability-probe-v1',diagnostic_only=True,
                runtime_calibration_accepted=False,robot_commands_sent=False,
                data_pose_order=[str(x.resolve()) for x in a.sessions],
                capture_sha256=[c['capture_sha256'] for c in captures],script_sha256=helper.digest(__file__),
                input_K=k.tolist(),input_distortion=d.tolist(),sdk_baseline_rms_px=baseline_errors,
                normal_span_deg=normal_span,column_normalized_zhang_singular_values=s.tolist(),
                center_pixels=[im.mean(axis=0).tolist() for im in images],runs=runs,leave_one_pose_out=folds,
                limitations=['Only six local poses; all derived from the same camera and physical board.',
                    'Frame averaging does not create independent viewpoints.',
                    'Fixed SDK distortion variants assume those coefficients, not validate them.',
                    'Held-out reprojection refits board pose and cannot establish angular ground truth.',
                    'Small reprojection alone is insufficient to authorize these candidate intrinsics.',
                    'Linearized parameter deviations assume the specified camera/flat-board model.'])
    a.output.mkdir(parents=True)
    (a.output/'candidates.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(model=r['model'],scale=r['initial_focal_scale'],K=r['camera_matrix'],
                           rms=r['fit_rms_px'],validation=r['heldout_mean_rms_px'],
                           angles=r['first_three_relative_angles_deg']) for r in runs],indent=2))


if __name__=='__main__': main()
