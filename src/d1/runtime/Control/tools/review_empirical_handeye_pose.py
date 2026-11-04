#!/usr/bin/env python3
"""Replay a saved pose with frozen intrinsics; no hardware or config writes."""
import argparse
import datetime
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import diagnose_handeye_geometry as geometry
import kr260_handeye as helper
import kr260_handeye_live as live
from j4_smooth_test import JOINT_LIMITS
from review_joint_mapping import fk


def mean_pose(cap, obj, k, d, detector):
    rotations, translations, residuals = [], [], []
    for f in cap['frames']:
        r, t, rv = geometry.solve(obj, f[detector], k, d, 'iterative')
        if np.min((obj @ r.T + t.ravel())[:, 2]) <= 0:
            raise ValueError('Nonpositive board depth')
        rotations.append(r); translations.append(t.ravel())
        projected = cv2.projectPoints(obj, rv, t, k, d)[0].reshape(-1, 2)
        residuals.append(np.linalg.norm(projected-f[detector], axis=1))
    rr = Rotation.from_matrix(rotations)
    mean = rr.mean()
    t = np.eye(4); t[:3,:3] = mean.as_matrix(); t[:3,3] = np.mean(translations, axis=0)
    return dict(T_camera_board=t.tolist(), rms_px=float(np.sqrt(np.mean(np.array(residuals)**2))),
                rotation_repeatability_max_deg=float(np.rad2deg((mean.inv()*rr).magnitude()).max()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    p.add_argument('capture', type=Path)
    p.add_argument('--previous', type=Path)
    p.add_argument('--split', choices=['train','validation'], required=True)
    p.add_argument('--origin-label', required=True,
                   help='Physical corner label checked by the caller on the annotated image')
    args = p.parse_args()
    out = args.capture/'trial_review.json'
    if out.exists():
        raise ValueError('Review already exists')
    protocol = json.loads((args.root/'collection_protocol.json').read_text())
    candidate_path = Path(protocol['camera_intrinsics_file'])
    if helper.digest(candidate_path) != protocol['camera_intrinsics_sha256']:
        raise ValueError('Frozen intrinsics changed')
    candidate = json.loads(candidate_path.read_text())
    board = json.loads(live.DEFAULT_BOARD.read_text())
    _, obj = live.geometry(board)
    # Classic corners are an optional diagnostic; all original raw-hash,
    # feedback, complete-board SB and reprojection checks still run.
    record=json.loads((args.capture/'capture.json').read_text())
    assessor=None
    if record['schema']=='kr260-live-quantized-paired-v1':
        from quantized_stationarity import assess_window
        assessor=assess_window
        if helper.digest(Path(__file__).with_name('quantized_stationarity.py'))!=record['quality_assessor_sha256']:
            raise ValueError('Quantized sampling assessor changed')
        if record['visual_max_corner_shift_px']>.5 or record['visual_center_shift_px']>.15:
            raise ValueError('Quantized-sample visual stability failed')
    cap = geometry.load_capture(args.capture, board, require_classic=False,stationarity_assessor=assessor)
    if assessor:
        pp=np.array([f['sb'] for f in cap['frames']])
        if (np.linalg.norm(pp-pp[0],axis=2).max()>.5+1e-5 or
            np.linalg.norm(pp.mean(axis=1)-pp[0].mean(axis=0),axis=1).max()>.15+1e-4):
            raise ValueError('Replayed image stationarity failed')
    q = np.array(cap['q'])
    if any(not lo <= v <= hi for v,(lo,hi) in zip(q, JOINT_LIMITS)):
        raise ValueError('Capture feedback outside existing limits')
    if any(f['meta']['calibration'] != candidate['original_sdk_calibration'] or
           f['meta']['device']['serial'] != candidate['camera_serial'] for f in cap['frames']):
        raise ValueError('Camera identity/geometry changed')
    k = np.array(candidate['selected_candidate']['camera_matrix'])
    d = np.array(candidate['selected_candidate']['distortion'])
    estimates = {'sb':mean_pose(cap, obj, k, d, 'sb')}
    if all(f['classic'] is not None for f in cap['frames']):
        estimates['classic'] = mean_pose(cap, obj, k, d, 'classic')
    if estimates['sb']['rms_px'] > 1:
        raise ValueError('Frozen-intrinsic reprojection RMS >1 px')
    xy = np.mean([f['sb'] for f in cap['frames']], axis=0)
    tbe = fk(np.deg2rad(q[:6]))
    report = dict(schema='empirical-handeye-pose-v1',
        reviewed_at=datetime.datetime.now().astimezone().isoformat(),
        capture=str(args.capture.resolve()),capture_sha256=cap['capture_sha256'],split=args.split,
        fixed_board_epoch=protocol['fixed_board_epoch'], origin_label=args.origin_label,
        origin_reviewer='Codex annotated-image visual review', joint_median=q.tolist(),
        feedback_replayed=True, all_joint_limits_checked=True,sampling_schema=record['schema'],
        candidate_sha256=helper.digest(candidate_path), T_base_end_nominal=tbe.tolist(),
        poses=estimates,classic_diagnostic_errors=[f['classic_error'] for f in cap['frames']],
        corner_bounds_px=[xy.min(axis=0).tolist(),xy.max(axis=0).tolist()],
        center_px=xy.mean(axis=0).tolist(),robot_commands_sent=False,
        operational_mapping_validated=False,handeye_calibrated=False)
    if args.previous:
        prev=json.loads((args.previous/'trial_review.json').read_text())
        if prev['fixed_board_epoch'] != report['fixed_board_epoch'] or prev['origin_label'] != args.origin_label:
            raise ValueError('Mixed board frame')
        a=np.array(prev['T_base_end_nominal'])[:3,:3].T @ tbe[:3,:3]
        pair=dict(previous=str(args.previous.resolve()),dq_deg=(q-np.array(prev['joint_median'])).tolist(),
                  nominal_all_joint_rotation_deg=float(np.rad2deg(Rotation.from_matrix(a).magnitude())))
        for det in ('sb','classic'):
            if det not in estimates or det not in prev['poses']:
                continue
            c0=np.array(prev['poses'][det]['T_camera_board']); c1=np.array(estimates[det]['T_camera_board'])
            b=c0[:3,:3] @ c1[:3,:3].T
            value=float(np.rad2deg(Rotation.from_matrix(b).magnitude()))
            if value > 10:
                raise ValueError('Unexpected large visual rotation: review board movement/origin')
            pair[det+'_visual_rotation_deg']=value
        report['relative_motion']=pair
    live.save(out,report)
    review=json.loads((args.capture/'orientation_review.json').read_text())
    review.update(reviewed=True,physical_origin_label=args.origin_label,reviewer='Codex',
                  note='Annotated image inspected; same physical inner corner in this fixed-board epoch. Nominal robot mapping remains unvalidated.')
    live.save(args.capture/'orientation_review.json',review)
    dataset_path=args.root/'dataset.json'
    dataset=json.loads(dataset_path.read_text()) if dataset_path.exists() else dict(
        schema='empirical-handeye-dataset-v1',fixed_board_epoch=protocol['fixed_board_epoch'],poses=[])
    if any(Path(row['capture']).resolve()==args.capture.resolve() for row in dataset['poses']):
        raise ValueError('Duplicate capture in dataset')
    for row in dataset['poses']:
        old=json.loads(Path(row['review']).read_text())
        if np.max(np.abs(np.array(old['joint_median'])[:6]-q[:6])) < .5:
            raise ValueError('Near-duplicate robot pose; excluded from dataset')
    dataset['poses'].append(dict(capture=str(args.capture.resolve()),review=str(out.resolve()),
                                 review_sha256=helper.digest(out),split=args.split))
    live.save(dataset_path,dataset)
    live.save(args.root/'progress.json',dict(updated_at=report['reviewed_at'],
        phase='collecting_'+args.split,training_views=sum(r['split']=='train' for r in dataset['poses']),
        validation_views=sum(r['split']=='validation' for r in dataset['poses']),
        latest_accepted_capture=str(args.capture.resolve()),joint_median=q.tolist(),
        independent_mechanical_angle_check_required=False,runtime_calibration_written=False,
        handeye_calibrated=False))
    print(json.dumps(dict(capture=str(args.capture),q=q.tolist(),bounds=report['corner_bounds_px'],
                          rms_px=estimates['sb']['rms_px'],relative_motion=report.get('relative_motion'),
                          accepted_pose_count=len(dataset['poses'])),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
