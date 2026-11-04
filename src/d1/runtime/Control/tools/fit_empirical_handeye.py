#!/usr/bin/env python3
"""Offline hand-eye fit or frozen-model validation for an explicit board epoch.

Uses the nominal robot mapping declared by the session. Does not validate joint
mechanics, install a transform, or write any operational calibration file.
"""
import argparse
import datetime
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import kr260_handeye as helper
import kr260_handeye_live as live


METHODS = {'PARK':cv2.CALIB_HAND_EYE_PARK,'TSAI':cv2.CALIB_HAND_EYE_TSAI,
           'HORAUD':cv2.CALIB_HAND_EYE_HORAUD,'ANDREFF':cv2.CALIB_HAND_EYE_ANDREFF,
           'DANIILIDIS':cv2.CALIB_HAND_EYE_DANIILIDIS}


def rigid(r,t):
    if (not np.isfinite(r).all() or not np.isfinite(t).all() or
        np.linalg.norm(r.T@r-np.eye(3)) > 1e-6 or abs(np.linalg.det(r)-1) > 1e-6):
        raise ValueError('Solver did not return a finite proper rigid transform')
    x=np.eye(4);x[:3,:3]=r;x[:3,3]=np.asarray(t).ravel();return x


def solve(ee,cc,method):
    r,t=cv2.calibrateHandEye([e[:3,:3] for e in ee],[e[:3,3] for e in ee],
                            [c[:3,:3] for c in cc],[c[:3,3] for c in cc],method=METHODS[method])
    return rigid(r,t)


def mean_transform(poses):
    result=np.eye(4)
    result[:3,:3]=Rotation.from_matrix([p[:3,:3] for p in poses]).mean().as_matrix()
    result[:3,3]=np.mean([p[:3,3] for p in poses],axis=0)
    return result


def assess(ee,cc,x,y,limits):
    poses=[e@x@c for e,c in zip(ee,cc)]
    te=np.array([np.linalg.norm(p[:3,3]-y[:3,3]) for p in poses])
    re=np.array([np.rad2deg(Rotation.from_matrix(y[:3,:3].T@p[:3,:3]).magnitude()) for p in poses])
    result=dict(translation_rms_m=float(np.sqrt(np.mean(te**2))),rotation_rms_deg=float(np.sqrt(np.mean(re**2))),
                max_translation_m=float(te.max()),max_rotation_deg=float(re.max()),
                per_pose_translation_m=te.tolist(),per_pose_rotation_deg=re.tolist())
    result['empirical_residual_thresholds_pass']=(
        result['translation_rms_m']<=limits['training_and_heldout_translation_rms_m'] and
        result['rotation_rms_deg']<=limits['training_and_heldout_rotation_rms_deg'] and
        result['max_translation_m']<=limits['max_translation_m'] and
        result['max_rotation_deg']<=limits['max_rotation_deg'])
    return result


def load_rows(root,split,protocol):
    dataset=json.loads((root/'dataset.json').read_text());rows=[]
    if dataset['fixed_board_epoch'] != protocol['fixed_board_epoch']:
        raise ValueError('Mixed board epoch')
    for row in dataset['poses']:
        if row['split'] != split:continue
        if helper.digest(row['review']) != row['review_sha256']:
            raise ValueError('Reviewed pose changed')
        pose=json.loads(Path(row['review']).read_text())
        capture=Path(row['capture'])/'capture.json'
        if helper.digest(capture) != pose['capture_sha256']:
            raise ValueError('Capture changed')
        rec=json.loads(capture.read_text())
        for name,sha in rec['raw_sha256'].items():
            if helper.digest(capture.parent/name)!=sha:raise ValueError('Raw evidence changed')
        if (not rec['complete'] or pose['candidate_sha256']!=protocol['camera_intrinsics_sha256'] or
            pose['fixed_board_epoch']!=protocol['fixed_board_epoch'] or pose['split']!=split):
            raise ValueError('Incompatible observation')
        rows.append(dict(entry=row,pose=pose))
    if not rows or len({r['pose']['origin_label'] for r in rows})!=1:
        raise ValueError('Missing views or inconsistent physical board origins')
    ee=[np.array(r['pose']['T_base_end_nominal']) for r in rows]
    cc=[np.array(r['pose']['poses']['sb']['T_camera_board']) for r in rows]
    return rows,ee,cc


def self_test():
    rng=np.random.default_rng(774)
    x=rigid(Rotation.from_rotvec([.3,-.2,.5]).as_matrix(),[.045,-.025,.065])
    y=rigid(Rotation.from_rotvec([.8,.1,-.4]).as_matrix(),[.5,.1,.2])
    ee=[rigid(Rotation.from_rotvec(rng.normal(0,.5,3)).as_matrix(),rng.normal(0,.15,3)) for _ in range(18)]
    cc=[np.linalg.inv(x)@np.linalg.inv(e)@y for e in ee]
    errors={}
    for method in METHODS:
        fitted=solve(ee[:13],cc[:13],method)
        errors[method]=float(np.max(np.abs(fitted-x)))
        assert errors[method]<1e-8,(method,errors[method])
        for e,c in zip(ee[13:],cc[13:]):assert np.max(np.abs(e@fitted@c-y))<1e-8
    return errors


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['fit','validate','self-test'])
    p.add_argument('--root',type=Path)
    p.add_argument('--candidate',type=Path)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if args.mode=='self-test':print(json.dumps(self_test(),indent=2));return
    if not args.root or not args.output or args.output.exists():
        raise ValueError('Root and fresh output file required')
    protocol=json.loads((args.root/'collection_protocol.json').read_text())
    if helper.digest(protocol['camera_intrinsics_file'])!=protocol['camera_intrinsics_sha256']:
        raise ValueError('Frozen intrinsics changed')
    limits=protocol['empirical_acceptance_rules']
    base=dict(schema='empirical-handeye-'+args.mode+'-v1',created_at=datetime.datetime.now().astimezone().isoformat(),
        robot_commands_sent=False,runtime_calibration_written=False,verified=False,
        independently_verified_joint_mapping=False,independent_angle_check_required=False,
        nominal_joint_mapping=protocol['nominal_joint_mapping'],fixed_board_epoch=protocol['fixed_board_epoch'],
        protocol_sha256=helper.digest(args.root/'collection_protocol.json'),
        intrinsic_candidate_sha256=protocol['camera_intrinsics_sha256'],script_sha256=helper.digest(__file__),
        transform_convention='T_end_camera maps camera coordinates in metres to nominal URDF Link6 coordinates.',
        reference='https://docs.opencv.org/4.6.0/d9/d0c/group__calib3d.html',
        limitations=['Empirical consistency is conditional on the declared nominal joint mapping.',
                     'Independent mechanical joint scale and zero accuracy are not established.',
                     'Known earlier relative-motion discrepancies are retained in the session evidence.',
                     'No automatic runtime installation or robot movement is performed.'])
    if args.mode=='fit':
        dataset=json.loads((args.root/'dataset.json').read_text())
        if any(r['split']=='validation' for r in dataset['poses']):
            raise ValueError('Freeze fit before collecting validation poses')
        rows,ee,cc=load_rows(args.root,'train',protocol)
        if len(rows)<protocol['training_goal']:raise ValueError('Training goal not reached')
        vectors=Rotation.from_matrix([ee[0][:3,:3].T@e[:3,:3] for e in ee[1:]]).as_rotvec()
        singular=np.linalg.svd(vectors,compute_uv=False)
        diversity_ok=(len(rows)>=limits['min_unique_training_poses'] and
                      singular[1]>=limits['min_training_rotation_second_singular_value_rad'])
        models={}
        for method in METHODS:
            try:
                x=solve(ee,cc,method); y=mean_transform([e@x@c for e,c in zip(ee,cc)])
                models[method]=dict(T_end_camera=x.tolist(),T_base_board=y.tolist(),
                    camera_offset_norm_m=float(np.linalg.norm(x[:3,3])),training=assess(ee,cc,x,y,limits))
            except (ValueError,cv2.error) as error:models[method]=dict(error=str(error))
        base.update(training_view_count=len(rows),training_views=[r['entry'] for r in rows],
            primary_method=protocol['primary_solver'].split()[-1],models=models,
            rotation_singular_values_rad=singular.tolist(),rotation_diversity_pass=bool(diversity_ok),
            validation_views_used_for_fitting_or_selection=0,independent_validation_complete=False)
        primary=models[base['primary_method']]
        base['training_empirical_acceptance_pass']=bool(diversity_ok and 'error' not in primary and
            primary['training']['empirical_residual_thresholds_pass'])
    else:
        if not args.candidate:raise ValueError('Frozen hand-eye candidate required')
        candidate=json.loads(args.candidate.read_text())
        freeze=json.loads((args.root/'handeye_freeze.json').read_text())
        if helper.digest(args.candidate)!=freeze['candidate_sha256']:
            raise ValueError('Hand-eye candidate changed after freezing')
        if candidate['protocol_sha256']!=base['protocol_sha256']:raise ValueError('Protocol changed')
        rows,ee,cc=load_rows(args.root,'validation',protocol)
        if len(rows)<protocol['independent_validation_goal']:raise ValueError('Validation goal not reached')
        training_paths={r['capture'] for r in candidate['training_views']}
        if any(r['entry']['capture'] in training_paths for r in rows):raise ValueError('Overlapping split')
        for row in candidate['training_views']:
            if helper.digest(row['review'])!=row['review_sha256']:raise ValueError('Frozen training evidence changed')
        models={}
        for method,model in candidate['models'].items():
            if 'error' in model:models[method]={'error':model['error']};continue
            models[method]=assess(ee,cc,np.array(model['T_end_camera']),np.array(model['T_base_board']),limits)
        primary=candidate['primary_method']
        base.update(candidate_file=str(args.candidate.resolve()),candidate_sha256=helper.digest(args.candidate),
            validation_views=[r['entry'] for r in rows],validation_view_count=len(rows),parameters_refitted=False,
            primary_method=primary,models=models,independent_validation_complete=True,
            empirical_validation_pass=bool(candidate['training_empirical_acceptance_pass'] and
                models[primary].get('empirical_residual_thresholds_pass',False)))
    live.save(args.output,base)
    if args.mode=='fit':
        freeze_path=args.root/'handeye_freeze.json'
        if freeze_path.exists():
            raise ValueError('A frozen candidate already exists; use a new session for a revised fit')
        live.save(freeze_path,dict(created_at=base['created_at'],candidate_file=str(args.output.resolve()),
                                  candidate_sha256=helper.digest(args.output),validation_views_collected=0))
    print(json.dumps({k:v for k,v in base.items() if k not in ['training_views','validation_views','limitations']},indent=2))


if __name__=='__main__':main()
