#!/usr/bin/env python3
"""Training-only discrete sign selection and frozen held-out evaluation.

Does not change robot configuration, command hardware, or fit angle scale/zero.
"""
import argparse
import datetime
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import fit_empirical_handeye as fit
import kr260_handeye as helper
from review_joint_mapping import fk


def poses(rows, signs):
    return [fk(np.deg2rad(r['pose']['joint_median'][:6])*signs) for r in rows]


def score(metrics, limits):
    return ((metrics['rotation_rms_deg']/limits['training_and_heldout_rotation_rms_deg'])**2 +
            (metrics['translation_rms_m']/limits['training_and_heldout_translation_rms_m'])**2)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['freeze','validate'])
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--candidate',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('Fresh output required')
    root=args.root;protocol=json.loads((root/'collection_protocol.json').read_text())
    if helper.digest(protocol['camera_intrinsics_file'])!=protocol['camera_intrinsics_sha256']:
        raise ValueError('Frozen intrinsics changed')
    limits=protocol['empirical_acceptance_rules']
    train,_,cc=fit.load_rows(root,'train',protocol)
    result=dict(schema='empirical-handeye-sign-'+args.mode+'-v1',
        created_at=datetime.datetime.now().astimezone().isoformat(),
        protocol_sha256=helper.digest(root/'collection_protocol.json'),
        script_sha256=helper.digest(__file__),
        robot_commands_sent=False,runtime_calibration_written=False,
        independently_verified_joint_mapping=False,verified=False,
        scales=[1]*6,zero_offsets_deg=[0]*6,solver='PARK',
        transform_convention='T_end_camera maps camera optical coordinates in metres to URDF Link6 using the associated sign hypothesis.',
        limitations=['Direction combinations are empirical hypotheses, not independent mechanical validation.',
                     'J1 and J2 are nearly fixed in training; their directions and absolute zeros are not identified.',
                     'Evaluation is local to the sampled pose and image region.'])
    if args.mode=='freeze':
        if len(train)<protocol['training_goal']:raise ValueError('Insufficient training views')
        dataset=json.loads((root/'dataset.json').read_text())
        if any(r['split']=='validation' for r in dataset['poses']):raise ValueError('Validation already collected')
        models=[]
        for signs in itertools.product((-1,1),repeat=6):
            e=poses(train,np.array(signs))
            model=dict(signs=list(signs))
            try:
                x=fit.solve(e,cc,'PARK');y=fit.mean_transform([a@x@b for a,b in zip(e,cc)])
                metrics=fit.assess(e,cc,x,y,limits)
                model.update(T_end_camera=x.tolist(),T_base_board=y.tolist(),training=metrics,
                    training_score=score(metrics,limits),camera_offset_norm_m=float(np.linalg.norm(x[:3,3])))
            except (ValueError,fit.cv2.error) as error:model['error']=str(error)
            models.append(model)
        valid=sorted([m for m in models if 'error' not in m],key=lambda m:m['training_score'])
        if not valid:raise ValueError('No proper rigid hypothesis')
        preferred=valid[0];signs=np.array(preferred['signs']);ee=poses(train,signs)
        sv=np.linalg.svd(Rotation.from_matrix([ee[0][:3,:3].T@e[:3,:3] for e in ee[1:]]).as_rotvec(),compute_uv=False)
        # Leave-one-out perturbations are a training-only stability diagnostic.
        loo=[];fixed_x=np.array(preferred['T_end_camera'])
        for i in range(len(train)):
            ix=[j for j in range(len(train)) if j!=i]
            x=fit.solve([ee[j] for j in ix],[cc[j] for j in ix],'PARK')
            y=fit.mean_transform([ee[j]@x@cc[j] for j in ix])
            loo.append(dict(omitted=i,heldout=fit.assess([ee[i]],[cc[i]],x,y,limits),
                transform_translation_delta_m=float(np.linalg.norm(x[:3,3]-fixed_x[:3,3])),
                transform_rotation_delta_deg=float(np.rad2deg(Rotation.from_matrix(fixed_x[:3,:3].T@x[:3,:3]).magnitude()))))
        result.update(training_views=[r['entry'] for r in train],models=models,
            preferred_signs=preferred['signs'],preferred_selected_before_validation=True,
            selection_rule='Minimize training (rotation_RMS/1deg)^2 + (translation_RMS/10mm)^2 over all64 sign combinations. No scale or zero fitting; no validation use.',
            validation_views_used=0,rotation_singular_values_rad=sv.tolist(),
            rotation_diversity_pass=bool(sv[1]>=limits['min_training_rotation_second_singular_value_rad']),
            preferred_training=preferred['training'],preferred_leave_one_out=loo)
        result['preferred_training_acceptance_pass']=bool(result['rotation_diversity_pass'] and preferred['training']['empirical_residual_thresholds_pass'])
    else:
        if not args.candidate:raise ValueError('Frozen candidate required')
        freeze=json.loads((root/'sign_hypothesis_freeze.json').read_text())
        if helper.digest(args.candidate)!=freeze['candidate_sha256']:raise ValueError('Frozen candidate changed')
        candidate=json.loads(args.candidate.read_text())
        if candidate['protocol_sha256']!=result['protocol_sha256']:raise ValueError('Protocol changed')
        if candidate['training_views']!=[r['entry'] for r in train]:raise ValueError('Training changed')
        rows,_,vc=fit.load_rows(root,'validation',protocol)
        if len(rows)<protocol['independent_validation_goal']:raise ValueError('Insufficient validation views')
        if {r['pose']['origin_label'] for r in rows}!={r['pose']['origin_label'] for r in train}:raise ValueError('Board origin mismatch')
        models=[]
        for model in candidate['models']:
            if 'error' in model:continue
            e=poses(rows,np.array(model['signs']))
            models.append(dict(signs=model['signs'],validation=fit.assess(e,vc,np.array(model['T_end_camera']),np.array(model['T_base_board']),limits)))
        preferred=next(m for m in models if m['signs']==candidate['preferred_signs'])
        result.update(candidate_file=str(args.candidate.resolve()),candidate_sha256=helper.digest(args.candidate),
            validation_views=[r['entry'] for r in rows],validation_view_count=len(rows),models=models,
            preferred_signs=candidate['preferred_signs'],preferred_validation=preferred['validation'],
            parameters_refitted=False,preferred_changed_using_validation=False,
            empirical_validation_pass=bool(candidate['preferred_training_acceptance_pass'] and preferred['validation']['empirical_residual_thresholds_pass']))
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    if args.mode=='freeze':
        path=root/'sign_hypothesis_freeze.json'
        with path.open('x') as stream:json.dump(dict(candidate_file=str(args.output.resolve()),candidate_sha256=helper.digest(args.output),created_at=result['created_at'],validation_views_collected=0),stream,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ['models','training_views','validation_views','preferred_leave_one_out','limitations']},indent=2))


if __name__=='__main__':main()
