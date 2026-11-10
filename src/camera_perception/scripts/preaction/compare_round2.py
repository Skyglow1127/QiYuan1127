#!/usr/bin/env python3
"""Compare fixed old/new PREACTION heads on the exact same new-data tensors.

Offline inference only. No model selection, training, capture or robot APIs.
The randomized action control is a seeded within-split permutation constrained
to change the commanded joint/direction for every sample.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import torch

from core import PreAction, box_of, red_mask, future_box, box_iou, render_forecast


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def direction(row):
    delta = np.asarray(row['action']['planned_delta_sdk'][:6],dtype=np.float64)
    if delta.shape != (6,) or not np.isfinite(delta).all():
        raise ValueError('invalid_planned_delta')
    joint = int(np.argmax(np.abs(delta)))
    if abs(delta[joint]) < 1e-6:
        return 'hold'
    return 'j'+str(joint)+('+' if delta[joint] > 0 else '-')


def cross_direction_permutation(splits, directions, seed=42,selected_splits=('train','val','test')):
    """Random bipartite matching preserves action marginals and split ownership."""
    rng = np.random.default_rng(seed)
    assignment = np.arange(len(splits),dtype=np.int64)
    for split in selected_splits:
        indices = np.flatnonzero(np.asarray(splits) == split).tolist()
        if not indices:
            raise ValueError('missing_split:'+split)
        neighbors = {i:rng.permutation([j for j in indices if directions[j] != directions[i]]).tolist()
                     for i in indices}
        owner = {}

        def assign(i,seen):
            for donor in neighbors[i]:
                if donor in seen:
                    continue
                seen.add(donor)
                if donor not in owner or assign(owner[donor],seen):
                    owner[donor] = i
                    return True
            return False

        for i in rng.permutation(indices).tolist():
            if not assign(i,set()):
                raise ValueError('cannot_permute_every_sample_across_directions:'+split)
        for donor,recipient in owner.items():
            assignment[recipient] = donor
    if len(set(assignment.tolist())) != len(assignment):
        raise AssertionError('not_a_permutation')
    for recipient,donor in enumerate(assignment):
        if splits[recipient] not in selected_splits:continue
        if splits[recipient] != splits[donor] or directions[recipient] == directions[donor]:
            raise AssertionError('permutation_failed_split_or_direction_constraint')
    return assignment


def validate_regression_scope(pairs,old_pairs):
    """No new split reassignment; old holdout content is byte-value equivalent."""
    holdouts={row['id']:row for row in old_pairs if row['split'] in ('val','test')}
    selected=[row for row in pairs if row['split'] in ('val','test')]
    if {row['id'] for row in selected}!=set(holdouts):
        raise ValueError('regression_requires_all_original_holdouts')
    for row in selected:
        copy=dict(row)
        if copy.pop('collection_role',None)!='historical_holdout_regression' or copy!=holdouts[row['id']]:
            raise ValueError('historical_holdout_was_modified')
    for row in pairs:
        if row['split']=='train' and (row.get('collection_role')!='new_action_collection'
            or row.get('interaction_context',{}).get('object_held') is not False):
            raise ValueError('fallback_train_must_be_new_declared_unheld_samples')


def validate_dataset(run,data,pairs,config):
    if sha(run/'pairs.json') != config['pairs_sha256']:
        raise ValueError('pairs_hash_does_not_match_training')
    if data['id'].tolist() != [p['id'] for p in pairs] or data['split'].tolist() != [p['split'] for p in pairs]:
        raise ValueError('tensor_pair_order_mismatch')
    if list(data['x'].shape) != [len(pairs),*config['input_shape'][1:]]:
        raise ValueError('tensor_shape_mismatch')
    for split in ('train','val','test'):
        if sum(row['split'] == split for row in pairs) != config[split+'_pairs']:
            raise ValueError('training_split_count_mismatch')
    for i,row in enumerate(pairs):
        q = np.asarray(row['q_current'],np.float32)/np.asarray([180.]*6+[100.],np.float32)
        action = np.asarray(row['delta_q_command'],np.float32)/np.asarray([5.]*6+[20.],np.float32)
        planes = np.r_[q,action,np.float32(row['horizon_s']/2)].astype(np.float32)
        if not np.equal(data['x'][i,4:],planes[:,None,None]).all():
            raise ValueError('saved_input_state_action_horizon_mismatch:'+row['id'])
        for key,path_key in [('image_t','current_image'),('image_t1','future_image')]:
            if sha(row[path_key]) != row[key]['sha256']:
                raise ValueError('source_image_changed:'+row[path_key])


def aggregate(rows):
    keys = ('center_error_px','bbox_iou','foreground_rgb_mae','forecast_mask_iou')
    result = dict(n=len(rows),segments=len({row['segment_id'] for row in rows}))
    for key in keys:
        values = [row[key] for row in rows if row.get(key) is not None]
        if values:
            result[key] = float(np.mean(values))
    groups = sorted({row['segment_id'] for row in rows})
    if groups:
        result['segment_macro_center_error_px'] = float(np.mean([
            np.mean([row['center_error_px'] for row in rows if row['segment_id'] == group]) for group in groups]))
    return result


def compare(old_run,new_run,output,seed=42,aborted_collection_regression=False):
    old_run = Path(old_run).resolve();new_run = Path(new_run).resolve();output = Path(output).resolve()
    if output.exists():
        raise ValueError('comparison_output_must_be_new')
    old_config = json.loads((old_run/'train_config.json').read_text())
    new_config = json.loads((new_run/'train_config.json').read_text())
    if old_config['yolo_sha256'] != new_config['yolo_sha256'] or sha(new_config['yolo_weights']) != new_config['yolo_sha256']:
        raise ValueError('detector_must_be_fixed_to_previous_checkpoint')
    data = np.load(new_run/'dataset.npz',allow_pickle=False)
    manifest = json.loads((new_run/'pairs.json').read_text());pairs = manifest['pairs']
    validate_dataset(new_run,data,pairs,new_config)
    if any(row.get('collection_role') == 'historical_train_replay' for row in pairs):
        raise ValueError('round2_protocol_forbids_historical_replay')
    fallback_manifest=manifest.get('summary',{}).get('evaluation_mode')=='aborted_collection_historical_regression'
    if fallback_manifest!=aborted_collection_regression:
        raise ValueError('comparison_fallback_flag_must_match_explicit_frozen_protocol')
    if not aborted_collection_regression and any(row.get('interaction_context',{}).get('object_held') is not False for row in pairs):
        raise ValueError('round2_protocol_requires_declared_unheld_context')
    old_pairs_path = old_run/'pairs.json'
    if sha(old_pairs_path) != old_config['pairs_sha256']:
        raise ValueError('old_training_pairs_hash_mismatch')
    old_pairs = json.loads(old_pairs_path.read_text())['pairs']
    if aborted_collection_regression:validate_regression_scope(pairs,old_pairs)
    old_horizons = {float(row['horizon_s']) for row in old_pairs if row['split'] == 'train'}
    new_horizons = {float(row['horizon_s']) for row in pairs}
    if not new_horizons.issubset(old_horizons):
        raise ValueError('new_horizon_unsupported_by_old_checkpoint')
    old_hashes = {row[key]['sha256'] for row in old_pairs for key in ('image_t','image_t1')}
    new_rows=[row for row in pairs if not aborted_collection_regression or row['split']=='train']
    if any(row[key]['sha256'] in old_hashes for row in new_rows for key in ('image_t','image_t1')):
        raise ValueError('new_dataset_reuses_old_image')
    directions = [direction(row) for row in pairs]
    shuffle_splits=('train',) if aborted_collection_regression else ('train','val','test')
    permutation = cross_direction_permutation(data['split'],directions,seed,shuffle_splits)
    shuffle_available=np.isin(data['split'],shuffle_splits)
    x = data['x'];zero = x.copy();zero[:,11:18] = 0
    shuffled = x.copy();shuffled[:,11:18] = x[permutation,11:18]
    predictions = {}
    for name,run in [('old',old_run),('new',new_run)]:
        model = PreAction(pretrained=False)
        model.load_state_dict(torch.load(run/'best.pt',map_location='cpu',weights_only=True));model.eval()
        with torch.no_grad():
            for condition,inputs in [('correct_action',x),('zero_action',zero),('cross_direction_shuffled',shuffled)]:
                predictions[name+'_'+condition] = model(torch.from_numpy(inputs)).numpy().reshape(-1,4)
                if condition=='cross_direction_shuffled':
                    predictions[name+'_'+condition][~shuffle_available]=np.nan
    predictions['persistence'] = np.zeros((len(pairs),4),np.float32)
    hsv_boxes = [box_of(red_mask(cv2.imread(row['current_image']))) for row in pairs]
    all_records = {}
    for name,values in predictions.items():
        records = []
        for i,pred in enumerate(values):
            if not np.isfinite(pred).all():continue  # Explicitly unavailable control, never identity called a shuffle.
            predicted = future_box(data['current_box'][i],data['spec'][i],pred)
            target = data['future_box'][i]
            rgb = np.uint8(np.clip(x[i,:3].transpose(1,2,0)*255,0,255))
            forecast,mask = render_forecast(rgb,x[i,3],data['current_box'][i],data['spec'][i],pred)
            union = (x[i,3] > .5)|(data['future_mask'][i] > .5)
            error = np.abs(forecast.astype(np.float32)/255-data['future_rgb'][i].transpose(1,2,0))
            fbin = mask > .5;tbin = data['future_mask'][i] > .5
            true_motion = np.linalg.norm((target[:2]+target[2:]-hsv_boxes[i][:2]-hsv_boxes[i][2:])/2)
            records.append(dict(id=pairs[i]['id'],split=pairs[i]['split'],episode=pairs[i]['episode'],
                segment_id=pairs[i]['segment_id'],direction=directions[i],horizon_s=float(pairs[i]['horizon_s']),
                center_error_px=float(np.linalg.norm((predicted[:2]+predicted[2:]-target[:2]-target[2:])/2)),
                bbox_iou=box_iou(predicted,target),foreground_rgb_mae=float(error[union].mean()) if union.any() else None,
                forecast_mask_iou=float((fbin&tbin).sum()/max(1,(fbin|tbin).sum())),
                weak_reference_motion_px=float(true_motion),forecast_bbox=predicted.tolist(),motion_parameters_scaled=pred.tolist()))
        all_records[name] = records
    all_records['hsv_persistence'] = []
    for i,row in enumerate(pairs):
        target = data['future_box'][i];predicted = hsv_boxes[i]
        all_records['hsv_persistence'].append(dict(id=row['id'],split=row['split'],episode=row['episode'],
            segment_id=row['segment_id'],direction=directions[i],horizon_s=float(row['horizon_s']),
            center_error_px=float(np.linalg.norm((predicted[:2]+predicted[2:]-target[:2]-target[2:])/2)),
            bbox_iou=box_iou(predicted,target)))
    summary = {name:{split:aggregate([r for r in records if r['split'] == split]) for split in ('train','val','test')}
               for name,records in all_records.items()}
    by_direction = {name:{split:{d:aggregate([r for r in records if r['split'] == split and r['direction'] == d])
                    for d in sorted(set(directions))} for split in ('train','val','test')} for name,records in all_records.items()}
    comparisons = {}
    for split in ('train','val','test'):
        new = summary['new_correct_action'][split]['center_error_px']
        comparisons[split] = {name+'_minus_new_center_error_px':summary[name][split]['center_error_px']-new if 'center_error_px' in summary[name][split] else None
            for name in ('old_correct_action','persistence','hsv_persistence','new_zero_action','new_cross_direction_shuffled')}
        comparisons[split]['meaning'] = 'Positive means the new correct-action head has lower error; descriptive, no independence or significance claim.'
    test_comparison = comparisons['test']
    lower_than_controls = None if aborted_collection_regression else all(test_comparison[name+'_minus_new_center_error_px'] > 0
                             for name in ('persistence','hsv_persistence','new_zero_action','new_cross_direction_shuffled'))
    result = dict(schema='preaction-round2-fixed-input-comparison-v1',summary=summary,by_direction=by_direction,
        paired_comparisons=comparisons,records=all_records,
        outcome=dict(test_new_better_than_old=test_comparison['old_correct_action_minus_new_center_error_px'] > 0,
            test_new_lower_error_than_all_controls=lower_than_controls,
            scope='New training-set fit and previously observed old-holdout regression only; no new independent or blind test, and no new-action generalization claim.' if aborted_collection_regression else 'Descriptive evidence on one held-out repeat run in the same scene; empty-gripper viewpoint prediction only.'),
        permutation=dict(seed=seed,policy='Randomized exact permutation within each split; every donor has a different planned dominant joint/direction.',
            available_splits=list(shuffle_splits),unavailable_splits=['val','test'] if aborted_collection_regression else [],
            unavailable_reason='Historical single-direction holdouts cannot support cross-direction action permutation.' if aborted_collection_regression else None,
            direction_definition='Dominant absolute planned arm delta and its sign; return legs classified by actual planned delta, not anchor offset.',
            rows=[dict(recipient=pairs[i]['id'],donor=pairs[int(j)]['id'],split=pairs[i]['split'],
                recipient_direction=directions[i],donor_direction=directions[int(j)]) for i,j in enumerate(permutation) if shuffle_available[i]]),
        provenance=dict(old_run=str(old_run),new_run=str(new_run),old_weights_sha256=sha(old_run/'best.pt'),
            new_weights_sha256=sha(new_run/'best.pt'),old_train_config_sha256=sha(old_run/'train_config.json'),
            new_train_config_sha256=sha(new_run/'train_config.json'),dataset_sha256=sha(new_run/'dataset.npz'),
            pairs_sha256=sha(new_run/'pairs.json'),fixed_detector_sha256=new_config['yolo_sha256'],
            input_tensor_sha256=hashlib.sha256(x.tobytes()).hexdigest(),comparison_source_sha256=sha(__file__)),
        protocol=dict(shared_preprocessing='Both heads use identical saved new-data input tensor, current ROI, weak future labels and frozen prior-round YOLO.',
            evaluation_mode=manifest.get('summary',{}).get('evaluation_mode','new_collection_whole_run_holdouts'),
            protocol_amendment=manifest.get('protocol_amendment'),
            train_seed=new_config['seed'],steps_requested=new_config['steps_requested'],steps_run=new_config['steps_run'],
            checkpoint_selection='Already completed validation-only selection; this comparison never retrains or selects checkpoints.',
            horizons_s=sorted(new_horizons),historical_replay=False,
            new_action_semantics=sorted({row['action'].get('action_semantics','legacy_full_arm_plan_with_gripper_hold') for row in pairs}),
            old_action_semantics=sorted({row['action'].get('action_semantics','legacy_full_arm_plan_with_gripper_hold') for row in old_pairs}),
            limitations=['Future labels are HSV weak masks, not independently measured truth.',
                'Several windows overlap within each physical segment; pair count is not independent trial count.',
                'Cross-direction action replacement can be inconsistent with the current state; it is a sensitivity control, not a physically executed counterfactual.',
                'For partial commands, inactive delta zero means no update, not an observed hold. The model has no explicit command-mask input; transfer from older full-arm commands includes this action-contract change.',
                'No grasp/contact/action-execution skill is evaluated and no robot policy is promoted.']),
        training_executed=False,hardware_commands_sent=0,automatic_promotion=False)
    output.mkdir(parents=True,exist_ok=False)
    (output/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    np.savez_compressed(output/'predictions.npz',**predictions,action_donor_index=permutation,
                        cross_direction_available=shuffle_available,id=data['id'],split=data['split'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old-run',type=Path,required=True)
    parser.add_argument('--new-run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--shuffle-seed',type=int,default=42)
    parser.add_argument('--aborted-collection-regression',action='store_true',help='Match frozen fallback: old val/test regression, cross-direction shuffle only on new train.')
    args = parser.parse_args();torch.set_num_threads(4);cv2.setNumThreads(1)
    result = compare(args.old_run,args.new_run,args.output,args.shuffle_seed,args.aborted_collection_regression)
    print(json.dumps(dict(status='FAIR_OFFLINE_COMPARISON_COMPLETE',output=str(args.output.resolve()),
        test_summary={name:splits['test'] for name,splits in result['summary'].items()},outcome=result['outcome']),ensure_ascii=False))


if __name__ == '__main__':
    main()
