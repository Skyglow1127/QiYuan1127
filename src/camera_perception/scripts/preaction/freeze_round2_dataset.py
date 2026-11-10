#!/usr/bin/env python3
"""Freeze real new-run pairs with whole-run splits and optional old-train replay.

Offline only: does not capture images, train a model, or call robot interfaces.
Capture log mtimes provide a collection guard, not image/exposure synchronization.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

SPLITS = ('train', 'val', 'test')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def validate_rows(rows, check_files=True):
    """Return run ownership and image hashes; never split one run across sets."""
    run_splits = {}; hashes = {}; ids = set()
    for row in rows:
        split = row['split']; episode = row['episode']
        if split not in SPLITS or not episode:
            raise ValueError('invalid_split_or_missing_episode')
        if episode in run_splits and run_splits[episode] != split:
            raise ValueError('one_physical_run_crosses_splits:'+episode)
        run_splits[episode] = split
        if row['id'] in ids:
            raise ValueError('duplicate_pair_id:'+row['id'])
        ids.add(row['id'])
        horizon = row['horizon_s']
        if not isinstance(horizon, (int, float)) or not math.isfinite(horizon) or horizon <= 0:
            raise ValueError('invalid_horizon')
        if row['image_t']['sha256'] == row['image_t1']['sha256']:
            raise ValueError('future_image_is_current_image')
        for field, path_field in [('image_t','current_image'),('image_t1','future_image')]:
            digest = row[field]['sha256']
            if digest in hashes:
                raise ValueError('reused_image_hash:'+digest)
            hashes[digest] = split
            if check_files and sha(row[path_field]) != digest:
                raise ValueError('source_image_hash_changed:'+row[path_field])
    return run_splits, hashes


def merge_replay(fresh, replay=None, check_files=True):
    """Old validation/test never become training data, including hash aliases."""
    fresh_rows = deepcopy(fresh['pairs'])
    fresh_runs, fresh_hashes = validate_rows(fresh_rows, check_files)
    counts = Counter(row['split'] for row in fresh_rows)
    if not all(counts[s] for s in SPLITS):
        raise ValueError('new_runs_must_supply_train_val_and_test')
    horizons = {float(row['horizon_s']) for row in fresh_rows}
    if len(horizons) != 1:
        raise ValueError('single_fresh_horizon_required')
    for row in fresh_rows:
        row['collection_role'] = 'new_action_collection'
    old_rows = [] if replay is None else replay['pairs']
    old_hashes = {row[key]['sha256'] for row in old_rows for key in ('image_t','image_t1')}
    if set(fresh_hashes) & old_hashes:
        raise ValueError('new_capture_reuses_historical_image')
    historical_holdout_hashes = {row[key]['sha256'] for row in old_rows if row['split'] != 'train'
                                for key in ('image_t','image_t1')}
    used = set(fresh_hashes); rows = fresh_rows; omitted = Counter()
    for source in old_rows:
        if source['split'] != 'train':
            omitted['historical_validation_or_test'] += 1; continue
        digests = {source[key]['sha256'] for key in ('image_t','image_t1')}
        if digests & historical_holdout_hashes:
            raise ValueError('historical_train_contains_holdout_image')
        if float(source['horizon_s']) not in horizons:
            omitted['different_horizon'] += 1; continue
        if source['episode'] in fresh_runs:
            raise ValueError('historical_replay_shares_new_run')
        if digests & used:
            omitted['duplicate_replay_image'] += 1; continue
        row = deepcopy(source); row['id'] = 'historical_replay__'+row['id']
        row['collection_role'] = 'historical_train_replay'
        rows.append(row); used.update(digests)
    validate_rows(rows, check_files)
    return rows, dict(omitted)


def merge_aborted_regression(fresh,historical,expected_segments,check_files=True):
    """Keep all completed new data as train; preserve old val/test verbatim."""
    new=deepcopy(fresh['pairs']);validate_rows(new,check_files)
    if not new or any(row['split']!='train' for row in new):
        raise ValueError('aborted_collection_new_rows_must_all_remain_train')
    segments={row['segment_id'] for row in new}
    if expected_segments is None or len(segments)!=expected_segments:
        raise ValueError('unexpected_number_of_completed_new_segments')
    old=historical['pairs'];validate_rows(old,check_files)
    old_hashes={row[key]['sha256'] for row in old for key in ('image_t','image_t1')}
    if any(row[key]['sha256'] in old_hashes for row in new for key in ('image_t','image_t1')):
        raise ValueError('new_capture_reuses_historical_image')
    for row in new:
        row['collection_role']='new_action_collection'
        if check_files:
            result_path=Path(row['evidence']['physical_command_result'])
            result=read(result_path)
            if sha(result_path)!=row['evidence']['result_sha256']:
                raise ValueError('new_segment_result_hash_changed')
            if result.get('completed') is not True or result.get('hardware_commands_sent') is not True:
                raise ValueError('incomplete_new_segment_cannot_enter_regression_fallback')
    holdouts=[deepcopy(row) for row in old if row['split'] in ('val','test')]
    if not all(any(row['split']==split for row in holdouts) for split in ('val','test')):
        raise ValueError('historical_validation_and_test_required')
    for row in holdouts:row['collection_role']='historical_holdout_regression'
    rows=new+holdouts;validate_rows(rows,check_files)
    if len({float(row['horizon_s']) for row in rows})!=1:
        raise ValueError('single_matching_horizon_required')
    return rows,dict(historical_train_excluded=sum(row['split']=='train' for row in old))


def freeze(fresh_path, output, capture_after_unix, replay_path=None,*,aborted_collection_regression=False,
           historical_holdouts=None,expected_new_segments=None,original_protocol=None,abort_record=None,amendment_output=None):
    if not math.isfinite(capture_after_unix) or capture_after_unix <= 0:
        raise ValueError('positive_collection_start_unix_required')
    fresh_path = Path(fresh_path).resolve(); fresh = read(fresh_path)
    replay_path = Path(replay_path).resolve() if replay_path else None
    replay = read(replay_path) if replay_path else None
    amendment=None
    if aborted_collection_regression:
        if replay_path or not all([historical_holdouts,original_protocol,abort_record,amendment_output]):
            raise ValueError('fallback_requires_historical_holdouts_original_protocol_abort_record_amendment_and_no_replay')
        historical_holdouts=Path(historical_holdouts).resolve();original_protocol=Path(original_protocol).resolve()
        abort_record=Path(abort_record).resolve();amendment_output=Path(amendment_output).resolve()
        if amendment_output.exists():raise ValueError('amendment_output_must_be_new')
        rows,omitted=merge_aborted_regression(fresh,read(historical_holdouts),expected_new_segments)
        if read(abort_record).get('completed') is not False:
            raise ValueError('abort_record_must_record_incomplete_execution')
        amendment=dict(schema='preaction-collection-abort-protocol-amendment-v1',
            original_protocol=dict(path=str(original_protocol),sha256=sha(original_protocol)),
            abort_record=dict(path=str(abort_record),sha256=sha(abort_record),reported_error=read(abort_record).get('error')),
            reason='Collection stopped after joint-feedback limit anomaly; root cause unresolved. No further motion authorized by this offline amendment.',
            new_protocol='Only completed new segments train the predictor. Historical validation/test preserve original IDs, images and split; historical train is excluded.',
            evaluation_scope='New training-set fit and old-holdout regression only; no new independent or blind test and no new-action generalization claim.',
            original_protocol_preserved=True,fresh_all_splits_gate_satisfied=False,
            training=dict(seed=42,max_steps=600,runs=1,selection='historical validation loss only; historical test does not select checkpoint'),
            action_control_scope='Cross-direction shuffle is available for new train only; historical single-direction val/test do not support it.',
            hardware_commands_sent=0,training_executed=False,automatic_promotion=False)
    else:
        if any([historical_holdouts,expected_new_segments,original_protocol,abort_record,amendment_output]):
            raise ValueError('fallback_arguments_require_explicit_aborted_collection_regression_flag')
        rows, omitted = merge_replay(fresh, replay)
    capture_sources = {}
    for row in fresh['pairs']:
        source = Path(row['image_t']['source_log']).resolve()
        if source.stat().st_mtime < capture_after_unix:
            raise ValueError('capture_log_predates_collection_start:'+str(source))
        capture_sources[str(source)] = dict(sha256=sha(source),mtime_unix=source.stat().st_mtime)
        if row['action'].get('future_feedback_used') is not False or row['action'].get('future_logged_commands_used_as_features') is not False:
            raise ValueError('causal_planned_action_contract_missing')
        plan = row['action']['plan_source']
        if sha(plan) != row['action']['plan_sha256']:
            raise ValueError('preexecution_plan_hash_changed:'+plan)
    counts = Counter(row['split'] for row in rows)
    manifest = dict(schema=fresh.get('schema','causal-preaction-image-pairs-v1'),pairs=rows,
        summary=dict(pair_count=len(rows),split_counts=dict(counts),
            new_collection_pair_count=len(fresh['pairs']),historical_train_replay_count=0 if aborted_collection_regression else len(rows)-len(fresh['pairs']),
            historical_holdout_regression_count=len(rows)-len(fresh['pairs']) if aborted_collection_regression else 0,
            evaluation_mode='aborted_collection_historical_regression' if aborted_collection_regression else 'new_collection_whole_run_holdouts',
            new_collection_run_count=len({row['episode'] for row in fresh['pairs']}),
            unique_images=len(rows)*2,synthetic_pair_count=0,
            horizon_s=float(rows[0]['horizon_s']),hash_overlap_between_splits=0,
            split_policy=amendment['new_protocol'] if amendment else 'Complete new physical collection runs have one split each. Historical train only may be replayed; historical val/test excluded.',
            sampling_limitation='Several pairs from one short action segment overlap in time; pair count is not independent-episode count.',
            hardware_commands_sent=0,training_performed=False,automatic_promotion=False),
        provenance=dict(fresh_manifest=dict(path=str(fresh_path),sha256=sha(fresh_path)),
            replay_manifest=dict(path=str(replay_path),sha256=sha(replay_path)) if replay_path else None,
            capture_after_unix=capture_after_unix,capture_sources=capture_sources,replay_omitted=omitted,
            freshness_scope='Original capture-log mtime guard only; causal alignment comes from source exporter, not file timestamps.'),
        causal_contract=fresh.get('causal_contract',{}),stage_reports=fresh.get('stage_reports',[]))
    if amendment:
        manifest['provenance']['historical_holdouts']=dict(path=str(historical_holdouts),sha256=sha(historical_holdouts))
        manifest['protocol_amendment']=amendment
    output = Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    save(output/'pairs.json',manifest)
    save(output/'freeze_receipt.json',dict(pairs_sha256=sha(output/'pairs.json'),
        training_executed=False,hardware_commands_sent=0,automatic_promotion=False))
    if amendment:
        amendment['frozen_manifest']=dict(path=str(output/'pairs.json'),sha256=sha(output/'pairs.json'))
        with amendment_output.open('x') as stream:
            json.dump(amendment,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fresh-pairs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--capture-after-unix',type=float,required=True)
    parser.add_argument('--replay-pairs',type=Path,help='Optional previous manifest; only matching-horizon train pairs are eligible.')
    parser.add_argument('--aborted-collection-regression',action='store_true',help='Explicit fallback: new train plus unchanged historical val/test; no new blind evaluation.')
    parser.add_argument('--historical-holdouts',type=Path)
    parser.add_argument('--expected-new-segments',type=int)
    parser.add_argument('--original-protocol',type=Path)
    parser.add_argument('--abort-record',type=Path)
    parser.add_argument('--amendment-output',type=Path)
    args = parser.parse_args()
    manifest = freeze(args.fresh_pairs,args.output,args.capture_after_unix,args.replay_pairs,
        aborted_collection_regression=args.aborted_collection_regression,historical_holdouts=args.historical_holdouts,
        expected_new_segments=args.expected_new_segments,original_protocol=args.original_protocol,
        abort_record=args.abort_record,amendment_output=args.amendment_output)
    print(json.dumps(dict(status='OFFLINE_DATASET_FROZEN',output=str(args.output.resolve()),summary=manifest['summary']),ensure_ascii=False))


if __name__ == '__main__':
    main()
