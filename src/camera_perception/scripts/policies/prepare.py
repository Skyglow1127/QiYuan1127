"""Export real, causally aligned *stage fragments* for ACT. No hardware access."""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np
from .contracts import validate_sample, CAMERAS

EXPORTER=Path('/home/aup/D1/Control/tools/export_grasp_dataset.py')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path): return json.loads(Path(path).read_text())


def chunks(rows, k=10, rate=5, tolerance=.05):
    """Break at every invalid row, gap, or segment boundary. Never bridge faults."""
    groups=[]; current=[]
    for row in rows:
        if row is None:
            if current: groups.append(current)
            current=[];continue
        if current and (row['segment_id']!=current[-1]['segment_id'] or
            abs(row['command_host_monotonic']-current[-1]['command_host_monotonic']-1/rate)>tolerance or
            row['sample_index']!=current[-1]['sample_index']+1):
            groups.append(current);current=[]
        current.append(row)
    if current:groups.append(current)
    out=[]
    for group in groups:
        for i,row in enumerate(group):
            target=group[i:i+k]
            if len(target)<2:continue
            out.append(dict(id=f"{row['segment_id']}:{row['sample_index']}",
                segment_id=row['segment_id'],sample_index=row['sample_index'],
                command_host_monotonic=row['command_host_monotonic'],
                q=row['observation']['q'],cameras=row['observation']['cameras'],
                actions=[x['action']['sdk_target'] for x in target]+[[0.]*7]*(k-len(target)),
                is_pad=[False]*len(target)+[True]*(k-len(target)),
                action_source_lines=[x['action']['source_line'] for x in target],
                actual_target_times=[x['command_host_monotonic'] for x in target],
                action_source_log=row['action']['source_log']))
    return out


def prepare(config, output):
    config=Path(config).resolve(); cfg=read(config);out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('causal_export',EXPORTER);exp=importlib.util.module_from_spec(spec);spec.loader.exec_module(exp)
    sources={str(config):sha(config),str(EXPORTER):sha(EXPORTER)}; image_hashes={}; counts=Counter();all_examples=[]; reports=[]
    root_splits={}; observed_images={}
    def verify(path, expected):
        if path not in image_hashes:image_hashes[path]=sha(path)
        if image_hashes[path]!=expected:raise ValueError('image_hash_mismatch')
    (out/'causal').mkdir()
    for i,stage in enumerate(cfg['stages']):
        run=Path(stage['run']).resolve();folder=Path(stage['path']).resolve();split=stage['split']
        if split not in ('train','val','test'):raise ValueError('unknown_split')
        group=stage.get('episode_group',str(run))
        if group in root_splits and root_splits[group]!=split:raise ValueError('episode_group_split_leakage')
        root_splits[group]=split
        context=exp.fsm_context(run,sources) if stage.get('fsm') else None
        project=run/'project_status.json'
        segment=exp.export_stage(folder,out/'causal',run,read(project) if project.exists() else {},project,sources,
            semantic_stage=stage['stage'],segment_name=f'{i:02d}_{stage["stage"]}',fsm=context)
        if project.exists():sources[str(project)]=sha(project)
        rows=[];reasons=Counter()
        for line in Path(segment['samples_file']).read_text().splitlines():
            row=json.loads(line)
            try:validate_sample(row,verify)
            except (ValueError,KeyError,TypeError) as e:
                reasons[str(e)]+=1;rows.append(None);continue
            rows.append(row)
        examples=chunks(rows,cfg['chunk_size'],cfg['rate_hz'])
        for row in examples:
            row.update(split=split,episode_group=group,stage=stage['stage'])
            for c in row['cameras'].values():
                key=c['sha256'];old=observed_images.get(key)
                if old is not None and old!=split:raise ValueError('identical_image_cross_split')
                observed_images[key]=split
        counts[split]+=len(examples);all_examples+=examples
        reports.append(dict(source=stage,export=segment,act_examples=len(examples),rejections=dict(reasons)))
    if not counts['train'] or not counts['val']:raise ValueError('train_and_distinct_group_validation_required')
    q=np.array([r['q'] for r in all_examples if r['split']=='train'],dtype=np.float32)
    # Every observed training command is counted once; padded actions and validation are excluded.
    unique={}
    for r in all_examples:
        if r['split']!='train':continue
        for j,line in enumerate(r['action_source_lines']):unique[(r['action_source_log'],line)]=r['actions'][j]
    a=np.array(list(unique.values()),dtype=np.float32)
    stats=dict(q_mean=q.mean(0).tolist(),q_std=np.maximum(q.std(0),.1).tolist(),
        action_mean=a.mean(0).tolist(),action_std=np.maximum(a.std(0),.1).tolist(),
        split_used='train_only',unique_train_commands=len(a))
    examples_path=out/'examples.jsonl';examples_path.write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in all_examples))
    manifest=dict(schema='d1-act-stage-dataset-v1',config=cfg,counts=dict(counts),stages=reports,stats=stats,
        examples_file=str(examples_path),examples_sha256=sha(examples_path),source_sha256=sources,
        image_sha256=image_hashes,complete_grasp_episodes=0,scope='causal successful stage fragments only',
        hardware_accessed=False,robot_commands=0,independent_scene_test=False,
        split_groups=root_splits,image_order=list(CAMERAS))
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();m=prepare(a.config,a.output);print(json.dumps({'counts':m['counts'],'stages':[(s['source']['stage'],s['act_examples']) for s in m['stages']],'complete_grasp_episodes':0}))
if __name__=='__main__':main()
