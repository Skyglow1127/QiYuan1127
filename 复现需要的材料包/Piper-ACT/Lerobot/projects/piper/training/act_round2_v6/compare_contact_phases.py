#!/usr/bin/env python3
"""Read-only parent/final ACT comparison on identical recorded contact phases.

All52 episodes were seen during training: reconstruction checks cannot establish
generalization, diagnose overfitting by themselves or prove physical success.
"""
import json
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'logs/act_grasp_release_fix_v7'
DATA = Path('/home/aup/Lerobot/datasets/rad_place_merged_v5')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    for key,value in {'HF_HUB_OFFLINE':'1','HF_DATASETS_OFFLINE':'1',
        'HF_HUB_DISABLE_TELEMETRY':'1','HF_DATASETS_CACHE':str(OUT/'cache'),
        'OMP_NUM_THREADS':'8','MKL_NUM_THREADS':'8','OPENBLAS_NUM_THREADS':'1'}.items():
        os.environ[key] = value
    import logging
    logging.getLogger('lerobot.utils.import_utils').setLevel(logging.ERROR)
    import numpy as np
    import pyarrow.dataset as pads
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies import make_pre_post_processors

    torch.set_num_threads(8)
    torch.set_num_interop_threads(1)
    episodes = [0,8,26,41,51]
    rows = pads.dataset(DATA/'data',format='parquet').to_table(columns=[
        'index','episode_index','frame_index','timestamp','observation.state','action'],
        filter=pads.field('episode_index').isin(episodes)).to_pylist()
    records = {ep:sorted([r for r in rows if r['episode_index']==ep],key=lambda r:r['index']) for ep in episodes}
    selections = []
    for ep,rs in records.items():
        widths = np.array([r['action'][6] for r in rs])
        close = [i for i in range(1,len(rs)) if widths[i]<.04 and widths[i-1]>=.04]
        release = [i for i in range(10,len(rs)) if widths[i]>.055 and widths[i-1]<=.055
            and sum(widths[max(0,i-100):i]<.04)>=10]
        if not close or not release:
            raise ValueError('Contact events missing:'+str(ep))
        for phase,event,offsets in (('close',close[0],(-10,0,10)),
                                   ('release',release[-1],(-10,0,20,60))):
            for offset in offsets:
                index = max(0,min(event+offset,len(rs)-1))
                selections.append((ep,index,phase,offset))
    dataset = LeRobotDataset(repo_id='local/'+DATA.name,root=DATA,
        episodes=episodes,video_backend='pyav',return_uint8=False)
    index_map = {int(index):position for position,index in enumerate(dataset.hf_dataset.data.column('index').to_pylist())}
    # Cache the exact same observations, not future actions, for both models.
    samples = []
    for ep,index,phase,offset in selections:
        row = records[ep][index]
        sample = dataset[index_map[int(row['index'])]]
        samples.append({'ep':ep,'index':index,'phase':phase,'offset':offset,
            'sample':{key:sample[key].clone() for key in ('observation.state',
                'observation.images.global','observation.images.wrist')},
            'target':np.array([r['action'] for r in records[ep][index:index+100]])})
    old = json.loads((ROOT/'deployment/act_launch_defaults.json').read_text())['checkpoint']
    new = str(ROOT/'training/act_round2_v6/result/pretrained_model')
    output = {'data_seen_during_training':True,'hardware_commands_sent':0,'dataset_changed':False,
        'comparison':'identical recorded observations; reconstruction, not held-out generalization',
        'samples_per_model':len(samples),'models':{}}
    for label,path in (('parent',old),('round2_final',new)):
        config = PreTrainedConfig.from_pretrained(path,local_files_only=True,
            cli_overrides=['--device=cpu','--use_amp=false','--pretrained_backbone_weights=null'])
        config.pretrained_path = Path(path)
        policy = ACTPolicy.from_pretrained(path,config=config,local_files_only=True,strict=True)
        policy.eval().requires_grad_(False)
        pre,post = make_pre_post_processors(policy_cfg=config,pretrained_path=path,
            preprocessor_overrides={'device_processor':{'device':'cpu'},'normalizer_processor':{'device':'cpu'}},
            postprocessor_overrides={'device_processor':{'device':'cpu'},'unnormalizer_processor':{'device':'cpu'}})
        results = []
        for entry in samples:
            policy.reset();pre.reset();post.reset()
            start = time.monotonic()
            with torch.inference_mode():
                prediction = post(policy.predict_action_chunk(pre(entry['sample']))).detach().cpu().numpy()[0]
            target = entry['target'];n = min(20,len(target))
            results.append({'ep':entry['ep'],'frame':entry['index'],'phase':entry['phase'],'offset_frames':entry['offset'],
                'first20_joint_mae_deg':float(np.mean(np.abs(prediction[:n,:6]-target[:n,:6]))),
                'first20_gripper_mae_mm':float(np.mean(np.abs(prediction[:n,6]-target[:n,6]))*1000),
                'first_predicted_gripper_mm':float(prediction[0,6]*1000),
                'first_demo_gripper_mm':float(target[0,6]*1000),
                'first20_predicted_open_count':int(sum(prediction[:n,6]>.055)),
                'first20_demo_open_count':int(sum(target[:n,6]>.055)),
                'prediction_ms':(time.monotonic()-start)*1000,
                'prediction_first20':prediction[:n].tolist(),'demo_first20':target[:n].tolist()})
        summary = {}
        for phase in ('close','release'):
            subset = [r for r in results if r['phase']==phase]
            summary[phase] = {'joint_mae_deg':float(np.mean([r['first20_joint_mae_deg'] for r in subset])),
                'gripper_mae_mm':float(np.mean([r['first20_gripper_mae_mm'] for r in subset])),
                'gripper_open_count_mismatch':sum(abs(r['first20_predicted_open_count']-r['first20_demo_open_count']) for r in subset)}
        output['models'][label] = {'checkpoint':path,'summary':summary,'samples':results}
        (OUT/'model_comparison.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({'model':label,'summary':summary},ensure_ascii=False),flush=True)
        del policy,pre,post
    print('Comparison saved:'+str(OUT/'model_comparison.json'),flush=True)


if __name__ == '__main__':
    main()
