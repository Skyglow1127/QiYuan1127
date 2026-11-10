"""Frozen protocol for future-ROI forecasts, persistence, and action ablations."""
from pathlib import Path
import argparse,json,math,time
import cv2,numpy as np,torch
from PIL import Image,ImageDraw
from core import PreAction,SIZE,PARAM_SCALE,red_mask,box_of,box_iou,future_box,render_forecast,sha

def evaluation_metadata(out,data,pairs,config):
    """Describe only the hash-bound rows actually supplied to evaluation."""
    if not config.get('pairs_sha256') or sha(out/'pairs.json')!=config['pairs_sha256']:
        raise ValueError('Training pairs metadata missing or SHA256 mismatch')
    if (data['id'].tolist()!=[row['id'] for row in pairs] or
        data['split'].tolist()!=[row['split'] for row in pairs]):
        raise ValueError('Dataset row IDs/splits do not match training pairs in order')
    counts={split:sum(row['split']==split for row in pairs) for split in ('train','val','test')}
    if sum(counts.values())!=len(pairs) or not counts['train']:
        raise ValueError('Expected train/val/test splits and a nonempty training split')
    if any(count!=config[split+'_pairs'] for split,count in counts.items()):
        raise ValueError('Dataset split counts do not match training configuration')
    horizons=[row['horizon_s'] for row in pairs]
    if any(isinstance(h,bool) or not isinstance(h,(int,float)) or not math.isfinite(h) or h<=0 for h in horizons):
        raise ValueError('Training metadata requires finite positive numeric horizons')
    trained=sorted({float(row['horizon_s']) for row in pairs if row['split']=='train'})
    if any(h not in trained for h in horizons):
        raise ValueError('Evaluation horizon was not represented in the training split')
    expected_shape=[len(pairs),*config['input_shape'][1:]]
    if list(data['x'].shape)!=expected_shape:
        raise ValueError('Dataset input shape does not match training configuration')
    expected_plane=np.asarray(np.asarray(horizons)/2,dtype=np.float32)[:,None,None]
    if not np.equal(data['x'][:,18],expected_plane).all():
        raise ValueError('Dataset horizon channel does not match training pairs')
    test=[row for row in pairs if row['split']=='test']
    stages=sorted({row['stage'] for row in test})
    episodes=sorted({str(row['episode']) for row in test if row.get('episode') is not None})
    return dict(trained_horizons_s=trained,split_counts=counts,
        test_scope=dict(samples=len(test),stages=stages,episodes=episodes,
            horizons_s=sorted({float(row['horizon_s']) for row in test}),
            scene_independence='not established by this manifest'),
        pairs_sha256=config['pairs_sha256'],train_config_sha256=sha(out/'train_config.json'))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--npu-outputs',type=Path)
    a=p.parse_args();out=a.run.resolve();torch.set_num_threads(4);cv2.setNumThreads(1)
    d=np.load(out/'dataset.npz',allow_pickle=False);pairs=json.loads((out/'pairs.json').read_text())['pairs']
    config=json.loads((out/'train_config.json').read_text())
    metadata=evaluation_metadata(out,d,pairs,config)
    model=PreAction(pretrained=False);model.load_state_dict(torch.load(out/'best.pt',weights_only=True));model.eval()
    x=d['x'];ablation_zero=x.copy();ablation_zero[:,11:18]=0
    shuffled=x.copy()
    for split in ('train','val','test'):
        ids=np.where(d['split']==split)[0];shuffled[ids,11:18]=x[np.roll(ids,1),11:18]
    with torch.no_grad():
        variants={name:model(torch.from_numpy(inp)).numpy().reshape(-1,4)
            for name,inp in [('preaction',x),('zero_action',ablation_zero),('shuffled_action',shuffled)]}
    variants['persistence']=np.zeros_like(variants['preaction'])
    if a.npu_outputs:
        saved=np.load(a.npu_outputs);key='npu' if 'npu' in saved else 'npu_output0'
        values=saved[key].reshape(-1,4)
        if len(values)!=len(x):raise ValueError('NPU rows must match original eval_inputs order')
        variants['npu_int8']=values
    # Keep the stronger same-definition HSV persistence baseline explicit.
    hsv_boxes=[box_of(red_mask(cv2.imread(r['current_image']))) for r in pairs]
    all_records={};visual_rows=[]
    for name,predictions in variants.items():
        records=[]
        for i,pred in enumerate(predictions):
            current_box=d['current_box'][i];target=d['future_box'][i];spec=d['spec'][i]
            predicted=future_box(current_box,spec,pred)
            pc=(predicted[:2]+predicted[2:])/2;tc=(target[:2]+target[2:])/2
            rgb=np.uint8(np.clip(x[i,:3].transpose(1,2,0)*255,0,255));mask=x[i,3]
            forecast,fmask=render_forecast(rgb,mask,current_box,spec,pred)
            union=(mask>.5)|(d['future_mask'][i]>.5)
            actual=d['future_rgb'][i].transpose(1,2,0)
            error=np.abs(forecast.astype(np.float32)/255-actual)
            fbin=fmask>.5;tbin=d['future_mask'][i]>.5
            expanded_center=pc;half=(predicted[2:]-predicted[:2])*.625
            region=np.r_[expanded_center-half,expanded_center+half]
            area=float(np.prod(target[2:]-target[:2]));overlap=np.prod(np.maximum(0,np.minimum(region[2:],target[2:])-np.maximum(region[:2],target[:2])))
            true_shift=np.linalg.norm(tc-(hsv_boxes[i][:2]+hsv_boxes[i][2:])/2)
            records.append(dict(id=str(d['id'][i]),split=str(d['split'][i]),stage=pairs[i]['stage'],horizon_s=float(pairs[i]['horizon_s']),
                center_error_px=float(np.linalg.norm(pc-tc)),bbox_iou=box_iou(predicted,target),
                foreground_rgb_mae=float(error[union].mean()) if union.any() else None,
                forecast_mask_iou=float((fbin&tbin).sum()/max(1,(fbin|tbin).sum())),
                expanded_roi_target_coverage=float(overlap/max(area,1.)),
                weak_reference_motion_px=float(true_shift),moving_at_least_2px=bool(true_shift>=2),
                forecast_bbox=predicted.tolist(),motion_parameters_scaled=pred.tolist()))
            if name=='preaction' and d['split'][i]=='test':
                actual8=np.uint8(np.clip(actual*255,0,255))
                visual_rows.append((str(d['id'][i]),rgb,forecast,actual8,records[-1]))
        all_records[name]=records
    all_records['hsv_persistence']=[dict(id=str(d['id'][i]),split=str(d['split'][i]),stage=pairs[i]['stage'],horizon_s=float(pairs[i]['horizon_s']),
        center_error_px=float(np.linalg.norm((hsv_boxes[i][:2]+hsv_boxes[i][2:]-d['future_box'][i,:2]-d['future_box'][i,2:])/2)),
        bbox_iou=box_iou(hsv_boxes[i],d['future_box'][i]),
        weak_reference_motion_px=all_records['preaction'][i]['weak_reference_motion_px'],
        moving_at_least_2px=all_records['preaction'][i]['moving_at_least_2px']) for i in range(len(x))]
    summary={}
    keys=('center_error_px','bbox_iou','foreground_rgb_mae','forecast_mask_iou','expanded_roi_target_coverage')
    for name,rows in all_records.items():
        summary[name]={}
        for split in ('train','val','test'):
            subset=[r for r in rows if r['split']==split]
            s=dict(n=len(subset),moving_samples=sum(r['moving_at_least_2px'] for r in subset))
            for key in keys:
                values=[r[key] for r in subset if r.get(key) is not None]
                if values:s[key]=float(np.mean(values))
            for group,moving in [('moving',True),('stationary',False)]:
                members=[r for r in subset if r['moving_at_least_2px']==moving]
                s[group]=dict(n=len(members),center_error_px=float(np.mean([r['center_error_px'] for r in members])) if members else None)
            summary[name][split]=s
    metrics=dict(schema='preaction-evaluation-v1',summary=summary,records=all_records,
        test_scope=metadata['test_scope'],training_metadata=metadata,
        supervision='future HSV component weak boxes, not physical truth',
        limitations=['Image forecast is an affine warp of current pixels; unseen content is not generated',
                     f'Trained horizons: {metadata["trained_horizons_s"]} seconds; arbitrary-horizon generalization is not established',
                     'Current YOLO mask and future HSV label definitions differ; HSV persistence is included'],
        model_sha256=sha(out/'best.pt'),robot_commands=0,motion_command_ready=False)
    name='evaluation_with_npu.json' if a.npu_outputs else 'evaluation.json'
    (out/name).write_text(json.dumps(metrics,indent=2))
    sheet=Image.new('RGB',(960,50+len(visual_rows)*280),'white');draw=ImageDraw.Draw(sheet)
    for k,title in enumerate(('CURRENT ROI','PREACTION (affine forecast)','RECORDED FUTURE')):
        draw.text((k*320+8,10),title,fill='black')
    for j,(label,cur,pred,actual,row) in enumerate(visual_rows):
        y=50+j*280
        for k,im in enumerate((cur,pred,actual)):
            sheet.paste(Image.fromarray(im).resize((264,264)),(k*320,y))
        draw.text((8,y-14),f'{label} +{row["horizon_s"]:g}s err={row["center_error_px"]:.2f}px',fill='black')
    sheet.save(out/'forecast_comparison.jpg')
    print(json.dumps(summary),flush=True)

if __name__=='__main__':main()
