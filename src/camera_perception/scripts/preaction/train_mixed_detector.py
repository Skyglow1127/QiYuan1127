"""One small mixed-replay detector experiment; saved images only, no hardware."""
import os
os.environ.setdefault('YOLO_OFFLINE','true')
os.environ.setdefault('YOLO_CONFIG_DIR','/home/aup/Camera/output/preaction_20261023/detector_settings')
os.environ.setdefault('OMP_NUM_THREADS','4')
os.environ.setdefault('MKL_NUM_THREADS','4')
from pathlib import Path
import argparse,json,time,shutil,sys
import cv2,numpy as np,torch
from ultralytics import YOLO,settings
sys.path.insert(0,'/home/aup/Camera/scripts/preaction')
from core import sha,red_mask

ORIGINAL=Path('/home/aup/Camera/output/fewshot_qwen_yolo_20261023')
NEAR=Path('/home/aup/Camera/output/preaction_20261023/detector_run_001')
DEVELOPMENT=Path('/home/aup/D1/Control/logs/grasp_100mm_20261023_1950/perception/wrist.jpg')


def prepare(out,pairs_path=None):
    rows=[];source_split={};image_split={};files=[]
    near_source=NEAR
    if pairs_path is not None:
        # Current-frame data construction only; this does not train a second detector.
        from train_detector import prepare as prepare_current_frames
        near_source=out/'current_pairs_source';near_source.mkdir(parents=True,exist_ok=False)
        prepare_current_frames(pairs_path.resolve(),near_source)
    source_roots=(ORIGINAL,near_source)
    for collection,root in [('original',ORIGINAL),('nearfield',near_source)]:
        manifest_path=root/'dataset_manifest.json'
        manifest=json.loads(manifest_path.read_text())
        for source in manifest['rows']:
            row=dict(source);row['collection']=collection;row['id']=collection+'_'+source['id']
            assert sha(row['source_image'])==row['source_sha256']
            for table,digest in [(source_split,row['source_sha256']),(image_split,sha(row['image']))]:
                if digest in table and table[digest]!=row['split']:
                    raise ValueError('Cross-split original or derived image hash collision')
                table[digest]=row['split']
            rows.append(row)
            # Replay changes sampling frequency only, never independent-sample count.
            repeats=4 if collection=='original' and row['split']=='train' else 1
            for repeat in range(repeats):
                key=f'{row["id"]}_replay{repeat}'
                for kind in ('images','labels'):(out/'dataset'/kind/row['split']).mkdir(parents=True,exist_ok=True)
                ip=out/'dataset/images'/row['split']/(key+'.jpg')
                lp=out/'dataset/labels'/row['split']/(key+'.txt')
                shutil.copyfile(row['image'],ip);shutil.copyfile(row['label'],lp)
                files.append(dict(id=key,unique_row_id=row['id'],split=row['split'],image=str(ip),label=str(lp),replay_index=repeat))
    development_hash=sha(DEVELOPMENT)
    assert development_hash not in source_split and development_hash not in image_split
    counts={s:dict(unique_original_images=len({r['source_sha256'] for r in rows if r['split']==s}),
        positive_images=sum(r['split']==s and r['target_present'] for r in rows),
        negative_crops=sum(r['split']==s and not r['target_present'] for r in rows),
        training_files_with_replay=sum(r['split']==s for r in files)) for s in ('train','val','test')}
    result=dict(schema='mixed-replay-detector-v1',rows=rows,files=files,counts=counts,
        source_manifests=[dict(path=str(root/'dataset_manifest.json'),sha256=sha(root/'dataset_manifest.json')) for root in source_roots],
        pairs_input=str(pairs_path.resolve()) if pairs_path is not None else None,
        pairs_input_sha256=sha(pairs_path) if pairs_path is not None else None,
        cross_split_original_hash_collisions=0,cross_split_derived_hash_collisions=0,
        split_policy='Preserve both existing splits; original/source and actual-image SHA-256 checked',
        replay_policy='Original training images/crops repeated four times; nearfield once; validation/test no replay',
        development=dict(image=str(DEVELOPMENT),sha256=development_hash,used_in_training=False,used_for_checkpoint_selection=False,
            role='Previously observed failure prompted this intervention; development regression check, not independent test'),
        limitations=['Historical test images have been evaluated previously; this is a regression suite, not a fresh blind test',
            'All historical train/val/test come from one original experiment/scene; temporally related stages remain related',
            'Original silhouette labels are assistant weak polygons; nearfield labels are HSV weak masks; neither independent measured truth'],
        hardware_commands=0,synthetic_real_robot_samples=0)
    (out/'dataset_manifest.json').write_text(json.dumps(result,indent=2))
    (out/'dataset.yaml').write_text(f'path: {out}/dataset\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: red_cube\n')
    return result


def predict(model,frame):
    try:hsv=red_mask(frame)
    except ValueError:hsv=np.zeros(frame.shape[:2],np.float32)
    start=time.perf_counter()
    result=model.predict(frame,imgsz=416,rect=False,conf=.25,retina_masks=True,max_det=10,device='cpu',verbose=False)[0]
    elapsed=(time.perf_counter()-start)*1000
    candidates=[]
    if result.masks is not None:
        for polygon,confidence in zip(result.masks.xy,result.boxes.conf.cpu().numpy()):
            mask=np.zeros(frame.shape[:2],np.uint8)
            cv2.fillPoly(mask,[np.rint(polygon).astype(np.int32)],1)
            area=mask.sum()
            if area and float((mask*hsv).sum()/area)>=.2:candidates.append((float(confidence),mask))
    selected=max(candidates,key=lambda x:x[0]) if candidates else None
    return selected,len(result.boxes),elapsed


def evaluate(out,manifest,weights):
    all_rows=[];development={};panels=[]
    for model_name,path in weights.items():
        model=YOLO(str(path))
        for row in manifest['rows']:
            frame=cv2.imread(row['image']);truth=cv2.imread(row['mask'],0)>0
            selected,raw_count,ms=predict(model,frame)
            confidence,mask=selected if selected else (None,np.zeros(truth.shape,np.uint8))
            union=np.logical_or(mask,truth).sum()
            iou=float(np.logical_and(mask,truth).sum()/max(int(union),1)) if row['target_present'] else None
            all_rows.append(dict(model=model_name,id=row['id'],collection=row['collection'],split=row['split'],
                target_present=row['target_present'],detected=selected is not None,confidence=confidence,
                raw_detection_count=raw_count,weak_mask_iou=iou,predict_ms=ms))
        frame=cv2.imread(str(DEVELOPMENT));truth=red_mask(frame)>0
        selected,raw_count,ms=predict(model,frame)
        confidence,mask=selected if selected else (None,np.zeros(truth.shape,np.uint8))
        dev=dict(detected=selected is not None,confidence=confidence,raw_detection_count=raw_count,
            weak_hsv_iou=float(np.logical_and(mask,truth).sum()/max(int(np.logical_or(mask,truth).sum()),1)),
            image=str(DEVELOPMENT),used_in_training=False,used_for_checkpoint_selection=False,predict_ms=ms)
        development[model_name]=dev
        cv2.imwrite(str(out/f'development_{model_name}_mask.png'),mask*255)
        panel=frame.copy();green=np.zeros_like(panel);green[:,:,1]=255;sel=mask>0
        panel[sel]=cv2.addWeighted(panel,.55,green,.45,0)[sel]
        cv2.rectangle(panel,(0,0),(640,30),(0,0,0),-1)
        cv2.putText(panel,model_name+': '+(f'{confidence:.3f}' if confidence else 'miss'),(8,22),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1,cv2.LINE_AA)
        panels.append(panel)
    metrics={}
    for model_name in weights:
        metrics[model_name]={}
        for collection in ('original','nearfield'):
            metrics[model_name][collection]={}
            for split in ('train','val','test'):
                rows=[r for r in all_rows if r['model']==model_name and r['collection']==collection and r['split']==split]
                pos=[r for r in rows if r['target_present']];neg=[r for r in rows if not r['target_present']]
                metrics[model_name][collection][split]=dict(positive_images=len(pos),positives_detected=sum(r['detected'] for r in pos),
                    mean_weak_mask_iou=float(np.mean([r['weak_mask_iou'] for r in pos])),negative_images=len(neg),
                    color_filtered_false_positive_images=sum(r['detected'] for r in neg),
                    raw_false_positive_images=sum(r['raw_detection_count']>0 for r in neg))
    result=dict(metrics=metrics,development=development,rows=all_rows,
        weights={k:dict(path=str(v),sha256=sha(v)) for k,v in weights.items()},
        inference_settings=dict(confidence=.25,imgsz=416,rect=False,retina_masks=True,max_det=10,red_fraction_min=.2,
            device='cpu',threads=4,selection='highest confidence among color-compatible masks; no HSV fallback'),
        historical_test_scope='Fixed original splits; already evaluated in past rounds, not a fresh blind test',
        development_scope='Current failure frame excluded from training and validation; intervention-specific diagnostic',
        training_repeated_after_evaluation=False,robot_commands=0)
    (out/'comparison.json').write_text(json.dumps(result,indent=2))
    cv2.imwrite(str(out/'development_comparison.jpg'),np.hstack(panels))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pairs',type=Path,help='New causal pairs JSON; only current frames are added with their existing split')
    a=p.parse_args()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(4);cv2.setNumThreads(1)
    settings.update({k:False for k in ('sync','wandb','mlflow','comet','clearml','dvc') if k in settings})
    manifest=prepare(out,a.pairs)
    base=ORIGINAL/'training/weights/best.pt';near=NEAR/'training/weights/best.pt'
    original_hashes={str(path):sha(path) for path in (base,near)}
    config=dict(data=str(out/'dataset.yaml'),epochs=22,time=.025,patience=8,imgsz=416,batch=4,
        device='cpu',workers=0,cache=False,optimizer='AdamW',lr0=.0005,lrf=.1,momentum=.9,
        weight_decay=.0005,warmup_epochs=2.,freeze=11,seed=42,deterministic=True,amp=False,
        plots=False,val=True,save=True,save_period=-1,project=str(out),name='training',exist_ok=False,
        mosaic=0.,mixup=0.,copy_paste=0.,degrees=8.,translate=.1,scale=.15,fliplr=.5,
        flipud=0.,hsv_h=0.,hsv_s=.25,hsv_v=.25,close_mosaic=0,verbose=False)
    (out/'train_config.json').write_text(json.dumps(config,indent=2))
    started=time.perf_counter();YOLO(str(base)).train(**config);training_seconds=time.perf_counter()-started
    mixed=out/'training/weights/best.pt'
    result=evaluate(out,manifest,{'old_fewshot':base,'nearfield_only':near,'mixed_replay':mixed})
    summary=dict(status='MIXED_REPLAY_EXPERIMENT_COMPLETE_NOT_DEPLOYED',training_seconds=training_seconds,
        weights=str(mixed),weights_sha256=sha(mixed),previous_weights_unchanged=all(sha(p)==v for p,v in original_hashes.items()),
        counts=manifest['counts'],development=result['development'],metrics=result['metrics'],
        new_action_samples=0,action_model_trained=False,grasp_success_measured=False,
        deployment_changed=False,robot_commands=0)
    (out/'training_result.json').write_text(json.dumps(summary,indent=2))
    shutil.copyfile(__file__,out/'reproduce_training.py')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
