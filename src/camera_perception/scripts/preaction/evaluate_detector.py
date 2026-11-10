"""One final held-out current-frame comparison; no tuning or hardware access."""
import os
os.environ.setdefault('YOLO_OFFLINE','true')
os.environ.setdefault('YOLO_CONFIG_DIR','/home/aup/Camera/output/preaction_20261023/detector_settings')
from pathlib import Path
import argparse,json
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from core import red_mask,sha,box_of,box_iou


def select(model, frame):
    # Same current-frame color filter as PREACTION. No fallback is scored as a detection.
    hsv=red_mask(frame)
    result=model.predict(frame,imgsz=416,rect=False,conf=.25,device='cpu',
                         retina_masks=True,max_det=10,verbose=False)[0]
    candidates=[]
    if result.masks is not None:
        for polygon,confidence in zip(result.masks.xy,result.boxes.conf.cpu().numpy()):
            mask=np.zeros(frame.shape[:2],np.uint8)
            cv2.fillPoly(mask,[np.rint(polygon).astype(np.int32)],1)
            area=mask.sum()
            if area and float((mask*hsv).sum()/area)>=.2:
                candidates.append((float(confidence),mask))
    if not candidates:
        return None,None
    confidence,mask=max(candidates,key=lambda x:x[0])
    return mask,confidence


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve()
    output=run/'test_evaluation.json'
    if output.exists():
        raise FileExistsError('Final evaluation exists; do not silently overwrite')
    manifest=json.loads((run/'dataset_manifest.json').read_text())
    trained=json.loads((run/'training_result.json').read_text())
    paths={'old_fewshot':Path(manifest['base_weights']),'adapted_detector':Path(trained['best'])}
    assert sha(paths['old_fewshot'])==manifest['base_sha256']
    assert sha(paths['adapted_detector'])==trained['best_sha256']
    torch.set_num_threads(4);cv2.setNumThreads(1)
    references=[r for r in manifest['rows'] if r['split']=='test']
    if not references or not all(r['target_present'] and not r['future_frame_used'] for r in references):
        raise ValueError('A nonempty current-frame positive test split is required')
    rows=[];summary={}
    for key,path in paths.items():
        model=YOLO(str(path));model_rows=[]
        for ref in references:
            assert sha(ref['source_image'])==ref['source_sha256']
            frame=cv2.imread(ref['image'])
            truth=cv2.imread(ref['mask'],cv2.IMREAD_GRAYSCALE)>0
            mask,confidence=select(model,frame)
            hit=mask is not None
            pred=mask.astype(bool) if hit else np.zeros(truth.shape,bool)
            iou=float(np.logical_and(pred,truth).sum()/max(int(np.logical_or(pred,truth).sum()),1))
            target_box=box_of(truth)
            row=dict(id=ref['id'],stage=ref['stage'],model=key,image=ref['image'],
                image_sha256=sha(ref['image']),detected=hit,confidence=confidence,
                weak_mask_iou=iou,weak_box_iou=box_iou(box_of(mask),target_box) if hit else 0.,
                reference_box=target_box.tolist(),predicted_box=box_of(mask).tolist() if hit else None)
            rows.append(row);model_rows.append(row)
        summary[key]=dict(positive_images=len(model_rows),positives_detected=sum(x['detected'] for x in model_rows),
            mean_weak_mask_iou=float(np.mean([x['weak_mask_iou'] for x in model_rows])),
            mean_weak_box_iou=float(np.mean([x['weak_box_iou'] for x in model_rows])))
    result=dict(status='FINAL_HELDOUT_DETECTOR_EVALUATION',metrics=summary,rows=rows,
        settings=dict(confidence=.25,imgsz=416,rect=False,retina_masks=True,max_det=10,
            red_fraction_min=.2,red_definition='core.red_mask largest current-frame red component',
            target_selection='highest confidence among color-compatible candidates',
            hsv_fallback_scored_as_detection=False),
        weights={k:dict(path=str(v),sha256=sha(v)) for k,v in paths.items()},
        protocol='Test stage held out during training/selection; final evaluation after model checkpoint fixed; no retuning',
        label_source='Current-frame HSV largest-component masks; weak labels, not independent human ground truth',
        split_scope=dict(current_test_images=len(references),
            test_stages=sorted({r['stage'] for r in references}),
            source_split_policy=manifest.get('split_policy'),
            source_scope=manifest.get('scope')),
        future_images_evaluated=0,training_repeated=False,production_replacement=False,robot_commands=0)
    output.write_text(json.dumps(result,indent=2));print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
