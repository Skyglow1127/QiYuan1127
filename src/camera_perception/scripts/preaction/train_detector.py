"""Isolated YOLO adaptation on current frames only; no camera or robot APIs."""
import os
os.environ.setdefault('YOLO_OFFLINE', 'true')
os.environ.setdefault('YOLO_CONFIG_DIR', '/home/aup/Camera/output/preaction_20261023/detector_settings')
os.environ.setdefault('OMP_NUM_THREADS', '4')
os.environ.setdefault('MKL_NUM_THREADS', '4')
from pathlib import Path
import argparse, json, shutil, time
import cv2
import numpy as np
import torch
from ultralytics import YOLO, settings
from core import YOLO_WEIGHTS, red_mask, sha


def prepare(pairs_path, out):
    source = json.loads(pairs_path.read_text())
    rows = []
    hashes = {}
    for pair in source['pairs']:
        split, key = pair['split'], pair['id']
        original = Path(pair['current_image'])
        digest = sha(original)
        assert digest == pair['image_t']['sha256']
        assert digest not in hashes, 'Current image reused across samples'
        hashes[digest] = split
        frame = cv2.imread(str(original))
        if frame is None:
            raise ValueError(f'Cannot load {original}')
        mask = red_mask(frame).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)
        polygon = cv2.approxPolyDP(contour, 0.002 * cv2.arcLength(contour, True), True).reshape(-1, 2)
        if len(polygon) < 3:
            raise ValueError(f'Invalid weak polygon for {key}')
        for kind in ('images', 'labels', 'masks'):
            (out/'dataset'/kind/split).mkdir(parents=True, exist_ok=True)
        ip = out/'dataset/images'/split/(key+'.jpg')
        lp = out/'dataset/labels'/split/(key+'.txt')
        mp = out/'dataset/masks'/split/(key+'.png')
        shutil.copyfile(original, ip)
        normalized = polygon.astype(np.float32)/np.array([frame.shape[1], frame.shape[0]], np.float32)
        lp.write_text('0 '+' '.join(f'{v:.7f}' for v in normalized.flatten())+'\n')
        cv2.imwrite(str(mp), mask*255)
        rows.append(dict(id=key, split=split, stage=pair['stage'], source_image=str(original),
            source_sha256=digest, image=str(ip), label=str(lp), mask=str(mp),
            target_present=True, label_kind='largest_red_HSV_component_outer_polygon_weak_label',
            polygon_px=polygon.tolist(), human_ground_truth=False, future_frame_used=False))
        # Fixed top strip only, and only when every red candidate and target pixel is absent.
        # Derived negatives retain the original split and do not count as new observations.
        if split in ('train', 'val'):
            background = frame[:120].copy()
            hsv = cv2.cvtColor(background, cv2.COLOR_BGR2HSV)
            raw_red = (((hsv[:,:,0] <= 10) | (hsv[:,:,0] >= 170)) &
                       (hsv[:,:,1] >= 85) & (hsv[:,:,2] >= 45))
            if not mask[:120].any() and not raw_red.any():
                key_bg = key+'_background'
                bip = out/'dataset/images'/split/(key_bg+'.jpg')
                blp = out/'dataset/labels'/split/(key_bg+'.txt')
                bmp = out/'dataset/masks'/split/(key_bg+'.png')
                cv2.imwrite(str(bip), background); blp.write_text('')
                cv2.imwrite(str(bmp), np.zeros(background.shape[:2], np.uint8))
                rows.append(dict(id=key_bg, split=split, stage=pair['stage'], source_image=str(original),
                    source_sha256=digest, image=str(bip), label=str(blp), mask=str(bmp),
                    target_present=False, label_kind='target_absent_top_strip_zero_red_pixels',
                    crop_xyxy=[0, 0, frame.shape[1], 120], human_ground_truth=False,
                    future_frame_used=False))
    manifest = dict(schema='preaction-current-frame-detector-v1', rows=rows,
        source_pairs=str(pairs_path), source_pairs_sha256=sha(pairs_path),
        positives={s:sum(r['split']==s and r['target_present'] for r in rows) for s in ('train','val','test')},
        background_crops={s:sum(r['split']==s and not r['target_present'] for r in rows) for s in ('train','val','test')},
        split_policy='Whole original motion stages; source current images remain in the existing pair split',
        scope='One scene/episode, HSV weak labels; no independent human ground truth',
        base_weights=str(YOLO_WEIGHTS), base_sha256=sha(YOLO_WEIGHTS),
        test_inference_performed=False, future_frame_used=False,
        production_replacement=False, robot_commands=0)
    (out/'dataset_manifest.json').write_text(json.dumps(manifest, indent=2))
    (out/'dataset.yaml').write_text(f'path: {out}/dataset\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: red_cube\n')
    shutil.copyfile(pairs_path, out/'source_pairs.json')
    return manifest


def evaluate_train_val(weights, manifest):
    model = YOLO(str(weights))
    rows = []
    for row in manifest['rows']:
        if row['split'] not in ('train','val'):
            continue
        result = model.predict(row['image'], imgsz=416, rect=False, conf=.25,
            retina_masks=True, device='cpu', max_det=10, verbose=False)[0]
        ref = cv2.imread(row['mask'], cv2.IMREAD_GRAYSCALE) > 0
        predicted = np.zeros(ref.shape, np.uint8)
        detected = result.masks is not None and len(result.boxes) > 0
        confidence = None
        if detected:
            best = int(result.boxes.conf.argmax().item())
            polygon = np.rint(result.masks.xy[best]).astype(np.int32)
            cv2.fillPoly(predicted, [polygon], 1)
            confidence = float(result.boxes.conf[best].item())
        union = np.logical_or(predicted, ref).sum()
        iou = float(np.logical_and(predicted, ref).sum()/max(int(union),1)) if row['target_present'] else None
        rows.append(dict(id=row['id'], split=row['split'], target_present=row['target_present'],
            detected=bool(detected), confidence=confidence, weak_mask_iou=iou))
    summary = {}
    for split in ('train','val'):
        pos=[r for r in rows if r['split']==split and r['target_present']]
        neg=[r for r in rows if r['split']==split and not r['target_present']]
        summary[split] = dict(positive_images=len(pos), positives_detected=sum(r['detected'] for r in pos),
            mean_weak_mask_iou=float(np.mean([r['weak_mask_iou'] for r in pos])),
            negative_images=len(neg), false_positive_images=sum(r['detected'] for r in neg))
    return dict(confidence_threshold=.25, threshold_selected_on='fixed before training; no test access',
        target_selection='highest-confidence single-class segmentation; no ground-truth selection',
        metrics=summary, rows=rows, test_inference_performed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    out=args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    settings.update({k:False for k in ('sync','wandb','mlflow','comet','clearml','dvc') if k in settings})
    manifest=prepare(args.pairs, out)
    config=dict(data=str(out/'dataset.yaml'), epochs=60, patience=15, imgsz=416, batch=4,
        device='cpu', workers=0, cache=False, optimizer='AdamW', lr0=.001, lrf=.1,
        momentum=.9, weight_decay=.0005, warmup_epochs=2., freeze=11, seed=42,
        deterministic=True, amp=False, plots=False, val=True, save=True, save_period=-1,
        project=str(out), name='training', exist_ok=False, mosaic=0., mixup=0., copy_paste=0.,
        degrees=8., translate=.1, scale=.15, fliplr=.5, flipud=0., hsv_h=0., hsv_s=.25, hsv_v=.25,
        close_mosaic=0, verbose=False)
    (out/'train_config.json').write_text(json.dumps(config, indent=2))
    started=time.perf_counter()
    model=YOLO(str(YOLO_WEIGHTS)); model.train(**config)
    elapsed=time.perf_counter()-started
    best=out/'training/weights/best.pt'
    evaluation=evaluate_train_val(best,manifest)
    (out/'train_val_evaluation.json').write_text(json.dumps(evaluation,indent=2))
    result=dict(status='DETECTOR_CANDIDATE_TEST_UNEVALUATED', training_seconds=elapsed,
        best=str(best), best_sha256=sha(best),
        base_unchanged=sha(YOLO_WEIGHTS)==manifest['base_sha256'],
        trained_component='YOLO26n-seg neck/head with first 11 modules frozen',
        positives=manifest['positives'], background_crops=manifest['background_crops'],
        train_val_metrics=evaluation['metrics'], test_inference_performed=False,
        future_frame_used=False, production_replacement=False, robot_commands=0)
    (out/'training_result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)


if __name__=='__main__':
    main()
