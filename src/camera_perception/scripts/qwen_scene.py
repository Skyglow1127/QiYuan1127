"""Qwen3.5 -> SAM2.1 -> calibrated measured-depth evidence, on one frozen snapshot.

The semantic branch is intentionally outside the fast camera/detection loop.
RGB masks estimate visible appearance only. All output is offline and cannot
authorize motion, fill missing depth, or identify a robot contact/TCP pose.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

from grasp_semantics import SCENE_PROMPT, parse_objects, pixel_box, associate_masks, box_overlap, color_consensus_prompt, requested_color_evidence, duplicate_inventory_masks

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('YOLO_CONFIG_DIR', str(ROOT/'models/.ultralytics'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def label_preview(image, labels):
    """Render UTF-8 labels when a CJK font is available; never show broken '?' text."""
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    has_cjk = font_path.is_file()
    font = ImageFont.truetype(str(font_path),14) if has_cjk else ImageFont.load_default()
    view = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)); draw = ImageDraw.Draw(view)
    title = '记录画面 | 青色：外观与深度片段 | 橙色：轮廓截断 | 非执行指令' if has_cjk else 'RECORDED | cyan=appearance | orange=partial | NOT motion'
    draw.text((6,3),title,font=font,fill=(255,255,0),stroke_width=1,stroke_fill=(30,30,30))
    for identifier,text,position,color in labels:
        text = f'{identifier}: '+text
        if not has_cjk and not text.isascii(): text = f'Object {identifier} (UTF-8 label in JSON)'
        # Keep long labels inside the image; full labels remain in the JSON.
        while len(text)>10 and draw.textbbox((0,0),text,font=font)[2]>view.width-12:
            text = text[:-4]+'...'
        bounds = draw.textbbox((0,0),text,font=font)
        position = (max(6,min(position[0],view.width-bounds[2]-6)),
                    max(24,min(position[1],view.height-(bounds[3]-bounds[1])-8)))
        draw.text(position,text,font=font,fill=color,stroke_width=1,stroke_fill=(30,30,30))
    return cv2.cvtColor(np.array(view),cv2.COLOR_RGB2BGR)


def expanded_box(box, width, height):
    x1,y1,x2,y2 = box
    pad = min(32,max(8,round(.12*max(x2-x1,y2-y1))))
    return [max(0,x1-pad),max(0,y1-pad),min(width,x2+pad),min(height,y2+pad)]


def mask_quality(mask, score, box):
    import cv2
    import numpy as np
    x,y,w,h = cv2.boundingRect(mask)
    agreement = box_overlap(box,[x,y,x+w,y+h])
    status = None
    if np.count_nonzero(mask)<40: status = 'mask_too_small'
    elif not math.isfinite(score) or score<.5: status = 'low_mask_quality'
    elif agreement<.25: status = 'mask_disagrees_with_proposal'
    elif any(np.any(edge) for edge in (mask[0],mask[-1],mask[:,0],mask[:,-1])):
        status = 'appearance_cut_by_image_edge'
    return status,agreement


def freeze_snapshot(replay, directory):
    """Copy the exact saved observation and detect concurrent source edits."""
    import cv2
    replay = Path(replay).resolve()
    storage = cv2.FileStorage(str(replay), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError('Cannot open replay metadata')
    color = (replay.parent/storage.getNode('color').string()).resolve()
    depth = (replay.parent/storage.getNode('depth').string()).resolve()
    storage.release()
    paths = [replay, color, depth]
    before = {str(p): digest(p) for p in paths}
    directory.mkdir(parents=True, exist_ok=False)
    color_name, depth_name = 'color'+color.suffix, 'depth'+depth.suffix
    shutil.copyfile(color, directory/color_name)
    shutil.copyfile(depth, directory/depth_name)
    metadata = replay.read_text()
    for key, name in [('color', color_name), ('depth', depth_name)]:
        metadata, count = re.subn(r'^'+key+r'\s*:.*$', key+': '+json.dumps(name), metadata, flags=re.MULTILINE)
        if count != 1:
            raise ValueError('Expected one '+key+' path in snapshot metadata')
    frozen = directory/'snapshot.yml'
    frozen.write_text(metadata)
    if any(digest(p) != before[str(p)] for p in paths):
        raise ValueError('Source observation changed while copying')
    return frozen, directory/color_name, {
        'original_sha256': before,
        'frozen_sha256': {str(p.resolve()): digest(p) for p in [frozen, directory/color_name, directory/depth_name]}}


class SamRefiner:
    def __init__(self, model, threads=4, image_size=512):
        import torch
        from ultralytics import SAM
        if image_size not in (512, 1024):
            raise ValueError('SAM resolution must be 512 or 1024')
        if not Path(model).is_file():
            raise FileNotFoundError('Missing local SAM model: '+str(model))
        torch.set_num_threads(threads)
        self.model = SAM(str(model))
        self.image_size = image_size

    def refine(self, image, boxes):
        import cv2
        import numpy as np
        if not boxes:
            return [], [], 0.
        started = time.perf_counter()
        result = self.model.predict(image, bboxes=boxes, imgsz=self.image_size, device='cpu', verbose=False)[0]
        if result.masks is None or len(result.masks.data) != len(boxes):
            raise ValueError('SAM did not return one mask per proposal')
        masks = [cv2.resize(x.astype(np.uint8), (image.shape[1], image.shape[0]),
                           interpolation=cv2.INTER_NEAREST)*255 for x in result.masks.data.cpu().numpy()]
        return masks, result.boxes.conf.tolist(), 1000*(time.perf_counter()-started)

    def refine_with_points(self, image, box, prompt):
        import cv2
        import numpy as np
        result = self.model.predict(image,bboxes=[box],points=[prompt['points']],labels=[prompt['labels']],
                                    imgsz=self.image_size,device='cpu',verbose=False)[0]
        if result.masks is None or len(result.masks.data)!=1:
            raise ValueError('SAM point refinement did not return one mask')
        mask = cv2.resize(result.masks.data[0].cpu().numpy().astype(np.uint8),
                          (image.shape[1],image.shape[0]),interpolation=cv2.INTER_NEAREST)*255
        return mask,float(result.boxes.conf[0])


def analyze_scene(replay, config, output, runtime, sam_model, target=None, sam_size=512, threads=4, sam_refiner=None):
    import cv2
    import numpy as np
    from PIL import Image
    from qwen_runtime import target_prompt

    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report = {'schema_version': 1, 'status': 'running', 'offline_replay': True,
              'motion_command_ready': False, 'semantic_verified': False, 'requires_recapture': True,
              'timing_scope': 'analysis only; Qwen model startup is reported separately and can be reused by a worker',
              'scope': 'frozen observation only; semantic labels and masks are hypotheses, metric depth is measured',
              'target': target, 'objects': [], 'created_unix': time.time()}
    save_json(output/'scene_result.json', report)
    try:
        frozen, color_path, provenance = freeze_snapshot(replay, output/'observation')
        config = Path(config).resolve()
        report.update(provenance=provenance, config=str(config), config_sha256=digest(config))
        with (output/'baseline.log').open('w') as log:
            subprocess.run([str(ROOT/'build/detect'), '--replay', str(frozen), '--config', str(config),
                            '--mode', 'fusion', '--output', str(output/'baseline'), '--headless'],
                           check=True, stdout=log, stderr=subprocess.STDOUT, timeout=90)
        baseline = json.loads((output/'baseline/replay_result.json').read_text())
        image = cv2.imread(str(color_path))
        if image is None:
            raise ValueError('Cannot decode color observation')
        height, width = image.shape[:2]
        report['rgbd_pair_accepted'] = bool(baseline['rgbd_pair_accepted'])
        if not report['rgbd_pair_accepted']:
            raise ValueError('RGB-D pair is not accepted')
        report['visual_motion'] = baseline.get('visual_motion', {})
        if not baseline.get('depth_temporally_usable', True):
            raise ValueError('Moving or unverified RGB-D observation; recapture after settling')
        prompt = target_prompt(target) if target else SCENE_PROMPT
        if hasattr(runtime, 'object_limit'): runtime.object_limit = 1 if target else 6
        raw, timing = runtime.infer(Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), prompt, 256)
        report.update(raw_response=raw, prompt=prompt, qwen_timing=timing)
        save_json(output/'scene_result.json', report)
        if timing.get('generation_hit_token_limit') or timing.get('generation_hit_time_limit'):
            raise ValueError('Qwen generation exceeded its bound; no partial response accepted')
        objects = parse_objects(raw)
        report['qwen_proposed_count'] = len(objects)
        for item in objects: item['proposal_source'] = 'qwen3.5'
        if target and len(objects)>1:
            raise ValueError('Target request returned multiple objects')
        boxes = [pixel_box(item['bbox_2d'], width, height) for item in objects]
        # Coarse VLM boxes can miss a few pixels of a thin tool. A bounded search
        # margin lets SAM use edges; the resulting mask must still agree spatially.
        search_boxes = [expanded_box(box,width,height) for box in boxes]
        refiner = (sam_refiner or SamRefiner(sam_model, threads, sam_size)) if boxes else None
        masks, scores, sam_ms = refiner.refine(image, search_boxes) if refiner else ([], [], 0.)
        report['sam_timing'] = {'inference_and_preprocess_ms': sam_ms, 'input_size': sam_size,
                                'model': str(Path(sam_model).resolve()), 'model_sha256': digest(sam_model)}
        candidates = baseline['candidates']
        depth_masks = []
        for i in range(len(candidates)):
            path = output/f'baseline/replay_depth_component_{i}.png'
            depth_masks.append(cv2.imread(str(path), cv2.IMREAD_GRAYSCALE) if path.exists() else None)
        # Excluded model boxes also count as evidence against accepting an overlapping mask.
        for i, candidate in enumerate(candidates):
            if candidate['status'] in ('class_excluded_from_grasp', 'detector_class_conflict'):
                seed = np.zeros((height, width), np.uint8)
                x, y, w, h = candidate['box_xywh']
                seed[y:y+h, x:x+w] = 255
                depth_masks[i] = seed
        quality = [mask_quality(mask,score,box) for mask,score,box in zip(masks,scores,boxes)]
        duplicate = duplicate_inventory_masks([mask if quality[i][0] is None else np.zeros_like(mask)
                    for i,mask in enumerate(masks)],depth_masks,candidates) if target is None else set()
        for i in duplicate: quality[i] = ('duplicate_semantic_masks',quality[i][1])
        report['duplicate_semantic_count'] = len(duplicate)
        fallback_boxes = []
        # Inventory must not lose a measured object simply because the VLM omitted
        # its name. Targeted requests never fall back to an unrelated unknown object.
        if target is None:
            for j,candidate in enumerate(candidates):
                seed = depth_masks[j]
                if len(objects)>=8: break
                if candidate['source']!='depth_component' or not candidate.get('depth_measurement_valid') or seed is None:
                    continue
                area = np.count_nonzero(seed)
                if not area: continue
                # A known partial view stays partial: never turn one of its tiny
                # depth fragments into a second, apparently complete object.
                covered = any(quality[i][0] in (None,'appearance_cut_by_image_edge') and
                              np.count_nonzero((mask!=0)&(seed!=0))/area>=.65 for i,mask in enumerate(masks))
                if covered: continue
                path = output/f'baseline/replay_rgb_contour_{j}.png'
                rgb = cv2.imread(str(path),cv2.IMREAD_GRAYSCALE) if path.exists() else None
                if rgb is None or np.count_nonzero(rgb)<150 or np.count_nonzero((rgb!=0)&(seed!=0))/area<.9:
                    continue
                x,y,w,h = cv2.boundingRect(rgb); box = [x,y,x+w,y+h]
                objects.append({'label':'unclassified object','bbox_2d':[1000*x/width,1000*y/height,1000*(x+w)/width,1000*(y+h)/height],
                                'proposal_source':'measured_depth_and_rgb_fallback','baseline_candidate_index':j})
                boxes.append(box); search = expanded_box(box,width,height)
                search_boxes.append(search); fallback_boxes.append(search)
        report['geometry_fallback_count'] = len(fallback_boxes)
        if fallback_boxes:
            refiner = refiner or sam_refiner or SamRefiner(sam_model,threads,sam_size)
            extra_masks,extra_scores,extra_ms = refiner.refine(image,fallback_boxes)
            start = len(masks); masks.extend(extra_masks); scores.extend(extra_scores)
            quality.extend(mask_quality(mask,score,box) for mask,score,box in zip(extra_masks,extra_scores,boxes[start:]))
            report['sam_timing']['inference_and_preprocess_ms'] += extra_ms
        matches = associate_masks([mask if quality[i][0] is None else np.zeros_like(mask)
                                   for i, mask in enumerate(masks)], depth_masks, candidates)
        refinements = [{'status':'not_needed_or_insufficient_rgb_evidence'} for _ in masks]
        refine_started = time.perf_counter()
        for i, match in enumerate(matches):
            if match['candidate_index'] is None: continue
            j = match['candidate_index']
            path = output/f'baseline/replay_rgb_contour_{j}.png'
            color_mask = cv2.imread(str(path),cv2.IMREAD_GRAYSCALE) if path.exists() else None
            point_prompt = color_consensus_prompt(masks[i],color_mask,depth_masks[j])
            if point_prompt is None: continue
            revised, revised_score = refiner.refine_with_points(image,search_boxes[i],point_prompt)
            seed_support = np.count_nonzero((revised!=0)&(depth_masks[j]!=0))/max(1,np.count_nonzero(depth_masks[j]))
            color_support = np.count_nonzero((revised!=0)&(color_mask!=0))/max(1,np.count_nonzero(color_mask))
            outside_before = np.count_nonzero((masks[i]!=0)&(color_mask==0))
            outside_after = np.count_nonzero((revised!=0)&(color_mask==0))
            accepted = revised_score>=.5 and seed_support>=.90 and color_support>=.85 and outside_after<=.7*outside_before
            refinements[i] = dict(point_prompt,status='rgb_consensus_refined' if accepted else 'rgb_disagreement_unresolved',
                                  revised_sam_predicted_iou=revised_score)
            if accepted:
                masks[i],scores[i] = revised,revised_score
                quality[i] = mask_quality(revised,revised_score,boxes[i])
            else:
                quality[i] = ('appearance_color_disagreement',quality[i][1])
        report['sam_timing']['color_consensus_refine_ms'] = 1000*(time.perf_counter()-refine_started)
        color_checks = [requested_color_evidence(target,image,mask) for mask in masks]
        for i,check in enumerate(color_checks):
            if check['status']=='requested_color_mismatch' and quality[i][0] is None:
                quality[i] = ('requested_color_mismatch',quality[i][1])
        matches = associate_masks([mask if quality[i][0] is None else np.zeros_like(mask)
                                   for i, mask in enumerate(masks)],depth_masks,candidates)
        proposal_items = []
        accepted_rows = []
        overlay = image.copy()
        preview_labels = []
        for i, (item, box, mask, score, match) in enumerate(zip(objects, boxes, masks, scores, matches)):
            row = dict(item, box_xyxy_pixels=box, sam_predicted_iou=float(score), association=match,
                       semantic_verified=False, motion_command_ready=False, status=match['status'])
            row['sam_search_box_xyxy_pixels'] = search_boxes[i]
            row['mask_refinement'] = refinements[i]
            row['requested_color_check'] = color_checks[i]
            row['mask_box_iou'] = quality[i][1]
            nonzero = int(np.count_nonzero(mask))
            row['visible_mask_pixels'] = nonzero
            if quality[i][0] is not None: row['status'] = quality[i][0]
            # A person/conflict box must never be bypassed by a small semantic mask.
            for candidate in candidates:
                if candidate['status'] not in ('class_excluded_from_grasp', 'detector_class_conflict'):
                    continue
                x, y, w, h = candidate['box_xywh']
                if np.count_nonzero(mask[y:y+h, x:x+w]):
                    row['status'] = 'excluded_or_conflicting_candidate'
            filename = f'appearance_mask_{i}.png'
            cv2.imwrite(str(output/filename), mask)
            row['appearance_mask'] = filename
            if row['status'] == 'measured_fragment_associated':
                row['status'] = 'appearance_and_measured_fragment'
                proposal_items.append({'label': item['label'], 'mask': filename, 'mask_sha256': digest(output/filename)})
                accepted_rows.append(i)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, (255, 255, 0), 2)
                preview_labels.append((i,item['label'],(box[0],max(24,box[1]-22)),(255,255,0)))
            elif row['status'] == 'appearance_cut_by_image_edge':
                contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay,contours,-1,(0,145,255),2)
                preview_labels.append((i,item['label']+' / partial view',(box[0],max(24,box[1]-22)),(255,160,30)))
            report['objects'].append(row)
        depth_path = next(Path(p) for p in provenance['frozen_sha256'] if Path(p).stem == 'depth')
        save_json(output/'mask_proposals.json', {'source_snapshot': str(frozen),
            'source_snapshot_sha256': digest(frozen), 'source_color_sha256': digest(color_path),
            'source_depth_sha256': digest(depth_path), 'objects': proposal_items})
        with (output/'geometry.log').open('w') as log:
            subprocess.run([str(ROOT/'build/mask_depth'), '--replay', str(frozen), '--config', str(config),
                            '--masks', str(output/'mask_proposals.json'), '--output', str(output/'geometry')],
                           check=True, stdout=log, stderr=subprocess.STDOUT, timeout=30)
        geometry = json.loads((output/'geometry/mask_depth_result.json').read_text())
        for index, measured in zip(accepted_rows, geometry['candidates']):
            report['objects'][index]['depth_evidence'] = measured
        for i,(index,label,position,color) in enumerate(preview_labels):
            measured = report['objects'][index].get('depth_evidence',{})
            z = measured.get('optical_axis_depth_m')
            if measured.get('depth_measurement_valid') and isinstance(z,(int,float)) and math.isfinite(z):
                label += f' | surface Z {z*1000:.0f} mm'
                if not measured.get('valid'): label += ' | geometry incomplete'
                preview_labels[i] = (index,label,position,color)
        report['table'] = geometry['table']
        report['accepted_appearance_count'] = len(accepted_rows)
        report['target_selection_status'] = ('scene_inventory' if not target else
            'target_not_proposed' if not objects else 'no_depth_supported_target' if not accepted_rows else
            'semantic_hypothesis_with_measured_support')
        report['status'] = 'completed' if objects else 'no_object_proposed'
        cv2.imwrite(str(output/'scene_annotated.png'), label_preview(overlay,preview_labels))
        frozen_hashes = provenance['frozen_sha256']
        if any(digest(p) != h for p, h in frozen_hashes.items()) or digest(config) != report['config_sha256']:
            raise ValueError('Snapshot or configuration changed during inference')
    except Exception as error:
        report.update(status='failed', error=str(error), accepted_appearance_count=0, objects=[])
        raise
    finally:
        report['elapsed_s'] = time.perf_counter()-started
        save_json(output/'scene_result.json', report)
    return report


def main():
    program_started = time.perf_counter()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay', type=Path, required=True)
    p.add_argument('--config', type=Path, default=ROOT/'configs/kr260_tabletop.yml')
    p.add_argument('--model', type=Path, default=ROOT/'models/Qwen3.5-2B')
    p.add_argument('--sam-model', type=Path, default=ROOT/'models/sam2.1_t.pt')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--target', help='Optional natural-language target; semantics remain unverified')
    p.add_argument('--threads', type=int, default=4)
    p.add_argument('--dtype', choices=['float32', 'bfloat16'], default='bfloat16')
    p.add_argument('--sam-size', type=int, choices=[512, 1024], default=512)
    p.add_argument('--backend', choices=['transformers', 'llamacpp'], default='transformers')
    p.add_argument('--device', choices=['auto','cpu','vulkan'], default='auto', help='llama.cpp device; auto prefers a prepared local Vulkan backend')
    a = p.parse_args()
    if a.output.exists() or not 1 <= a.threads <= 24 or (a.target is not None and not a.target.strip()):
        p.error('Output must be new; threads 1..24; target must be nonempty')
    from qwen_runtime import QwenRuntime
    runtime_log = None
    if a.backend == 'llamacpp':
        from qwen_llama import create_llama_runtime
        a.output.parent.mkdir(parents=True,exist_ok=True)
        runtime_log = a.output.with_name(a.output.name+'_runtime.log')
        runtime = create_llama_runtime(a.model, a.threads, device=a.device, log_path=runtime_log)
    else:
        runtime = QwenRuntime(a.model, a.threads, a.dtype)
    try:
        result = analyze_scene(a.replay, a.config, a.output, runtime, a.sam_model, a.target, a.sam_size, a.threads)
    finally:
        if hasattr(runtime, 'close'): runtime.close()
        if runtime_log and runtime_log.exists() and a.output.is_dir():
            runtime_log.replace(a.output/'runtime.log')
    result['program_elapsed_s'] = time.perf_counter()-program_started
    save_json(a.output/'scene_result.json', result)
    print(json.dumps({'status': result['status'], 'accepted_appearance_count': result['accepted_appearance_count'],
                      'elapsed_s': result['elapsed_s'], 'output': str(a.output.resolve())}, ensure_ascii=False))


if __name__ == '__main__':
    main()
