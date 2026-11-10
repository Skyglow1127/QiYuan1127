"""Local, on-demand Qwen3.5 grounding on one saved RGB-D observation.

Does not block the camera loop. Does not reuse a box on a newer frame. Qwen's
box selects an existing YOLO/depth candidate; Qwen never supplies metric depth.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('HF_HUB_DISABLE_TELEMETRY', '1')

def parse_box(text):
    text = text.strip()
    if text.startswith('```') and text.endswith('```'):
        text = '\n'.join(text.splitlines()[1:-1]).strip()
    data = json.loads(text)
    # Qwen's grounding training also produces a list of {bbox_2d,label} objects.
    # Accept a singleton, but never silently pick one of multiple proposed targets.
    if isinstance(data,list):
        if not data:
            return None
        if len(data)!=1 or not isinstance(data[0],dict) or 'bbox_2d' not in data[0]:
            raise ValueError('Ambiguous or malformed grounding list')
        data=dict(data[0])
        data.setdefault('found',True)
    if isinstance(data,dict) and 'bbox_2d' in data:
        data=dict(data)
        data.setdefault('found',True)
    if not isinstance(data, dict) or type(data.get('found')) is not bool:
        raise ValueError('Expected JSON object with boolean found')
    if not data['found']:
        return None
    # bbox_2d is explicitly requested in normalized 0..1000 coordinates by the
    # prompt below. Never infer coordinate units from the size of returned numbers.
    if 'bbox_2d' in data:
        if data.get('bbox_format','xyxy_1000') != 'xyxy_1000':
            raise ValueError('Unexpected bbox coordinate format')
        box = data['bbox_2d']
    else:
        if data.get('bbox_format') != 'xyxy_1000':
            raise ValueError('Expected explicit xyxy_1000 coordinate format')
        box = data.get('box')
    if not isinstance(box, list) or len(box) != 4 or any(
        type(x) not in (int, float) or not math.isfinite(x) or not 0 <= x <= 1000 for x in box
    ):
        raise ValueError('Invalid normalized box coordinates')
    if box[0] >= box[2] or box[1] >= box[3]:
        raise ValueError('Empty/reversed box')
    return box

def match_candidate(box, candidates, width, height, min_iou=.5):
    if box is None:
        return None, 'target_not_found', 0.
    x1, y1, x2, y2 = [box[0]*width/1000, box[1]*height/1000,
                       box[2]*width/1000, box[3]*height/1000]
    scores = []
    # Include excluded candidates in matching: a person's box must not be silently
    # skipped in favour of an overlapping allowed object.
    for index, candidate in enumerate(candidates):
        x, y, w, h = candidate['box_xywh']
        intersection = max(0, min(x2,x+w)-max(x1,x))*max(0,min(y2,y+h)-max(y1,y))
        union = (x2-x1)*(y2-y1)+w*h-intersection
        scores.append((intersection/union if union>0 else 0., index))
    scores.sort(reverse=True)
    if not scores or scores[0][0] < min_iou:
        return None, 'no_matching_visual_candidate', scores[0][0] if scores else 0.
    if len(scores)>1 and scores[1][0] >= min_iou and scores[0][0]-scores[1][0] < .1:
        return None, 'ambiguous_candidates', scores[0][0]
    overlap, index = scores[0]
    candidate = candidates[index]
    if candidate['status'] in ('class_excluded_from_grasp','detector_class_conflict'):
        return None, 'excluded_or_conflicting_candidate', overlap
    if not candidate.get('depth_measurement_valid'):
        return None, 'candidate_has_no_measured_depth', overlap
    return index, 'matched_geometric_candidate' if candidate['valid'] else 'matched_depth_only', overlap

def ground(image, target, model_path, threads, dtype='bfloat16'):
    from qwen_runtime import QwenRuntime, target_prompt
    engine = QwenRuntime(model_path, threads, dtype)
    raw, timing = engine.infer(image, target_prompt(target), 128)
    if timing['generation_hit_token_limit'] or timing['generation_hit_time_limit']:
        raise ValueError('Bounded Qwen generation did not finish')
    return raw, timing

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay', required=True, type=Path)
    p.add_argument('--target', required=True)
    p.add_argument('--model', type=Path, default=ROOT/'models/Qwen3.5-0.8B')
    p.add_argument('--mode', choices=['segment','fusion','hybrid','tabletop'], default='fusion')
    p.add_argument('--output', type=Path, default=ROOT/'output/qwen_target')
    p.add_argument('--threads', type=int, default=4)
    p.add_argument('--dtype', choices=['float32','bfloat16'], default='bfloat16')
    p.add_argument('--config', type=Path, default=ROOT/'grasp_config.yml')
    p.add_argument('--response-file', type=Path, help='Offline parser/replay test; explicitly recorded as simulated')
    a = p.parse_args()
    if a.threads < 1 or not a.target.strip():
        p.error('positive threads and nonempty target required')
    import cv2
    from PIL import Image
    replay = a.replay.resolve()
    output = a.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    temporary = output/'target_selection.json.tmp'
    temporary.write_text(json.dumps({'status':'running','offline_replay':True,
        'motion_command_ready':False,'matched_index':None,'candidate':None})+'\n')
    temporary.replace(output/'target_selection.json')
    storage = cv2.FileStorage(str(replay),cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError('Cannot read snapshot')
    color = (replay.parent/storage.getNode('color').string()).resolve()
    depth = (replay.parent/storage.getNode('depth').string()).resolve()
    storage.release()
    paths = [replay,color,depth,a.config.resolve()]
    hashes = {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    image = Image.open(color).convert('RGB')
    subprocess.run([str(ROOT/'build/detect'),'--replay',str(replay),'--mode',a.mode,
                    '--config',str(a.config.resolve()),
                    '--output',str(output),'--headless'],check=True)
    result = json.loads((output/'replay_result.json').read_text())
    report = {'schema_version':1, 'created_unix':time.time(), 'target':a.target, 'model':str(a.model.resolve()),
              'offline_replay':True, 'motion_command_ready':False,
              'simulated_response':bool(a.response_file), 'source_sha256':hashes,
              'source_host_received_ms':result['host_received_ms'],
              'scope':'selection on this snapshot only; recapture and revalidate before any physical action'}
    try:
        if a.response_file:
            raw = a.response_file.read_text()
            timing = {}
        else:
            raw,timing = ground(image,a.target,a.model,a.threads,a.dtype)
        report.update(raw_response=raw,timing=timing)
        box = parse_box(raw)
        index,status,iou = match_candidate(box,result['candidates'],*image.size)
        report.update(box_xyxy_1000=box,matched_index=index,status=status,match_iou=iou,
                      candidate=result['candidates'][index] if index is not None else None)
        if box is not None:
            import PIL.ImageDraw
            preview = image.copy()
            draw = PIL.ImageDraw.Draw(preview)
            draw.rectangle([box[0]*image.width/1000,box[1]*image.height/1000,
                            box[2]*image.width/1000,box[3]*image.height/1000],outline='orange',width=3)
            preview.save(output/'qwen_box.png')
    except Exception as error:
        report.update(status='grounding_failed',error=str(error),matched_index=None,candidate=None)
    # Saved image/metadata changes during long VLM inference invalidate the result.
    if any(hashlib.sha256(path.read_bytes()).hexdigest()!=hashes[str(path)] for path in paths):
        report.update(status='snapshot_changed',matched_index=None,candidate=None)
    temporary = output/'target_selection.json.tmp'
    temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(output/'target_selection.json')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if report['status'] in ('grounding_failed','snapshot_changed'):
        raise SystemExit(1)

if __name__ == '__main__':
    main()
