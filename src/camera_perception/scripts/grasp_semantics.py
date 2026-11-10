"""Strict, model-independent validation of semantic proposals for a frozen RGB-D frame."""
import json
import math
import re


def parse_objects(text, max_objects=8):
    """Accept only a complete JSON array; never repair truncated or ambiguous output."""
    text = text.strip()
    if text.startswith('```') and text.endswith('```'):
        text = '\n'.join(text.splitlines()[1:-1]).strip()
    data = json.loads(text)
    if not isinstance(data, list) or len(data) > max_objects:
        raise ValueError('Expected a bounded list of objects')
    result = []
    for item in data:
        if not isinstance(item, dict):
            raise ValueError('Each object must be a JSON object')
        box = item.get('bbox_2d')
        if (not isinstance(box, list) or len(box) != 4 or
                any(type(x) not in (int, float) or not math.isfinite(x) or not 0 <= x <= 1000 for x in box) or
                box[0] >= box[2] or box[1] >= box[3]):
            raise ValueError('Invalid normalized xyxy box')
        label = item.get('label')
        if not isinstance(label, str) or not label.strip() or len(label) > 120 or any(ord(c) < 32 for c in label):
            raise ValueError('Invalid object label')
        if item.get('bbox_format', 'xyxy_1000') != 'xyxy_1000':
            raise ValueError('Unexpected coordinate convention')
        result.append({'label': label.strip(), 'bbox_2d': box})
    return result


def pixel_box(box, width, height):
    return [max(0, math.floor(box[0]*width/1000)), max(0, math.floor(box[1]*height/1000)),
            min(width, math.ceil(box[2]*width/1000)), min(height, math.ceil(box[3]*height/1000))]


def box_overlap(a, b):
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    aa = max(0, a[2]-a[0])*max(0, a[3]-a[1])
    bb = max(0, b[2]-b[0])*max(0, b[3]-b[1])
    return intersection/(aa+bb-intersection) if aa+bb>intersection else 0.


def requested_color_evidence(target, image, mask, minimum_fraction=.20):
    """Check a simple, explicitly requested chromatic surface attribute.

    This is not color constancy or language understanding. Complex descriptions
    (colored labels/parts, multiple colors, negations) remain unverified. A pass
    only means enough requested-color pixels are visible, not object identity.
    """
    import cv2
    import numpy as np
    if not target:
        return {'status':'not_requested'}
    text = target.lower()
    if re.search(r'\b(?:not|without|except|with|label|cap|handle)\b|不|非|除外|除了|标签|瓶盖|把手|带有',text):
        return {'status':'complex_description_not_checked'}
    colors = {
        'red': ('红',[(0,10),(170,179)]), 'orange':('橙',[(11,22)]),
        'yellow':('黄',[(23,35)]), 'green':('绿',[(36,85)]),
        'blue':('蓝',[(86,130)]), 'purple':('紫',[(131,169)])}
    requested = [name for name,(chinese,_) in colors.items()
                 if re.search(r'\b'+name+r'\b',text) or chinese+'色' in text]
    if len(requested)!=1:
        return {'status':'no_single_supported_color'}
    if image.shape[:2]!=mask.shape or np.count_nonzero(mask)<40:
        return {'status':'insufficient_mask'}
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    hue = np.zeros(mask.shape,dtype=bool)
    for low,high in colors[requested[0]][1]: hue |= (hsv[:,:,0]>=low)&(hsv[:,:,0]<=high)
    visible = mask!=0
    # Ignore gray backgrounds and near-black sensor/shadow pixels as positive evidence.
    matched = hue & (hsv[:,:,1]>=70) & (hsv[:,:,2]>=35) & visible
    fraction = float(np.count_nonzero(matched)/np.count_nonzero(visible))
    return {'status':'supported' if fraction>=minimum_fraction else 'requested_color_mismatch',
            'color':requested[0],'visible_color_fraction':fraction,'minimum_fraction':minimum_fraction,
            'scope':'RGB appearance evidence only; not semantic verification'}


def associate_masks(appearance_masks, depth_masks, candidates, min_seed_coverage=.65):
    """Require a unique two-way mask association, not just overlapping boxes.

    A tiny measured fragment can support object identity but cannot establish the
    complete outline or grasp width. Geometry validity is never upgraded here.
    """
    import numpy as np
    if len(depth_masks) != len(candidates):
        raise ValueError('Candidate/mask count mismatch')
    coverage = np.zeros((len(appearance_masks), len(depth_masks)), dtype=float)
    for i, appearance in enumerate(appearance_masks):
        if appearance.ndim != 2:
            raise ValueError('Masks must be two-dimensional')
        for j, seed in enumerate(depth_masks):
            if seed is None:
                continue
            if seed.shape != appearance.shape:
                raise ValueError('Aligned masks must have equal dimensions')
            area = np.count_nonzero(seed)
            if area:
                coverage[i, j] = np.count_nonzero((appearance != 0) & (seed != 0))/area
    matches = []
    for i in range(len(appearance_masks)):
        possible = np.flatnonzero(coverage[i] >= min_seed_coverage)
        if not len(possible):
            matches.append({'status': 'no_measured_object_support', 'candidate_index': None})
            continue
        if len(possible) != 1:
            matches.append({'status': 'multiple_depth_objects_in_mask', 'candidate_index': None})
            continue
        j = int(possible[0])
        if np.count_nonzero(coverage[:, j] >= min_seed_coverage) != 1:
            matches.append({'status': 'duplicate_or_ambiguous_semantic_masks', 'candidate_index': None})
            continue
        candidate = candidates[j]
        if candidate.get('status') in ('class_excluded_from_grasp', 'detector_class_conflict'):
            status = 'excluded_or_conflicting_candidate'
        elif not candidate.get('depth_measurement_valid'):
            status = 'candidate_has_no_measured_depth'
        else:
            status = 'measured_fragment_associated'
        matches.append({'status': status, 'candidate_index': j if status == 'measured_fragment_associated' else None,
                        'measured_seed_coverage': float(coverage[i, j])})
    return matches


def duplicate_inventory_masks(masks, depth_masks, candidates):
    """Find near-identical semantic masks sharing exactly one measured seed.

    These names must be rejected; inventory may independently recover the
    physical object using its depth/color seed. Different objects stay separate.
    """
    import numpy as np
    supports = []
    for mask in masks:
        supports.append([j for j,seed in enumerate(depth_masks) if seed is not None and
            candidates[j].get('depth_measurement_valid') and
            candidates[j].get('status') not in ('class_excluded_from_grasp','detector_class_conflict') and
            np.count_nonzero(seed)>0 and
            np.count_nonzero((mask!=0)&(seed!=0))/np.count_nonzero(seed)>=.65])
    duplicate = set()
    for i in range(len(masks)):
        for j in range(i):
            if len(supports[i])!=1 or supports[i]!=supports[j]: continue
            union = np.count_nonzero((masks[i]!=0)|(masks[j]!=0))
            intersection = np.count_nonzero((masks[i]!=0)&(masks[j]!=0))
            if union and intersection/union>=.85: duplicate.update((i,j))
    return duplicate


def color_consensus_prompt(mask, color_mask, depth_mask):
    """Find a background click only when a complete local color contour agrees.

    A tiny color patch on a reflective tool is insufficient evidence to remove its
    unseen-depth shaft. This is appearance refinement, never depth completion.
    """
    import cv2
    import numpy as np
    if color_mask is None or depth_mask is None or mask.shape != color_mask.shape or mask.shape != depth_mask.shape:
        return None
    area, color_area, seed_area = (np.count_nonzero(m) for m in (mask,color_mask,depth_mask))
    if min(area,color_area,seed_area)<40 or not .65<=color_area/area<=1.3:
        return None
    if np.count_nonzero((color_mask!=0)&(depth_mask!=0))/seed_area<.90:
        return None
    if np.count_nonzero((color_mask!=0)&(mask!=0))/color_area<.85:
        return None
    padded = cv2.dilate((color_mask!=0).astype(np.uint8),np.ones((7,7),np.uint8))
    extra = ((mask!=0)&(padded==0)).astype(np.uint8)
    if not 30<=np.count_nonzero(extra)<=.35*area:
        return None
    core = ((depth_mask!=0)&(color_mask!=0)&(mask!=0)).astype(np.uint8)
    _, positive_radius, _, positive = cv2.minMaxLoc(cv2.distanceTransform(core,cv2.DIST_L2,3))
    _, negative_radius, _, negative = cv2.minMaxLoc(cv2.distanceTransform(extra,cv2.DIST_L2,3))
    if positive_radius<2 or negative_radius<2:
        return None
    return {'points':[list(positive),list(negative)], 'labels':[1,0],
            'source':'measured_object_core_and_agreeing_local_rgb_contour',
            'excess_fraction_before':float(np.count_nonzero(extra)/area)}


SCENE_PROMPT = (
    'Describe each visible loose tabletop object with its actual short physical name and a tight bounding box. '
    'If the category is uncertain, use a broad shape or material name. Return only a JSON array; '
    'each object has keys bbox_2d and label. bbox_2d is [left,top,right,bottom] normalized to 0..1000 '
    'relative to the entire image. Exclude supporting surfaces, holes, shadows, the wall, and the dark robot fingers. '
    'Include at most six loose objects. If no object is visible, return [].')
