"""Prepare current-image tensors and a full ACT sequence for PREACTION v2.

This is an offline input adapter, not a trained forecast or a robot executor.
It never loads legacy endpoint weights into the new sequence architecture.
Run with: python -m scripts.preaction.act_sequence --request ... --output ...
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .core import box_of, crop, crop_spec, red_mask
from .sequence_contract import build_act_sequence, validate_sequence_packet

REQUEST_SCHEMA = 'd1-preaction-act-request-v1'
SERIALS = {'global': 'CC1365302K0', 'wrist': 'CC1365301EY'}
CONTRACT_FIELDS = ('q_current', 'observation_id', 'q_observed_at',
                   'prediction_started_at', 'prediction_completed_at',
                   'valid_steps', 'commanded_joint_mask')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    def invalid(value):
        raise ValueError('nonfinite_json_constant:' + value)
    return json.loads(Path(path).read_text(), parse_constant=invalid)


def build_request_packet(request):
    if not isinstance(request, dict) or request.get('schema') != REQUEST_SCHEMA:
        raise ValueError('unsupported_sequence_request')
    if any(key not in request for key in (*CONTRACT_FIELDS, 'chunk', 'current_image')):
        raise ValueError('missing_explicit_request_fields')
    return build_act_sequence(request['chunk'], **{k: request[k] for k in CONTRACT_FIELDS})


def prepare_observation(packet, current_image):
    """Use only an identified, hash-bound image available before inference.

    Color selection is explicitly a current-frame HSV prototype, not a new
    YOLO/Qwen result, ground-truth segmentation, or physical object identity.
    """
    packet = validate_sequence_packet(packet)
    if not isinstance(current_image, dict):
        raise ValueError('missing_current_image_metadata')
    role = current_image.get('role')
    if role not in SERIALS or current_image.get('serial') != SERIALS[role]:
        raise ValueError('camera_identity_mismatch')
    available = current_image.get('available_t')
    if (isinstance(available, bool) or not isinstance(available, (int, float))
            or not math.isfinite(available)):
        raise ValueError('missing_image_availability_time')
    age = packet['prediction_started_at'] - available
    if not 0 < age <= .25:
        raise ValueError('noncausal_or_stale_current_image')
    board_age = current_image.get('board_age_at_fetch_ms')
    capture_age = current_image.get('capture_age_estimate_s')
    for value in (board_age, capture_age):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0):
            raise ValueError('missing_or_invalid_image_capture_age')
    estimated_age = age + board_age / 1000
    if estimated_age > .25 or capture_age > .25:
        raise ValueError('stale_image_capture')
    if not math.isclose(capture_age, estimated_age, rel_tol=0, abs_tol=1e-6):
        raise ValueError('inconsistent_image_capture_age')
    path = Path(current_image['path']).resolve()
    encoded = path.read_bytes()
    if hashlib.sha256(encoded).hexdigest() != current_image.get('sha256'):
        raise ValueError('current_image_hash_mismatch')
    bgr = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None or bgr.shape != (480, 640, 3):
        raise ValueError('expected_original_640x480_rgb')
    mask = red_mask(bgr)
    box = box_of(mask)
    spec = crop_spec(box)
    rgb = cv2.cvtColor(crop(bgr, spec), cv2.COLOR_BGR2RGB)
    local_mask = crop(mask, spec, nearest=True)
    rgb_mask = np.concatenate((rgb.transpose(2, 0, 1).astype(np.float32) / 255,
                               local_mask[None].astype(np.float32)), axis=0)[None]
    arrays = dict(rgb_mask=rgb_mask,
                  q_features=np.asarray(packet['q_features'], dtype=np.float32)[None],
                  sequence_features=np.asarray(packet['sequence_features'], dtype=np.float32)[None],
                  # Float64 preserves absolute host clock precision in exports.
                  raw_commands=np.asarray(packet['chunk']['commands'], dtype=np.float64)[None],
                  target_times=np.asarray(packet['chunk']['target_times'], dtype=np.float64)[None],
                  valid_steps=np.asarray(packet['valid_steps'], dtype=np.bool_)[None],
                  commanded_joint_mask=np.asarray(packet['commanded_joint_mask'], dtype=np.bool_)[None],
                  future_at_completion_mask=np.asarray(packet['future_at_completion_mask'], dtype=np.bool_)[None],
                  valid_future_steps=np.asarray(packet['valid_future_steps'], dtype=np.bool_)[None])
    metadata = {**current_image, 'path': str(path), 'available_age_s': age,
                'selection': 'explicit_current_frame_HSV_largest_red_component',
                'selection_is_ground_truth': False, 'crop': spec,
                'box_xyxy': box.tolist(), 'future_image_used': False,
                'capture_age_verified_s': estimated_age,
                'capture_age_is_estimate': True}
    return arrays, metadata, rgb


def prepare_request(request, output):
    packet = build_request_packet(request)
    arrays, image_metadata, rgb = prepare_observation(packet, request['current_image'])
    out = Path(output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'request.json').write_text(json.dumps(request, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    (out / 'sequence.json').write_text(json.dumps(packet, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    np.savez_compressed(out / 'inputs.npz', **arrays)
    cv2.imwrite(str(out / 'current_roi.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    summary = dict(schema='d1-preaction-act-input-result-v1', status='INPUTS_PREPARED_ONLY',
                   observation_id=packet['observation_id'],
                   shapes={k: list(v.shape) for k, v in arrays.items()},
                   image=image_metadata, valid_future_steps=packet['valid_future_steps'],
                   raw_target_times_preserved=True, time_shift_applied_s=0,
                   old_endpoint_model_used=False, new_model_inference=False,
                   forecast_generated=False, trained_sequence_weights_available=False,
                   model_input_note='Sequence architecture requires separately trained sequence weights.',
                   input_provenance=request.get('provenance', {}),
                   robot_commands=0, hardware_accessed=False, live_eligible=False,
                   files_sha256={f: sha(out / f) for f in ('request.json', 'sequence.json', 'inputs.npz')})
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing results are never replaced')
    args = parser.parse_args()
    print(json.dumps(prepare_request(read_json(args.request), args.output), ensure_ascii=False))


if __name__ == '__main__':
    main()
