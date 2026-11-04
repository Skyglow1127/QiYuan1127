#!/usr/bin/env python3
"""Replay manual intrinsic evidence and summarize coverage without fitting K."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import kr260_handeye as helper
import kr260_handeye_live as live


def summarize(root):
    board = json.loads(live.DEFAULT_BOARD.read_text())
    _, obj = live.geometry(board)
    accepted, rejected, normals = [], [], []
    calibration = None
    for path in sorted(root.glob('*/capture.json')):
        record = json.loads(path.read_text())
        if record.get('schema') != 'kr260-manual-intrinsics-view-v1':
            continue
        if not record.get('complete'):
            rejected.append(dict(folder=path.parent.name, error=record.get('error')))
            continue
        assert record['board_sha256'] == helper.digest(live.DEFAULT_BOARD)
        for name, sha in record['raw_sha256'].items():
            assert helper.digest(path.parent/name) == sha, str(path.parent/name)
        events = [helper.parse_feedback(r['line'], r['t']) for r in map(json.loads,
            (path.parent/'feedback.jsonl').read_text().splitlines())]
        obs = record['observation']
        replay = helper.assess_window([e for e in events if e], obs['begin_aipc_monotonic_s'],
                                      obs['end_aipc_monotonic_s'])
        assert np.allclose(replay['joint_median'], obs['joint_median'], atol=1e-8, rtol=0)
        cal = record['camera_calibration']
        if calibration is not None:
            assert cal == calibration, 'Mixed camera parameters'
        calibration = cal
        xy = np.array(record['mean_corners_sb'], dtype=np.float64)
        k = np.array(cal['camera_matrix'], dtype=np.float64)
        d = np.array(cal['distortion'], dtype=np.float64)
        ok, rv, tv = cv2.solvePnP(obj, xy, k, d)
        if not ok:
            raise ValueError('Diagnostic PnP failed')
        n = cv2.Rodrigues(rv)[0][:, 2]
        normals.append(n)
        accepted.append(dict(folder=path.parent.name, split=record['split'], label=record['label'],
            capture_sha256=helper.digest(path), center_px=record['center_px'],
            corner_bounds_px=record['corner_bounds_px'],
            inner_grid_image_area_fraction=record['inner_grid_image_area_fraction'],
            max_corner_shift_px=record['max_corner_shift_px'],
            approximate_normal_using_original_K=n.tolist(),
            approximate_normal_xz_yz_deg=np.rad2deg(np.arctan2(n[:2], n[2])).tolist()))
    normal_span = max([float(np.rad2deg(np.arccos(np.clip(np.dot(a,b), -1, 1))))
                       for a in normals for b in normals] or [0.])
    result = dict(schema='manual-intrinsic-coverage-v1', robot_commands_sent=False,
        calibration_written=False, accepted=accepted, rejected=rejected,
        counts={split:sum(r['split']==split for r in accepted) for split in ['train','validation']},
        approximate_board_normal_span_deg=normal_span,
        note='Normals use the original unverified K only to guide view diversity. They are not angular ground truth. One folder is one independent board view.')
    live.save(root/'coverage.json', result)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    args = p.parse_args()
    print(json.dumps(summarize(args.root), ensure_ascii=False, indent=2))
