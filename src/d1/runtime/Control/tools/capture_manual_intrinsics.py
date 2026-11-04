#!/usr/bin/env python3
"""One read-only intrinsic view from existing camera streams and D1 feedback.

Each folder is one independently positioned board view, never four training
views. No PnP/reprojection gate based on the intrinsics being investigated.
"""
import argparse
import base64
import datetime
import hashlib
import json
from pathlib import Path
import time
import urllib.request

import cv2
import numpy as np
import kr260_handeye as helper
import kr260_handeye_live as live


def decode(data, board):
    if data.get('schema') != 'kr260-color-handeye-v1' or data['device']['serial'] != board['camera_serial']:
        raise ValueError('Wrong schema or camera')
    if not 0 <= data['board_frame_age_ms'] <= 250:
        raise ValueError('Stale camera frame')
    raw = base64.b64decode(data['image_base64'], validate=True)
    if hashlib.sha256(raw).hexdigest() != data['image_sha256']:
        raise ValueError('Image checksum mismatch')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    cal = data['calibration']
    if image is None or image.shape != (480, 640, 3) or (cal['width'], cal['height']) != (640, 480):
        raise ValueError('Unexpected image dimensions')
    if cal.get('mirrored') is not False or cal.get('distortion_order') != 'k1,k2,p1,p2,k3,k4,k5,k6':
        raise ValueError('Unverified pixel convention')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    pattern, _ = live.geometry(board)
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:
        raise ValueError('Full 48-corner board not detected')
    xy = corners.reshape(-1, 2)
    if not np.isfinite(xy).all() or len(xy) != 48:
        raise ValueError('Invalid corners')
    classic_ok, classic = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if classic_ok:
        classic = cv2.cornerSubPix(gray, classic, (5, 5), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, .001)).reshape(-1, 2)
        if np.linalg.norm(classic[::-1]-xy) < np.linalg.norm(classic-xy):
            classic = classic[::-1].copy()
    else:
        classic = None
    annotated = image.copy()
    cv2.drawChessboardCorners(annotated, pattern, corners, True)
    return raw, xy, classic, annotated


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--split', choices=['train', 'validation'], required=True)
    p.add_argument('--label', required=True)
    args = p.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    baseline = json.loads(args.baseline.read_text())
    if not baseline.get('complete'):
        raise ValueError('Invalid fixed robot baseline')
    board = json.loads(live.DEFAULT_BOARD.read_text())
    result = dict(schema='kr260-manual-intrinsics-view-v1', complete=False,
        created_at=datetime.datetime.now().astimezone().isoformat(), split=args.split,
        label=args.label, robot_commands_sent=False, handeye_sample=False,
        board_sha256=helper.digest(live.DEFAULT_BOARD), baseline_sha256=helper.digest(args.baseline),
        collection_script_sha256=helper.digest(__file__), independent_view_count=1,
        calibrated=False, note='Manually moved board with fixed robot/camera. Frames within this folder are repeats, not independent training views.')
    observer = None
    try:
        helper.check_link('enp99s0')
        observer = helper.Observer(live.CONTROL/'build/arm_control', 'enp99s0', out)
        observer.pump(2)
        begin = time.monotonic()
        observer.pump(1.8)
        stable_deadline = time.monotonic() + 15
        rejected_windows = []
        while True:
            end = time.monotonic()
            begin = max(begin, end - 1.8)
            try:
                helper.assess_window(observer.events, begin, end)
                break
            except ValueError as exc:
                if not str(exc).startswith('Robot not stationary:') or end >= stable_deadline:
                    raise
                rejected_windows.append(dict(begin=begin, end=end, error=str(exc)))
                observer.pump(.2)
        result['pre_capture_rejected_stationarity_windows'] = rejected_windows
        op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        meta, points, classic_points, windows = [], [], [], []
        for i in range(4):
            started = time.monotonic()
            with op.open('http://127.0.0.1:18760/handeye-snapshot', timeout=3) as response:
                data = json.load(response)
            ended = time.monotonic()
            if ended-started > 1:
                raise ValueError('Snapshot latency >1s')
            raw = base64.b64decode(data['image_base64'], validate=True)
            (out/f'color_{i}.jpg').write_bytes(raw)
            m = {k:v for k,v in data.items() if k != 'image_base64'}
            live.save(out/f'camera_{i}.json', m)
            _, xy, classic, annotation = decode(data, board)
            if points and np.linalg.norm(xy[::-1]-points[0]) < np.linalg.norm(xy-points[0]):
                xy = xy[::-1].copy()
                if classic is not None:
                    classic = classic[::-1].copy()
            points.append(xy); classic_points.append(None if classic is None else classic.tolist())
            cv2.imwrite(str(out/f'corners_{i}.jpg'), annotation)
            meta.append(m); windows.append([started, ended])
            observer.pump(.3)
            helper.assess_window(observer.events, begin, time.monotonic())
        observer.pump(1.1)
        observation = helper.assess_window(observer.events, begin, time.monotonic())
        if np.max(np.abs(np.array(observation['joint_median'])-baseline['observation']['joint_median'])) > .2+1e-5:
            raise ValueError('Robot changed from fixed session baseline by >0.2')
        if any(b['frame']['index'] <= a['frame']['index'] for a,b in zip(meta, meta[1:])):
            raise ValueError('Repeated/reordered camera frame')
        if any(m['calibration'] != meta[0]['calibration'] for m in meta):
            raise ValueError('Camera parameters changed')
        shift = max(float(np.max(np.linalg.norm(x-points[0], axis=1))) for x in points)
        if shift > .75:
            raise ValueError(f'Board not stable enough: maximum corner shift {shift:.3f}px >0.75px')
        observer.close(); observer = None
        xy = np.mean(points, axis=0)
        hull = cv2.convexHull(xy.astype(np.float32))
        paths = ['feedback.jsonl'] + [f'{a}_{i}.{b}' for i in range(4) for a,b in [('color','jpg'),('camera','json')]]
        result.update(complete=True, observation=observation, frame_indices=[m['frame']['index'] for m in meta],
            camera_serial=board['camera_serial'], camera_calibration=meta[0]['calibration'], request_windows=windows,
            max_corner_shift_px=shift, corners_sb=np.array(points).tolist(), corners_classic=classic_points,
            mean_corners_sb=xy.tolist(), center_px=xy.mean(axis=0).tolist(),
            corner_bounds_px=[xy.min(axis=0).tolist(), xy.max(axis=0).tolist()],
            inner_grid_image_area_fraction=float(cv2.contourArea(hull)/(640*480)),
            raw_sha256={name:helper.digest(out/name) for name in paths})
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if observer is not None:
            observer.close()
        live.save(out/'capture.json', result)
    print(json.dumps({k:v for k,v in result.items() if k not in
        ['corners_sb','corners_classic','mean_corners_sb','camera_calibration','raw_sha256']}, ensure_ascii=False, indent=2))
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
