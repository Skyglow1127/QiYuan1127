#!/usr/bin/env python3
"""Fit/freeze an intrinsic candidate from TRAIN views only, never deploy it.

Model selection uses leave-one-view-out training errors. Validation images
must be evaluated later against this frozen file without refitting.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import kr260_handeye as helper
import kr260_handeye_live as live
from summarize_manual_intrinsics import summarize


def errors(obj, images, k, d):
    result = []
    for xy in images:
        ok, rv, tv = cv2.solvePnP(obj, xy, k, d)
        if not ok:
            raise ValueError('PnP failed')
        projected = cv2.projectPoints(obj, rv, tv, k, d)[0].reshape(-1, 2)
        result.append(float(np.sqrt(np.mean(np.sum((projected-xy)**2, axis=1)))))
    return result


def fit(obj, images, k0, d0, model, scale=1.):
    if model == 'sdk_fixed':
        return dict(camera_matrix=k0.tolist(), distortion=d0.ravel().tolist(),
                    rms_px=None, linearized_standard_deviations=None)
    flags = cv2.CALIB_USE_INTRINSIC_GUESS
    if model == 'brown4':
        flags |= cv2.CALIB_FIX_K3
    elif model != 'brown5':
        raise ValueError(model)
    k = k0.copy(); k[0,0] *= scale; k[1,1] *= scale
    d = d0[:5].copy()
    if model == 'brown4':
        d[4] = 0
    rms, k, d, _, _, sd, _, per_view = cv2.calibrateCameraExtended(
        [obj.astype(np.float32) for _ in images],
        [xy.astype(np.float32) for xy in images], (640, 480), k, d,
        flags=flags, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 200, 1e-10))
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('Invalid fitted parameters')
    return dict(camera_matrix=k.tolist(), distortion=d.ravel().tolist(), rms_px=float(rms),
                per_training_view_rms_px=per_view.ravel().tolist(),
                linearized_standard_deviations=sd.ravel().tolist())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('New output file required')
    coverage = summarize(args.root)
    selected = [x for x in coverage['accepted'] if x['split'] == 'train']
    if len(selected) < 15:
        raise ValueError('At least 15 independently positioned training views required')
    records = [json.loads((args.root/x['folder']/'capture.json').read_text()) for x in selected]
    board = json.loads(live.DEFAULT_BOARD.read_text())
    _, obj = live.geometry(board)
    cal = records[0]['camera_calibration']
    k0 = np.array(cal['camera_matrix'], dtype=float)
    d0 = np.array(cal['distortion'], dtype=float)
    if np.any(d0[5:] != 0):
        raise ValueError('Nonzero rational SDK coefficients require explicit model review')
    images = [np.array(r['mean_corners_sb'], dtype=float) for r in records]
    runs = []
    for model in ['sdk_fixed', 'brown4', 'brown5']:
        full = fit(obj, images, k0, d0, model)
        folds = []
        for i in range(len(images)):
            train = [xy for j,xy in enumerate(images) if j != i]
            fitted = fit(obj, train, k0, d0, model)
            err = errors(obj, [images[i]], np.array(fitted['camera_matrix']),
                         np.array(fitted['distortion']))[0]
            folds.append(dict(heldout_training_folder=selected[i]['folder'], rms_px=err,
                              camera_matrix=fitted['camera_matrix'], distortion=fitted['distortion']))
        ee = np.array([f['rms_px'] for f in folds])
        runs.append(dict(model=model, full_training_fit=full, training_leave_one_view_out=folds,
                         cv_mean_rms_px=float(ee.mean()), cv_max_rms_px=float(ee.max()),
                         cv_mean_standard_error_px=float(ee.std(ddof=1)/np.sqrt(len(ee)))))
    best = min(runs, key=lambda r:r['cv_mean_rms_px'])
    # One-standard-error preference for the simpler camera model.
    cutoff = best['cv_mean_rms_px']+best['cv_mean_standard_error_px']
    chosen = next(r for r in runs if r['cv_mean_rms_px'] <= cutoff)
    multi_start = []
    if chosen['model'] != 'sdk_fixed':
        for scale in [.8, 1.2]:
            multi_start.append(dict(initial_focal_scale=scale,
                result=fit(obj, images, k0, d0, chosen['model'], scale)))
    classic_fit = None
    if all(all(x is not None for x in r['corners_classic']) for r in records):
        classic_images = [np.mean(np.array(r['corners_classic']), axis=0) for r in records]
        classic_fit = fit(obj, classic_images, k0, d0, chosen['model'])
    result = dict(schema='manual-intrinsic-frozen-training-candidate-v1',
        robot_commands_sent=False, runtime_calibration_written=False,
        independent_validation_complete=False, handeye_calibrated=False,
        camera_serial=records[0]['camera_serial'], resolution=[640,480],
        board_sha256=helper.digest(live.DEFAULT_BOARD), script_sha256=helper.digest(__file__),
        training_views=[dict(folder=r['folder'], capture_sha256=r['capture_sha256']) for r in selected],
        validation_views_used_for_fitting_or_model_selection=[],
        model_selection='Training-only leave-one-view-out mean reprojection; prefer simpler model within one standard error of best.',
        selected_model=chosen['model'], selected_candidate=chosen['full_training_fit'],
        original_sdk_calibration=cal, training_diagnostics=runs,
        selected_model_initialization_sensitivity=multi_start,
        alternate_detector_training_fit=classic_fit,
        limitations=['Independent held-out board views still required.',
                     'Reprojection refits board pose; it is not external angular ground truth.',
                     'Linearized uncertainty assumes a flat rigid board and the specified camera model.',
                     'This freezes a diagnostic candidate, never authorizes installation or hand-eye export.'])
    live.save(args.output, result)
    print(json.dumps(dict(output=str(args.output), selected_model=chosen['model'],
                         candidate=chosen['full_training_fit']), indent=2))


if __name__ == '__main__':
    main()
