#!/usr/bin/env python3
"""Evaluate a frozen intrinsic candidate on independent views, never refit."""
import argparse
import datetime
import json
from pathlib import Path

import cv2
import numpy as np
import kr260_handeye as helper
import kr260_handeye_live as live
from summarize_manual_intrinsics import summarize


def residual(obj, xy, k, d):
    ok, rv, tv = cv2.solvePnP(obj, xy, k, d)
    if not ok or np.min((obj @ cv2.Rodrigues(rv)[0].T + tv.ravel())[:, 2]) <= 0:
        raise ValueError('Invalid validation pose')
    projected = cv2.projectPoints(obj, rv, tv, k, d)[0].reshape(-1, 2)
    e = np.linalg.norm(projected - xy, axis=1)
    return dict(rms_px=float(np.sqrt(np.mean(e**2))), max_px=float(e.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Fresh output directory required')
    root = args.root
    protocol = json.loads((root/'validation_protocol.json').read_text())
    candidate_path = root/protocol['candidate_file']
    if helper.digest(candidate_path) != protocol['candidate_sha256']:
        raise ValueError('Candidate changed after freezing')
    candidate = json.loads(candidate_path.read_text())
    if candidate['validation_views_used_for_fitting_or_model_selection']:
        raise ValueError('Validation data leaked into training')
    coverage = summarize(root)
    validation = [r for r in coverage['accepted'] if r['split'] == 'validation']
    if len(validation) < protocol['required_validation_views']:
        raise ValueError('Need at least five independently positioned validation views')
    training_names = set()
    training_images = set()
    for entry in candidate['training_views']:
        p = root/entry['folder']/'capture.json'
        if helper.digest(p) != entry['capture_sha256']:
            raise ValueError('Training evidence changed')
        record = json.loads(p.read_text())
        if record['split'] != 'train':
            raise ValueError('Invalid frozen training split')
        training_names.add(entry['folder'])
        training_images.update(sha for name, sha in record['raw_sha256'].items() if name.endswith('.jpg'))
    board = json.loads(live.DEFAULT_BOARD.read_text())
    if candidate['board_sha256'] != helper.digest(live.DEFAULT_BOARD):
        raise ValueError('Board configuration changed')
    _, obj = live.geometry(board)
    original = candidate['original_sdk_calibration']
    fitted = candidate['selected_candidate']
    models = {'original_sdk': (np.array(original['camera_matrix']), np.array(original['distortion'])),
              'frozen_candidate': (np.array(fitted['camera_matrix']), np.array(fitted['distortion']))}
    rows = []
    all_validation_points = []
    for view in validation:
        if view['folder'] in training_names:
            raise ValueError('Overlapping training and validation folders')
        rec = json.loads((root/view['folder']/'capture.json').read_text())
        if rec['camera_serial'] != candidate['camera_serial'] or rec['camera_calibration'] != original:
            raise ValueError('Camera identity or original parameters changed')
        if any(sha in training_images for name, sha in rec['raw_sha256'].items() if name.endswith('.jpg')):
            raise ValueError('Reused training image in validation')
        xy = np.array(rec['mean_corners_sb'], dtype=float)
        all_validation_points.append(xy)
        row = dict(folder=view['folder'], capture_sha256=view['capture_sha256'],
                   center_px=rec['center_px'], corner_bounds_px=rec['corner_bounds_px'],
                   max_corner_shift_px=rec['max_corner_shift_px'], mean_frame_sb={})
        for name, (k, d) in models.items():
            row['mean_frame_sb'][name] = residual(obj, xy, k, d)
        row['per_frame_sb_rms_px'] = {name:[residual(obj, np.array(p, dtype=float), k, d)['rms_px']
            for p in rec['corners_sb']] for name, (k, d) in models.items()}
        if all(p is not None for p in rec['corners_classic']):
            classic = np.mean(np.array(rec['corners_classic']), axis=0)
            row['mean_frame_classic'] = {name:residual(obj, classic, k, d)
                                         for name, (k, d) in models.items()}
        rows.append(row)
    totals = {}
    for name in models:
        values = np.array([r['mean_frame_sb'][name]['rms_px'] for r in rows])
        totals[name] = dict(mean_view_rms_px=float(values.mean()),
                            pooled_corner_rms_px=float(np.sqrt(np.mean(values**2))),
                            max_view_rms_px=float(values.max()))
    old = totals['original_sdk']['mean_view_rms_px']
    new = totals['frozen_candidate']['mean_view_rms_px']
    report = dict(schema='manual-intrinsic-independent-validation-v1',
        evaluated_at=datetime.datetime.now().astimezone().isoformat(),
        candidate_file=str(candidate_path.resolve()), candidate_sha256=helper.digest(candidate_path),
        validation_protocol_sha256=helper.digest(root/'validation_protocol.json'),
        evaluation_script_sha256=helper.digest(__file__),
        independent_view_count=len(rows), training_view_count=len(training_names),
        model=candidate['selected_model'], parameters_refitted=False, candidate_changed=False,
        robot_commands_sent=False, runtime_calibration_written=False, handeye_calibrated=False,
        validation_data_collection_complete=True, automatic_robot_control_acceptance=False,
        camera_serial=candidate['camera_serial'], selected_camera_parameters=fitted,
        comparisons=rows, summary=totals,
        mean_view_rms_reduction_percent=float(100*(old-new)/old),
        views_with_lower_candidate_rms=sum(r['mean_frame_sb']['frozen_candidate']['rms_px'] <
                                          r['mean_frame_sb']['original_sdk']['rms_px'] for r in rows),
        training_corner_bounds_px=protocol['training_corner_bounds_px'],
        validation_corner_bounds_px=[np.min(all_validation_points, axis=(0,1)).tolist(),
                                     np.max(all_validation_points, axis=(0,1)).tolist()],
        limitations=['Each validation image fits only its board pose, never the camera parameters.',
                     'Low held-out reprojection does not establish angular ground truth or mechanical joint accuracy.',
                     'All views share one physical board; board flatness/printing errors may be systematic.',
                     'Coverage is limited to observed image regions; do not claim unobserved lower-frame accuracy.',
                     'Robot relative-motion consistency must be re-evaluated before formal hand-eye calibration.'])
    args.output.mkdir(parents=True)
    live.save(args.output/'validation.json', report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11,4), constrained_layout=True)
    x = np.arange(len(rows))
    for i, (name, title) in enumerate([('original_sdk','Original SDK'),('frozen_candidate','Frozen candidate')]):
        ax[0].bar(x + (i-.5)*.35, [r['mean_frame_sb'][name]['rms_px'] for r in rows], .35, label=title)
    ax[0].set(xticks=x, xticklabels=[str(i+1) for i in x], xlabel='Independent validation view',
              ylabel='Reprojection RMS (px)', title='No camera parameter refitting')
    ax[0].legend(); ax[0].grid(axis='y', alpha=.2)
    for name in training_names:
        xy = np.array(json.loads((root/name/'capture.json').read_text())['mean_corners_sb'])
        ax[1].scatter(xy[:,0], xy[:,1], s=4, alpha=.12, color='gray')
    for i, xy in enumerate(all_validation_points):
        ax[1].scatter(xy[:,0], xy[:,1], s=7, label=f'V{i+1}')
    ax[1].set(xlim=(0,640), ylim=(480,0), xlabel='Image u (px)', ylabel='Image v (px)',
              title='Gray: train; color: independent validation', aspect='equal')
    ax[1].legend(fontsize=7)
    fig.savefig(args.output/'validation_reprojection.png', dpi=160)
    plt.close(fig)
    print(json.dumps(dict(summary=totals, reduction_percent=report['mean_view_rms_reduction_percent'],
                         independent_views=len(rows), output=str(args.output)), indent=2))


if __name__ == '__main__':
    main()
