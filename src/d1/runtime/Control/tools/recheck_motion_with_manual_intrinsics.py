#!/usr/bin/env python3
"""Replay saved robot/camera evidence using separately fitted, frozen intrinsics.

Offline only: no device access, motion, mapping edits or runtime installation.
Training leave-one-view-out parameters are sensitivity checks, not new fits or
confidence intervals. All independent validation views remain untouched.
"""
import argparse
import datetime
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import diagnose_handeye_geometry as geometry
import kr260_handeye as helper
import kr260_handeye_live as live
from review_joint_mapping import URDF, fk, fit_rotation


LOGS = Path(__file__).resolve().parents[1] / 'logs'
GROUPS = {
    'midnight': ('handeye_motion_20261022', [
        'baseline', 'pose_j4_plus2', 'pose_j5_plus2_retry',
        'pose_j4_second', 'pose_j4_return_first', 'final_capture']),
    'day': ('relative_axes_20261022_1252', [
        'baseline_after_j1_recovery', 'pose_j3_plus2',
        'pose_j2_interrupted_retry', 'baseline_after_j4_hold',
        'pose_j2_after_hold_retry', 'pose_j0_plus2']),
}
# Explicit pairs preserve the fixed-board sessions and the interrupted segment.
PAIRS = [
    ('J4 first', 'midnight', 0, 1, 4, True),
    ('J5', 'midnight', 1, 2, 5, True),
    ('J4 second', 'midnight', 2, 3, 4, True),
    ('J4 return', 'midnight', 3, 4, 4, True),
    ('J4/J5 final return', 'midnight', 4, 5, None, False),
    ('J3', 'day', 0, 1, 3, True),
    ('J2 interrupted', 'day', 1, 2, 2, False),
    ('J4 hold', 'day', 2, 3, None, False),
    ('J2', 'day', 3, 4, 2, True),
    ('J0', 'day', 4, 5, 0, True),
]


def angle(r):
    return float(np.rad2deg(Rotation.from_matrix(r).magnitude()))


def params(row):
    return np.array(row['camera_matrix'], float), np.array(row['distortion'], float)


def estimate(cap, obj, k, d, detector):
    rotations, residuals = [], []
    for frame in cap['frames']:
        r, t, rv = geometry.solve(obj, frame[detector], k, d, 'iterative')
        if np.min((obj @ r.T + t.ravel())[:, 2]) <= 0:
            raise ValueError('Nonpositive board depth')
        xy = cv2.projectPoints(obj, rv, t, k, d)[0].reshape(-1, 2)
        residuals.append(np.linalg.norm(xy-frame[detector], axis=1))
        rotations.append(r)
    rr = Rotation.from_matrix(rotations)
    mean = rr.mean()
    return dict(rotation=mean.as_matrix().tolist(),
                per_frame_rotation=np.array(rotations).tolist(),
                rms_px=float(np.sqrt(np.mean(np.array(residuals)**2))),
                within_capture_max_deg=float(np.rad2deg((mean.inv()*rr).magnitude()).max()))


def direction_check(rows, variants, pair_rotations):
    chosen = [next(r for r in rows if r['label'] == label) for label in ('J3', 'J2', 'J0')]
    aa = [np.array(r['nominal_relative_rotation']) for r in chosen]
    av = Rotation.from_matrix(aa).as_rotvec()
    result = {}
    for variant in variants:
        bb = [pair_rotations[r['label']][variant] for r in chosen]
        bv = Rotation.from_matrix(bb).as_rotvec()
        comparisons = []
        for i, j in ((0, 1), (0, 2), (1, 2)):
            def separation(v):
                return float(np.rad2deg(np.arccos(np.clip(np.dot(v[i], v[j]) /
                    (np.linalg.norm(v[i])*np.linalg.norm(v[j])), -1, 1))))
            comparisons.append(dict(axes=[chosen[i]['label'], chosen[j]['label']],
                nominal_signed_axis_separation_deg=separation(av),
                visual_signed_axis_separation_deg=separation(bv)))
        heldout = []
        for i in range(3):
            train = [j for j in range(3) if j != i]
            x, singular = fit_rotation(av[train], bv[train])
            heldout.append(dict(heldout_axis=chosen[i]['label'],
                residual_deg=angle(aa[i].T @ x @ bb[i] @ x.T),
                covariance_singular_values=singular.tolist()))
        result[variant] = dict(signed_axis_comparisons=comparisons,
                              rotation_only_leave_one_axis_out=heldout)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Fresh output directory required')
    protocol = json.loads((args.root/'validation_protocol.json').read_text())
    candidate_path = args.root/protocol['candidate_file']
    if helper.digest(candidate_path) != protocol['candidate_sha256']:
        raise ValueError('Frozen candidate changed')
    candidate = json.loads(candidate_path.read_text())
    validation = json.loads((args.root/'independent_validation/validation.json').read_text())
    if (validation['candidate_sha256'] != helper.digest(candidate_path) or
        validation['parameters_refitted'] or validation['independent_view_count'] < 5):
        raise ValueError('Independent validation is incomplete or incompatible')
    if candidate['validation_views_used_for_fitting_or_model_selection']:
        raise ValueError('Validation leaked into training')
    if helper.digest(URDF) != '0c9d6fb9664953cc5cbd1b28d0210b63546f0f7a27ae016e87a99eaad7fd886f':
        raise ValueError('Nominal robot model changed')
    if helper.digest(live.DEFAULT_BOARD) != candidate['board_sha256']:
        raise ValueError('Board model changed')
    board = json.loads(live.DEFAULT_BOARD.read_text())
    _, obj = live.geometry(board)
    models = {'original_sdk': params(candidate['original_sdk_calibration']),
              'frozen_candidate': params(candidate['selected_candidate'])}
    cv_models = next(m['training_leave_one_view_out'] for m in candidate['training_diagnostics']
                     if m['model'] == candidate['selected_model'])
    for i, entry in enumerate(cv_models):
        models[f'train_loo_{i:02d}'] = params(entry)
    captures, poses = {}, {}
    train_points = np.concatenate([np.array(json.loads((args.root/r['folder']/'capture.json').read_text())
                                            ['mean_corners_sb']) for r in candidate['training_views']])
    hull = cv2.convexHull(train_points.astype(np.float32))
    for group, (session, folders) in GROUPS.items():
        for i, folder in enumerate(folders):
            cap = geometry.load_capture(LOGS/session/folder, board)
            for frame in cap['frames']:
                if (frame['meta']['device']['serial'] != candidate['camera_serial'] or
                    frame['meta']['calibration'] != candidate['original_sdk_calibration']):
                    raise ValueError('Camera identity/geometry differs from independent session')
            xy = np.mean([f['sb'] for f in cap['frames']], axis=0)
            cap['corner_bounds_px'] = [xy.min(axis=0).tolist(), xy.max(axis=0).tolist()]
            cap['corners_inside_training_convex_hull'] = sum(
                cv2.pointPolygonTest(hull, tuple(map(float, p)), False) >= 0 for p in xy)
            captures[group, i] = cap
            poses[group, i] = {}
            for name, (k, d) in models.items():
                for detector in (('sb', 'classic') if not name.startswith('train_loo_') else ('sb',)):
                    poses[group, i][f'{name}/{detector}'] = estimate(cap, obj, k, d, detector)
    rows, pair_rotations = [], {}
    for label, group, i, j, target, primary in PAIRS:
        before, after = captures[group, i], captures[group, j]
        q0, q1 = np.array(before['q']), np.array(after['q'])
        dq = q1-q0
        a = fk(np.deg2rad(q0[:6]))[:3,:3].T @ fk(np.deg2rad(q1[:6]))[:3,:3]
        variants, rotations = {}, {}
        for variant in poses[group, i]:
            p0, p1 = poses[group, i][variant], poses[group, j][variant]
            b = np.array(p0['rotation']) @ np.array(p1['rotation']).T
            rotations[variant] = b
            variants[variant] = dict(rotation_deg=angle(b), visual_minus_nominal_deg=angle(b)-angle(a),
                within_capture_max_deg=[p0['within_capture_max_deg'], p1['within_capture_max_deg']],
                fit_rms_px=[p0['rms_px'], p1['rms_px']],
                repeat_frame_pair_angle_range_deg=[
                    min(angle(np.array(r0)@np.array(r1).T) for r0 in p0['per_frame_rotation'] for r1 in p1['per_frame_rotation']),
                    max(angle(np.array(r0)@np.array(r1).T) for r0 in p0['per_frame_rotation'] for r1 in p1['per_frame_rotation'])])
        row = dict(label=label, primary=primary, fixed_board_session=group,
                   before=before['folder'], after=after['folder'], feedback_delta_deg=dq.tolist(),
                   nominal_all_joint_rotation_deg=angle(a), nominal_relative_rotation=a.tolist(),
                   visual=variants)
        if target is not None:
            other = float(np.sum(np.abs(dq[:6]))-abs(dq[target]))
            row.update(target_joint=target, target_feedback_delta_deg=float(dq[target]),
                       non_target_absolute_sum_deg=other,
                       angle_bounds_from_feedback_triangle_inequality_deg=[max(0, float(abs(dq[target])-other)),
                                                                          min(180, float(abs(dq[target])+other))])
        loo = [v['rotation_deg'] for k,v in variants.items() if k.startswith('train_loo_')]
        row['training_leave_one_out_intrinsic_sensitivity_range_deg'] = [min(loo), max(loo)]
        row['frozen_candidate_visual_over_nominal_percent'] = (
            100*variants['frozen_candidate/sb']['rotation_deg']/angle(a) if angle(a) > 1e-9 else None)
        rows.append(row); pair_rotations[label] = rotations
    # Exact regression against the previously saved SDK/SB analysis prevents
    # accidentally changing the pair convention or detector in this comparison.
    prior = json.loads((LOGS/'relative_axes_20261022_1252/multi_axis_audit.json').read_text())
    for old in prior['rows']:
        new = next(r for r in rows if r['label'] == old['axis'])
        if abs(old['visual_sb_rotation_deg']-new['visual']['original_sdk/sb']['rotation_deg']) > 1e-6:
            raise ValueError('Original SDK analysis does not reproduce')
    directions = direction_check(rows, ('original_sdk/sb','frozen_candidate/sb',
                                       'original_sdk/classic','frozen_candidate/classic'), pair_rotations)
    report = dict(schema='motion-recheck-independent-intrinsics-v1',
        created_at=datetime.datetime.now().astimezone().isoformat(), diagnostic_only=True,
        robot_commands_sent=False, runtime_calibration_written=False, mapping_validated=False,
        handeye_calibrated=False, camera_parameters_refitted=False, original_analysis_reproduced=True,
        camera_serial=candidate['camera_serial'], candidate_sha256=helper.digest(candidate_path),
        validation_sha256=helper.digest(args.root/'independent_validation/validation.json'),
        board_sha256=helper.digest(live.DEFAULT_BOARD), urdf_sha256=helper.digest(URDF),
        script_sha256=helper.digest(__file__), opencv_version=cv2.__version__,
        source_scripts_sha256={str(p):helper.digest(p) for p in
            [Path(geometry.__file__), Path(helper.__file__), Path(live.__file__),
             Path(__file__).with_name('review_joint_mapping.py')]},
        nominal_mapping='SDK J0..J5 -> URDF Joint1..Joint6, signs +1 and offsets 0; unverified hypothesis.',
        observations=[dict(group=g, index=i, **{k:v for k,v in cap.items() if k!='frames'},
                           poses=poses[g,i]) for (g,i),cap in captures.items()],
        pairs=rows, direction_diagnostics=directions,
        limitations=[
            'SB and classic use the same images; their agreement is not independent angular metrology.',
            'Leave-one-training-view-out range is sensitivity, not a confidence interval; shared board errors remain.',
            'Feedback bounds omit encoder rounding, unknown scale and physical transmission error; no measured angular accuracy claim.',
            'Non-target changes are included in nominal FK. Unknown signs/offsets remain unvalidated.',
            'Manual board placements are for intrinsics only and are not added to fixed-board hand-eye observations.',
            'Rotation-only held-out-axis fits are diagnostics, not formal hand-eye calibration.',
            'Lower image regions outside observed training coverage remain unvalidated.'])
    args.output.mkdir(parents=True)
    live.save(args.output/'motion_recheck.json', report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    primary = [r for r in rows if r['primary']]
    fig, ax = plt.subplots(figsize=(10,4.7), constrained_layout=True)
    x = np.arange(len(primary))
    ax.plot(x, [r['nominal_all_joint_rotation_deg'] for r in primary], 'ks-', label='Nominal FK (all reported joints)')
    for key, label, style in [('original_sdk/sb','Original SDK / SB','o--'),
                              ('frozen_candidate/sb','Independent intrinsics / SB','o-'),
                              ('frozen_candidate/classic','Independent intrinsics / classic','x--')]:
        ax.plot(x, [r['visual'][key]['rotation_deg'] for r in primary], style, label=label)
    ax.set(xticks=x, xticklabels=[r['label'] for r in primary], ylabel='Relative rotation (degrees)',
           title='Recorded motions replayed; no new robot movement')
    ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.savefig(args.output/'motion_recheck.png', dpi=160); plt.close(fig)
    print(json.dumps([dict(label=r['label'], nominal_deg=r['nominal_all_joint_rotation_deg'],
                          sdk_sb_deg=r['visual']['original_sdk/sb']['rotation_deg'],
                          candidate_sb_deg=r['visual']['frozen_candidate/sb']['rotation_deg'],
                          candidate_classic_deg=r['visual']['frozen_candidate/classic']['rotation_deg'],
                          loo_range_deg=r['training_leave_one_out_intrinsic_sensitivity_range_deg'])
                     for r in primary], indent=2))


if __name__ == '__main__':
    main()
