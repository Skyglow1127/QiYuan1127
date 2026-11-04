#!/usr/bin/env python3
"""Offline diagnostics only. No device access, motion or calibration writes.

Replays source hashes and feedback gates, then compares corner detectors, PnP
solvers, spatial subsets and direct inter-image homographies. These methods
share images/intrinsics; agreement is not independent absolute metrology.
"""
import argparse
import base64
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import kr260_handeye as helper
import kr260_handeye_live as live


def angle(a, b):
    return float(np.rad2deg(Rotation.from_matrix(a @ b.T).magnitude()))


def solve(obj, corners, k, d, method):
    obj, corners = np.ascontiguousarray(obj), np.ascontiguousarray(corners)
    if method.startswith('ippe'):
        count, rvs, tvs, _ = cv2.solvePnPGeneric(obj, corners, k, d, flags=cv2.SOLVEPNP_IPPE)
        if not count:
            raise ValueError('IPPE failed')
        candidates = []
        for rv, tv in zip(rvs, tvs):
            if np.min((obj @ cv2.Rodrigues(rv)[0].T + tv.ravel())[:, 2]) <= 0:
                continue
            err = cv2.projectPoints(obj, rv, tv, k, d)[0].reshape(-1, 2) - corners.reshape(-1, 2)
            candidates.append((float(np.mean(err ** 2)), rv, tv))
        candidates.sort(key=lambda v: v[0])
        if not candidates:
            raise ValueError('No positive-depth IPPE solution')
        _, rv, tv = candidates[0]
        if method == 'ippe_lm':
            rv, tv = cv2.solvePnPRefineLM(obj, corners, k, d, rv, tv)
    else:
        flag = cv2.SOLVEPNP_ITERATIVE if method == 'iterative' else cv2.SOLVEPNP_SQPNP
        ok, rv, tv = cv2.solvePnP(obj, corners, k, d, flags=flag)
        if not ok:
            raise ValueError('PnP failed')
    return cv2.Rodrigues(rv)[0], tv, rv


def load_capture(folder, board, require_classic=True, stationarity_assessor=None):
    record = json.loads((folder / 'capture.json').read_text())
    if not record.get('complete') or record['board_sha256'] != helper.digest(live.DEFAULT_BOARD):
        raise ValueError('Incomplete/incompatible capture: ' + str(folder))
    expected = {'feedback.jsonl'} | {f'{a}_{i}.{b}' for i in range(4)
                                   for a, b in [('color', 'jpg'), ('camera', 'json')]}
    if set(record['raw_sha256']) != expected:
        raise ValueError('Missing evidence: ' + str(folder))
    for name, sha in record['raw_sha256'].items():
        if helper.digest(folder / name) != sha:
            raise ValueError('Changed evidence: ' + str(folder / name))
    events = [helper.parse_feedback(r['line'], r['t']) for r in
              map(json.loads, (folder / 'feedback.jsonl').read_text().splitlines())]
    obs = record['observation']
    assessor = stationarity_assessor or helper.assess_window
    replay = assessor([e for e in events if e], obs['begin_aipc_monotonic_s'],
                      obs['end_aipc_monotonic_s'])
    if not np.allclose(replay['joint_median'], obs['joint_median'], atol=1e-8, rtol=0):
        raise ValueError('Stored median differs from replay')
    frames = []
    for i in range(4):
        meta = json.loads((folder / f'camera_{i}.json').read_text())
        raw = (folder / f'color_{i}.jpg').read_bytes()
        item = live.inspect(dict(meta, image_base64=base64.b64encode(raw).decode()), board)
        gray = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        ok, classic = cv2.findChessboardCorners(gray, (board['inner_columns'], board['inner_rows']),
                                               cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        sb = item['corners'].reshape(-1, 2).astype(float)
        classic_error = None
        if not ok:
            classic_error = 'Classic detector failed: ' + str(folder)
        else:
            classic = cv2.cornerSubPix(gray, classic, (5, 5), (-1, -1),
                                      (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, .001))
            classic = classic.reshape(-1, 2).astype(float)
            if np.linalg.norm(classic[::-1] - sb) < np.linalg.norm(classic - sb):
                classic = classic[::-1].copy()
            if np.max(np.linalg.norm(classic - sb, axis=1)) > 2:
                classic_error = 'Unresolved detector indexing'
        if classic_error:
            if require_classic:
                raise ValueError(classic_error)
            classic = None
        frames.append(dict(sb=sb, classic=classic, classic_error=classic_error,
                           k=np.array(meta['calibration']['camera_matrix'], float),
                           d=np.array(meta['calibration']['distortion'], float), meta=meta))
    if any(f['meta']['calibration'] != frames[0]['meta']['calibration'] for f in frames):
        raise ValueError('Changing calibration')
    return dict(folder=str(folder.resolve()), q=replay['joint_median'], feedback_replay=replay,
                capture_sha256=helper.digest(folder / 'capture.json'), frames=frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('sessions', type=Path, nargs='+')
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('New output directory required')
    board = json.loads(live.DEFAULT_BOARD.read_text())
    pattern, obj = live.geometry(board)
    captures = [load_capture(p, board) for p in args.sessions]
    if any(c['frames'][0]['meta']['calibration'] != captures[0]['frames'][0]['meta']['calibration'] for c in captures):
        raise ValueError('Different intrinsics across captures')
    cols, rows = pattern
    xx, yy = np.meshgrid(np.arange(cols), np.arange(rows))
    masks = dict(full=np.ones(len(obj), bool), left=xx.ravel() < 4, right=xx.ravel() >= cols-4,
                 top=yy.ravel() < 5, bottom=yy.ravel() >= rows-5,
                 center=((xx.ravel() > 0) & (xx.ravel() < cols-1) & (yy.ravel() > 0) & (yy.ravel() < rows-1)))
    variants = [(det, method, 'full') for det in ('sb', 'classic')
                for method in ('iterative', 'ippe', 'ippe_lm', 'sqpnp')]
    variants += [('sb', 'iterative', name) for name in masks if name != 'full']
    results = {}
    for det, method, region in variants:
        key = '/'.join((det, method, region))
        results[key] = []
        mask = masks[region]
        for cap in captures:
            rotations, translations, residuals, heldout = [], [], [], []
            for f in cap['frames']:
                r, t, rv = solve(obj[mask], f[det][mask], f['k'], f['d'], method)
                residual = cv2.projectPoints(obj, rv, t, f['k'], f['d'])[0].reshape(-1, 2) - f[det]
                rotations.append(r); translations.append(t.ravel()); residuals.append(residual)
                if not mask.all():
                    heldout.append(float(np.sqrt(np.mean(np.sum(residual[~mask]**2, axis=1)))))
            rot = Rotation.from_matrix(rotations)
            mean = rot.mean()
            results[key].append(dict(rotation=mean.as_matrix().tolist(),
                                     translation_m=np.mean(translations, axis=0).tolist(),
                                     within_capture_max_deg=float(np.rad2deg((mean.inv()*rot).magnitude()).max()),
                                     fit_rms_px=float(np.sqrt(np.mean(np.sum(np.array(residuals)[:, mask]**2, axis=2)))),
                                     withheld_rms_px=float(np.mean(heldout)) if heldout else None,
                                     mean_residual_px=np.mean(residuals, axis=0).tolist()))
    pairs = []
    for i in range(1, len(captures)):
        a, b = captures[i-1:i+1]
        dq = np.array(b['q'])[:6] - np.array(a['q'])[:6]
        dominant = int(np.argmax(np.abs(dq)))
        angles = {key: angle(np.array(values[i-1]['rotation']), np.array(values[i]['rotation']))
                  for key, values in results.items()}
        # Direct homography uses corresponding pixels and K/D; no square-size or
        # regular-grid geometry. Its planar-scene and camera-model assumptions remain.
        pa, pb = [np.mean([f['sb'] for f in c['frames']], axis=0) for c in (a, b)]
        k, d = a['frames'][0]['k'], a['frames'][0]['d']
        ua, ub = [cv2.undistortPoints(p.reshape(-1, 1, 2), k, d).reshape(-1, 2) for p in (pa, pb)]
        h, _ = cv2.findHomography(ua, ub, method=0)
        _, rotations, translations, normals = cv2.decomposeHomographyMat(h, np.eye(3))
        solutions = [dict(angle_deg=angle(r, np.eye(3)), normal=n.ravel().tolist())
                     for r, n in zip(rotations, normals)]
        reference_normal = np.array(results['sb/iterative/full'][i-1]['rotation'])[:, 2]
        for solution in solutions:
            solution['normal_difference_from_pnp_deg'] = float(np.rad2deg(np.arccos(
                np.clip(abs(np.dot(reference_normal, solution['normal'])), 0, 1))))
        # Report all branches; do not choose the desired encoder angle.
        reproj = cv2.perspectiveTransform(ua.reshape(-1, 1, 2), h).reshape(-1, 2)
        norm_rms = float(np.sqrt(np.mean(np.sum((reproj-ub)**2, axis=1))))
        pairs.append(dict(before=Path(a['folder']).name, after=Path(b['folder']).name,
                          dq_deg=dq.tolist(), dominant_joint=dominant,
                          dominant_abs_deg=float(abs(dq[dominant])),
                          other_reported_absolute_sum_deg=float(np.sum(abs(dq))-abs(dq[dominant])),
                          visual_angles_deg=angles, homography_solutions=solutions,
                          homography_normalized_rms=norm_rms))
    f = captures[0]['frames'][0]
    r0, t0, _ = solve(obj, f['sb'], f['k'], f['d'], 'iterative')
    scale_check = []
    for scale in (.5, 2):
        r, t, _ = solve(obj*scale, f['sb'], f['k'], f['d'], 'iterative')
        scale_check.append(dict(square_scale=scale, rotation_change_deg=angle(r0, r),
                                translation_scale=float(np.linalg.norm(t)/np.linalg.norm(t0))))
    residuals = np.array([v['mean_residual_px'] for v in results['sb/iterative/full']])
    all_rotations = [np.array(v['rotation']) for v in results['sb/iterative/full']]
    report = dict(schema='d1-offline-geometry-diagnostic-v1', diagnostic_only=True,
                  robot_commands_sent=False, calibration_written=False,
                  opencv_version=cv2.__version__, board_sha256=helper.digest(live.DEFAULT_BOARD),
                  script_sha256=helper.digest(__file__),
                  sessions=[{k:v for k,v in c.items() if k!='frames'} for c in captures],
                  pairs=pairs, variants=results, uniform_square_scale_check=scale_check,
                  maximum_observed_orientation_separation_deg=max(angle(a, b) for a in all_rotations for b in all_rotations),
                  residual_correlation=np.corrcoef(residuals.reshape(len(captures), -1)).tolist(),
                  limitations=['Shared images and intrinsics: not independent absolute validation.',
                               'Subsets change conditioning; their spread cannot identify board shape alone.',
                               'Homography has multiple physical branches and assumes a fixed plane.',
                               'Feedback magnitudes are reported values, not externally measured rotations.',
                               'Diagnostic does not approve physical board origin or formal export.'])
    args.output.mkdir(parents=True)
    (args.output/'diagnostic.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    primary = pairs[:3]
    x = np.arange(len(primary))
    ax[0].plot(x, [p['dominant_abs_deg'] for p in primary], 'ks-', label='Reported dominant joint')
    for key in ('sb/iterative/full', 'classic/iterative/full', 'sb/ippe/full', 'sb/sqpnp/full'):
        ax[0].plot(x, [p['visual_angles_deg'][key] for p in primary], 'o--', label=key)
    ax[0].set(xticks=x, xticklabels=[f"J{p['dominant_joint']} step {i+1}" for i,p in enumerate(primary)],
              ylabel='Relative rotation (deg)', title='Same images: solver/detector cross-check')
    ax[0].legend(fontsize=7); ax[0].grid(alpha=.2)
    res = residuals[0]
    pixels = np.mean([f['sb'] for f in captures[0]['frames']], axis=0)
    ax[1].quiver(pixels[:,0], pixels[:,1], res[:,0], res[:,1], angles='xy', scale_units='xy', scale=.01)
    ax[1].set(xlabel='Image u (pixel)', ylabel='Image v (pixel)',
              title='Baseline image residuals (arrows magnified 100x)', aspect='equal')
    ax[1].set_xlim(float(pixels[:,0].min()-40), float(pixels[:,0].max()+40))
    ax[1].set_ylim(float(pixels[:,1].min()-40), float(pixels[:,1].max()+40))
    ax[1].invert_yaxis();ax[1].grid(alpha=.2)
    fig.savefig(args.output/'geometry_diagnostic.png', dpi=160);plt.close(fig)
    print(json.dumps(pairs, indent=2))


if __name__ == '__main__':
    main()
