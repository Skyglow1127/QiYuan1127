#!/usr/bin/env python3
"""Offline speed/acceleration comparison on saved paths; no execution option.

Produces aggregate reference timings only, never an executable trajectory.
Existing actuator limits, historical plans and controller files are not changed.
Candidates are mathematical comparisons, not manufacturer ratings or validated
hardware settings. The saved retreat path may only have been partly executed.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

CONTROL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROL/'ros2_ws/src/d1_pick_place'))
from d1_pick_place import path_timing

DEFAULT_SOURCE = CONTROL/'logs/grasp_100mm_resume_20261023_2037'
DEFAULT_OUTPUT = CONTROL/'logs/speed_restart_20261023/offline_speed_comparison'
STAGES = [('approach', 'cycle_execution_v4/00_approach'),
          ('lift', 'cycle_execution_v4/02_lift'),
          ('lower', 'cycle_execution_v4/03_lower'),
          ('retreat', 'cycle_finish_v5/01_retreat')]
BASE_PROFILES = [('v0.5_a1', .5, 1.), ('v1_a1', 1., 1.), ('v2_a1', 2., 1.)]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def analytical_peaks(run):
    if run['shape'] == 'hold':
        return 0., 0.
    if run['shape'] == 'quintic':
        duration, distance = run['duration_s'], run['distance']
        return 1.875*distance/duration, 10/math.sqrt(3)*distance/duration**2
    if run['shape'] == 'cosine_cruise':
        stretch = run['stretch']
        return (run['peak_speed']/stretch,
                math.pi*run['peak_speed']/(2*run['ramp_s']*stretch**2))
    raise ValueError('Unknown timing profile shape')


def compare_saved(source, profiles):
    historical = source['sdk_plan']
    if historical['timing_profile'] != path_timing.PROFILE or historical['rate'] != 5:
        raise ValueError('Expected the recorded collinear_bounded_v1 plan at 5 Hz')
    q = np.asarray(historical['sdk'], float)
    ts = np.asarray(historical['waypoint_times_s'], float)
    if (q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all()
            or ts.shape != (len(q),) or not np.isfinite(ts).all() or abs(ts[0]) > 1e-8
            or np.any(np.diff(ts) <= 0) or abs(ts[-1]-historical['duration_s']) > 1e-7):
        raise ValueError('Invalid saved geometry/timestamps')
    expected_stops = [(r['start_index'], r['end_index']) for r in historical['runs']]
    # Saved timestamps already contain the old retiming. Keeping them as lower
    # bounds would prohibit any speed comparison. Use one common controller
    # interval per edge, while retaining explicit stationary dwell durations.
    edge_floors = np.full(len(q)-1, .2)
    holds = np.max(abs(np.diff(q, axis=0)), axis=1) < 1e-14
    edge_floors[holds] = np.maximum(edge_floors[holds], np.diff(ts)[holds])
    comparison_times = np.r_[0., np.cumsum(edge_floors)]
    baseline = path_timing.retime(q, comparison_times, 5., .5, 1.)
    error = baseline['duration_s']-historical['duration_s']
    if abs(error) > 1e-7:
        raise ValueError('Saved baseline duration is not reproduced; unknown source-time constraints require review')

    rows = []
    for name, speed, acceleration in profiles:
        if not 0 < speed <= 2 or not 0 < acceleration <= 2:
            raise ValueError('Only the explicitly scoped offline candidates are accepted')
        plan = path_timing.retime(q, comparison_times, 5., speed, acceleration)
        boundaries = [(r['start_index'], r['end_index']) for r in plan['runs']]
        if boundaries != expected_stops:
            raise ValueError('Retiming changed a stopped run boundary')
        velocities, accelerations = [], []
        for run in plan['runs']:
            if (not np.array_equal(run['start'], q[run['start_index']])
                    or not np.array_equal(run['end'], q[run['end_index']])):
                raise ValueError('Retiming changed path geometry')
            v, a = analytical_peaks(run)
            velocities.append(v); accelerations.append(a)
            if v > speed+1e-8 or a > acceleration+1e-8:
                raise ValueError('Analytical derivative bound failed')
            if abs(run['duration_s']*5-run['count']) > 1e-8:
                raise ValueError('Run timing is not aligned with 5 Hz ticks')
            for at in (0., run['duration_s']):
                _, velocity, acc = path_timing.run_state(run, at)
                if np.max(abs(velocity)) > 1e-8 or np.max(abs(acc)) > 1e-8:
                    raise ValueError('A retained stop has nonzero endpoint derivatives')
        rows.append(dict(profile=name, rate_hz=5., velocity_bound_deg_s=speed,
            acceleration_bound_deg_s2=acceleration,
            reference_duration_s=plan['duration_s'],
            analytical_peak_velocity_deg_s=max(velocities),
            analytical_peak_acceleration_deg_s2=max(accelerations),
            stopped_runs=len(plan['runs']), internal_stop_count=plan['stopped_corners'],
            continuous_internal_points=plan['continuous_internal_points'],
            reduction_vs_reproduced_baseline_percent=100*(1-plan['duration_s']/baseline['duration_s']),
            geometry_preserved=True, stop_boundaries_preserved=True,
            executable=False, hardware_verified=False))
    return dict(points=len(q), recorded_reference_duration_s=historical['duration_s'],
        reproduced_baseline_duration_s=baseline['duration_s'],
        baseline_reproduction_error_s=error,
        original_time_span_s=float(ts[-1]-ts[0]),
        geometry_sha256=hashlib.sha256(json.dumps(q.tolist(), separators=(',', ':')).encode()).hexdigest(),
        original_times_sha256=hashlib.sha256(json.dumps(ts.tolist(), separators=(',', ':')).encode()).hexdigest(),
        preserved_stationary_edges=int(np.sum(holds)), profiles=rows)


def build_report(source_root, include_acceleration_2=False):
    profiles = BASE_PROFILES + ([('v2_a2', 2., 2.)] if include_acceleration_2 else [])
    rows = []
    for stage, relative in STAGES:
        source = Path(source_root)/relative/'executed/plan.json'
        result_path = source.with_name('result.json')
        recorded_result = json.loads(result_path.read_text())
        row = compare_saved(json.loads(source.read_text()), profiles)
        row.update(stage=stage, source=str(source.resolve()), source_sha256=digest(source),
            recorded_result=str(result_path.resolve()), recorded_result_sha256=digest(result_path),
            recorded_stage_completed=recorded_result.get('completed') is True,
            recorded_execution_error=recorded_result.get('error'))
        rows.append(row)
    totals = []
    for index, (name, speed, acceleration) in enumerate(profiles):
        duration = sum(row['profiles'][index]['reference_duration_s'] for row in rows)
        baseline = sum(row['reproduced_baseline_duration_s'] for row in rows)
        totals.append(dict(profile=name, nominal_arm_reference_subtotal_s=duration,
            reduction_percent=100*(1-duration/baseline),
            scope='Four complete saved arm reference paths; not measured execution or a completed cycle'))
    return dict(schema='d1-offline-speed-comparison-v1', hardware_commands_sent=0,
        hardware_executed=False, executable=False, controller_limits_changed=False,
        manufacturer_speed_rating_established=False,
        timing_implementation=str(Path(path_timing.__file__).resolve()),
        timing_implementation_sha256=digest(path_timing.__file__),
        time_floor_policy='Common 0.2 s per moving edge, existing stationary dwells preserved; saved already-retimed times retained as evidence. Original 0.5 deg/s, 1 deg/s2 baseline must reproduce before comparison.',
        explanation='Short stopped runs are acceleration-limited. Quintic minimum duration is max(1.875*d/v, sqrt((10/sqrt(3))*d/a)), rounded to 0.2 s. Raising only v cannot reduce an acceleration-dominated duration. Keeping many stop points can therefore dominate the total.',
        limitations=['Reference peaks, not physical measured velocities or accelerations.',
                     'No dynamics, torque, payload, collision envelope or manufacturer capability validation is added.',
                     'Higher acceleration can increase load and tracking error; these candidates are not approved for hardware.',
                     'Retreat previously stopped on stale feedback; its full-path subtotal is hypothetical.',
                     'Gripper actions, acquisition, perception, planning, waiting and feedback settling are excluded.',
                     'This report contains no joint trajectory, command stream or execution option.'],
        stages=rows, nominal_reference_subtotals=totals)


def summary_text(report):
    lines = ['OFFLINE ONLY — reference timing comparison; not hardware verified or executable.',
             'No existing limit or original log was changed.',
             'stage | profile | duration_s | peak_deg_s | peak_deg_s2 | stopped_runs']
    for row in report['stages']:
        for item in row['profiles']:
            lines.append(f"{row['stage']} | {item['profile']} | {item['reference_duration_s']:.1f} | "
                         f"{item['analytical_peak_velocity_deg_s']:.6f} | "
                         f"{item['analytical_peak_acceleration_deg_s2']:.6f} | {item['stopped_runs']}")
    lines.append('Nominal four-path reference subtotal (not measured cycle time):')
    for total in report['nominal_reference_subtotals']:
        lines.append(f"{total['profile']}: {total['nominal_arm_reference_subtotal_s']:.1f} s, reduction {total['reduction_percent']:.2f}%")
    lines.extend(['', report['time_floor_policy'], report['explanation'], *report['limitations']])
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output-root', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--include-acceleration-2', action='store_true', help='Also report the unvalidated 2 deg/s, 2 deg/s2 candidate')
    args = parser.parse_args()
    report = build_report(args.source_root, args.include_acceleration_2)
    output = args.output_root
    if output.exists():
        output = output.with_name(output.name+'_'+datetime.now().strftime('%Y%m%dT%H%M%S%f'))
    output.mkdir(parents=True, exist_ok=False)
    (output/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (output/'summary.txt').write_text(summary_text(report))
    print(summary_text(report))
    print('Saved: '+str(output))


if __name__ == '__main__':
    main()
