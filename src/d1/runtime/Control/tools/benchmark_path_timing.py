#!/usr/bin/env python3
"""Offline timing ablation on saved SDK paths; never opens robot connections.

Uses the identical path and commissioning bounds for both timing policies.
Historical retimed timestamps are replaced by 0.2 s edge placeholders because
they encode the old policy, not an independent physical constraint. Actual
historical durations are retained separately. These are reference timings,
not measured physical completion times or a complete grasp trial.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

CONTROL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROL/'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import Mapping, NAMES, prepare, references
from d1_pick_place.path_timing import run_state


def benchmark(path):
    raw = path.read_bytes()
    source = json.loads(raw)
    historical = source['sdk_plan']
    q = np.asarray(historical['sdk'])
    times = np.arange(len(q))*.2
    mapping = Mapping(np.ones(6), np.zeros(6))
    plans = {profile: prepare(NAMES, np.deg2rad(q), times, mapping,
                             timing_profile=profile)
             for profile in ['edge_quintic_v1', 'collinear_bounded_v1']}
    new = plans['collinear_bounded_v1']
    maximum_velocity = maximum_acceleration = 0.
    for run in new['runs']:
        for t in np.linspace(0, run['duration_s'], 1001):
            _, v, a = run_state(run, t)
            maximum_velocity = max(maximum_velocity, float(np.max(abs(v))))
            maximum_acceleration = max(maximum_acceleration, float(np.max(abs(a))))
    samples = list(references(new))
    values = np.array([v for _, v in samples])
    baseline = plans['edge_quintic_v1']['duration_s']
    assert np.max(abs(values[0]-q[0])) < 1e-10
    assert np.max(abs(values[-1]-q[-1])) < 1e-10
    assert maximum_velocity <= .5+1e-10 and maximum_acceleration <= 1.+1e-10
    return dict(source=str(path.resolve()), source_sha256=hashlib.sha256(raw).hexdigest(),
                path_sha256=hashlib.sha256(json.dumps(q.tolist()).encode()).hexdigest(),
                source_points=len(q), historical_planned_duration_s=historical['duration_s'],
                baseline_recomputed_duration_s=baseline,
                optimized_reference_duration_s=new['duration_s'],
                reference_reduction_percent=100*(baseline-new['duration_s'])/baseline,
                continuous_internal_points=new['continuous_internal_points'],
                retained_stopped_corners=new['stopped_corners'],
                reference_commands=len(samples),
                sampled_peak_velocity_deg_s=maximum_velocity,
                sampled_peak_acceleration_deg_s2=maximum_acceleration,
                sampled_command_speed_deg_s=float(np.max(abs(np.diff(values,axis=0)))*5),
                sampled_command_acceleration_deg_s2=(float(np.max(abs(np.diff(values,n=2,axis=0)))*25)
                    if len(values)>2 else 0.),
                hardware_executed=False, geometry_preserved=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plans', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = [benchmark(p) for p in args.plans]
    report = dict(hardware_executed=False, policy='same geometry and same commissioning bounds',
                  caveat='Reference durations only. Source paths may come from failed or partial trials; do not sum as a completed grasp.',
                  rate_hz=5., velocity_bound_deg_s=.5, acceleration_bound_deg_s2=1.,
                  bound_source='existing SDK commissioning limits; manufacturer rating not established',
                  source_timing_policy='old retiming replaced with identical 0.2 s placeholders for both policies',
                  rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    for row in rows:
        print(f"{Path(row['source']).parent.name}: {row['baseline_recomputed_duration_s']:.1f} -> "
              f"{row['optimized_reference_duration_s']:.1f} s ({row['reference_reduction_percent']:.1f}%)")


if __name__ == '__main__':
    main()
