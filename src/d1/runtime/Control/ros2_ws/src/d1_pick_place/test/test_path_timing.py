import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from d1_pick_place import path_timing
from d1_pick_place.sdk_executor import Mapping, NAMES, prepare, references
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from run_arm_stage import bounded_prefix


def prepared(q, times=None, profile=path_timing.PROFILE):
    q = np.asarray(q)
    return prepare(NAMES, np.deg2rad(q),
        np.arange(len(q))*.2 if times is None else times,
        Mapping(np.ones(6), np.zeros(6)), timing_profile=profile)


def test_collinear_waypoints_have_nonzero_velocity_and_unchanged_geometry():
    q = np.linspace(np.zeros(6), [40, 20, -10, 1, 0, 0], 9)
    p = prepared(q)
    old = prepared(q, profile='edge_quintic_v1')
    assert len(p['runs']) == 1 and p['continuous_internal_points'] == 7
    assert p['duration_s'] < old['duration_s'] * .7
    assert all(v[0] > 0 for v in p['waypoint_velocities_deg_s'][1:-1])
    for _, target in references(p):
        assert np.max(abs(target - q[-1]*(target[0]/40))) < 1e-10


def test_real_corner_reversal_and_dwell_are_not_smoothed_away():
    q = np.zeros((6, 6))
    q[:, :2] = [[0, 0], [2, 0], [2, 1], [2, 1], [2, 2], [2, 0]]
    p = prepared(q)
    assert len(p['runs']) == 5
    assert np.max(np.abs(p['waypoint_velocities_deg_s'])) < 1e-12
    for run in p['runs']:
        for endpoint in [0., run['duration_s']]:
            _, velocity, acceleration = path_timing.run_state(run, endpoint)
            assert np.max(abs(velocity)) < 1e-12
            assert np.max(abs(acceleration)) < 1e-12


@pytest.mark.parametrize('distance', [.00001, .05, .2, 2., 20., 120.])
def test_continuous_and_sampled_bounds_and_smooth_ramp_joins(distance):
    q = np.linspace(np.zeros(6), [distance, -.4*distance, .1*distance, 0, 0, 0],
                    max(2, int(distance/10)+2))
    p = prepared(q)
    for run in p['runs']:
        for t in np.linspace(0, run['duration_s'], 3001):
            _, v, a = path_timing.run_state(run, t)
            assert np.max(abs(v)) <= .5+1e-12
            assert np.max(abs(a)) <= 1.+1e-12
        for t in [run['ramp_s']*run['stretch'],
                  (run['ramp_s']+run['cruise_s'])*run['stretch']]:
            before = path_timing.run_state(run, t-1e-7)
            after = path_timing.run_state(run, t+1e-7)
            assert np.max(abs(before[1]-after[1])) < 1e-6
            assert np.max(abs(before[2]-after[2])) < 1e-4
    points = list(references(p))
    times = np.array([t for t, _ in points])
    values = np.array([q for _, q in points])
    assert np.max(abs(np.diff(values, axis=0))) <= .5/5+1e-12
    if len(values) > 2:
        assert np.max(abs(np.diff(values, n=2, axis=0)))*25 <= 1.+1e-12
    assert np.allclose(values[0], q[0]) and np.allclose(values[-1], q[-1])
    assert times[-1] == pytest.approx(p['duration_s'])


def test_long_source_duration_and_timestamp_retiming_are_idempotent():
    q = np.linspace(np.zeros(6), [10, 1, 0, 0, 0, 0], 9)
    p = prepared(q, np.linspace(0, 100, len(q)))
    assert p['duration_s'] == 100
    b = prepared(q, np.round(p['waypoint_times_s'], 9))
    assert p == b


def test_legacy_saved_plans_still_emit_original_quintic():
    q = np.array([[0]*6, [2, 0, 0, 0, 0, 0]])
    p = prepared(q, profile='edge_quintic_v1')
    before = list(references(p))
    del p['timing_profile']
    after = list(references(p))
    assert all(t == u and np.array_equal(a, b) for (t, a), (u, b) in zip(before, after))


def test_geometry_change_and_large_jump_are_not_hidden():
    q = np.zeros((3, 6)); q[:, 0] = [0, 1, 2]; q[1, 1] = 1e-5
    assert len(prepared(q)['runs']) == 2
    q = np.zeros((2, 6)); q[1, 0] = 20.01
    with pytest.raises(ValueError, match='Discontinuous'):
        prepared(q)


def test_prefix_cannot_cut_moving_internal_waypoint_or_keep_later_runs():
    q = np.zeros((5, 6)); q[:, :2] = [[0, 0], [2, 0], [4, 0], [4, 1], [4, 2]]
    p = prepared(q)
    assert len(p['runs']) == 2
    with pytest.raises(ValueError, match='complete stopped'):
        bounded_prefix(p, p['waypoint_times_s'][1]+.1)
    prefix, selection = bounded_prefix(p, p['runs'][0]['duration_s']+.1)
    assert selection['target_point_index'] == 2
    assert len(prefix['runs']) == 1 and len(prefix['sdk']) == 3
    commands = list(references(prefix))
    assert commands[-1][0] == prefix['duration_s']
    assert np.array_equal(commands[-1][1], q[2])
    assert all(command[1][1] == 0 for command in commands)
    assert np.max(np.abs(prefix['waypoint_velocities_deg_s'][-1])) < 1e-12


def test_old_prefix_remains_supported_and_full_new_plan_is_not_truncated():
    q = np.zeros((3, 6)); q[:, 0] = [0, 1, 2]
    old = prepared(q, profile='edge_quintic_v1')
    prefix, _ = bounded_prefix(old, old['ticks'][0]/old['rate'])
    assert len(prefix['sdk']) == 2
    full = prepared(q)
    selected, selection = bounded_prefix(full, None)
    assert selected == full and not selection['truncated']
