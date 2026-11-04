"""Geometry-preserving timing; no hardware or ROS dependencies.

Adjacent collinear edges share acceleration and cruise. Actual corners and
explicit dwell edges remain stopped: blending a corner would change the path
already checked for collision. Half-cosine velocity ramps with cruise improve
long runs; shorter runs retain quintic timing when it is faster. Both have
continuous position, velocity and acceleration; no device jerk limit is claimed.
"""
import math
import numpy as np

PROFILE = 'collinear_bounded_v1'
COLLINEAR_TOLERANCE_DEG = 1e-10


def _profile(distance, minimum_time, rate, speed, acceleration):
    if distance < 1e-14:
        count = max(1, math.ceil(minimum_time * rate - 1e-8))
        return dict(shape='hold', distance=0., peak_speed=0., ramp_s=0., cruise_s=0.,
                    stretch=1., count=count, duration_s=count/rate)
    peak = min(speed, math.sqrt(2 * acceleration * distance / math.pi))
    ramp = math.pi * peak / (2 * acceleration)
    cruise = max(0., distance / peak - ramp)
    natural = 2*ramp + cruise
    quintic = max(1.875*distance/speed, math.sqrt(10/math.sqrt(3)*distance/acceleration))
    if quintic <= natural:
        count = max(1, math.ceil(max(quintic, minimum_time)*rate-1e-8))
        duration = count/rate
        return dict(shape='quintic', distance=distance, peak_speed=1.875*distance/duration,
                    ramp_s=duration/2, cruise_s=0., stretch=1., count=count, duration_s=duration)
    count = max(1, math.ceil(max(natural, minimum_time) * rate - 1e-8))
    duration = count/rate
    return dict(shape='cosine_cruise', distance=distance, peak_speed=peak, ramp_s=ramp,
                cruise_s=cruise, stretch=duration/natural,
                count=count, duration_s=duration)


def scalar_state(run, time_s):
    """Distance, speed and acceleration along the run (SDK degree units)."""
    d, v, a, c, stretch = (run[k] for k in
        ['distance', 'peak_speed', 'ramp_s', 'cruise_s', 'stretch'])
    if d == 0:
        return 0., 0., 0.
    if run['shape'] == 'quintic':
        duration = run['duration_s']
        u = min(max(time_s/duration, 0.), 1.)
        return (d*(10*u**3-15*u**4+6*u**5),
                d*30*u**2*(1-u)**2/duration,
                d*60*u*(1-u)*(1-2*u)/duration**2)
    t = min(max(time_s/stretch, 0.), 2*a+c)
    if t <= 0:
        return 0., 0., 0.
    if t >= 2*a+c:
        return d, 0., 0.
    if t < a or t > a+c:
        decelerating = t > a+c
        r = 2*a+c-t if decelerating else t
        phase = math.pi*r/a
        s = .5*v*(r-a/math.pi*math.sin(phase))
        velocity = .5*v*(1-math.cos(phase))/stretch
        acc = .5*v*math.pi/a*math.sin(phase)/stretch**2
        return (d-s, velocity, -acc) if decelerating else (s, velocity, acc)
    return .5*v*a+v*(t-a), v/stretch, 0.


def run_state(run, time_s):
    a, b = np.asarray(run['start']), np.asarray(run['end'])
    if run['distance'] == 0:
        return a.copy(), np.zeros(6), np.zeros(6)
    s, v, acc = scalar_state(run, time_s)
    direction = (b-a)/run['distance']
    return a+direction*s, direction*v, direction*acc


def _runs(q):
    start = 0
    while start < len(q)-1:
        end = start+1
        # Preserve duplicate points as explicit dwells; do not smooth over them.
        if np.max(abs(q[end]-q[start])) >= 1e-14:
            for candidate in range(end+1, len(q)):
                if np.max(abs(q[candidate]-q[candidate-1])) < 1e-14:
                    break
                delta = q[candidate]-q[start]
                axis = int(np.argmax(abs(delta)))
                if abs(delta[axis]) < 1e-14:
                    break
                u = (q[start:candidate+1, axis]-q[start, axis])/delta[axis]
                residual = np.max(abs(q[start:candidate+1]-(q[start]+u[:, None]*delta)))
                if residual > COLLINEAR_TOLERANCE_DEG or np.min(np.diff(u)) <= 0:
                    break
                end = candidate
        yield start, end
        start = end


def retime(q, source_times, rate, speed, acceleration):
    q = np.asarray(q, dtype=float)
    runs, waypoint_times, waypoint_velocities, waypoint_accelerations = [], [0.], [[0.]*6], [[0.]*6]
    elapsed = 0.
    for start, end in _runs(q):
        delta = q[end]-q[start]
        distance = float(np.max(abs(delta)))
        run = _profile(distance, float(source_times[end]-source_times[start]), rate, speed, acceleration)
        run.update(start=q[start].tolist(), end=q[end].tolist(),
                   start_index=start, end_index=end, start_time_s=elapsed)
        axis = int(np.argmax(abs(delta)))
        for index in range(start+1, end+1):
            if index == end:
                t = run['duration_s']
            else:
                wanted = (q[index, axis]-q[start, axis])/delta[axis]*distance
                lo, hi = 0., run['duration_s']
                for _ in range(55):
                    middle = (lo+hi)/2
                    if scalar_state(run, middle)[0] < wanted:
                        lo = middle
                    else:
                        hi = middle
                t = (lo+hi)/2
            _, v, acc = run_state(run, t)
            waypoint_times.append(elapsed+t)
            waypoint_velocities.append(v.tolist())
            waypoint_accelerations.append(acc.tolist())
        elapsed += run['duration_s']
        runs.append(run)
    return dict(timing_profile=PROFILE, runs=runs, duration_s=elapsed,
                # Kept for clients that sum per-edge times. Fractional ticks at
                # internal points are intentional; commands stay on whole ticks.
                ticks=(np.diff(waypoint_times)*rate).tolist(),
                waypoint_times_s=waypoint_times,
                waypoint_velocities_deg_s=waypoint_velocities,
                waypoint_accelerations_deg_s2=waypoint_accelerations,
                continuous_internal_points=sum(r['end_index']-r['start_index']-1 for r in runs),
                stopped_corners=max(0, len(runs)-1),
                geometry_preserved=True, collinear_tolerance_deg=COLLINEAR_TOLERANCE_DEG)


def references(plan):
    step = 0
    yield 0., np.asarray(plan['sdk'][0])
    for run in plan['runs']:
        for index in range(1, run['count']+1):
            step += 1
            q, _, _ = run_state(run, index/plan['rate'])
            yield step/plan['rate'], q
