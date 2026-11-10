#!/usr/bin/env python3
"""Analyze an ended local joint log; never opens a device or a network socket.

Only analysis artifacts are written. Recorded timestamps/positions are preserved;
velocity is an estimate from host DDS receipt time, not encoder velocity.
"""
from __future__ import annotations

import argparse
import ast
from collections import deque
import hashlib
import html
import json
import math
from pathlib import Path
import statistics


LIMITS = [[-135, 135], [-90, 90], [-90, 90], [-135, 135], [-90, 90], [-135, 135], [0, 65]]
LIMIT_SOURCE = Path('/media/aup/UbuntuData/ACT训练/02_程序与源码/D1/Control/ros2_ws/src/d1_pick_place/d1_pick_place/sdk_executor.py')
UNITS = ['deg'] * 6 + ['sdk_raw']
GAP_S = 0.5
SMOOTH_WINDOW_S = 0.4
START_LABELS = {'开始演示', 'demonstration_started', 'manual_demo_started'}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON value: ' + value)))


def records(path):
    rows, rejected = [], []
    if not path.exists():
        return rows, rejected
    with path.open() as stream:
        for line, text in enumerate(stream, 1):
            if not text.strip():
                continue
            try:
                row = json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON value: ' + value)))
                if not isinstance(row, dict):
                    raise ValueError('Record is not an object')
                rows.append((line, row))
            except (ValueError, TypeError) as exc:
                rejected.append({'line': line, 'reason': str(exc)})
    return rows, rejected


def save_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def metric(values):
    valid = [v for v in values if finite(v)]
    return {'count': len(valid), 'min': min(valid) if valid else None,
            'max': max(valid) if valid else None,
            'max_abs': max(map(abs, valid)) if valid else None}


def svg_plot(path, points, start):
    """Dependency-free fallback, with discontinuities explicitly broken."""
    width, row_h = 1300, 165
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{7*row_h+100}" viewBox="0 0 {width} {7*row_h+100}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="30" y="28" font-family="sans-serif" font-size="20">D1 measured positions and estimated velocities</text>',
             '<text x="30" y="50" font-family="sans-serif" font-size="12">Original elapsed time; gaps remain gaps. J6 uses SDK raw units. Replay is not approved.</text>']
    ts = [p['t'] for p in points if finite(p['t'])]
    xmin, xmax = (min(ts), max(ts)) if ts else (0, 1)
    if xmax == xmin:
        xmax = xmin + 1
    for axis in range(7):
        for column, key in enumerate(('q', 'v_smooth')):
            left, top, plot_w, plot_h = 75 + column*640, 95 + axis*row_h, 540, 110
            ys = [p[key][axis] for p in points if finite(p['t']) and finite(p[key][axis])]
            low, high = (min(ys), max(ys)) if ys else (0, 1)
            pad = max((high-low)*0.1, 0.05)
            low -= pad; high += pad
            x = lambda t: left + (t-xmin)/(xmax-xmin)*plot_w
            y = lambda v: top + plot_h - (v-low)/(high-low)*plot_h
            if start is not None:
                end = min(max(start, xmin), xmax)
                parts.append(f'<rect x="{left}" y="{top}" width="{x(end)-left}" height="{plot_h}" fill="#eef1f5"/>')
            parts.append(f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#bac5d1"/>')
            unit = UNITS[axis] + ('/s' if column else '')
            label = f'J{axis} {"estimated velocity" if column else "position"} ({unit})'
            parts.append(f'<text x="{left}" y="{top-10}" font-family="sans-serif" font-size="13">{html.escape(label)}</text>')
            parts.append(f'<text x="{left-6}" y="{top+10}" text-anchor="end" font-family="sans-serif" font-size="10">{high:.3g}</text>')
            parts.append(f'<text x="{left-6}" y="{top+plot_h}" text-anchor="end" font-family="sans-serif" font-size="10">{low:.3g}</text>')
            path_data, previous = [], None
            for point in points:
                v, t = point[key][axis], point['t']
                if not finite(v) or not finite(t):
                    previous = None
                    continue
                command = 'M' if previous is None or point['break_before'] or t <= previous else 'L'
                path_data.append(f'{command}{x(t):.2f},{y(v):.2f}')
                previous = t
                if not column and (v < LIMITS[axis][0] or v > LIMITS[axis][1]):
                    parts.append(f'<circle cx="{x(t):.2f}" cy="{y(v):.2f}" r="2.5" fill="#d53a30"/>')
            parts.append(f'<path d="{" ".join(path_data)}" fill="none" stroke="{"#b46316" if column else "#1765ad"}" stroke-width="1.5"/>')
            if start is not None and xmin <= start <= xmax:
                parts.append(f'<path d="M{x(start):.2f},{top}V{top+plot_h}" stroke="#788492" stroke-dasharray="4 4"/>')
            parts.append(f'<text x="{left}" y="{top+plot_h+16}" font-family="sans-serif" font-size="10">{xmin:.3f} s</text>')
            parts.append(f'<text x="{left+plot_w}" y="{top+plot_h+16}" text-anchor="end" font-family="sans-serif" font-size="10">{xmax:.3f} s</text>')
    parts.append('</svg>')
    path.write_text('\n'.join(parts))


def make_plot(output, points, start, force_svg=False):
    if not force_svg:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            plt = None
        if plt is not None:
            figure, axes = plt.subplots(7, 2, figsize=(15, 18), sharex=True)
            for index in range(7):
                for column, key in enumerate(('q', 'v_smooth')):
                    ax = axes[index, column]
                    xs, ys = [], []
                    for point in points:
                        if point['break_before']:
                            xs.append(float('nan')); ys.append(float('nan'))
                        xs.append(point['t'] if finite(point['t']) else float('nan'))
                        ys.append(point[key][index] if finite(point[key][index]) else float('nan'))
                    ax.plot(xs, ys, linewidth=1, color='#ac6418' if column else '#1765ad')
                    if not column:
                        bad = [p for p in points if finite(p['t']) and finite(p['q'][index]) and not LIMITS[index][0] <= p['q'][index] <= LIMITS[index][1]]
                        if bad:
                            ax.scatter([p['t'] for p in bad], [p['q'][index] for p in bad], color='#c32d27', s=10, label='Outside local range')
                            ax.legend(fontsize=7)
                    if start is not None:
                        ax.axvline(start, color='#68778b', linestyle='--', linewidth=.8)
                        lower = min((p['t'] for p in points if finite(p['t'])), default=start)
                        if start > lower:
                            ax.axvspan(lower, start, color='#68778b', alpha=.10)
                    ax.grid(alpha=.2)
                    ax.set_ylabel(f'J{index} {UNITS[index]}' + ('/s' if column else ''))
            axes[0, 0].set_title('Measured positions (red: outside existing local ranges)')
            axes[0, 1].set_title('Estimated velocity: endpoint slope over <= 0.4 s')
            for ax in axes[-1]:
                ax.set_xlabel('Original recorder elapsed time (s)')
            figure.suptitle('D1 recorded demonstration: no time trimming, no angle clipping\n' +
                           ('Shaded: before the first explicit demonstration-start marker' if start is not None else 'No explicit start marker: preparation and motion remain unsegmented'), fontsize=12)
            figure.tight_layout(rect=(0, 0, 1, .96))
            target = output/'joint_timeline.png'
            figure.savefig(target, dpi=130)
            plt.close(figure)
            return target
    target = output/'joint_timeline.svg'
    svg_plot(target, points, start)
    return target


def analyze(session: Path, output: Path | None = None, force_svg=False):
    global LIMITS
    if LIMIT_SOURCE.exists():
        tree = ast.parse(LIMIT_SOURCE.read_text())
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == 'LIMITS' for target in node.targets):
                if not isinstance(node.value, ast.Call) or not node.value.args:
                    raise ValueError('Local limit declaration no longer has the reviewed literal structure')
                limits = ast.literal_eval(node.value.args[0])
                if len(limits) != 7 or any(len(pair) != 2 or not all(finite(v) for v in pair) or pair[0] >= pair[1] for pair in limits):
                    raise ValueError('Malformed local D1 limit declaration')
                LIMITS = limits
                break
        else:
            raise ValueError('Local D1 source contains no explicit LIMITS declaration')
    session = session.resolve()
    summary_path = session/'summary.json'
    if not summary_path.is_file():
        raise ValueError('Session has no summary.json: recording is not confirmed ended; no formal analysis generated.')
    summary = read_json(summary_path)
    if summary.get('state') not in {'stopped', 'error'}:
        raise ValueError('Session summary does not confirm a stopped/error recorder state.')
    metadata_path, samples_path, events_path = (session/name for name in ('metadata.json', 'joint_samples.jsonl', 'events.jsonl'))
    metadata = read_json(metadata_path)
    if not samples_path.is_file():
        raise FileNotFoundError(samples_path)
    rows, rejected = records(samples_path)
    events, event_rejected = records(events_path)
    markers = [{'line': line, **event} for line, event in events if event.get('label') in START_LABELS and finite(event.get('elapsed_s'))]
    start = min((event['elapsed_s'] for event in markers), default=None)
    issues = {'invalid_json_rows': rejected, 'invalid_event_json_rows': event_rejected,
              'nonfinite_or_malformed_rows': [], 'nonmonotonic_rows': [], 'time_basis_mismatch_rows': [],
              'gaps': [], 'samples_after_stop_cutoff': [], 'invalid_stored_velocity_rows': []}
    points, previous, window = [], None, deque()
    valid_dts = []
    cutoff = summary.get('stop_cutoff_monotonic_s')
    origin = metadata.get('host_monotonic_start_s')
    for line, row in rows:
        t, stamp, q = row.get('elapsed_s'), row.get('dds_received_monotonic_s'), row.get('q')
        valid = finite(t) and finite(stamp) and isinstance(q, list) and len(q) == 7 and all(finite(v) for v in q)
        point = {'source_line': line, 'sample': row.get('sample'), 't': t if finite(t) else None,
                 'stamp': stamp if finite(stamp) else None, 'q': q if valid else [None]*7,
                 'v_raw': [None]*7, 'v_smooth': [None]*7, 'break_before': previous is None}
        for key in ('velocity', 'velocity_smoothed'):
            values = row.get(key)
            if values is not None and (not isinstance(values, list) or len(values) != 7 or any(v is not None and not finite(v) for v in values)):
                issues['invalid_stored_velocity_rows'].append({'line': line, 'field': key})
        if not valid:
            issues['nonfinite_or_malformed_rows'].append(line)
            previous = None; window.clear(); point['break_before'] = True
            points.append(point)
            continue
        if finite(origin) and abs((stamp-origin)-t) > .001:
            issues['time_basis_mismatch_rows'].append(line)
        if finite(cutoff) and stamp > cutoff:
            issues['samples_after_stop_cutoff'].append(line)
        dt = None if previous is None else stamp-previous['stamp']
        backwards = previous is not None and (dt <= 0 or t <= previous['t'])
        skipped_invalid_record = previous is not None and any(previous['source_line'] < entry['line'] < line for entry in rejected)
        gap = bool(row.get('gap_before')) or (dt is not None and dt > GAP_S)
        if backwards:
            issues['nonmonotonic_rows'].append(line)
        if gap:
            issues['gaps'].append({'line': line, 'before_elapsed_s': previous['t'] if previous else None, 'after_elapsed_s': t, 'duration_s': dt})
        if backwards or gap or skipped_invalid_record:
            window.clear(); point['break_before'] = True
        if dt is not None and dt > 0:
            valid_dts.append(dt)
        if previous is not None and not backwards and not gap and not skipped_invalid_record:
            velocity = [(q[i]-previous['q'][i])/dt for i in range(7)]
            if all(finite(v) for v in velocity):
                point['v_raw'] = velocity
            else:
                issues['nonfinite_or_malformed_rows'].append(line)
        if not backwards:
            window.append((stamp, q))
            while len(window) > 1 and stamp-window[0][0] > SMOOTH_WINDOW_S:
                window.popleft()
            if len(window) > 1:
                smooth = [(q[i]-window[0][1][i])/(stamp-window[0][0]) for i in range(7)]
                if all(finite(v) for v in smooth):
                    point['v_smooth'] = smooth
        points.append(point)
        previous = None if backwards else point
    axes = []
    for axis in range(7):
        bad = [p for p in points if finite(p['q'][axis]) and not LIMITS[axis][0] <= p['q'][axis] <= LIMITS[axis][1]]
        axes.append({'joint': f'J{axis}', 'unit': UNITS[axis], 'position': metric([p['q'][axis] for p in points]),
                     'estimated_raw_velocity': metric([p['v_raw'][axis] for p in points]),
                     'estimated_smoothed_velocity': metric([p['v_smooth'][axis] for p in points]),
                     'local_range': LIMITS[axis], 'out_of_range_sample_count': len(bad),
                     'out_of_range_source_lines': [p['source_line'] for p in bad]})
    preparation = sum(finite(p['t']) and p['t'] < start for p in points) if start is not None else None
    output = (output or session/'analysis').resolve()
    protected = {summary_path, metadata_path, samples_path, events_path}
    output.mkdir(parents=True, exist_ok=True)
    if any((output/name).resolve() in protected for name in ('joint_timeline.png', 'joint_timeline.svg', 'analysis_review.json')):
        raise ValueError('Analysis output overlaps source logs')
    figure = make_plot(output, points, start, force_svg)
    review = {
        'schema': 'd1-ended-measured-demo-analysis-v1', 'session': str(session),
        'source_logs_read_only': True, 'hardware_accessed': False, 'robot_commands_sent': 0,
        'replay_ready': False, 'actual_motion_replay_ready': False,
        'automatic_unlock_restore_or_replay': False,
        'grasp_success_verified_by_this_analysis': False,
        'replay_blockers': ['Position-control/holding recovery has not been established by this log analysis.',
                            'Current robot start state, collision-free physical path and speed/acceleration require separate review.',
                            'Explicit approval of the concrete real-robot target, path and parameters remains required.'],
        'full_vla_training_dataset': False, 'camera_recorded': summary.get('camera_recorded', False),
        'commands_recorded': summary.get('commands_recorded', False),
        'source_summary_state': summary.get('state'), 'source_stop_reason': summary.get('stop_reason'),
        'subscriber_pipe_drained': summary.get('subscriber_pipe_drained'),
        'summary_sample_count': summary.get('sample_count'), 'parsed_sample_rows': len(rows),
        'summary_count_matches_parsed_rows': summary.get('sample_count') == len(rows),
        'phase': {'explicit_start_markers': markers, 'first_start_elapsed_s': start,
                  'preparation_samples': preparation,
                  'demonstration_or_later_samples': len(points)-preparation if preparation is not None else None,
                  'automatic_static_trimming': False,
                  'interpretation': 'All original samples/time remain in the figure. Before first marker is shaded preparation.' if start is not None else 'No start marker found; preparation and motion are not inferred or trimmed.'},
        'time': {'basis': 'Original DDS host-monotonic receipt timestamps; original elapsed time retained.',
                 'first_elapsed_s': next((p['t'] for p in points if finite(p['t'])), None),
                 'last_elapsed_s': next((p['t'] for p in reversed(points) if finite(p['t'])), None),
                 'median_positive_interval_s': statistics.median(valid_dts) if valid_dts else None,
                 'gap_threshold_s': GAP_S, 'smoothing_window_s': SMOOTH_WINDOW_S},
        'velocity_note': 'Finite differences and endpoint slopes estimate motion from received positions; transport jitter, quantization and undetected packet loss can distort speed. Not measured encoder velocity. No unwrap, position clipping, interpolation across gaps, or trajectory retiming.',
        'range_source': {'path': str(LIMIT_SOURCE), 'sha256': sha(LIMIT_SOURCE) if LIMIT_SOURCE.exists() else None,
                         'scope': 'Existing local commissioning limits; not newly verified manufacturer ratings. Out-of-range samples are reported and never clipped.'},
        'quality': issues, 'joints': axes,
        'artifacts': {'time_plot': str(figure), 'review': str(output/'analysis_review.json')},
        'sources': {p.name: {'path': str(p), 'sha256': sha(p)} for p in (summary_path, metadata_path, samples_path, events_path) if p.exists()},
    }
    review['requires_data_review'] = any(issues.values()) or not review['summary_count_matches_parsed_rows'] or any(a['out_of_range_sample_count'] for a in axes) or summary.get('subscriber_pipe_drained') is False
    save_json(output/'analysis_review.json', review)
    return review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--svg', action='store_true', help='Use dependency-free SVG even if matplotlib is installed')
    args = parser.parse_args()
    result = analyze(args.session, args.output, args.svg)
    print(json.dumps({'status': 'ended_log_analyzed', 'samples': result['parsed_sample_rows'],
                      'requires_data_review': result['requires_data_review'], 'replay_ready': False,
                      'actual_motion_replay_ready': False,
                      'artifacts': result['artifacts']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
