"""Monotonic grasp timing. Overlapping work is never added into cycle duration.

The primary interval is recognition_start -> retreat_complete. Human approval
waiting is excluded only when explicitly recorded; missing stages remain missing.
"""
from contextlib import contextmanager
import json
import math
from pathlib import Path
import time
import uuid

CATEGORIES = ('acquisition', 'recognition', 'pose', 'planning', 'communication',
              'arm_execution', 'gripper_execution', 'stage_wait',
              'human_confirmation', 'verification', 'startup', 'serialization')


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def union_seconds(intervals, window=None):
    values = []
    for start, end in intervals:
        if not all(math.isfinite(x) for x in (start, end)) or end < start:
            raise ValueError('Invalid timing interval')
        if window:
            start, end = max(start, window[0]), min(end, window[1])
        if end >= start:
            values.append((start, end))
    total = 0.; right = -math.inf
    for start, end in sorted(values):
        total += max(0., end - max(start, right)); right = max(right, end)
    return total


class Trace:
    def __init__(self, path, clock=time.monotonic):
        self.path = Path(path); self.path.parent.mkdir(parents=True, exist_ok=True)
        self.clock = clock; self.boot = boot_id()

    def write(self, row):
        with self.path.open('a') as stream:
            stream.write(json.dumps(dict(boot_id=self.boot, wall_time=time.time(), **row),
                                    allow_nan=False) + '\n')

    def mark(self, name, **data):
        self.write(dict(kind='mark', name=name, monotonic=self.clock(), **data))

    def interval(self, category, name, start, end, **data):
        if category not in CATEGORIES or not all(math.isfinite(x) for x in (start, end)) or end < start:
            raise ValueError('Invalid measured span')
        self.write(dict(kind='span', category=category, name=name, start=start, end=end,
                        duration_s=end-start, **data))

    @contextmanager
    def span(self, category, name, **data):
        start = self.clock(); outcome = 'completed'
        try:
            yield
        except BaseException:
            outcome = 'failed'; raise
        finally:
            self.interval(category, name, start, self.clock(), outcome=outcome, **data)


def summarize(rows):
    boots = {r['boot_id'] for r in rows}
    if len(boots) > 1:
        raise ValueError('Cannot combine monotonic timestamps from different boots')
    marks = {}
    for row in rows:
        if row['kind'] == 'mark': marks.setdefault(row['name'], []).append(row['monotonic'])
    if any(len(marks.get(key, [])) > 1 for key in ('recognition_start', 'retreat_complete')):
        raise ValueError('Multiple trials need separate timing files')
    start = next(iter(marks.get('recognition_start', [])), None)
    end = next(iter(marks.get('retreat_complete', [])), None)
    window = (start, end) if start is not None and end is not None else None
    if window and end < start: raise ValueError('Negative cycle duration')
    spans = [r for r in rows if r['kind'] == 'span']
    categories = {}
    for category in CATEGORIES:
        selected = [r for r in spans if r['category'] == category]
        categories[category] = dict(measured=bool(selected), calls=len(selected),
            union_s=union_seconds([(r['start'], r['end']) for r in selected], window) if selected else None)
    waits = [(r['start'], r['end']) for r in spans if r['category'] == 'human_confirmation']
    human = union_seconds(waits, window) if window else None
    return dict(schema='d1-grasp-timing-v1', complete_interval=window is not None,
        definition='recognition_start to feedback-confirmed retreat_complete',
        wall_cycle_s=end-start if window else None,
        explicit_human_wait_s=human,
        cycle_excluding_explicit_human_wait_s=end-start-human if window else None,
        categories=categories,
        notes=['Category intervals can overlap; never sum them as total time.',
               'Execution includes communication and feedback settling; it is not an independent motor-motion measurement.',
               'Unrecorded historical waits are not retrospectively classified as human or vision time.'])


def load_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def import_recognition(trace, report):
    """Import immutable perception timing only from this boot, without rebasing."""
    timing = report.get('timing', {})
    if timing.get('boot_id') != trace.boot:
        raise ValueError('Perception timing is not from the current host boot')
    start, end = timing['start_monotonic'], timing['end_monotonic']
    if not 0 <= start <= end <= trace.clock(): raise ValueError('Invalid perception timestamps')
    trace.write(dict(kind='mark', name='recognition_start', monotonic=start))
    for row in timing.get('spans', []):
        if not start <= row['start'] <= row['end'] <= end:
            raise ValueError('Perception span outside its capture interval')
        trace.interval(row['category'], row['name'], row['start'], row['end'])


def import_transport(trace, folder, stage):
    """Record command-to-receipt durations; these overlap execution intervals."""
    folder = Path(folder)
    path = folder / 'transport.jsonl'
    if not path.exists(): path = folder / 'motion/events.jsonl'
    if not path.exists(): return dict(measured=False)
    pending = None; count = 0; unmatched = 0
    for row in load_rows(path):
        stamp = row.get('host_monotonic', row.get('t'))
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp): continue
        if row.get('kind') == 'command':
            if pending is not None: unmatched += 1
            pending = stamp
        elif row.get('kind') == 'output' and row.get('line', '').startswith('[ok] '):
            if pending is not None and stamp >= pending:
                trace.interval('communication', stage+'_receipt', pending, stamp); count += 1
            else: unmatched += 1
            pending = None
    return dict(measured=count > 0, receipt_spans=count, unmatched=unmatched+int(pending is not None))
