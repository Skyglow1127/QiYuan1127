"""Saved 17:40 run evidence and explicitly simulated bounded angle recovery.

The saved log ends before recovery.  Any new frame below is synthetic and is
not evidence that this real robot later recovered or reached its endpoint.
"""
import json
from pathlib import Path
import socket
import subprocess
import sys

import numpy as np
import pytest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP / 'runtime/Control/ros2_ws/src/d1_pick_place'))
from d1_pick_place import sdk_executor

SAVED = APP / 'logs/20261026_174030_090949/execution/01_fixed_pregrasp/executed'


@pytest.fixture(autouse=True)
def offline_only(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('This regression must never connect or launch hardware I/O')
    monkeypatch.setattr(sdk_executor, 'CliTransport', forbidden)
    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setattr(socket, 'socket', forbidden)


@pytest.fixture
def evidence():
    originals = {name: (SAVED / name).read_bytes()
                 for name in ('result.json', 'plan.json', 'transport.jsonl')}
    events = [json.loads(line) for line in originals['transport.jsonl'].splitlines()]
    value = dict(result=json.loads(originals['result.json']),
                 plan=json.loads(originals['plan.json']), events=events)
    yield value
    for name, original in originals.items():
        assert (SAVED / name).read_bytes() == original, 'Saved real-run evidence changed'


def received(events, *, address, funcode):
    rows = []
    for event in events:
        if event['kind'] != 'output' or not event['line'].startswith('[rx] '):
            continue
        message = json.loads(event['line'][5:])
        if message.get('address') == address and message.get('funcode') == funcode:
            rows.append((event['t'], message['data']))
    return rows


def test_real_run_lost_angles_while_status_and_receipts_continued(evidence):
    events = evidence['events']
    result = evidence['result']
    raw = received(events, address=2, funcode=1)
    status = received(events, address=2, funcode=3)
    receipts = received(events, address=3, funcode=1)
    accepted = [e for e in events if e['kind'] == 'output'
                and e['line'].startswith('[feedback-accepted] ')]
    rejected = [e for e in events if e['kind'] == 'output'
                and e['line'].startswith('[feedback-rejected] ')]
    assert len(raw) == len(accepted) == 49
    assert rejected == [] and result['feedback_rejected_count'] == 0
    assert raw[-1][0] == pytest.approx(291063.085008165)
    assert status[-1][0] == pytest.approx(291063.699348556)
    assert status[-1][0] > raw[-1][0]
    assert receipts[-1][0] > raw[-1][0]
    assert status[-1][1] == dict(power_status=1, enable_status=1, error_status=0)
    intervals = np.diff([stamp for stamp, _ in raw])
    assert np.median(intervals) == pytest.approx(.11076930401031859)
    assert max(intervals) == pytest.approx(.40268302999902517)
    assert result['error_code'] == 'Angle feedback stale'
    assert result['final_valid_feedback_age_s'] == pytest.approx(.614639972045552)
    assert result['final_raw_feedback_age_s'] > .6


def test_real_final_commands_match_plan_but_last_angles_do_not_prove_arrival(evidence):
    commands = [e for e in evidence['events'] if e['kind'] == 'command']
    last_by_joint = {e['joint']: e for e in commands}
    result = evidence['result']
    planned = np.asarray(evidence['plan']['sdk_plan']['sdk'][-1])
    assert len(commands) == result['commands'] == result['receipts'] == 299
    assert set(last_by_joint) == set(range(6))
    sent = [last_by_joint[j]['target_sdk_units'] for j in range(6)]
    assert sent == [round(value, 3) for value in planned]
    # This is the approved MoveIt numerical endpoint, not an exact ideal pose.
    actual = np.asarray(result['final_angles'][:6])
    error = actual - planned
    assert error[2] == pytest.approx(1.166293096938791)
    assert error[4] == pytest.approx(1.8191228922649976)
    assert np.max(abs(error)) > result['endpoint_tolerance_deg'] == 1.
    last_angle_time = received(evidence['events'], address=2, funcode=1)[-1][0]
    assert commands[-1]['t'] - last_angle_time == pytest.approx(.41312603803817183)
    assert sum(e['t'] > last_angle_time for e in commands) == 17
    assert result['completed'] is False


class SimulatedAfterLogTransport:
    """Start from the saved stale age; later frames are simulation only."""
    def __init__(self, result, *, synthetic_frame_at=None):
        self.t = result['final_valid_feedback_age_s']
        self.q_time = 0.
        self.raw_q_time = self.t - result['final_raw_feedback_age_s']
        self.q = np.asarray(result['final_angles'], dtype=float)
        self.q_seq = 49
        self.synthetic_frame_at = synthetic_frame_at
        self.synthetic_frame_delivered = False
        self.status = result['final_status'].copy()
        self.status_observations = 0
        self.acks = result['receipts']
        self.commands = 0

    def now(self):
        return self.t

    def snapshot(self):
        self.status_observations += 1
        return dict(q=self.q.copy(), q_time=self.q_time, raw_q_time=self.raw_q_time,
                    q_seq=self.q_seq, status=self.status.copy(),
                    status_time={key: self.t for key in self.status}, fault=None,
                    feedback_pending=False, feedback_rejected_count=0,
                    feedback_pause_total_s=0.)

    def wait(self, seconds):
        self.t += seconds
        self.acks += 1  # Fresh receipts and status cannot refresh the angle age.
        if (self.synthetic_frame_at is not None and not self.synthetic_frame_delivered
                and self.t >= self.synthetic_frame_at):
            self.q += np.array([0., .1, -.1, 0., -.2, 0., 0.])
            self.q_time = self.raw_q_time = self.synthetic_frame_at
            self.q_seq += 1
            self.synthetic_frame_delivered = True

    def send(self, *args, **kwargs):
        raise AssertionError('A recovery wait must never produce a motion command')


def test_saved_stale_age_can_recover_with_explicitly_synthetic_frame_at_075s(evidence):
    transport = SimulatedAfterLogTransport(evidence['result'], synthetic_frame_at=.75)
    recovery = sdk_executor.FeedbackRecovery(transport)
    q, snapshot = recovery.read()
    assert transport.synthetic_frame_delivered and snapshot['q_seq'] == 50
    assert snapshot['q_time'] == .75
    assert .75 <= transport.now() <= .76
    assert np.array_equal(q, transport.q)
    assert recovery.metrics()['feedback_gap_recoveries'] == 1
    assert .13 < recovery.gap_wait_s < .15
    assert transport.commands == 0


def test_saved_stale_age_hard_stops_at_two_seconds_despite_simulated_status(evidence):
    transport = SimulatedAfterLogTransport(evidence['result'])
    recovery = sdk_executor.FeedbackRecovery(transport)
    with pytest.raises(RuntimeError, match='All joint feedback recovery timeout') as caught:
        recovery.read()
    assert transport.now() == pytest.approx(2.)  # From last valid angle, not wait start.
    assert transport.q_time == 0. and transport.q_seq == 49
    assert transport.status_observations > 100 and transport.acks > evidence['result']['receipts']
    assert caught.value.feedback_diagnostics['final_valid_feedback_age_s'] == pytest.approx(2.)
    assert recovery.metrics()['feedback_gap_recoveries'] == 0
    assert transport.commands == 0


@pytest.mark.parametrize('sample_stamp',[1.99,2.,2.01])
def test_delayed_waiter_uses_frame_receive_time_for_the_hard_deadline(evidence,sample_stamp):
    class DelayedWakeTransport(SimulatedAfterLogTransport):
        def wait(self,seconds):
            # The accepted sample arrives first; the consumer is scheduled late.
            self.t=2.02
            self.q_time=self.raw_q_time=sample_stamp
            self.q_seq=50
    transport=DelayedWakeTransport(evidence['result'])
    recovery=sdk_executor.FeedbackRecovery(transport)
    if sample_stamp<=2.:
        _,snapshot=recovery.read()
        assert snapshot['q_time']==sample_stamp
        assert recovery.gap_recoveries==1
    else:
        with pytest.raises(RuntimeError,match='All joint feedback recovery timeout'):
            recovery.read()
    assert transport.commands==0
