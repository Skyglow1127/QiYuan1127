"""Replay the serial-side status that arrives after the upstream receipt ACK."""
import json
from test_feedback_jump_filter import reader, angle_message


def diagnostic(event, **extra):
    return '[rx] '+json.dumps(dict(address=2,funcode=90,seq=0,
        data=dict(schema='d1_uart_integrity_v1',event=event,**extra)))


def test_board_rejection_after_receipt_stops_future_targets():
    t=reader(False);t.t=10.
    t.proc.stdout=iter(['[ok] received\n',diagnostic('command_rejected',
        command_seq=123,reason='serial short write',write_ok=False)+'\n'])
    t._read()
    assert t.acks==1
    assert 'serial short write' in t.snapshot()['fault']
    assert t.snapshot()['onboard_diagnostics']['command_seq']==123


def test_query_diagnostics_do_not_refresh_position_or_certify_arrival():
    t=reader(False);t.t=10.
    t.proc.stdout=iter(['[rx] '+json.dumps(angle_message([0.]*7))+'\n'])
    t._read();first=t.snapshot();t.t=11.
    t.proc.stdout=iter([diagnostic('feedback_scan',queries_invalid=1,dropped_frames=1)+'\n',
        diagnostic('command_write',write_ok=True,command_seq=123)+'\n'])
    t._read();last=t.snapshot()
    assert last['q_time']==first['q_time']==10.
    assert last['q_seq']==first['q_seq']==1
    assert t.acks==0 and last['fault'] is None
    assert last['onboard_diagnostics_time']==11.
