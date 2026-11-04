"""Exercise final artifacts using saved synthetic logs, without any device I/O."""
import csv
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import reporting

CLOCK = {'monotonic': 100.0, 'unix': 1700000000.0}


def rx(stamp, q, **extra):
    return dict(t=stamp, kind='output', line='[rx] ' + json.dumps({
        'address': 2, 'funcode': 1, 'data': {f'angle{i}': value for i, value in enumerate(q)}}), **extra)


def log(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row) + '\n' for row in records))


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def base_result(**extra):
    return dict(mode='live', status='failed', grasp_result='unknown', total_time_s=10.0, **extra)


def test_rx_feedback_and_command_attempts_are_separate(tmp_path):
    actual = [1, 2, 3, 4, 5, 6, 41.25]
    target = [10, 20, 30, 40, 50, 60, 61.0]
    folder = tmp_path / 'execution/00_approach'
    log(folder / 'executed/transport.jsonl', [
        {'t': 102.0, 'kind': 'command', 'sdk': target},
        rx(102.2, actual),
        {'t': 102.3, 'kind': 'output', 'line': '[tx] ' + json.dumps({'sdk': target})},
        {'t': 102.4, 'kind': 'output', 'line': '[ok] received'},
        {'t': 103.0, 'kind': 'command', 'joint': 6, 'target_sdk_units': 55},
    ])
    log(folder / 'observation/readonly.jsonl', [rx(101.0, [0] * 7)])
    result = reporting.export_report(tmp_path, base_result(), CLOCK)
    samples = read_csv(tmp_path / 'joint_positions.csv')
    commands = read_csv(tmp_path / 'commands.csv')
    assert len(samples) == 2 and len(commands) == 2
    assert float(samples[1]['J0_deg']) == actual[0]
    assert float(samples[1]['J6_sdk_position']) == actual[6]
    assert 'J6_deg' not in samples[1]
    assert samples[1]['time_iso_utc8'].endswith('+08:00')
    assert float(samples[1]['elapsed_s']) == pytest.approx(2.2)
    assert samples[1]['stage'] == '00_approach'
    assert samples[1]['data_origin'] == 'robot_rx'
    assert json.loads(commands[0]['target_sdk_json']) == target
    assert commands[1]['target_units'] == 'SDK_position'
    assert commands[1]['delivery_status'] == 'attempt_recorded_delivery_not_established'
    assert result['feedback_samples'] == result['actual_feedback_samples'] == 2
    assert result['command_count'] == 2


def test_rejected_raw_angles_stay_auditable_but_not_in_valid_position_export(tmp_path):
    path=tmp_path/'execution/01_pregrasp/executed/transport.jsonl'
    normal=[0.,0.,0.,0.,39.4,0.,35.9]
    bad=normal.copy();bad[4]=-165.3
    recovered=normal.copy();recovered[4]=39.5
    records=[dict(t=100.,kind='feedback_filter',enabled=True)]
    for stamp,q,accepted in [(100.1,normal,True),(100.4,bad,False),(100.5,recovered,True)]:
        raw=rx(stamp,q);records.append(raw)
        message=json.loads(raw['line'].split('[rx] ',1)[1])
        event=dict(message=message,received_monotonic_s=stamp,pause_total_s=0. if stamp<100.5 else .1)
        prefix='accepted' if accepted else 'rejected'
        records.append(dict(t=stamp+.001,kind='output',
                            line=f'[feedback-{prefix}] '+json.dumps(event)))
    log(path,records)
    original=path.read_bytes()
    diagnostics={}
    samples=list(reporting.feedback_records(tmp_path/'execution',diagnostics))
    assert [row['q'][4] for row in samples]==[39.4,39.5]
    assert [row['t'] for row in samples]==[100.1,100.5]
    assert diagnostics['rejected_angle_frames']==1
    assert path.read_bytes()==original and '-165.3' in path.read_text()


@pytest.mark.parametrize('prefix',['accepted','rejected'])
def test_per_axis_marker_exports_merged_safe_angles_and_original_axis_ages(tmp_path,prefix):
    path=tmp_path/'execution/01_pregrasp/executed/transport.jsonl'
    raw=[0.,1.,2.,3.,-165.3,5.,36.]
    safe=[0.,1.,2.,3.,39.4,5.,36.]
    times=[101.]*7;times[4]=100.2
    rejections=[dict(joint=4,reason='speed_jump',delta=204.7,dt_s=.1,threshold_deg=.15)]
    message=json.loads(rx(101.,raw)['line'].split('[rx] ',1)[1])
    event=dict(message=message,received_monotonic_s=101.,merged_angles=safe,
               joint_times_s=times,joint_sequences=[2,2,2,2,1,2,2],
               joint_pending=[False,False,False,False,True,False,False],accepted_joints=[0,1,2,3,5,6],
               rejected_joints=[4],joint_rejections=rejections)
    records=[dict(t=100.,kind='feedback_filter',enabled=True),rx(101.,raw),
             dict(t=101.001,kind='output',line=f'[feedback-{prefix}] '+json.dumps(event))]
    log(path,records);original=path.read_bytes()
    diagnostics={}
    samples=list(reporting.feedback_records(tmp_path/'execution',diagnostics))
    assert len(samples)==1 and samples[0]['q']==safe
    assert samples[0]['joint_times_s']==times and samples[0]['joint_sequences'][4]==1
    assert samples[0]['joint_feedback_age_s'][4]==pytest.approx(.8)
    assert samples[0]['stale_feedback_joints']==[4] and samples[0]['pending_feedback_joints']==[4]
    assert samples[0]['feedback_warning']
    assert samples[0]['rejected_joints']==[4] and samples[0]['joint_rejections']==rejections
    assert diagnostics['rejected_angle_frames']==diagnostics['frames_with_rejected_joints']==1
    assert diagnostics['partially_rejected_angle_frames']==1
    reporting.export_report(tmp_path,base_result(),CLOCK)
    row=read_csv(tmp_path/'joint_positions.csv')[0]
    assert float(row['J4_deg'])==39.4
    assert json.loads(row['joint_times_s_json'])==times
    assert json.loads(row['joint_sequences_json'])==[2,2,2,2,1,2,2]
    assert json.loads(row['pending_feedback_joints_json'])==[4]
    assert json.loads(row['stale_feedback_joints_json'])==[4]
    assert json.loads(row['rejected_joints_json'])==[4]
    assert json.loads(row['joint_rejections_json'])==rejections
    assert row['feedback_value_source']=='merged_valid_axes'
    assert path.read_bytes()==original and '-165.3' in path.read_text()


def test_all_rejected_marker_counts_one_frame_and_no_partial_frame(tmp_path):
    path=tmp_path/'execution/01_pregrasp/executed/transport.jsonl'
    event=dict(received_monotonic_s=101.,merged_angles=[0.]*7,
               accepted_joints=[],rejected_joints=list(range(7)),joint_rejections=[])
    log(path,[dict(t=101.,kind='output',line='[feedback-rejected] '+json.dumps(event))])
    diagnostics={}
    samples=list(reporting.feedback_records(tmp_path/'execution',diagnostics))
    assert len(samples)==1 and samples[0]['q']==[0.]*7
    assert diagnostics['rejected_angle_frames']==diagnostics['frames_with_rejected_joints']==1
    assert diagnostics.get('partially_rejected_angle_frames',0)==0


def test_malformed_or_truncated_logs_do_not_abort_report(tmp_path):
    path = tmp_path / 'execution/03_lift/executed/transport.jsonl'
    log(path, [None, [], 15, {'kind': 'output', 't': True, 'line': '[rx] {}'},
               {'kind': 'output', 't': 101, 'line': 5},
               {'kind': 'output', 't': 101, 'line': '[rx] null'},
               {'kind': 'output', 't': 101, 'line': '[rx] {"address":2,"funcode":1,"data":{}}'},
               rx(101, [True] + [0] * 6), rx(101, [float('nan')] + [0] * 6),
               {'kind': 'command', 't': 101, 'sdk': [1, 2]}, rx(102, [1] * 7)])
    with path.open('a') as stream:
        stream.write('{"truncated":\n')
    result = reporting.export_report(tmp_path, base_result(), CLOCK)
    assert result['feedback_samples'] == 1
    assert result['command_count'] == 0
    assert result['log_diagnostics']['malformed_jsonl_rows'] >= 4
    assert result['log_diagnostics']['malformed_joint_feedback_rows'] >= 3
    assert result['log_diagnostics']['malformed_command_rows'] == 1
    assert (tmp_path / 'report.html').is_file()


def test_stage_times_include_failed_planning_and_separate_execution(tmp_path):
    origin = CLOCK['unix']
    events = [
        dict(time=origin + 1, state='observing', stage='approach'),
        dict(time=origin + 2, state='planning', stage='approach'),
        dict(time=origin + 3, state='executing', stage='approach'),
        dict(time=origin + 6, state='stage_completed', stage='approach'),
        dict(time=origin + 7, state='observing', stage='lift'),
        dict(time=origin + 8, state='planning', stage='lift'),
        dict(time=origin + 9, state='stopped_no_recovery'),
        None, dict(time='bad', state='executing', stage='open'),
    ]
    log(tmp_path / 'execution/events.jsonl', events)
    result = reporting.export_report(tmp_path, base_result(), CLOCK)
    assert result['stage_times'] == {'approach': 5.0, 'lift': 2.0}
    assert result['stage_timings']['approach']['execution_s'] == 3.0
    assert result['stage_timings']['lift']['execution_s'] == 0.0
    assert result['stage_timings']['lift']['outcome'] == 'stopped'


def test_cancelled_stage_without_terminal_event_keeps_elapsed_time(tmp_path):
    log(tmp_path / 'execution/events.jsonl', [
        dict(time=CLOCK['unix'] + 2, state='observing', stage='close'),
        dict(time=CLOCK['unix'] + 3, state='executing', stage='close'),
    ])
    result = base_result()
    result['status'] = 'cancelled'
    exported = reporting.export_report(tmp_path, result, CLOCK)
    assert exported['stage_timings']['close']['elapsed_s'] == 8
    assert exported['stage_timings']['close']['execution_s'] == 7
    assert exported['stage_timings']['close']['outcome'] == 'interrupted'


@pytest.mark.parametrize('visual_status,label', [('unknown', '无法确认'), ('failure', '未抓起（视觉证据）')])
def test_visual_unknown_and_failure_stay_distinct(tmp_path, visual_status, label):
    details = {'completed': True, 'visual_verifications': {'lift': {
        'status': visual_status, 'reason': 'deterministic observation result'}}}
    result = base_result(details=details)
    result.update(status='completed', grasp_result='success')
    exported = reporting.export_report(tmp_path, result, CLOCK)
    assert exported['grasp_result'] == visual_status
    assert exported['grasp_verified_by_visual_evidence'] is False
    assert label in (tmp_path / 'report.html').read_text()


def test_completed_alone_does_not_establish_grasp_or_release(tmp_path):
    result = base_result(details={'completed': True, 'physical_grasp_verified': True})
    result.update(status='completed', grasp_result='success')
    exported = reporting.export_report(tmp_path, result, CLOCK)
    assert exported['grasp_result'] == 'unknown'
    assert exported['grasp_verified_by_visual_evidence'] is False
    assert exported['jaw_release_measured'] is False


def test_preparation_failure_still_creates_header_only_csv_and_report(tmp_path):
    folder = tmp_path / 'not_started'
    result = base_result()
    result.update(status='blocked', grasp_result='not_attempted', total_time_s=0)
    exported = reporting.export_report(folder, result, CLOCK)
    assert exported['feedback_samples'] == exported['command_count'] == 0
    assert exported['stage_times'] == {}
    assert read_csv(folder / 'joint_positions.csv') == []
    assert read_csv(folder / 'commands.csv') == []
    assert 'J6_sdk_position' in (folder / 'joint_positions.csv').read_text(encoding='utf-8-sig')
    assert 'target_sdk_json' in (folder / 'commands.csv').read_text(encoding='utf-8-sig')
    assert '未执行抓取' in (folder / 'report.html').read_text()
    assert exported['execution_started_at'] is None
    assert exported['execution_ended_at'] is None
    assert exported['session_started_at'] == reporting._iso(CLOCK['unix'])


def test_preparation_failure_reports_measured_steps_without_execution_samples(tmp_path):
    log(tmp_path / 'perception/timing.jsonl', [
        dict(kind='mark', name='recognition_start', monotonic=100.1),
        dict(kind='span', name='read_only_feedback_discovery_and_stationarity', category='stage_wait',
             start=100.2, end=101.6, duration_s=1.4, outcome='completed'),
        dict(kind='span', name='wrist_http_status_and_image', category='acquisition',
             start=101.6, end=101.65, duration_s=.05, outcome='completed'),
        dict(kind='span', name='local_opencv_foreground', category='recognition',
             start=101.65, end=101.66, duration_s=.01, outcome='completed'),
        dict(kind='mark', name='perception_complete', monotonic=101.7),
    ])
    log(tmp_path / 'perception/readonly.jsonl', [rx(100.5, [0] * 7)])
    result = base_result(cycle_result='not_attempted')
    result.update(status='blocked', grasp_result='not_attempted', total_time_s=1.8)
    exported = reporting.export_report(tmp_path, result, CLOCK)
    assert exported['preparation_time_s'] == 1.8
    assert exported['perception_time_s'] == pytest.approx(1.6)
    assert [item['duration_s'] for item in exported['preparation_timings']] == [1.4, .05, .01]
    assert exported['stage_times'] == {}
    assert exported['cycle_result'] == 'not_attempted'
    assert exported['actual_feedback_samples'] == 0
    assert exported['execution_started_at'] is None
    assert read_csv(tmp_path / 'joint_positions.csv') == []
    page = (tmp_path / 'report.html').read_text()
    assert '腕部图像采集' in page and 'OpenCV 前景识别' in page
    assert '抓放阶段未开始' in page


def test_preparation_timing_handles_bad_rows_and_preserves_full_preparation_duration(tmp_path):
    log(tmp_path / 'perception/timing.jsonl', [
        dict(kind='span', name='outer', category='acquisition', start=90, end=92, duration_s=2),
        dict(kind='span', name='nested', category='recognition', start=91, end=91.5, duration_s=.5),
        dict(kind='span', name='invalid', category='pose', start=92, end=91, duration_s=-1),
        dict(kind='span', name='mismatch', category='pose', start=92, end=93, duration_s=2),
        dict(kind='span', name='missing', category='pose', start=93, end=94),
    ])
    log(tmp_path / 'execution/03_lift/executed/transport.jsonl', [rx(101, [0] * 7)])
    result = reporting.export_report(tmp_path, base_result(preparation_time_s=8.0), CLOCK)
    assert result['preparation_time_s'] == 8.0
    assert result['perception_time_s'] == 2.0
    assert len(result['preparation_timings']) == 2
    assert result['log_diagnostics']['invalid_preparation_timing_rows'] == 3
    assert result['execution_started_at'] == reporting._iso(CLOCK['unix'])


def test_synthetic_feedback_is_never_counted_as_actual(tmp_path):
    log(tmp_path / 'execution/03_lift/executed/transport.jsonl', [rx(101, [0] * 7, synthetic=True)])
    result = base_result()
    result.update(mode='demo', status='demo_completed', grasp_result='simulated_success')
    exported = reporting.export_report(tmp_path, result, CLOCK)
    assert exported['simulated_feedback_samples'] == 1
    assert exported['actual_feedback_samples'] == 0
    assert exported['grasp_verified_by_visual_evidence'] is False
    assert read_csv(tmp_path / 'joint_positions.csv')[0]['data_origin'] == 'synthetic'
    assert '模拟成功（无实机）' in (tmp_path / 'report.html').read_text()


def test_visual_failure_timing_includes_verification_but_not_as_motion(tmp_path):
    log(tmp_path / 'execution/events.jsonl', [
        dict(time=CLOCK['unix'] + 1, state='observing', stage='lift'),
        dict(time=CLOCK['unix'] + 2, state='executing', stage='lift'),
        dict(time=CLOCK['unix'] + 5, state='stage_completed', stage='lift'),
        dict(time=CLOCK['unix'] + 7, state='automatic_lift_result', stage='lift', status='unknown'),
        dict(time=CLOCK['unix'] + 8, state='stopped_no_recovery'),
    ])
    result = reporting.export_report(tmp_path, base_result(error='<script>bad()</script>'), CLOCK)
    assert result['stage_timings']['lift']['elapsed_s'] == 7
    assert result['stage_timings']['lift']['execution_s'] == 3
    assert result['stage_timings']['lift']['outcome'] == 'visual_unknown'
    page = (tmp_path / 'report.html').read_text()
    assert '<script>bad()</script>' not in page
    assert '&lt;script&gt;' in page
