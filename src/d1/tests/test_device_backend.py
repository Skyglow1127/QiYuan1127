"""Device controls use mock subprocesses and temporary logs; no hardware IO."""
import json
from pathlib import Path
import threading
import time

import pytest

from app import backend as module
from app.backend import Backend, DEFAULTS, ProcessFailure, device_status_from_observation
from app.reporting import load, write_json


@pytest.fixture
def backend(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'APP',tmp_path)
    events=[]
    instance=Backend(emit=events.append)
    instance.run_dir=tmp_path/'existing_grasp'
    instance.run_dir.mkdir()
    write_json(instance.run_dir/'result.json',{'status':'existing evidence'})
    instance._review={'ready':True,'original':'review'}
    return instance,events


def save_result(argv,**updates):
    output=Path(argv[argv.index('--output')+1])
    output.mkdir(parents=True)
    result=dict(completed=True,status={'power_status':1,'enable_status':0,'error_status':0},
                status_fresh=True,snapshot_received_at=time.time(),
                robot_commands_sent=0,sdk_angles=[0.,-20.,40.,0.,20.,0.,36.])
    result.update(updates)
    write_json(output/'result.json',result)
    return output,result


def test_refresh_uses_separate_logs_preserves_review_and_grasp(backend,monkeypatch):
    instance,events=backend
    folder=instance.run_dir
    review=instance._review
    calls=[]
    def run(argv,log_path,timeout,monitor_run):
        assert monitor_run is False
        calls.append((argv,log_path))
        save_result(argv)
    monkeypatch.setattr(instance,'_run',run)
    result=instance.control_device('refresh')
    assert result['completed'] is True
    assert instance.run_dir==folder and instance._review is review
    assert load(folder/'result.json')['status']=='existing evidence'
    assert Path(result['log_dir']).parent==module.APP/'logs/device_controls'
    assert calls[0][0][2]=='refresh'
    assert load(Path(result['log_dir'])/'result.json')==result
    assert [event['kind'] for event in events]==['device_status','joint','log']


@pytest.mark.parametrize('action',['power_on','enable'])
def test_mutating_request_invalidates_review_even_when_child_refuses(backend,monkeypatch,action):
    instance,events=backend
    def run(argv,log_path,timeout,monitor_run):
        assert instance._review is None
        save_result(argv,completed=False,error='驱动不支持该请求')
        Path(log_path).write_text('ignored trailing JSON\n')
        raise ProcessFailure(argv,1,log_path)
    monkeypatch.setattr(instance,'_run',run)
    result=instance.control_device(action)
    assert result['completed'] is False and result['error']=='驱动不支持该请求'
    assert result['status']['power_status']==1
    assert result['robot_commands_sent']==0
    assert instance._review is None
    assert events[0]['data']==result


def test_nonzero_exit_cannot_be_success_despite_report(backend,monkeypatch):
    instance,_=backend
    def run(argv,log_path,timeout,monitor_run):
        save_result(argv)
        raise RuntimeError('child failed after status write')
    monkeypatch.setattr(instance,'_run',run)
    result=instance.control_device('refresh')
    assert result['completed'] is False
    assert result['status_fresh'] is True
    assert result['error']=='child failed after status write'


def test_exit_zero_without_completion_report_is_failure(backend,monkeypatch):
    instance,_=backend
    monkeypatch.setattr(instance,'_run',lambda *a,**kw:None)
    result=instance.control_device('refresh')
    assert result['completed'] is False
    assert result['status_fresh'] is False
    assert result['robot_commands_sent']==0
    assert result['error']


def test_demo_state_persists_and_enable_never_connects(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'APP',tmp_path)
    instance=Backend(mode='demo')
    def no_subprocess(*args,**kwargs):
        pytest.fail('demo device actions must not create a subprocess')
    monkeypatch.setattr(instance,'_run',no_subprocess)
    assert instance.control_device('refresh')['status']['power_status']==0
    powered=instance.control_device('power_on')
    assert powered['completed'] and powered['status']['power_status']==1
    assert instance.control_device('refresh')['status']['power_status']==1
    denied=instance.control_device('enable')
    assert denied['completed'] is False and denied['status']['enable_status']==0
    assert denied['mode']=='demo' and denied['synthetic'] is True
    assert denied['robot_commands_sent']==0
    assert '使能' in denied['error']


def test_same_backend_cannot_overlap_control_prepare_or_execute(backend,monkeypatch):
    instance,_=backend
    started=threading.Event()
    finish=threading.Event()
    def run(argv,log_path,timeout,monitor_run):
        started.set()
        assert finish.wait(5.)
        save_result(argv)
    monkeypatch.setattr(instance,'_run',run)
    outcomes=[]
    worker=threading.Thread(target=lambda:outcomes.append(instance.control_device('refresh')))
    worker.start()
    try:
        assert started.wait(5.)
        lease=instance._lockfile
        review=instance._review
        for action in (lambda:instance.prepare(DEFAULTS),
                       lambda:instance.execute({},'approved'),
                       lambda:instance.control_device('refresh')):
            with pytest.raises(RuntimeError,match='当前程序正在处理设备请求'):
                action()
            assert instance._lockfile is lease and not lease.closed
            assert instance._review is review
    finally:
        finish.set()
        worker.join(5.)
    assert not worker.is_alive() and outcomes[0]['completed']
    assert instance._lockfile is None


def test_other_backend_cannot_claim_device_during_control(backend,monkeypatch):
    instance,_=backend
    second=Backend()
    def run(argv,log_path,timeout,monitor_run):
        with pytest.raises(RuntimeError,match='另一个程序正在使用设备'):
            second._claim_device()
        assert second._lockfile is None
        save_result(argv)
    monkeypatch.setattr(instance,'_run',run)
    assert instance.control_device('refresh')['completed']
    second._claim_device()
    second._release_device()


def test_cancel_refresh_does_not_write_stop_into_existing_grasp(backend,monkeypatch):
    instance,_=backend
    folder=instance.run_dir
    review=instance._review
    def run(*args,**kwargs):
        instance.cancel()
        raise module.Cancelled('cancelled status request')
    monkeypatch.setattr(instance,'_run',run)
    result=instance.control_device('refresh')
    assert not result['completed'] and result['robot_commands_sent']==0
    assert not (folder/'STOP').exists()
    assert instance._review is review
    assert instance._active_device_control is None


def test_lock_failure_still_saves_independent_control_result(backend,monkeypatch):
    instance,_=backend
    owner=Backend()
    owner._claim_device()
    try:
        result=instance.control_device('refresh')
        assert not result['completed']
        assert '另一个程序' in result['error']
        assert load(Path(result['log_dir'])/'result.json')==result
        assert instance._lockfile is None
        assert owner._lockfile is not None and not owner._lockfile.closed
    finally:
        owner._release_device()


def test_invalid_action_does_not_invalidate_review_or_create_logs(backend):
    instance,_=backend
    original=instance._review
    with pytest.raises(ValueError): instance.control_device('damping')
    assert instance._review is original
    assert not (module.APP/'logs').exists()


def test_saved_status_preserves_old_timestamp_and_requires_all_ages():
    now=time.time()
    source=dict(status={'power_status':1,'enable_status':0,'error_status':0},
                snapshot_received_at=now-60.,status_age_s={'power_status':.01,'enable_status':.01,'error_status':.01})
    result=device_status_from_observation(source)
    assert result['status_fresh'] is False
    assert result['snapshot_received_at']==source['snapshot_received_at']
    source['snapshot_received_at']=time.time()
    assert device_status_from_observation(source)['status_fresh'] is True
    del source['status_age_s']['enable_status']
    assert device_status_from_observation(source)['status_fresh'] is False


def test_preparation_failure_exposes_observed_device_status(backend,monkeypatch):
    instance,events=backend
    monkeypatch.setattr(module,'check_environment',lambda:[])
    def run(argv,log_path,timeout):
        output=Path(argv[argv.index('--output')+1])
        output.mkdir(parents=True)
        write_json(output/'wrist_observation.json',dict(
            status={'power_status':0,'enable_status':0,'error_status':0},
            sdk_angles=[0.,-20.,40.,0.,20.,0.,36.],snapshot_received_at=time.time(),
            status_age_s={'power_status':.01,'enable_status':.01,'error_status':.01}))
        raise RuntimeError('目标不在画面内')
    monkeypatch.setattr(instance,'_run',run)
    review=instance.prepare(DEFAULTS)
    assert not review['ready']
    assert review['device_status']['status']['power_status']==0
    assert any(e['kind']=='device_status' and e['data']==review['device_status'] for e in events)
