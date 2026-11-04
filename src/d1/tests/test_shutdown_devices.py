"""Shutdown requests remain truthful and target only exact camera services."""
import json
from pathlib import Path
import signal
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from app import camera_services as camera
from app import backend as bm
from app.backend import Backend
from app.device_control import POWER_OFF_UNAVAILABLE
from app.gui import GraspApp


def test_remote_stop_uses_known_camera_role_and_never_starts_a_server(tmp_path,monkeypatch):
    import shlex
    manager=camera.CameraServices(tmp_path)
    run=Mock(return_value=SimpleNamespace(returncode=0,stdout='{"stopped":true}\n',stderr=''))
    monkeypatch.setattr(camera.subprocess,'run',run)
    assert manager.stop_remote(camera.CAMERAS[0])['stopped']
    argv=shlex.split(run.call_args.args[0][-1])
    assert argv[2]==camera.REMOTE_STOP
    assert argv[3:]==['wrist','18760','CC1365301EY',camera.SERVER]
    assert 'subprocess.Popen' not in camera.REMOTE_STOP
    compile(camera.REMOTE_STOP,'remote_camera_stop','exec')


def test_local_stop_matches_only_exact_single_camera_forward(tmp_path,monkeypatch):
    proc=tmp_path/'proc';proc.mkdir()
    variants=[
        ['ssh','-N','-L','127.0.0.1:18760:127.0.0.1:18760',camera.HOST],
        ['ssh','-N','-L','127.0.0.1:18760:127.0.0.1:18760','-L','123:localhost:123',camera.HOST],
        ['ssh','-N','-L','127.0.0.1:18760:127.0.0.1:18760','other@host'],
        ['python3','robot_controller.py'],
    ]
    for i,args in enumerate(variants,100):
        p=proc/str(i);p.mkdir();(p/'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
    original=Path
    monkeypatch.setattr(camera,'Path',lambda value:proc if value=='/proc' else original(value))
    assert camera.CameraServices(tmp_path).matching_forwards(camera.CAMERAS[0])==[100]


def test_stop_forward_sends_sigterm_only_to_matching_tunnel(tmp_path,monkeypatch):
    manager=camera.CameraServices(tmp_path)
    monkeypatch.setattr(manager,'matching_forwards',Mock(side_effect=[[123],[],[]]))
    monkeypatch.setattr(manager,'port_open',lambda p:False)
    kill=Mock();monkeypatch.setattr(camera.os,'kill',kill)
    assert manager.stop_forward(camera.CAMERAS[0])['stopped']
    kill.assert_called_once_with(123,signal.SIGTERM)


def test_unknown_listener_not_killed_or_reported_closed(tmp_path,monkeypatch):
    manager=camera.CameraServices(tmp_path)
    monkeypatch.setattr(camera,'STOP_TIMEOUT_S',0.)
    monkeypatch.setattr(manager,'matching_forwards',lambda c:[])
    monkeypatch.setattr(manager,'port_open',lambda p:True)
    kill=Mock();monkeypatch.setattr(camera.os,'kill',kill)
    with pytest.raises(RuntimeError,match='未强制终止未知进程'):manager.stop_forward(camera.CAMERAS[0])
    kill.assert_not_called()


def test_partial_camera_stop_and_remote_failure_preserve_detailed_status(tmp_path,monkeypatch):
    key=tmp_path/'key';key.touch();monkeypatch.setattr(camera,'KEY',key)
    manager=camera.CameraServices(tmp_path)
    def stop(c):
        if c['role']=='global':raise RuntimeError('远端未响应')
        return dict(stopped=True)
    monkeypatch.setattr(manager,'stop_remote',stop)
    local=Mock(return_value=dict(stopped=True));monkeypatch.setattr(manager,'stop_forward',local)
    result=manager.stop()
    assert not result['completed']
    assert result['cameras']['wrist']['stopped']
    assert not result['cameras']['global']['stopped']
    assert '全局相机：远端未响应' in result['error']
    assert local.call_count==1


def test_shutdown_runs_camera_stop_after_unsupported_power_off_and_never_claims_all_off(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    backend=Backend();backend._review={'ready':True};backend.run_dir=tmp_path/'old_grasp'
    actions=[]
    def run(argv,log_path,timeout,monitor_run):
        assert not monitor_run and backend._review is None
        with pytest.raises(RuntimeError,match='正在处理设备请求'):backend.start_cameras()
        action=argv[argv.index('--action')+1];actions.append(action)
        output=Path(argv[-1]);output.mkdir(parents=True)
        if action=='power_off':
            report=dict(completed=False,error=POWER_OFF_UNAVAILABLE,robot_commands_sent=0)
        else:report=dict(completed=True,cameras={role:dict(stopped=True) for role in ['wrist','global']})
        (output/'result.json').write_text(json.dumps(report))
        if action=='power_off':raise RuntimeError('unsupported')
    monkeypatch.setattr(backend,'_run',run)
    result=backend.shutdown_devices()
    assert actions==['power_off','stop']
    assert not result['completed'] and result['camera_stop']['completed']
    assert '没有实现下电' in result['error']
    assert result['robot_commands_sent']==result['position_commands_sent']==0
    assert backend._lockfile is None and backend._active_device_control is None
    assert not backend.run_dir.exists()


def test_demo_shutdown_has_no_device_or_service_io(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    backend=Backend(mode='demo');backend._demo_device_status['power_status']=1
    monkeypatch.setattr(backend,'_run',Mock(side_effect=AssertionError('No real shutdown in demo')))
    result=backend.shutdown_devices()
    assert result['completed'] and result['synthetic']
    assert result['power_off']['status']['power_status']==0
    assert result['robot_commands_sent']==0


def stub():
    app=GraspApp.__new__(GraspApp)
    app.busy='';app.closing=False;app.review={'ready':True};app.log_dir=None
    app.backend=Mock()
    for name in ['signature_label','stage_detail','state_label','camera_hint','result_text','notebook']:setattr(app,name,Mock())
    for name in ['_draw_path','_append_log','_start_job','_set_text','_refresh_controls']:setattr(app,name,Mock())
    return app


def test_shutdown_button_dispatches_one_worker_and_is_disabled_during_execution():
    app=stub();app.shutdown_devices()
    assert app.review is None
    kind,operation=app._start_job.call_args.args
    assert kind=='shutdown_devices'
    operation();app.backend.shutdown_devices.assert_called_once_with()
    app.backend.execute.assert_not_called();app.backend.prepare.assert_not_called()
    app=stub();app.busy='execute';app.shutdown_devices();app._start_job.assert_not_called()


def test_partial_shutdown_ui_does_not_display_power_off_as_success():
    app=stub()
    app._show_shutdown_devices(dict(completed=False,power_off=dict(completed=False,error=POWER_OFF_UNAVAILABLE),
        camera_stop=dict(completed=True,cameras={r:dict(stopped=True) for r in ['wrist','global']})))
    text=app._set_text.call_args.args[1]
    assert '机械臂下电：未确认下电' in text and '没有实现下电' in text
    assert '腕部相机：已关闭' in text and '全局相机：已关闭' in text
    assert '设备关闭完成' not in text
    assert app.device_snapshot.get('status',{}).get('power_status') is None
