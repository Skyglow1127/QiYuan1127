"""Camera service button regressions; all HTTP/SSH/process IO is fake."""
import io
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from app import camera_services as camera
from app import backend as backend_module
from app.backend import Backend
from app.gui import GraspApp


def healthy():
    return dict(ready=True,capture_fps=30.,frame_indices=[10,15],checked_at=100.)


@pytest.fixture
def manager(tmp_path,monkeypatch):
    key=tmp_path/'test-key';key.touch()
    monkeypatch.setattr(camera,'KEY',key)
    monkeypatch.setattr(camera,'READY_TIMEOUT_S',0.)
    return camera.CameraServices(tmp_path/'services')


def test_healthy_camera_reused_without_ssh_or_restarting(manager,monkeypatch):
    monkeypatch.setattr(manager,'probe',lambda c:healthy())
    remote=Mock(side_effect=AssertionError('Healthy camera must not restart'))
    monkeypatch.setattr(manager,'ensure_remote',remote)
    result=manager.start()
    assert result['completed'] and result['robot_commands_sent']==0
    assert all(c['action']=='reused' for c in result['cameras'].values())
    remote.assert_not_called()


@pytest.mark.parametrize('already_forwarded',[False,True])
def test_missing_server_and_optional_forward_restored(manager,monkeypatch,already_forwarded):
    probe=Mock(side_effect=[ConnectionRefusedError('absent'),healthy()])
    remote=Mock(return_value=dict(action='started'))
    forward=Mock(return_value=SimpleNamespace(poll=lambda:None))
    monkeypatch.setattr(manager,'probe',probe)
    monkeypatch.setattr(manager,'ensure_remote',remote)
    monkeypatch.setattr(manager,'port_open',lambda p:already_forwarded)
    monkeypatch.setattr(manager,'start_forward',forward)
    result=manager.ensure(camera.CAMERAS[0])
    assert result['ready'] and result['action']=='started'
    assert result['forward_action']==('reused' if already_forwarded else 'started')
    assert forward.call_count==(0 if already_forwarded else 1)
    remote.assert_called_once_with(camera.CAMERAS[0])


def test_unknown_local_listener_is_not_terminated(manager,monkeypatch):
    monkeypatch.setattr(manager,'probe',Mock(side_effect=RuntimeError('身份不符')))
    monkeypatch.setattr(manager,'ensure_remote',lambda c:dict(action='reused'))
    monkeypatch.setattr(manager,'port_open',lambda p:True)
    forward=Mock(side_effect=AssertionError('No duplicate local tunnel'))
    monkeypatch.setattr(manager,'start_forward',forward)
    with pytest.raises(RuntimeError,match='未终止已有服务'):
        manager.ensure(camera.CAMERAS[0])
    forward.assert_not_called()


def test_one_camera_failure_never_claims_both_ready(manager,monkeypatch):
    def ensure(c):
        if c['role']=='global':raise RuntimeError('板卡不可达')
        return healthy()
    monkeypatch.setattr(manager,'ensure',ensure)
    result=manager.start()
    assert not result['completed']
    assert result['cameras']['wrist']['ready']
    assert not result['cameras']['global']['ready']
    assert '全局相机：板卡不可达' in result['error']


def test_missing_key_explained_before_starting_remote_process(manager,monkeypatch):
    monkeypatch.setattr(camera,'KEY',manager.folder/'absent-key')
    monkeypatch.setattr(manager,'probe',Mock(side_effect=ConnectionRefusedError()))
    monkeypatch.setattr(manager,'ensure_remote',Mock(side_effect=AssertionError('No SSH without key')))
    with pytest.raises(RuntimeError,match='缺少已有 KR260 SSH 密钥'):
        manager.ensure(camera.CAMERAS[0])


@pytest.mark.parametrize('failure',[None,'wrong_serial','stopped','frozen','invalid_jpeg','stale'])
def test_probe_checks_identity_health_advancing_frames_and_pixels(manager,monkeypatch,failure):
    spec=camera.CAMERAS[0]
    status=dict(service='kr260-live-v1',role=spec['role'],device=dict(serial=spec['serial']),
                stopped=False,error=None,streams=dict(color=dict(capture_fps=30.)))
    if failure=='wrong_serial':status['device']['serial']='OTHER'
    if failure=='stopped':status['stopped']=True
    _,jpg=cv2.imencode('.jpg',np.zeros((480,640,3),np.uint8))
    class Opener:
        count=0
        def open(self,url,timeout):
            if url.endswith('/status'):return io.BytesIO(json.dumps(status).encode())
            self.count+=1
            response=io.BytesIO(b'bad jpeg' if failure=='invalid_jpeg' else jpg.tobytes())
            response.headers={'X-Frame-Index':str(10 if failure=='frozen' else 10+self.count),
                              'X-Board-Age-Ms':'300' if failure=='stale' else '0'}
            return response
    manager.opener=Opener()
    monkeypatch.setattr(camera.time,'sleep',lambda x:None)
    if failure is None:
        assert manager.probe(spec)['frame_indices']==[11,12]
    else:
        with pytest.raises(RuntimeError):manager.probe(spec)


def test_forward_uses_persistent_loopback_only_ssh_and_logs_pid(manager,monkeypatch):
    manager.folder.mkdir()
    popen=Mock(return_value=SimpleNamespace(pid=12345))
    monkeypatch.setattr(camera.subprocess,'Popen',popen)
    manager.start_forward(camera.CAMERAS[0])
    argv=popen.call_args.args[0]
    assert '-N' in argv and '127.0.0.1:18760:127.0.0.1:18760' in argv
    assert 'StrictHostKeyChecking=yes' in argv and 'BatchMode=yes' in argv
    assert popen.call_args.kwargs['start_new_session'] is True
    assert json.loads((manager.folder/'wrist_forward.json').read_text())['pid']==12345


def test_remote_helper_invokes_only_known_camera_script(manager,monkeypatch):
    import shlex
    manager.folder.mkdir()
    run=Mock(return_value=SimpleNamespace(returncode=0,stdout='{"action":"started"}\n',stderr=''))
    monkeypatch.setattr(camera.subprocess,'run',run)
    assert manager.ensure_remote(camera.CAMERAS[0])['action']=='started'
    argv=run.call_args.args[0]
    assert argv[-2]==camera.HOST
    remote=shlex.split(argv[-1])
    assert remote[:2]==['python3','-c']
    assert remote[2]==camera.REMOTE_ENSURE
    assert remote[3:]==['wrist','18760','CC1365301EY',camera.SERVER]


def test_backend_camera_action_uses_own_logs_and_invalidates_review(tmp_path,monkeypatch):
    monkeypatch.setattr(backend_module,'APP',tmp_path)
    backend=Backend();backend.run_dir=tmp_path/'old_grasp';backend.run_dir.mkdir()
    backend._review={'ready':True}
    def run(argv,log_path,timeout,monitor_run):
        assert backend._review is None and monitor_run is False
        with pytest.raises(RuntimeError,match='正在处理设备请求'):backend.start_cameras()
        folder=Path(argv[-1]);folder.mkdir(parents=True)
        (folder/'result.json').write_text(json.dumps(dict(completed=True,cameras={r:healthy() for r in ['wrist','global']})))
    monkeypatch.setattr(backend,'_run',run)
    result=backend.start_cameras()
    assert result['completed'] and result['robot_commands_sent']==0
    assert Path(result['log_dir']).parent==tmp_path/'logs/camera_services'
    assert not (backend.run_dir/'result.json').exists()
    assert backend._lockfile is None and backend._active_device_control is None


def test_demo_camera_button_never_starts_subprocess(tmp_path,monkeypatch):
    monkeypatch.setattr(backend_module,'APP',tmp_path)
    backend=Backend(mode='demo')
    monkeypatch.setattr(backend,'_run',Mock(side_effect=AssertionError('No live services in demo')))
    result=backend.start_cameras()
    assert result['completed'] and result['synthetic'] and result['robot_commands_sent']==0


def test_camera_button_dispatches_one_job_without_preparing_or_executing():
    app=GraspApp.__new__(GraspApp)
    app.busy='';app.closing=False;app.review={'ready':True}
    app.backend=Mock()
    for name in ['signature_label','stage_detail','camera_hint','state_label']:
        setattr(app,name,Mock())
    app._draw_path=Mock();app._append_log=Mock();app._start_job=Mock()
    app.start_cameras()
    assert app.review is None
    kind,operation=app._start_job.call_args.args
    assert kind=='camera_services'
    operation()
    app.backend.start_cameras.assert_called_once_with()
    app.backend.prepare.assert_not_called();app.backend.execute.assert_not_called()
    app.busy='execute';app._start_job.reset_mock();app.start_cameras()
    app._start_job.assert_not_called()


def test_camera_result_preserves_arm_snapshot_and_shows_partial_failure():
    app=GraspApp.__new__(GraspApp)
    app.log_dir=None;app.device_snapshot={'status':{'power_status':1}}
    for name in ['camera_hint','state_label','result_text','notebook']:setattr(app,name,Mock())
    app._set_text=Mock();app._refresh_controls=Mock()
    app._show_camera_services(dict(completed=False,cameras={'wrist':healthy(),
        'global':dict(ready=False,error='相机服务未响应')}))
    text=app._set_text.call_args.args[1]
    assert '腕部相机：已就绪' in text and '全局相机：未就绪' in text
    assert '相机服务未响应' in text
    assert app.device_snapshot=={'status':{'power_status':1}}
