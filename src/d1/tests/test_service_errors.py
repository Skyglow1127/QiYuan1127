"""Preparation network failures rendered from real saved reports and fake HTTP."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import urllib.error

import pytest

APP=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP))
sys.path.insert(0,str(APP/'runtime/Control/tools'))
from app.backend import DEFAULTS, ProcessFailure, preparation_diagnostics
from app.gui import GraspApp, preparation_result
from app.service_errors import ServiceRequestError, service_request
import prepare_accel_grasp as perception


def test_screenshot_run_reaches_result_widget_with_specific_camera_cause():
    folder=APP/'logs/20261026_132642_970a71'
    source=folder/'perception/result.json';original=source.read_bytes()
    failure=ProcessFailure(['prepare_accel_grasp.py','--output',folder/'perception'],1,folder/'perception.log')
    details=preparation_diagnostics(folder,DEFAULTS,failure)
    result=preparation_result(dict(ready=False,run_dir=str(folder),preparation_time_s=1.69,**details))
    result['preparation_timings']=[dict(name='wrist_http_status_and_image',duration_s=.001,outcome='failed')]
    app=GraspApp.__new__(GraspApp)
    app.mode='live';app.log_dir=None
    for name in ('state_label','elapsed_label','result_text','notebook'):
        setattr(app,name,Mock())
    app._set_text=Mock();app._append_log=Mock();app._draw_joint_plot=Mock()
    app._refresh_controls=Mock()
    app._show_result(result)
    text=app._set_text.call_args_list[0].args[1]
    assert '腕部相机服务无法访问' in text and 'http://127.0.0.1:18760' in text
    assert '连接被拒绝' in text and 'SSH 转发' in text
    assert text.count('腕部相机服务无法访问')==1
    assert 'urlopen' not in text and '[Errno' not in text
    assert '查看保存的诊断图' not in text and '（未通过）' in text
    assert '抓取结果：未执行' in text and '完整日志：' in text
    assert source.read_bytes()==original


@pytest.mark.parametrize('error,code,phrase',[
    (urllib.error.URLError(ConnectionRefusedError(111,'Connection refused')),'service_connection_refused','连接被拒绝'),
    (urllib.error.URLError(TimeoutError('timed out')),'service_timeout','等待响应超时'),
    (urllib.error.HTTPError('http://127.0.0.1:18760/status',503,'Unavailable',{},None),'service_http_error','HTTP 503'),
])
def test_capture_reports_endpoint_and_preserves_raw_network_error(tmp_path,monkeypatch,error,code,phrase):
    class Opener:
        def open(self,*a,**kw): raise error
    def observation(*args):
        with service_request(Opener(),'http://127.0.0.1:18760/status','腕部相机服务'):
            pytest.fail('Failed request must not produce an image')
    monkeypatch.setattr(perception,'live_observation',observation)
    args=SimpleNamespace(output=tmp_path/'capture',capture_live=True,backend='hsv',preprocess='host',
        endpoint=None,fpga_url=None,candidate=perception.DEFAULT_CANDIDATE,table_pose=perception.DEFAULT_TABLE)
    result=perception.prepare(args)
    assert not result['completed'] and result['robot_commands_sent']==0
    assert result['blockers'][0]['code']==code
    assert phrase in result['error'] and '/status' in result['error']
    assert result['technical_error']==str(error)
    assert '诊断图' not in result['blockers'][0]['action']
    assert not (args.output/'wrist.jpg').exists()


def test_failure_without_acquisition_evidence_does_not_blame_wrist_or_hsv_port(tmp_path):
    p=tmp_path/'perception';p.mkdir()
    (p/'result.json').write_text(json.dumps(dict(error='<urlopen error [Errno 111] Connection refused>',
        endpoint='http://127.0.0.1:18822')))
    (tmp_path/'perception.log').write_text('failed')
    failure=ProcessFailure(['prepare_accel_grasp.py','--output',p],1,tmp_path/'perception.log')
    assert '服务访问失败' in str(failure)
    assert '腕部相机' not in str(failure) and '18822' not in str(failure)
