"""Offline regression for the user's clipped image; never opens a device."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
sys.path.insert(0, str(APP/'runtime/Control/tools'))
from app.hsv import infer_remote
import prepare_accel_grasp as perception


def test_real_clipped_image_keeps_evidence_and_never_becomes_a_pose(tmp_path, monkeypatch):
    original = APP/'logs/20261025_224213_276e77/perception/wrist_observation.json'
    raw = original.read_bytes()
    def unexpected(*args, **kwargs):
        pytest.fail('Rejected contour must not fit a pose or connect to hardware')
    monkeypatch.setattr(perception, 'fit_pose', unexpected)
    monkeypatch.setattr(perception, 'CliTransport', unexpected)
    args = SimpleNamespace(output=tmp_path/'replay', capture_live=False, observation=original,
        backend='hsv', preprocess='host', endpoint=None, fpga_url=None,
        candidate=perception.DEFAULT_CANDIDATE, table_pose=perception.DEFAULT_TABLE)
    result = perception.prepare(args)
    foreground = json.loads((args.output/'foreground.json').read_text())
    assert not result['completed'] and result['robot_commands_sent'] == 0
    assert result['source_kind'] == 'saved_image'
    assert foreground['rejection_code'] == 'target_clipped'
    assert foreground['clipped_edges'] == ['bottom']
    assert '下沿' in result['error']
    assert (args.output/'mask.png').is_file()
    assert cv2.imread(result['diagnostic_image']).shape == (480, 640, 3)
    assert result['sdk_angles'][1] < -90 and result['robot_ready'] is False
    assert not (args.output/'pose.json').exists()
    assert original.read_bytes() == raw


@pytest.mark.parametrize('rectangles,expected', [
    ([(230, 200, 310, 280)], None),
    ([(230, 400, 310, 479)], 'target_clipped'),
    ([(30, 200, 110, 280), (300, 200, 380, 280)], 'multiple_targets'),
    ([], 'target_not_found'),
])
def test_contour_guard_is_preserved(rectangles, expected):
    image = np.zeros((480, 640, 3), np.uint8)
    for x0, y0, x1, y1 in rectangles:
        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 255), -1)
    _, encoded = cv2.imencode('.png', image)
    result = infer_remote(encoded.tobytes())
    assert result['rejection_code'] == expected
    assert (result['status'] == 'FOREGROUND_CANDIDATE') == (expected is None)


def test_all_robot_blockers_reported_together_without_changing_snapshot():
    snapshot = dict(q=np.array([2., -90.201, 87.9, 10.1, 13.3, -7.3, 43.9]),
        q_time=9.99, status={'power_status': 0, 'enable_status': 0, 'error_status': 0},
        status_time={key: 9.99 for key in ['power_status', 'enable_status', 'error_status']}, fault=None)
    before = snapshot['q'].copy()
    errors = perception.robot_readiness_errors(snapshot, 10.)
    assert len(errors) == 2
    assert 'J1' in errors[0] and '-90.201' in errors[0]
    assert 'power_status=0' in errors[1]
    assert not any('enable_status=0' in item for item in errors)
    assert np.array_equal(snapshot['q'], before)
    snapshot['q'][1] = -90.
    snapshot['status'].update(power_status=1, enable_status=1)
    assert perception.robot_readiness_errors(snapshot, 10.) == []
    assert len(perception.robot_readiness_errors(snapshot, 11.)) == 4


def test_unstarted_position_mode_and_arbitrary_legal_jaw_are_prepare_candidates():
    snapshot = dict(q=np.array([0., -80., 70., 1., 17., -7., 6.5]), q_time=5.,
        status={'power_status':1,'enable_status':0,'error_status':0},
        status_time={key:5. for key in ['power_status','enable_status','error_status']}, fault=None)
    assert perception.robot_readiness_errors(snapshot, 5.1) == []
    snapshot['status']['enable_status'] = 2
    assert any('enable_status=2' in e for e in perception.robot_readiness_errors(snapshot,5.1))
