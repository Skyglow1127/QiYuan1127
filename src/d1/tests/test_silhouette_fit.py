"""Recorded RGB/mask regressions and negative geometry; no device access."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP/'runtime/Control/tools'))
import cube_mask_pose as fitter
import prepare_accel_grasp as perception


def recorded(relative):
    folder = APP/'logs'/relative
    pose = json.loads((folder/'pose.json').read_text())
    candidate = json.loads(Path(pose['source_observation']).with_name('result.json').read_text())
    candidate = json.loads(Path(candidate['candidate']).read_text())
    args = [np.array(pose['T_base_camera']), np.array(pose['T_base_table']),
            np.array(candidate['camera_matrix']), np.array(candidate['distortion'])]
    return cv2.imread(str(folder/'mask.png'), 0), args, pose


@pytest.mark.parametrize('relative,mode', [
    ('20261026_091225_6f9d2d/perception', 'bounded_boundary'),
    ('20261026_005646_d7459d/execution/00_prepare_gripper/target_recheck', 'bounded_boundary'),
    ('20261026_005646_d7459d/perception', 'strict'),
])
def test_complete_recorded_cube_accepted_without_changing_geometry(relative, mode):
    mask, args, previous = recorded(relative)
    result = fitter.fit_pose(mask, *args)
    assert result['geometric_fit_accepted']
    assert result['fit_acceptance_mode'] == mode
    assert result['fit_rejection_reasons'] == []
    # Changing image-fit acceptance must not move the fit or calibration.
    np.testing.assert_allclose(result['T_base_cube'], previous['T_base_cube'], atol=1e-12, rtol=0)
    np.testing.assert_array_equal(result['T_base_camera'], args[0])
    np.testing.assert_array_equal(result['T_base_table'], args[1])
    assert result['cube_size_m'] == .04


@pytest.mark.parametrize('shape', ['circle', 'wide_rectangle', 'wrong_size', 'occluded', 'border', 'multiple', 'empty'])
def test_bad_shapes_still_rejected(shape):
    original, args, _ = recorded('20261026_091225_6f9d2d/perception')
    mask = np.zeros_like(original)
    if shape == 'circle':
        cv2.circle(mask, (253, 418), 44, 255, -1)
    elif shape == 'wide_rectangle':
        cv2.rectangle(mask, (214, 404), (294, 435), 255, -1)
    elif shape == 'wrong_size':
        reduced = cv2.resize(original, None, fx=.6, fy=.6)
        mask[160:160+reduced.shape[0], 100:100+reduced.shape[1]] = reduced
    elif shape == 'occluded':
        mask = original.copy(); mask[400:430, 210:240] = 0
    elif shape == 'border':
        mask = original.copy(); mask[-10:, 230:270] = 255
    elif shape == 'multiple':
        mask = original.copy(); cv2.rectangle(mask, (350, 300), (420, 390), 255, -1)
    try:
        result = fitter.fit_pose(mask, *args)
    except ValueError:
        return
    assert not result['geometric_fit_accepted']
    assert result['fit_acceptance_mode'] == 'rejected'
    assert result['fit_rejection_reasons']


def test_saved_user_failure_completes_read_only_preparation(tmp_path, monkeypatch):
    def no_device(*args, **kwargs):
        pytest.fail('Saved-image regression must never connect to a device')
    monkeypatch.setattr(perception, 'CliTransport', no_device)
    folder = APP/'logs/20261026_091225_6f9d2d/perception'
    old_result = json.loads((folder/'result.json').read_text())
    observation = folder/'wrist_observation.json'
    before = observation.read_bytes()
    args = SimpleNamespace(output=tmp_path/'replay', capture_live=False, observation=observation,
        backend='hsv', preprocess='host', endpoint=None, fpga_url=None,
        candidate=Path(old_result['candidate']), table_pose=Path(old_result['table_pose']))
    result = perception.prepare(args)
    assert result['completed'] and result['robot_commands_sent'] == 0
    assert not result['live_execution_eligible']
    assert result['source_kind'] == 'saved_image'
    assert observation.read_bytes() == before
    assert (args.output/'pose_overlay.jpg').is_file()


def test_latest_concave_mask_reports_measured_solidity_without_bypassing_filter():
    mask,args,_=recorded('20261026_134130_314dc2/perception')
    mask=cv2.imread(str(APP/'logs/20261026_134130_314dc2/execution/00_prepare_gripper/target_recheck/mask.png'),0)
    with pytest.raises(ValueError,match='当前 94.47%，要求 ≥95%'):
        fitter.fit_pose(mask,*args)
