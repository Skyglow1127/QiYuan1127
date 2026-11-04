"""Synthetic saved RGB fixtures exercise visual evidence without hardware I/O."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import vision


def save_json(path, value):
    path.write_text(json.dumps(value))
    return path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def scene(tmp_path):
    candidate = save_json(tmp_path / "empirical_handeye_candidate.json", {
        "mapping": {"order": [f"J{i}->Joint{i+1}" for i in range(6)], "signs": [1] * 6,
                    "zero_offsets_deg": [0] * 6},
        "T_end_camera": np.eye(4).tolist(), "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
        "distortion": [0, 0, 0, 0, 0],
    })
    # A simple camera branch and translating TCP make the hypotheses exactly
    # separable. This is synthetic geometry, not a physical D1 configuration.
    urdf = tmp_path / "d1.urdf"
    urdf.write_text('''<robot name="fixture"><link name="base_link"/><link name="Link6"/>
      <link name="grasp_tcp"/><joint name="camera" type="fixed"><parent link="base_link"/>
      <child link="Link6"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
      <joint name="Joint1" type="prismatic"><parent link="base_link"/><child link="grasp_tcp"/>
      <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/></joint></robot>''')
    cube = np.eye(4)
    cube[:3, 3] = [0, 0, 0.6]
    table = np.array([[0, 0, 1, -0.02], [0, 1, 0, 0], [-1, 0, 0, 0.6], [0, 0, 0, 1.]])
    plan = save_json(tmp_path / "lift.json", dict(completed=True, stage="lift", T_tcp_cube=cube.tolist(),
                    T_base_start_tcp=np.eye(4).tolist(), T_base_cube=cube.tolist(),
                    mapping_candidate=str(candidate), mapping_sha256=sha(candidate)))
    manifest = dict(created_at=time.time() - 1, spec=dict(candidate=str(candidate), urdf=str(urdf), lift_height_m=0.08),
                    target=dict(T_base_cube=cube.tolist(), T_base_table=table.tolist(), cube_size_m=0.04),
                    stages=[dict(name="lift", plan_file=str(plan))], files={str(candidate): sha(candidate), str(urdf): sha(urdf)})

    def observation(kind="held", age_s=0):
        image = np.zeros((480, 640, 3), np.uint8)
        draw_cube = cube.copy()
        if kind == "held":
            draw_cube[0, 3] += 0.08
        if kind in {"held", "stationary"}:
            hull = vision.projected_hull(draw_cube, np.eye(4), np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]]), np.zeros(5))
            cv2.fillConvexPoly(image, np.rint(hull).astype(np.int32), (0, 0, 255))
        if kind == "ambiguous":
            cv2.rectangle(image, (100, 100), (140, 140), (0, 0, 255), -1)
            cv2.rectangle(image, (300, 100), (340, 140), (0, 0, 255), -1)
        image_path = tmp_path / "wrist.png"
        assert cv2.imwrite(str(image_path), image)
        data = dict(image=str(image_path), image_sha256=sha(image_path), camera_serial="CC1365301EY",
                    captured_at=datetime.fromtimestamp(time.time() - age_s, timezone.utc).isoformat(),
                    headers={"X-Board-Age-Ms": "10", "X-Frame-Index": "25"}, feedback_age_s=0.02,
                    sdk_angles=[float(np.rad2deg(0.08)), 0, 0, 0, 0, 0, 20], stationary_span_deg=[0] * 7)
        return save_json(tmp_path / "wrist_observation.json", data)
    return manifest, plan, observation, tmp_path


def test_held_and_stationary_are_distinguished(scene):
    manifest, plan, observation, root = scene
    held = vision.verify_lift(manifest, observation("held"), root / "held", plan)
    assert held["status"] == "success", held
    assert held["evidence"]["held"]["iou"] > 0.99
    stationary = vision.verify_lift(manifest, observation("stationary"), root / "stationary", plan)
    assert stationary["status"] == "failure", stationary
    assert stationary["human_confirmation"] is False
    assert stationary["physical_motion_commands_sent"] == 0


@pytest.mark.parametrize("kind", ["missing", "ambiguous"])
def test_missing_or_multiple_targets_remain_unknown(scene, kind):
    manifest, plan, observation, root = scene
    result = vision.verify_lift(manifest, observation(kind), root / kind, plan)
    assert result["status"] == "unknown"


def test_stale_frame_and_hash_mismatch_remain_unknown(scene):
    manifest, plan, observation, root = scene
    stale = vision.verify_lift(manifest, observation("held", age_s=60), root / "stale", plan)
    assert stale["status"] == "unknown"
    path = observation("held")
    data = json.loads(path.read_text())
    data["image_sha256"] = "0" * 64
    save_json(path, data)
    altered = vision.verify_lift(manifest, path, root / "altered", plan)
    assert altered["status"] == "unknown"
    assert "hash mismatch" in altered["reason"]


def test_no_actual_lift_is_not_grasp_success(scene):
    manifest, plan, observation, root = scene
    path = observation("stationary")
    data = json.loads(path.read_text())
    data["sdk_angles"][0] = 0
    save_json(path, data)
    result = vision.verify_lift(manifest, path, root / "no_lift", plan)
    assert result["status"] == "unknown"
    assert "insufficient" in result["reason"]


def test_depth_only_lift_with_ambiguous_projection_remains_unknown(scene):
    manifest, plan, observation, root = scene
    # Movement nearly along the optical axis changes apparent size but gives
    # overlapping silhouettes, so a match alone must not become success.
    urdf = Path(manifest["spec"]["urdf"])
    urdf.write_text(urdf.read_text().replace('axis xyz="1 0 0"', 'axis xyz="0 0 1"'))
    manifest["files"][str(urdf)] = sha(urdf)
    table = np.eye(4)
    table[2, 3] = 0.58
    manifest["target"]["T_base_table"] = table.tolist()
    result = vision.verify_lift(manifest, observation("stationary"), root / "depth", plan)
    assert result["status"] == "unknown", result
    assert result["evidence"]["hypotheses_iou"] > vision.THRESHOLDS["maximum_hypotheses_iou"]


def test_binding_and_wrong_camera_are_rejected(scene):
    manifest, plan, observation, root = scene
    path = observation("held")
    data = json.loads(path.read_text())
    data["camera_serial"] = "CC1365302K0"
    save_json(path, data)
    assert vision.verify_lift(manifest, path, root / "camera", plan)["status"] == "unknown"
    path = observation("held")
    manifest["files"][manifest["spec"]["urdf"]] = "0" * 64
    result = vision.verify_lift(manifest, path, root / "binding", plan)
    assert result["status"] == "unknown"
    assert "dependency hash changed" in result["reason"]


def test_return_visibility_does_not_claim_release(scene):
    manifest, _, observation, root = scene
    result = vision.verify_return(manifest, observation("stationary"), root / "return")
    assert result["status"] == "success", result
    assert result["release_verified"] is False
    assert result["contact_force_verified"] is False
    missing = vision.verify_return(manifest, observation("missing"), root / "return_unknown")
    assert missing["status"] == "unknown"


def frozen_observations(scene, *, start_age_s=120, offset_deg=1.0):
    """Bind synthetic pre/post frames to a slow frozen lift without hardware."""
    manifest, plan, observation, root = scene
    end_path = observation("held")
    end = json.loads(end_path.read_text())
    end_image = root / "post_lift.png"
    end_image.write_bytes(Path(end["image"]).read_bytes())
    end["image"] = str(end_image)
    end["sdk_angles"][0] += offset_deg
    final_path = save_json(root / "post_lift_observation.json", end)
    start_path = observation("stationary")
    start = json.loads(start_path.read_text())
    start_image = root / "pre_lift.png"
    start_image.write_bytes(Path(start["image"]).read_bytes())
    captured = datetime.fromisoformat(end["captured_at"]).timestamp() - start_age_s
    start.update(image=str(start_image), image_sha256=sha(start_image),
                 captured_at=datetime.fromtimestamp(captured, timezone.utc).isoformat(),
                 sdk_angles=[offset_deg, 0, 0, 0, 0, 0, 20], hardware_commands_sent=0)
    start["headers"]["X-Frame-Index"] = "24"
    initial_path = save_json(root / "pre_lift_observation.json", start)
    manifest["created_at"] = captured - 1
    manifest["stages"][0].update(expected_angles=[0, 0, 0, 0, 0, 0, 20],
                                sdk_plan={"sdk": [[0] * 6, [float(np.rad2deg(.08)), 0, 0, 0, 0, 0]]})
    manifest["files"][str(plan)] = sha(plan)
    return manifest, plan, initial_path, final_path, captured + .5, root


def test_frozen_lift_uses_actual_start_attachment_without_editing_control_plan(scene):
    manifest, plan, start, end, began, root = frozen_observations(scene)
    before = plan.read_bytes()
    result = vision.verify_lift(manifest, end, root / "frozen", plan,
                               lift_start_observation_path=start, lift_execution_started_at=began)
    assert result["status"] == "success", result
    assert result["evidence"]["held"]["iou"] > .99
    assert result["evidence"]["attachment_basis"] == "measured_pre_lift_observation_for_visual_hypothesis_only"
    assert result["evidence"]["lift_start_observation"]["observation_sha256"] == sha(start)
    assert result["evidence"]["lift_start_delta_from_approved_deg"][0] == pytest.approx(1.0)
    assert result["evidence"]["T_base_held_cube"][0][3] == pytest.approx(.08)
    assert plan.read_bytes() == before


@pytest.mark.parametrize("case", ["stale_at_start", "future_start", "wrong_frame", "wrong_camera",
                                  "arm_start_mismatch", "j6_start_mismatch", "stale_feedback",
                                  "unbound_plan", "nominal_fk_mismatch", "image_changed"])
def test_frozen_lift_rejects_misbound_or_stale_start_evidence(scene, case):
    manifest, plan, start, end, began, root = frozen_observations(scene)
    data = json.loads(start.read_text())
    if case == "stale_at_start": began += 31
    if case == "future_start": began -= 1
    if case == "wrong_frame": data["headers"]["X-Frame-Index"] = "25"
    if case == "wrong_camera": data["camera_serial"] = "another_camera"
    if case == "arm_start_mismatch": data["sdk_angles"][0] = 1.001
    if case == "j6_start_mismatch": data["sdk_angles"][6] = 20.501
    if case == "stale_feedback": data["feedback_age_s"] = .601
    if case == "unbound_plan": del manifest["files"][str(plan)]
    if case == "nominal_fk_mismatch": manifest["stages"][0]["sdk_plan"]["sdk"][0][0] = .1
    if case == "image_changed": data["image_sha256"] = "0" * 64
    save_json(start, data)
    result = vision.verify_lift(manifest, end, root / case, plan,
                               lift_start_observation_path=start, lift_execution_started_at=began)
    assert result["status"] == "unknown", (case, result)


def test_frozen_lift_requires_execution_time_binding(scene):
    manifest, plan, start, end, began, root = frozen_observations(scene)
    missing = vision.verify_lift(manifest, end, root / "missing_time", plan,
                                lift_start_observation_path=start)
    assert missing["status"] == "unknown"
    wrong_order = vision.verify_lift(manifest, end, root / "bad_time", plan,
                                     lift_start_observation_path=start,
                                     lift_execution_started_at=time.time() + 60)
    assert wrong_order["status"] == "unknown"


def test_visual_feedback_uses_shared_point_two_degree_tolerance(scene):
    manifest, plan, observation, root = scene
    path = observation("held")
    data = json.loads(path.read_text())
    data["sdk_angles"][1] = -90.2
    save_json(path, data)
    result = vision.verify_lift(manifest, path, root / "tolerance", plan)
    assert result["status"] == "success", result
    assert result["evidence"]["actual_sdk_angles"][1] == -90.2
    data["sdk_angles"][1] = -90.201
    save_json(path, data)
    result = vision.verify_lift(manifest, path, root / "over_tolerance", plan)
    assert result["status"] == "unknown"
    assert "反馈容差" in result["reason"]
