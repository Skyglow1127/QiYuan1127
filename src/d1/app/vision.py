"""Deterministic RGB evidence checks; this module never opens a hardware device.

Success means that the observed red silhouette supports the stated geometric
hypothesis. It does not measure jaw force, contact or collision clearance.
Thresholds below are engineering filters, not calibrated success probabilities.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np

CONTROL = Path(__file__).resolve().parents[1] / "runtime" / "Control"
for _directory in (CONTROL / "tools", CONTROL / "ros2_ws/src/d1_pick_place"):
    if str(_directory) not in sys.path:
        sys.path.insert(0, str(_directory))

from cube_mask_pose import projected_hull, distances_to_polygon, sample_boundary
from d1_pick_place.kinematics import forward
from d1_pick_place.sdk_executor import LIMITS, Mapping, NAMES, feedback_limit_errors, ARM_POSITION_TOLERANCE_DEG

THRESHOLDS = {
    "maximum_observation_age_s": 30.0,
    "maximum_camera_age_ms": 250.0,
    "maximum_feedback_age_s": 0.6,
    "maximum_stationary_span_deg": 0.3,
    "minimum_foreground_area_px": 100,
    "minimum_solidity": 0.90,
    "minimum_match_iou": 0.65,
    "maximum_other_iou": 0.35,
    "minimum_iou_margin": 0.30,
    "maximum_hypotheses_iou": 0.45,
    "maximum_boundary_rms_px": 6.5,
}


def _read(path):
    return json.loads(Path(path).read_text())


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    return float(value)


def _transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-5)):
        raise ValueError(f"{name}: rigid transform required")
    return matrix


def extract_red_mask(bgr):
    """Return the unique substantial red component and its measured evidence.

    Missing, clipped, ambiguous or strongly occluded components raise ValueError.
    Callers must translate these into unknown, never an inferred failed grasp.
    """
    if bgr is None or bgr.shape != (480, 640, 3) or bgr.dtype != np.uint8:
        raise ValueError("expected original 640x480 uint8 BGR image")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    binary = (cv2.inRange(hsv, np.array([0, 80, 50]), np.array([10, 255, 255]))
              | cv2.inRange(hsv, np.array([170, 80, 50]), np.array([179, 255, 255])))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count < 2:
        raise ValueError("no visible red component")
    areas = stats[1:, cv2.CC_STAT_AREA]
    index = 1 + int(np.argmax(areas))
    area = int(areas.max())
    if area < THRESHOLDS["minimum_foreground_area_px"]:
        raise ValueError("visible red component too small")
    if int(np.sum(areas > 0.2 * area)) != 1:
        raise ValueError("multiple comparable red components")
    x, y, width, height = [int(v) for v in stats[index, :4]]
    if x < 2 or y < 2 or x + width > 638 or y + height > 478:
        raise ValueError("red component touches image border")
    mask = (labels == index).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = cv2.contourArea(contour) / max(hull_area, 1.0)
    if solidity < THRESHOLDS["minimum_solidity"]:
        raise ValueError("red silhouette is occluded or nonconvex")
    return mask, dict(area_px=area, bounding_box_xywh=[x, y, width, height],
                      solidity=float(solidity), comparable_components=1,
                      method="OpenCV HSV H=[0,10] or [170,179], S>=80, V>=50")


def _resolve_bound_path(manifest, key, alternatives=()):
    spec = manifest.get("spec", {})
    value = spec.get(key)
    if value:
        return Path(value).resolve()
    for candidate in alternatives:
        if candidate:
            return Path(candidate).resolve()
    files = manifest.get("files", {})
    for name in files:
        path = Path(name)
        if (key == "candidate" and path.name == "empirical_handeye_candidate.json"
                or key == "urdf" and path.name == "d1.urdf"):
            return path.resolve()
    raise ValueError(f"manifest does not identify {key}")


def _binding(manifest, path, *, alternate_hash=None):
    hashes = {str(Path(key).resolve()): value for key, value in manifest.get("files", {}).items()}
    expected = hashes.get(str(path.resolve())) or alternate_hash
    actual = _sha(path)
    if not isinstance(expected, str) or len(expected) != 64:
        raise ValueError(f"unbound dependency: {path.name}")
    if actual != expected:
        raise ValueError(f"dependency hash changed: {path.name}")
    return actual


def _observation(manifest, observation_path, *, reference_time=None, require_foreground=True):
    observation_path = Path(observation_path).resolve()
    observation = _read(observation_path)
    if observation.get("camera_serial") != "CC1365301EY":
        raise ValueError("verification requires the calibrated wrist camera")
    if observation.get("source_kind") == "saved_image":
        raise ValueError("saved-image replay is not current physical evidence")
    captured = datetime.fromisoformat(observation["captured_at"])
    if captured.tzinfo is None:
        raise ValueError("capture timestamp requires an explicit timezone")
    captured_s = captured.timestamp()
    now = time.time() if reference_time is None else _finite(reference_time, "observation reference time")
    if not -2.0 <= now - captured_s <= THRESHOLDS["maximum_observation_age_s"]:
        raise ValueError("observation timestamp is stale or future-dated")
    created_at = manifest.get("created_at")
    if created_at is not None and captured_s < _finite(created_at, "manifest.created_at") - 0.1:
        raise ValueError("observation predates this grasp plan")
    headers = {key.lower(): value for key, value in observation.get("headers", {}).items()}
    camera_age = float(headers["x-board-age-ms"])
    if not math.isfinite(camera_age) or not 0 <= camera_age <= THRESHOLDS["maximum_camera_age_ms"]:
        raise ValueError("stale or invalid camera frame age")
    if "image_fetch_elapsed_s" in observation:
        elapsed = _finite(observation["image_fetch_elapsed_s"], "image fetch time")
        if elapsed < 0 or camera_age + elapsed * 1000 > THRESHOLDS["maximum_camera_age_ms"]:
            raise ValueError("camera frame exceeded age limit during transfer")
    frame_index = int(headers["x-frame-index"])
    if frame_index < 0:
        raise ValueError("invalid camera frame index")
    feedback_age = _finite(observation["feedback_age_s"], "feedback age")
    if not 0 <= feedback_age <= THRESHOLDS["maximum_feedback_age_s"]:
        raise ValueError("stale robot feedback")
    q = np.asarray(observation["sdk_angles"], dtype=float)
    span = np.asarray(observation["stationary_span_deg"], dtype=float)
    errors = feedback_limit_errors(q)
    if errors:
        raise ValueError("invalid or out-of-limit actual joint feedback: " + "; ".join(errors))
    if (span.shape != (7,) or not np.isfinite(span).all() or np.any(span < 0)
            or span.max() > THRESHOLDS["maximum_stationary_span_deg"] + 1e-5):
        raise ValueError("stationary observation not established")
    image_path = Path(observation["image"])
    if not image_path.is_absolute():
        image_path = observation_path.parent / image_path
    actual_hash = _sha(image_path)
    if actual_hash != observation.get("image_sha256"):
        raise ValueError("observed image hash mismatch")
    image = cv2.imread(str(image_path))
    if image is None or image.shape != (480, 640, 3):
        raise ValueError("invalid observation image")
    mask, foreground = extract_red_mask(image) if require_foreground else (None, None)
    return dict(q=q, image=image, mask=mask, foreground=foreground,
                evidence=dict(observation=str(observation_path), observation_sha256=_sha(observation_path),
                              image=str(image_path.resolve()), image_sha256=actual_hash,
                              captured_at=observation["captured_at"], captured_at_unix=captured_s,
                              freshness_reference_unix=now, frame_index=frame_index,
                              camera_serial=observation["camera_serial"], camera_age_ms=camera_age,
                              feedback_age_s=feedback_age, actual_sdk_angles=q.tolist(),
                              foreground=foreground))


def _scene(manifest, observation_path, plan=None, *, reference_time=None, require_foreground=True):
    context = _observation(manifest, observation_path, reference_time=reference_time,
                           require_foreground=require_foreground)
    refresh = manifest.get("recognition_refresh") or {}
    plan = plan or {}
    candidate_path = _resolve_bound_path(manifest, "candidate", (refresh.get("candidate"), plan.get("mapping_candidate")))
    urdf_path = _resolve_bound_path(manifest, "urdf")
    candidate_hash = _binding(manifest, candidate_path, alternate_hash=plan.get("mapping_sha256"))
    urdf_hash = _binding(manifest, urdf_path)
    candidate = _read(candidate_path)
    mapping = Mapping.from_candidate(candidate_path)
    values = dict(zip(NAMES, mapping.to_model(context["q"])))
    camera = forward(urdf_path, values, "Link6") @ _transform(candidate["T_end_camera"], "T_end_camera")
    tcp = forward(urdf_path, values, "grasp_tcp")
    cube = _transform(manifest["target"]["T_base_cube"], "T_base_cube")
    table = _transform(manifest["target"]["T_base_table"], "T_base_table")
    size = _finite(manifest["target"].get("cube_size_m", 0.04), "cube size")
    if not 0.0399 <= size <= 0.0401:
        raise ValueError("only the existing 40 mm cube is supported")
    k = np.asarray(candidate["camera_matrix"], dtype=float)
    d = np.asarray(candidate["distortion"], dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.isfinite(d).all():
        raise ValueError("invalid calibrated camera intrinsics")
    context.update(camera=camera, tcp=tcp, cube=cube, table=table, k=k, d=d, size=size)
    context["evidence"].update(candidate=str(candidate_path), candidate_sha256=candidate_hash,
                               urdf=str(urdf_path), urdf_sha256=urdf_hash,
                               T_base_camera=camera.tolist(), T_base_tcp=tcp.tolist())
    return context


def _projection(context, cube):
    hull = projected_hull(cube, context["camera"], context["k"], context["d"], context["size"])
    if (not np.isfinite(hull).all() or np.min(hull[:, 0]) < 2 or np.max(hull[:, 0]) > 637
            or np.min(hull[:, 1]) < 2 or np.max(hull[:, 1]) > 477):
        raise ValueError("hypothesis silhouette is not fully visible")
    predicted = np.zeros((480, 640), dtype=np.uint8)
    cv2.fillConvexPoly(predicted, np.rint(hull).astype(np.int32), 1)
    if predicted.sum() < THRESHOLDS["minimum_foreground_area_px"]:
        raise ValueError("hypothesis silhouette too small")
    return hull, predicted


def _iou(a, b):
    return float(np.count_nonzero((a != 0) & (b != 0)) / max(1, np.count_nonzero((a != 0) | (b != 0))))


def _score(mask, hull, predicted):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
    boundary = sample_boundary(contour, 96)
    residual = np.r_[distances_to_polygon(boundary, hull),
                     distances_to_polygon(sample_boundary(hull, 96), cv2.convexHull(contour.astype(np.float32)).reshape(-1, 2))]
    return dict(iou=_iou(mask, predicted), boundary_rms_px=float(np.sqrt(np.mean(residual ** 2))),
                predicted_hull_px=hull.tolist(), predicted_area_px=int(predicted.sum()))


def _matches(score):
    return (score["iou"] >= THRESHOLDS["minimum_match_iou"]
            and score["boundary_rms_px"] <= THRESHOLDS["maximum_boundary_rms_px"])


def _save_visuals(context, projections, output_dir):
    overlay = context["image"].copy()
    colors = [(0, 255, 0), (0, 255, 255)]
    for index, (name, hull) in enumerate(projections.items()):
        cv2.polylines(overlay, [np.rint(hull).astype(np.int32)], True, colors[index % 2], 2)
        cv2.putText(overlay, name, (12, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[index % 2], 2)
    for name, data in (("observed_mask.png", context["mask"] * 255), ("hypotheses.jpg", overlay)):
        if not cv2.imwrite(str(output_dir / name), data):
            raise OSError(f"cannot save {name}")
    return dict(mask=str(output_dir / "observed_mask.png"), overlay=str(output_dir / "hypotheses.jpg"))


def _finish(result, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    result.update(schema="d1-automatic-visual-evidence-v1", thresholds=dict(THRESHOLDS),
                  threshold_basis="engineering filters; no calibrated probability claim",
                  human_confirmation=False, contact_force_verified=False,
                  physical_motion_commands_sent=0)
    (output_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return result


def verify_lift(manifest: dict, observation_path: Path, output_dir: Path, lift_plan_path=None,
                *, lift_start_observation_path=None, lift_execution_started_at=None) -> dict:
    """Compare actual wrist RGB against held and stationary cube hypotheses.

    Missing evidence, insufficient lift or nondiscriminating projections produce
    unknown. Failure requires a visible cube matching the stationary hypothesis.
    The approved lift attachment remains unchanged. Frozen-path callers provide
    the pre-motion observation and execution start time so only the visual held
    hypothesis uses the measured start FK. Historical/legacy callers can still
    verify the attachment already stored in their actual-start lift plan.
    """
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = dict(status="unknown", reason="insufficient visual evidence", evidence={}, kind="lift",
                  claim_scope="red cube silhouette moved with the measured gripper pose")
    try:
        if lift_plan_path is None:
            lift_plan_path = next(stage["plan_file"] for stage in manifest["stages"] if stage["name"] == "lift")
        lift_plan_path = Path(lift_plan_path).resolve()
        plan = _read(lift_plan_path)
        if plan.get("completed") is not True or plan.get("stage") != "lift":
            raise ValueError("a completed geometric lift plan is required")
        context = _scene(manifest, observation_path, plan)
        result["evidence"] = context["evidence"]
        attachment = _transform(plan["T_tcp_cube"], "T_tcp_cube")
        start_tcp = _transform(plan["T_base_start_tcp"], "T_base_start_tcp")
        plan_cube = _transform(plan["T_base_cube"], "lift plan cube")
        if not np.allclose(plan_cube, context["cube"], atol=1e-6, rtol=0):
            raise ValueError("lift plan target differs from the reviewed target")
        if not np.allclose(start_tcp @ attachment, plan_cube, atol=1e-6, rtol=0):
            raise ValueError("attachment does not bind the lift start and observed cube")
        result["evidence"].update(attachment_basis="stored_lift_plan",
                                  T_tcp_cube_approved=attachment.tolist())
        if lift_start_observation_path is not None:
            started_at = _finite(lift_execution_started_at, "lift execution start time")
            approved_stages = [stage for stage in manifest["stages"] if stage.get("name") == "lift"]
            if len(approved_stages) != 1:
                raise ValueError("exactly one approved lift stage is required")
            approved_stage = approved_stages[0]
            if Path(approved_stage["plan_file"]).resolve() != lift_plan_path:
                raise ValueError("lift start observation must bind the approved frozen lift plan")
            _binding(manifest, lift_plan_path)
            nominal = np.asarray(approved_stage["sdk_plan"]["sdk"][0], dtype=float)
            expected = np.asarray(approved_stage["expected_angles"], dtype=float)
            if (nominal.shape != (6,) or expected.shape != (7,) or
                    not np.isfinite(nominal).all() or not np.isfinite(expected).all()):
                raise ValueError("invalid approved lift start")
            if not manifest["created_at"] <= started_at <= context["evidence"]["captured_at_unix"]:
                raise ValueError("lift execution time does not precede the ending observation in this plan")
            # A slow lift can last longer than the live-image age budget. Validate
            # the saved start observation against when execution began, not now.
            start_context = _scene(manifest, lift_start_observation_path, plan,
                                   reference_time=started_at, require_foreground=False)
            start_evidence = start_context["evidence"]
            if not 0 <= started_at - start_evidence["captured_at_unix"] <= THRESHOLDS["maximum_observation_age_s"]:
                raise ValueError("lift start observation is stale or after execution began")
            if (start_evidence["frame_index"] >= context["evidence"]["frame_index"] or
                    start_evidence["captured_at_unix"] >= context["evidence"]["captured_at_unix"]):
                raise ValueError("lift start and end observations are not ordered independent camera frames")
            nominal_mapping = Mapping.from_candidate(start_evidence["candidate"])
            nominal_tcp = forward(Path(start_evidence["urdf"]),
                                  dict(zip(NAMES, nominal_mapping.to_model(nominal))), "grasp_tcp")
            if not np.allclose(nominal_tcp, start_tcp, atol=1e-6, rtol=0):
                raise ValueError("approved nominal joint start does not match the lift plan TCP")
            delta = start_context["q"][:6] - nominal
            if np.max(np.abs(delta)) > ARM_POSITION_TOLERANCE_DEG + 1e-5 or abs(start_context["q"][6] - expected[6]) > .5 + 1e-5:
                raise ValueError("measured lift start differs from approved nominal start (>1 deg / J6 0.5 SDK)")
            # Do not mutate the trajectory plan, its attachment, or lower targets.
            attachment = np.linalg.inv(start_context["tcp"]) @ plan_cube
            result["evidence"].update(attachment_basis="measured_pre_lift_observation_for_visual_hypothesis_only",
                lift_execution_started_at=started_at, lift_start_observation=start_evidence,
                lift_start_delta_from_approved_deg=delta.tolist(),
                lift_start_j6_delta_sdk=float(start_context["q"][6] - expected[6]),
                T_base_actual_lift_start_tcp=start_context["tcp"].tolist(),
                T_tcp_cube_visual_hypothesis=attachment.tolist(),
                approved_control_plan_unchanged=True)
        elif lift_execution_started_at is not None:
            raise ValueError("lift execution start time requires its pre-motion observation")
        held_cube = context["tcp"] @ attachment
        lift_m = float((held_cube[:3, 3] - context["cube"][:3, 3]) @ context["table"][:3, 2])
        requested = _finite(manifest["spec"]["lift_height_m"], "lift height")
        minimum_lift = max(0.008, min(0.02, requested * 0.5))
        result["evidence"].update(lift_plan=str(lift_plan_path), lift_plan_sha256=_sha(lift_plan_path),
                                   actual_hypothesized_lift_m=lift_m, minimum_required_lift_m=minimum_lift,
                                   T_base_held_cube=held_cube.tolist())
        if lift_m < minimum_lift:
            raise ValueError("actual lift displacement is insufficient to verify grasp")
        held_hull, held_mask = _projection(context, held_cube)
        stationary_hull, stationary_mask = _projection(context, context["cube"])
        held = _score(context["mask"], held_hull, held_mask)
        stationary = _score(context["mask"], stationary_hull, stationary_mask)
        overlap = _iou(held_mask, stationary_mask)
        result["evidence"].update(held=held, stationary=stationary, hypotheses_iou=overlap,
                                   artifacts=_save_visuals(context, {"held": held_hull, "stationary": stationary_hull}, output_dir))
        if overlap > THRESHOLDS["maximum_hypotheses_iou"]:
            result["reason"] = "held and stationary hypotheses cannot be distinguished in this view"
        elif (_matches(held) and stationary["iou"] <= THRESHOLDS["maximum_other_iou"]
                and held["iou"] - stationary["iou"] >= THRESHOLDS["minimum_iou_margin"]):
            result.update(status="success", reason="observed silhouette supports held cube and rejects stationary cube")
        elif (_matches(stationary) and held["iou"] <= THRESHOLDS["maximum_other_iou"]
                and stationary["iou"] - held["iou"] >= THRESHOLDS["minimum_iou_margin"]):
            result.update(status="failure", reason="observed silhouette supports cube remaining at its original table position")
        else:
            result["reason"] = "observed silhouette does not clearly support either geometric hypothesis"
    except (OSError, ValueError, KeyError, TypeError, StopIteration, cv2.error) as error:
        result.update(status="unknown", reason=str(error) or type(error).__name__)
    return _finish(result, output_dir)


def verify_return(manifest: dict, observation_path: Path, output_dir: Path) -> dict:
    """Report visible return to the reviewed target; never claim verified release."""
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = dict(status="unknown", reason="insufficient visual evidence", evidence={}, kind="return",
                  claim_scope="red cube visible at the reviewed table position",
                  release_verified=False, grip_release_force_verified=False)
    try:
        context = _scene(manifest, observation_path)
        result["evidence"] = context["evidence"]
        hull, mask = _projection(context, context["cube"])
        target = _score(context["mask"], hull, mask)
        result["evidence"].update(target=target, artifacts=_save_visuals(context, {"table target": hull}, output_dir))
        if _matches(target):
            result.update(status="success", reason="red cube silhouette is visible at the reviewed table position; jaw release is not measured")
        else:
            result["reason"] = "no sufficient visual evidence of return to the reviewed table position"
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
        result.update(status="unknown", reason=str(error) or type(error).__name__)
    return _finish(result, output_dir)
