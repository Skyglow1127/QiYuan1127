"""No robot I/O. SI units; ArUco + depth verified, top-down tabletop cubes only."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def transform(rotation=None, translation=None):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    if translation is not None:
        t[:3, 3] = translation
    return t


def validate_transform(t):
    t = np.asarray(t, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-5)):
        raise ValueError('Invalid rigid transform')
    return t


def to_pose(t):
    from geometry_msgs.msg import Pose
    t = validate_transform(t)
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, t[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
        float, Rotation.from_matrix(t[:3, :3]).as_quat())
    return p


def from_pose(p):
    q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError('Invalid quaternion')
    return validate_transform(transform(Rotation.from_quat(q).as_matrix(),
                                       [p.position.x, p.position.y, p.position.z]))


def intersect(pixels, k, distortion, normal, offset):
    rays = cv2.undistortPoints(np.array(pixels, np.float64).reshape(-1, 1, 2),
                              k, distortion).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    denom = rays @ normal
    if np.any(abs(denom) < .15):
        raise ValueError('Grazing view')
    points = rays * (offset / denom)[:, None]
    if np.any(points[:, 2] <= 0):
        raise ValueError('Plane behind camera')
    return points


def detect(color, depth, k, distortion, marker_size=.06, marker_id=23, cube_size=.035):
    """Returns camera->cube(center) and camera->marker; rejects ambiguity.

    Marker is flat on the same table, +Z points out of table. The visible cube
    top must be roughly front-facing. Depth is aligned to color, optical-Z metres.
    Neither an arbitrary red object nor marker-only PnP is enough for acceptance.
    """
    k, distortion = np.asarray(k, np.float64), np.asarray(distortion, np.float64)
    if (color.ndim != 3 or color.shape[2] != 3 or depth.shape != color.shape[:2]
            or k.shape != (3, 3) or not np.isfinite(k).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.isfinite(distortion).all() or not .01 <= marker_size <= .5
            or not .01 <= cube_size <= .1):
        raise ValueError('Invalid RGB-D calibration or physical sizes')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _ = cv2.aruco.detectMarkers(color, dictionary)
    matches = [] if ids is None else [c for c, i in zip(corners, ids.ravel()) if i == marker_id]
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one ArUco 4X4_50 ID {marker_id}')
    half = marker_size / 2
    object_points = np.array([[-half, half, 0], [half, half, 0],
                              [half, -half, 0], [-half, -half, 0]], np.float64)
    image_points = matches[0].reshape(4, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, k, distortion,
                                flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('Marker PnP failed')
    rotation = cv2.Rodrigues(rvec)[0]
    marker = transform(rotation, tvec.ravel())
    normal = rotation[:, 2]
    if normal[2] > -.75 or not .15 <= tvec[2, 0] <= 1.5:
        raise ValueError('Need top-down marker view, within 0.15..1.5 m')
    reproj = cv2.projectPoints(object_points, rvec, tvec, k, distortion)[0].reshape(4, 2)
    if np.max(np.linalg.norm(reproj - image_points, axis=1)) > 1.5:
        raise ValueError('Marker reprojection error exceeds 1.5 px')
    # Verify measured depth around marker against the PnP table plane.
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, image_points.astype(np.int32), 255)
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
    ys, xs = np.where(mask > 0)
    valid = np.isfinite(depth[ys, xs]) & (depth[ys, xs] > .15) & (depth[ys, xs] < 1.5)
    if valid.sum() < max(20, len(xs) * .3):
        raise ValueError('Insufficient marker depth')
    table_offset = float(normal @ marker[:3, 3])
    predicted = intersect(np.column_stack((xs[valid], ys[valid])), k, distortion,
                          normal, table_offset)
    if np.median(abs(depth[ys[valid], xs[valid]] - predicted[:, 2])) > .006:
        raise ValueError('Marker metric size, depth or plane inconsistent')
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 100, 60), (10, 255, 255)) | cv2.inRange(
        hsv, (170, 100, 60), (179, 255, 255))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        if cv2.contourArea(contour) < 60:
            continue
        mask[:] = 0
        cv2.drawContours(mask, [contour], -1, 255, -1)
        inner = cv2.erode(mask, np.ones((5, 5), np.uint8))
        ys, xs = np.where(inner > 0)
        if not len(xs):
            continue
        z = depth[ys, xs]
        good = np.isfinite(z) & (z > .15) & (z < 1.5)
        if good.mean() < .7:
            continue
        predicted = intersect(np.column_stack((xs[good], ys[good])), k, distortion,
                              normal, table_offset + cube_size)
        # A red patch on the table and unknown red object heights fail here.
        if np.quantile(abs(z[good] - predicted[:, 2]), .8) > .006:
            continue
        rectangle = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rectangle)
        points = intersect(box, k, distortion, normal, table_offset + cube_size)
        edges = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        if np.any(abs(edges - cube_size) > cube_size * .25):
            continue
        if cv2.contourArea(contour) / max(rectangle[1][0] * rectangle[1][1], 1) < .8:
            continue
        top_center = points.mean(axis=0)
        xaxis = points[1] - points[0]
        xaxis -= normal * (xaxis @ normal)
        xaxis /= np.linalg.norm(xaxis)
        # Cubic symmetry: choose an axis close to marker X for stable orientation.
        yaxis = np.cross(normal, xaxis)
        choices = [xaxis, yaxis, -xaxis, -yaxis]
        xaxis = max(choices, key=lambda a: a @ rotation[:, 0])
        axes = np.column_stack((xaxis, np.cross(normal, xaxis), normal))
        candidates.append(transform(axes, top_center - normal * cube_size / 2))
    if len(candidates) != 1:
        raise ValueError(f'Expected exactly one depth-verified 35 mm red cube; got {len(candidates)}')
    return candidates[0], marker
