"""Fit the silhouette of one known cube on an existing, fixed table plane.

This reuses the empirical camera mapping and table; it does not recalibrate them.
Cube yaw is defined modulo 90 degrees. No jaw/contact or clearance is inferred.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares

# Engineering image-fit filters, not calibrated grasp accuracy. The bounded
# route tolerates modest HSV/ideal-cube boundary disagreement while requiring
# good overlap AND low typical, tail and worst-case boundary errors.
FIT_LIMITS = dict(minimum_iou=.88, maximum_rms_px=3.25,
                  maximum_p95_px=6.5, maximum_boundary_px=8.)


def cube_transform(table, xy_yaw, size=.04):
    x, y, yaw = xy_yaw; c, s = np.cos(yaw), np.sin(yaw)
    local = np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, size/2], [0, 0, 0, 1.]])
    return np.asarray(table) @ local


def projected_hull(cube, camera, k, d, size=.04):
    obj = np.array([[x, y, z] for x in [-.5, .5] for y in [-.5, .5] for z in [-.5, .5]]) * size
    t = np.linalg.inv(camera) @ cube
    transformed = obj @ t[:3, :3].T + t[:3, 3]
    if np.min(transformed[:, 2]) <= .02:
        raise ValueError('Cube is behind or too close to the camera')
    pixels = cv2.projectPoints(obj, cv2.Rodrigues(t[:3, :3])[0], t[:3, 3], k, d)[0].reshape(-1, 2)
    return cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2)


def sample_boundary(polygon, count=64):
    p = np.asarray(polygon); edge = np.roll(p, -1, axis=0) - p
    length = np.linalg.norm(edge, axis=1); cumulative = np.r_[0, np.cumsum(length)]
    distances = np.arange(count) * cumulative[-1] / count
    indices = np.minimum(np.searchsorted(cumulative, distances, side='right')-1, len(p)-1)
    return p[indices] + edge[indices] * ((distances-cumulative[indices])/np.maximum(length[indices], 1e-9))[:, None]


def distances_to_polygon(points, polygon):
    p = np.asarray(polygon); e = np.roll(p, -1, axis=0)-p
    delta = points[:, None, :] - p[None, :, :]
    fraction = np.clip(np.sum(delta*e, axis=2)/np.maximum(np.sum(e*e, axis=1), 1e-9), 0, 1)
    return np.sqrt(np.min(np.sum((delta-fraction[:, :, None]*e)**2, axis=2), axis=1))


def fit_pose(mask, camera, table, k, d, size=.04):
    mask = np.asarray(mask) > 0
    if mask.shape != (480, 640): raise ValueError('Expected original 640x480 mask')
    if not .0399 <= size <= .0401: raise ValueError('Only the existing 40 mm cube is supported')
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: raise ValueError('No foreground')
    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    if x < 2 or y < 2 or x+w > 638 or y+h > 478: raise ValueError('Cube touches image border')
    if cv2.contourArea(contour) < 100: raise ValueError('Cube too small for silhouette fitting')
    if sum(cv2.contourArea(c) > .2*cv2.contourArea(contour) for c in contours) > 1:
        raise ValueError('Multiple comparable foreground components')
    observed = sample_boundary(contour.reshape(-1, 2).astype(float), 96)
    hull = cv2.convexHull(contour).reshape(-1, 2).astype(float)
    solidity=cv2.contourArea(contour)/cv2.contourArea(hull.astype(np.float32))
    if solidity < .95:
        raise ValueError(f'红块轮廓完整度不足：当前 {solidity:.2%}，要求 ≥95%；分割轮廓有缺口或凹陷，请检查遮挡与颜色分割')
    camera, table = np.asarray(camera, float), np.asarray(table, float)
    for matrix in (camera, table):
        if matrix.shape != (4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0,0,0,1]):
            raise ValueError('Invalid transform')
    center = np.mean(hull, axis=0)
    ray = np.r_[cv2.undistortPoints(center.reshape(1,1,2), k, d).ravel(), 1.]
    ray = camera[:3,:3] @ ray; origin = camera[:3,3]
    normal = table[:3,2]; plane = table[:3,3] + size/2*normal
    denominator = normal @ ray
    if abs(denominator) < 1e-4: raise ValueError('Viewing ray almost parallel to table')
    distance = normal @ (plane-origin)/denominator
    if distance <= 0: raise ValueError('Table intersection behind camera')
    seed = (np.linalg.inv(table) @ np.r_[origin+distance*ray, 1])[:2]
    candidates = []
    def residual(v):
        predicted = projected_hull(cube_transform(table,v,size), camera,k,d,size)
        # Two directions penalize both shrinkage and oversized silhouette fits.
        return np.r_[distances_to_polygon(observed,predicted),
                     distances_to_polygon(sample_boundary(predicted,64),hull)]
    for yaw in [0., np.pi/8, np.pi/4, 3*np.pi/8]:
        fit = least_squares(residual, [*seed,yaw],
            bounds=([seed[0]-.06,seed[1]-.06,-np.pi], [seed[0]+.06,seed[1]+.06,np.pi]),
            diff_step=1e-3, max_nfev=60, ftol=1e-7, xtol=1e-7, gtol=1e-7,
            loss='soft_l1', f_scale=1.)
        candidates.append(fit)
    fit = min(candidates, key=lambda f:np.mean(residual(f.x)**2))
    cube = cube_transform(table,fit.x,size)
    predicted = projected_hull(cube,camera,k,d,size)
    rendered = np.zeros(mask.shape,np.uint8)
    cv2.fillConvexPoly(rendered,np.rint(predicted).astype(np.int32),1)
    intersection = int(np.count_nonzero(mask & (rendered>0)))
    union = int(np.count_nonzero(mask | (rendered>0)))
    iou = intersection/union
    boundary = residual(fit.x)
    rms = float(np.sqrt(np.mean(boundary**2)))
    p95 = float(np.percentile(boundary,95))
    maximum = float(np.max(boundary))
    strict = iou >= .90 and rms <= 2.5
    errors = []
    if iou < FIT_LIMITS['minimum_iou']:
        errors.append(f"轮廓重合度 {iou:.2%} < {FIT_LIMITS['minimum_iou']:.0%}")
    for label, value, key in [('边界均方误差',rms,'maximum_rms_px'),
                              ('边界95分位误差',p95,'maximum_p95_px'),
                              ('边界最大误差',maximum,'maximum_boundary_px')]:
        if value > FIT_LIMITS[key]:
            errors.append(f"{label} {value:.2f}px > {FIT_LIMITS[key]:.2f}px")
    accepted = strict or not errors
    return dict(schema='d1-mask-cube-pose-v1', geometric_fit_accepted=accepted,
        T_base_cube=cube.tolist(), T_base_table=table.tolist(), T_base_camera=camera.tolist(),
        cube_size_m=size, silhouette_iou=iou, silhouette_boundary_rms_px=rms,
        silhouette_boundary_p95_px=p95, silhouette_boundary_max_px=maximum,
        fit_acceptance_mode='strict' if strict else ('bounded_boundary' if accepted else 'rejected'),
        fit_limits=FIT_LIMITS.copy(), fit_rejection_reasons=[] if accepted else errors,
        predicted_hull_px=predicted.tolist(), yaw_modulo_90_deg=float(np.rad2deg(fit.x[2])%90),
        fit_parameters_table=[float(x) for x in fit.x], optimizer_evaluations=int(fit.nfev),
        hardware_executed=False, human_grasp_approval_received=False,
        limitations=['Existing fixed table plane and camera mapping assumed; no new calibration.',
                     'Single complete unoccluded 40 mm cube only; yaw is symmetric modulo 90 degrees.',
                     'Fit thresholds are engineering filters, not measured grasp accuracy or clearance.'])
