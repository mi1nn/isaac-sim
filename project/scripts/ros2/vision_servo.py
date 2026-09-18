"""Geometry and detection shared by the MEP capture and peg-in-hole vision servos.

Split out of `mep_suction_capture.py` so the parts that can be checked without a
simulator live somewhere a unit test can reach them (see
`project/tests/vision_servo_test.py`).

Everything below NumPy-only; `cv2` is imported lazily inside the two functions that
need it, so the fitting maths stays importable in an environment without OpenCV.

Two shape fits carry the whole servo:

  * `fit_plane_ransac` gives the MEP capture marker's front face. The marker is a
    puck (radius 0.0825 m, 0.11 m thick), so the green blob's point cloud is a flat
    disc plus a rim of side wall; RANSAC on the near half of the cloud isolates the
    disc and its normal is the axis the arm must approach along. Aligning to it is
    what stops the MEP being welded on crooked.

  * `fit_circle_3d` gives the green ring around the thruster nozzle. A ring is worth
    more than a blob: it yields the hole's centre AND its axis, and it still fits
    from a partial arc, which matters because the far side of the ring goes out of
    frame as the peg closes in.
"""

from typing import Optional, Tuple

import numpy as np

Plane = Tuple[np.ndarray, np.ndarray]  # (point on the plane, unit normal)
Circle = Tuple[np.ndarray, np.ndarray, float]  # (centre, unit normal, radius)


## Small vector helpers ##


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def clamp_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = np.linalg.norm(v)
    return v * (max_norm / n) if n > max_norm else v


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    return float(
        np.arccos(np.clip(np.dot(normalize(a), normalize(b)), -1.0, 1.0))
    )


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = normalize(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


## Point-cloud selection ##


def near_face_points(
    points: np.ndarray, *, fraction: float = 0.5, min_points: int = 12
) -> np.ndarray:
    """Keep the points closest to the camera, i.e. the flat face of a stubby shape.

    `points` are in the camera optical frame, where +Z is depth, so "closest" is
    simply the smallest Z. The marker's side wall is always farther than its front
    face, which is what makes this a good enough segmentation to fit a plane to.
    """
    if len(points) <= min_points:
        return points
    keep = max(min_points, int(round(fraction * len(points))))
    order = np.argsort(points[:, 2])
    return points[order[:keep]]


## Shape fits ##


def fit_plane_ransac(
    points: np.ndarray,
    *,
    iterations: int = 64,
    inlier_tol: float = 3e-3,
    orient_towards: Optional[np.ndarray] = None,
    rng: Optional[np.random.Generator] = None,
) -> Optional[Plane]:
    """Fit a plane, returning its inlier centroid and unit normal.

    RANSAC rather than a plain SVD because the green mask always drags in a few
    pixels of background or side wall at its edge, and those sit far enough off the
    face to tilt a least-squares fit by several degrees.

    `orient_towards` flips the normal so it points at that position (the camera),
    which removes the sign ambiguity a plane fit cannot resolve on its own.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        return None
    rng = rng if rng is not None else np.random.default_rng()

    best_inliers: Optional[np.ndarray] = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        if np.linalg.norm(normal) < 1e-12:  # collinear sample
            continue
        normal = normalize(normal)
        inliers = np.abs((points - sample[0]) @ normal) < inlier_tol
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers

    subset = points[best_inliers] if best_inliers is not None else points
    if len(subset) < 3:
        subset = points

    # Refit on every inlier: the 3-point hypothesis only selected them.
    centroid = subset.mean(axis=0)
    _, _, vt = np.linalg.svd(subset - centroid)
    normal = normalize(vt[2])

    if orient_towards is not None and np.dot(np.asarray(orient_towards) - centroid, normal) < 0.0:
        normal = -normal
    return centroid, normal


def fit_circle_3d(points: np.ndarray) -> Optional[Circle]:
    """Fit a circle to a 3D point cloud that lies on one (an arc, or an annulus).

    The plane comes from an SVD rather than RANSAC: unlike the marker face, every
    point of the ring is genuinely on the ring, so there is no second surface to
    reject. In-plane the radius and centre come from Kasa's algebraic fit, which is
    linear, needs no initial guess and stays well conditioned on a partial arc.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 4:
        return None

    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    normal = normalize(vt[2])
    u, v = vt[0], vt[1]

    local = (points - centroid) @ np.stack([u, v], axis=1)  # (N, 2)
    # Kasa: solve |p|^2 = 2*cx*x + 2*cy*y + c for (cx, cy, c)
    A = np.column_stack([2.0 * local, np.ones(len(local))])
    b = np.sum(local**2, axis=1)
    solution, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = solution
    radius_sq = c + cx * cx + cy * cy
    if not np.isfinite(radius_sq) or radius_sq <= 0.0:
        return None

    centre = centroid + cx * u + cy * v
    return centre, normal, float(np.sqrt(radius_sq))


## Image -> point cloud ##


def detect_color_blob(
    rgb: np.ndarray,
    hsv_lower,
    hsv_upper,
    *,
    min_pixels: int = 12,
    depth: Optional[np.ndarray] = None,
    max_depth: Optional[float] = None,
) -> np.ndarray:
    """Largest connected component within an HSV range, as a uint8 mask.

    One detector serves both phases: the thruster ring is painted the same emissive
    green as the capture marker on purpose, and the two never share a camera.

    That sharing is exactly why `depth`/`max_depth` exist. The wrist camera can see
    both -- and at the start of a run the ring, though 17 m away, covers more pixels
    than the marker at 6 m, so "largest green component" picks the wrong one. Gating
    on range BEFORE the component search fixes that; gating afterwards would merely
    discard the detection and leave the arm blind.
    """
    import cv2

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    if depth is not None and max_depth is not None:
        in_range = np.isfinite(depth) & (depth > 0.0) & (depth < max_depth)
        mask = np.where(in_range, mask, 0).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return np.zeros_like(mask)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    blob = (labels == largest).astype(np.uint8) * 255
    return blob if int(stats[largest, cv2.CC_STAT_AREA]) >= min_pixels else np.zeros_like(mask)


def backproject(
    mask: np.ndarray,
    depth: np.ndarray,
    K: np.ndarray,
    *,
    min_pixels: int = 12,
    max_depth: float = 30.0,
) -> Optional[np.ndarray]:
    """Masked pixels -> points in the camera optical frame."""
    vs, us = np.nonzero(mask)
    if len(us) < min_pixels:
        return None
    z = depth[vs, us].astype(np.float64)
    valid = np.isfinite(z) & (z > 0.0) & (z < max_depth)
    if valid.sum() < min_pixels:
        return None
    us, vs, z = us[valid], vs[valid], z[valid]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)
