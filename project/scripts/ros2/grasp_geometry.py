"""ROS-independent RGB-D geometry. Metres; optical +Z; ROS xyzw quaternion."""
import numpy as np
from scipy.spatial.transform import Rotation


def backproject(mask, depth, k, min_pixels=12, max_depth=30.0):
    if mask.shape != depth.shape or k.shape != (3, 3):
        raise ValueError("Mask/depth resolution or camera intrinsics mismatch")
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("Invalid camera intrinsics")
    v, u = np.nonzero(mask)
    z = depth[v, u]
    valid = np.isfinite(z) & (z > 0) & (z < max_depth)
    u, v, z = u[valid], v[valid], z[valid]
    if len(z) < min_pixels:
        return None
    return np.column_stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z))


def grasp_from_points(points, camera_position, previous_axis=None, radius=0.0):
    """Side grasp: tool X follows the pin, tool -Z approaches it.

    All arguments/results are in the robot base frame. Radius compensates the
    visible cylinder surface, and is a calibrated handle parameter, not GT pose.
    """
    center = np.median(points, axis=0)
    values, vectors = np.linalg.eigh(np.cov((points-center).T))
    if values[-1] < 3.0 * max(values[-2], 1e-10):
        return None
    x = vectors[:, -1]
    if previous_axis is not None and np.dot(x, previous_axis) < 0:
        x = -x
    direction = center-camera_position
    direction -= np.dot(direction, x)*x
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return None
    direction /= length
    center += radius*direction
    z = -direction
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return center, Rotation.from_matrix(np.column_stack((x, y, z))).as_quat(), x


def limited(vector, limit):
    norm = np.linalg.norm(vector)
    return vector * min(1.0, limit/max(norm, 1e-12))


def pose_error(target_position, target_xyzw, current_position, current_xyzw):
    # Left rotation error is expressed in the same base coordinates as translation.
    rotation = Rotation.from_quat(target_xyzw) * Rotation.from_quat(current_xyzw).inv()
    return target_position-current_position, rotation.as_rotvec()
