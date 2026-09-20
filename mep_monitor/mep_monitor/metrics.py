"""Pure monitoring metric calculations.

This module contains no ROS2 or Isaac Sim dependencies.

Internal quaternion convention:
    (x, y, z, w)

This does NOT assume that Isaac Sim publishes quaternions in this order.
The future ROS2/Isaac adapter must convert the actual source convention
to this internal convention after the GPU-side interface is verified.
"""

import math


def _finite_vector(values, size, name):
    values = tuple(float(v) for v in values)

    if len(values) != size:
        raise ValueError(f'{name} must contain exactly {size} values')

    if not all(math.isfinite(v) for v in values):
        raise ValueError(f'{name} must contain only finite values')

    return values


def position_error(target_xyz, actual_xyz):
    """Return (ex, ey, ez, norm) using target - actual."""
    target = _finite_vector(target_xyz, 3, 'target_xyz')
    actual = _finite_vector(actual_xyz, 3, 'actual_xyz')

    ex = target[0] - actual[0]
    ey = target[1] - actual[1]
    ez = target[2] - actual[2]

    norm = math.sqrt(ex * ex + ey * ey + ez * ez)

    return ex, ey, ez, norm


def orientation_error_deg(target_xyzw, actual_xyzw):
    """Return shortest quaternion orientation difference in degrees."""
    target = _finite_vector(target_xyzw, 4, 'target_xyzw')
    actual = _finite_vector(actual_xyzw, 4, 'actual_xyzw')

    target_norm = math.sqrt(sum(v * v for v in target))
    actual_norm = math.sqrt(sum(v * v for v in actual))

    if target_norm <= 1e-12:
        raise ValueError('target quaternion must have non-zero norm')

    if actual_norm <= 1e-12:
        raise ValueError('actual quaternion must have non-zero norm')

    target_unit = tuple(v / target_norm for v in target)
    actual_unit = tuple(v / actual_norm for v in actual)

    # q and -q represent the same physical orientation.
    dot = abs(sum(t * a for t, a in zip(target_unit, actual_unit)))
    dot = max(0.0, min(1.0, dot))

    angle_rad = 2.0 * math.acos(dot)

    return math.degrees(angle_rad)


def relative_linear_velocity(target_velocity_xyz, actual_velocity_xyz):
    """Return magnitude of target velocity minus actual velocity [m/s]."""
    target = _finite_vector(target_velocity_xyz, 3, 'target_velocity_xyz')
    actual = _finite_vector(actual_velocity_xyz, 3, 'actual_velocity_xyz')

    vx = target[0] - actual[0]
    vy = target[1] - actual[1]
    vz = target[2] - actual[2]

    return math.sqrt(vx * vx + vy * vy + vz * vz)
