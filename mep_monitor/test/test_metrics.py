import math

import pytest

from mep_monitor.metrics import (
    orientation_error_deg,
    position_error,
    relative_linear_velocity,
)


def test_position_error():
    ex, ey, ez, norm = position_error(
        target_xyz=(4.0, 6.0, 3.0),
        actual_xyz=(1.0, 2.0, 3.0),
    )

    assert ex == pytest.approx(3.0)
    assert ey == pytest.approx(4.0)
    assert ez == pytest.approx(0.0)
    assert norm == pytest.approx(5.0)


def test_orientation_error_identity():
    error = orientation_error_deg(
        target_xyzw=(0.0, 0.0, 0.0, 1.0),
        actual_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert error == pytest.approx(0.0)


def test_orientation_error_90_deg_about_z():
    half_angle = math.radians(45.0)

    target = (
        0.0,
        0.0,
        math.sin(half_angle),
        math.cos(half_angle),
    )

    actual = (0.0, 0.0, 0.0, 1.0)

    assert orientation_error_deg(target, actual) == pytest.approx(90.0)


def test_quaternion_sign_represents_same_orientation():
    assert orientation_error_deg(
        target_xyzw=(0.0, 0.0, 0.0, 1.0),
        actual_xyzw=(0.0, 0.0, 0.0, -1.0),
    ) == pytest.approx(0.0)


def test_relative_linear_velocity():
    speed = relative_linear_velocity(
        target_velocity_xyz=(0.03, 0.04, 0.0),
        actual_velocity_xyz=(0.0, 0.0, 0.0),
    )

    assert speed == pytest.approx(0.05)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        orientation_error_deg(
            target_xyzw=(0.0, 0.0, 0.0, 0.0),
            actual_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
