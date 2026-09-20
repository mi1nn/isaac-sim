import pytest

from mep_monitor.csv_logger import (
    build_metric_values,
)


def test_csv_metric_values():

    values = build_metric_values(
        target_xyz=(4.0, 6.0, 3.0),
        actual_xyz=(1.0, 2.0, 3.0),

        target_xyzw=(
            0.0,
            0.0,
            0.70710678,
            0.70710678,
        ),

        actual_xyzw=(
            0.0,
            0.0,
            0.0,
            1.0,
        ),

        target_velocity_xyz=(
            0.03,
            0.04,
            0.0,
        ),

        actual_velocity_xyz=(
            0.0,
            0.0,
            0.0,
        ),
    )

    assert values[
        'error_x'
    ] == pytest.approx(3.0)

    assert values[
        'error_y'
    ] == pytest.approx(4.0)

    assert values[
        'error_z'
    ] == pytest.approx(0.0)

    assert values[
        'active_position_error'
    ] == pytest.approx(5.0)

    assert values[
        'orientation_error_deg'
    ] == pytest.approx(90.0)

    assert values[
        'relative_velocity_mps'
    ] == pytest.approx(0.05)

    assert values[
        'metrics_valid'
    ] is True
