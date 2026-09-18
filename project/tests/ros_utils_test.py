from unittest.mock import patch

import pytest

from srb.utils import ros


def test_require_ros2_bridge_raises_when_enable_fails():
    """Catch regressions that let startup continue without an available bridge."""
    with patch.object(ros, "enable_ros2_bridge", return_value=False):
        with pytest.raises(RuntimeError, match="ROS 2 bridge initialization failed"):
            ros.require_ros2_bridge()
