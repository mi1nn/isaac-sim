"""Make ROS 2 importable from inside Isaac Sim, whichever way it is available.

Isaac Sim runs its own CPython (3.11 for Isaac Sim 5.x), so a system ROS 2 built for
a different interpreter cannot be imported into it. For that case the
`isaacsim.ros2.bridge` extension ships a complete ROS 2 Python stack compiled for
Isaac Sim's interpreter, under

    <isaac-sim>/exts/isaacsim.ros2.bridge/<distro>/rclpy

`enable_ros2_bridge()` puts that directory on `sys.path` and the matching native
libraries on `LD_LIBRARY_PATH`.

One gap has to be papered over: the bundled stack ships `tf2_msgs` but not
`tf2_ros`, because `tf2_ros` pulls in the compiled `tf2_py` module. The ROS
interface only ever *publishes* transforms, and both broadcasters need nothing but
`rclpy`, `tf2_msgs` and `geometry_msgs`, so equivalents are provided below and used
whenever the real `tf2_ros` is absent.
"""

import sys
from functools import cache
from importlib.util import find_spec
from os import environ
from pathlib import Path
from typing import Literal

## Distros the ros2_bridge extension may ship, newest first.
SUPPORTED_DISTROS = ("jazzy", "humble")


@cache
def enable_ros2_bridge(
    distro: str | None = None,
    rmw_implementation: Literal["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"]
    | str = "rmw_fastrtps_cpp",
) -> bool:
    from srb.utils import logging

    ## Skip if ROS 2 is already sourced
    if find_spec("rclpy"):
        logging.debug(
            'ROS 2 is already sourced in the current environment, so "ros2_bridge" does not need to be enabled'
        )
        return True

    ## Update environment
    distro = _resolve_distro(distro)
    bridge_dir = _get_ros2_bridge_dir(distro)
    _append_ld_library_path(bridge_dir.joinpath("lib"))
    # The native libraries above are found by the dynamic loader, but the Python
    # packages need to be on sys.path as well -- without this, enabling the extension
    # leaves `import rclpy` failing with ModuleNotFoundError.
    _prepend_sys_path(bridge_dir.joinpath("rclpy"))
    if not environ.get("RMW_IMPLEMENTATION"):
        environ["RMW_IMPLEMENTATION"] = rmw_implementation
    if not environ.get("ROS_DISTRO"):
        environ["ROS_DISTRO"] = distro

    ## Enable extension
    from isaacsim.core.utils.extensions import enable_extension

    assert enable_extension("isaacsim.ros2.bridge")

    ## Check if ROS 2 is now available
    if not find_spec("rclpy"):
        logging.error(
            'ROS 2 Python client library "rclpy" is still not available after trying '
            f'to enable the "ros2_bridge" extension (distro "{distro}", '
            f"searched {bridge_dir.joinpath('rclpy')})"
        )
        return False

    logging.info(f'ROS 2 "{distro}" enabled from the ros2_bridge extension')
    return True

def require_ros2_bridge() -> None:
    """Enable the ROS 2 bridge or stop startup with an actionable error."""
    if not enable_ros2_bridge():
        raise RuntimeError("ROS 2 bridge initialization failed")



@cache
def _resolve_distro(distro: str | None) -> str:
    """Pick the ROS 2 distro whose bundle to use.

    An explicit argument wins, then `ROS_DISTRO` if the extension ships it, then
    whichever bundled distro is present. Guessing a single hardcoded distro breaks
    silently on machines that have the other one.
    """
    from srb.utils.isaacsim import get_isaacsim_path

    exts_dir = Path(get_isaacsim_path()).joinpath("exts", "isaacsim.ros2.bridge")
    available = [d for d in SUPPORTED_DISTROS if exts_dir.joinpath(d).is_dir()]
    if not available:
        raise FileNotFoundError(
            f"The ros2_bridge extension ships no ROS 2 distro under {exts_dir}"
        )

    if distro is not None:
        if distro not in available:
            raise ValueError(
                f'ROS 2 distro "{distro}" is not shipped by the ros2_bridge extension; '
                f"available: {available}"
            )
        return distro

    env_distro = environ.get("ROS_DISTRO")
    if env_distro in available:
        return env_distro
    return available[0]


@cache
def _get_ros2_bridge_dir(distro: str) -> Path:
    from srb.utils.isaacsim import get_isaacsim_path

    bridge_dir = Path(get_isaacsim_path()).joinpath(
        "exts", "isaacsim.ros2.bridge", distro
    )
    assert bridge_dir.exists(), f"Missing ros2_bridge distro directory: {bridge_dir}"
    return bridge_dir


@cache
def _prepend_sys_path(path: Path):
    entry = path.as_posix()
    if path.is_dir() and entry not in sys.path:
        sys.path.insert(0, entry)


@cache
def _append_ld_library_path(lib_path: Path):
    ld_library_path = environ.get("LD_LIBRARY_PATH", "")
    if ld_library_path:
        if lib_path.as_posix() in ld_library_path:
            return
        ld_library_path = f"{ld_library_path}:".replace("::", ":")
    environ["LD_LIBRARY_PATH"] = ld_library_path + lib_path.as_posix()


## Transform broadcasters ##


def get_transform_broadcasters():
    """Return `(TransformBroadcaster, StaticTransformBroadcaster)`.

    Uses the real `tf2_ros` when it is importable, and the equivalents below when it
    is not -- which is the case inside Isaac Sim, whose bundled ROS 2 stack omits
    `tf2_ros` (it depends on the compiled `tf2_py`, which is not shipped).
    """
    try:
        from tf2_ros import TransformBroadcaster as _Dynamic
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster as _Static

        return _Dynamic, _Static
    except ImportError:
        return TransformBroadcaster, StaticTransformBroadcaster


class TransformBroadcaster:
    """Publishes `geometry_msgs/TransformStamped` on `/tf`.

    Behaviourally identical to `tf2_ros.TransformBroadcaster` for publishing; the
    parts of `tf2_ros` that listen to and interpolate transforms are not reproduced.
    """

    def __init__(self, node, qos=None):
        from rclpy.qos import QoSProfile
        from tf2_msgs.msg import TFMessage

        if qos is None:
            qos = QoSProfile(depth=100)
        elif isinstance(qos, int):
            qos = QoSProfile(depth=qos)
        self.pub_tf = node.create_publisher(TFMessage, "/tf", qos)

    def sendTransform(self, transform):  # noqa: N802 (matches the tf2_ros API)
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        if isinstance(transform, TransformStamped):
            transform = [transform]
        if any(not isinstance(t, TransformStamped) for t in transform):
            raise TypeError(
                "sendTransform() expects a TransformStamped or a list of them"
            )
        self.pub_tf.publish(TFMessage(transforms=transform))


class StaticTransformBroadcaster:
    """Publishes latched transforms on `/tf_static`.

    Like `tf2_ros.StaticTransformBroadcaster`, it keeps the full set it has been given
    and republishes it whenever it changes, so that late subscribers receive every
    static transform rather than only the most recent one.
    """

    def __init__(self, node, qos=None):
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        from tf2_msgs.msg import TFMessage

        if qos is None:
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        self._net_message = TFMessage()
        self.pub_tf = node.create_publisher(TFMessage, "/tf_static", qos)

    def sendTransform(self, transform):  # noqa: N802 (matches the tf2_ros API)
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        if isinstance(transform, TransformStamped):
            transform = [transform]
        if any(not isinstance(t, TransformStamped) for t in transform):
            raise TypeError(
                "sendTransform() expects a TransformStamped or a list of them"
            )
        for new in transform:
            # One transform per (parent, child) pair: a repeat replaces the old one.
            self._net_message.transforms = [
                existing
                for existing in self._net_message.transforms
                if existing.header.frame_id != new.header.frame_id
                or existing.child_frame_id != new.child_frame_id
            ]
            self._net_message.transforms.append(new)
        self.pub_tf.publish(self._net_message)


__all__ = [
    "SUPPORTED_DISTROS",
    "StaticTransformBroadcaster",
    "TransformBroadcaster",
    "enable_ros2_bridge",
    "get_transform_broadcasters",
    "require_ros2_bridge",
]
