from functools import cache
from importlib.util import find_spec
from os import environ
from pathlib import Path
from typing import Literal


def prepare_ros2_process():
    """Set native library paths before Kit starts (dlopen ignores late LD changes).

    External ROS nodes still run with their own system Python. Isaac uses its
    bundled Python-compatible bridge when no rclpy is already available.
    """
    import os
    import sys

    if find_spec("rclpy"):
        return
    distro = environ.get("ROS_DISTRO") or ("jazzy" if Path("/opt/ros/jazzy").exists() else "humble")
    lib = _get_ros2_bridge_lib_path(distro).as_posix()
    environ["ROS_DISTRO"] = distro
    environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    paths = environ.get("LD_LIBRARY_PATH", "").split(":")
    if lib not in paths:
        environ["LD_LIBRARY_PATH"] = ":".join([lib, *filter(None, paths)])
        # Re-exec once, before SimulationApp/GPU startup, preserving all CLI args.
        # The CLI parser consumes sys.argv; orig_argv retains flags and subcommands.
        os.execve(sys.executable, [sys.executable, *sys.orig_argv[1:]], dict(environ))


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
    distro = distro or environ.get("ROS_DISTRO", "humble")
    environ.setdefault("ROS_DISTRO", distro)
    _append_ld_library_path(_get_ros2_bridge_lib_path(distro))
    if not environ.get("RMW_IMPLEMENTATION"):
        environ["RMW_IMPLEMENTATION"] = rmw_implementation

    ## Enable extension
    from isaacsim.core.utils.extensions import enable_extension

    assert enable_extension("isaacsim.ros2.bridge")

    ## Check if ROS 2 is now available
    if not find_spec("rclpy"):
        logging.error(
            'ROS 2 Python client library "rclpy" is still not available after trying to enable the "ros2_bridge" extension'
        )
        return False

    return True


@cache
def _get_ros2_bridge_lib_path(distro: str) -> Path:
    from srb.utils.isaacsim import get_isaacsim_path

    ros2_lib_path = (
        Path(get_isaacsim_path())
        .joinpath("exts")
        .joinpath("isaacsim.ros2.bridge")
        .joinpath(distro)
        .joinpath("lib")
    )
    assert ros2_lib_path.exists()
    return ros2_lib_path


@cache
def _append_ld_library_path(lib_path: Path):
    ld_library_path = environ.get("LD_LIBRARY_PATH", "")
    if ld_library_path:
        if lib_path.as_posix() in ld_library_path:
            return
        ld_library_path = f"{ld_library_path}:".replace("::", ":")
    environ["LD_LIBRARY_PATH"] = ld_library_path + lib_path.as_posix()
