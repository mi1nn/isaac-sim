from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset import ArticulationCfg
from srb.core.asset.common import Frame
from srb.core.asset.robot.mobile.mobile_robot import MobileRobot, MobileRobotRegistry
from srb.core.asset.robot.mobile.mobile_robot_type import MobileRobotType


class GroundRobot(MobileRobot, mobile_robot_entrypoint=MobileRobotType.GROUND):
    ## Model
    asset_cfg: ArticulationCfg

    ## Frames
    frame_front_camera: Frame

    @classmethod
    def mobile_robot_registry(cls) -> Sequence[Type[GroundRobot]]:
        return MobileRobotRegistry.registry.get(MobileRobotType.GROUND, [])  # type: ignore


class WheeledRobot(GroundRobot, mobile_robot_metaclass=True):
    pass


class LeggedRobot(GroundRobot, mobile_robot_metaclass=True):
    ## Links
    regex_feet_links: str
