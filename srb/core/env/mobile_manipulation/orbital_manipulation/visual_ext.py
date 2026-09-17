from srb.core.env.common.extension.visual import VisualExtCfg
from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat

from .env import OrbitalManipulationEnvCfg


@configclass
class OrbitalManipulationEnvVisualExtCfg(VisualExtCfg):
    def wrap(self, env_cfg: OrbitalManipulationEnvCfg):
        self.cameras_cfg = {
            "cam_scene": CameraCfg(
                prim_path=f"{env_cfg._robot.asset_cfg.prim_path}{('/' + env_cfg._robot.frame_base.prim_relpath) if env_cfg._robot.frame_base.prim_relpath else ''}/camera_scene",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=(-2.5, 0.0, 2.5),
                    rot=rpy_to_quat(0.0, 45.0, 0.0),
                ),
                spawn=PinholeCameraCfg(
                    clipping_range=(0.05, 75.0 - 0.05),
                ),
            ),
            "cam_wrist": CameraCfg(
                prim_path=f"{env_cfg._robot.manipulator.asset_cfg.prim_path}/{env_cfg._robot.manipulator.frame_wrist_camera.prim_relpath}",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=env_cfg._robot.manipulator.frame_wrist_camera.offset.pos,
                    rot=env_cfg._robot.manipulator.frame_wrist_camera.offset.rot,
                ),
                spawn=PinholeCameraCfg(
                    focal_length=10.0,
                    horizontal_aperture=16.0,
                    clipping_range=(0.001, 1.5 - 0.001),
                ),
            ),
        }

        super().wrap(env_cfg)
