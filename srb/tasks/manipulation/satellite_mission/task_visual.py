from typing import Dict, Sequence

import torch

from srb.core.env import ManipulationEnvVisualExtCfg, VisualExt
from srb.utils.cfg import configclass
from srb.core.sensor import CameraCfg
from srb.utils.math import rpy_to_quat

from .task import Task, TaskCfg


@configclass
class VisualTaskCfg(ManipulationEnvVisualExtCfg, TaskCfg):
    debug_vis: bool = True
    camera_resolution = (320, 240)
    camera_update_period: float = 0.1
    # Camera translations are in the parent link frame. Isaac quaternions: wxyz.
    wrist_camera_pos = (0.0, 0.0, -1.1)
    wrist_camera_rpy = (0.0, 0.0, -90.0)
    overview_camera_pos = (10.0, -12.0, 10.0)
    overview_camera_rpy = (0.0, 15.8, 118.7)

    def __post_init__(self):
        TaskCfg.__post_init__(self)
        ManipulationEnvVisualExtCfg.wrap(self, env_cfg=self)
        self.scene.cam_wrist.offset = CameraCfg.OffsetCfg(
            convention="opengl", pos=self.wrist_camera_pos,
            rot=rpy_to_quat(*self.wrist_camera_rpy),
        )
        self.scene.cam_base.offset = CameraCfg.OffsetCfg(
            convention="world", pos=self.overview_camera_pos,
            rot=rpy_to_quat(*self.overview_camera_rpy),
        )
        self.scene.cam_base.spawn.clipping_range = (0.1, 100.0)


class VisualTask(VisualExt, Task):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        Task.__init__(self, cfg, **kwargs)
        VisualExt.__init__(self, cfg, **kwargs)

    def _reset_idx(self, env_ids: Sequence[int]):
        Task._reset_idx(self, env_ids)
        VisualExt._reset_idx(self, env_ids)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            **Task._get_observations(self),
            **VisualExt._get_observations(self),
        }
