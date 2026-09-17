from typing import Dict, Sequence, Tuple

import torch
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, UsdShade

from srb.core.env import ManipulationEnvVisualExtCfg, VisualExt
from srb.core.sensor import CameraCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat
from srb.utils.str import resolve_env_prim_path

from .task import Task, TaskCfg


@configclass
class VisualTaskCfg(ManipulationEnvVisualExtCfg, TaskCfg):
    ## Wrist camera (mounted on the 7th link, looking along the gripper approach axis)
    # NOTE: The offset uses the USD/OpenGL camera convention (looks along its local -Z),
    #       which matches the values set via the Isaac Sim property panel
    wrist_camera_pos: Tuple[float, float, float] = (0.0, 0.0, -1.1)
    wrist_camera_rpy: Tuple[float, float, float] = (0.0, 0.0, -90.0)
    wrist_camera_resolution: Tuple[int, int] = (320, 240)

    ## Grasp handle marker (makes the MEP handle detectable in the RGB image)
    # Only the pin (the grasped part) is coloured; the Y-shaped mount keeps its grey
    # look, so the detector does not mix the mount into the pin estimate
    handle_prim_relpath: str = "mep/gripper_fixture/Cylinder_01"
    handle_marker_color: Tuple[float, float, float] | None = (0.0, 1.0, 0.0)

    def __post_init__(self):
        TaskCfg.__post_init__(self)
        ManipulationEnvVisualExtCfg.wrap(self, env_cfg=self)

        # Keep the skydome (Earth) orientation fixed instead of randomizing it
        # (re-asserted here in case the visual extension's wrap() re-enables it)
        self.events.randomize_skydome_orientation = None

        # Wrist camera
        cam_wrist = getattr(self.scene, "cam_wrist", None)
        if isinstance(cam_wrist, CameraCfg):
            cam_wrist.offset = CameraCfg.OffsetCfg(
                convention="opengl",
                pos=self.wrist_camera_pos,
                rot=rpy_to_quat(*self.wrist_camera_rpy),
            )
            cam_wrist.width, cam_wrist.height = self.wrist_camera_resolution


class VisualTask(VisualExt, Task):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        Task.__init__(self, cfg, **kwargs)
        VisualExt.__init__(self, cfg, **kwargs)

    def _setup_scene(self):
        super()._setup_scene()

        if self.cfg.handle_marker_color is not None:
            self._apply_handle_marker(self.cfg.handle_marker_color)

    def _apply_handle_marker(self, color: Tuple[float, float, float]):
        stage = get_current_stage()
        material_path = "/World/Looks/handle_marker"
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        # Emissive so that the marker keeps its hue in shadow and in direct sunlight
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )

        for i in range(self.num_envs):
            prim = stage.GetPrimAtPath(
                resolve_env_prim_path(
                    f"{{ENV_REGEX_NS}}/{self.cfg.handle_prim_relpath}", i
                )
            )
            if not prim.IsValid():
                continue
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, UsdShade.Tokens.strongerThanDescendants
            )

    def _reset_idx(self, env_ids: Sequence[int]):
        Task._reset_idx(self, env_ids)
        VisualExt._reset_idx(self, env_ids)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            **Task._get_observations(self),
            **VisualExt._get_observations(self),
        }
