from typing import Dict, Sequence, Tuple

import torch
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, UsdShade

from srb.core.env import ManipulationEnvVisualExtCfg, VisualExt
from srb.core.sensor import CameraCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat

from .task import Task, TaskCfg


@configclass
class VisualTaskCfg(ManipulationEnvVisualExtCfg, TaskCfg):
    ## Wrist camera, mounted on link 7 and looking the way the arm reaches.
    # NOTE: The offset uses the USD/OpenGL camera convention (the camera looks along
    #       its local -Z), which matches the values set via the Isaac Sim property
    #       panel. These defaults repeat `Canadarm3.frame_wrist_camera` on purpose:
    #       `scripts/ros2/mep_suction_capture.py` needs the exact mount to turn a pixel
    #       into a point in the env frame, and reads these very numbers as parameters.
    wrist_camera_pos: Tuple[float, float, float] = (0.0, 0.0, -0.9)
    wrist_camera_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    wrist_camera_resolution: Tuple[int, int] = (320, 240)

    ## Capture marker colour.
    # `CaptureCfg.marker_relpath` is the cylinder whose centre the arm must reach, but
    # in the raw asset it is the same grey as the rest of the MEP and no colour-based
    # detector can find it. Painting it emissive green is what makes the capture point
    # visible to the wrist camera. Set to None to keep the asset's own material.
    marker_color: Tuple[float, float, float] | None = (0.0, 1.0, 0.0)

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

        if self.cfg.marker_color is not None:
            self._apply_marker_color(self.cfg.marker_color)

    def _apply_marker_color(self, color: Tuple[float, float, float]):
        """Paint the MEP capture marker so the wrist camera can find it."""
        stage = get_current_stage()
        material_path = "/World/Looks/capture_marker"
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        # Emissive so the marker keeps its hue in shadow and in direct sunlight alike,
        # which is what lets a fixed HSV range work across the whole orbit.
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )

        debris_name = self.cfg.scene.debris.prim_path.rsplit("/", 1)[-1]
        marker_relpath = self.cfg.capture.marker_relpath
        bound = 0
        for env_prim_path in self.scene.env_prim_paths:
            prim = stage.GetPrimAtPath(
                f"{env_prim_path}/{debris_name}/{marker_relpath}"
            )
            if not prim.IsValid():
                continue
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, UsdShade.Tokens.strongerThanDescendants
            )
            bound += 1
        if not bound:
            raise ValueError(
                f"Capture marker '{marker_relpath}' not found under the debris prim; "
                "check CaptureCfg.marker_relpath against the MEP asset."
            )
        print(f"[capture] marker painted {color} on {bound} env(s)", flush=True)

    def _reset_idx(self, env_ids: Sequence[int]):
        Task._reset_idx(self, env_ids)
        VisualExt._reset_idx(self, env_ids)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            **Task._get_observations(self),
            **VisualExt._get_observations(self),
        }
