import math
from typing import Dict, Sequence, Tuple

import torch
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, UsdGeom, UsdShade

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

    ## Docking ring around the thruster nozzle.
    # The peg-in-hole phase needs a colour cue on the hole for the same reason the
    # capture phase needs one on the marker, and the nozzle offers none: it is dark,
    # small, and set against an equally dark bus.
    # NOTE: The satellite already carries a ring (`GOES_R/ThrustRim`), but it cannot
    # serve: it spans r 0.477-0.623 at y -2.196..-2.128 (asset units) while the bus
    # face at that radius reaches y=-2.198, so the rim sits *behind* the face and is
    # occluded. Hence a new one, out in front of the nozzle mouth.
    # All values are in the satellite's asset frame, which the spawn scales by 2.0.
    thruster_ring_color: Tuple[float, float, float] | None = (0.0, 1.0, 0.0)
    # Mouth of the nozzle is at y=-2.721; 0.04 in front of it avoids z-fighting and
    # keeps the ring 1.08 m clear of the bus face.
    thruster_ring_center: Tuple[float, float, float] = (-0.05, -2.761, -0.01)
    thruster_ring_axis: Tuple[float, float, float] = (0.0, -1.0, 0.0)
    # Inner radius stays outside the nozzle's outer wall (0.166) so the ring can never
    # block the peg, whose radius never exceeds 0.154 over the inserted length.
    # In world units this is a band from 0.42 m to 0.60 m.
    thruster_ring_radii: Tuple[float, float] = (0.21, 0.30)
    thruster_ring_segments: int = 64

    ## MEP-mounted camera, published as a sensor so the peg-in-hole node can see the
    ## ring from the peg's own point of view.
    # NOTE: Deliberately registered on the scene only, NOT in `cameras_cfg`. That dict
    # drives the visual observation space (`VisualExt.__init__`), which would both
    # change the observation shape for every agent and dereference
    # `cameras_cfg[key].spawn.clipping_range` -- None here. Everything in
    # `scene.sensors` is published by the ROS interface regardless, which is all this
    # camera is for.
    # `spawn=None` attaches to the camera the MEP asset already carries, the one
    # `TaskCfg.debris_camera_xform` aims down the insertion axis. Isaac Lab only
    # applies `CameraCfg.offset` when it spawns the prim itself, so the pose set in
    # `Task._place_debris_camera` survives.
    mep_camera_enabled: bool = True
    mep_camera_resolution: Tuple[int, int] = (320, 240)

    def __post_init__(self):
        TaskCfg.__post_init__(self)
        ManipulationEnvVisualExtCfg.wrap(self, env_cfg=self)

        # MEP camera (see `mep_camera_enabled`)
        if self.mep_camera_enabled and self.debris_camera_relpath:
            debris_name = self.scene.debris.prim_path.rsplit("/", 1)[-1]
            width, height = self.mep_camera_resolution
            self.scene.cam_mep = CameraCfg(
                # The asset's "Camera" is an Xform; the UsdGeom.Camera is its child.
                prim_path=(
                    "{ENV_REGEX_NS}/"
                    f"{debris_name}/{self.debris_camera_relpath}/Camera"
                ),
                spawn=None,
                width=width,
                height=height,
                data_types=["rgb", "depth"],
                update_period=self.agent_rate,
            )

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
        if self.cfg.thruster_ring_color is not None:
            self._spawn_thruster_ring(self.cfg.thruster_ring_color)

    @staticmethod
    def _emissive_material(stage, material_path: str, color) -> UsdShade.Material:
        """A flat, self-lit material in `color`.

        Emissive so the target keeps its hue in shadow and in direct sunlight alike,
        which is what lets one fixed HSV range work across the whole orbit.
        """
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        return material

    def _spawn_thruster_ring(self, color: Tuple[float, float, float]):
        """Build the green docking ring in front of the nozzle (see the cfg notes).

        A flat annulus rather than a disc, so the peg passes through it, and
        double-sided so it stays visible whichever face the camera ends up on. It is
        created after the satellite has spawned, so it picks up no CollisionAPI and
        contributes neither collision nor mass -- it is purely a colour cue.
        """
        stage = get_current_stage()
        material = self._emissive_material(stage, "/World/Looks/thruster_ring", color)

        inner, outer = self.cfg.thruster_ring_radii
        if not 0.0 < inner < outer:
            raise ValueError(
                f"thruster_ring_radii must be 0 < inner < outer, got {(inner, outer)}"
            )
        points, counts, indices = self._annulus_geometry(
            center=self.cfg.thruster_ring_center,
            axis=self.cfg.thruster_ring_axis,
            inner=inner,
            outer=outer,
            segments=self.cfg.thruster_ring_segments,
        )

        satellite_name = self.cfg.scene.satellite.prim_path.rsplit("/", 1)[-1]
        for env_prim_path in self.scene.env_prim_paths:
            mesh = UsdGeom.Mesh.Define(
                stage, f"{env_prim_path}/{satellite_name}/docking_ring"
            )
            mesh.CreatePointsAttr().Set(points)
            mesh.CreateFaceVertexCountsAttr().Set(counts)
            mesh.CreateFaceVertexIndicesAttr().Set(indices)
            mesh.CreateDoubleSidedAttr().Set(True)
            # Flat polygons: subdivision would round the ring's edges into the bore.
            mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
                material, UsdShade.Tokens.strongerThanDescendants
            )
        print(
            f"[capture] thruster ring {color} spawned on "
            f"{len(self.scene.env_prim_paths)} env(s): r {inner}-{outer} "
            f"(asset units) at {self.cfg.thruster_ring_center}",
            flush=True,
        )

    @staticmethod
    def _annulus_geometry(*, center, axis, inner: float, outer: float, segments: int):
        """Vertices and quads of a flat ring, in the frame `center`/`axis` live in."""
        n = Gf.Vec3d(*axis).GetNormalized()
        # Any vector not parallel to the axis seeds the in-plane basis
        seed = Gf.Vec3d(1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else Gf.Vec3d(0.0, 1.0, 0.0)
        u = Gf.Cross(n, seed).GetNormalized()
        v = Gf.Cross(n, u)
        origin = Gf.Vec3d(*center)

        points, counts, indices = [], [], []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            radial = math.cos(angle) * u + math.sin(angle) * v
            points.append(Gf.Vec3f(origin + inner * radial))
            points.append(Gf.Vec3f(origin + outer * radial))
        for i in range(segments):
            j = (i + 1) % segments
            counts.append(4)
            indices.extend([2 * i, 2 * i + 1, 2 * j + 1, 2 * j])
        return points, counts, indices

    def _apply_marker_color(self, color: Tuple[float, float, float]):
        """Paint the MEP capture marker so the wrist camera can find it."""
        stage = get_current_stage()
        material = self._emissive_material(stage, "/World/Looks/capture_marker", color)

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
