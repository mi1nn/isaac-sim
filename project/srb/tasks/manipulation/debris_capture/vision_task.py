"""Vision-capture variant of the docking task (MRV Phase 1).

Adds to `DockingTask` (whose measured MEP placement, EE contact frame and magnet-style
`CaptureManager` FixedJoint are reused unchanged):

- 4 AprilTags on the MEP attachment face, centred on `gripper_fixture/Cylinder_01`
  (the ground-truth docking point, left untouched)
- `cam_wrist`: an RGB pinhole camera on the Canadarm3 last link, on the EE axis
- the 3 t MEP drifting through its nominal capture pose: linearly (`mep.motion_mode:
  translation_only`) or with a combined roll/pitch/yaw rate as well (`six_dof`)

The translucent capture cylinder of `CaptureManager` stays visible, sized to the
Canadarm3 flange (`capture.cylinder_radius_m`), and the MEP marker `Cylinder_01` is
scaled to the same radius (position, orientation and height unchanged) so that the
two overlap when aligned. The wrist camera looks through the capture cylinder along
the EE axis; its index of refraction is `capture.cylinder_ior` (1.0: no refraction).
"""

from pathlib import Path
from typing import Tuple

import numpy as np
from pxr import Usd, UsdGeom

from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass

from .docking import DockingTask, DockingTaskCfg, prim_frame, prim_scale, set_prim_pose
from .frames import Frame, frame_from_axes
from .vision import (
    CAM_MOUNT_QUAT_ROS,
    DEFAULT_CONFIG_PATH,
    VisionCaptureConfig,
    check_constellation_layout,
    load_vision_config,
    rendezvous_start_pose,
    spawn_tag_constellation,
)


@configclass
class VisionCaptureTaskCfg(DockingTaskCfg):
    vision_config_path: str = DEFAULT_CONFIG_PATH.as_posix()
    # `section.key=value` overrides of the YAML (see `load_vision_config`)
    vision_overrides: Tuple[str, ...] = ()

    def __post_init__(self):
        super().__post_init__()
        self.apply_vision_config()

    def load_vision(self) -> VisionCaptureConfig:
        return load_vision_config(self.vision_config_path, self.vision_overrides)

    def apply_vision_config(self):
        """(Re)apply the YAML values that live in the env config (call after overrides)."""
        v = self.load_vision()
        self.scene.debris.spawn.mass_props.mass = float(v.mep.mass_kg)
        self.scene.debris.init_state.lin_vel = tuple(v.mep.linear_velocity_w().tolist())
        # World frame [rad/s]; zero in translation_only
        self.scene.debris.init_state.ang_vel = tuple(v.mep.angular_velocity_w().tolist())
        # Capture cylinder (visual, collider, contact face) and the MEP marker
        # Cylinder_01 share one radius; the marker keeps its pose and height
        r = float(v.capture.cylinder_radius_m)
        self.capture.radius = r
        if self.debris_marker_xform is not None:
            translate, orient, scale = self.debris_marker_xform
            # UsdGeom.Cylinder radius 0.5 (axis Z) under the uniformly scaled MEP prim
            radial = r / (0.5 * float(self.scene.debris.spawn.scale[0]))
            self.debris_marker_xform = (translate, orient, (radial, radial, scale[2]))
        c = v.camera
        # Global RTX anti-aliasing (the default DLSS renders at a lower resolution and
        # upsamples, which moves tag edges by sub-pixels from frame to frame)
        self.sim.render.antialiasing_mode = c.antialiasing
        link = self._robot.frame_flange.prim_relpath
        setattr(
            self.scene,
            c.name,
            CameraCfg(
                prim_path=f"{self._robot.asset_cfg.prim_path}/{link}/{c.name}",
                offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, c.mount_link_z_m), rot=CAM_MOUNT_QUAT_ROS, convention="ros"),
                update_period=0.0,
                width=int(c.width) * int(c.supersample),
                height=int(c.height) * int(c.supersample),
                data_types=["rgb"],
                spawn=PinholeCameraCfg(
                    focal_length=c.focal_length_mm,
                    horizontal_aperture=c.horizontal_aperture_mm,
                    clipping_range=tuple(c.clipping_range_m),
                ),
            ),
        )


class VisionCaptureTask(DockingTask):
    cfg: VisionCaptureTaskCfg

    def _setup_scene(self):
        super()._setup_scene()
        self.vision_cfg = self.cfg.load_vision()
        v = self.vision_cfg
        stage = self.scene.stage
        geo = self.docking_geometry
        mep_path = geo.mep_path

        ## Ground-truth docking point (Cylinder_01) in the MEP body frame
        cyl_path = f"{mep_path}/{self.cfg.capture.marker_relpath}"
        mep_w = prim_frame(stage, mep_path)
        self.t_m_y: Frame = mep_w.inv() @ prim_frame(stage, cyl_path)
        self.cylinder_radius, self.cylinder_height = _cylinder_size(stage, cyl_path)

        ## Tag constellation frame T: on the attachment face (+ offset), centred on the
        ## projection of the Cylinder_01 centre, +Z = face normal, +X = grasp +X
        n = geo.mep_grasp.rot[:, 2]
        y = self.t_m_y.pos
        origin = y - float((y - geo.mep_grasp.pos) @ n) * n + v.apriltag.face_offset_m * n
        self.t_m_t: Frame = frame_from_axes(origin, n, geo.mep_grasp.rot[:, 0])
        # Fixed, design-time relation used by the estimator: T -> Cylinder_01
        self.t_t_y: Frame = self.t_m_t.inv() @ self.t_m_y
        self.layout_ok, self.layout_detail = check_constellation_layout(
            v.apriltag, self.cylinder_radius, geo.face.half_extent
        )
        scale = prim_scale(stage, mep_path)
        if np.ptp(scale) > 1e-6:
            raise ValueError(f"MEP scale must be uniform for the tag layer, got {scale}")
        texture_dir = Path(self.cfg.vision_config_path).resolve().parents[1].joinpath(v.logging.dir, "tag_textures")
        self.tag_prim_paths = spawn_tag_constellation(stage, mep_path, float(scale[0]), self.t_m_t, v.apriltag, texture_dir)

        ## The capture cylinder is shown as a finely tessellated, smooth-shaded mesh: the
        ## implicit UsdGeom.Cylinder renders with a coarse, lumpy outline when translucent.
        ## The implicit shape stays (hidden) as the collider and the contact-face
        ## reference. The glass material is re-used, with no refraction (ior), so the
        ## tags the camera sees through it are not displaced.
        from pxr import Sdf, UsdShade

        cyl_path = f"{geo.link_path}/capture_cylinder"
        material = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(cyl_path)):
            if prim.IsA(UsdShade.Shader):
                UsdShade.Shader(prim).CreateInput("ior", Sdf.ValueTypeNames.Float).Set(float(v.capture.cylinder_ior))
            if prim.IsA(UsdShade.Material) and material is None:
                material = UsdShade.Material(prim)
            if prim.IsA(UsdGeom.Cylinder):
                UsdGeom.Imageable(prim).MakeInvisible()
        smooth = spawn_smooth_cylinder(stage, f"{cyl_path}/visual", self.cfg.capture.radius, self.cfg.capture.length,
                                       segments=128, parent_scale=float(prim_scale(stage, cyl_path).mean()))
        if material is not None:
            UsdShade.MaterialBindingAPI.Apply(smooth.GetPrim()).Bind(material)
        self.ee_cylinder_radius = geo.ee_contact_radius

        ## Drift: start upstream of the nominal pose, pass through it at
        ## `rendezvous_time_s`. translation_only: zero angular velocity (Phase 1).
        ## six_dof: constant world-frame w, R(t) = Exp([w] t) R(0), so the start
        ## orientation is Q R_nominal with Q = Exp(-[w] t_r), rotated about Cylinder_01:
        ## at t = 0 the wrist camera (at the observation pose planned from the nominal
        ## pose) sees the tags only drifted and turned in place, as in Phase 1. PhysX
        ## rotates the body about its centre of mass (unknown before the simulation
        ## starts), so Cylinder_01 passes the nominal pose at t_r only approximately:
        ## miss ~ |w| t_r |COM -> Cylinder_01| (NEEDS_ISAAC_VALIDATION; the COM offset is
        ## logged at start). With w = 0, Q = I and this is exactly the Phase 1 pose.
        _, mep_nominal, _ = geo.placement()
        self.mep_nominal = mep_nominal
        drift = v.mep.linear_velocity_w()
        omega = v.mep.angular_velocity_w()
        mep_start = rendezvous_start_pose(mep_nominal, self.t_m_y, drift, omega, v.mep.rendezvous_time_s)
        set_prim_pose(stage, mep_path, mep_start)
        for cfg in (self.cfg.scene.debris, self.scene["debris"].cfg):
            cfg.init_state.pos = tuple(mep_start.pos.tolist())
            cfg.init_state.rot = mep_start.quat
            cfg.init_state.lin_vel = tuple(drift.tolist())
            cfg.init_state.ang_vel = tuple(omega.tolist())


def spawn_smooth_cylinder(stage, path: str, radius: float, length: float, segments: int = 128, parent_scale: float = 1.0):
    """Closed cylinder mesh along Z centred at the parent origin: smooth side normals,
    flat caps. Visual only (no collider, no mass)."""
    import math

    from pxr import Gf

    r, h, n = radius / parent_scale, 0.5 * length / parent_scale, int(segments)
    ang = [2.0 * math.pi * i / n for i in range(n)]
    pts = [Gf.Vec3f(r * math.cos(a), r * math.sin(a), z) for z in (-h, h) for a in ang]
    pts += [Gf.Vec3f(0.0, 0.0, -h), Gf.Vec3f(0.0, 0.0, h)]
    cb, ct = 2 * n, 2 * n + 1
    counts, idx, normals = [], [], []
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        idx += [i, j, n + j, n + i]
        normals += [Gf.Vec3f(math.cos(ang[k]), math.sin(ang[k]), 0.0) for k in (i, j, j, i)]
        counts += [3, 3]
        idx += [ct, n + i, n + j, cb, j, i]
        normals += [Gf.Vec3f(0.0, 0.0, 1.0)] * 3 + [Gf.Vec3f(0.0, 0.0, -1.0)] * 3
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(-r, -r, -h), Gf.Vec3f(r, r, h)])
    return mesh


def _cylinder_size(stage: Usd.Stage, path: str) -> Tuple[float, float]:
    """World radius and height of a UsdGeom.Cylinder prim (axis-aligned scale)."""
    from pxr import Gf

    cyl = UsdGeom.Cylinder(stage.GetPrimAtPath(path))
    scale = Gf.Transform(UsdGeom.XformCache().GetLocalToWorldTransform(cyl.GetPrim())).GetScale()
    axis = {"X": 0, "Y": 1, "Z": 2}[cyl.GetAxisAttr().Get()]
    radial = [scale[i] for i in range(3) if i != axis]
    return float(cyl.GetRadiusAttr().Get() * max(radial)), float(cyl.GetHeightAttr().Get() * scale[axis])
