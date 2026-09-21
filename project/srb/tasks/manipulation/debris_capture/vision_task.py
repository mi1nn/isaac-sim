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

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
from pxr import Gf, Usd, UsdGeom

from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass

from .docking import DockingTask, DockingTaskCfg, prim_frame, prim_scale, set_prim_pose
from .frames import Frame, frame_from_axes
from .task import EventCfg
from .orbit_return import (
    OrbitReference,
    OrbitReferenceCfg,
    client_start_point,
    derive_center_candidates,
    initial_offset_check,
    sensor_keepout_check,
    tube_mesh,
)
from .vision import (
    CAM_MOUNT_QUAT_ROS,
    DEFAULT_CONFIG_PATH,
    VisionCaptureConfig,
    check_constellation_layout,
    load_vision_config,
    rendezvous_start_pose,
    spawn_tag_constellation,
)


# MEP translation from the grasp pose to the docked pose (`DockingPlacementCfg.dock_offset`),
# i.e. the satellite is placed this far from the MEP. The GT docking demo keeps its
# (0.0, 5.5, 0.0); the vision demo uses a shorter sideways carry (-0.7 m, -12.7 %) to cut the
# transport time. Same +Y direction, so the docking axis and approach direction are unchanged.
# NEEDS_ISAAC_VALIDATION: reachability of the shifted pre-dock / docked poses is not re-checked.
VISION_DOCK_OFFSET_M: Tuple[float, float, float] = (0.0, 3.2, 0.0)


@configclass
class VisionEventCfg(EventCfg):
    # Writes the satellite's init_state (pose + drift velocity) at reset. Only the MEP has such a reset
    # event in `EventCfg`, so a satellite `init_state.lin_vel` was never applied (measured: v = 0).
    # Set only when the satellite drifts, so runs without drift keep their previous reset path.
    randomize_sat_state: EventTermCfg | None = None


@configclass
class VisionCaptureTaskCfg(DockingTaskCfg):
    events: VisionEventCfg = VisionEventCfg()
    vision_config_path: str = DEFAULT_CONFIG_PATH.as_posix()
    # `section.key=value` overrides of the YAML (see `load_vision_config`)
    vision_overrides: Tuple[str, ...] = ()

    def __post_init__(self):
        super().__post_init__()
        self.docking.placement.dock_offset = VISION_DOCK_OFFSET_M
        self.apply_vision_config()

    def load_vision(self) -> VisionCaptureConfig:
        return load_vision_config(self.vision_config_path, self.vision_overrides)

    def apply_vision_config(self):
        """(Re)apply the YAML values that live in the env config (call after overrides)."""
        v = self.load_vision()
        self.scene.debris.spawn.mass_props.mass = float(v.mep.mass_kg)
        # Free flight: no velocity damping. Left unset, PhysX applies its default angular
        # damping (0.05 1/s), which slowed the 6-DoF spin by ~5 % per second (measured
        # 0.0084 -> 0.0025 rad/s over 25 s), so the MEP was not constant-twist.
        self.scene.debris.spawn.rigid_props.linear_damping = 0.0
        self.scene.debris.spawn.rigid_props.angular_damping = 0.0
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
        # Docking phase: the satellite free-floats too (translation only in this phase)
        sat_v = np.asarray(v.docking.satellite_drift_direction, dtype=float) * float(v.docking.satellite_velocity_mps)
        self.scene.satellite.spawn.rigid_props.linear_damping = 0.0
        self.scene.satellite.spawn.rigid_props.angular_damping = 0.0
        self.scene.satellite.init_state.lin_vel = tuple(sat_v.tolist())
        self.scene.satellite.init_state.ang_vel = (0.0, 0.0, 0.0)
        zero = {k: (0.0, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")}
        self.events.randomize_sat_state = (
            EventTermCfg(func=reset_root_state_uniform, mode="reset",
                         params={"asset_cfg": SceneEntityCfg("satellite"), "pose_range": dict(zero), "velocity_range": dict(zero)})
            if float(np.linalg.norm(sat_v)) > 0.0 else None
        )
        # RGB-D camera on the Ares1 probe. The prim is a child of the MEP body so it
        # follows it; its pose on the probe axis is measured and written in
        # `_setup_scene` (the probe tip is only known once the USD is on the stage).
        pc = v.probe_camera
        if pc.enabled:
            setattr(
                self.scene,
                pc.name,
                CameraCfg(
                    prim_path=f"{self.scene.debris.prim_path}/{pc.name}",
                    offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0), convention="ros"),
                    update_period=0.0,
                    width=int(pc.width),
                    height=int(pc.height),
                    data_types=["rgb", "distance_to_image_plane"],
                    spawn=PinholeCameraCfg(
                        focal_length=pc.focal_length_mm,
                        horizontal_aperture=pc.horizontal_aperture_mm,
                        clipping_range=tuple(pc.clipping_range_m),
                    ),
                ),
            )
        # `--orbit-only`: a camera at the GUI start view of the orbit, to save what the operator would see
        # (red arc against the Earth of the sky dome). Its pose is written in `_setup_orbit`.
        if v.orbit_reference.enabled and v.orbit_reference.observe_only:
            setattr(
                self.scene,
                ORBIT_VIEW_CAMERA,
                CameraCfg(
                    prim_path=f"{{ENV_REGEX_NS}}/{ORBIT_VIEW_CAMERA}",
                    offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0), convention="world"),
                    update_period=0.0,
                    width=1280,
                    height=720,
                    data_types=["rgb"],
                    spawn=PinholeCameraCfg(focal_length=14.96, horizontal_aperture=20.955, clipping_range=(0.1, 5000.0)),  # 70 deg
                ),
            )
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


ORBIT_VIEW_CAMERA = "cam_orbit_view"


@dataclass
class OrbitSetup:
    """The Client reference orbit as built for this run (world frame)."""

    cfg: OrbitReferenceCfg
    orbit: OrbitReference
    theta_nominal: float
    nominal_point: np.ndarray  # orbit point the Client nominally sat on
    client_ref_in_sat: Frame   # the rigid Client point that has to reach the orbit (satellite body frame)
    client_start: np.ndarray   # its start position (world)
    client_start_before_shift: np.ndarray
    placement_shift: np.ndarray  # satellite move applied to reach `client_start` (zero for a derived centre)
    center_derived: bool
    keepout_m: Dict[str, float]
    keepout_ok: bool
    prim_path: str

    def view(self) -> Tuple[np.ndarray, np.ndarray]:
        """(eye, target) for the GUI viewport. The eye sits on the ring axis, above the ring plane by
        radius * tan(view_elevation), looking at the nominal orbit point: the ring is then a circle of
        constant elevation around the viewer, just like the Earth limb of the sky dome (concentric),
        `view_elevation_deg` below the horizon (the limb is EARTH_LIMB_DIP_DEG below it)."""
        o = self.orbit
        eye = o.center + o.normal * (o.a * float(np.tan(np.radians(self.cfg.view_elevation_deg))))
        return eye, self.nominal_point.copy()


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
        ## RGB-D camera on the probe: on the probe axis, `offset_from_tip_m` in front of
        ## the tip, looking along the insertion direction (+Z), so the rod never occludes
        ## the principal ray used for the depth reading. `probe_dock` is the measured
        ## PROBE_DOCK_POINT (tip + axis) in the MEP body frame.
        self.probe_cam_in_mep: Optional[Frame] = None
        pc = v.probe_camera
        if pc.enabled:
            axis = geo.probe_dock.rot[:, 2]  # insertion direction (out of the tip)
            # A USD camera looks along its own -Z, so the prim's +Z is the *opposite* of
            # the viewing direction. Writing the prim transform directly (below) bypasses
            # the `convention="ros"` conversion CameraCfg would have done, so the USD
            # convention is applied here instead -- with +Z = axis the camera looked
            # backwards along the probe and every depth reading came back empty.
            cam = frame_from_axes(
                geo.probe_dock.pos + pc.offset_from_tip_m * axis, -axis, geo.probe_dock.rot[:, 0]
            )
            self.probe_cam_in_mep = cam
            # The translucent dock indicator is a full disc across the nozzle exit, and a
            # depth image records the first hit whatever its opacity, so it would be the
            # only thing this camera ever ranges to (measured: raw 0.93 m at the pre-dock
            # pose, i.e. the exit plane, instead of the nozzle interior). The docking
            # phase shows the same state through its debug draw instead.
            indicator = stage.GetPrimAtPath(self.dock_indicator_path)
            if indicator.IsValid():
                UsdGeom.Imageable(indicator).MakeInvisible()
                print(f"[INIT] dock indicator disc hidden: it would block every {pc.name} depth ray", flush=True)
            # ...and replaced by a ring on the nozzle rim: it marks the docking interface
            # (colour = docking state, set by the demo) without covering the axis
            self.dock_ring_path = None
            try:
                self.dock_ring_path = spawn_dock_ring(stage, geo)
                print(f"[INIT] dock ring (rim marker, coloured by the docking state): {self.dock_ring_path}", flush=True)
            except Exception as e:  # a marker must never stop the run
                print(f"[INIT] dock ring not created ({e}); the nozzle has no marker", flush=True)
            # Local (parent-scaled) transform under the MEP prim
            set_local_pose(stage, f"{mep_path}/{pc.name}", cam, float(scale[0]))
            print(f"[INIT] {pc.name}: probe-axis camera at {np.round(cam.pos, 4).tolist()} (MEP body frame), viewing direction "
                  f"{np.round(axis, 4).tolist()} (USD prim +Z = {np.round(cam.rot[:, 2], 4).tolist()}), "
                  f"{pc.offset_from_tip_m*1000:.0f} mm in front of the tip, {pc.width}x{pc.height}, "
                  f"FOV {pc.horizontal_fov_deg:g} deg, rgb + distance_to_image_plane", flush=True)

        _, mep_nominal, _ = geo.placement()
        self.mep_nominal = mep_nominal
        drift = v.mep.linear_velocity_w()
        omega = v.mep.angular_velocity_w()
        ## rotation_start "diverge": no back-rotation -> the MEP starts in the nominal
        ## orientation (position still passes the nominal at t_r) and turns away from it
        omega_back = omega if v.mep.rotation_start == "converge" else np.zeros(3)
        mep_start = rendezvous_start_pose(mep_nominal, self.t_m_y, drift, omega_back, v.mep.rendezvous_time_s)
        set_prim_pose(stage, mep_path, mep_start)
        for cfg in (self.cfg.scene.debris, self.scene["debris"].cfg):
            cfg.init_state.pos = tuple(mep_start.pos.tolist())
            cfg.init_state.rot = mep_start.quat
            cfg.init_state.lin_vel = tuple(drift.tolist())
            cfg.init_state.ang_vel = tuple(omega.tolist())
        ## The satellite's drift also goes to the scene asset's own init state (as for the MEP above);
        ## the reset event `randomize_sat_state` then writes it to the simulation
        sat_v = np.asarray(v.docking.satellite_drift_direction, dtype=float) * float(v.docking.satellite_velocity_mps)
        for cfg in (self.cfg.scene.satellite, self.scene["satellite"].cfg):
            cfg.init_state.lin_vel = tuple(sat_v.tolist())
            cfg.init_state.ang_vel = (0.0, 0.0, 0.0)

        ## Client reference orbit (red line) and the Client's start off it
        self.orbit: Optional[OrbitSetup] = None
        if v.orbit_reference.enabled:
            self.orbit = self._setup_orbit(stage, geo, v.orbit_reference, mep_nominal, v)

    def _setup_orbit(self, stage, geo, o: OrbitReferenceCfg, mep_nominal: Frame, v: VisionCaptureConfig) -> OrbitSetup:
        """Build the reference orbit, place the Client `client_initial_offset_m` off it and draw it.

        The Client keeps its existing (validated) docking placement whenever `center_w` is null: the
        orbit centre is derived from that placement. With a fixed centre the satellite is moved
        (before the simulation starts, like every other placement) to nominal point + offset.
        The orbit is a visual prim only, and must stay clear of the docking axis (`cam_probe` depth
        ray) and of the wrist-camera-to-tags corridor.
        """
        env = self.scene.env_prim_paths[0]
        ref_in_sat = geo.sat_dock if o.client_reference == "sat_dock_point" else Frame(np.zeros(3), np.eye(3))
        sat_body = prim_frame(stage, geo.sat_body_path)
        s0 = (sat_body @ ref_in_sat).pos
        dock_w = sat_body @ geo.sat_dock
        axis = dock_w.rot[:, 2]
        base = prim_frame(stage, f"{env}/robot").pos
        grasp = mep_nominal @ geo.mep_grasp  # +Z = outward normal of the attachment face (towards the arm)

        if o.center_w is None:
            candidates = derive_center_candidates(o, s0, axis, base)
        else:
            full = OrbitReference.from_cfg(o, o.center_w)
            full.arc_span = 2.0 * np.pi
            theta = float(np.radians(o.nominal_angle_deg)) if o.nominal_angle_deg is not None else full.closest(s0)[1]
            candidates = [(np.asarray(o.center_w, dtype=float), theta)]

        tried = []
        chosen = None
        for centre, theta in candidates:
            orbit = OrbitReference.from_cfg(o, centre, theta)
            start = client_start_point(o, orbit, theta)
            shift = start - s0
            dk = dock_w.pos + shift
            segments = {
                # depth ray of cam_probe: pre-dock approach up to the thruster behind the dock point
                "docking_axis": (dk - axis * (float(self.cfg.docking.dock_depth) + v.docking.pre_dock_distance_m + 2.0), dk + axis * 3.0),
                # cam_wrist: from the attachment face out along its normal past the observation pose
                "wrist_camera_corridor": (grasp.pos, grasp.pos + grasp.rot[:, 2] * (v.approach.observe_distance_m + 1.0)),
            }
            ok, dist = sensor_keepout_check(orbit, segments, o.sensor_keepout_m)
            tried.append(dist)
            if ok:
                chosen = (orbit, theta, start, shift, dist)
                break
        if chosen is None:
            raise ValueError(
                f"The reference orbit passes closer than orbit_reference.sensor_keepout_m = {o.sensor_keepout_m} m to a sensor ray "
                f"(distances per candidate {tried}). A line in front of cam_probe / cam_wrist would corrupt the depth reading / AprilTag pose: "
                f"change orbit_reference.center_w / normal_w / nominal_angle_deg / client_initial_offset_m."
            )
        orbit, theta, start, shift, dist = chosen

        moved = float(np.linalg.norm(shift)) > 1e-9
        if moved:
            sat_prim = prim_frame(stage, geo.sat_prim_path)
            new = Frame(sat_prim.pos + shift, sat_prim.rot)
            set_prim_pose(stage, geo.sat_prim_path, new)
            for cfg in (self.cfg.scene.satellite, self.scene["satellite"].cfg):
                cfg.init_state.pos = tuple(new.pos.tolist())
        path = spawn_reference_orbit(stage, o, orbit)
        setup = OrbitSetup(
            cfg=o, orbit=orbit, theta_nominal=float(theta), nominal_point=orbit.point_at(theta), client_ref_in_sat=ref_in_sat,
            client_start=start, client_start_before_shift=s0, placement_shift=shift, center_derived=o.center_w is None,
            keepout_m=dist, keepout_ok=True, prim_path=path,
        )
        if o.observe_only:
            # USD camera looks along its own -Z with +Y up
            eye, target = setup.view()
            fwd = (target - eye) / np.linalg.norm(target - eye)
            set_prim_pose(stage, f"{env}/{ORBIT_VIEW_CAMERA}", frame_from_axes(eye, -fwd, np.cross(fwd, [0.0, 0.0, 1.0])))
        return setup


def spawn_reference_orbit(stage, cfg: OrbitReferenceCfg, orbit: OrbitReference) -> str:
    """Red tube along the orbit arc (a closed ring when the span is 360 deg) at `cfg.prim_path`: geometry only.

    Visual only: no rigid body, no collider, no physics schema, so nothing collides with it and
    no contact sensor sees it. A tube mesh (`cfg.line_width_m` across, 8 sides) with an emissive
    material is used instead of `UsdGeom.BasisCurves`, whose width RTX does not draw reliably.
    """
    from pxr import Gf, Sdf, UsdShade, Vt

    if stage.GetPrimAtPath(cfg.prim_path).IsValid():
        stage.RemovePrim(cfg.prim_path)
    UsdGeom.Xform.Define(stage, cfg.prim_path)
    verts, normals, counts, idx = tube_mesh(orbit.points(cfg.segments), orbit.normal, 0.5 * cfg.line_width_m, sides=8, closed=orbit.is_full)
    mesh = UsdGeom.Mesh.Define(stage, f"{cfg.prim_path}/line")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in verts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in normals]))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    mesh.CreateExtentAttr([Gf.Vec3f(*(float(x) for x in lo)), Gf.Vec3f(*(float(x) for x in hi))])
    rgb = Gf.Vec3f(*(float(x) for x in cfg.color_rgb))
    mesh.CreateDisplayColorAttr([rgb])
    mat = UsdShade.Material.Define(stage, f"{cfg.prim_path}/Material")
    sh = UsdShade.Shader.Define(stage, f"{cfg.prim_path}/Material/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(rgb)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return cfg.prim_path


def spawn_dock_ring(stage, geo, inner_factor: float = 0.75, segments: int = 96) -> str:
    """Flat ring on the nozzle exit rim (visual only, no collider, open in the middle).

    A full disc across the exit blocks the depth ray on the docking axis, so the marker
    is an annulus between `inner_factor` x the exit radius and the exit radius. Its
    colour is the `diffuseColor` / `emissiveColor` of the preview surface below it.
    """
    from pxr import Gf, Sdf, UsdShade, Vt

    scale = float(np.mean(geo.sat_scale))
    frame = geo.sat_exit  # +Z = into the nozzle, in the satellite body frame
    path = f"{geo.sat_body_path}/dock_ring"
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*(frame.pos / scale).tolist()))
    xf.AddOrientOp().Set(Gf.Quatf(*frame.quat))
    r_out = geo.nozzle.exit_radius / scale
    r_in = r_out * inner_factor
    ang = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    pts = [Gf.Vec3f(float(r_in * np.cos(a)), float(r_in * np.sin(a)), 0.0) for a in ang]
    pts += [Gf.Vec3f(float(r_out * np.cos(a)), float(r_out * np.sin(a)), 0.0) for a in ang]
    idx = []
    for i in range(segments):
        j = (i + 1) % segments
        idx += [i, j, segments + j, segments + i]
    mesh = UsdGeom.Mesh.Define(stage, f"{path}/ring")
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * segments))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateDoubleSidedAttr(True)
    mat = UsdShade.Material.Define(stage, f"{path}/Material")
    sh = UsdShade.Shader.Define(stage, f"{path}/Material/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.2, 0.2))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.2, 0.2))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return path


def set_local_pose(stage, path: str, frame: Frame, parent_scale: float = 1.0):
    """Write `frame` as the prim's local transform under a uniformly scaled parent.

    PhysX/USD apply the parent's scale to a child's translation, so a pose measured in
    the parent's *metric* body frame is divided by that scale here (the same correction
    the tag constellation and the capture marker use).
    """
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise ValueError(f"Prim not found (camera not spawned?): {path}")
    xf = UsdGeom.Xformable(prim)
    # The spawner already authored xform ops on this prim, and `AddXformOp` refuses to
    # re-add an existing one, so the old ops are removed before the measured pose is
    # written (the op *order* alone can be cleared, the attributes stay).
    xf.ClearXformOpOrder()
    for name in ("xformOp:translate", "xformOp:orient", "xformOp:rotateXYZ", "xformOp:rotateXYZW", "xformOp:scale", "xformOp:transform"):
        if prim.HasAttribute(name):
            prim.RemoveProperty(name)
    xf.AddTranslateOp().Set(Gf.Vec3d(*(frame.pos / parent_scale).tolist()))
    xf.AddOrientOp().Set(Gf.Quatf(*frame.quat))


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
