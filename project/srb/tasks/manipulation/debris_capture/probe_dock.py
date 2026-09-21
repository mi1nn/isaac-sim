"""Probe-tip docking task: GPS telemetry + a depth camera on the MEP probe (Phase 2).

Adds to `DockingTask` (whose measured `PROBE_DOCK_POINT` / `SAT_DOCK_POINT` frames,
thruster collider rebuild and magnet-style `CaptureManager` are reused unchanged):

- the two unused cameras baked into the MEP asset are removed from the stage
- `cam_probe`: a depth camera at the probe tip, looking along the insertion direction
- a model of the thruster bore, so a depth reading becomes an insertion depth

The controller does not read the satellite's true pose. It gets a noisy, delayed
`PoseStamped` of the thruster over ROS 2 (the "GPS" link, `gps_link.py`) for the
*direction*, and the depth camera for how far it still has to go along that direction.

Camera placement is a two-step dance, because a sensor has to be in the scene config
before the scene is built, while `PROBE_DOCK_POINT` can only be measured from the USD
once it is: the camera is spawned at the configured nominal offset, then `_setup_scene`
rewrites its local transform to the measured tip pose and checks the result.
"""

import math
from typing import Optional, Tuple

import numpy as np
from pxr import Gf, Usd, UsdGeom

from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass

from .docking import (
    DockingTask,
    DockingTaskCfg,
    mesh_points_and_triangles,
    prim_frame,
    prim_scale,
    set_prim_pose,
)
from .frames import Frame, quat_wxyz_to_rotmat

# The camera looks along the probe's insertion direction (+Z of PROBE_DOCK_POINT).
# `convention="ros"` optical frame: +X right, +Y down, +Z forward, so the mount is a
# half turn about X relative to the tip frame -- the same relation `cam_wrist` uses.
CAM_PROBE_QUAT_ROS = (0.0, 1.0, 0.0, 0.0)


@configclass
class ProbeCameraCfg:
    """Depth camera at the probe tip."""

    name: str = "cam_probe"
    width: int = 320
    height: int = 240
    horizontal_fov_deg: float = 60.0
    horizontal_aperture_mm: float = 20.955
    # The bore is ~2.4 m deep and the tip starts ~1 m outside it, so 6 m is plenty. The
    # near plane must be small: at full insertion the throat is barely a metre away.
    clipping_range_m: Tuple[float, float] = (0.02, 6.0)
    # Offset along the tip axis [m], positive = in front of the tip face. It must stay
    # positive: the probe is a solid rod, so a camera recessed behind its end face sits
    # inside the mesh and every pixel comes back at the near plane. The measured range
    # is corrected for this offset, so it does not bias the insertion depth.
    mount_offset_m: float = 0.01
    # Nominal tip pose in the MEP body frame, used only to spawn the prim before the
    # real one is measured. `_setup_scene` overwrites it and reports the correction.
    nominal_tip_pos_m: Tuple[float, float, float] = (-1.8104, -2.4356, 0.1065)
    # Radius [px fraction] of the ring of pixels used for the off-axis depth check
    ring_fraction: float = 0.6
    # How often the sensor refreshes. The insertion creeps at centimetres per second,
    # so 20 Hz is ample and costs a seventh of rendering it every physics step.
    update_rate_hz: float = 20.0
    # RGB alongside the depth channel. The depth maths is unchanged -- this is the human
    # view: the GUI switches to it for the insertion, where the probe tip is the only
    # place anything interesting is happening.
    rgb: bool = True
    # The bore is a hole pointing away from the sun, so an RGB frame taken inside it is
    # black. A docking camera carries its own illuminator; so does this one. Depth is
    # geometric and does not care either way.
    light_intensity: float = 60000.0
    light_radius_m: float = 0.03
    # Along the tip axis, relative to the camera (positive = further forward)
    light_offset_m: float = 0.02

    @property
    def data_types(self):
        return (["rgb"] if self.rgb else []) + ["distance_to_image_plane"]

    @property
    def focal_length_mm(self) -> float:
        return self.horizontal_aperture_mm / (2.0 * math.tan(math.radians(self.horizontal_fov_deg) / 2.0))


@configclass
class WristCameraCfg:
    """RGB camera on the Canadarm3 last link -- the view of the capture, as in Phase 1."""

    name: str = "cam_wrist"
    width: int = 640
    height: int = 360
    horizontal_fov_deg: float = 90.0
    horizontal_aperture_mm: float = 20.955
    clipping_range_m: Tuple[float, float] = (0.01, 100.0)
    # On the EE axis just in front of the flange, the same mount Phase 1 measured
    mount_link_z_m: float = -0.45

    @property
    def focal_length_mm(self) -> float:
        return self.horizontal_aperture_mm / (2.0 * math.tan(math.radians(self.horizontal_fov_deg) / 2.0))


@configclass
class ProbeDockTaskCfg(DockingTaskCfg):
    probe_camera: ProbeCameraCfg = ProbeCameraCfg()
    wrist_camera: WristCameraCfg = WristCameraCfg()
    # Cameras baked into the MEP asset. They are never read, they clutter the Isaac Sim
    # camera menu, and each is another render product the RTX renderer keeps alive.
    # Relative to the MEP prim; the `Camera` Xform holding each one is removed.
    remove_asset_camera_relpaths: Tuple[str, ...] = (
        "Xform_Ares1/Camera",
        "Fermi_Gamma_ray_Large_Area_Space_Telescope/Camera",
    )
    # The docking demo's translucent marker disc sits across the nozzle exit. A depth
    # camera ranges to it regardless of opacity, which hides the bore from outside.
    hide_dock_indicator: bool = True
    # Capture cylinder radius. `CaptureCfg` defaults to 0.6 m, which is the whole grasp
    # face; Phase 1 sized it to the Canadarm3 flange instead (link radius 0.17-0.19 m
    # there) and scaled the MEP marker `Cylinder_01` to match, so the two overlap
    # exactly when aligned and any offset is visible. Same value here.
    capture_radius_m: float = 0.20
    # Index of refraction of the glass the wrist camera looks through (1.0 = none)
    capture_cylinder_ior: float = 1.0
    # RTX anti-aliasing for the whole app: Off | FXAA | DLSS | TAA | DLAA. The default
    # experience file uses DLSS, which renders low and upscales -- costly here for no
    # benefit, since the depth channel is geometric and the RGB is only for a human.
    antialiasing: str = "Off"

    def _match_vision_capture_gripper(self):
        """Shrink the capture cylinder to the flange, and the MEP marker with it."""
        r = float(self.capture_radius_m)
        self.capture.radius = r
        if self.debris_marker_xform is not None:
            translate, orient, scale = self.debris_marker_xform
            # UsdGeom.Cylinder radius 0.5 (axis Z) under the uniformly scaled MEP prim
            radial = r / (0.5 * float(self.scene.debris.spawn.scale[0]))
            self.debris_marker_xform = (translate, orient, (radial, radial, scale[2]))

    def __post_init__(self):
        super().__post_init__()
        self._match_vision_capture_gripper()
        self.sim.render.antialiasing_mode = self.antialiasing
        self._add_probe_camera()



    def _add_probe_camera(self):
        c = self.probe_camera
        setattr(
            self.scene,
            c.name,
            CameraCfg(
                # Under the MEP rigid body, so it rides with the probe
                prim_path="{ENV_REGEX_NS}/debris/" + c.name,
                offset=CameraCfg.OffsetCfg(
                    pos=tuple(c.nominal_tip_pos_m),
                    rot=CAM_PROBE_QUAT_ROS,
                    convention="ros",
                ),
                # The docking loop consumes depth at `depth_rate_hz`, not every physics
                # step; filling this render product 150 times a second is wasted work.
                update_period=1.0 / max(1e-6, c.update_rate_hz),
                width=int(c.width),
                height=int(c.height),
                # Depth is geometric, so it works inside the unlit bore; RGB is for the
                # operator's view and relies on the probe illuminator spawned below.
                data_types=list(c.data_types),
                spawn=PinholeCameraCfg(
                    focal_length=c.focal_length_mm,
                    horizontal_aperture=c.horizontal_aperture_mm,
                    clipping_range=tuple(c.clipping_range_m),
                ),
            ),
        )


def _ray_triangle_depths(origin, direction, pts: np.ndarray, tris: np.ndarray, eps: float = 1e-9):
    """Moller-Trumbore, vectorised: depths along `direction` where the ray hits `tris`."""
    v0, v1, v2 = pts[tris[:, 0]], pts[tris[:, 1]], pts[tris[:, 2]]
    e1, e2 = v1 - v0, v2 - v0
    pvec = np.cross(direction, e2)
    det = np.einsum("ij,ij->i", e1, pvec)
    ok = np.abs(det) > eps
    if not ok.any():
        return []
    inv_det = np.zeros_like(det)
    inv_det[ok] = 1.0 / det[ok]
    tvec = origin - v0
    u = np.einsum("ij,ij->i", tvec, pvec) * inv_det
    qvec = np.cross(tvec, e1)
    v = np.einsum("j,ij->i", direction, qvec) * inv_det
    depth = np.einsum("ij,ij->i", e2, qvec) * inv_det
    hit = ok & (u >= -eps) & (v >= -eps) & (u + v <= 1.0 + eps) & (depth > 0.0)
    return depth[hit].tolist()


class BoreModel:
    """Design-time model of the client thruster bore, along its axis.

    `depth` is measured from the nozzle exit plane, positive into the nozzle -- the
    convention `DockingGeometry` / `NozzleGeometry` already use. `axial_extent` is how
    far the bore reaches from that plane, so a depth camera on the axis looking in reads

        insertion_depth = axial_extent - measured_axial_range

    In a real mission this comes from the client's design drawings. Here it is measured
    once from the spawned USD, exactly as the AprilTag `T_T_Y` is in Phase 1.
    """

    def __init__(self, nozzle, tip_radius: float, axial_extent: Optional[float] = None):
        self.profile = list(nozzle.profile)
        self.exit_radius = float(nozzle.exit_radius)
        self.tip_radius = float(tip_radius)
        # `NozzleGeometry.profile` samples the bore in 24 depth bins and stores each
        # bin's *centre*, so its deepest entry falls short of the real end of the bore.
        # When the on-axis extent is available (ray-cast against the mesh) it is used
        # instead: both alternatives were measured as a pure constant bias on every
        # reading (31.4 mm for the binned profile, 81.6 mm for the deepest mesh point).
        self.axial_extent = float(axial_extent if axial_extent is not None else max(d for d, _ in self.profile))
        throat = min(self.profile, key=lambda dr: dr[1])
        self.throat_depth, self.throat_radius = float(throat[0]), float(throat[1])

    def inner_radius(self, depth: float) -> float:
        ds = [d for d, _ in self.profile]
        rs = [r for _, r in self.profile]
        return float(np.interp(depth, ds, rs))

    def wall_clearance(self, depth: float, radial_offset: float) -> float:
        """Probe surface to the bore wall at `depth`, for a tip `radial_offset` off axis."""
        return self.inner_radius(depth) - self.tip_radius - radial_offset

    def hit_depth(self, radial: float) -> float:
        """Depth at which a ray parallel to the axis, `radial` off it, meets the wall.

        A centred ray runs all the way to `axial_extent`. An off-axis one is stopped
        early by the converging bore, where the inner radius first drops to `radial`.
        Ignoring this was measured as a 0.55 m depth error at a 100 mm offset -- the
        reading was not noisy, it was answering a different question.
        """
        if radial <= self.throat_radius:
            return self.axial_extent
        for (d0, r0), (d1, r1) in zip(self.profile, self.profile[1:]):
            if (r0 - radial) * (r1 - radial) <= 0.0 and r0 != r1:
                return float(d0 + (d1 - d0) * (r0 - radial) / (r0 - r1))
        return self.axial_extent

    def depth_from_range(self, axial_range: float, radial: float = 0.0) -> float:
        """Insertion depth [m] from a range reading (negative = still outside).

        `radial` is how far the ray is off the bore axis; at 0 this is the on-axis
        calibration validated to 0.0 mm over -1.0 .. 0.8 m.
        """
        return self.hit_depth(radial) - axial_range

    def range_at_depth(self, depth: float) -> float:
        """Inverse of `depth_from_range`, for predicting what the camera should read."""
        return self.axial_extent - depth

    def __repr__(self) -> str:
        return (
            f"BoreModel(exit_radius={self.exit_radius:.4f} m, axial_extent={self.axial_extent:.4f} m, "
            f"throat r={self.throat_radius:.4f} m at depth {self.throat_depth:.4f} m, "
            f"probe tip radius={self.tip_radius:.4f} m)"
        )


class ProbeDockTask(DockingTask):
    cfg: ProbeDockTaskCfg

    def _setup_scene(self):
        super()._setup_scene()
        stage = self.scene.stage
        geo = self.docking_geometry

        self.removed_cameras = self._remove_asset_cameras(stage, geo.mep_path)
        self.bore = BoreModel(geo.nozzle, geo.probe.tip_radius, self._measure_bore_extent(stage, geo))
        print(f"[PROBE] {self.bore}", flush=True)
        self.t_tip_cam = self._place_probe_camera(stage, geo)
        self.probe_camera_path = f"{self.scene.env_prim_paths[0]}/debris/{self.cfg.probe_camera.name}"
        self.wrist_camera_path = self._spawn_wrist_camera(stage, geo)
        self._spawn_probe_light(stage, geo)
        self._hide_dock_indicator(stage)
        self._smooth_capture_cylinder(stage, geo)
        print(
            f"[PROBE] cameras: wrist {self.wrist_camera_path} | probe {self.probe_camera_path}",
            flush=True,
        )

    def _smooth_capture_cylinder(self, stage, geo):
        """Phase 1's treatment of the capture cylinder, so it looks the same here.

        The implicit `UsdGeom.Cylinder` renders with a coarse, lumpy silhouette when
        translucent; it stays as the collider and the contact-face reference but is made
        invisible, and a finely tessellated mesh of the *same* radius is drawn instead.
        Nothing about it changes when the MEP attaches -- the radius is fixed at
        `capture_radius_m` from the start.
        """
        from pxr import Sdf, UsdShade

        from .vision_task import spawn_smooth_cylinder

        cyl_path = f"{geo.link_path}/capture_cylinder"
        root = stage.GetPrimAtPath(cyl_path)
        if not root.IsValid():
            return
        material = None
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdShade.Shader):
                UsdShade.Shader(prim).CreateInput("ior", Sdf.ValueTypeNames.Float).Set(
                    float(self.cfg.capture_cylinder_ior)
                )
            if prim.IsA(UsdShade.Material) and material is None:
                material = UsdShade.Material(prim)
            if prim.IsA(UsdGeom.Cylinder):
                UsdGeom.Imageable(prim).MakeInvisible()
        smooth = spawn_smooth_cylinder(
            stage, f"{cyl_path}/visual", self.cfg.capture.radius, self.cfg.capture.length,
            segments=128, parent_scale=float(prim_scale(stage, cyl_path).mean()),
        )
        if material is not None:
            UsdShade.MaterialBindingAPI.Apply(smooth.GetPrim()).Bind(material)
        print(f"[PROBE] capture cylinder r={self.cfg.capture.radius:.3f} m (matched to Phase 1)", flush=True)

    def _spawn_wrist_camera(self, stage, geo) -> str:
        """A plain USD camera on the last link -- a viewpoint, not a sensor.

        Nothing in this demo *reads* the wrist view; it exists so a person can watch the
        capture. Making it a `CameraCfg` sensor would add a second render product that
        the RTX renderer fills every frame and nobody consumes, which is pure wall-clock
        cost. A bare `UsdGeom.Camera` prim can be picked by the viewport just the same.
        A USD camera looks down its own -Z, which is already the capture direction of
        this link, so it needs no rotation.
        """
        c = self.cfg.wrist_camera
        path = f"{geo.link_path}/{c.name}"
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateFocalLengthAttr(float(c.focal_length_mm))
        cam.CreateHorizontalApertureAttr(float(c.horizontal_aperture_mm))
        cam.CreateVerticalApertureAttr(float(c.horizontal_aperture_mm) * c.height / c.width)
        near, far = c.clipping_range_m
        cam.CreateClippingRangeAttr(Gf.Vec2f(float(near), float(far)))
        UsdGeom.Xformable(cam).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(c.mount_link_z_m)))
        return path

    def _spawn_probe_light(self, stage, geo):
        """Illuminator beside the probe camera, so the RGB view inside the bore is usable."""
        c = self.cfg.probe_camera
        if not c.rgb or c.light_intensity <= 0.0:
            return None
        from srb.core.sim import SphereLightCfg

        scale = float(prim_scale(stage, geo.mep_path).mean())
        t_tip_light = Frame(np.array([0.0, 0.0, c.mount_offset_m + c.light_offset_m]), np.eye(3))
        light_in_body = geo.probe_dock @ t_tip_light
        path = f"{self.scene.env_prim_paths[0]}/debris/{c.name}_light"
        cfg = SphereLightCfg(intensity=c.light_intensity, radius=c.light_radius_m / scale)
        cfg.func(path, cfg, translation=tuple((light_in_body.pos / scale).tolist()))
        print(f"[PROBE] probe illuminator at {path} ({c.light_intensity:.0f} cd)", flush=True)
        return path

    def _measure_bore_extent(self, stage, geo) -> float:
        """Depth of the bore surface the camera actually sees: an axis ray-cast.

        Two things this is not. It is not the deepest *mesh point*: that sits on the rim
        of the inner tube, off axis, and using it biased every reading by 81.6 mm. It is
        also not the farthest hit along the axis: the bore ends in a thin cap, and taking
        its back face left a 32.5 mm bias. A camera sees the *nearest* surface in front
        of it, so the calibration is the nearest axis hit past the throat -- everything
        shallower than the throat is the open bore the probe flies through.
        """
        sat = prim_frame(stage, geo.sat_body_path)
        path = f"{geo.sat_prim_path}/{self.cfg.docking.thruster_rel}"
        origin, direction = geo.nozzle.exit_centre, geo.nozzle.direction
        throat_depth = float(min(geo.nozzle.profile, key=lambda dr: dr[1])[0])
        hits = []
        for _, pts, tris in mesh_points_and_triangles(stage, path, sat):
            if tris.size == 0:
                continue
            hits.extend(_ray_triangle_depths(origin, direction, pts, tris))
        beyond = [h for h in hits if h > throat_depth]
        if not beyond:
            raise ValueError(
                f"The bore axis does not hit the thruster mesh at {path} past the throat "
                f"(depth {throat_depth:.3f} m); the depth camera cannot be calibrated. "
                f"{len(hits)} shallower hits."
            )
        return float(min(beyond))

    def _hide_dock_indicator(self, stage):
        """Take the docking-demo's marker disc out of the probe camera's view.

        `spawn_dock_indicator` puts a translucent disc across the nozzle exit for the
        human watching the GUI. A depth camera does not care that it is translucent: it
        ranges to the disc, so from outside the nozzle every pixel comes back at the
        distance to it and the bore is invisible. This demo reads the bore, and it gets
        its own status from the console, so the disc is hidden.
        """
        if not self.cfg.hide_dock_indicator:
            return
        path = getattr(self, "dock_indicator_path", None)
        prim = stage.GetPrimAtPath(path) if path else None
        if prim is None or not prim.IsValid():
            return
        UsdGeom.Imageable(prim).MakeInvisible()
        print(f"[PROBE] hid the docking indicator disc ({path}): it would block the probe camera", flush=True)

    def _remove_asset_cameras(self, stage, mep_path: str):
        """Deactivate the cameras baked into the MEP asset.

        `Stage.RemovePrim` silently does nothing to them: they come in through the
        asset's reference, so there is no prim spec on the edit target to erase.
        Deactivating writes an override that composes out the prim and its subtree,
        which is what actually takes them off the stage (and out of the camera menu).
        Prims reached through an instance proxy cannot carry that override, so the
        enclosing instance is made non-instanceable first.
        """
        removed = []
        for relpath in self.cfg.remove_asset_camera_relpaths:
            path = f"{mep_path}/{relpath}"
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise ValueError(
                    f"Asset camera to remove not found: {path}. List the stage's "
                    f"UsdGeom.Camera prims and update `remove_asset_camera_relpaths`."
                )
            if prim.IsInstanceProxy():
                inst = prim
                while inst.IsValid() and not inst.IsInstance():
                    inst = inst.GetParent()
                if inst.IsValid():
                    inst.SetInstanceable(False)
                    prim = stage.GetPrimAtPath(path)
            if not prim.SetActive(False):
                raise ValueError(f"Could not deactivate the asset camera {path}")
            removed.append(path)
        left = [
            p.GetPath().pathString
            for p in Usd.PrimRange(stage.GetPrimAtPath(mep_path))
            if p.IsA(UsdGeom.Camera) and self.cfg.probe_camera.name not in p.GetPath().pathString
        ]
        if left:
            raise ValueError(f"MEP still carries active camera prims after the removal: {left}")
        print(f"[PROBE] removed {len(removed)} unused MEP cameras: {removed}", flush=True)
        return removed

    def _place_probe_camera(self, stage, geo) -> Frame:
        """Move `cam_probe` onto the measured probe tip. Returns tip -> camera."""
        c = self.cfg.probe_camera
        scale = prim_scale(stage, geo.mep_path)
        if float(np.ptp(scale)) > 1e-6:
            raise ValueError(f"MEP scale must be uniform to mount the probe camera, got {scale}")
        # `geo.probe_dock` is the tip in the MEP *body* frame (unscaled). The camera prim
        # is a child of the MEP prim, which carries a USD scale, so the local translation
        # is divided by it -- the correction `DockingManager.dock()` makes for its joints.
        t_tip_cam = Frame(np.array([0.0, 0.0, c.mount_offset_m]), np.eye(3))
        cam_in_body = geo.probe_dock @ t_tip_cam
        local = cam_in_body.pos / scale

        path = f"{self.scene.env_prim_paths[0]}/debris/{c.name}"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise ValueError(f"Probe camera prim not found at {path} (was it added to the scene config?)")
        nominal = np.asarray(c.nominal_tip_pos_m, dtype=float)
        # The prim already carries translate/orient ops from the camera spawn. Rewrite
        # them in place via the shared helper: it matches the op's existing precision
        # (this one is quatd, and AddOrientOp() would ask for quatf and raise).
        # The stored rotation is the tip orientation composed with the ROS optical
        # mount, i.e. the camera looks along the probe's insertion direction.
        cam_rot = geo.probe_dock.rot @ quat_wxyz_to_rotmat(CAM_PROBE_QUAT_ROS)
        set_prim_pose(stage, path, Frame(local, cam_rot))
        print(
            f"[PROBE] {c.name}: {c.width}x{c.height}, hFOV {c.horizontal_fov_deg} deg, clip {c.clipping_range_m}, "
            f"{c.mount_offset_m*1000:+.0f} mm along the tip axis",
            flush=True,
        )
        print(
            f"[PROBE] {c.name} moved onto the measured probe tip: local {np.round(local, 4).tolist()} "
            f"(nominal {np.round(nominal / scale, 4).tolist()}, correction "
            f"{np.linalg.norm(local - nominal / scale) * 1000:.2f} mm)",
            flush=True,
        )
        return t_tip_cam
