"""MRV rendezvous phase in front of the verified vision capture (`vision_capture_demo`).

The capture / transport / docking pipeline is not touched by anything in this module.
What is added ahead of it:

    folded arm -> MRV translation step 1 -> step 2 -> arm deploy -> (existing SEARCH ...)

The "MRV" in this scene is the Canadarm3 base together with the spacecraft hull it is
mounted on (`{ENV}/robot`, a fixed-base articulation, and `{ENV}/scenery`, a static
`XFormPrim`); neither is a free-flying rigid body, so the transit is a commanded rigid
translation of both, not a thruster-driven trajectory. The MRV starts displaced by
`start_offset_m` from its nominal pose and the two legs cancel that offset exactly, so
from ARM_DEPLOY onwards every frame of the scene is the one the existing pipeline was
verified in. The MEP keeps its `six_dof` motion throughout: nothing here touches it (it
only has to start further upstream, see `MrvApproachCfg.planned_duration_s`).

`ThrusterVfx` is a purely visual plume layer on separate prims. It never applies a
force, never carries a collider and never gates the pipeline: a failure to create or
update it is logged and the run continues.

Frames follow the rest of the package: world-frame vectors, quaternions (w, x, y, z).
"""

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

import numpy as np

from .frames import Frame

if TYPE_CHECKING:
    import torch

    from .vision_task import VisionCaptureTask


##############
### Config ###
##############


@dataclass
class MrvApproachCfg:
    """`mrv:` section of `project/config/vision_capture.yaml`."""

    # false: the demo starts exactly as before (arm at the observation pose, MRV at its
    # nominal pose) -- the configuration the capture/docking pipeline was verified in
    enabled: bool = True

    ## Initial placement ------------------------------------------------------------
    # World-frame offset of the MRV *start* pose from its nominal pose [m]. The two
    # translation legs cancel it, so the MRV arrives exactly at the nominal pose.
    # Negative X = further back from the MEP (the approach axis is world +X), negative
    # Z = below it, so leg 1 is a horizontal run and leg 2 a climb, as in the spec.
    start_offset_m: List[float] = field(default_factory=lambda: [-5.0, 0.0, -3.0])
    # Per-axis fraction of `start_offset_m` cancelled by leg 1; leg 2 cancels the rest.
    # [1, 1, 0]: leg 1 is the horizontal move, leg 2 the vertical one.
    step1_axis_mask: List[float] = field(default_factory=lambda: [1.0, 1.0, 0.0])

    ## Transit ----------------------------------------------------------------------
    speed_mps: float = 0.4
    accel_mps2: float = 0.15
    # Arrival is judged on the distance to the leg target, never on elapsed time
    position_tolerance_m: float = 0.02
    # Pause at the end of each leg (visible separation of the two steps)
    settle_time_s: float = 1.0

    ## Folded arm -------------------------------------------------------------------
    # `canadarm3_large_joint_1..7` [deg]. The asset's rest pose (joints 2-5 at zero)
    # already folds the two 3.66 m booms back onto each other (measured from the USD
    # joint frames: link 3 -> 4 is +3.658 m along Z, link 4 -> 5 is -3.663 m; the elbow
    # itself still swings up to 4.25 m before folding back -- that excursion is
    # unavoidable given the boom length and stays clear of the hull either way).
    # Joint 1 keeps the azimuth of the arm's own `init_state` so the deployment only has
    # to unfold the booms. Joint 6 (wrist pitch) sets where the folded stub ends up: an
    # FK search over joint 6 found 135 deg tucks the wrist close ABOVE the shoulder
    # (EE contact point 1.32 m above the base, only 0.72 m sideways -- a vertical stow,
    # base reach 1.50 m) instead of out to the side (-90 deg: 0.59 m up but 2.17 m
    # sideways, base reach 2.24 m, reads as a horizontal fold).
    folded_joint_deg: List[float] = field(default_factory=lambda: [50.0, 0.0, 0.0, 0.0, 0.0, 135.0, 0.0])
    # Joint-space interpolation folded -> the pose the existing pipeline starts from
    deploy_duration_s: float = 10.0
    # Upper bound on the time the drives get to settle on the deployed pose before
    # SEARCH takes over; the state ends as soon as every joint is within
    # `deploy_tolerance_deg` of the target. 1.5 s left 0.36 deg / 57 mm at the EE
    # (measured), which is more than the 30 mm SEARCH treats as "arrived".
    deploy_settle_s: float = 4.0
    deploy_tolerance_deg: float = 0.1
    # Abort the phase if a leg or the deployment does not finish within this time [s]
    stage_timeout_s: float = 120.0

    ## MEP rendezvous ---------------------------------------------------------------
    # The MEP drifts and turns during the whole approach. To make the capture start from
    # exactly the state the pipeline was verified in, the free-flying bodies are rolled
    # BACK by this many seconds at t = 0 (`back_propagate_free_body`), so that after the
    # approach they are where the verified run has them at its own t = 0.
    # < 0: auto = `planned_duration_s()`, which is what it should normally be.
    mep_rendezvous_delay_s: float = -1.0

    ## Thruster plume (visual only) -------------------------------------------------
    vfx_enabled: bool = True
    # Plume cone size [m], sized to the 3.4x scaled hull
    plume_length_m: float = 2.5
    plume_radius_m: float = 0.45
    # Per-step random variation of the plume length (0 = steady)
    plume_flicker: float = 0.18
    plume_color: List[float] = field(default_factory=lambda: [0.55, 0.78, 1.0])
    # How far apart the four plumes sit on the exhaust face, as a fraction of the
    # thruster box half extent (1.0 = its corners). The box spans the full thruster
    # layout (6.4 m vertically on this hull), so the corners read as four jets floating
    # away from the bus; 0.45 keeps the quad against it.
    plume_spread: float = 0.45
    # Apex position along the exhaust axis, as a fraction of the box half extent. < 1
    # tucks the nozzle slightly inside the hull silhouette instead of hanging off it.
    plume_inset: float = 0.85

    ## Derived ----------------------------------------------------------------------
    def offset(self) -> np.ndarray:
        return np.asarray(self.start_offset_m, dtype=float).reshape(3)

    def mask(self) -> np.ndarray:
        return np.clip(np.asarray(self.step1_axis_mask, dtype=float).reshape(3), 0.0, 1.0)

    def leg_targets(self) -> Tuple[np.ndarray, np.ndarray]:
        """Offsets (relative to the nominal pose) at the end of leg 1 and leg 2."""
        d0 = self.offset()
        after_1 = d0 * (1.0 - self.mask())  # the masked components are cancelled
        return after_1, np.zeros(3)

    def leg_distances(self) -> Tuple[float, float]:
        after_1, _ = self.leg_targets()
        return float(np.linalg.norm(after_1 - self.offset())), float(np.linalg.norm(after_1))

    def leg_time_s(self, distance: float) -> float:
        """Duration of a trapezoidal leg (`speed_mps`, `accel_mps2`) of `distance` [m]."""
        if distance <= 0.0:
            return 0.0
        v, a = max(self.speed_mps, 1e-6), max(self.accel_mps2, 1e-6)
        if distance <= v * v / a:  # triangular profile: never reaches `speed_mps`
            return 2.0 * math.sqrt(distance / a)
        return distance / v + v / a

    def planned_duration_s(self) -> float:
        """Simulated time from t = 0 until the capture pipeline takes over (SEARCH).

        Used for the MEP rendezvous offset, so the MEP reaches its nominal pose when the
        capture starts instead of while the MRV is still in transit.
        """
        d1, d2 = self.leg_distances()
        return (
            0.5  # INIT
            + self.leg_time_s(d1)
            + self.settle_time_s
            + self.leg_time_s(d2)
            + self.settle_time_s
            + self.deploy_duration_s
            + self.deploy_settle_s
        )

    def roll_back_s(self) -> float:
        """Seconds the free-flying bodies are rolled back at t = 0 (0 when disabled)."""
        return self.rendezvous_delay_s()

    def rendezvous_delay_s(self) -> float:
        if not self.enabled:
            return 0.0
        return self.planned_duration_s() if self.mep_rendezvous_delay_s < 0.0 else float(self.mep_rendezvous_delay_s)

    def folded_joint_rad(self) -> np.ndarray:
        return np.radians(np.asarray(self.folded_joint_deg, dtype=float))


def validate_mrv_cfg(cfg: MrvApproachCfg):
    if len(cfg.start_offset_m) != 3 or not np.isfinite(cfg.start_offset_m).all():
        raise ValueError(f"mrv.start_offset_m must be 3 finite values [m], got {cfg.start_offset_m}")
    if len(cfg.step1_axis_mask) != 3:
        raise ValueError(f"mrv.step1_axis_mask must be 3 values, got {cfg.step1_axis_mask}")
    if len(cfg.folded_joint_deg) != 7:
        raise ValueError(f"mrv.folded_joint_deg must hold 7 values (canadarm3_large_joint_1..7), got {cfg.folded_joint_deg}")
    if len(cfg.plume_color) != 3:
        raise ValueError(f"mrv.plume_color must be 3 values (r, g, b), got {cfg.plume_color}")
    if cfg.speed_mps <= 0.0 or cfg.accel_mps2 <= 0.0:
        raise ValueError("mrv.speed_mps and mrv.accel_mps2 must be > 0")
    if cfg.position_tolerance_m <= 0.0:
        raise ValueError("mrv.position_tolerance_m must be > 0")
    if cfg.deploy_duration_s <= 0.0:
        raise ValueError("mrv.deploy_duration_s must be > 0")
    if not cfg.enabled:
        return
    d1, d2 = cfg.leg_distances()
    if min(d1, d2) < 1e-3:
        raise ValueError(
            "mrv: both translation legs must have a non-zero length "
            f"(leg 1 {d1:.3f} m, leg 2 {d2:.3f} m from start_offset_m {cfg.start_offset_m} "
            f"and step1_axis_mask {cfg.step1_axis_mask}); the demo requires two visible steps"
        )


####################
### Transit legs ###
####################


class TransitLeg:
    """One straight, velocity-profiled translation between two MRV offsets.

    The reference accelerates at `accel_mps2` up to `speed_mps` and decelerates on the
    same ramp into the target, so the (heavy, visually large) MRV never steps.
    """

    def __init__(self, start: np.ndarray, target: np.ndarray, speed: float, accel: float):
        self.start = np.asarray(start, dtype=float).copy()
        self.target = np.asarray(target, dtype=float).copy()
        delta = self.target - self.start
        self.length = float(np.linalg.norm(delta))
        self.dir = delta / self.length if self.length > 1e-9 else np.zeros(3)
        self.speed = float(speed)
        self.accel = float(accel)
        self.travelled = 0.0
        self.v = 0.0

    @property
    def remaining(self) -> float:
        return max(0.0, self.length - self.travelled)

    def step(self, dt: float) -> np.ndarray:
        """Advance the reference by one control step and return the new MRV offset."""
        # Decelerate so the reference arrives with v -> 0 (v^2 = 2 a s_remaining)
        v_stop = math.sqrt(max(0.0, 2.0 * self.accel * self.remaining))
        self.v = min(self.speed, self.v + self.accel * dt, v_stop)
        self.travelled = min(self.length, self.travelled + self.v * dt)
        return self.start + self.travelled * self.dir

    def done(self, tolerance: float) -> bool:
        return self.remaining <= tolerance


class MrvTransit:
    """Rigid translation of the MRV: Canadarm3 articulation root + spacecraft hull.

    The robot is a fixed-base articulation (`canadarm3_large_0/root_joint`), so its pose
    is commanded through `write_root_pose_to_sim` (which moves the whole articulation and
    leaves the joint positions untouched) and the hull, a static `XFormPrim`, follows with
    the same offset. Orientation is never changed -- this phase is pure translation.
    """

    def __init__(self, task: "VisionCaptureTask", arm):
        import torch

        self._torch = torch
        self.task = task
        self.arm = arm
        self.robot = task._robot
        self.device = self.robot.device
        self.nominal_root = self.robot.data.root_pos_w[0].clone()
        self.root_quat = self.robot.data.root_quat_w[0].clone()
        self.scenery = task.scene.extras.get("scenery")
        self.scenery_nominal = None
        self.scenery_quat = None
        self.hull_nominal: Optional[Frame] = None
        if self.scenery is not None:
            pos, quat = self.scenery.get_world_poses()
            self.scenery_nominal = pos[0].clone()
            self.scenery_quat = quat[0].clone()
            # Hull pose at zero offset, for the plume layer (which is built after this)
            self.hull_nominal = Frame.from_pos_quat(
                self.scenery_nominal.cpu().numpy().tolist(), self.scenery_quat.cpu().numpy().tolist()
            )
        self._offset = np.zeros(3)

    @property
    def offset(self) -> np.ndarray:
        return self._offset.copy()

    def root_pos(self) -> np.ndarray:
        return self.robot.data.root_pos_w[0].cpu().numpy().astype(float)

    def apply(self, offset) -> None:
        """Place the MRV at `nominal + offset` (world, translation only)."""
        torch = self._torch
        d = np.asarray(offset, dtype=float).reshape(3)
        self._offset = d.copy()
        delta = torch.tensor(d, dtype=self.nominal_root.dtype, device=self.nominal_root.device)
        pos = (self.nominal_root + delta).reshape(1, 3)
        pose = torch.cat((pos, self.root_quat.reshape(1, 4)), dim=1)
        self.robot.write_root_pose_to_sim(pose)
        if self.scenery is not None:
            s_delta = delta.to(self.scenery_nominal.device).to(self.scenery_nominal.dtype)
            self.scenery.set_world_poses(
                positions=(self.scenery_nominal + s_delta).reshape(1, 3),
                orientations=self.scenery_quat.reshape(1, 4),
            )
        # The IK of the capture pipeline expresses its goals in the arm base frame
        self.arm.refresh_base()

    def error_m(self, target) -> float:
        """Distance of the *measured* articulation root from `nominal + target` [m]."""
        return float(np.linalg.norm(self.root_pos() - (self.nominal_root.cpu().numpy() + np.asarray(target, dtype=float))))


def back_propagate_free_body(pose: Frame, v_w, w_w, com_body, dt: float) -> Frame:
    """Pose a free-flying rigid body must START from to be at `pose` after `dt` seconds.

    Free flight as PhysX integrates it: the centre of mass travels in a straight line and
    the body turns about it at a constant world-frame rate,

        COM(t) = COM(0) + v t,   R(t) = Exp([w] t) R(0),   p(t) = COM(t) - R(t) c_body

    so the inverse is R(0) = Exp(-[w] dt) R(dt) and COM(0) = COM(dt) - v dt. `com_body`
    is the centre of mass in the body frame, a mass property that is only known once the
    simulation is up -- which is why this is applied in the demo's `start()` and not when
    the scene is built.
    """
    # Imported here, not at module import time: `vision.py` imports this module.
    from .vision import so3_exp

    v = np.asarray(v_w, dtype=float).reshape(3)
    w = np.asarray(w_w, dtype=float).reshape(3)
    c = np.asarray(com_body, dtype=float).reshape(3)
    com_end = pose.pos + pose.rot @ c
    rot_start = so3_exp(-w * float(dt)) @ pose.rot
    com_start = com_end - v * float(dt)
    return Frame(com_start - rot_start @ c, rot_start)


####################
### Thruster VFX ###
####################


def thruster_box(task: "VisionCaptureTask") -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Half extents and centre (hull body frame, metres) of the MRV's thruster cluster.

    Read from the thruster positions the spacecraft asset itself declares
    (`ThrustActionGroup` -> `ThrusterCfg.offset`, in the unscaled body frame) and scaled
    by the spawn scale of the hull, so nothing about the nozzle layout is hard-coded
    here. Returns None when the asset declares no thrusters.
    """
    scenery = getattr(task.cfg, "_scenery", None)
    if scenery is None:
        return None
    actions = getattr(scenery, "actions", None)
    thrust = getattr(actions, "thrust", None)
    thrusters = getattr(thrust, "thrusters", None)
    if not thrusters:
        return None
    pts = np.array([list(t.offset) for t in thrusters], dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        return None
    scale = np.asarray(scenery.asset_cfg.spawn.scale, dtype=float)
    pts = pts * scale
    centre = 0.5 * (pts.max(axis=0) + pts.min(axis=0))
    half = 0.5 * (pts.max(axis=0) - pts.min(axis=0))
    return centre, np.maximum(half, 1e-3)


class ThrusterVfx:
    """Translucent plume cones on their own prims, switched on while the MRV translates.

    Deliberately kept outside the MRV asset: the prims live under
    `{ENV}/mrv_thruster_vfx`, are re-posed from the MRV offset every control step and
    carry no collider, no rigid body and no physics force. The MRV moves because
    `MrvTransit` commands it to; the plume only says so visually.
    """

    def __init__(self, task: "VisionCaptureTask", cfg: MrvApproachCfg,
                 hull_nominal: Optional[Frame] = None, quiet: bool = False):
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

        self._gf, self._usdgeom = Gf, UsdGeom
        self.cfg = cfg
        self.stage = task.scene.stage
        self.task = task
        self.visible = False
        self.num_nozzles = 4
        self._rng = np.random.default_rng(7)

        box = thruster_box(task)
        if box is None:
            raise ValueError("the MRV hull asset declares no thrusters (ThrustActionGroup)")
        self.cluster_centre, self.cluster_half = box
        # Hull body frame -> world at the nominal pose (the plume anchors are expressed
        # in the hull frame and re-posed with the MRV offset, never re-measured)
        # `hull_nominal` is the hull pose at zero MRV offset. It is passed in (from
        # `MrvTransit`, which measured it before anything moved) because the MRV may
        # already be displaced by the time the plume layer is built; reading the prim
        # here would then double-count the offset in `update`.
        if hull_nominal is None:
            scenery = task.scene.extras.get("scenery")
            if scenery is None:
                raise ValueError("no 'scenery' prim: the MRV hull is not in the scene")
            pos, quat = scenery.get_world_poses()
            hull_nominal = Frame.from_pos_quat(pos[0].cpu().numpy().tolist(), quat[0].cpu().numpy().tolist())
        self.hull_nominal = hull_nominal

        env = task.scene.env_prim_paths[0]
        self.root_path = f"{env}/mrv_thruster_vfx"
        root = UsdGeom.Xform.Define(self.stage, self.root_path)
        UsdGeom.Imageable(root.GetPrim()).MakeInvisible()

        # Three nested cones (length factor, radius factor, opacity): a bright narrow
        # core inside progressively wider, fainter shells, so the plume fades out with
        # distance from the nozzle instead of reading as one flat opaque triangle.
        self.shells = ((0.35, 0.30, 0.60), (0.65, 0.60, 0.28), (1.0, 1.0, 0.12))
        colour = np.asarray(cfg.plume_color, dtype=float)
        self.nozzles = []
        for i in range(self.num_nozzles):
            path = f"{self.root_path}/nozzle_{i}"
            xf = UsdGeom.Xform.Define(self.stage, path)
            xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            xf.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            for j, (f_len, f_rad, opacity) in enumerate(self.shells):
                mesh_path = f"{path}/shell_{j}"
                _cone_mesh(self.stage, mesh_path, cfg.plume_radius_m * f_rad, cfg.plume_length_m * f_len, Gf, UsdGeom, Vt)
                tint = np.clip(colour + (1.0 - colour) * (1.0 - j / max(1, len(self.shells) - 1)), 0.0, 1.0)
                _emissive_material(self.stage, f"{mesh_path}/Material", tint, opacity, Gf, Sdf, UsdShade)
            self.nozzles.append(xf)
        if not quiet:
            print(f"[MRV-VFX] {self.num_nozzles} plumes on separate prims under {self.root_path} "
                  f"(thruster cluster half extents {np.round(self.cluster_half, 2).tolist()} m, hull body frame; "
                  "visual only: no collider, no rigid body, no force)", flush=True)

    def anchors(self, plume_dir_w: np.ndarray, hull: Frame) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Nozzle (position, plume direction) pairs in the world frame.

        The plumes sit on the face of the declared thruster box that the exhaust leaves
        through, i.e. the trailing face for the commanded motion, spread over that face
        so they read as an RCS quad rather than one jet.
        """
        d = np.asarray(plume_dir_w, dtype=float)
        n = np.linalg.norm(d)
        if n < 1e-9:
            return []
        d = d / n
        d_body = hull.rot.T @ d  # plume direction in the hull body frame
        axis = int(np.argmax(np.abs(d_body)))
        lat = [i for i in range(3) if i != axis]
        sign = math.copysign(1.0, d_body[axis])
        spread = float(np.clip(self.cfg.plume_spread, 0.0, 1.0))
        out = []
        for s0 in (-1.0, 1.0):
            for s1 in (-1.0, 1.0):
                p = self.cluster_centre.copy()
                p[axis] += sign * self.cluster_half[axis] * float(np.clip(self.cfg.plume_inset, 0.0, 1.0))
                p[lat[0]] += s0 * spread * self.cluster_half[lat[0]]
                p[lat[1]] += s1 * spread * self.cluster_half[lat[1]]
                out.append((hull.point(p), d))
        return out

    def update(self, plume_dir_w: Optional[np.ndarray], mrv_offset: np.ndarray):
        """Show the plumes along `plume_dir_w` (None / zero = off) at the current MRV pose."""
        Gf, UsdGeom = self._gf, self._usdgeom
        root = self.stage.GetPrimAtPath(self.root_path)
        on = plume_dir_w is not None and float(np.linalg.norm(plume_dir_w)) > 1e-9 and self.cfg.vfx_enabled
        if not on:
            if self.visible:
                UsdGeom.Imageable(root).MakeInvisible()
                self.visible = False
            return
        hull = Frame(self.hull_nominal.pos + np.asarray(mrv_offset, dtype=float), self.hull_nominal.rot)
        pairs = self.anchors(np.asarray(plume_dir_w, dtype=float), hull)
        for xf, (pos, direction) in zip(self.nozzles, pairs):
            frame = _frame_towards(pos, direction)
            ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
            ops["xformOp:translate"].Set(Gf.Vec3d(*frame.pos.tolist()))
            ops["xformOp:orient"].Set(Gf.Quatf(*frame.quat))
        if self.cfg.plume_flicker > 0.0:
            for xf in self.nozzles:
                for prim in xf.GetPrim().GetChildren():
                    mesh = UsdGeom.Mesh(prim)
                    if not mesh:
                        continue
                    f = 1.0 + self.cfg.plume_flicker * float(self._rng.uniform(-1.0, 1.0))
                    attr = prim.GetAttribute("xformOp:scale")
                    if attr:
                        attr.Set(Gf.Vec3f(1.0, 1.0, float(f)))
        if not self.visible:
            UsdGeom.Imageable(root).MakeVisible()
            self.visible = True

    def off(self):
        self.update(None, np.zeros(3))


def _frame_towards(origin: np.ndarray, z_dir: np.ndarray) -> Frame:
    """Frame at `origin` with +Z along `z_dir` (the cone opens along its own +Z)."""
    from .frames import frame_from_axes

    z = np.asarray(z_dir, dtype=float)
    hint = [1.0, 0.0, 0.0] if abs(z[0] / max(1e-9, np.linalg.norm(z))) < 0.9 else [0.0, 0.0, 1.0]
    return frame_from_axes(origin, z, hint)


def _cone_mesh(stage, path: str, radius: float, length: float, Gf, UsdGeom, Vt, segments: int = 24):
    """Open cone: apex at the nozzle (origin), base circle of `radius` at +Z * `length`.

    Visual only -- no collision API and no physics API is applied anywhere on it.
    """
    ang = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    pts = [Gf.Vec3f(0.0, 0.0, 0.0)]
    pts += [Gf.Vec3f(float(radius * np.cos(a)), float(radius * np.sin(a)), float(length)) for a in ang]
    counts, idx = [], []
    for i in range(segments):
        j = (i + 1) % segments
        counts.append(3)
        # (apex, ring j, ring i): this winding puts the face normals on the OUTSIDE.
        # The other order points them inward, and the cone then renders with a dark
        # interior wedge wherever it is seen against a bright background (measured).
        idx += [0, 1 + j, 1 + i]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr([Gf.Vec3f(-radius, -radius, 0.0), Gf.Vec3f(radius, radius, float(length))])
    mesh.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    return mesh


def _emissive_material(stage, path: str, colour: np.ndarray, opacity: float, Gf, Sdf, UsdShade):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    c = Gf.Vec3f(*[float(x) for x in colour])
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(c)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(c)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path.rsplit("/", 1)[0])).Bind(mat)
    return mat


############################
### Arm deploy (joints)  ###
############################


def wrap_joint_target(q_from: "torch.Tensor", q_to: "torch.Tensor") -> "torch.Tensor":
    """Shortest-turn equivalent of `q_to`, seen from `q_from`.

    The Canadarm3 joints are continuous (`physics:lowerLimit/upperLimit` are -inf/+inf
    in `canadarm3_large.usdz`), so q and q + 2*pi*k are the same arm configuration. The
    IK that plans the deployed pose is iterative and happily returns a solution many
    turns away from the folded pose -- interpolating to *that* number would spin the
    joints for thousands of degrees. Re-expressing the target within +-180 deg of the
    folded pose gives the identical link poses over the shortest path.
    """
    import torch

    two_pi = 2.0 * math.pi
    delta = q_to - q_from
    return q_from + torch.remainder(delta + math.pi, two_pi) - math.pi


def joint_lerp(q_start: "torch.Tensor", q_goal: "torch.Tensor", alpha: float) -> "torch.Tensor":
    """q(t) = q_start + alpha * (q_goal - q_start), alpha clamped to [0, 1]."""
    a = float(min(1.0, max(0.0, alpha)))
    return q_start + a * (q_goal - q_start)


def smoothstep(u: float) -> float:
    """C1 ease-in/ease-out on [0, 1]: the deployment starts and ends at zero rate."""
    u = float(min(1.0, max(0.0, u)))
    return u * u * (3.0 - 2.0 * u)


def joint_span_deg(q_a_deg: Sequence[float], q_b_deg: Sequence[float]) -> float:
    """Largest single-joint travel between two configurations, both in degrees."""
    a = np.asarray(q_a_deg, dtype=float)
    b = np.asarray(q_b_deg, dtype=float)
    return float(np.abs(b - a).max()) if a.size else 0.0
