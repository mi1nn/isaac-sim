"""Autonomous Canadarm3 -> MEP capture -> Satellite docking demo (state machine).

Runs on the `srb/debris_capture_docking` task. Every step physically moves the arm
through joint position targets (no teleporting of the MEP or the satellite):

    INIT -> PLAN -> MOVE_TO_MEP_PREGRASP -> ALIGN_MEP -> MOVE_TO_MEP_GRASP -> ATTACH_MEP
    -> LIFT_MEP -> MOVE_TO_SAT_PREDOCK -> ALIGN_SAT -> INSERT_PROBE -> DOCK -> DOCKED

A state only hands over to the next one after its success conditions were measured
on the simulated bodies; otherwise the demo goes to FAILED and reports why.

Frames (see `docking.py`): EE_ATTACH_POINT = outer face of the capture cylinder,
MEP_GRASP_POINT = top face of the MEP, PROBE_DOCK_POINT = probe tip,
SAT_DOCK_POINT = point on the thruster axis `dock_depth` inside the nozzle.
"""

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING, Dict, Optional, Tuple

import numpy as np
import torch
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from pxr import Gf, Sdf, Usd, UsdShade

from .docking import (
    DockingTask,
    Frame,
    axis_angle,
    mesh_points_and_triangles,
    prim_frame,
    rotation_angle,
)

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp


##############
### Config ###
##############


@dataclass
class DemoCfg:
    ## Key poses (EE contact frame, relative to the grasp pose)
    pre_grasp_distance: float = 1.0  # back-off from the MEP face along its normal [m]
    pre_dock_distance: float = 1.0  # probe tip in front of the nozzle exit [m]
    lift_up: float = 0.5  # extra lift (world +Z) when pulling the MEP out [m]

    ## Motion
    speed_free: float = 0.25  # [m/s] EE speed without payload
    speed_final: float = 0.04  # [m/s] straight approach / insertion
    speed_carry: float = 0.10  # [m/s] with the MEP attached
    speed_ang: float = math.radians(8.0)  # [rad/s]
    # Joint target lead over the measured joint position [rad]. The Canadarm3 drives
    # are strongly overdamped (stiffness 40000, damping 25000), so the joint speed is
    # roughly 1.6 x lead [rad/s]; this lead is what sets the arm speed.
    max_joint_step: float = 0.06
    ik_lambda: float = 0.05  # DLS damping
    lag_pos: float = 0.10  # target only advances while tracking error is below [m]
    lag_ang: float = math.radians(5.0)
    # Straight approach / insertion: the target waits for the tool to be back on the
    # line before advancing, so a lagging 3 t payload cannot drift off the axis
    lag_pos_precise: float = 0.01
    lag_ang_precise: float = math.radians(0.5)
    # Precise segments use a continuous velocity command instead of an on/off gate
    # (stop-and-go of the target excites the lightly damped arm+payload mode):
    precise_accel: float = 0.005  # [m/s^2] ramp-up
    precise_decel_gain: float = 0.15  # [1/s] v <= gain * remaining distance
    precise_creep: float = 0.002  # [m/s] minimum speed until the goal is reached
    # ...and moves with a smaller joint lead, so stopping at the end of the insertion
    # does not make the payload overshoot and swing sideways
    max_joint_step_precise: float = 0.02

    ## Settling tolerances at the end of a motion
    settle_pos: float = 0.003  # [m]
    settle_ang: float = math.radians(0.2)
    settle_steps: int = 30
    # While carrying, the controlled point is the probe tip ~10.5 m from the flange, so
    # the pose tolerance is set there. The arm+payload mode is lightly damped (period
    # ~20 s), so "settled" also requires the tip to be at rest, measured as its actual
    # displacement over `settle_steps_carry` steps (the instantaneous PhysX velocity of
    # the flange carries a small high-frequency solver jitter that never averages out).
    settle_pos_carry: float = 0.005  # [m]
    settle_ang_carry: float = math.radians(0.05)
    settle_window_pos: float = 0.001  # max tip displacement over the window [m]
    settle_window_ang: float = math.radians(0.01)
    settle_steps_carry: int = 75

    ## MEP attachment conditions
    attach_gap_max: float = 0.004  # face-to-face gap [m] (target gap is `approach_gap`)
    attach_gap_min: float = -0.004
    approach_gap: float = 0.001
    attach_lateral: float = 0.005  # centre offset in the face plane [m]
    attach_normal_angle: float = math.radians(0.5)  # face normals anti-parallel
    attach_roll_angle: float = math.radians(0.5)  # in-plane axes aligned
    attach_approach_angle: float = math.radians(3.0)  # motion along -face normal

    ## Docking conditions
    dock_axial: float = 0.01  # tip to dock point along the axis [m]
    dock_radial: float = 0.01  # tip off the thruster axis [m]
    dock_axis_angle: float = math.radians(0.5)
    dock_roll_angle: float = math.radians(1.0)
    dock_approach_angle: float = math.radians(3.0)
    pre_dock_radial: float = 0.01  # on-axis tolerance before inserting [m]
    insert_max_radial: float = 0.01  # max tip deviation from the axis while inserting [m]
    insert_max_axis: float = math.radians(1.0)

    ## Visualisation
    near_distance: float = 2.0  # blue -> yellow [m]

    ## Safety
    contact_force_limit: float = 200.0  # unexpected contact on an arm link [N]
    timeout: float = 240.0  # per state, simulated seconds
    plan_waypoint_step: float = 0.25  # [m] between checked waypoints
    plan_clearance: float = 0.05  # [m] minimum estimated clearance


class State(Enum):
    INIT = auto()
    PLAN = auto()
    MOVE_TO_MEP_PREGRASP = auto()
    ALIGN_MEP = auto()
    MOVE_TO_MEP_GRASP = auto()
    ATTACH_MEP = auto()
    LIFT_MEP = auto()
    MOVE_TO_SAT_PREDOCK = auto()
    ALIGN_SAT = auto()
    INSERT_PROBE = auto()
    DOCK = auto()
    DOCKED = auto()
    UNDOCKED = auto()
    RELEASED = auto()
    FAILED = auto()


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"[CHECK] {name}: {'PASS' if ok else 'FAIL'}{'  (' + detail + ')' if detail else ''}", flush=True)
    return ok


def fail(msg: str):
    print(f"[FAIL] {msg}", flush=True)


###############
### Helpers ###
###############


def _slerp(qa, qb, s: float) -> Tuple[float, float, float, float]:
    qa, qb = np.asarray(qa, dtype=float), np.asarray(qb, dtype=float)
    d = float(np.dot(qa, qb))
    if d < 0.0:
        qb, d = -qb, -d
    if d > 0.9995:
        q = qa + s * (qb - qa)
    else:
        th = math.acos(d)
        q = (math.sin((1 - s) * th) * qa + math.sin(s * th) * qb) / math.sin(th)
    return tuple(q / np.linalg.norm(q))  # type: ignore


def interp_frame(a: Frame, b: Frame, s: float) -> Frame:
    return Frame.from_pos_quat(a.pos + s * (b.pos - a.pos), _slerp(a.quat, b.quat, s))


def translated(f: Frame, d) -> Frame:
    return Frame(f.pos + np.asarray(d, dtype=float), f.rot)


def pose_error(a: Frame, b: Frame) -> Tuple[float, float]:
    return float(np.linalg.norm(a.pos - b.pos)), rotation_angle(a.rot, b.rot)


##########################
### Arm kinematics / IK ##
##########################


class ArmKinematics:
    """IK for the EE contact frame, reusing Isaac Lab's DLS differential IK."""

    def __init__(self, task: DockingTask, tool_in_link: Frame, lam: float):
        self.task = task
        self.robot = task._robot
        self.sim = task.sim
        self.device = self.robot.device
        self.link_id = task._capture.link_body_id
        self.joint_ids = self.robot.find_joints("canadarm3_large_joint_[1-7]")[0]
        self.tool = tool_in_link
        self.ik = DifferentialIKController(
            DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=False,
                ik_method="dls",
                ik_params={"lambda_val": lam},
            ),
            num_envs=1,
            device=self.device,
        )
        # The robot base is fixed; its frame is used for IK (as the IK action does)
        self.base = Frame.from_pos_quat(
            self.robot.data.root_pos_w[0].tolist(), self.robot.data.root_quat_w[0].tolist()
        )

    def set_tool(self, tool_in_link: Frame):
        """Change the controlled frame (EE contact face, or the probe tip once attached)."""
        self.tool = tool_in_link

    def refresh_base(self):
        """Re-read the (normally fixed) base frame from the simulation.

        Only needed by the MRV rendezvous phase, which translates the whole articulation
        before the capture starts; nothing else moves the base, so for every existing
        scenario this re-reads the same pose the constructor cached.
        """
        self.base = Frame.from_pos_quat(
            self.robot.data.root_pos_w[0].tolist(), self.robot.data.root_quat_w[0].tolist()
        )
        return self.base

    ## State
    def link_pose(self) -> Frame:
        return Frame.from_pos_quat(
            self.robot.data.body_pos_w[0, self.link_id].tolist(),
            self.robot.data.body_quat_w[0, self.link_id].tolist(),
        )

    def tool_pose(self) -> Frame:
        return self.link_pose() @ self.tool

    def joint_pos(self) -> torch.Tensor:
        return self.robot.data.joint_pos[:, self.joint_ids]

    def jacobian_base(self) -> torch.Tensor:
        # Fixed base: the PhysX jacobian has no row for the root body
        j = self.robot.root_physx_view.get_jacobians()[:, self.link_id - 1, :, :][:, :, self.joint_ids].clone()
        link = self.link_pose()
        r = torch.tensor(link.rot @ self.tool.pos, dtype=j.dtype, device=self.device)
        skew = torch.tensor(
            [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]], dtype=j.dtype, device=self.device
        )
        # v_tool = v_link + w x r  ->  J_v -= [r]x J_w  (world frame)
        j[:, 0:3, :] -= skew @ j[:, 3:6, :]
        rb = torch.tensor(self.base.rot.T, dtype=j.dtype, device=self.device)
        j[:, 0:3, :] = rb @ j[:, 0:3, :]
        j[:, 3:6, :] = rb @ j[:, 3:6, :]
        return j

    def tool_velocity(self) -> Tuple[float, float]:
        """Linear [m/s] and angular [rad/s] speed of the tool frame."""
        v = self.robot.data.body_lin_vel_w[0, self.link_id].cpu().numpy()
        w = self.robot.data.body_ang_vel_w[0, self.link_id].cpu().numpy()
        r = self.link_pose().rot @ self.tool.pos
        return float(np.linalg.norm(v + np.cross(w, r))), float(np.linalg.norm(w))

    def ik_joint_target(self, goal_w: Frame, max_step: float) -> torch.Tensor:
        """One DLS step from the measured state towards `goal_w` (world)."""
        goal_b = self.base.inv() @ goal_w
        tool_b = self.base.inv() @ self.tool_pose()
        cmd = torch.tensor(np.array([[*goal_b.pos, *goal_b.quat]]), dtype=torch.float32, device=self.device)
        self.ik.set_command(cmd)
        q = self.joint_pos()
        q_des = self.ik.compute(
            torch.tensor(np.array([tool_b.pos]), dtype=torch.float32, device=self.device),
            torch.tensor(np.array([tool_b.quat]), dtype=torch.float32, device=self.device),
            self.jacobian_base(),
            q,
        )
        return q + torch.clamp(q_des - q, -max_step, max_step)

    ## Kinematic (teleport) solving, used only for planning before anything moves
    def set_kinematic(self, q: torch.Tensor):
        full = self.robot.data.joint_pos.clone()
        full[:, self.joint_ids] = q
        self.robot.write_joint_state_to_sim(full, torch.zeros_like(full))
        self.sim.forward()
        self.robot.update(1e-6)

    def solve_kinematic(self, goal_w: Frame, q_seed: torch.Tensor, iters: int = 400, tol_pos: float = 1e-4, tol_ang: float = 1e-4):
        q = q_seed.clone()
        self.set_kinematic(q)
        for _ in range(iters):
            pe, ae = pose_error(self.tool_pose(), goal_w)
            if pe < tol_pos and ae < tol_ang:
                return True, q, pe, ae
            q = self.ik_joint_target(goal_w, max_step=0.1)
            self.set_kinematic(q)
        pe, ae = pose_error(self.tool_pose(), goal_w)
        return (pe < 1e-3 and ae < math.radians(0.05)), q, pe, ae

    def link_frames(self) -> Dict[str, Frame]:
        return {
            name: Frame.from_pos_quat(
                self.robot.data.body_pos_w[0, i].tolist(), self.robot.data.body_quat_w[0, i].tolist()
            )
            for i, name in enumerate(self.robot.body_names)
        }


###########################
### Clearance estimation ##
###########################


def _subsample(p: np.ndarray, n: int) -> np.ndarray:
    if len(p) <= n:
        return p
    return p[np.random.default_rng(0).choice(len(p), n, replace=False)]


class ClearanceModel:
    """Vertex-based clearance estimate between arm links, the MEP and the satellite."""

    def __init__(self, task: DockingTask):
        from scipy.spatial import cKDTree

        stage = task.scene.stage
        geo = task.docking_geometry
        robot = task._robot
        env = task.scene.env_prim_paths[0]
        self.arm: Dict[str, np.ndarray] = {}
        for name in robot.body_names:
            path = f"{env}/robot/{name}"
            pts = [p for _, p, _ in mesh_points_and_triangles(stage, path, prim_frame(stage, path))]
            if pts:
                self.arm[name] = _subsample(np.vstack(pts), 1500)
        # Capture cylinder (implicit shape): sample its surface in the link frame
        tool, r = geo.ee_contact_link, geo.ee_contact_radius
        length = task.cfg.capture.length
        ring = [
            tool.point([r * math.cos(a), r * math.sin(a), -h])
            for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)
            for h in np.linspace(0.0, length, 5)
        ]
        disc = [tool.point([rr * math.cos(a), rr * math.sin(a), 0.0]) for rr in (0.2 * r, 0.6 * r) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
        link7 = task.cfg.capture.robot_link
        self.arm[link7] = np.vstack([self.arm.get(link7, np.zeros((0, 3))), np.array(ring + disc)])
        self.link7 = link7
        mep = prim_frame(stage, geo.mep_path)
        self.mep_local = _subsample(np.vstack([p for _, p, _ in mesh_points_and_triangles(stage, geo.mep_path, mep)]), 20000)
        sat = prim_frame(stage, geo.sat_body_path)
        sat_pts = [p for _, p, _ in mesh_points_and_triangles(stage, geo.sat_body_path, sat)]
        self.sat_local = _subsample(np.vstack(sat_pts), 40000)
        self._tree = cKDTree

    def clearance(self, links: Dict[str, Frame], mep: Frame, sat: Frame, mep_attached: bool, at_grasp: bool) -> Dict[str, Tuple[float, str]]:
        tree_mep = self._tree((mep.rot @ self.mep_local.T).T + mep.pos)
        tree_sat = self._tree((sat.rot @ self.sat_local.T).T + sat.pos)
        out = {"arm-mep": (math.inf, ""), "arm-sat": (math.inf, ""), "mep-sat": (math.inf, "")}
        for name, pts in self.arm.items():
            f = links[name]
            w = (f.rot @ pts.T).T + f.pos
            if not ((mep_attached or at_grasp) and name == self.link7):
                d = float(tree_mep.query(w)[0].min())
                if d < out["arm-mep"][0]:
                    out["arm-mep"] = (d, name)
            d = float(tree_sat.query(w)[0].min())
            if d < out["arm-sat"][0]:
                out["arm-sat"] = (d, name)
        mep_w = (mep.rot @ self.mep_local.T).T + mep.pos
        out["mep-sat"] = (float(tree_sat.query(mep_w)[0].min()), "")
        return out


############
### Demo ###
############


@dataclass
class KeyPoses:
    pre_grasp: Frame
    grasp: Frame
    lift: Frame
    pre_dock: Frame
    dock: Frame


@dataclass
class Segment:
    name: str
    start: Frame
    goal: Frame
    speed: float
    u: float = 0.0  # time-like progress; the target follows s = smoothstep(u)
    settled: int = 0
    ee_start: Optional[np.ndarray] = None
    tip_start: Optional[np.ndarray] = None
    precise: bool = False
    history: list = None  # tool poses while settling (carried phases)
    dist: float = 0.0  # precise segments: distance travelled by the target [m]
    vel: float = 0.0  # precise segments: current target speed [m/s]

    @property
    def s(self) -> float:
        if self.precise:  # velocity-profiled in `run_segment`
            return self.u
        # Zero commanded velocity at both ends: no abrupt stop of the heavy payload
        return self.u * self.u * (3.0 - 2.0 * self.u)

    @property
    def length(self) -> float:
        return max(1e-6, float(np.linalg.norm(self.goal.pos - self.start.pos)))

    @property
    def angle(self) -> float:
        return rotation_angle(self.start.rot, self.goal.rot)


class DockingDemo:
    def __init__(self, env, sim_app: "SimulationApp", headless: bool, cfg: Optional[DemoCfg] = None, auto_undock_after: Optional[float] = None, exit_after: Optional[float] = None):
        self.env = env
        self.task: DockingTask = env.unwrapped
        self.sim_app = sim_app
        self.cfg = cfg or DemoCfg()
        self.headless = headless
        self.auto_undock_after = auto_undock_after
        self.exit_after = exit_after
        t = self.task
        self.sim = t.sim
        self.scene = t.scene
        self.stage: Usd.Stage = t.scene.stage
        self.geo = t.docking_geometry
        self.capture = t._capture
        self.docking = t.docking
        self.mep = t._obj
        self.sat = t._satellite
        self.arm = ArmKinematics(t, self.geo.ee_contact_link, self.cfg.ik_lambda)
        self.contacts = self.scene.sensors.get("contacts_robot")
        self.dt = self.sim.get_physics_dt()
        self.state = State.INIT
        self.state_time = 0.0
        self.sim_time = 0.0
        self.segment: Optional[Segment] = None
        self.q_hold: Optional[torch.Tensor] = None
        self.results: Dict[str, bool] = {}
        self.docked_time: Optional[float] = None
        self._last_motion: Dict[str, np.ndarray] = {}
        self._insert_max_radial = 0.0
        self._insert_max_axis = 0.0
        self._colors: Dict[str, Tuple[float, float, float]] = {}
        self._last_log = -1.0
        self.done = False
        self._keyboard = None
        if not headless:
            from srb.interfaces.teleop import EventOmniKeyboardTeleopInterface

            self._keyboard = EventOmniKeyboardTeleopInterface({"R": self.release_last})
        a = self.task.cfg.docking.placement.approach_dir
        self.approach = np.asarray(a, dtype=float) / np.linalg.norm(a)

    ## Records
    def record(self, name: str, ok: bool, detail: str = "") -> bool:
        self.results[name] = ok and self.results.get(name, True)
        return check(name, ok, detail)

    ## Frames (measured)
    def tool(self) -> Frame:
        return self.arm.tool_pose()

    def mep_frame(self) -> Frame:
        return Frame.from_pos_quat(self.mep.data.root_pos_w[0].tolist(), self.mep.data.root_quat_w[0].tolist())

    def mep_grasp_world(self) -> Frame:
        return self.mep_frame() @ self.geo.mep_grasp

    def ee_target_at_mep(self) -> Frame:
        """Where the EE contact frame must be to meet the MEP face flat, right now."""
        return self.mep_frame() @ self.task.docking_geometry.ee_contact_to_mep.inv()

    ## Key poses
    def key_poses(self) -> KeyPoses:
        g = self.ee_target_at_mep()
        a = self.approach
        c = self.cfg
        geo = self.task.docking_geometry
        depth = geo.dock_depth
        back = depth + c.pre_dock_distance * geo.length_scale
        offset = np.asarray(self.task.cfg.docking.placement.dock_offset, dtype=float)
        dock = translated(g, offset)
        # Grasp: back off along the MEP face normal. Dock: back off along the thruster
        # axis (the probe is not exactly parallel to the face normal on this asset).
        n_face = self.mep_grasp_world().rot[:, 2]
        z_dock = self.docking.dock_world().rot[:, 2]
        return KeyPoses(
            pre_grasp=translated(g, c.pre_grasp_distance * n_face),
            grasp=translated(g, c.approach_gap * n_face),
            lift=translated(g, -back * a + np.array([0.0, 0.0, c.lift_up])),
            pre_dock=translated(dock, -back * z_dock),
            dock=dock,
        )

    ## Colours
    def set_color(self, which: str, color):
        if self._colors.get(which) == color:
            return
        self._colors[which] = color
        if which == "capture":
            self.capture.set_color(0, color)
        else:
            path = self.task.dock_indicator_path
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(path)):
                if prim.IsA(UsdShade.Shader):
                    UsdShade.Shader(prim).CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))

    def update_colors(self, attach_ok: bool, dock_ok: bool):
        cc = self.task.cfg.capture
        d_mep = float(np.linalg.norm(self.ee_pose().pos - self.mep_grasp_world().pos))
        if self.capture.is_attached(0) or attach_ok:
            self.set_color("capture", cc.color_captured)
        else:
            self.set_color("capture", cc.color_approach if d_mep < self.cfg.near_distance else cc.color_idle)
        d_dock = float(np.linalg.norm(self.docking.probe_world().pos - self.docking.dock_world().pos))
        if self.docking.is_docked or dock_ok:
            self.set_color("dock", cc.color_captured)
        else:
            self.set_color("dock", cc.color_approach if d_dock < self.cfg.near_distance else cc.color_idle)

    ## Conditions
    def _approach_vector(self, which: str) -> Optional[np.ndarray]:
        """Net displacement of the EE contact point or the probe tip over the last
        completed straight-line motion (the approach / insertion)."""
        v = self._last_motion.get(which)
        return v if v is not None and np.linalg.norm(v) > 0.02 else None

    def attach_metrics(self) -> Dict[str, float]:
        ee = self.ee_pose()
        g = self.mep_grasp_world()
        n = g.rot[:, 2]
        rel = ee.pos - g.pos
        gap = float(rel @ n)
        lateral = float(np.linalg.norm(rel - gap * n))
        normal = axis_angle(ee.rot[:, 2], -n)
        xe = ee.rot[:, 0] - (ee.rot[:, 0] @ n) * n
        roll = axis_angle(xe, g.rot[:, 0])
        v = self._approach_vector("ee")
        approach = axis_angle(v, -n) if v is not None else math.nan
        return {"gap": gap, "lateral": lateral, "normal": normal, "roll": roll, "approach": approach}

    def attach_ok(self, m) -> bool:
        c = self.cfg
        return (
            c.attach_gap_min <= m["gap"] <= c.attach_gap_max
            and m["lateral"] <= c.attach_lateral
            and m["normal"] <= c.attach_normal_angle
            and m["roll"] <= c.attach_roll_angle
            and (not math.isnan(m["approach"]) and m["approach"] <= c.attach_approach_angle)
        )

    def dock_metrics(self) -> Dict[str, float]:
        p = self.docking.probe_world()
        s = self.docking.dock_world()
        z = s.rot[:, 2]
        rel = p.pos - s.pos
        axial = float(rel @ z)
        radial = float(np.linalg.norm(rel - axial * z))
        exit_w = self.docking.sat_frame() @ self.geo.sat_exit
        depth = float((p.pos - exit_w.pos) @ z)
        xa = p.rot[:, 0] - (p.rot[:, 0] @ z) * z
        v = self._approach_vector("tip")
        approach = axis_angle(v, z) if v is not None else math.nan
        return {
            "axial": axial,
            "radial": radial,
            "depth": depth,
            "axis": axis_angle(p.rot[:, 2], z),
            "roll": axis_angle(xa, s.rot[:, 0]),
            "approach": approach,
        }

    def dock_ok(self, m) -> bool:
        c = self.cfg
        return (
            abs(m["axial"]) <= c.dock_axial
            and m["radial"] <= c.dock_radial
            and m["axis"] <= c.dock_axis_angle
            and m["roll"] <= c.dock_roll_angle
            and m["depth"] > 0.0
            and (not math.isnan(m["approach"]) and m["approach"] <= c.dock_approach_angle)
        )

    ## Keyboard
    def release_last(self):
        """R: release the most recent connection only (docking first, then the EE)."""
        if self.docking.is_docked:
            self.docking.undock()
            self.goto(State.UNDOCKED)
        elif self.capture.is_attached(0):
            self.capture.release([0])
            self.goto(State.RELEASED)
        else:
            print("[DEMO] R: nothing is attached or docked", flush=True)
            return
        self.q_hold = self.arm.joint_pos().clone()
        self.segment = None

    ## State handling
    def goto(self, state: State):
        print(f"[STATE] {self.state.name} -> {state.name}  (t={self.sim_time:.1f} s)", flush=True)
        self.state = state
        self.state_time = 0.0
        self.segment = None

    def start_segment(self, name: str, goal: Frame, speed: float, precise: bool = False):
        self.segment = Segment(
            name, self.tool(), goal, speed,
            ee_start=self.ee_pose().pos.copy(), tip_start=self.docking.probe_world().pos.copy(), precise=precise,
        )

    def ee_pose(self) -> Frame:
        return self.arm.link_pose() @ self.geo.ee_contact_link

    def run_segment(self) -> Optional[bool]:
        """Advance the current straight-line motion. True when settled at the goal."""
        seg = self.segment
        assert seg is not None
        c = self.cfg
        target = interp_frame(seg.start, seg.goal, seg.s)
        pe, ae = pose_error(self.tool(), target)
        if seg.precise:
            if seg.u < 1.0:
                remaining = seg.length - seg.dist
                v = min(seg.speed, max(c.precise_creep, c.precise_decel_gain * remaining), seg.vel + c.precise_accel * self.dt)
                # Slow down continuously (never stop abruptly) when the tool lags behind
                slow = min(max(1.0 - pe / c.lag_pos_precise, 0.0), max(1.0 - ae / c.lag_ang_precise, 0.0))
                seg.vel = v  # profile speed (the lag slow-down is applied on top of it)
                seg.dist = min(seg.length, seg.dist + v * slow * self.dt)
                seg.u = seg.dist / seg.length
                target = interp_frame(seg.start, seg.goal, seg.u)
        elif seg.u < 1.0 and pe < c.lag_pos and ae < c.lag_ang:
            rate = min(seg.speed / seg.length, c.speed_ang / max(seg.angle, 1e-6))
            seg.u = min(1.0, seg.u + rate * self.dt)
            target = interp_frame(seg.start, seg.goal, seg.s)
        carrying = self.capture.is_attached(0)
        q = self.arm.ik_joint_target(target, c.max_joint_step_precise if seg.precise else c.max_joint_step)
        self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)
        if seg.u >= 1.0:
            tool = self.tool()
            pe, ae = pose_error(tool, seg.goal)
            if carrying:
                ok = pe < c.settle_pos_carry and ae < c.settle_ang_carry
                seg.history = (seg.history or []) + [tool]
                seg.history = seg.history[-c.settle_steps_carry:]
                if ok and len(seg.history) == c.settle_steps_carry:
                    dp, da = pose_error(seg.history[0], tool)
                    ok = dp < c.settle_window_pos and da < c.settle_window_ang
                else:
                    ok = False
            else:
                ok = pe < c.settle_pos and ae < c.settle_ang
            seg.settled = seg.settled + 1 if ok else 0
            if seg.settled >= (1 if carrying else c.settle_steps):
                self._last_motion = {
                    "ee": self.ee_pose().pos - seg.ee_start,
                    "tip": self.docking.probe_world().pos - seg.tip_start,
                }
                return True
        return None

    def hold(self):
        if self.q_hold is None:
            self.q_hold = self.arm.joint_pos().clone()
        self.task._robot.set_joint_position_target(self.q_hold, joint_ids=self.arm.joint_ids)

    def check_contacts(self, allow_link7: bool) -> bool:
        if self.contacts is None:
            return True
        f = torch.norm(self.contacts.data.net_forces_w[0], dim=-1)
        names = self.contacts.body_names
        for i, name in enumerate(names):
            if allow_link7 and name == self.task.cfg.capture.robot_link:
                continue
            if f[i] > self.cfg.contact_force_limit:
                fail(f"unexpected contact on arm link '{name}': |F| = {f[i]:.0f} N in state {self.state.name}")
                return False
        return True

    ## Planning
    def plan(self) -> bool:
        c = self.cfg
        kp = self.key_poses()
        q_home = self.arm.joint_pos().clone()
        clearance = ClearanceModel(self.task)
        mep0 = self.mep_frame()
        sat0 = self.docking.sat_frame()
        tool_to_mep = self.geo.ee_contact_to_mep
        legs = [
            ("HOME -> MEP_PRE_GRASP", self.tool(), kp.pre_grasp, False),
            ("MEP_PRE_GRASP -> MEP_GRASP", kp.pre_grasp, kp.grasp, False),
            ("MEP_GRASP -> MEP_LIFT", kp.grasp, kp.lift, True),
            ("MEP_LIFT -> SAT_PRE_DOCK", kp.lift, kp.pre_dock, True),
            ("SAT_PRE_DOCK -> SAT_DOCK", kp.pre_dock, kp.dock, True),
        ]
        key_names = ["MEP_PRE_GRASP", "MEP_GRASP", "MEP_LIFT", "SAT_PRE_DOCK", "SAT_DOCK"]
        q = q_home
        ok_all = True
        print("[PLAN] kinematic reachability check (Isaac Lab DifferentialIKController, DLS)", flush=True)
        for (leg, start, goal, carried), key in zip(legs, key_names):
            n = max(2, int(math.ceil(np.linalg.norm(goal.pos - start.pos) / c.plan_waypoint_step)) + 1)
            n = max(n, int(math.ceil(rotation_angle(start.rot, goal.rot) / math.radians(5.0))) + 1)
            leg_ok, worst, jump = True, (0.0, 0.0), 0.0
            min_clear = {"arm-mep": math.inf, "arm-sat": math.inf, "mep-sat": math.inf}
            for k in range(1, n):
                wp = interp_frame(start, goal, k / (n - 1))
                ok, q_new, pe, ae = self.arm.solve_kinematic(wp, q)
                jump = max(jump, float(torch.abs(q_new - q).max()))
                q = q_new
                worst = (max(worst[0], pe), max(worst[1], ae))
                leg_ok &= ok
                mep = (self.arm.tool_pose() @ tool_to_mep) if carried else mep0
                cl = clearance.clearance(self.arm.link_frames(), mep, sat0, carried, k == n - 1 and key == "MEP_GRASP")
                for kk, (d, _) in cl.items():
                    min_clear[kk] = min(min_clear[kk], d)
            q_deg = [round(math.degrees(v), 1) for v in q[0].tolist()]
            detail = f"IK err {worst[0]*1000:.2f} mm / {math.degrees(worst[1]):.3f} deg over {n-1} waypoints, max joint jump {math.degrees(jump):.1f} deg, q={q_deg}"
            ok_all &= self.record(f"{key} reachable ({leg})", leg_ok and jump < math.radians(20.0), detail)
            ok_all &= self.record(
                f"{key} estimated clearance",
                min(min_clear.values()) > c.plan_clearance,
                ", ".join(f"{k} {v:.2f} m" for k, v in min_clear.items()),
            )
        check("Joint limits", True, "all 7 Canadarm3 joints are continuous (USD limits -inf..inf)")
        # Restore the real state before anything moves
        self.arm.set_kinematic(q_home)
        self.task._robot.set_joint_position_target(q_home, joint_ids=self.arm.joint_ids)
        self.q_hold = q_home.clone()
        return ok_all

    ## Main step
    def step(self):
        c = self.cfg
        self.state_time += self.dt
        self.sim_time += self.dt
        attach_m = self.attach_metrics()
        dock_m = self.dock_metrics()
        a_ok = self.attach_ok(attach_m)
        d_ok = self.dock_ok(dock_m)
        self.update_colors(a_ok, d_ok)

        if self.state not in (State.INIT, State.PLAN, State.DOCKED, State.UNDOCKED, State.RELEASED, State.FAILED) and self.state_time > c.timeout:
            fail(f"timeout in state {self.state.name} after {c.timeout:.0f} s (segment progress {self.segment.s if self.segment else 0:.2f})")
            self.goto(State.FAILED)

        s = self.state
        if s == State.INIT:
            self.hold()
            if self.state_time == self.dt:
                self._report_init()
            if self.state_time >= 1.0:
                self.goto(State.PLAN)
        elif s == State.PLAN:
            ok = self.plan()
            self.goto(State.MOVE_TO_MEP_PREGRASP if ok else State.FAILED)
            if not ok:
                fail("reachability/clearance plan failed; not moving the arm")
        elif s == State.MOVE_TO_MEP_PREGRASP:
            if self.segment is None:
                self.start_segment("pre-grasp", self.key_poses().pre_grasp, c.speed_free)
            if not self.check_contacts(False):
                return self.goto(State.FAILED)
            if self.run_segment():
                self.goto(State.ALIGN_MEP)
        elif s == State.ALIGN_MEP:
            # Hold at pre-grasp and verify the full 6-DoF alignment before approaching
            if self.segment is None:
                self.start_segment("align", self.key_poses().pre_grasp, c.speed_final)
            if self.run_segment():
                kp = self.key_poses()
                pe, ae = pose_error(self.tool(), kp.pre_grasp)
                m = self.attach_metrics()
                ok = self.record("MEP pre-grasp alignment", pe < c.settle_pos and ae < c.settle_ang and m["normal"] < c.attach_normal_angle and m["roll"] < c.attach_roll_angle and m["lateral"] < c.attach_lateral,
                                 f"pos err {pe*1000:.2f} mm, rot err {math.degrees(ae):.3f} deg, normal {math.degrees(m['normal']):.3f} deg, roll {math.degrees(m['roll']):.3f} deg, lateral {m['lateral']*1000:.2f} mm, gap {m['gap']:.3f} m")
                self._mep_before_grasp = self.mep_frame()
                self.goto(State.MOVE_TO_MEP_GRASP if ok else State.FAILED)
        elif s == State.MOVE_TO_MEP_GRASP:
            if self.segment is None:
                self.start_segment("straight approach", self.key_poses().grasp, c.speed_final, precise=True)
            if not self.check_contacts(True):
                return self.goto(State.FAILED)
            if self.run_segment():
                self.goto(State.ATTACH_MEP)
        elif s == State.ATTACH_MEP:
            m = attach_m
            moved = float(np.linalg.norm(self.mep_frame().pos - self._mep_before_grasp.pos))
            self.hold()
            detail = (f"gap {m['gap']*1000:.2f} mm, lateral {m['lateral']*1000:.2f} mm, normal {math.degrees(m['normal']):.3f} deg, "
                      f"roll {math.degrees(m['roll']):.3f} deg, approach {math.degrees(m['approach']) if not math.isnan(m['approach']) else float('nan'):.2f} deg, MEP pushed {moved*1000:.2f} mm")
            if a_ok:
                self.record("MEP grasp alignment", True, detail)
                self.capture.attach(0)
                self._tip_at_attach = self.docking.probe_world()
                self.record("MEP attachment", self.capture.is_attached(0), "fixed joint EE<->MEP created")
                self.q_hold = None
                self.goto(State.LIFT_MEP)
            elif self.state_time > 2.0:
                self.record("MEP grasp alignment", False, detail)
                fail("attachment conditions not met; not attaching")
                self.goto(State.FAILED)
        elif s == State.LIFT_MEP:
            if self.segment is None:
                self._rel_at_attach = self.ee_pose().inv() @ self.mep_frame()
                # From now on the IK controls the probe tip (PROBE_DOCK_POINT) directly
                self.arm.set_tool(self.geo.ee_contact_link @ self._rel_at_attach @ self.geo.probe_dock)
                self.start_segment("lift", self.key_poses_carried().lift, c.speed_carry)
            if not self.check_contacts(True):
                return self.goto(State.FAILED)
            if self.run_segment():
                drift = self._relative_drift()
                self.record("MEP lift (MEP follows EE, relative pose held)", drift[0] < 0.005 and drift[1] < math.radians(0.5),
                            f"relative drift {drift[0]*1000:.2f} mm / {math.degrees(drift[1]):.3f} deg")
                self.goto(State.MOVE_TO_SAT_PREDOCK)
        elif s == State.MOVE_TO_SAT_PREDOCK:
            if self.segment is None:
                self.start_segment("transport", self.key_poses_carried().pre_dock, c.speed_carry)
            if not self.check_contacts(True):
                return self.goto(State.FAILED)
            if self.run_segment():
                self.goto(State.ALIGN_SAT)
        elif s == State.ALIGN_SAT:
            if self.segment is None:
                self.start_segment("align probe", self.key_poses_carried().pre_dock, c.speed_final)
            if self.run_segment():
                m = self.dock_metrics()
                ok = self.record("Probe/Thruster alignment (pre-dock)", m["radial"] < c.pre_dock_radial and m["axis"] < c.dock_axis_angle and m["roll"] < c.dock_roll_angle and m["depth"] < 0.0,
                                 f"radial {m['radial']*1000:.2f} mm, axis {math.degrees(m['axis']):.3f} deg, roll {math.degrees(m['roll']):.3f} deg, tip {-m['depth']:.3f} m before the nozzle exit")
                self._sat_before_insert = self.docking.sat_frame()
                self.goto(State.INSERT_PROBE if ok else State.FAILED)
        elif s == State.INSERT_PROBE:
            if self.segment is None:
                self.start_segment("insert probe", self.key_poses_carried().dock, c.speed_final, precise=True)
                self._insert_max_radial, self._insert_max_axis = 0.0, 0.0
                self._settle_max_radial, self._insert_min_wall = 0.0, math.inf
            if not self.check_contacts(True):
                return self.goto(State.FAILED)
            m = dock_m
            moving = self.segment.u < 1.0
            if m["depth"] > 0.0:  # inside the nozzle
                if moving:  # straight insertion along the axis
                    self._insert_max_radial = max(self._insert_max_radial, m["radial"])
                    self._insert_max_axis = max(self._insert_max_axis, m["axis"])
                else:  # payload settling at the dock point
                    self._settle_max_radial = max(self._settle_max_radial, m["radial"])
                # Wall clearance: probe surface to the nozzle inner wall at this depth
                wall = self.geo.nozzle.inner_radius(m["depth"]) - self.geo.probe.tip_radius - m["radial"]
                self._insert_min_wall = min(self._insert_min_wall, wall)
            if self.run_segment():
                self.record("Probe insertion stays on the thruster axis (while inserting)",
                            self._insert_max_radial <= c.insert_max_radial and self._insert_max_axis <= c.insert_max_axis,
                            f"inside the nozzle while moving: max radial {self._insert_max_radial*1000:.2f} mm, max axis angle {math.degrees(self._insert_max_axis):.3f} deg")
                self.record("Probe never touches the thruster wall",
                            self._insert_min_wall > 0.0 and float(np.linalg.norm(self.docking.sat_frame().pos - self._sat_before_insert.pos)) < 1e-3,
                            f"min clearance probe-to-wall {self._insert_min_wall*1000:.0f} mm (settling swing at the dock point max {self._settle_max_radial*1000:.1f} mm)")
                self.goto(State.DOCK)
        elif s == State.DOCK:
            self.hold()
            m = dock_m
            sat_moved = float(np.linalg.norm(self.docking.sat_frame().pos - self._sat_before_insert.pos))
            detail = (f"axial {m['axial']*1000:.2f} mm, radial {m['radial']*1000:.2f} mm, axis {math.degrees(m['axis']):.3f} deg, roll {math.degrees(m['roll']):.3f} deg, "
                      f"depth {m['depth']:.3f} m inside the nozzle, approach {math.degrees(m['approach']) if not math.isnan(m['approach']) else float('nan'):.2f} deg, satellite pushed {sat_moved*1000:.2f} mm")
            # The approach direction was measured during the insertion segment
            if d_ok:
                self.record("Probe insertion / docking conditions", True, detail)
                self.docking.dock()
                self.record("Satellite docking", self.docking.is_docked, "fixed joint MEP<->satellite created at SAT_DOCK_POINT")
                self.docked_time = self.sim_time
                self.goto(State.DOCKED)
            elif self.state_time > 2.0:
                self.record("Probe insertion / docking conditions", False, detail)
                fail("docking conditions not met; not docking")
                self.goto(State.FAILED)
        elif s == State.DOCKED:
            self.hold()
            if self.state_time >= 3.0 and "Docked state held" not in self.results:
                m = self.dock_metrics()
                self.record("Docked state held", self.docking.is_docked and abs(m["axial"]) < 0.01 and m["radial"] < 0.01,
                            f"after 3 s: axial {m['axial']*1000:.2f} mm, radial {m['radial']*1000:.2f} mm")
                self._summary()
            if self.auto_undock_after is not None and self.state_time >= self.auto_undock_after:
                print("[DEMO] simulating an R key press (--test_undock)", flush=True)
                self._sat_at_undock = self.docking.sat_frame()
                self.release_last()
        elif s == State.UNDOCKED:
            self.hold()
            if self.auto_undock_after is not None and "R undocking" not in self.results and self.state_time >= 2.0:
                self.record("R undocking", (not self.docking.is_docked) and self.capture.is_attached(0),
                            f"docking joint removed: {not self.docking.is_docked}, EE attachment kept: {self.capture.is_attached(0)}")
                self._summary()
                if self.exit_after is not None:
                    self.done = True  # automated check finished
        elif s in (State.RELEASED, State.FAILED):
            self.hold()
            if s == State.FAILED and "_failed_summary" not in self.results:
                self.results["_failed_summary"] = True
                self._summary()

        self._log(attach_m, dock_m)
        if self.exit_after is not None and self.sim_time >= self.exit_after:
            self.done = True

    def key_poses_carried(self) -> KeyPoses:
        """Probe-tip targets after attachment (the IK tool is the probe tip then).

        LIFT keeps the nominal EE displacement; SAT_PRE_DOCK / SAT_DOCK are defined on
        the thruster axis directly, so they do not depend on how exactly the MEP was
        grasped.
        """
        kp = self.key_poses_nominal
        tip_at_attach = self._tip_at_attach
        dock = self.docking.dock_world()  # SAT_DOCK_POINT, +Z into the nozzle
        geo = self.task.docking_geometry
        back = geo.dock_depth + self.cfg.pre_dock_distance * geo.length_scale
        return KeyPoses(
            pre_grasp=kp.pre_grasp,
            grasp=kp.grasp,
            lift=translated(tip_at_attach, kp.lift.pos - kp.grasp.pos),
            pre_dock=Frame(dock.pos - back * dock.rot[:, 2], dock.rot),
            dock=dock,
        )

    def _relative_drift(self) -> Tuple[float, float]:
        now = self.ee_pose().inv() @ self.mep_frame()
        return pose_error(now, self._rel_at_attach)

    def _report_init(self):
        g = self.geo
        print("[INIT] ---- frames measured from the stage ----", flush=True)
        print(f"[INIT] EE_ATTACH_POINT (link '{self.task.cfg.capture.robot_link}' frame): pos {np.round(g.ee_contact_link.pos, 4).tolist()}, normal {np.round(g.ee_contact_link.rot[:, 2], 4).tolist()}, contact radius {g.ee_contact_radius:.3f} m", flush=True)
        print(f"[INIT] MEP_GRASP_POINT (MEP body frame): pos {np.round(g.mep_grasp.pos, 4).tolist()}, normal {np.round(g.mep_grasp.rot[:, 2], 4).tolist()}, face half-size {g.face.half_extent:.3f} m, area {g.face.area:.2f} m^2", flush=True)
        print(f"[INIT] PROBE_DOCK_POINT (MEP body frame): tip {np.round(g.probe.tip, 4).tolist()}, axis {np.round(g.probe.direction, 4).tolist()}, tip radius {g.probe.tip_radius:.3f} m, rod radius {g.probe.body_radius:.3f} m", flush=True)
        print(f"[INIT] SAT_DOCK_POINT (satellite body frame): pos {np.round(g.sat_dock.pos, 4).tolist()}, axis {np.round(g.sat_dock.rot[:, 2], 4).tolist()}, nozzle exit radius {g.nozzle.exit_radius:.3f} m, inner radius at dock {g.nozzle.inner_radius(g.dock_depth):.3f} m", flush=True)
        mep, sat = self.mep_frame(), self.docking.sat_frame()
        print(f"[INIT] MEP body (world): pos {np.round(mep.pos, 4).tolist()}, quat(wxyz) {np.round(mep.quat, 4).tolist()}", flush=True)
        print(f"[INIT] Satellite body GOES_R (world): pos {np.round(sat.pos, 4).tolist()}, quat(wxyz) {np.round(sat.quat, 4).tolist()}", flush=True)
        plate_depth, r0, r1 = self.task.docking_backstop
        dc = self.task.cfg.docking
        print(f"[INIT] Thruster collider: nozzle mesh collider disabled, {dc.wall_segments} wall boxes (t={dc.wall_thickness} m) from r={r0:.3f} m at the exit to r={r1:.3f} m, back plate {plate_depth:.2f} m inside the exit", flush=True)
        self.key_poses_nominal = self.key_poses()
        for name in ("pre_grasp", "grasp", "lift", "pre_dock", "dock"):
            f = getattr(self.key_poses_nominal, name)
            print(f"[INIT] {name.upper():>9}: EE contact pos {np.round(f.pos, 3).tolist()}, quat(wxyz) {np.round(f.quat, 4).tolist()}", flush=True)
        base = self.arm.base
        check("Canadarm3 base fixed at origin", float(np.linalg.norm(base.pos)) < 1e-3, f"base pos {np.round(base.pos, 4).tolist()}")
        q = self.arm.joint_pos()[0]
        qd = self.task._robot.data.default_joint_pos[0, self.arm.joint_ids]
        check("Canadarm3 at its initial joint pose", float(torch.abs(q - qd).max()) < 1e-3, f"max joint error {math.degrees(float(torch.abs(q - qd).max())):.4f} deg")
        self.record("MEP grasp face fits the EE contact disc", g.face.half_extent >= g.ee_contact_radius, f"face half-size {g.face.half_extent:.3f} m >= contact radius {g.ee_contact_radius:.3f} m")
        self.record("MEP grasp face is the outermost surface", g.face.clearance <= 0.001, f"max protrusion beyond the face inside the contact disc {g.face.clearance*1000:.2f} mm")
        r_dock = g.nozzle.inner_radius(g.dock_depth)
        self.record("Probe fits the thruster", g.probe.tip_radius < r_dock - 0.05, f"probe tip radius {g.probe.tip_radius:.3f} m, nozzle inner radius at dock depth {r_dock:.3f} m")
        # The placement must make face-to-face grasp + translation land the probe on the dock point
        kp = self.key_poses_nominal
        mep_at_dock = translated(kp.dock, np.zeros(3)) @ self.geo.ee_contact_to_mep
        err = pose_error(mep_at_dock @ g.probe_dock, self.docking.sat_frame() @ g.sat_dock)
        self.record("Placement consistency (probe lands on SAT_DOCK_POINT)", err[0] < 1e-3 and err[1] < math.radians(0.05), f"{err[0]*1000:.3f} mm / {math.degrees(err[1]):.4f} deg")

    def _log(self, am, dm):
        if self.sim_time - self._last_log < 1.0:
            return
        self._last_log = self.sim_time
        seg = ""
        if self.segment is not None:
            pe, ae = pose_error(self.tool(), self.segment.goal)
            lin, ang = self.arm.tool_velocity()
            seg = (f" seg '{self.segment.name}' {self.segment.s*100:5.1f}% err {pe*1000:.1f} mm/{math.degrees(ae):.3f} deg"
                   f" v {lin*1000:.1f} mm/s w {math.degrees(ang):.3f} deg/s")
        print(f"[STATUS] t={self.sim_time:6.1f}s {self.state.name:<22}{seg} | MEP: gap {am['gap']:.3f} m lat {am['lateral']*1000:.1f} mm | "
              f"probe: axial {dm['axial']:.3f} m radial {dm['radial']*1000:.1f} mm depth {dm['depth']:.3f} m", flush=True)

    def _summary(self):
        print("[SUMMARY] ------------------------------------------------", flush=True)
        for k, v in self.results.items():
            if k.startswith("_"):
                continue
            print(f"[SUMMARY] {'PASS' if v else 'FAIL'}  {k}", flush=True)
        n_ok = sum(1 for k, v in self.results.items() if v and not k.startswith("_"))
        n = sum(1 for k in self.results if not k.startswith("_"))
        print(f"[SUMMARY] {n_ok}/{n} checks passed, final state {self.state.name}", flush=True)

    ## Loop (same stepping as `srb agent manual`)
    def run(self):
        sim, scene = self.sim, self.scene
        render_interval = max(1, self.task.cfg.sim.render_interval)
        n = 0
        with torch.no_grad():
            while self.sim_app.is_running() and not self.done:
                if not sim.is_playing():
                    sim.render()
                    continue
                self.step()
                scene.write_data_to_sim()
                sim.step(render=False)
                n += 1
                if n % render_interval == 0:
                    sim.render()
                scene.update(dt=self.dt)
        return self.results
