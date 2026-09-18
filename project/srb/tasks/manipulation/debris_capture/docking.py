"""Satellite docking variant of `debris_capture`: Canadarm3 -> MEP -> Satellite.

Everything geometric in here is measured from the spawned USD at runtime rather than
hard-coded, so the frames always match the assets actually in the stage:

- EE contact frame  : centre/normal of the outer end face of the capture cylinder
                      (`capture.py`) on the Canadarm3 flange link.
- MEP grasp frame   : centre/normal of the outermost flat face of the MEP body that
                      faces away from the probe (the top of the Fermi telescope bus,
                      coaxial with the probe).
- Probe dock frame  : tip of the Ares1 rod ("probe") and its axis (pointing out).
- Satellite dock    : a point on the GOES_R thruster (bell nozzle) axis, `dock_depth`
                      inside the nozzle exit, axis pointing into the nozzle.

The MEP and the satellite are *placed* from those frames and from an EE grasp pose
chosen for the Canadarm3 workspace (see `DockingPlacementCfg`), so the whole demo is
physically consistent: grasping the MEP face-to-face and translating it by `offset`
puts the probe tip exactly on the satellite dock point, coaxial with the thruster.
"""

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterable, List, Sequence, Tuple

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from srb.utils.cfg import configclass

from .capture import CaptureCfg
from .task import Task, TaskCfg

if TYPE_CHECKING:
    from srb.core.asset import RigidObject


###############
### Helpers ###
###############


@dataclass
class Frame:
    """Rigid frame: `pos` (3,) and rotation matrix `rot` (3, 3), columns = axes."""

    pos: np.ndarray
    rot: np.ndarray

    def __matmul__(self, other: "Frame") -> "Frame":
        return Frame(self.pos + self.rot @ other.pos, self.rot @ other.rot)

    def inv(self) -> "Frame":
        return Frame(-self.rot.T @ self.pos, self.rot.T)

    def point(self, p) -> np.ndarray:
        return self.pos + self.rot @ np.asarray(p, dtype=float)

    @property
    def quat(self) -> Tuple[float, float, float, float]:
        return rotmat_to_quat_wxyz(self.rot)

    @staticmethod
    def from_pos_quat(pos, quat_wxyz) -> "Frame":
        return Frame(np.asarray(pos, dtype=float), quat_wxyz_to_rotmat(quat_wxyz))

    @staticmethod
    def identity() -> "Frame":
        return Frame(np.zeros(3), np.eye(3))


def quat_wxyz_to_rotmat(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotmat_to_quat_wxyz(r: np.ndarray) -> Tuple[float, float, float, float]:
    m = np.asarray(r, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    if q[0] < 0:
        q = tuple(-v for v in q)
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)  # type: ignore


def frame_from_axes(origin, z, x_hint) -> Frame:
    """Right-handed frame with +Z = `z` and +X as close to `x_hint` as possible."""
    z = np.asarray(z, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x_hint, dtype=float) - np.dot(x_hint, z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0.0, 0.0, 1.0] if abs(z[2]) < 0.9 else [1.0, 0.0, 0.0])
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return Frame(np.asarray(origin, dtype=float), np.column_stack((x, y, z)))


def rotation_angle(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """Angle [rad] of the rotation taking frame a onto frame b."""
    c = (np.trace(r_a.T @ r_b) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def axis_angle(u, v) -> float:
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return math.acos(max(-1.0, min(1.0, c)))


def _world_gf(stage: Usd.Stage, path: str) -> Gf.Matrix4d:
    return UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))


def prim_frame(stage: Usd.Stage, path: str) -> Frame:
    """World frame of a prim without its scale (the physics body frame)."""
    tf = Gf.Transform(_world_gf(stage, path))
    q = tf.GetRotation().GetQuat()
    return Frame.from_pos_quat(
        tuple(tf.GetTranslation()), (q.GetReal(), *tuple(q.GetImaginary()))
    )


def prim_scale(stage: Usd.Stage, path: str) -> np.ndarray:
    return np.array(tuple(Gf.Transform(_world_gf(stage, path)).GetScale()))


def mesh_points_and_triangles(
    stage: Usd.Stage, root_path: str, frame: Frame
) -> List[Tuple[str, np.ndarray, np.ndarray]]:
    """Points (in `frame`, metres) and fan-triangulated faces of every active mesh."""
    out = []
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise ValueError(f"Prim not found: {root_path}")
    inv = frame.inv()
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        m = np.array(_world_gf(stage, str(prim.GetPath())))  # row-vector convention
        p = np.asarray(pts, dtype=float)
        pw = np.hstack((p, np.ones((len(p), 1)))) @ m
        pb = (inv.rot @ pw[:, :3].T).T + inv.pos
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        tris = []
        k = 0
        for c in counts:
            for i in range(1, c - 1):
                tris.append((idx[k], idx[k + i], idx[k + i + 1]))
            k += c
        out.append((str(prim.GetPath()), pb, np.asarray(tris, dtype=int).reshape(-1, 3)))
    return out


def fit_line(points: np.ndarray, iterations: int = 8, keep: float = 0.9) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Trimmed PCA line fit; returns (centre, unit direction, inlier mask).

    Each pass drops the points farthest from the current line (keeping `keep` of the
    remaining inliers), so small detached meshes cannot tilt the axis of a long rod.
    """
    mask = np.ones(len(points), dtype=bool)
    for _ in range(iterations):
        p = points[mask]
        c = p.mean(axis=0)
        d = np.linalg.svd(p - c, full_matrices=False)[2][0]
        rel = points - c
        dist = np.linalg.norm(rel - np.outer(rel @ d, d), axis=1)
        mask = mask & (dist <= np.quantile(dist[mask], keep))
    # Final inliers: everything close to the converged line (restores trimmed rod points)
    mask = dist < max(0.05, 3.0 * float(np.quantile(dist[mask], 0.95)))
    p = points[mask]
    c = p.mean(axis=0)
    d = np.linalg.svd(p - c, full_matrices=False)[2][0]
    return c, d / np.linalg.norm(d), mask


##########################
### Geometry extraction ##
##########################


@dataclass
class ProbeGeometry:
    tip: np.ndarray  # MEP body frame [m]
    direction: np.ndarray  # unit, pointing out of the tip (insertion direction)
    tip_radius: float  # max radius within `tip_section` of the tip [m]
    body_radius: float  # max radius of the whole rod [m]
    length: float


@dataclass
class FaceGeometry:
    centre: np.ndarray  # body frame [m]
    normal: np.ndarray  # unit, pointing out of the body
    half_extent: float  # inscribed half-size of the face around its centre [m]
    area: float
    clearance: float  # how far any other MEP geometry protrudes beyond the face [m]


@dataclass
class NozzleGeometry:
    exit_centre: np.ndarray  # satellite body frame [m]
    direction: np.ndarray  # unit, from the exit into the nozzle
    exit_radius: float
    profile: List[Tuple[float, float]]  # (depth, inner radius) [m]

    def inner_radius(self, depth: float) -> float:
        d = np.array([p[0] for p in self.profile])
        r = np.array([p[1] for p in self.profile])
        return float(np.interp(depth, d, r))


def extract_probe(stage, mep_path: str, probe_rel: str, body_rel: str, frame: Frame, tip_section: float) -> ProbeGeometry:
    pts = np.vstack([p for _, p, _ in mesh_points_and_triangles(stage, f"{mep_path}/{probe_rel}", frame)])
    c, d, inliers = fit_line(pts)
    rod = pts[inliers]
    body = np.vstack([p for _, p, _ in mesh_points_and_triangles(stage, f"{mep_path}/{body_rel}", frame)])
    t = (rod - c) @ d
    # The tip is the end of the rod farther from the telescope body
    if np.linalg.norm(c + d * t.min() - body.mean(axis=0)) > np.linalg.norm(c + d * t.max() - body.mean(axis=0)):
        d, t = -d, -t
    tip = c + d * t.max()
    rel = rod - c
    radial = np.linalg.norm(rel - np.outer(rel @ d, d), axis=1)
    near_tip = t > t.max() - tip_section
    return ProbeGeometry(
        tip=tip,
        direction=d,
        tip_radius=float(radial[near_tip].max()),
        body_radius=float(radial.max()),
        length=float(t.max() - t.min()),
    )


def extract_grasp_face(stage, mep_path: str, body_rel: str, frame: Frame, outward: np.ndarray, contact_radius: float, min_area: float = 0.05) -> FaceGeometry:
    """Outermost flat face (>= `min_area`) of the MEP body whose normal points along `outward`."""
    tris_c, tris_n, tris_a, tris_v = [], [], [], []
    for _, p, tris in mesh_points_and_triangles(stage, f"{mep_path}/{body_rel}", frame):
        if len(tris) == 0:
            continue
        a, b, c = p[tris[:, 0]], p[tris[:, 1]], p[tris[:, 2]]
        n = np.cross(b - a, c - a)
        area = 0.5 * np.linalg.norm(n, axis=1)
        ok = area > 1e-10
        n[ok] /= (2.0 * area[ok])[:, None]
        tris_c.append((a + b + c) / 3.0)
        tris_n.append(n)
        tris_a.append(np.where(ok, area, 0.0))
        tris_v.append(np.stack((a, b, c), axis=1))
    cen, nrm, area, verts = (np.concatenate(x) for x in (tris_c, tris_n, tris_a, tris_v))
    # Candidates roughly facing `outward` (the probe axis may be tilted a little from
    # the true face normal), then a reference normal from their area-weighted mean
    cand = (area > 0) & (nrm @ outward > math.cos(math.radians(3.0)))
    if not cand.any():
        raise RuntimeError("No flat face found on the MEP facing away from the probe")
    n_ref = (nrm[cand] * area[cand, None]).sum(axis=0)
    n_ref /= np.linalg.norm(n_ref)
    sel = (area > 0) & (nrm @ n_ref > math.cos(math.radians(0.5)))
    offset = cen @ n_ref
    # Group coplanar triangles into 1 cm planes; keep the outermost plane that is a real
    # face (enough area), not a sliver or a small bump
    bins = np.round(offset[sel] / 0.01).astype(int)
    planes = {}
    for b_, a_ in zip(bins, area[sel]):
        planes[b_] = planes.get(b_, 0.0) + a_
    top_bin = max(b_ for b_, a_ in planes.items() if a_ >= min_area)
    face = sel.copy()
    face[sel] = np.abs(bins - top_bin) <= 1
    area_f = float(area[face].sum())
    centre = (cen[face] * area[face, None]).sum(axis=0) / area_f
    normal = (nrm[face] * area[face, None]).sum(axis=0)
    normal /= np.linalg.norm(normal)
    area = area_f
    verts = verts[face].reshape(-1, 3)
    # Inscribed half-size: distance from the centre to the nearest face boundary,
    # approximated by the in-plane extent of the face vertices in 16 directions
    rel = verts - centre
    rel -= np.outer(rel @ normal, normal)
    u = np.cross(normal, [1.0, 0.0, 0.0] if abs(normal[0]) < 0.9 else [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    half = min(
        float(np.max(rel @ (math.cos(a) * u + math.sin(a) * v)))
        for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)
    )
    # Anything of the whole MEP protruding beyond the face inside the contact disc
    everything = np.vstack([p for _, p, _ in mesh_points_and_triangles(stage, mep_path, frame)])
    rel_all = everything - centre
    along = rel_all @ normal
    lateral = np.linalg.norm(rel_all - np.outer(along, normal), axis=1)
    inside = lateral < contact_radius
    protrusion = float(along[inside].max()) if inside.any() else 0.0
    return FaceGeometry(centre=centre, normal=normal, half_extent=half, area=area, clearance=protrusion)


def extract_nozzle(stage, nozzle_path: str, frame: Frame) -> NozzleGeometry:
    pts = np.vstack([p for _, p, _ in mesh_points_and_triangles(stage, nozzle_path, frame)])
    c, d, _ = fit_line(pts, iterations=1)
    t = (pts - c) @ d
    length = t.max() - t.min()

    def end_ring(at_max: bool):
        sel = t > t.max() - 0.02 * length if at_max else t < t.min() + 0.02 * length
        ring = pts[sel]
        centre = ring.mean(axis=0)
        rel = ring - centre
        return centre, float(np.linalg.norm(rel - np.outer(rel @ d, d), axis=1).mean())

    (c_hi, r_hi), (c_lo, r_lo) = end_ring(True), end_ring(False)
    if r_hi > r_lo:  # the wide end is the exit
        exit_c, exit_r, into = c_hi, r_hi, -d
    else:
        exit_c, exit_r, into = c_lo, r_lo, d
    rel = pts - exit_c
    depth = rel @ into
    radial = np.linalg.norm(rel - np.outer(depth, into), axis=1)
    profile = []
    edges = np.linspace(0.0, depth.max(), 25)
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (depth >= lo) & (depth < hi)
        if sel.any():
            profile.append((float(0.5 * (lo + hi)), float(radial[sel].min())))
    profile.insert(0, (0.0, exit_r))
    return NozzleGeometry(exit_centre=exit_c, direction=into, exit_radius=exit_r, profile=profile)


def extract_ee_contact(stage, link_path: str, cylinder_path: str) -> Tuple[Frame, float]:
    """Frame of the outer end face of the capture cylinder in the link frame.

    +Z of the returned frame is the outward face normal (the approach direction),
    +X follows the link's +X. Also returns the cylinder radius.
    """
    link = prim_frame(stage, link_path)
    geom = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(cylinder_path)):
        if prim.IsA(UsdGeom.Cylinder):
            geom = prim
            break
    if geom is None:
        raise ValueError(f"No UsdGeom.Cylinder under {cylinder_path}")
    cyl = UsdGeom.Cylinder(geom)
    h, r, axis = cyl.GetHeightAttr().Get(), cyl.GetRadiusAttr().Get(), cyl.GetAxisAttr().Get()
    a = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    m = _world_gf(stage, str(geom.GetPath()))
    ends = [np.array(tuple(m.Transform(Gf.Vec3d(*(s * 0.5 * h * np.array(a)))))) for s in (1.0, -1.0)]
    ends = [link.inv().point(e) for e in ends]
    contact = max(ends, key=np.linalg.norm)  # the end farther from the link origin
    other = min(ends, key=np.linalg.norm)
    normal = (contact - other) / np.linalg.norm(contact - other)
    radius = float(r * np.abs(np.array(tuple(Gf.Transform(m).GetScale()))).max())
    return frame_from_axes(contact, normal, [1.0, 0.0, 0.0]), radius


##############
### Config ###
##############


@configclass
class DockingPlacementCfg:
    """Where the demo happens, expressed as EE *contact-face* poses (world frame).

    The MEP is placed so that its grasp face exactly meets the EE contact face when
    the EE contact frame is at `grasp_pos` pointing along `approach_dir`. The satellite
    is placed so that translating that grasped MEP by `dock_offset` puts the probe tip
    on the satellite dock point, coaxial with the thruster. The Canadarm3 must reach all
    key poses with the EE contact frame pointing along `approach_dir` (horizontal);
    `dock_offset` is along +Y so the satellite (which extends ~4 m below its nozzle axis
    towards the MEP start) stays clear of the MEP's initial pose. Reachability and
    clearance are re-checked by the demo's PLAN state on every start.
    """

    grasp_pos: Tuple[float, float, float] = (5.0, -2.75, 3.0)
    approach_dir: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    # Direction of MEP-body +X around the approach axis (sets the roll of the MEP)
    roll_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    dock_offset: Tuple[float, float, float] = (0.0, 5.5, 0.0)


@configclass
class DockingCfg:
    placement: DockingPlacementCfg = DockingPlacementCfg()

    ## MEP
    mep_body_rel: str = "Fermi_Gamma_ray_Large_Area_Space_Telescope"
    probe_rel: str = "Xform_Ares1"
    probe_tip_section: float = 0.9  # length at the tip used for its radius [m]

    ## Satellite
    satellite_body_rel: str = "GOES_R"
    thruster_rel: str = "GOES_R/Thruster"
    # Depth of SAT_DOCK_POINT inside the nozzle exit, along the thruster axis [m]
    dock_depth: float = 0.8
    # The back plate that stops the probe sits this far behind the dock point [m]
    backstop_gap: float = 0.1
    # Hollow thruster collider (replaces the solid convex hull of the nozzle mesh)
    wall_segments: int = 16
    wall_thickness: float = 0.03  # [m]
    wall_overlap: float = 1.15  # tangential overlap between neighbouring segments

    ## Visual feedback for the docking (translucent disc at the nozzle exit)
    indicator_radius_factor: float = 1.0  # x nozzle exit radius
    indicator_thickness: float = 0.05  # [m]


class DockingGeometry:
    """All docking-relevant frames, measured once from the spawned stage."""

    def __init__(self, stage: Usd.Stage, env_prim_path: str, cfg: DockingCfg, capture_cfg: CaptureCfg, robot_link_path: str):
        self.cfg = cfg
        self.mep_path = f"{env_prim_path}/debris"
        self.sat_prim_path = f"{env_prim_path}/satellite"
        self.sat_body_path = f"{self.sat_prim_path}/{cfg.satellite_body_rel}"
        self.link_path = robot_link_path

        ## EE contact frame (link frame)
        self.ee_contact_link, self.ee_contact_radius = extract_ee_contact(
            stage, robot_link_path, f"{robot_link_path}/capture_cylinder"
        )

        ## MEP frames (MEP body frame)
        mep = prim_frame(stage, self.mep_path)
        self.probe = extract_probe(stage, self.mep_path, cfg.probe_rel, cfg.mep_body_rel, mep, cfg.probe_tip_section)
        self.face = extract_grasp_face(
            stage, self.mep_path, cfg.mep_body_rel, mep, -self.probe.direction, self.ee_contact_radius
        )
        # MEP_GRASP_POINT: +Z = outward face normal; +X = MEP body +X
        self.mep_grasp = frame_from_axes(self.face.centre, self.face.normal, [1.0, 0.0, 0.0])
        # PROBE_DOCK_POINT: +Z = insertion direction (out of the tip); +X = MEP body +X
        self.probe_dock = frame_from_axes(self.probe.tip, self.probe.direction, [1.0, 0.0, 0.0])

        ## Satellite frames (satellite body frame)
        sat = prim_frame(stage, self.sat_body_path)
        self.sat_scale = prim_scale(stage, self.sat_body_path)
        self.nozzle = extract_nozzle(stage, f"{self.sat_prim_path}/{cfg.thruster_rel}", sat)
        dock_point = self.nozzle.exit_centre + cfg.dock_depth * self.nozzle.direction
        # SAT_DOCK_POINT: +Z = into the nozzle; +X = satellite body +X
        self.sat_dock = frame_from_axes(dock_point, self.nozzle.direction, [1.0, 0.0, 0.0])
        self.sat_exit = frame_from_axes(self.nozzle.exit_centre, self.nozzle.direction, [1.0, 0.0, 0.0])

        ## Prim <-> body transforms (the satellite body is a child of its prim)
        self.sat_prim_from_body = prim_frame(stage, self.sat_prim_path).inv() @ prim_frame(stage, self.sat_body_path)

    ## Relations
    @property
    def ee_contact_to_mep(self) -> Frame:
        """MEP body pose expressed in the EE contact frame when the faces meet.

        The faces are coincident and facing each other: contact +Z = -grasp +Z,
        contact +X = grasp +X (a half turn about X).
        """
        flip = Frame(np.zeros(3), np.diag([1.0, -1.0, -1.0]))
        return flip @ self.mep_grasp.inv()

    def placement(self) -> Tuple[Frame, Frame, Frame]:
        """World poses of (EE contact at grasp, MEP body, satellite body)."""
        p = self.cfg.placement
        a = np.asarray(p.approach_dir, dtype=float)
        # The EE contact +Z is the approach direction; its +X is chosen so that the
        # MEP body +X ends up along `roll_hint` (the grasp X axes coincide).
        ee_grasp = frame_from_axes(p.grasp_pos, a, p.roll_hint)
        mep = ee_grasp @ self.ee_contact_to_mep
        mep_docked = Frame(mep.pos + np.asarray(p.dock_offset, dtype=float), mep.rot)
        probe_docked = mep_docked @ self.probe_dock
        sat_body = probe_docked @ self.sat_dock.inv()
        return ee_grasp, mep, sat_body


###########################
### Docking attachment ###
###########################


class DockingManager:
    """Fixed joint between the MEP body and the satellite body, anchored at the dock point."""

    def __init__(self, stage: Usd.Stage, env_prim_path: str, geometry: DockingGeometry, mep: "RigidObject", satellite: "RigidObject"):
        self._stage = stage
        self._geo = geometry
        self._mep = mep
        self._sat = satellite
        self._joint_path = f"{env_prim_path}/docking_joint"
        self._mep_scale = prim_scale(stage, geometry.mep_path)
        self._sat_scale = geometry.sat_scale

    @property
    def is_docked(self) -> bool:
        return self._stage.GetPrimAtPath(self._joint_path).IsValid()

    def mep_frame(self) -> Frame:
        return Frame.from_pos_quat(self._mep.data.root_pos_w[0].tolist(), self._mep.data.root_quat_w[0].tolist())

    def sat_frame(self) -> Frame:
        return Frame.from_pos_quat(self._sat.data.root_pos_w[0].tolist(), self._sat.data.root_quat_w[0].tolist())

    def probe_world(self) -> Frame:
        return self.mep_frame() @ self._geo.probe_dock

    def dock_world(self) -> Frame:
        return self.sat_frame() @ self._geo.sat_dock

    def dock(self):
        if self.is_docked:
            return
        # Freeze the current relative pose (no snap); anchor at the probe tip so any
        # joint compliance acts at the docking interface itself.
        mep, sat = self.mep_frame(), self.sat_frame()
        anchor = self.probe_world()
        in_mep = mep.inv() @ anchor
        in_sat = sat.inv() @ anchor
        joint = UsdPhysics.FixedJoint.Define(self._stage, self._joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._geo.mep_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(self._geo.sat_body_path)])
        # PhysX applies each body's USD scale to the joint local positions
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(in_mep.pos / self._mep_scale).tolist()))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(*in_mep.quat))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*(in_sat.pos / self._sat_scale).tolist()))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(*in_sat.quat))
        print("[DOCK] MEP docked to satellite thruster", flush=True)

    def undock(self):
        if self.is_docked:
            self._stage.RemovePrim(self._joint_path)
            print("[DOCK] MEP undocked from satellite", flush=True)


#############################
### Stage modifications ###
#############################


def build_thruster_collider(stage: Usd.Stage, geo: DockingGeometry, cfg: DockingCfg):
    """Replace the solid nozzle hull with a hollow shell + a back plate.

    PhysX turns the thruster mesh of a dynamic body into its convex hull, i.e. a solid
    cone that no probe can enter. The mesh collider is disabled and replaced by
    `wall_segments` thin boxes along the inner wall (straight between the exit and the
    back plate) plus a back plate `backstop_gap` behind the dock point. The entrance
    stays open; the probe cannot pass through the satellite.
    """
    thruster_path = f"{geo.sat_prim_path}/{cfg.thruster_rel}"
    for prim in Usd.PrimRange(stage.GetPrimAtPath(thruster_path)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)

    # Everything is authored under the satellite body in its *local* (scaled) units
    scale = float(geo.sat_scale.mean())
    body_from_local = 1.0 / scale
    root = f"{geo.sat_body_path}/DockingThrusterCollider"
    UsdGeom.Xform.Define(stage, root)
    nz = geo.nozzle
    plate_depth = cfg.dock_depth + cfg.backstop_gap
    r0, r1 = nz.inner_radius(0.0), nz.inner_radius(plate_depth)
    t = cfg.wall_thickness
    length = math.hypot(plate_depth, r0 - r1)
    tilt = math.atan2(r0 - r1, plate_depth)
    frame0 = geo.sat_exit
    n = cfg.wall_segments
    width = 2.0 * math.pi * (0.5 * (r0 + r1) + t) / n * cfg.wall_overlap

    def add_box(name: str, frame: Frame, size):
        cube = UsdGeom.Cube.Define(stage, f"{root}/{name}")
        cube.CreateSizeAttr(1.0)
        cube.CreatePurposeAttr(UsdGeom.Tokens.guide)  # collision only, not rendered
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*(frame.pos * body_from_local).tolist()))
        xf.AddOrientOp().Set(Gf.Quatf(*frame.quat))
        xf.AddScaleOp().Set(Gf.Vec3f(*(np.asarray(size) * body_from_local).tolist()))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    for i in range(n):
        phi = 2.0 * math.pi * i / n
        radial = frame0.rot @ np.array([math.cos(phi), math.sin(phi), 0.0])
        tangent = frame0.rot @ np.array([-math.sin(phi), math.cos(phi), 0.0])
        axis = frame0.rot[:, 2]
        # Segment axis follows the wall from the exit (r0) to the plate depth (r1)
        along = math.cos(tilt) * axis - math.sin(tilt) * radial
        mid_r = 0.5 * (r0 + r1) + 0.5 * t / math.cos(tilt)
        centre = frame0.pos + 0.5 * plate_depth * axis + mid_r * radial
        # Box: X = tangential width, Y = thickness (radial-ish), Z = along the wall
        rot = np.column_stack((tangent, np.cross(along, tangent), along))
        add_box(f"wall_{i:02d}", Frame(centre, rot), (width, t, length))
    plate = Frame(frame0.pos + (plate_depth + 0.5 * t) * frame0.rot[:, 2], frame0.rot)
    add_box("back_plate", plate, (2.2 * r1 + 2 * t, 2.2 * r1 + 2 * t, t))
    return plate_depth, r0, r1


def spawn_dock_indicator(stage: Usd.Stage, geo: DockingGeometry, cfg: DockingCfg, capture_cfg: CaptureCfg) -> str:
    """Translucent visual-only disc at the nozzle exit, recoloured by the demo."""
    from srb.core.sim import CylinderCfg, PreviewSurfaceCfg

    scale = float(geo.sat_scale.mean())
    frame = Frame(geo.sat_exit.pos - 0.5 * cfg.indicator_thickness * geo.sat_exit.rot[:, 2], geo.sat_exit.rot)
    path = f"{geo.sat_body_path}/dock_indicator"
    shape = CylinderCfg(
        radius=geo.nozzle.exit_radius * cfg.indicator_radius_factor / scale,
        height=cfg.indicator_thickness / scale,
        axis="Z",
        visual_material=PreviewSurfaceCfg(diffuse_color=capture_cfg.color_idle, opacity=capture_cfg.opacity),
    )
    shape.func(path, shape, translation=tuple((frame.pos / scale).tolist()), orientation=frame.quat)
    return path


def set_prim_pose(stage: Usd.Stage, path: str, frame: Frame):
    prim = stage.GetPrimAtPath(path)
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*frame.pos.tolist()))
    orient = prim.GetAttribute("xformOp:orient")
    q = frame.quat
    if orient.GetTypeName() == Sdf.ValueTypeNames.Quatf:
        orient.Set(Gf.Quatf(*q))
    else:
        orient.Set(Gf.Quatd(*q))


def disable_collisions(stage: Usd.Stage, paths: Iterable[str]):
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)


############
### Task ###
############


@configclass
class DockingTaskCfg(TaskCfg):
    docking: DockingCfg = DockingCfg()
    # MEP prims whose colliders are removed: the capture marker sits on the grasp face
    # and would otherwise stick 5 mm out of it (it is only a visual marker).
    mep_collision_free_relpaths: Tuple[str, ...] = ("gripper_fixture",)

    def __post_init__(self):
        super().__post_init__()
        # The demo decides when to attach (6-DoF alignment), not the distance alone,
        # and the EE contact face needs a collider to meet the MEP face physically.
        self.capture.auto_attach = False
        self.capture.collision = True


class DockingTask(Task):
    cfg: DockingTaskCfg

    def __init__(self, cfg: DockingTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        self._satellite: "RigidObject" = self.scene["satellite"]
        self.docking = DockingManager(
            self.scene.stage, self.scene.env_prim_paths[0], self.docking_geometry, self._obj, self._satellite
        )

    def _setup_scene(self):
        super()._setup_scene()
        stage = self.scene.stage
        env = self.scene.env_prim_paths[0]
        if len(self.scene.env_prim_paths) != 1:
            raise ValueError("The docking demo supports a single environment")
        disable_collisions(stage, [f"{env}/debris/{p}" for p in self.cfg.mep_collision_free_relpaths])
        link = f"{env}/robot/{self.cfg.capture.robot_link}"
        geo = DockingGeometry(stage, env, self.cfg.docking, self.cfg.capture, link)

        ## Place the MEP and the satellite from the measured frames
        _, mep, sat_body = geo.placement()
        sat_prim = sat_body @ geo.sat_prim_from_body.inv()
        set_prim_pose(stage, geo.mep_path, mep)
        set_prim_pose(stage, geo.sat_prim_path, sat_prim)
        # Keep resets consistent with the computed placement
        self.cfg.scene.debris.init_state.pos = tuple(mep.pos.tolist())
        self.cfg.scene.debris.init_state.rot = mep.quat
        self.cfg.scene.satellite.init_state.pos = tuple(sat_prim.pos.tolist())
        self.cfg.scene.satellite.init_state.rot = sat_prim.quat
        for name, frame in (("debris", mep), ("satellite", sat_prim)):
            asset_cfg = self.scene[name].cfg
            asset_cfg.init_state.pos = tuple(frame.pos.tolist())
            asset_cfg.init_state.rot = frame.quat

        self.docking_backstop = build_thruster_collider(stage, geo, self.cfg.docking)
        self.dock_indicator_path = spawn_dock_indicator(stage, geo, self.cfg.docking, self.cfg.capture)
        self.docking_geometry = geo

    def _reset_idx(self, env_ids: Sequence[int]):
        # Undock before the reset teleports the bodies (same reason as the capture)
        if hasattr(self, "docking"):
            self.docking.undock()
        super()._reset_idx(env_ids)
