"""Rigid-frame helpers shared by the docking demo and the vision capture.

Pure numpy (no Isaac Sim / USD imports), so the vision math can be unit-tested
without launching the simulator.
"""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


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
