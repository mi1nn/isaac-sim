from typing import Iterable, List, Tuple

import numpy
import torch
from isaaclab.envs.mdp.events import _randomize_prop_by_op  # noqa: F401
from isaaclab.utils.math import (  # noqa: F401
    sample_cylinder,
    sample_gaussian,
    sample_log_uniform,
    sample_triangle,
    sample_uniform,
)
from oxidasim.sampling import (  # noqa: F401
    sample_poisson_disk_2d,
    sample_poisson_disk_2d_looped,
    sample_poisson_disk_2d_parallel,
    sample_poisson_disk_3d,
    sample_poisson_disk_3d_looped,
    sample_poisson_disk_3d_parallel,
)
from pxr import Gf


def sample_grid(
    num_instances: int,
    spacing: float,
    global_pos_offset: numpy.ndarray | torch.Tensor | Iterable | None = None,
    global_rot_offset: numpy.ndarray | torch.Tensor | Iterable | None = None,
) -> Tuple[
    Tuple[int, int],
    Tuple[List[Tuple[float, float, float]], List[Tuple[float, float, float, float]]],
]:
    if global_pos_offset is not None:
        if isinstance(global_pos_offset, torch.Tensor):
            global_pos_offset = global_pos_offset.detach().cpu().numpy()
        elif not isinstance(global_pos_offset, numpy.ndarray):
            global_pos_offset = numpy.asarray(global_pos_offset)
    if global_rot_offset is not None:
        if isinstance(global_rot_offset, torch.Tensor):
            global_rot_offset = global_rot_offset.detach().cpu().numpy()
        elif not isinstance(global_rot_offset, numpy.ndarray):
            global_rot_offset = numpy.asarray(global_rot_offset)

    num_per_row = numpy.ceil(numpy.sqrt(num_instances))
    num_rows = numpy.ceil(num_instances / num_per_row).item()
    num_cols = numpy.ceil(num_instances / num_rows).item()

    row_offset = 0.5 * spacing * (num_rows - 1)
    col_offset = 0.5 * spacing * (num_cols - 1)

    positions = []
    orientations = []

    for i in range(num_instances):
        row = i // num_cols
        col = i % num_cols
        x = row_offset - row * spacing
        y = col * spacing - col_offset

        position = [x, y, 0]
        if global_pos_offset is not None:
            translation = tuple((global_pos_offset + position).tolist())
        else:
            translation = position
        positions.append(translation)

        orientation: Gf.Quatd = Gf.Quatd.GetIdentity()  # type: ignore
        if global_rot_offset is not None:
            orientation = (
                Gf.Quatd(
                    global_rot_offset[0].item(),
                    Gf.Vec3d(global_rot_offset[1:].tolist()),
                )
                * orientation
            )
        orientations.append(
            (
                orientation.GetReal(),
                orientation.GetImaginary()[0],
                orientation.GetImaginary()[1],
                orientation.GetImaginary()[2],
            )
        )

    return ((num_rows, num_cols), (positions, orientations))
