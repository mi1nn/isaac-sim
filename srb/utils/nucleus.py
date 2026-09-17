from pathlib import Path

import carb.settings

from srb.utils import logging

NUCLEUS_ASSET_ROOT_DIR = carb.settings.get_settings().get(
    "/persistent/isaac/asset_root/cloud"
)
NVIDIA_NUCLEUS_DIR = f"{NUCLEUS_ASSET_ROOT_DIR}/NVIDIA"
ISAAC_NUCLEUS_DIR = f"{NUCLEUS_ASSET_ROOT_DIR}/Isaac"
ISAACLAB_NUCLEUS_DIR = f"{ISAAC_NUCLEUS_DIR}/IsaacLab"


def get_local_or_nucleus_path(local_path: Path, nucleus_path: str) -> str:
    if local_path.exists():
        return local_path.as_posix()
    logging.debug(
        f"Falling back to nucleus path {nucleus_path} because {local_path} does not exist"
    )
    return nucleus_path
