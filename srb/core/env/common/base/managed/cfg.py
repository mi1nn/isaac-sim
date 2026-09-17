from isaaclab.envs import ManagerBasedRLEnvCfg as __ManagerBasedRLEnvCfg

from srb.utils.cfg import configclass

from ..env_cfg import BaseEnvCfg


@configclass
class ManagedEnvCfg(BaseEnvCfg, __ManagerBasedRLEnvCfg):
    # Disable UI window by default
    ui_window_class_type: type | None = None

    def __post_init__(self):
        BaseEnvCfg.__post_init__(self)
