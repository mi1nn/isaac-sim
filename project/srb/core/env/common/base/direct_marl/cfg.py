from isaaclab.envs import DirectMARLEnvCfg as __DirectMARLEnvCfg

from srb.utils.cfg import configclass

from ..env_cfg import BaseEnvCfg


@configclass
class DirectMarlEnvCfg(BaseEnvCfg, __DirectMARLEnvCfg):
    # Disable UI window by default
    ui_window_class_type: type | None = None

    def __post_init__(self):
        BaseEnvCfg.__post_init__(self)
