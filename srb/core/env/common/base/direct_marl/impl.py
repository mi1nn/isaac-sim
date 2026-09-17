from isaaclab.envs import DirectMARLEnv as __DirectMARLEnv

from .cfg import DirectMarlEnvCfg


class DirectMarlEnv(__DirectMARLEnv):
    cfg: DirectMarlEnvCfg

    def __init__(self, cfg: DirectMarlEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        # Apply visuals
        self.cfg.visuals.func(self.cfg.visuals)
