from isaaclab.envs import ManagerBasedRLEnv as __ManagerBasedRLEnv

from .cfg import ManagedEnvCfg


class ManagedEnv(__ManagerBasedRLEnv):
    cfg: ManagedEnvCfg

    def __init__(self, cfg: ManagedEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        # Apply visuals
        self.cfg.visuals.func(self.cfg.visuals)
