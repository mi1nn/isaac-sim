from typing import TYPE_CHECKING

import elements
from embodied import Driver

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


class DriverParallelEnv(Driver):
    def __init__(self, env: "AnyEnvCfg", num_envs: int, **kwargs):
        self.kwargs = kwargs
        self.length = num_envs
        self.env = env
        self.act_space = self.env.act_space  # type: ignore
        self.callbacks = []
        self.acts = None
        self.carry = None
        self.reset()

    def close(self):
        self.env.close()

    def _step(self, policy, step, episode):
        acts = self.acts
        obs = self.env.step(acts)  # type: ignore
        logs = {k: v for k, v in obs.items() if k.startswith("log/")}  # type: ignore
        obs = {k: v for k, v in obs.items() if not k.startswith("log/")}  # type: ignore
        self.carry, acts, outs = policy(self.carry, obs, **self.kwargs)
        if obs["is_last"].any():
            mask = ~obs["is_last"]
            acts = {k: self._mask(v, mask) for k, v in acts.items()}
        self.acts = {**acts, "reset": obs["is_last"].copy()}
        trans = {**obs, **acts, **outs, **logs}
        for i in range(self.length):
            trn = elements.tree.map(lambda x: x[i], trans)
            [fn(trn, i, **self.kwargs) for fn in self.callbacks]
        step += len(obs["is_first"])
        episode += obs["is_last"].sum()
        return step, episode
