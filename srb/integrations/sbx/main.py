from typing import TYPE_CHECKING

import gymnasium
import numpy
import rl_zoo3
import rl_zoo3.enjoy
import rl_zoo3.train
import torch
from gymnasium.core import ObservationWrapper, ObsType, WrapperObsType
from rl_zoo3 import exp_manager
from sbx import DDPG, DQN, PPO, SAC, TD3, TQC, CrossQ

from srb.integrations.sb3 import main as sb3

if TYPE_CHECKING:
    from srb._typing import AnyEnv


sb3.FRAMEWORK_NAME = "sbx"

rl_zoo3.ALGOS["ddpg"] = DDPG
rl_zoo3.ALGOS["dqn"] = DQN
rl_zoo3.ALGOS["sac"] = SAC
rl_zoo3.ALGOS["ppo"] = PPO
rl_zoo3.ALGOS["td3"] = TD3
rl_zoo3.ALGOS["tqc"] = TQC
rl_zoo3.ALGOS["crossq"] = CrossQ
rl_zoo3.train.ALGOS = rl_zoo3.ALGOS
rl_zoo3.enjoy.ALGOS = rl_zoo3.ALGOS
exp_manager.ALGOS = rl_zoo3.ALGOS


def run(env: "AnyEnv", **kwargs):
    # SBX does not support MultiInputPolicy, so merge all obs from dict into a single box
    env = MergeDictObsWrapper(env)  # type: ignore

    sb3.run(env=env, **kwargs)


class MergeDictObsWrapper(ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)

        assert isinstance(
            env.unwrapped.single_observation_space,  # type: ignore
            gymnasium.spaces.Dict,
        ), "The observation space must be of type gymnasium.spaces.Dict"

        self.dict_keys = tuple(env.unwrapped.single_observation_space.spaces.keys())  # type: ignore

        # Collect all the low and high bounds for the new Box
        low = []
        high = []
        for key in self.dict_keys:
            space = env.unwrapped.single_observation_space.spaces[key]  # type: ignore
            space_length = numpy.prod(space.shape)  # type: ignore
            space_low = space.low  # type: ignore
            space_high = space.high  # type: ignore

            if isinstance(space_low, numpy.ndarray) and space_length > 1:
                low.extend(space_low.flatten())
                high.extend(space_high.flatten())
            else:
                low.extend((space_low,) * space_length)
                high.extend((space_high,) * space_length)

        # Define the new merged observation space
        self.unwrapped.single_observation_space = gymnasium.spaces.Box(  # type: ignore
            low=numpy.array(low),
            high=numpy.array(high),
            dtype=numpy.float32,
        )

        # Ensure the correct observation and action spaces are set
        self._observation_space = self.unwrapped.single_observation_space  # type: ignore
        self._action_space = self.unwrapped.single_action_space  # type: ignore

    def observation(self, observation: ObsType) -> WrapperObsType:  # type: ignore
        return torch.cat(  # type: ignore
            [
                torch.flatten(
                    observation[key],  # type: ignore
                    start_dim=1,
                )
                for key in self.dict_keys
            ],
            dim=1,
        )
