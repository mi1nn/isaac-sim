from typing import Dict, NamedTuple, Tuple, TypeAlias

from torch import Tensor

_StepReturn: TypeAlias = Tuple[
    Dict[
        str,
        Dict[str, Tensor],
    ],
    Dict[str, Tensor],
    Tensor,
    Tensor,
]


class StepReturn(NamedTuple):
    observation: Dict[str, Dict[str, Tensor]]
    reward: Dict[str, Tensor]
    termination: Tensor
    truncation: Tensor
    info: Dict[str, Tensor] | None = None

    @staticmethod
    def from_tuple(step_return: _StepReturn) -> "StepReturn":
        return StepReturn(*step_return)
