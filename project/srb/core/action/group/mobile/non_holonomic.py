from dataclasses import MISSING

import torch

from srb.core.action import NonHolonomicActionCfg
from srb.core.action.action_group import ActionGroup
from srb.utils.cfg import configclass


@configclass
class NonHolonomicActionGroup(ActionGroup):
    cmd_vel: NonHolonomicActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return twist[:2]
