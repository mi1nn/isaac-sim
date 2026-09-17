from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import WheeledDriveActionCfg
from srb.utils.cfg import configclass


@configclass
class WheeledDriveActionGroup(ActionGroup):
    cmd_vel: WheeledDriveActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return twist[:2]
