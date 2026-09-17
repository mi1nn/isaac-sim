import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import BodyAccelerationActionCfg
from srb.utils.cfg import configclass


@configclass
class BodyAccelerationActionGroup(ActionGroup):
    body_vel: BodyAccelerationActionCfg = BodyAccelerationActionCfg(asset_name="robot")

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return twist


@configclass
class BodyAccelerationRelativeActionGroup(ActionGroup):
    body_vel: BodyAccelerationActionCfg = BodyAccelerationActionCfg(
        asset_name="robot", relative=True
    )

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return twist
