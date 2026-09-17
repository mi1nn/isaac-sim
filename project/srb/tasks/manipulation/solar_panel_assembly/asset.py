from typing import TYPE_CHECKING, Tuple

from pydantic import BaseModel

from srb import assets
from srb.core.asset import RigidObjectCfg

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


class PanelCfg(BaseModel, arbitrary_types_allowed=True):
    asset_cfg: RigidObjectCfg
    offset_pos: Tuple[float, float, float] = (0.0, 0.0, 0.15)


@staticmethod
def select_solar_panel(
    env_cfg: "AnyEnvCfg", *, init_state: RigidObjectCfg.InitialStateCfg
) -> PanelCfg:
    offset_pos = (0.0, 0.0, 0.15)
    init_state.pos = (
        init_state.pos[0] + offset_pos[0],
        init_state.pos[1] + offset_pos[1],
        init_state.pos[2] + offset_pos[2],
    )

    cfg = assets.SolarPanel().asset_cfg
    cfg.init_state = init_state

    return PanelCfg(
        asset_cfg=cfg,
        offset_pos=offset_pos,
    )
