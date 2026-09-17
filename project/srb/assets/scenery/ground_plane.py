from srb.core.asset import AssetBaseCfg, Terrain
from srb.core.sim import GroundPlaneCfg


class GroundPlane(Terrain):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/ground_plane",
        spawn=GroundPlaneCfg(
            color=(0.0, 158.0 / 255.0, 218.0 / 255.0),
        ),
    )
