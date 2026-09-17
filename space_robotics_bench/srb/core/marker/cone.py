from srb.core.marker import VisualizationMarkersCfg
from srb.core.sim import ConeCfg

CONE_CFG = VisualizationMarkersCfg(
    markers={
        "cone": ConeCfg(
            radius=0.1,
            height=1.0,
        ),
    }
)
