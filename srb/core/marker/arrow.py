from srb.core.marker import VisualizationMarkersCfg
from srb.core.sim import ArrowCfg

ARROW_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": ArrowCfg(
            tail_length=0.2,
            tail_radius=0.05,
            head_radius=0.1,
            head_length=0.15,
        ),
    }
)
