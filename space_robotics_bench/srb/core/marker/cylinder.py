from srb.core.marker import VisualizationMarkersCfg
from srb.core.sim import CylinderCfg

CYLINDER_CFG = VisualizationMarkersCfg(
    markers={
        "cylinder": CylinderCfg(
            radius=0.1,
            height=1.0,
        ),
    }
)
