from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MultiShapeSpawnerCfg,
    RigidBodyPropertiesCfg,
)


class RandomShape(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/shape",
        spawn=MultiShapeSpawnerCfg(
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
    )
