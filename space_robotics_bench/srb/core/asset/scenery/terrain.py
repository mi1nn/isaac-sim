from __future__ import annotations

from typing import ClassVar, Sequence, Type

from isaaclab.terrains import *  # noqa: F403

from srb.core.asset.scenery.scenery import Scenery, SceneryRegistry
from srb.core.asset.scenery.scenery_type import SceneryType
from srb.core.domain import Domain


class Terrain(Scenery, scenery_entrypoint=SceneryType.TERRAIN):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = (
        Domain.ASTEROID,
        Domain.EARTH,
        Domain.MARS,
        Domain.MOON,
    )

    @classmethod
    def scenery_registry(cls) -> Sequence[Type[Terrain]]:
        return SceneryRegistry.registry.get(SceneryType.TERRAIN, [])  # type: ignore
