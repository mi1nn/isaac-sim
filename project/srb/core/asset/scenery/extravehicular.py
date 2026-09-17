from __future__ import annotations

from typing import ClassVar, Sequence, Type

from srb.core.asset.scenery.scenery import Scenery, SceneryRegistry
from srb.core.asset.scenery.scenery_type import SceneryType
from srb.core.domain import Domain


class ExtravehicularScenery(Scenery, scenery_entrypoint=SceneryType.EXTRAVEHICULAR):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = (Domain.ORBIT,)

    @classmethod
    def scenery_registry(cls) -> Sequence[Type[ExtravehicularScenery]]:
        return SceneryRegistry.registry.get(SceneryType.EXTRAVEHICULAR, [])  # type: ignore
