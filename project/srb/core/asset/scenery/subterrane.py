from __future__ import annotations

from typing import ClassVar, Sequence, Type

from srb.core.asset.scenery.scenery import Scenery, SceneryRegistry
from srb.core.asset.scenery.scenery_type import SceneryType
from srb.core.domain import Domain


class Subterrane(Scenery, scenery_entrypoint=SceneryType.SUBTERRANE):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = (
        Domain.ASTEROID,
        Domain.EARTH,
        Domain.MARS,
        Domain.MOON,
    )

    @classmethod
    def scenery_registry(cls) -> Sequence[Type[Subterrane]]:
        return SceneryRegistry.registry.get(SceneryType.SUBTERRANE, [])  # type: ignore
