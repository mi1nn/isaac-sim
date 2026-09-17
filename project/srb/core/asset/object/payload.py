from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Payload(Object, object_entrypoint=ObjectType.PAYLOAD):
    @classmethod
    def object_registry(cls) -> Sequence[Type[Payload]]:
        return ObjectRegistry.registry.get(ObjectType.PAYLOAD, [])  # type: ignore
