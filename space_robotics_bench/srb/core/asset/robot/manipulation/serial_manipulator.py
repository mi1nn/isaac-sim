from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset.robot.manipulation.manipulator import (
    Manipulator,
    ManipulatorRegistry,
)
from srb.core.asset.robot.manipulation.manipulator_type import ManipulatorType


class SerialManipulator(Manipulator, manipulator_entrypoint=ManipulatorType.SERIAL):
    @classmethod
    def manipulator_registry(cls) -> Sequence[Type[SerialManipulator]]:
        return ManipulatorRegistry.registry.get(ManipulatorType.SERIAL, [])  # type: ignore
