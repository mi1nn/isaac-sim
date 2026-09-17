from enum import Enum, auto

from typing_extensions import Self


class ManipulatorType(str, Enum):
    SERIAL = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )
