from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class InterfaceBase:
    def __init__(self, env: "AnyEnv", *args, **kwargs):
        pass

    def update(self, *args, **kwargs):
        raise NotImplementedError
