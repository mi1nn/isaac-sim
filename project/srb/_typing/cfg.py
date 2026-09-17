from typing import TYPE_CHECKING, TypeAlias

if TYPE_CHECKING:
    from srb.core.env import (
        BaseEnvCfg,
        DirectEnv,
        DirectEnvCfg,
        ManagedEnv,
        ManagedEnvCfg,
    )

AnyEnv: TypeAlias = "DirectEnv | ManagedEnv"
AnyEnvCfg: TypeAlias = "BaseEnvCfg | DirectEnvCfg | ManagedEnvCfg"
