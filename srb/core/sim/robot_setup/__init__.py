try:
    import isaacsim.robot_setup.assembler as _  # noqa: F401
except ImportError:
    from isaacsim.core.utils.extensions import enable_extension

    assert enable_extension("isaacsim.robot_setup.assembler")

from .assembled_bodies import AssembledBodies  # noqa: F401
from .assembled_robot import AssembledRobot  # noqa: F401
from .robot_assembler import RobotAssembler, RobotAssemblerCfg  # noqa: F401
