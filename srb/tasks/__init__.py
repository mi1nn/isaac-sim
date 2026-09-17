from srb.utils import logging
from srb.utils.importer import import_recursively
from srb.utils.isaacsim import is_isaacsim_initialized
from srb.utils.registry import get_srb_tasks

if is_isaacsim_initialized():
    import_recursively(__name__)
    logging.debug(
        f'Recursively imported Space Robotics Bench module "{__name__}" ({len(get_srb_tasks())} registered tasks)'
    )
else:
    raise RuntimeError(
        "Tasks of the Space Robotics Bench cannot be registered because Isaac Sim is not initialized. "
        f'Please import the "{__name__}" module after starting the Omniverse simulation app.'
    )
