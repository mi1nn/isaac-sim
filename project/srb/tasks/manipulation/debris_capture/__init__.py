from srb.utils.registry import register_srb_tasks

from .docking import DockingTask, DockingTaskCfg
from .task import Task, TaskCfg
from .task_visual import VisualTask, VisualTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_visual": {
            "entry_point": VisualTask,
            "task_cfg": VisualTaskCfg,
        },
        f"{BASE_TASK_NAME}_docking": {
            "entry_point": DockingTask,
            "task_cfg": DockingTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
