import subprocess
from importlib.util import find_spec
from typing import Iterable, Literal

from srb.utils import logging


def terminate_process(
    cmd: str | Iterable[str],
    process: subprocess.Popen | None = None,
    signal: Literal[
        "INT",
        "QUIT",
        "ABRT",
        "KILL",
        "TERM",
        "STOP",
    ] = "TERM",
):
    terminate_cmd = (
        "pkill",
        "--signal",
        signal,
        "--full",
        " ".join(cmd) if isinstance(cmd, Iterable) else cmd,
    )

    try:
        subprocess.run(terminate_cmd, check=False)

        if not process or find_spec("psutil") is None:
            return
        import psutil

        if not psutil.pid_exists(process.pid):
            return

        parent = psutil.Process(process.pid)
        for child in parent.children(recursive=True):
            if child.is_running():
                child.terminate()
        if parent.is_running():
            parent.terminate()

        for child in parent.children(recursive=True):
            if child.is_running():
                child.kill()
        if parent.is_running():
            parent.kill()
    except Exception as e:
        logging.error(f"Failed to terminate process: {e}")
    finally:
        subprocess.run(terminate_cmd, check=False)
