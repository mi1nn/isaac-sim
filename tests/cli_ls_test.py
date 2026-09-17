import subprocess

import pytest

from srb.utils import logging
from srb.utils.isaacsim import get_isaacsim_python


@pytest.mark.order(2)
def test_cli_ls():
    cmd = (
        get_isaacsim_python(),
        "-m",
        "srb",
        "ls",
        "--show_hidden",
    )

    try:
        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )

        if process.wait(timeout=600.0) != 0:
            logging.critical(f"Failed command: {' '.join(cmd)}")
            stdout, stderr = process.communicate()
            pytest.fail(f"Process failed\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}")
    except Exception as e:
        pytest.fail(f"Failed to start process\nException: {e}")
