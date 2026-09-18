"""Signal handling that keeps an Isaac Sim session from outliving its terminal.

`srb` runs as `python.sh` (a bash wrapper that does not `exec`) -> `python3`, so a
terminal job is two processes. Observed with `srb agent ...`:

- Ctrl+Z suspends both (SIGTSTP); `fg` resumes cleanly.
- `kill %<job>` (SIGTERM) or closing the terminal (SIGHUP) while the job is suspended
  kills the bash wrapper right away, but `python3` survives as an orphan re-parented
  to `systemd --user`, keeps its GPU memory and never exits: Isaac Lab's SIGTERM
  handler calls `SimulationApp.close()` from inside a running physics step, which
  stalls after "Replicator Stop" (PhysX logs "... not allowed while simulation is
  running"). The job is gone from the shell, so the stale process is invisible.
- SIGINT (Ctrl+C) goes through `SimulationApp`'s own handler (`app.shutdown()` +
  `sys.exit`) and exits within seconds even from that stalled state.
"""

import os
import signal
import sys
from pathlib import Path
from typing import List, Tuple

from srb.utils import logging


def install_session_signal_handlers():
    """Route SIGTERM/SIGHUP to the Ctrl+C shutdown path and explain Ctrl+Z.

    Must be called after Isaac Sim is launched, since both `SimulationApp` (SIGINT)
    and Isaac Lab's `AppLauncher` (SIGTERM) install their handlers during startup.
    Ctrl+Z keeps its default meaning: the process is still suspended.
    """
    sigint_handler = signal.getsignal(signal.SIGINT)
    if callable(sigint_handler):
        for sig in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, sigint_handler)
    signal.signal(signal.SIGTSTP, _on_sigtstp)


def _on_sigtstp(signum, frame):
    pid = os.getpid()
    print(
        f"\n[srb] Suspended by Ctrl+Z (SIGTSTP): Isaac Sim (PID {pid}) and its window "
        "are frozen but still hold GPU memory.\n"
        "[srb]   resume: fg    |    quit: fg, then Ctrl+C (or: kill %<job>)",
        file=sys.stderr,
        flush=True,
    )
    # Suspend for real with the default action, then re-arm once continued
    signal.signal(signal.SIGTSTP, signal.SIG_DFL)
    os.kill(pid, signal.SIGTSTP)
    signal.signal(signal.SIGTSTP, _on_sigtstp)


def warn_about_stale_sessions():
    """Report `srb` processes of this user that are suspended or orphaned.

    Nothing is killed: other sessions (e.g. a teammate's) may be legitimate.
    """
    stale = _find_stale_sessions()
    if not stale:
        return
    lines = "\n".join(f"    PID {pid:>7}  [{reason}]  {cmd}" for pid, reason, cmd in stale)
    logging.warning(
        "Found other Space Robotics Bench processes that are suspended or orphaned "
        "(their shell job has ended). They still hold GPU/CPU memory:\n"
        f"{lines}\n"
        "  If they are yours and no longer needed, stop them with: kill -INT <PID>"
    )


def _find_stale_sessions() -> List[Tuple[int, str, str]]:
    uid = os.getuid()
    own = {os.getpid(), os.getppid()}
    stale = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) in own:
            continue
        try:
            if proc.stat().st_uid != uid:
                continue
            cmdline = proc.joinpath("cmdline").read_bytes().replace(b"\0", b" ")
            cmd = cmdline.decode(errors="replace").strip()
            if "srb" not in cmd or "python3" not in cmd.split(" ", 1)[0]:
                continue
            # /proc/<pid>/stat: "pid (comm) state ppid pgrp session tty_nr ..."
            fields = proc.joinpath("stat").read_text().rsplit(")", 1)[1].split()
        except (OSError, IndexError):
            continue
        state, ppid, tty_nr = fields[0], int(fields[1]), int(fields[4])
        if state == "T":
            stale.append((int(proc.name), "suspended", cmd))
        elif tty_nr == 0 and not _is_python_sh(ppid):
            # Its `python.sh` wrapper is gone: the shell job it belonged to has ended
            stale.append((int(proc.name), "orphaned", cmd))
    return stale


def _is_python_sh(pid: int) -> bool:
    try:
        return b"python.sh" in Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return False
