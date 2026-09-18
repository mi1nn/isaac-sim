"""Reproduce Ctrl+Z on `srb agent ...` inside a real pty with an interactive bash.

Usage: python3 docs/debris_capture_tests/ctrlz_repro.py <scenario> <log>
Runs `srb agent zero --headless ...` by default so no window opens (override: SRB_CMD).
Scenarios:
  fg      : start, Ctrl+Z, show state, `fg`, let it run, Ctrl+C
  rerun   : start, Ctrl+Z, start the same command again (2nd instance), Ctrl+C both
  killjob : start, Ctrl+Z, `kill %1`
  exit    : start, Ctrl+Z, `exit` the shell twice (bash warns first)
"""

import os
import pty
import re
import select
import subprocess
import sys
import time

scenario, log_path = sys.argv[1], sys.argv[2]
CMD = os.environ.get(
    "SRB_CMD",
    "srb agent zero --headless --env debris_capture_visual "
    '--kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" '
    "env.robot=canadarm3",
)
READY = os.environ.get("SRB_READY", r"\[CAPTURE\] State: IDLE")

log = open(log_path, "w")
buf = ""


def note(msg):
    line = f"\n##### [{time.strftime('%H:%M:%S')}] {msg}\n"
    log.write(line)
    log.flush()
    print(line.strip(), flush=True)


pid, fd = pty.fork()
if pid == 0:
    os.environ["PS1"] = "PROMPT$ "
    os.environ["PYTHONUNBUFFERED"] = "1"
    os.execvp("bash", ["bash", "--norc", "--noprofile", "-i"])


def pump(timeout):
    global buf
    end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                data = os.read(fd, 65536).decode(errors="replace")
            except OSError:
                return
            buf += data
            log.write(data)
            log.flush()


def wait_for(pattern, timeout, start=0):
    end = time.time() + timeout
    while time.time() < end:
        if re.search(pattern, buf[start:]):
            return True
        pump(1.0)
    return False


def send(s):
    os.write(fd, s.encode())


def ps_tree():
    out = subprocess.run(
        "ps -eo pid,ppid,pgid,tpgid,stat,etime,cmd | grep -E 'kit/python|python.sh|srb agent' | grep -v grep | cut -c1-170",
        shell=True, capture_output=True, text=True,
    ).stdout
    gpu = subprocess.run(
        "nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader",
        shell=True, capture_output=True, text=True,
    ).stdout
    note("ps:\n" + out + "gpu apps:\n" + gpu)


pump(2)
mark = len(buf)
send(CMD + "\n")
note(f"started: {CMD}")
ok = wait_for(READY, 600, mark)
note(f"ready={ok}")
pump(5)
ps_tree()

send("\x1a")  # Ctrl+Z
note("sent Ctrl+Z (0x1a)")
pump(5)
ps_tree()

if scenario == "fg":
    pump(20)
    note("stopped for 20 s; sending `fg`")
    mark = len(buf)
    send("fg\n")
    pump(30)
    ps_tree()
    send("\x03")  # Ctrl+C
    note("sent Ctrl+C")
    wait_for(r"PROMPT\$ $", 120, mark)
    pump(3)
    ps_tree()
elif scenario == "rerun":
    mark = len(buf)
    send(CMD + "\n")
    note("started a 2nd instance while the 1st is stopped")
    ok = wait_for(READY + r"|Traceback|rror", 600, mark)
    note(f"2nd instance ready-or-error={ok}")
    pump(10)
    ps_tree()
    send("\x03")
    note("sent Ctrl+C to 2nd instance")
    wait_for(r"PROMPT\$ $", 120, mark)
    pump(3)
    send("jobs -l\n")
    pump(3)
    ps_tree()
    send("kill %1\n")
    note("kill %1 (cleanup of the stopped 1st instance)")
    pump(30)
    send("jobs -l\n")
    pump(3)
    ps_tree()
elif scenario == "killjob":
    mark = len(buf)
    send("jobs -l\n")
    pump(2)
    send("kill %1\n")
    note("sent `kill %1`")
    pump(45)
    send("jobs -l\n")
    pump(3)
    ps_tree()
elif scenario == "exit":
    send("exit\n")
    note("sent `exit` (1st)")
    pump(3)
    send("exit\n")
    note("sent `exit` (2nd)")
    pump(45)
    ps_tree()

note("done")
try:
    os.kill(pid, 9)
except ProcessLookupError:
    pass
