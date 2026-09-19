"""Shared runner for the MRV Phase 1 simulation tests.

Each run (`static`, `dynamic` at 0.01 m/s, `dynamic_v002` at 0.02 m/s) runs once per pytest session in its own Isaac Sim
process (`scripts/vision_capture.py --headless`). Set `PHASE1_REUSE=1` to judge the
results of the latest run in `logs/vision_capture/` without simulating again.
"""

import json
import os
import subprocess
from pathlib import Path

import pytest

PROJECT = Path(__file__).resolve().parents[1]
OUT_DIR = PROJECT / "logs" / "vision_capture"
PYTHON = os.environ.get("ISAAC_PYTHON", os.path.expanduser("~/isaac-sim/python.sh"))
_cache: dict = {}
# run name -> (scenario, extra arguments); same runs as scripts/run_phase1_tests.py
RUNS = {
    "static": ("static", []),
    "dynamic": ("dynamic", ["--set", "mep.linear_velocity_mps=0.01"]),
    "dynamic_v002": ("dynamic", ["--set", "mep.linear_velocity_mps=0.02"]),
}
DYNAMIC_RUNS = ["dynamic", "dynamic_v002"]


def scenario_result(name: str) -> dict:
    if name in _cache:
        return _cache[name]
    path = OUT_DIR / f"{name}_result.json"
    if os.environ.get("PHASE1_REUSE") != "1":
        if not Path(PYTHON).exists():
            pytest.skip(f"Isaac Sim Python not found at {PYTHON}")
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        if path.exists():
            path.unlink()
        with open(OUT_DIR / f"{name}_console.log", "w") as log:
            scenario, extra = RUNS[name]
            subprocess.run([PYTHON, str(PROJECT / "scripts" / "vision_capture.py"), "--scenario", scenario, "--tag", name, "--headless", *extra],
                           stdout=log, stderr=subprocess.STDOUT, cwd=PROJECT.parent, timeout=3600)
    if not path.exists():
        pytest.fail(f"{name} scenario produced no result ({OUT_DIR / (name + '_console.log')})")
    _cache[name] = json.loads(path.read_text())
    return _cache[name]


def assert_checks(result: dict, prefix: str, label: str):
    checks = {k: v for k, v in result["checks"].items() if k.startswith(prefix) or not k.startswith("[")}
    failed = [f"- {k}: {v.get('detail', '')}" for k, v in checks.items() if not v["pass"]]
    if result.get("failure"):
        failed.append(f"- final state {result['final_state']}: {result['failure']}")
    print(f"{label}: {'PASS' if not failed else 'FAIL'}")
    assert not failed, f"{label}: FAIL\nReason:\n" + "\n".join(failed)
