"""TEST 2 -- Linear drift intercept: vision tracking, prediction, approach and capture.

    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_dynamic_intercept.py -s
"""

import pytest
from phase1_scenarios import DYNAMIC_RUNS, assert_checks, scenario_result


@pytest.mark.parametrize("run", DYNAMIC_RUNS)
def test_dynamic_intercept(run):
    r = scenario_result(run)
    cap = r["metrics"].get("capture") or {}
    if cap:
        print(f"captured at {cap['time_s']:.2f} s, relative velocity {cap['gt']['rel_vel']:.4f} m/s (GT)")
    assert_checks(r, "[TEST2]", f"TEST 2 [{run}]")
