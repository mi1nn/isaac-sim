"""TEST 3 -- Holding & physics stability after the capture (same run as TEST 2).

    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_holding_stability.py -s
"""

import pytest
from phase1_scenarios import DYNAMIC_RUNS, assert_checks, scenario_result


@pytest.mark.parametrize("run", DYNAMIC_RUNS)
def test_holding_stability(run):
    r = scenario_result(run)
    h = r["metrics"].get("holding") or {}
    if h:
        print(f"held {h.get('duration_s', 0):.2f} s, max MEP-EE drift {h.get('max_drift_mm', float('nan')):.3f} mm")
    assert r["final_state"] == "SUCCESS"
    assert_checks(r, "[TEST3]", f"TEST 3 [{run}]")
