"""TEST 1 -- Static accuracy: stationary MEP, AprilTag -> PnP -> Cylinder_01 vs ground truth.

    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_static_accuracy.py -s
"""

from phase1_scenarios import assert_checks, scenario_result


def test_static_accuracy():
    r = scenario_result("static")
    for s in r["metrics"].get("static", []):
        print(f"standoff {s['standoff_m']:.2f} m: position {s['position_error_mm_max']:.3f} mm, angle {s['angle_error_deg_max']:.4f} deg (max)")
    assert r["final_state"] == "SUCCESS"
    assert_checks(r, "[TEST1]", "TEST 1")
