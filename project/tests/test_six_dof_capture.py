"""6-DoF free-floating MEP (XYZ drift + combined roll/pitch/yaw rate): vision tracking,
6-DoF prediction, capture, holding. Runs `vision_capture.py --set mep.motion_mode=six_dof`.

    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_six_dof_capture.py -s
"""

from phase1_scenarios import assert_checks, scenario_result


def test_six_dof_setup():
    r = scenario_result("six_dof")
    assert r["metrics"]["setup"]["motion_mode"] == "six_dof"
    assert_checks(r, "[6DOF] Angular velocity is a world-frame vector", "6-DoF setup")


def test_six_dof_prediction():
    r = scenario_result("six_dof")
    w = r["metrics"].get("angular_velocity_error") or {}
    p = r["metrics"].get("prediction_error") or {}
    print(f"angular velocity error mean {w.get('error_mean_rad_s')} rad/s, orientation prediction error mean {p.get('orientation_mean_deg')} deg")
    assert_checks(r, "[6DOF]", "6-DoF estimation / prediction")


def test_six_dof_capture():
    r = scenario_result("six_dof")
    assert_checks(r, "[TEST2]", "6-DoF capture")


def test_six_dof_holding():
    r = scenario_result("six_dof")
    assert r["final_state"] == "SUCCESS"
    assert_checks(r, "[TEST3]", "6-DoF holding")
