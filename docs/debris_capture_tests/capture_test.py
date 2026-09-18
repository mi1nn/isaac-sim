"""Headless end-to-end check of the MEP capture on `debris_capture_visual` + canadarm3.

Run: ~/isaac-sim/python.sh docs/debris_capture_tests/capture_test.py

Replicates the physics loop of `srb agent manual` (joint position targets ->
scene.write_data_to_sim -> sim.step -> scene.update -> env.update_capture) but drives
the joint targets from a script instead of the keyboard, so it runs without a window.
"""

import sys
import tempfile
import time

from srb.core.app import AppLauncher
from srb.utils.path import SRB_APPS_DIR

launcher = AppLauncher(
    headless=True,
    enable_cameras=True,
    experience=SRB_APPS_DIR.joinpath("srb.headless.rendering.kit"),
)

import gymnasium  # noqa: E402
import torch  # noqa: E402

import srb.tasks  # noqa: E402,F401
from srb.utils.hydra.sim import hydra_task_config  # noqa: E402
from srb.utils.math import quat_apply, subtract_frame_transforms  # noqa: E402

ENV_ID = "srb/debris_capture_visual"
sys.argv = [
    sys.argv[0],
    "env.robot=canadarm3",
    f"hydra.run.dir={tempfile.mkdtemp(prefix='srb_capture_test_')}",
    "hydra.output_subdir=null",
]

RESULTS = {}


def check(name, ok, detail=""):
    RESULTS[name] = ok
    print(f"[TEST] {'PASS' if ok else 'FAIL'} {name} {detail}", flush=True)


@hydra_task_config(task_name=ENV_ID)
def main(env_cfg, agent_cfg=None):
    env = gymnasium.make(ENV_ID, cfg=env_cfg)
    env.reset()
    u = env.unwrapped
    sim, scene = u.sim, u.scene
    robot, mep, cap = u._robot, u._obj, u._capture
    dt = sim.get_physics_dt()
    stage = scene.stage

    ## Static checks
    cyl = stage.GetPrimAtPath("/World/envs/env_0/robot/canadarm3_large_7/capture_cylinder")
    check("capture cylinder prim exists", cyl.IsValid(), str(cyl.GetPath()))
    marker = stage.GetPrimAtPath("/World/envs/env_0/debris/gripper_fixture/Cylinder_01")
    check("MEP marker prim exists", marker.IsValid())
    print("[INFO] marker xform:", [(a.GetName(), a.Get()) for a in marker.GetAttributes() if a.GetName().startswith("xformOp:")])
    print("[INFO] DiscMount active:", stage.GetPrimAtPath("/World/envs/env_0/debris/gripper_fixture/DiscMount").IsActive())
    print("[INFO] gravity:", sim.cfg.gravity)
    mep_mass = mep.root_physx_view.get_masses().sum().item()
    print("[INFO] MEP mass [kg]:", mep_mass)
    check("MEP mass set to ~1000 kg (was ~143,183 kg with density=1000)",
          abs(mep_mass - 1000.0) < 5.0, f"{mep_mass:.1f} kg")
    print(f"[INFO] capture cylinder radius={cap.cfg.radius:.3f} m, distance_threshold={cap.cfg.distance_threshold:.3f} m")
    print("[INFO] MEP marker offset in body frame:", cap._capture_point_mep.tolist())
    print("[INFO] robot capture point in link frame:", cap._capture_point_link.tolist())

    target = robot.data.default_joint_pos.clone()
    arm_ids = robot.find_joints("canadarm3_large_joint_[1-7]")[0]
    link_id = cap._link_body_id

    def step(n=1):
        for _ in range(n):
            robot.set_joint_position_target(target)
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(dt=dt)
            u.update_capture()

    def rel_link_mep():
        # marker centre (the capture point on the MEP) and MEP orientation, both
        # expressed in the arm link frame
        _, q = subtract_frame_transforms(
            robot.data.body_pos_w[:, link_id], robot.data.body_quat_w[:, link_id],
            mep.data.root_pos_w, mep.data.root_quat_w,
        )
        p, _ = subtract_frame_transforms(
            robot.data.body_pos_w[:, link_id], robot.data.body_quat_w[:, link_id],
            cap.mep_capture_pos_w,
        )
        return p, q

    ## 1) Idle: nothing moves
    mep0 = mep.data.root_pos_w.clone()
    step(300)
    d = torch.norm(mep.data.root_pos_w - mep0).item()
    check("MEP at rest while idle (2 s)", d < 1e-3, f"moved {d:.2e} m")
    print("[INFO] initial capture distance:", cap.distance.item(), "state:", cap.state.item())

    ## 2) Arm moves before capture, MEP independent
    mep0 = mep.data.root_pos_w.clone()
    ee0 = cap.robot_capture_pos_w.clone()
    target[:, arm_ids[0]] += 0.05
    step(300)
    target[:, arm_ids[0]] -= 0.05
    step(300)
    ee_moved = torch.norm(cap.robot_capture_pos_w - ee0).item()
    d = torch.norm(mep.data.root_pos_w - mep0).item()
    print(f"[INFO] joint1 +/-2.9deg: capture point max excursion (end) {ee_moved:.3f} m")
    check("MEP independent of arm before capture", d < 1e-3 and cap.state.item() != 2, f"MEP moved {d:.2e} m")

    def ik_step(goal):
        jac = robot.root_physx_view.get_jacobians()
        # fixed-base articulation: the jacobian has no row for the root body
        J = jac[0, link_id - 1, :, arm_ids]
        r = cap.robot_capture_pos_w[0] - robot.data.body_pos_w[0, link_id]
        Jp = J[:3] - torch.linalg.cross(r.unsqueeze(1).expand(3, len(arm_ids)).T, J[3:6].T).T
        err = goal - cap.robot_capture_pos_w[0]
        dist = torch.norm(err)
        v = err / dist * min(0.02, float(dist))
        dq = Jp.T @ torch.linalg.solve(Jp @ Jp.T + 0.05**2 * torch.eye(3, device=Jp.device), v)
        target[0, arm_ids] = robot.data.joint_pos[0, arm_ids] + torch.clamp(dq, -0.002, 0.002) * 5
        step(1)
        return float(dist)

    def approach(max_iters=8000):
        mep_start = mep.data.root_pos_w.clone()
        t0 = time.time()
        for it in range(max_iters):
            if cap.state.item() == 2:
                break
            ik_step(cap.mep_capture_pos_w[0].clone())
        pre = torch.norm(mep.data.root_pos_w - mep_start).item()
        return it, time.time() - t0, pre

    def settle(max_steps=3000):
        target[:] = target.clone()
        for i in range(max_steps // 30):
            step(30)
            if torch.abs(robot.data.joint_pos[0, arm_ids] - target[0, arm_ids]).max() < 0.002 \
               and torch.norm(mep.data.root_lin_vel_w).item() < 1e-3:
                return (i + 1) * 30
        return max_steps

    ## 3a) Approach to a point clearly outside the (widened) capture range and hold:
    ## must NOT capture, so a distant arm cannot trigger it by accident.
    mep_center = cap.mep_capture_pos_w[0].clone()
    to_center = mep_center - cap.robot_capture_pos_w[0]
    world_up = torch.tensor([0.0, 0.0, 1.0], device=to_center.device)
    perp = torch.linalg.cross(to_center, world_up)
    if torch.norm(perp) < 1e-3:
        perp = torch.linalg.cross(to_center, torch.tensor([0.0, 1.0, 0.0], device=to_center.device))
    perp = perp / torch.norm(perp)
    far_goal = mep_center + perp * 1.0  # 1.0 m off-centre, still < approach_distance (2.0 m)
    n = 0
    while n < 4000 and ik_step(far_goal) > 0.03:
        n += 1
    target[:] = robot.data.joint_pos.clone()
    settle()
    d_far = cap.distance.item()
    check("no capture 1.0 m off MEP centre (widened range must not over-capture)",
          cap.state.item() != 2 and d_far > cap.cfg.distance_threshold,
          f"distance {d_far:.3f} m, state {cap.state.item()} (0=IDLE, 1=APPROACH, 2=CAPTURED)")

    ## 3b) Approach the marker with position-only DLS IK on the capture point
    it, wall, pre = approach()
    d_at_capture = cap.distance.item()
    check("CAPTURED reached by approaching the marker", cap.state.item() == 2,
          f"after {it} steps ({it*dt:.1f} s sim, {wall:.1f} s wall), distance {d_at_capture:.3f} m")
    check("capture triggers over the widened range (captured farther than the old 0.3 m threshold)",
          0.35 < d_at_capture <= cap.cfg.distance_threshold + 0.02,
          f"captured at {d_at_capture:.3f} m (threshold {cap.cfg.distance_threshold:.3f} m)")
    check("MEP not pushed before capture (no contact)", pre < 1e-3, f"MEP moved {pre:.2e} m during approach")
    check("capture FixedJoint prim exists", stage.GetPrimAtPath("/World/envs/env_0/capture_joint").IsValid())

    ## 4) Hold, then move the arm: MEP must follow with constant relative pose
    rp_cap, rq_cap = rel_link_mep()
    target[:] = robot.data.joint_pos.clone()
    step(150)
    rp0, rq0 = rel_link_mep()
    snap = torch.norm(rp0 - rp_cap).item()
    check("no snap when the joint is created (marker point, 1 s after capture)", snap < 0.005, f"{snap*1000:.2f} mm")
    mep0 = mep.data.root_pos_w.clone()
    for sign in (+1, -1):
        max_dyn = 0.0
        for _ in range(100):
            target[:, arm_ids[0]] += sign * 0.001
            target[:, arm_ids[3]] += sign * 0.001
            step(3)
            rp, _ = rel_link_mep()
            max_dyn = max(max_dyn, torch.norm(rp - rp0).item())
        n = settle()
        rp, rq = rel_link_mep()
        pos_err = torch.norm(rp - rp0).item()
        ang_err = 2 * torch.acos(torch.clamp(torch.abs((rq * rq0).sum()), max=1.0)).item()
        mep_moved = torch.norm(mep.data.root_pos_w - mep0).item()
        print(f"[INFO] sweep {sign:+d} (joint1,joint4 {sign*5.7:+.1f} deg): MEP displacement from capture pose {mep_moved:.3f} m, "
              f"rel pos err max-while-moving {max_dyn*1000:.2f} mm / settled {pos_err*1000:.2f} mm, "
              f"rel ang err {ang_err*57.2958:.3f} deg, settle {n*dt:.1f} s", flush=True)
        if sign == +1:
            check("MEP moves with the arm after capture", mep_moved > 0.1, f"{mep_moved:.3f} m")
        check(f"relative transform held after sweep {sign:+d} (marker point settled <5 mm, <0.5 deg)",
              pos_err < 0.005 and ang_err < 0.5 / 57.2958, f"{pos_err*1000:.2f} mm / {ang_err*57.2958:.3f} deg")

    ## 5) Release, then retreat 1 m straight back: the MEP must not follow
    u.release_capture()
    check("joint removed on release", not stage.GetPrimAtPath("/World/envs/env_0/capture_joint").IsValid())
    # A free body's COM moves in a straight line; its root frame does not if it spins
    v_rel = mep.data.root_com_lin_vel_w[0].clone()
    mep0 = mep.data.root_com_pos_w[0].clone()
    ee0 = cap.robot_capture_pos_w[0].clone()
    away = ee0 - cap.mep_capture_pos_w[0]
    goal = ee0 + away / torch.norm(away) * 1.0
    n = 0
    while n < 3000 and ik_step(goal) > 0.02:
        n += 1
    target[:] = robot.data.joint_pos.clone()
    step(150)
    t_el = (n + 151) * dt
    mep_d = mep.data.root_com_pos_w[0] - mep0
    pred = v_rel * t_el
    arm_d = torch.norm(cap.robot_capture_pos_w[0] - ee0).item()
    print(f"[INFO] after release: arm retreated {arm_d:.3f} m in {t_el:.1f} s; MEP moved {torch.norm(mep_d).item():.4f} m "
          f"(ballistic from release velocity {torch.norm(pred).item():.4f} m); distance now {cap.distance.item():.3f} m", flush=True)
    check("MEP stays free after release (does not follow the arm)",
          arm_d > 0.5 and torch.norm(mep_d - pred).item() < 0.01 and cap.state.item() != 2,
          f"deviation from ballistic {torch.norm(mep_d - pred).item()*1000:.2f} mm")
    check("capture re-armed after moving away", bool(cap._armed[0]))

    ## 6) Capture again, then reset while captured
    it, wall, pre = approach()
    check("re-capture after release", cap.state.item() == 2, f"after {it} steps")
    env.reset()
    target[:] = robot.data.default_joint_pos.clone()
    step(300)
    d_mep = torch.norm(mep.data.root_pos_w - mep.data.default_root_state[:, :3] - scene.env_origins).item()
    v_mep = torch.norm(mep.data.root_lin_vel_w).item()
    jerr = torch.abs(robot.data.joint_pos - robot.data.default_joint_pos).max().item()
    check("reset while captured: joint removed, state IDLE, MEP/arm back at rest",
          cap.state.item() == 0 and d_mep < 1e-3 and v_mep < 1e-3 and jerr < 1e-3
          and not stage.GetPrimAtPath("/World/envs/env_0/capture_joint").IsValid(),
          f"MEP from init {d_mep:.2e} m, |v| {v_mep:.2e}, arm joint err {jerr:.2e} rad")

    print("[TEST] SUMMARY", sum(RESULTS.values()), "/", len(RESULTS), flush=True)
    env.close()


main()
launcher.app.close()
