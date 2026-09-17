"""Isaac-only bounded stability/open-close probe; no external ROS commands.

Run with Isaac Python; remaining arguments are ordinary `srb agent zero` options.
SRB_SMOKE_STEPS (default 450) and SRB_SMOKE_OUTPUT select duration/output.
"""
import json
import os
import sys
from pathlib import Path

import srb.__main__ as cli


def probe(env, sim_app, **kwargs):
    import torch
    from srb.utils.math import combine_frame_transforms, subtract_frame_transforms

    u = env.unwrapped
    u.cfg.truncate_episodes = False
    robot, gripper = u.scene["robot"], u.scene["end_effector"]
    ik = u.action_manager._terms["robot/differential_inverse_kinematics"]
    wrist = robot.find_bodies("canadarm3_large_7")[0][0]
    assembly = u.cfg.joint_assemblies["end_effector"]
    offset = torch.tensor([assembly.fixed_joint_offset], device=u.device)
    rotation = torch.tensor([assembly.fixed_joint_orient], device=u.device)
    actions = torch.zeros(env.action_space.shape, device=u.device)
    initial = robot.data.joint_pos.clone()
    samples = []
    steps = int(os.environ.get("SRB_SMOKE_STEPS", "450"))
    print("SMOKE_CONFIG", u.cfg.seed, u.step_dt, ik.cfg.body_offset, flush=True)
    print("SMOKE_MASSES", gripper.root_physx_view.get_masses().tolist(), flush=True)
    print("SMOKE_INERTIAS", gripper.root_physx_view.get_inertias().tolist(), flush=True)
    try:
        with torch.inference_mode():
            for i in range(steps):
                actions[:, -1] = -1.0 if steps // 3 <= i < 2 * steps // 3 else 1.0
                env.step(actions)
                expected_pos, expected_quat = combine_frame_transforms(
                    robot.data.body_pos_w[:, wrist], robot.data.body_quat_w[:, wrist],
                    offset, rotation,
                )
                _, mount_q = subtract_frame_transforms(
                    expected_pos, expected_quat, gripper.data.root_pos_w, gripper.data.root_quat_w,
                )
                record = dict(
                    step=i, sim_time=(i+1)*u.step_dt,
                    joint_drift=float((robot.data.joint_pos-initial).abs().max()),
                    arm_speed=float(robot.data.joint_vel.abs().max()),
                    finger_speed=float(gripper.data.joint_vel.abs().max()),
                    mount_error=float((expected_pos-gripper.data.root_pos_w).norm(dim=-1).max()),
                    mount_angle=float(2*torch.acos(mount_q[:, 0].abs().clamp(max=1)).max()),
                    fingers=gripper.data.joint_pos[0].tolist(),
                    mep=u.scene["mep"].data.root_pos_w[0].tolist(),
                    satellite=u.scene["satellite"].data.root_pos_w[0].tolist(),
                )
                samples.append(record)
                if i % 50 == 0 or i == steps-1:
                    print("SMOKE", json.dumps(record), flush=True)
                if not torch.isfinite(robot.data.joint_pos).all() or not torch.isfinite(gripper.data.joint_pos).all():
                    raise RuntimeError("Non-finite articulation state")
        Path(os.environ.get("SRB_SMOKE_OUTPUT", "/tmp/mep_smoke.json")).write_text(json.dumps(samples, indent=2))
    finally:
        env.close()
        sim_app.close()


cli.zero_agent = probe
sys.argv = ["srb", "agent", "zero", *sys.argv[1:]]
cli.main()
