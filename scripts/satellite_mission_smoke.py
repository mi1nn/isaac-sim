"""Bounded P0 physics checks using the normal SRB CLI and real assets.

Isaac Python scripts/satellite_mission_smoke.py --env satellite_mission --headless
    env.num_envs=1 env.seed=0 env.debug_vis=true
SRB_P0_OUTPUT selects the JSON report. A failing check exits nonzero.
"""
import json
import os
import sys
import time
from pathlib import Path
import srb.__main__ as cli


def probe(env, sim_app, **kwargs):
    import torch
    from srb.utils.math import combine_frame_transforms
    u = env.unwrapped
    robot, grip, mep = (u.scene[name] for name in ("robot", "end_effector", "mep"))
    report = {"seed": u.cfg.seed, "num_envs": u.num_envs, "step_dt": u.step_dt,
              "arm_damping": u.cfg.arm_damping, "arm_velocity_limit": u.cfg.arm_velocity_limit,
              "ik_action_scale": u.cfg.ik_action_scale, "checks": {}}
    output = Path(os.environ.get("SRB_P0_OUTPUT", "logs/satellite_mission/p0.json"))
    output.parent.mkdir(parents=True, exist_ok=True)
    action = torch.zeros(env.action_space.shape, device=u.device)
    action[:, -1] = 1.0
    wrist = robot.find_bodies("canadarm3_large_7")[0][0]
    assembly = u.cfg.joint_assemblies["end_effector"]
    offset = torch.tensor([assembly.fixed_joint_offset], device=u.device).repeat(u.num_envs, 1)
    rotation = torch.tensor([assembly.fixed_joint_orient], device=u.device).repeat(u.num_envs, 1)
    max_mount_error = 0.0
    started = time.monotonic()

    def step(count):
        nonlocal max_mount_error
        for _ in range(count):
            obs, _, terminated, truncated, _ = env.step(action)
            assert not terminated.any() and not truncated.any(), "Unexpected P0 reset"
            for body in (robot, grip, mep):
                assert torch.isfinite(body.data.root_state_w).all(), "Non-finite body state"
            assert torch.isfinite(robot.data.joint_pos).all() and torch.isfinite(grip.data.joint_pos).all()
            expected, _ = combine_frame_transforms(robot.data.body_pos_w[:, wrist], robot.data.body_quat_w[:, wrist], offset, rotation)
            max_mount_error = max(max_mount_error, float((expected-grip.data.root_pos_w).norm(dim=-1).max()))

    def check(name, passed, **metrics):
        report["checks"][name] = {"passed": bool(passed), **metrics}
        print("P0_CHECK", name, json.dumps(report["checks"][name]), flush=True)

    try:
        with torch.inference_mode():
            initial = robot.data.joint_pos.clone()
            initial_mep = mep.data.root_pos_w.clone()
            step(100)
            drift = float((robot.data.joint_pos-initial).abs().max())
            mep_drift = float((mep.data.root_pos_w-initial_mep).norm(dim=-1).max())
            check("T01_idle", drift < 0.01 and mep_drift < 0.001, joint_drift_rad=drift, mep_drift_m=mep_drift)
            if "cam_wrist" in u.scene.sensors:
                from PIL import Image
                for name in ("cam_base", "cam_wrist"):
                    camera = u.scene[name]
                    rgb = camera.data.output["rgb"][0, :, :, :3].cpu().numpy()
                    Image.fromarray(rgb).save(output.with_name(name + ".png"))
                    check("T06_" + name, rgb.shape[:2] == (240, 320) and rgb.std() > 1, shape=list(rgb.shape), std=float(rgb.std()))
            open_pos = grip.data.joint_pos.clone()
            action[:, -1] = -1.0
            step(200)
            closed_pos = grip.data.joint_pos.clone()
            action[:, -1] = 1.0
            step(200)
            travel = float((closed_pos-open_pos).min())
            reopen_error = float((grip.data.joint_pos-open_pos).abs().max())
            check("T02_gripper", travel > 0.7 and reopen_error < 0.1, min_travel_rad=travel, reopen_error_rad=reopen_error)
            speeds = {}
            for name, damping in (("baseline", 25000.0), ("updated", u.cfg.arm_damping)):
                env.reset()
                robot.write_joint_damping_to_sim(torch.full_like(robot.data.joint_pos, damping))
                action.zero_(); action[:, -1] = 1.0
                step(25)
                start = u._tf_end_effector.data.target_pos_source[:, 0].clone()
                action[:, 0] = 0.2
                step(100)
                delta = u._tf_end_effector.data.target_pos_source[:, 0] - start
                speeds[name] = float(delta[:, 0].mean()) / (100*u.step_dt)
            check("T03_speed", speeds["updated"] > speeds["baseline"] * 1.5 and speeds["baseline"] > 0, **speeds)
            # A collision probe, not a grasp claim: put the handle inside the open
            # gripper, close, and require filtered physical contact plus motion.
            for contact_env in range(u.num_envs):
                env.reset()
                action.zero_(); action[:, -1] = 1.0
                step(25)
                pose = mep.data.root_state_w[:, :7].clone()
                pose[contact_env, :3] = u._tf_end_effector.data.target_pos_w[contact_env, 0]
                mep.write_root_pose_to_sim(pose)
                mep.write_root_velocity_to_sim(torch.zeros_like(mep.data.root_vel_w))
                peak_contact = 0.0
                other_env_contact = 0.0
                action[:, -1] = -1.0
                for _ in range(75):
                    step(1)
                    forces = u._contacts_end_effector.data.force_matrix_w
                    per_env_contact = forces.norm(dim=-1).sum(dim=(-1,-2))
                    peak_contact = max(peak_contact, float(per_env_contact[contact_env]))
                    others = torch.arange(u.num_envs, device=u.device) != contact_env
                    if others.any():
                        other_env_contact = max(other_env_contact, float(per_env_contact[others].max()))
                displacement = float((mep.data.root_pos_w-pose[:, :3]).norm(dim=-1).max())
                check(f"T04_collision_env{contact_env}", peak_contact > 0.01 and displacement > 0.00001 and other_env_contact < 0.01,
                      peak_contact_n=peak_contact, displacement_m=displacement, other_env_contact_n=other_env_contact)
            env.reset()
            action.zero_(); action[:, -1] = 1.0
            step(10)
            reset_error = float((mep.data.root_pos_w-initial_mep).norm(dim=-1).max())
            check("T05_reset_mount", reset_error < 0.001 and max_mount_error < 0.02, reset_error_m=reset_error, max_mount_error_m=max_mount_error)
            report["wall_seconds"] = time.monotonic()-started
            report["passed"] = all(v["passed"] for v in report["checks"].values())
            assert report["passed"], "P0 checks failed; see JSON report"
    except Exception as exc:
        import traceback
        report["passed"] = False
        report["error"] = repr(exc)
        traceback.print_exc(file=sys.stdout)
        sys.stdout.flush()
        raise
    finally:
        output.write_text(json.dumps(report, indent=2))
        print("P0_REPORT", output, flush=True)
        env.close()
        sim_app.close()


if __name__ == "__main__":
    cli.zero_agent = probe
    sys.argv = ["srb", "agent", "zero", *sys.argv[1:]]
    cli.main()
