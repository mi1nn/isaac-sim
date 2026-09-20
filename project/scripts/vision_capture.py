#!/usr/bin/env python3
"""MRV: AprilTag vision capture of a free-floating 3 t MEP (linear drift or 6-DoF).

Must run with the Isaac Sim Python (the same interpreter `srb` uses):

    cd ~/space_robotics_bench
    # Test 1 -- static pose accuracy (MEP at rest)
    ~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario static --headless
    # Test 2 + 3 -- linear drift intercept, capture, 10 s holding, slow retreat
    ~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --headless
    # 6-DoF -- XYZ drift + combined roll/pitch/yaw rate (mep.angular_velocity_rad_s)
    ~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --headless --tag six_dof --set mep.motion_mode=six_dof
    # GUI (debug draw + camera overlay images); after a success the simulation keeps
    # running with the MEP held until the window is closed (--exit_when_done to quit)
    ~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic

Config: `project/config/vision_capture.yaml`; override any value with
`--set section.key=value` (e.g. `--set mep.linear_velocity_mps=0.02`).
Outputs: `project/logs/vision_capture/<tag>_{result.json,metrics.csv,overlay/}` (tag = --tag or the scenario).
Exit code 0 only if every check of the scenario passed.
"""

import argparse
import os
import sys
import tempfile
from datetime import datetime
from pathlib import Path


def parse_args():
    isaac_path = os.environ.get("ISAAC_PATH", os.path.expanduser("~/isaac-sim"))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", choices=("static", "dynamic"), default="dynamic")
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim window")
    parser.add_argument("--config", type=str, default=None, help="Vision config YAML (default: project/config/vision_capture.yaml)")
    parser.add_argument("--set", dest="sets", action="append", default=[], metavar="SECTION.KEY=VALUE", help="Override a config value (repeatable)")
    parser.add_argument("--out_dir", type=str, default=None, help="Output directory (default: project/logs/vision_capture)")
    parser.add_argument("--tag", type=str, default=None, help="Run name for the output files (default: the scenario name)")
    parser.add_argument("--exit_when_done", action="store_true",
                        help="GUI: close the app when the scenario ends (default: after a success the simulation keeps running until the window is closed)")
    parser.add_argument("--dock", action="store_true",
                        help="Run the Ares1 probe -> satellite thruster docking phase after the capture (config `docking:`)")
    parser.add_argument("--dock_only", action="store_true",
                        help="--dock, but skip the capture: attach the MEP at its nominal grasp pose and dock straight away")
    parser.add_argument("--start_yaw_deg", type=float, default=None, metavar="DEG",
                        help="Start the arm swung DEG degrees in azimuth (about its base axis) so it has to search for the MEP")
    parser.add_argument("--ros", action="store_true",
                        help="Enable the ROS 2 interface (telemetry topics + cmd/start, cmd/abort, cmd/capture_enable; see config `ros:`)")
    parser.add_argument("--ros_wait_start", action="store_true", help="With --ros: hold the arm until a cmd/start message arrives")
    parser.add_argument("--start_paused", action="store_true", help="Wait for Play in the toolbar before starting")
    parser.add_argument("--kit_args", type=str, default=None,
                        help=f'Extra Kit arguments. GUI default: "--ext-folder {isaac_path}/apps --enable isaacsim.exp.base"')
    parser.add_argument("overrides", nargs="*", help="Extra Hydra overrides for the env config")
    args = parser.parse_args()
    if args.kit_args is None and not args.headless:
        args.kit_args = f"--ext-folder {isaac_path}/apps --enable isaacsim.exp.base"
    return args


def ensure_ros2_env(args):
    """The Isaac Sim ROS 2 bridge libraries must be on LD_LIBRARY_PATH before the process
    starts (setting it later does not affect the dynamic loader): re-exec once with the
    environment the bridge needs."""
    wants_ros = args.ros or args.ros_wait_start or any(x.replace(" ", "") == "ros.enabled=true" for x in args.sets)
    if not wants_ros or os.environ.get("MRV_ROS_ENV_READY") == "1":
        return
    isaac_path = os.environ.get("ISAAC_PATH", os.path.expanduser("~/isaac-sim"))
    distro = os.environ.get("ROS_DISTRO") or "jazzy"
    lib = os.path.join(isaac_path, "exts", "isaacsim.ros2.bridge", distro, "lib")
    if not os.path.isdir(lib):
        sys.exit(f"[ROS] bridge libraries not found: {lib} (set ROS_DISTRO to humble / jazzy)")
    paths = [x for x in os.environ.get("LD_LIBRARY_PATH", "").split(":") if x]
    if lib not in paths:
        paths.append(lib)
    os.environ["LD_LIBRARY_PATH"] = ":".join(paths)
    os.environ["ROS_DISTRO"] = distro
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ["MRV_ROS_ENV_READY"] = "1"
    sys.stdout.flush()
    os.execv(sys.executable, [sys.executable, *sys.argv])


def main():
    args = parse_args()
    ensure_ros2_env(args)

    from srb.core.app import AppLauncher
    from srb.utils.path import SRB_APPS_DIR, SRB_LOGS_DIR
    from srb.utils.process import install_session_signal_handlers, warn_about_stale_sessions

    warn_about_stale_sessions()
    launcher_kwargs = dict(
        headless=args.headless,
        enable_cameras=True,
        experience=SRB_APPS_DIR.joinpath(f"srb.{'headless.' if args.headless else ''}rendering.kit"),
    )
    if args.kit_args:
        launcher_kwargs["kit_args"] = args.kit_args
    launcher = AppLauncher(**launcher_kwargs)
    install_session_signal_handlers()

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.util.debug_draw")

    import gymnasium

    import srb.tasks  # noqa: F401  (registers srb/debris_capture_vision)
    from srb.tasks.manipulation.debris_capture.vision import DEFAULT_CONFIG_PATH
    from srb.tasks.manipulation.debris_capture.vision_capture_demo import VisionCaptureDemo, default_out_dir
    from srb.utils.hydra.sim import hydra_task_config

    env_id = "srb/debris_capture_vision"
    hydra_dir = SRB_LOGS_DIR.joinpath("vision_capture", "hydra", datetime.now().strftime("%Y%m%d_%H%M%S"))
    try:
        hydra_dir.mkdir(parents=True, exist_ok=True)
    except OSError:
        hydra_dir = Path(tempfile.mkdtemp(prefix="vision_capture_"))
    sys.argv = [sys.argv[0], "env.robot=canadarm3", *args.overrides, f"hydra.run.dir={hydra_dir}", "hydra.output_subdir=null"]

    sets = list(args.sets)
    if args.start_yaw_deg is not None:
        sets.append(f"approach.start_yaw_offset_deg={args.start_yaw_deg}")
    if args.dock or args.dock_only:
        sets.append("docking.enabled=true")
    if args.dock_only:
        sets.append("docking.skip_capture=true")
    if args.ros or args.ros_wait_start:
        sets.append("ros.enabled=true")
    if args.ros_wait_start:
        sets.append("ros.require_start_cmd=true")
    if args.scenario == "static":
        # Test 1: the MEP is at rest (no drift, no rotation)
        sets.append("mep.linear_velocity_mps=0.0")
        sets.append("mep.motion_mode=translation_only")
    out_dir = Path(args.out_dir) if args.out_dir else default_out_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    state = {"ok": False}

    @hydra_task_config(task_name=env_id)
    def run(env_cfg, agent_cfg=None):
        env_cfg.vision_config_path = str(Path(args.config).resolve()) if args.config else DEFAULT_CONFIG_PATH.as_posix()
        env_cfg.vision_overrides = tuple(sets)
        env_cfg.apply_vision_config()
        env = gymnasium.make(env_id, cfg=env_cfg)
        env.reset()
        sim = env.unwrapped.sim
        if not args.headless:
            sim.set_camera_view(eye=(-1.0, -9.0, 7.0), target=(5.0, -2.7, 3.0))
        if args.start_paused:
            sim.pause()
            print("[DEMO] PAUSED - press Play in the Isaac Sim toolbar to start", flush=True)
        tag = args.tag or args.scenario
        csv_path = out_dir / f"{tag}_metrics.csv"
        demo = VisionCaptureDemo(env, launcher.app, args.scenario, args.headless, out_dir, csv_path, label=tag)
        results = demo.run()
        result_path = out_dir / f"{tag}_result.json"
        demo.results.metrics["command"] = " ".join([Path(sys.executable).name, *os.sys.orig_argv[1:]]) if hasattr(os.sys, "orig_argv") else ""
        demo.results.metrics["overrides"] = sets
        demo.write_json(result_path)
        print("[SUMMARY] ------------------------------------------------", flush=True)
        for name, c in results.checks.items():
            print(f"[SUMMARY] {'PASS' if c['pass'] else 'FAIL'}  {name}", flush=True)
        n_ok = sum(1 for c in results.checks.values() if c["pass"])
        print(f"[SUMMARY] {n_ok}/{len(results.checks)} checks passed, final state {results.final_state}"
              f"{'  (' + results.failure + ')' if results.failure else ''}", flush=True)
        print(f"[SUMMARY] results: {result_path}", flush=True)
        print(f"[SUMMARY] telemetry: {csv_path}", flush=True)
        state["ok"] = n_ok == len(results.checks)
        if not args.headless and state["ok"] and not args.exit_when_done:
            demo.idle()  # results are already written; keep the scene up until the window closes
        env.close()

    run()
    ## NOTE: `SimulationApp.close()` ends the process itself with exit code 0
    ## (`unload_all_plugins`), so a `sys.exit` after it never runs. The results are
    ## already written; leave with the real status instead.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if state["ok"] else 1)


if __name__ == "__main__":
    main()
