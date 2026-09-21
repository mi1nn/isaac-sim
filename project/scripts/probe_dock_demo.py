#!/usr/bin/env python3
"""Probe-into-thruster docking on GPS telemetry + a probe-tip RGB-D camera.

Canadarm3 captures the MEP exactly as `satellite_docking_demo.py` does, then docks it
into the client satellite's thruster *without reading the client's true pose*: the
direction comes from a ROS 2 `PoseStamped` the client publishes (noisy, late, biased),
and the remaining insertion depth comes from the depth camera on the probe tip.

Must run with the Isaac Sim Python (the same interpreter `srb` uses):

    cd ~/isaac_space
    # GUI -- the viewport switches to the probe's RGB camera when the docking starts
    ~/isaac-sim/python.sh project/scripts/probe_dock_demo.py
    # Headless automated check (exit code 0 only if every check passed)
    ~/isaac-sim/python.sh project/scripts/probe_dock_demo.py --headless

Watch the telemetry from another terminal:

    source /opt/ros/jazzy/setup.bash
    ros2 topic echo /client/thruster_pose

Config: `project/config/probe_dock.yaml`; override with `--set section.key=value`
(e.g. `--set gps.pos_noise_m=0.06`). `--no_ros` uses an in-process link instead.
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
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim window")
    parser.add_argument("--config", type=str, default=None, help="Docking config YAML (default: project/config/probe_dock.yaml)")
    parser.add_argument("--set", dest="sets", action="append", default=[], metavar="SECTION.KEY=VALUE",
                        help="Override a config value (repeatable)")
    parser.add_argument("--no_ros", action="store_true",
                        help="Use the in-process telemetry link instead of real ROS 2 topics")
    parser.add_argument("--exit_when_done", action="store_true",
                        help="GUI: close the app when the run ends (default: keep the scene up)")
    parser.add_argument("--start_paused", action="store_true", help="Wait for Play in the toolbar before starting")
    parser.add_argument("--exit_after", type=float, default=None, metavar="SEC",
                        help="Close after SEC simulated seconds (automated checks)")
    parser.add_argument("--kit_args", type=str, default=None,
                        help=f'Extra Kit arguments. GUI default: "--ext-folder {isaac_path}/apps --enable isaacsim.exp.base"')
    parser.add_argument("overrides", nargs="*", help="Extra Hydra overrides for the env config")
    args = parser.parse_args()
    if args.kit_args is None and not args.headless:
        args.kit_args = f"--ext-folder {isaac_path}/apps --enable isaacsim.exp.base"
    return args


def main():
    args = parse_args()

    from srb.core.app import AppLauncher
    from srb.utils.path import SRB_APPS_DIR, SRB_LOGS_DIR
    from srb.utils.process import install_session_signal_handlers, warn_about_stale_sessions

    warn_about_stale_sessions()
    launcher = AppLauncher(
        headless=args.headless,
        enable_cameras=True,  # the probe RGB-D camera is the point of this demo
        experience=SRB_APPS_DIR.joinpath(f"srb.{'headless.' if args.headless else ''}rendering.kit"),
        **({"kit_args": args.kit_args} if args.kit_args else {}),
    )
    install_session_signal_handlers()

    import gymnasium

    import srb.tasks  # noqa: F401  (registers srb/debris_capture_probe_dock)
    from srb.tasks.manipulation.debris_capture.probe_dock_config import load_probe_dock_config
    from srb.tasks.manipulation.debris_capture.probe_dock_demo import ProbeDockDemo
    from srb.utils.hydra.sim import hydra_task_config

    env_id = "srb/debris_capture_probe_dock"
    logdir = SRB_LOGS_DIR.joinpath("probe_dock_demo", datetime.now().strftime("%Y%m%d_%H%M%S"))
    try:
        logdir.mkdir(parents=True, exist_ok=True)
    except OSError:
        logdir = Path(tempfile.mkdtemp(prefix="probe_dock_demo_"))
    sys.argv = [sys.argv[0], "env.robot=canadarm3", *args.overrides,
                f"hydra.run.dir={logdir}", "hydra.output_subdir=null"]

    dock_cfg = load_probe_dock_config(args.config, args.sets)
    state = {"ok": False}

    @hydra_task_config(task_name=env_id)
    def run(env_cfg, agent_cfg=None):
        env = gymnasium.make(env_id, cfg=env_cfg)
        env.reset()
        sim = env.unwrapped.sim
        if not args.headless:
            sim.set_camera_view(eye=(-6.0, -16.0, 14.0), target=(9.0, 2.0, 3.0))
        if args.start_paused:
            sim.pause()
            print("[DEMO] PAUSED - press Play in the Isaac Sim toolbar to start", flush=True)
        demo = ProbeDockDemo(
            env, launcher.app, headless=args.headless, dock_cfg=dock_cfg,
            use_ros=not args.no_ros, exit_after=args.exit_after,
        )
        # The window must never disappear because the sequence reached some milestone --
        # not on success, not on failure, and not on a crash in the control code. In the
        # GUI the run always ends by idling, and the only way out is closing the window.
        results, crashed = {}, None
        try:
            results = demo.run()
        except Exception:
            import traceback

            crashed = traceback.format_exc()
            print(f"[ERROR] the control loop raised:\n{crashed}", flush=True)
        failed = [k for k, v in results.items() if not k.startswith("_") and not v]
        print("[SUMMARY] ------------------------------------------------", flush=True)
        for name, ok in results.items():
            if not name.startswith("_"):
                print(f"[SUMMARY] {'PASS' if ok else 'FAIL'}  {name}", flush=True)
        n_ok = sum(1 for k, v in results.items() if not k.startswith("_") and v)
        n_all = sum(1 for k in results if not k.startswith("_"))
        print(f"[SUMMARY] {n_ok}/{n_all} checks passed, final state {demo.state.name}"
              f"{' (CONTROL LOOP CRASHED)' if crashed else ''}", flush=True)
        try:
            demo.report_sensors()
        except Exception as e:
            print(f"[SENSOR] report unavailable: {e}", flush=True)
        state["ok"] = bool(results) and not failed and not crashed and demo.docking.is_docked
        try:
            demo.link.close()
        except Exception:
            pass
        if not args.headless and not args.exit_when_done:
            demo.idle_forever()
        env.close()

    run()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if state["ok"] else 1)


if __name__ == "__main__":
    main()
