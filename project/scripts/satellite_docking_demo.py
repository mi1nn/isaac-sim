#!/usr/bin/env python3
"""Satellite docking demo: Canadarm3 captures the MEP and docks it to the satellite.

Must run with the Isaac Sim Python (the same interpreter `srb` uses), e.g.:

    cd ~/space_robotics_bench
    ~/isaac-sim/python.sh project/scripts/satellite_docking_demo.py

Sequence (state machine in `srb/tasks/manipulation/debris_capture/docking_demo.py`):
INIT -> PLAN -> MEP PRE-GRASP -> ALIGN -> MEP GRASP -> ATTACH -> LIFT -> SAT PRE-DOCK
-> ALIGN -> INSERT -> DOCK -> DOCKED. Press R in the Isaac Sim window to release the
most recent connection (docking first, then the EE attachment).
"""

import argparse
import os
import sys
import tempfile
from datetime import datetime


def parse_args():
    isaac_path = os.environ.get("ISAAC_PATH", os.path.expanduser("~/isaac-sim"))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim window")
    parser.add_argument(
        "--kit_args",
        type=str,
        default=None,
        help="Extra Omniverse Kit arguments. Default (GUI): the ones used with `srb agent manual`: "
        f'"--ext-folder {isaac_path}/apps --enable isaacsim.exp.base"',
    )
    parser.add_argument("--start_paused", action="store_true", help="Wait for Play in the toolbar before starting")
    parser.add_argument("--test_undock", type=float, default=None, metavar="SEC",
                        help="Simulate an R key press SEC seconds after docking (automated check)")
    parser.add_argument("--exit_after", type=float, default=None, metavar="SEC",
                        help="Close the app after SEC simulated seconds (automated check)")
    parser.add_argument("overrides", nargs="*", help="Extra Hydra overrides, e.g. env.docking.dock_depth=0.7")
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
    launcher_kwargs = dict(
        headless=args.headless,
        enable_cameras=False,
        experience=SRB_APPS_DIR.joinpath(f"srb.{'headless.' if args.headless else ''}kit"),
    )
    if args.kit_args:
        launcher_kwargs["kit_args"] = args.kit_args
    launcher = AppLauncher(**launcher_kwargs)
    install_session_signal_handlers()

    import gymnasium

    import srb.tasks  # noqa: F401  (registers srb/debris_capture_docking)
    from srb.tasks.manipulation.debris_capture.docking_demo import DockingDemo
    from srb.utils.hydra.sim import hydra_task_config

    env_id = "srb/debris_capture_docking"
    logdir = SRB_LOGS_DIR.joinpath("satellite_docking_demo", datetime.now().strftime("%Y%m%d_%H%M%S"))
    try:
        logdir.mkdir(parents=True, exist_ok=True)
    except OSError:
        logdir = tempfile.mkdtemp(prefix="satellite_docking_demo_")
    sys.argv = [sys.argv[0], "env.robot=canadarm3", *args.overrides, f"hydra.run.dir={logdir}", "hydra.output_subdir=null"]

    results = {}

    @hydra_task_config(task_name=env_id)
    def run(env_cfg, agent_cfg=None):
        env = gymnasium.make(env_id, cfg=env_cfg)
        env.reset()
        sim = env.unwrapped.sim
        if not args.headless:
            # Frame the whole scene: arm, MEP and satellite
            sim.set_camera_view(eye=(-6.0, -16.0, 14.0), target=(9.0, 2.0, 3.0))
        if args.start_paused:
            sim.pause()
            print("[DEMO] PAUSED - press Play in the Isaac Sim toolbar to start", flush=True)
        demo = DockingDemo(
            env,
            launcher.app,
            headless=args.headless,
            auto_undock_after=args.test_undock,
            exit_after=args.exit_after,
        )
        results.update(demo.run())
        env.close()

    run()
    failed = [k for k, v in results.items() if not k.startswith("_") and not v]
    launcher.app.close()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
