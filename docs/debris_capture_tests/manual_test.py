"""Run the real `srb.__main__.manual_agent()` loop headless with a scripted keyboard.

Run: ~/isaac-sim/python.sh docs/debris_capture_tests/manual_test.py

Only the keyboard device is replaced (no window to receive key events); every key
press below goes through the exact callbacks `srb agent manual` binds.
"""

import sys
import tempfile

from srb.core.app import AppLauncher
from srb.utils.path import SRB_APPS_DIR

launcher = AppLauncher(
    headless=True,
    enable_cameras=True,
    experience=SRB_APPS_DIR.joinpath("srb.headless.rendering.kit"),
)

import gymnasium  # noqa: E402
import torch  # noqa: E402

import srb.interfaces.manual as manual_mod  # noqa: E402
import srb.tasks  # noqa: E402,F401
from srb.__main__ import manual_agent  # noqa: E402
from srb.utils.hydra.sim import hydra_task_config  # noqa: E402

ENV_ID = "srb/debris_capture_visual"
sys.argv = [
    sys.argv[0],
    "env.robot=canadarm3",
    f"hydra.run.dir={tempfile.mkdtemp(prefix='srb_capture_test_')}",
    "hydra.output_subdir=null",
]


class FakeKeyboard:
    instance = None

    def __init__(self, bindings):
        self.bindings = dict(bindings)
        FakeKeyboard.instance = self

    def add_callback(self, key, func):
        self.bindings[key] = func

    def press(self, key):
        print(f"[KEY] {key}", flush=True)
        self.bindings[key]()


manual_mod.EventOmniKeyboardTeleopInterface = FakeKeyboard


@hydra_task_config(task_name=ENV_ID)
def main(env_cfg, agent_cfg=None):
    env = gymnasium.make(ENV_ID, cfg=env_cfg)
    env.reset()
    u = env.unwrapped
    robot, cap = u._robot, u._capture
    arm_ids = robot.find_joints("canadarm3_large_joint_[1-7]")[0]
    link_id = cap._link_body_id
    results = {}

    class ScriptedApp:
        """`is_running()` is polled once per loop iteration; drive the script from it."""

        def __init__(self):
            self.frame = 0
            self.phase = "keys"

        def controller(self):
            # The ManualJointControlInterface owns the keyboard; reach it via a binding
            return FakeKeyboard.instance.bindings["UP"].__closure__[0].cell_contents

        def is_running(self):
            self.frame += 1
            kb = FakeKeyboard.instance
            ctrl = self.controller()
            if self.phase == "keys":
                if self.frame == 50:
                    j1 = robot.data.joint_pos[0, arm_ids[0]].item()
                    results["j1_before"] = j1
                    kb.press("KEY_1")
                    for _ in range(3):
                        kb.press("UP")
                if self.frame == 400:
                    moved = robot.data.joint_pos[0, arm_ids[0]].item() - results["j1_before"]
                    print(f"[CHECK] joint1 moved {moved*57.2958:.3f} deg after 3x UP (expected ~3 deg)", flush=True)
                    results["keys_move_joint"] = abs(moved * 57.2958 - 3.0) < 0.3
                    kb.press("G")
                    self.phase = "approach"
            elif self.phase == "approach":
                # Emulate the operator steering the arm: update the same joint targets
                # the arrow keys edit, using a small position IK on the capture point
                if cap.state[0].item() == 2:
                    kb.press("G")
                    results["captured_in_manual_loop"] = True
                    self.phase, self.t0 = "follow", self.frame
                    self.mep0 = u._obj.data.root_pos_w[0].clone()
                    return True
                jac = robot.root_physx_view.get_jacobians()
                J = jac[0, link_id - 1, :, arm_ids]
                r = cap.robot_capture_pos_w[0] - robot.data.body_pos_w[0, link_id]
                Jp = J[:3] - torch.linalg.cross(r.unsqueeze(1).expand(3, len(arm_ids)).T, J[3:6].T).T
                err = cap.mep_capture_pos_w[0] - cap.robot_capture_pos_w[0]
                v = err / torch.norm(err) * min(0.02, torch.norm(err).item())
                dq = Jp.T @ torch.linalg.solve(Jp @ Jp.T + 0.0025 * torch.eye(3, device=Jp.device), v)
                ctrl._arm_target[0, arm_ids] = robot.data.joint_pos[0, arm_ids] + torch.clamp(dq, -0.002, 0.002) * 5
                if self.frame > 12000:
                    results["captured_in_manual_loop"] = False
                    return False
            elif self.phase == "follow":
                if self.frame == self.t0 + 1:
                    kb.press("N")  # hold here
                    kb.press("KEY_1")
                    for _ in range(5):
                        kb.press("DOWN")
                if self.frame == self.t0 + 1500:
                    d = torch.norm(u._obj.data.root_pos_w[0] - self.mep0).item()
                    print(f"[CHECK] MEP moved {d:.3f} m after 5x DOWN on joint 1 while captured", flush=True)
                    results["mep_follows_keys"] = d > 0.05
                    kb.press("R")
                    results["released_by_R"] = cap.state[0].item() != 2
                    kb.press("L")
                if self.frame == self.t0 + 1800:
                    results["reset_ok"] = cap.state[0].item() == 0
                    kb.press("G")
                    return False
            return True

    manual_agent(env=env, sim_app=ScriptedApp(), headless=False, joint_step=1.0, autoplay=True)
    for k, v in results.items():
        if isinstance(v, bool):
            print(f"[TEST] {'PASS' if v else 'FAIL'} {k}", flush=True)
    env.close()


main()
launcher.app.close()
