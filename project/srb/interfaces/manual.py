from typing import TYPE_CHECKING, Callable, Dict, List

import torch

from srb.core.asset import Articulation, RigidObject
from srb.core.sensor import ContactSensor
from srb.interfaces.teleop import EventOmniKeyboardTeleopInterface
from srb.utils.math import deg_to_rad, rad_to_deg

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class ManualJointControlInterface:
    """Keyboard-driven joint-level control of a manipulator and its gripper.

    Commands joint position targets directly on the articulations, bypassing the
    environment's action manager. This is deliberate: the Canadarm3 action group
    exposes only a 6-DoF relative IK twist and the Kinova gripper only a binary
    open/close, neither of which can address an individual joint.
    """

    GRIPPER_STEP: float = 0.1

    def __init__(self, env: "AnyEnv", joint_step_deg: float = 1.0):
        self._env = env.unwrapped  # type: ignore
        self._scene = self._env.scene

        self._arm: Articulation = self._scene["robot"]
        end_effector = self._scene.articulations.get("end_effector")
        self._gripper: Articulation | None = (
            end_effector if isinstance(end_effector, Articulation) else None
        )
        self._debris: RigidObject | None = self._scene.rigid_objects.get("debris")
        self._contacts: ContactSensor | None = self._scene.sensors.get(
            "contacts_end_effector"
        )

        self._joint_step = deg_to_rad(joint_step_deg)
        self._active_joint = 0
        self._arm_joint_ids = self._resolve_arm_joint_ids()

        # Targets start at the configured default pose, not at the measured pose,
        # so the very first command carries zero error and nothing lurches.
        self._arm_target: torch.Tensor = self._arm.data.default_joint_pos.clone()
        self._gripper_target: torch.Tensor | None = (
            self._gripper.data.default_joint_pos.clone()
            if self._gripper is not None
            else None
        )
        self._gripper_fraction: float = 0.0

        self._gripper_open, self._gripper_close = self._resolve_gripper_commands()

        self._keyboard = EventOmniKeyboardTeleopInterface(self._build_bindings())

    @property
    def arm_joint_names(self) -> List[str]:
        return [self._arm.joint_names[i] for i in self._arm_joint_ids]

    def add_callback(self, key: str, func: Callable):
        self._keyboard.add_callback(key, func)

    def apply(self):
        """Write the current targets to the articulations. Call every physics step."""
        self._arm.set_joint_position_target(self._arm_target)
        if self._gripper is not None and self._gripper_target is not None:
            self._gripper.set_joint_position_target(self._gripper_target)

    def sync_to_measured(self):
        """Adopt the measured joint positions as targets (stops any commanded motion)."""
        self._arm_target = self._arm.data.joint_pos.clone()
        if self._gripper is not None:
            self._gripper_target = self._gripper.data.joint_pos.clone()
        print("[manual] targets synced to measured joint positions")

    def sync_to_default(self):
        """Adopt the configured initial joint positions as targets."""
        self._arm_target = self._arm.data.default_joint_pos.clone()
        if self._gripper is not None:
            self._gripper_target = self._gripper.data.default_joint_pos.clone()
        self._gripper_fraction = 0.0
        print("[manual] targets reset to configured initial joint positions")

    def _find_action_term(self, *required_attrs: str):
        action_cfg = getattr(self._env.cfg, "actions", None)
        if action_cfg is None:
            return None
        for candidate in vars(action_cfg).values():
            if all(hasattr(candidate, attr) for attr in required_attrs):
                return candidate
        return None

    def _resolve_arm_joint_ids(self) -> List[int]:
        """Restrict control to the arm's own joints.

        The gripper is attached with a fixed joint, so depending on how PhysX
        resolves the assembly the "robot" articulation view may expose the finger
        joints too. Selecting by the arm action term's joint names keeps the
        1..N key mapping pointing at the arm regardless.
        """
        term = self._find_action_term("joint_names", "body_name")
        if term is not None:
            joint_ids = self._arm.find_joints(term.joint_names)[0]
            if joint_ids:
                return sorted(joint_ids)
        return list(range(self._arm.num_joints))

    def _resolve_gripper_commands(
        self,
    ) -> tuple[torch.Tensor | None, torch.Tensor | None]:
        """Read open/close joint commands off the gripper's binary action term."""
        if self._gripper is None:
            return None, None

        open_cmd = self._gripper.data.default_joint_pos.clone()
        close_cmd = self._gripper.data.default_joint_pos.clone()

        term = self._find_action_term("open_command_expr", "close_command_expr")
        if term is None:
            print(
                "[manual] WARNING: no binary gripper action term found; "
                "open/close will hold the default joint positions"
            )
            return open_cmd, close_cmd

        for expr, value in term.open_command_expr.items():
            for idx in self._gripper.find_joints(expr)[0]:
                open_cmd[:, idx] = value
        for expr, value in term.close_command_expr.items():
            for idx in self._gripper.find_joints(expr)[0]:
                close_cmd[:, idx] = value

        return open_cmd, close_cmd

    def _build_bindings(self) -> Dict[str, Callable]:
        bindings: Dict[str, Callable] = {
            "Z": lambda: self._select_joint(self._active_joint - 1),
            "X": lambda: self._select_joint(self._active_joint + 1),
            "UP": lambda: self._nudge_joint(+1.0),
            "DOWN": lambda: self._nudge_joint(-1.0),
            "RIGHT": lambda: self._scale_step(2.0),
            "LEFT": lambda: self._scale_step(0.5),
            "O": lambda: self._set_gripper(0.0),
            "C": lambda: self._set_gripper(1.0),
            "K": lambda: self._set_gripper(self._gripper_fraction - self.GRIPPER_STEP),
            "M": lambda: self._set_gripper(self._gripper_fraction + self.GRIPPER_STEP),
            "N": self.sync_to_measured,
            "B": self.sync_to_default,
            "G": self.report_grasp_state,
            "H": self.print_help,
        }
        for i in range(min(9, len(self._arm_joint_ids))):
            bindings[f"KEY_{i + 1}"] = lambda i=i: self._select_joint(i)
            bindings[f"NUMPAD_{i + 1}"] = lambda i=i: self._select_joint(i)
        return bindings

    def _select_joint(self, index: int):
        self._active_joint = index % len(self._arm_joint_ids)
        self._print_joint_state()

    def _scale_step(self, factor: float):
        self._joint_step = max(
            deg_to_rad(0.05), min(deg_to_rad(15.0), self._joint_step * factor)
        )
        print(f"[manual] joint step = {rad_to_deg(self._joint_step):.3f} deg")

    def _nudge_joint(self, direction: float):
        idx = self._arm_joint_ids[self._active_joint]
        target = self._arm_target[:, idx] + direction * self._joint_step

        limits = self._arm.data.soft_joint_pos_limits
        if torch.all(torch.isfinite(limits[:, idx, :])):
            target = torch.clamp(target, limits[:, idx, 0], limits[:, idx, 1])

        self._arm_target[:, idx] = target
        self._print_joint_state()

    def _set_gripper(self, fraction: float):
        if (
            self._gripper is None
            or self._gripper_open is None
            or self._gripper_close is None
        ):
            print("[manual] no articulated gripper in this scene")
            return

        self._gripper_fraction = min(1.0, max(0.0, fraction))
        self._gripper_target = (
            self._gripper_open
            + (self._gripper_close - self._gripper_open) * self._gripper_fraction
        )
        print(
            f"[manual] gripper {100.0 * self._gripper_fraction:.0f}% closed "
            f"(0% = open, 100% = closed)"
        )

    def _print_joint_state(self):
        idx = self._arm_joint_ids[self._active_joint]
        name = self._arm.joint_names[idx]
        print(
            f"[manual] J{self._active_joint + 1} {name}: "
            f"target {rad_to_deg(self._arm_target[0, idx].item()):+8.3f} deg | "
            f"actual {rad_to_deg(self._arm.data.joint_pos[0, idx].item()):+8.3f} deg | "
            f"step {rad_to_deg(self._joint_step):.3f} deg"
        )

    def report_grasp_state(self):
        """Print the physical evidence that distinguishes a grasp from mere overlap."""
        lines = ["[manual] --- grasp state ---"]

        if self._gripper is not None:
            joints = ", ".join(
                f"{n}={rad_to_deg(v.item()):.1f}deg"
                for n, v in zip(
                    self._gripper.joint_names, self._gripper.data.joint_pos[0]
                )
            )
            lines.append(f"  gripper joints: {joints}")

        if self._contacts is not None:
            matrix = self._contacts.data.force_matrix_w
            if matrix is not None:
                per_body = torch.norm(matrix[0], dim=-1)
                total = per_body.sum().item()
                in_contact = int((per_body.max(dim=-1)[0] > 1e-3).sum().item())
                lines.append(
                    f"  gripper<->debris contact: {in_contact} bodies, "
                    f"total |F| = {total:.3f} N"
                )
                if in_contact == 0:
                    lines.append("    -> NO physical contact (visual overlap only)")
                elif in_contact < 2:
                    lines.append("    -> single-sided contact, not a grasp yet")
                else:
                    lines.append("    -> multi-body contact, grasp is plausible")
            else:
                lines.append("  gripper<->debris contact: no filtered force matrix")
        else:
            lines.append("  no end-effector contact sensor in this scene")

        if self._debris is not None:
            lin = self._debris.data.root_com_lin_vel_w[0]
            ang = self._debris.data.root_com_ang_vel_w[0]
            lines.append(
                f"  debris |v| = {torch.norm(lin).item():.4f} m/s, "
                f"|w| = {torch.norm(ang).item():.4f} rad/s"
            )
            tf = self._scene.sensors.get("tf_end_effector")
            if tf is not None:
                distance = torch.norm(
                    tf.data.target_pos_w[0, 0] - self._debris.data.root_com_pos_w[0]
                ).item()
                lines.append(f"  TCP <-> debris centre distance = {distance:.4f} m")

        capture = getattr(self._env, "_capture", None)
        if capture is not None:
            lines.append(f"  capture: {capture.status()}")

        print("\n".join(lines))

    def print_help(self):
        print(self)

    def __str__(self) -> str:
        names = "\n".join(
            f"\t    {i + 1}: {n}" for i, n in enumerate(self.arm_joint_names)
        )
        return (
            "\n\t====================== Manual control ======================\n"
            "\tSimulation starts PAUSED. Press Play in the Isaac Sim toolbar\n"
            "\t(or SPACE) to begin stepping physics. Nothing moves until you\n"
            "\tpress a key below.\n"
            "\t------------------------------------------------------------\n"
            "\tArm\n"
            "\t    1-9 / NUMPAD    select joint (see list below)\n"
            "\t    Z / X           select previous / next joint\n"
            "\t    UP / DOWN       selected joint +/- step\n"
            "\t    LEFT / RIGHT    halve / double the step size\n"
            "\tGripper\n"
            "\t    O               open   (0% closed)\n"
            "\t    C               close  (100% closed)\n"
            "\t    K / M           open / close by 10%\n"
            "\tState\n"
            "\t    G               print grasp diagnostics (contacts, velocities)\n"
            "\t    N               hold here (targets <- measured positions)\n"
            "\t    B               back to the configured initial joint pose\n"
            "\t    R               release a captured MEP (capture tasks only)\n"
            "\t    L               reset the whole scene\n"
            "\t    H               print this help\n"
            "\t------------------------------------------------------------\n"
            f"\tArm joints:\n{names}\n"
            "\t============================================================\n"
        )
