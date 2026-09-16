from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# USD 안의 camera_graph 가 ROS2 노드를 쓰므로 bridge 를 먼저 켠다. rclpy import 보다 앞에 둔다
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)

import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube_GB/m0609_camera_cube_GB.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#  카메라 ROS2 퍼블리시
# ══════════════════════════════════════════════════════════════
# USD 에 들어 있는 Action Graph 의 RGB 퍼블리셔. 원본 topicName 은 /rgb 이다
RGB_PUBLISH_NODE = "/World/Graph/camera_graph/RGBPublish"
RGB_TOPIC        = "/rgb1"

# color detector 가 /rgb1 을 받아 판별한 결과를 여기로 보낸다
COLOR_TOPIC      = "/color_id"


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"

# Drive 는 팔 6축에만 적용한다
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# 로봇 base 의 월드 pose — Lula 가 월드 좌표를 base 좌표로 바꿀 때 쓴다
ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 도달 범위 판정 기준 (URDF 실측)
#   어깨 높이 = base_link -> joint_1 = 0.1345
#   최대 반경 = 0.411 + 0.368 + 0.121 = 0.900
SHOULDER_Z = 0.1345
SPEC_REACH = 0.900

# 시작 자세 — 그리퍼가 아래를 향하도록 미리 굽혀 둔다
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
# finger_joint 가 구동 관절이고 나머지 5개는 Mimic 으로 따라온다
# 두 번째 이름은 ParallelGripper 가 요구하는 형식상 필요하다
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

# finger_joint 절대 목표값 (라디안)
#   Physics Inspector 는 도로 표시한다.  0.0 ~ 67.609 deg = 0.0 ~ 1.18 rad
GRIPPER_OPEN_POS  = 0.0     #   0.0 deg
GRIPPER_CLOSE_POS = 0.8     #  45.8 deg



# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
# link_6 로컬 좌표계에서 손가락 패드 끝까지의 거리 (실측)
#   손가락 패드 범위  0.13632 ~ 0.19671
#   링크 원점 0.14155 는 관절 위치이지 파지면이 아니다
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
# 작업 영역
# ══════════════════════════════════════════════════════════════

PICK_X_RANGE = (0.20, 0.35)
PICK_Y_RANGE = (-0.10, 0.10)

BLUE_PLACE_XY = np.array([0.45, -0.15])
GREEN_PLACE_XY = np.array([0.45, 0.15])

COLOR_BLUE = 1
COLOR_GREEN = 2

for _name, _xy in [("PICK corner", np.array([PICK_X_RANGE[1], PICK_Y_RANGE[1]])),
                    ("BLUE_PLACE_XY", BLUE_PLACE_XY),
                    ("GREEN_PLACE_XY", GREEN_PLACE_XY)]:
    _r = float(np.linalg.norm(_xy))
    assert _r < SPEC_REACH, f"{_name} radius {_r:.3f} m exceeds SPEC_REACH {SPEC_REACH} m"

# 바닥 마커 — place 위치를 눈으로 확인하기 위한 표시. place 좌표와 동일한 자리에 둔다
BLUE_MARKER_PATH  = "/World/blue_marker"
GREEN_MARKER_PATH = "/World/green_marker"

MARKER_RADIUS = 0.04
MARKER_HEIGHT = 0.002
MARKER_Z      = MARKER_HEIGHT / 2.0

# 손목 카메라(화각 약 90도)에 마커가 같이 찍힌다. color detector 가 마커를 큐브로 세지 않도록
# 채도를 detector 하한(S 80) 아래로 둔다. 렌더 후 OpenCV HSV 기준 blue S 50, green S 45
MARKER_BLUE_COLOR  = Gf.Vec3f(0.58, 0.68, 0.95)
MARKER_GREEN_COLOR = Gf.Vec3f(0.58, 0.90, 0.58)

# ══════════════════════════════════════════════════════════════
# Process FSM 상태
# ══════════════════════════════════════════════════════════════

WAIT_COLOR = 1
RUN_PICK_PLACE = 2
TASK_DONE = 3

# 색상 확정 조건
#   COLOR_SETTLE_STEPS   Play 직후 reset 전 자세의 영상이 섞이지 않도록 이만큼 지난 뒤 듣기 시작한다
#   COLOR_CONFIRM_COUNT  같은 색이 연속 이 횟수만큼 와야 확정한다
COLOR_SETTLE_STEPS  = 30
COLOR_CONFIRM_COUNT = 5

def random_pick_xy():
    x = np.random.uniform(*PICK_X_RANGE)
    y = np.random.uniform(*PICK_Y_RANGE)

    return np.array([x, y])

CUBE_PATHS = {
    COLOR_BLUE:  "/World/blue_block",
    COLOR_GREEN: "/World/green_block",
}

# 큐브 한 변 0.05 m — 루트 Xform 을 바닥에서 반 변 높이에 둔다
CUBE_Z = 0.025

# place 위치를 표시하는 색깔 원판 마커를 바닥에 만든다
def create_floor_marker(marker_path, xy, color):
    stage = omni.usd.get_context().get_stage()

    marker = UsdGeom.Cylinder.Define(stage, marker_path)
    marker.CreateRadiusAttr(MARKER_RADIUS)
    marker.CreateHeightAttr(MARKER_HEIGHT)
    marker.CreateAxisAttr("Z")

    UsdGeom.Xformable(marker).AddTranslateOp().Set(
        Gf.Vec3d(float(xy[0]), float(xy[1]), MARKER_Z)
    )

    material_path = marker_path + "_material"
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )

    UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(material)


def _set_translate(prim, value):
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(type(op.Get())(*value))
            return
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*value))


def normalize_cube(root_path):
    """
    USD 원본의 두 블럭 구조가 달라서 맞춘다.
      blue_block/Cube 는 부모 기준 (0.437, -1.255) 오프셋이 있어
      부모를 옮기면 큐브가 1.3 m 떨어진 곳에 보인다.
      blue_block/Cube 에만 RigidBodyAPI 가 있어 green 과 동작이 달랐다.
    Cube 오프셋을 0 으로, Rigid Body 는 두 블럭 모두 Cube 한 곳에만 둔다.
    """
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    cube = stage.GetPrimAtPath(root_path + "/Cube")
    if not root.IsValid() or not cube.IsValid():
        raise RuntimeError(f"cube prim not found: {root_path}/Cube")

    _set_translate(cube, (0.0, 0.0, 0.0))
    UsdPhysics.RigidBodyAPI.Apply(cube)
    root.RemoveAPI(UsdPhysics.RigidBodyAPI)


def place_cubes(color_id, pick_xy):
    """
    물리 시작 전(set_up_scene)에 USD 에 직접 써 둔다.
    Stop 은 Play 직전의 USD 값으로 되돌리므로 Stop/Play 를 반복해도 같은 자리에서 시작한다.
    안 쓰는 블럭은 비활성화해서 렌더링·물리 모두에서 뺀다.
    """
    stage = omni.usd.get_context().get_stage()
    for cid, path in CUBE_PATHS.items():
        normalize_cube(path)
        root = stage.GetPrimAtPath(path)
        if cid == color_id:
            _set_translate(root, (float(pick_xy[0]), float(pick_xy[1]), CUBE_Z))
        else:
            root.SetActive(False)


# 높이
#   PICK_Z    큐브 상단면. 여기서 그리퍼를 닫으면 큐브 옆면을 문다
#   PLACE_Z   놓을 때는 살짝 높게 두어 큐브가 튀지 않도록 한다
#   APPROACH  집기 전 대기 높이
#   LIFT      들고 이동할 높이
PICK_Z          = 0.05
PLACE_Z         = 0.055
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23

# 그리퍼를 닫고 기다리는 스텝 수
GRIPPER_WAIT = 120

# 보간 파라미터
#   스텝 수를 고정하면 시간이 고정되어 먼 구간일수록 빨라진다.
#   스텝당 이동 거리를 고정하고 구간 길이로 스텝 수를 계산한다.
TCP_SPEED  = 0.004     # 스텝당 TCP 이동 거리(m)
MIN_STEPS  = 60        # 짧은 구간이 순간이동하지 않도록
MAX_STEPS  = 600       # 스텝 수 폭주 방지
HOLD_STEPS = 60        # 지점 도착 후 멈춰 있는 시간

# 접근 방향 — 툴(link_6 로컬 +Z)이 어디를 향할지
#   roll  pitch      방향
#    180      0      바닥
#    180     90      +x 수평
#    180    -90      -x 수평
#     90      0      -y 수평
#    -90      0      +y 수평
#      0      0      하늘
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0

# 툴축 회전 — 접근 방향은 그대로, 손가락(로컬 +X)만 돌아간다
GRIPPER_YAW_DEG = 0.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """
    각도 세 개로 목표 자세를 만든다.

    roll, pitch 로 접근 방향을 정한 뒤 yaw 를 마지막에 곱한다.
    마지막에 곱하면 툴 로컬 Z축 회전이 되므로
    접근 방향은 유지되고 손가락 방향만 바뀐다.
    """
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """
    쿼터니언을 회전행렬로 바꾼다.
    각 열이 로컬 축의 월드 방향이다.
      1열 = 로컬 +X (손가락 방향)
      3열 = 로컬 +Z (툴 방향)
    """
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    """
    손가락 끝 목표를 플랜지 목표로 바꾼다.

    오프셋은 link_6 로컬 좌표이므로 목표 자세만큼 회전시킨 뒤 빼야 한다.
    """
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 손가락 끝의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  궤적 보간
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간"""
    return start + alpha * (goal - start)


class PickPlaceFSM:

    NAMES = [
        "APPROACH",
        "DESCEND",
        "GRASP",
        "LIFT",
        "MOVE",
        "LOWER",
        "RELEASE",
        "DONE"
    ]
    
    GRIPPER_STATES = {
        2: "close",
        6: "open"
    }  
    
    DONE_STATE = 7

    def __init__(self, robot):

        self._robot = robot

        self.pick_xy = np.array([0.0, 0.0])
        self.place_xy = np.array([0.0, 0.0])

        self.waypoints = []

        self.reset()

    def configure(self, pick_xy, place_xy):

        self.pick_xy = np.array(pick_xy, dtype=float)
        self.place_xy = np.array(place_xy, dtype=float)

        self._build_waypoints()

        self.reset()

        print(f"Pick  : {vec(self.pick_xy)}")
        print(f"Place : {vec(self.place_xy)}")

    def _build_waypoints(self):

        px, py = self.pick_xy
        gx, gy = self.place_xy

        self.waypoints = [
            np.array([px, py, APPROACH_HEIGHT]),
            np.array([px, py, PICK_Z]),
            np.array([px, py, PICK_Z]),
            np.array([px, py, LIFT_HEIGHT]),
            np.array([gx, gy, LIFT_HEIGHT]),
            np.array([gx, gy, PLACE_Z]),
            np.array([gx, gy, PLACE_Z]),
        ]
        
    def reset(self):
        self.state = 0
        self.step = 0
        self.start = None
        
        if self.waypoints:
            self.goal = self.waypoints[0]
        else:
            self.goal = None
            
        self.n_steps = MIN_STEPS
        self.gripper = "open"

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        """한 스텝 진행한다"""
        if self.state >= self.DONE_STATE:
            return

        # 단계에 처음 들어온 순간 시작점과 스텝 수를 정한다
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

            if self.state in self.GRIPPER_STATES:
                self.n_steps = GRIPPER_WAIT
                dist = 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}"
                  f"  {dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")

        self.step += 1
        if self.step >= self.n_steps:
            self._next()

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


class M0609Task(BaseTask):
    """
    set_up_scene 은 BaseTask 가 정한 이름이다. World 가 이 이름으로 부른다.
    _ 로 시작하는 메서드는 우리가 나눈 것이라 이름을 바꿔도 된다.
    """

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        # 스크립트 실행 1회당 한 번만 뽑는다. Stop/Play 로는 바뀌지 않는다
        self.color_id = int(np.random.choice([COLOR_BLUE, COLOR_GREEN]))
        self.pick_xy = random_pick_xy()

    # ── 프레임워크 규약 ──────────────────────────────────
    def set_up_scene(self, scene):
        """world.reset() 안에서 자동으로 불린다"""
        super().set_up_scene(scene)
        self._load_usd()
        self._set_rgb_topic()
        self._add_place_markers()
        self._setup_arm_drives()
        self._register_robot(scene)
        place_cubes(self.color_id, self.pick_xy)
        print(f"   cube         color={self.color_id} pick={vec(self.pick_xy)}")
        print("   scene        ready")

    # ── 우리가 나눈 단계 ─────────────────────────────────
    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

        print("   USD          loaded")

    def _set_rgb_topic(self):
        """camera_graph 의 RGB 퍼블리셔 토픽 이름을 바꾼다"""
        stage = omni.usd.get_context().get_stage()
        node = stage.GetPrimAtPath(RGB_PUBLISH_NODE)
        if not node.IsValid():
            raise RuntimeError(f"RGB publish node not found: {RGB_PUBLISH_NODE}")

        node.GetAttribute("inputs:topicName").Set(RGB_TOPIC)
        print(f"   rgb topic    {RGB_TOPIC}")

    def _add_place_markers(self):
        """place 좌표에 색깔 원판 마커를 바닥에 둔다"""
        create_floor_marker(BLUE_MARKER_PATH, BLUE_PLACE_XY, MARKER_BLUE_COLOR)
        create_floor_marker(GREEN_MARKER_PATH, GREEN_PLACE_XY, MARKER_GREEN_COLOR)

        print("   markers      placed")

    def _setup_arm_drives(self):
        """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
        stage = omni.usd.get_context().get_stage()
        count = 0

        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    count += 1

        print(f"   arm drives   {count}")

    def _register_robot(self, scene):
        """로봇과 그리퍼를 등록한다. world.scene 이 아니라 인자 scene 을 쓴다"""
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

        gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
                gripper=gripper,
            )
        )
        print(f"   EE frame     {ee_path}")

    @property
    def robot(self):
        return self._robot

class ColorSubscriber(Node):
    """
    detector 는 영상이 올 때마다 color_id 를 publish 한다.
    start_listening() 이후에 온 값만 쓰고, 확정되면 다시 듣지 않는다.
    그래서 pick & place 도중 손목 카메라가 본 값이 다음 판별에 남지 않는다.
    """

    def __init__(self):
        super().__init__("isaac_color_subscriber")
        self.reset()
        self.subscription = self.create_subscription(
            Int32,
            COLOR_TOPIC,
            self.color_callback,
            10,
        )

    def reset(self):
        """듣기를 멈추고 모아 둔 값을 모두 버린다"""
        self.listening = False
        self._candidate = None
        self._count = 0
        self._confirmed = None

    def start_listening(self):
        self.reset()
        self.listening = True
        print("[ROS2] waiting color_id")

    def color_callback(self, msg):
        if not self.listening:
            return

        if msg.data not in [COLOR_BLUE, COLOR_GREEN]:
            print(f"[ROS2] invalid color_id: {msg.data}")
            return

        if msg.data == self._candidate:
            self._count += 1
        else:
            self._candidate = msg.data
            self._count = 1

        if self._count >= COLOR_CONFIRM_COUNT:
            self._confirmed = self._candidate
            self.listening = False
            print(f"[ROS2] color_id = {self._confirmed} ({COLOR_CONFIRM_COUNT} in a row)")

    def get_color(self):
        color = self._confirmed
        self._confirmed = None
        return color

def init_gripper(robot, world):
    """그리퍼는 Articulation 초기화 이후에 따로 초기화한다"""
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    """시작 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버
# ══════════════════════════════════════════════════════════════
def create_ik_solver(robot):
    """
    Lula 계산기를 만들고 로봇과 연결한다.

    LulaKinematicsSolver         : URDF 만 읽는 계산기. 로봇을 모른다
    ArticulationKinematicsSolver : 계산 결과를 로봇 관절 명령으로 바꾼다
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )

    # 월드 좌표와 base 좌표를 잇는다. 지금은 항등이지만 반드시 호출한다
    lula.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )

    print(f"   controlled   {', '.join(lula.get_joint_names())}")

    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_target_info(target_quat):
    R = quat_to_matrix(target_quat)
    section("PLAN")
    print(
        f"   pick x range [{PICK_X_RANGE[0]:.3f}, "
        f"{PICK_X_RANGE[1]:.3f}]"
    )
    print(
        f"   pick y range [{PICK_Y_RANGE[0]:.3f}, "
        f"{PICK_Y_RANGE[1]:.3f}]"
    )
    print(f"   blue place   {vec(BLUE_PLACE_XY)}")
    print(f"   green place  {vec(GREEN_PLACE_XY)}")
    print()
    print(f"   approach z   {APPROACH_HEIGHT}")
    print(f"   pick z       {PICK_Z}")
    print(f"   lift z       {LIFT_HEIGHT}")
    print(f"   place z      {PLACE_Z}")
    print()
    print(f"   tcp speed    {TCP_SPEED} m/step")
    print(
        f"   gripper      open {GRIPPER_OPEN_POS} "
        f"close {GRIPPER_CLOSE_POS}"
    )
    print(f"   gripper wait {GRIPPER_WAIT} steps")
    print()
    print(
        f"   tool   +Z    "
        f"{vec(R @ np.array([0, 0, 1]))} "
        f"approach direction"
    )
    print(
        f"   finger +X    "
        f"{vec(R @ np.array([1, 0, 0]))} "
        f"finger direction"
    )


def print_dof_info(robot):
    """어떤 관절이 몇 번인지 확인한다"""
    section("DOF")
    for i, name in enumerate(robot.dof_names):
        tag = "arm" if name in ARM_JOINTS else "gripper"
        print(f"   [{i:2d}] {name:28s} {tag}")
    print()
    print(f"   finger index {robot.get_dof_index('finger_joint')}")
    print(f"   num_dof      {robot.num_dof}")


def print_gripper_state(robot, command):
    """명령값과 실제값, Mimic 관절 전체를 함께 본다"""
    q = robot.get_joint_positions()
    actual = q[robot.get_dof_index("finger_joint")]
    print(f"   gripper {command:5s}   finger {actual:+.4f}")
    print(f"   dof[6:12] {vec(q[6:12], 4)}")


def print_status(robot, solved, fsm, target_tcp):
    """현재 단계와 손가락 끝 위치를 함께 찍는다"""
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]

    if not solved:
        print(f"   {name:9s} IK FAILED  target {vec(target_tcp)}")
        return

    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
    print(f"   {name:9s} tcp {vec(tcp)}   finger {finger:+.4f}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    
    rclpy.init()
    color_node = ColorSubscriber()
    
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    # world.reset() 이 Task.set_up_scene() 을 자동으로 부른다
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)

    for _ in range(30):
        world.step(render=True)

    print_dof_info(robot)

    section("SOLVER")
    ik_solver = create_ik_solver(robot)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    print_target_info(target_quat)

    section("RUN")
    print("   press Play in the viewport\n")

    fsm = PickPlaceFSM(robot)
    was_playing = False
    step = 0
    
    process_state = WAIT_COLOR
    wait_step = 0
    current_pick_xy = task.pick_xy

    while simulation_app.is_running():
        world.step(render=True)

        # ROS2 callback 처리
        rclpy.spin_once(
            color_node,
            timeout_sec=0.0
        )

        is_playing = world.is_playing()

        # ─────────────────────────────────────
        # Play 시작 순간 초기화
        # ─────────────────────────────────────
        if is_playing and not was_playing:
            world.reset()

            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)

            # 블럭은 world.reset() 의 stop 이 Play 전 USD 위치로 되돌려 둔다
            step = 0
            process_state = WAIT_COLOR
            wait_step = 0
            color_node.reset()
            fsm.reset()

            print(f"\n[PROCESS] START  cube color={task.color_id} pick={vec(current_pick_xy)}")

        # Stop 상태면 아래 로직 실행하지 않음
        if not is_playing:
            was_playing = is_playing
            continue

        # ═════════════════════════════════════
        # 1. PC B의 색상 판별 결과 대기
        # ═════════════════════════════════════
        if process_state == WAIT_COLOR:

            wait_step += 1
            if wait_step == COLOR_SETTLE_STEPS:
                color_node.start_listening()

            color_id = color_node.get_color()

            # 아직 메시지가 안 왔으면 아무것도 하지 않음
            if color_id is None:
                was_playing = is_playing
                continue

            print(f"[PROCESS] received color_id={color_id}")

            # 색상에 따라 Place 위치 결정
            if color_id == COLOR_BLUE:
                place_xy = BLUE_PLACE_XY

            elif color_id == COLOR_GREEN:
                place_xy = GREEN_PLACE_XY

            else:
                print(f"[PROCESS] invalid color_id={color_id}")
                was_playing = is_playing
                continue

            # 이번 Pick & Place 좌표 설정
            fsm.configure(
                pick_xy=current_pick_xy,
                place_xy=place_xy
            )

            step = 0

            print(
                f"[PROCESS] pick={current_pick_xy} "
                f"place={place_xy}"
            )

            process_state = RUN_PICK_PLACE

        # ═════════════════════════════════════
        # 2. 실제 Pick & Place 수행
        # ═════════════════════════════════════
        elif process_state == RUN_PICK_PLACE:

            # 현재 FSM target
            target_tcp = fsm.current_target()

            flange_target = tcp_to_flange(
                target_tcp,
                target_quat
            )

            # IK 계산
            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )

            if solved:
                robot.apply_action(action)

            # Gripper 상태 유지
            robot.apply_action(
                robot.gripper.forward(
                    action=fsm.gripper
                )
            )

            # Pick & Place FSM 한 스텝 진행
            fsm.advance()

            # 로그
            if step % LOG_INTERVAL == 0:
                print_status(
                    robot,
                    solved,
                    fsm,
                    target_tcp
                )
            step += 1

            # 완료 확인
            if fsm.state >= fsm.DONE_STATE:
                print("[PROCESS] Pick & Place DONE")
                process_state = TASK_DONE

        # ═════════════════════════════════════
        # 3. 완료
        # ═════════════════════════════════════
        elif process_state == TASK_DONE:

            # 일단 아무것도 하지 않음
            pass

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()