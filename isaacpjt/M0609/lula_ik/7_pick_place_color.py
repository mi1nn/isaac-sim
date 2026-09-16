"""
Pick & Place — 색상 인식 연동
    isaac_python 7_pick_place_color.py

흐름
  1) Play 를 누르면 파란/초록 큐브 중 하나가 집을 수 있는 영역 안 랜덤 위치에
     스폰된다 (플레이당 정확히 한 번).
  2) 손목 카메라(RSD455 RGB)가 매 프레임 /rgb2 토픽으로 이미지를 publish 한다.
  3) 로봇은 스폰된 위치로 바로 접근해서 집고 들어올린다 — 여기까지는 색상을
     몰라도 진행한다.
  4) 들어올린 채로 정지하고, 다른 PC 의 색상 검출 노드가 /color_id2 토픽
     (std_msgs/Int32, data: 1=blue, 2=green)으로 알려줄 때까지 기다린다.
     이 단계에 들어온 뒤에 온 값만 쓰고, 같은 색이 연속으로 와야 확정한다.
  5) 응답을 받으면 그 색에 해당하는 바닥 마커(Cube_blue / Cube_green, 스크립트가
     m0609_camera_cube_GB.usd 위에 만든다) 위로 이동해 내려놓는다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ROS2 브리지는 다른 isaacsim 모듈을 import 하기 전에 켜야 한다
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import random
import time

import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from pxr import Usd, UsdPhysics

# 여기서 말하는 rclpy 는 시스템 ROS2 가 아니라 Isaac Sim 이 자체적으로 빌드해 둔 것이다.
# 반드시 isaacsim.ros2.bridge 를 enable 하고 simulation_app.update() 를 호출한 뒤에 import 한다.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from isaacsim.core.api import World
from isaacsim.core.api.materials import PreviewSurface
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube_GB/m0609_camera_cube_GB.usd")

# GB USD 에 들어 있지만 이 스크립트가 따로 만드는 것과 겹쳐서 끄는 prim
#   blue_block, green_block   스폰 큐브(/World/SpawnCube)를 따로 쓴다. 남겨두면 카메라에 찍힌다
#   camera_graph              /rgb 640x640 퍼블리셔. /rgb2 그래프를 따로 만들어서 렌더만 중복된다
UNUSED_PRIM_PATHS = [
    "/World/blue_block",
    "/World/green_block",
    "/World/Graph/camera_graph",
]
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


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

# 시작 자세 — 팔을 위로 뻗은 자세로 대기한다.
# 큐브가 스폰되기 전까지 이 자세로 대기하다가, 스폰된 뒤에 APPROACH 높이로
# 서서히 내려오면서 파지하러 간다 (0,0,0,0,0,0 이 관절을 하나도 굽히지 않은,
# 팔이 곧게 뻗은 기본 자세다)
READY_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
# finger_joint 가 구동 관절이고 나머지 5개는 Mimic 으로 따라온다
# 두 번째 이름은 ParallelGripper 가 요구하는 형식상 필요하다
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

# finger_joint 절대 목표값 (라디안)
#   Physics Inspector 는 도로 표시한다.  0.0 ~ 67.609 deg = 0.0 ~ 1.18 rad
#   CLOSE 값을 기계적 한계(1.18)에 가깝게 잡아야 CUBE_SIZE 처럼 작은 물체도
#   손가락이 실제로 접촉할 때까지 파고든다 — 0.8rad 로는 큐브에 닿기 전에 멈춰서
#   그리퍼가 헛닫히고(파지 실패) 만다.
GRIPPER_OPEN_POS  = 0.0     #   0.0 deg
GRIPPER_CLOSE_POS = 1.05    #  60.2 deg  (기계적 한계 1.18 rad 보다 살짝 여유를 둔다)


# ══════════════════════════════════════════════════════════════
#  색상 마커 & 랜덤 스폰
# ══════════════════════════════════════════════════════════════
# 바닥 마커 두 개의 월드 xy 좌표. 놓을 위치로 그대로 쓴다.
# GB USD 에는 마커가 없어서 m0609_colorcube.usd 의 /Cube_blue, /Cube_green 과 같은 자리·크기로 만든다.
MARKER_POS = {
    "blue":  np.array([0.34358, 0.19214]),
    "green": np.array([0.34358, -0.31902]),
}
MARKER_PRIM_PATHS = {
    "blue":  "/World/Cube_blue",
    "green": "/World/Cube_green",
}
# 마커 한 변의 크기(대략 5.7 x 5.5 cm).
# 스폰 큐브는 반드시 이보다 작아야 마커 위에 놓았을 때 테두리가 보인다.
MARKER_FOOTPRINT_XY = (0.057, 0.055)
MARKER_THICKNESS    = 0.002

# 마커 색 — 채도를 검출 노드 하한(S 80) 아래로 둔다.
# 렌더 후 OpenCV HSV 기준 blue S 50, green S 45 라 마커는 큐브로 세지 않는다
MARKER_RGB = {
    "blue":  np.array([0.58, 0.68, 0.95]),
    "green": np.array([0.58, 0.90, 0.58]),
}

# 검출 노드가 색을 잘 구분하도록 마커보다 채도 높은 순색을 쓴다
BLUE_RGB  = np.array([0.0, 0.0, 1.0])
GREEN_RGB = np.array([0.0, 1.0, 0.0])

# 집을 큐브 한 변 길이 — 마커보다는 작되(MARKER_FOOTPRINT_XY 보다 작아야 한다),
# 그리퍼가 실제로 물 수 있도록 너무 작지 않게 잡는다
CUBE_SIZE = 0.048

# 큐브가 랜덤 스폰되는 영역(xy). 로봇 도달범위 안이면서 두 마커 자리와는 겹치지 않는다.
SPAWN_X_RANGE = (0.20, 0.32)
SPAWN_Y_RANGE = (-0.12, 0.12)
MIN_MARKER_CLEARANCE = 0.10   # 마커 중심에서 이만큼은 떨어뜨려 스폰한다

# 다른 PC 의 색상 검출 결과를 받는 토픽. std_msgs/Int32 로 색 ID 를 보낸다 (1=blue, 2=green)
COLOR_TOPIC = "/color_id1"
COLOR_ID_MAP = {1: "blue", 2: "green"}

# 색상 확정 조건
#   검출 노드는 영상이 올 때마다 publish 하므로, WAIT_DETECT 에 들어오기 전 값은 버린다.
#   접근 중(TCP 0.25 m)에는 바닥 큐브와 마커가 카메라에서 비슷한 거리라 화면 크기가 비슷해
#   마커 색으로 잘못 판별될 수 있다. 들어올린 뒤에는 큐브가 카메라에 약 0.15 m 로 가까워
#   바닥 마커보다 화면에서 몇 배 크게 보인다.
#   COLOR_SETTLE_STEPS   WAIT_DETECT 진입 후 이만큼 지나서 듣기 시작한다 (LIFT 중 영상 제외)
#   COLOR_CONFIRM_COUNT  같은 색이 연속 이 횟수만큼 와야 확정한다
COLOR_SETTLE_STEPS  = 30
COLOR_CONFIRM_COUNT = 5

# 손목 카메라(RSD455 RGB) 렌더 결과를 publish 할 토픽/해상도
CAMERA_PRIM_NAME  = "Camera_OmniVision_OV9782_Color"
RGB_TOPIC         = "rgb1"
CAMERA_RESOLUTION = (640, 480)


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
# link_6 로컬 좌표계에서 손가락 패드 끝까지의 거리 (실측)
#   손가락 패드 범위  0.13632 ~ 0.19671
#   링크 원점 0.14155 는 관절 위치이지 파지면이 아니다
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  목표 높이
# ══════════════════════════════════════════════════════════════
#   PICK_Z    큐브 상단면. 여기서 그리퍼를 닫으면 큐브 옆면을 문다
#   PLACE_Z   놓을 때는 살짝 높게 두어 큐브가 튀지 않도록 한다
#   APPROACH  집기 전 대기 높이
#   LIFT      들고 이동할 높이
PICK_Z          = CUBE_SIZE
PLACE_Z         = CUBE_SIZE + 0.005
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
    """
    Pick & Place 순서를 담은 상태 기계.

      0 APPROACH     스폰된 큐브 위로 접근
      1 DESCEND      큐브까지 하강
      2 GRASP        그리퍼 닫기 (제자리)
      3 LIFT         들어올리기
      4 WAIT_DETECT  색상 검출 결과를 기다리며 제자리 유지
      5 MOVE         놓을 곳(감지된 색의 마커) 위로 이동
      6 LOWER        놓을 높이까지 하강
      7 RELEASE      그리퍼 열기 (제자리)
      8 DONE

    pick 목표는 begin() 으로, place 목표는 set_place() 로 나중에 확정한다 —
    스폰 직후에는 어디로 놓을지 아직 모르기 때문이다.
    """

    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT", "WAIT_DETECT",
             "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {2: "close", 7: "open"}     # 제자리에서 개폐만 하는 단계
    WAIT_DETECT_STATE = 4
    DONE_STATE = 8

    def __init__(self, robot):
        self._robot = robot
        self.reset()

    def reset(self):
        """다음 Play 를 위해 완전히 비운다. begin() 을 부르기 전까지는 대기 상태다"""
        self.state = -1
        self.step = 0
        self.start = None
        self.goal = None
        self.n_steps = MIN_STEPS
        self.gripper = "open"
        self.pick_xy = None
        self.place_xy = None
        self.waypoints = {}

    def begin(self, pick_xy):
        """스폰된 큐브 위치를 받아 pick 구간(0~3)을 확정하고 시작한다"""
        px, py = pick_xy
        self.pick_xy = np.array(pick_xy)
        self.place_xy = None
        self.waypoints = {
            0: np.array([px, py, APPROACH_HEIGHT]),
            1: np.array([px, py, PICK_Z]),
            2: np.array([px, py, PICK_Z]),
            3: np.array([px, py, LIFT_HEIGHT]),
        }
        self.gripper = "open"
        self.state = 0
        self._enter_state()

    def set_place(self, place_xy):
        """감지된 색에 해당하는 마커 위치를 받아 place 구간(5~7)을 확정한다"""
        gx, gy = place_xy
        self.place_xy = np.array(place_xy)
        self.waypoints[5] = np.array([gx, gy, LIFT_HEIGHT])
        self.waypoints[6] = np.array([gx, gy, PLACE_Z])
        self.waypoints[7] = np.array([gx, gy, PLACE_Z])

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        """한 스텝 진행한다"""
        if self.state < 0 or self.state >= self.DONE_STATE:
            return

        if self.state == self.WAIT_DETECT_STATE:
            if self.place_xy is None:
                return          # 색상 응답이 올 때까지 제자리에서 대기
            self.state += 1
            self._enter_state()
            return

        self.step += 1
        if self.step >= self.n_steps:
            self.state += 1
            if self.state >= self.DONE_STATE:
                print(f"   [{self.DONE_STATE}] DONE")
            else:
                self._enter_state()

    def _enter_state(self):
        """
        새 state 로 들어가는 순간 시작점/목표/스텝 수를 한 번에 확정한다.

        advance() 안에서 지연 계산하면(= 이번 프레임엔 아직 이전 목표를 들고 있다가
        다음 프레임에야 시작점을 잡으면), current_target() 이 보간 없이 goal 을
        그대로 돌려주는 프레임이 한 번 생긴다. Drive 강성이 워낙 커서(1e8) 그 프레임에
        팔이 목표로 바로 붙어버려 순간이동처럼 보인다. 그래서 state 가 바뀌는 바로 그
        순간 시작점을 현재 TCP 로 못박아 둔다.
        """
        self.step = 0
        self.start = get_tcp_pose(self._robot)

        if self.state == self.WAIT_DETECT_STATE:
            self.goal = self.start
            self.n_steps = 1
            print(f"   [{self.state}] {self.NAMES[self.state]:11s}"
                  f" {COLOR_TOPIC} 응답 대기 중 ...")
            return

        self.goal = self.waypoints[self.state]
        self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

        if self.state in self.GRIPPER_STATES:
            self.n_steps = GRIPPER_WAIT
            dist = 0.0
        else:
            self.n_steps, dist = steps_for(self.start, self.goal)

        print(f"   [{self.state}] {self.NAMES[self.state]:11s}"
              f" goal {vec(self.goal)}"
              f"  {dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")


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
        self._spawn_cube = None
        self._spawn_material = None

    # ── 프레임워크 규약 ──────────────────────────────────
    def set_up_scene(self, scene):
        """world.reset() 안에서 자동으로 불린다"""
        super().set_up_scene(scene)
        self._load_usd()
        self._create_markers(scene)
        self._setup_arm_drives()
        self._register_robot(scene)
        self._setup_camera_publisher()
        self._create_spawn_object(scene)
        print("   scene        ready")

    # ── 우리가 나눈 단계 ─────────────────────────────────
    def _load_usd(self):
        """
        reference 가 아니라 sublayer 로 합친다.

        이 USD 의 defaultPrim 은 /World 인데, 조명(/Environment/defaultLight)은
        /World 의 자식이 아니라 파일 최상위(pseudo-root)의 형제 prim 이다.
        reference 는 defaultPrim 서브트리만 끌어오므로 조명이 빠진다.
        sublayer 는 파일 최상위를 그대로 합치므로 조명도 함께 들어온다.
        """
        stage = omni.usd.get_context().get_stage()
        stage.GetRootLayer().subLayerPaths.append(USD_PATH)

        # 로드 직후, 그래프·물리가 올라오기 전에 끈다
        for path in UNUSED_PRIM_PATHS:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)
                print(f"   disabled     {path}")

        for _ in range(15):
            simulation_app.update()

        if not stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
            raise RuntimeError(f"USD load failed (no {ROBOT_PRIM_PATH}): {USD_PATH}")

        print("   USD          loaded")

    def _create_markers(self, scene):
        """놓을 자리를 표시하는 얇은 색 판. 충돌이 없는 시각용이라 큐브는 바닥에 그대로 놓인다"""
        for name, xy in MARKER_POS.items():
            scene.add(
                VisualCuboid(
                    prim_path=MARKER_PRIM_PATHS[name],
                    name=f"marker_{name}",
                    position=np.array([xy[0], xy[1], MARKER_THICKNESS / 2.0]),
                    scale=np.array([*MARKER_FOOTPRINT_XY, MARKER_THICKNESS]),
                    size=1.0,
                    color=MARKER_RGB[name],
                )
            )
        print(f"   markers      blue {vec(MARKER_POS['blue'])}  green {vec(MARKER_POS['green'])}")

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

    def _setup_camera_publisher(self):
        """손목 RGB 카메라를 RGB_TOPIC 으로 publish 하는 OmniGraph 를 만든다"""
        camera_path = find_prim_path(ROBOT_PRIM_PATH, CAMERA_PRIM_NAME)
        if camera_path is None:
            raise RuntimeError(f"'{CAMERA_PRIM_NAME}' not found under {ROBOT_PRIM_PATH}")
        setup_camera_ros_graph(camera_path)
        print(f"   ROS2 camera  /{RGB_TOPIC}  <-  {camera_path}")

    def _create_spawn_object(self, scene):
        """집을 큐브를 하나 만들어 둔다. 색/위치는 spawn_random_object() 가 매 Play 마다 바꾼다"""
        self._spawn_material = PreviewSurface(
            prim_path="/World/Looks/spawn_cube_material", color=BLUE_RGB
        )
        self._spawn_cube = scene.add(
            DynamicCuboid(
                prim_path="/World/SpawnCube",
                name="spawn_cube",
                size=CUBE_SIZE,
                visual_material=self._spawn_material,
                position=np.array([SPAWN_X_RANGE[0], SPAWN_Y_RANGE[0], CUBE_SIZE / 2.0]),
            )
        )
        print(f"   spawn cube   size {CUBE_SIZE} m  (marker ~{MARKER_FOOTPRINT_XY})")

    # ── 랜덤 스폰 ─────────────────────────────────────────
    def spawn_random_object(self):
        """플레이당 한 번, 랜덤 색 + 랜덤 위치로 큐브를 놓는다. (xy, 실제 색) 을 돌려준다"""
        color_name = random.choice(["blue", "green"])
        rgb = BLUE_RGB if color_name == "blue" else GREEN_RGB
        xy = self._sample_spawn_xy()

        self._spawn_cube.set_world_pose(
            position=np.array([xy[0], xy[1], CUBE_SIZE / 2.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self._spawn_cube.set_linear_velocity(np.zeros(3))
        self._spawn_cube.set_angular_velocity(np.zeros(3))
        self._spawn_material.set_color(rgb)
        self._spawn_cube.set_visibility(True)

        print(f"   spawned      {color_name:5s} (실제색)  xy {vec(xy)}")
        return xy, color_name

    def hide_spawn_object(self):
        """정지를 누르면 남아있던 큐브를 안 보이게 치운다"""
        self._spawn_cube.set_visibility(False)

    @staticmethod
    def _sample_spawn_xy():
        """스폰 영역 안에서, 두 마커와는 충분히 떨어진 xy 를 뽑는다"""
        xy = None
        for _ in range(50):
            xy = np.array([
                np.random.uniform(*SPAWN_X_RANGE),
                np.random.uniform(*SPAWN_Y_RANGE),
            ])
            if all(np.linalg.norm(xy - m) > MIN_MARKER_CLEARANCE for m in MARKER_POS.values()):
                return xy
        return xy

    @property
    def robot(self):
        return self._robot


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
#  ROS2 — 카메라 publish, 색상 결과 subscribe
# ══════════════════════════════════════════════════════════════
def setup_camera_ros_graph(camera_prim_path):
    """카메라 prim 을 렌더링해 RGB_TOPIC 으로 publish 하는 OmniGraph 를 만든다"""
    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": "/RGB2CameraGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("CreateRenderProduct.inputs:width", CAMERA_RESOLUTION[0]),
                ("CreateRenderProduct.inputs:height", CAMERA_RESOLUTION[1]),
                ("CameraHelperRgb.inputs:topicName", RGB_TOPIC),
                ("CameraHelperRgb.inputs:type", "rgb"),
                ("CameraHelperRgb.inputs:frameId", "wrist_camera"),
            ],
        },
    )
    return graph


class ColorSubscriber(Node):
    """
    다른 PC 의 색상 검출 노드가 COLOR_TOPIC(Int32, 1=blue/2=green)에 publish 하는 결과를 받는다.

    검출 노드는 영상이 올 때마다 publish 한다.
    start_listening() 이후에 온 값만 쓰고, 같은 색이 COLOR_CONFIRM_COUNT 번 연속 와야
    color 에 확정한다. 확정되면 다시 듣지 않는다.
    """

    def __init__(self):
        super().__init__("color_result_subscriber")
        self.reset()
        self.create_subscription(Int32, COLOR_TOPIC, self._on_color, 10)

    def reset(self):
        """듣기를 멈추고 모아 둔 값을 모두 버린다"""
        self.color = None
        self.listening = False
        self._candidate = None
        self._count = 0

    def start_listening(self):
        self.reset()
        self.listening = True
        print(f"   [ROS2] {COLOR_TOPIC}  listening (연속 {COLOR_CONFIRM_COUNT}회 같은 색이면 확정)")

    def _on_color(self, msg):
        if not self.listening:
            return

        value = COLOR_ID_MAP.get(msg.data)
        if value is None:
            print(f"   [ROS2] {COLOR_TOPIC}  ->  unknown id {msg.data} (무시)")
            return

        if value == self._candidate:
            self._count += 1
        else:
            self._candidate = value
            self._count = 1

        if self._count >= COLOR_CONFIRM_COUNT:
            self.color = value
            self.listening = False
            print(f"   [ROS2] {COLOR_TOPIC}  ->  {msg.data} ('{value}')  확정")


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


def print_plan_info(target_quat):
    """Pick & Place 계획을 확인한다"""
    R = quat_to_matrix(target_quat)

    section("PLAN")
    print(f"   spawn area   x {SPAWN_X_RANGE}  y {SPAWN_Y_RANGE}")
    print(f"   cube size    {CUBE_SIZE} m   (marker ~{MARKER_FOOTPRINT_XY})")
    print(f"   blue  marker {vec(MARKER_POS['blue'])}")
    print(f"   green marker {vec(MARKER_POS['green'])}")
    print()
    print(f"   approach z   {APPROACH_HEIGHT}")
    print(f"   pick z       {PICK_Z}")
    print(f"   lift z       {LIFT_HEIGHT}")
    print(f"   place z      {PLACE_Z}")
    print()
    print(f"   tcp speed    {TCP_SPEED} m/step")
    print(f"   gripper      open {GRIPPER_OPEN_POS}  close {GRIPPER_CLOSE_POS}")
    print(f"   gripper wait {GRIPPER_WAIT} steps")
    print()
    print(f"   tool   +Z    {vec(R @ np.array([0, 0, 1]))}   approach direction")
    print(f"   finger +X    {vec(R @ np.array([1, 0, 0]))}   finger direction")


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
    name = fsm.NAMES[fsm.state] if fsm.state >= 0 else "IDLE"

    if not solved:
        print(f"   {name:11s} IK FAILED  target {vec(target_tcp)}")
        return

    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
    print(f"   {name:11s} tcp {vec(tcp)}   finger {finger:+.4f}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60
SPAWN_DELAY_SEC = 0.0   # Play 를 누르면 바로 큐브를 스폰한다 (이동 자체는 보간으로 천천히)


def main():
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
    print_plan_info(target_quat)

    section("ROS2")
    rclpy.init()
    color_sub = ColorSubscriber()
    print(f"   subscribed   {COLOR_TOPIC}  (std_msgs/Int32: 1=blue | 2=green)")

    section("RUN")
    print("   press Play in the viewport\n")

    fsm = PickPlaceFSM(robot)
    was_playing = False
    spawned = True          # 다음 Play 전까지는 스폰할 필요 없음
    play_started_at = 0.0
    step = 0
    wait_step = 0           # WAIT_DETECT 에 머문 스텝 수

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)
        rclpy.spin_once(color_sub, timeout_sec=0.0)

        is_playing = world.is_playing()

        # Play 를 누른 순간 대기 자세로 되돌린다. 큐브는 아직 스폰하지 않고
        # SPAWN_DELAY_SEC 뒤에 스폰한다 (플레이당 한 번)
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)

            fsm.reset()
            color_sub.reset()
            wait_step = 0
            spawned = False
            play_started_at = time.time()
            step = 0
            print()

        if is_playing:
            if not spawned:
                # 큐브가 나오기 전까지는 시작 자세를 유지하며 대기한다
                set_ready_pose(robot)
                robot.apply_action(robot.gripper.forward(action="open"))

                if time.time() - play_started_at >= SPAWN_DELAY_SEC:
                    pick_xy, _ = task.spawn_random_object()
                    fsm.begin(pick_xy)
                    spawned = True
            else:
                # 들어올린 채 대기 중이고 아직 놓을 곳이 안 정해졌다면, 색상 응답을 확인한다
                if fsm.state == fsm.WAIT_DETECT_STATE and fsm.place_xy is None:
                    # 들어올린 뒤 흔들림이 멎고 새 영상이 도착할 시간을 준 다음 듣기 시작한다
                    wait_step += 1
                    if wait_step == COLOR_SETTLE_STEPS:
                        color_sub.start_listening()

                    if color_sub.color is not None:
                        place_xy = MARKER_POS[color_sub.color]
                        fsm.set_place(place_xy)
                        print(f"   place target {color_sub.color:5s} marker  xy {vec(place_xy)}")

                # 팔 — 이번 스텝의 목표를 보간으로 구해 IK 로 푼다
                target_tcp = fsm.current_target()
                flange_target = tcp_to_flange(target_tcp, target_quat)

                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target,
                    target_orientation=target_quat,
                )
                if solved:
                    robot.apply_action(action)

                # 그리퍼 — 현재 단계가 정한 상태를 유지한다
                robot.apply_action(robot.gripper.forward(action=fsm.gripper))

                fsm.advance()

                if step % LOG_INTERVAL == 0:
                    print_status(robot, solved, fsm, target_tcp)
                step += 1

        # 정지를 누른 순간 스폰돼 있던 큐브를 치운다
        if was_playing and not is_playing:
            task.hide_spawn_object()

        was_playing = is_playing

    color_sub.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
