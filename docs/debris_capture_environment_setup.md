# Debris Capture — 수동 파지 테스트 환경 구성 작업 로그

> 이 문서는 최종 결과 문서가 아니라 **작업 로그 + 진행도**입니다.
> 작업 단계가 끝날 때마다 계속 갱신합니다.
> 실제로 실행/확인한 내용만 기록하고, 확인하지 못한 것은 `아직 확인 중`으로 남깁니다.

- 최초 작성: 2026-09-17
- 대상 환경: `debris_capture_visual` (`env.robot=canadarm3+kinova300_large`)
- 작업 브랜치: `feature/grip`

---

## 1. 작업 목표

로봇팔이 장착된 위성에서, **사람이 직접 관절과 그리퍼를 조작하여** 부유 중인
MEP(debris)의 `gripper_fixture` 손잡이를 실제로 파지할 수 있는지 검증한다.

```
안정적인 Isaac Sim 환경
        +
MEP 초기 정지 상태
        +
Robot 초기 정지 상태
        +
수동 Joint Control
        +
수동 Gripper Control
        +
실제 Collision (visual 겹침이 아님)
        =
Manual Grasp 검증
```

비목표(이번 단계에서 하지 않는 것):
- 위성 본체(base)의 자세 제어 / 미션 수행
- 완전한 6-DOF free-floating 랑데부 동역학
- 학습(RL) 파이프라인

---

## 2. 진행도

- [x] Step 1. 현재 환경 구조 분석
- [x] Step 1-1. MEP(debris) Prim / 배치 / USD 참조 구조 확인
- [x] Step 2. Robot 자동 움직임 원인 분석 (코드 레벨)
- [x] Step 3. Simulation 초기 상태 수정 (PAUSED 시작) — 코드 작성 완료 / **실행 검증 미실행**
- [x] Step 4. MEP physics 초기 정지 상태 수정 — 코드 작성 완료 / **실행 검증 미실행**
- [x] Step 5. Collision approximation 수정 — 코드 작성 완료 / **실행 검증 미실행**
- [x] Step 6. 수동 Joint 제어 구성 — 코드 작성 완료 / **실행 검증 미실행**
- [x] Step 7. 수동 Gripper 제어 구성 — 코드 작성 완료 / **실행 검증 미실행**
- [x] Step 7-1. Play 폭발 원인 정밀 측정 (USD 실측 + FK 계산) — 코드 작성 완료 / **실행 검증 미실행**
- [ ] Step 8. Manual Grasp 테스트 (실기 검증)
- [ ] Step 9. 최종 검증

> ⚠️ **실행 환경 제약**
> 코드 분석/수정은 macOS 개발 머신에서 수행되었고, 이 머신에는 Isaac Sim /
> Isaac Lab / simforge 가 설치되어 있지 않습니다.
> (`python3 -c "import isaaclab"` → `ModuleNotFoundError`)
> 따라서 **실제 시뮬레이션 실행 검증은 Isaac Sim이 설치된 머신(`/home/rokey/...`)에서
> 사용자가 직접 수행**해야 하며, 그 결과를 이 문서의 "테스트 결과" 절에 기록합니다.
> 아직 실행하지 않은 항목은 `미실행`으로 표기합니다.

---

## 3. Step 1 — 현재 환경 구조 분석 (완료)

### 3-1. 환경 정의 위치

| 항목 | 경로 |
|---|---|
| Task (env) 정의 | `project/srb/tasks/manipulation/debris_capture/task.py` |
| Visual variant | `project/srb/tasks/manipulation/debris_capture/task_visual.py` |
| Task 등록 | `project/srb/tasks/manipulation/debris_capture/__init__.py` |
| Manipulation 베이스 env | `project/srb/core/env/manipulation/env.py` |
| 공통 베이스 env 구현 | `project/srb/core/env/common/base/direct/impl.py` |
| 공통 베이스 env cfg | `project/srb/core/env/common/base/env_cfg.py` |
| 공통 event cfg | `project/srb/core/env/common/base/event_cfg.py` |
| CLI / agent 루프 | `project/srb/__main__.py` |

`debris_capture_visual` 은 `__init__.py` 의 `register_srb_tasks()` 로
`{BASE_TASK_NAME}_visual` 형태로 등록됩니다. 즉 `debris_capture` 태스크의
카메라 관측이 추가된 변형입니다.

### 3-2. 로봇 구성 (`canadarm3+kinova300_large`)

| 항목 | 경로 | 요약 |
|---|---|---|
| Canadarm3 | `project/srb/assets/robot/manipulation/canadarm3.py` | 7-DOF serial manipulator |
| Kinova300Large | `project/srb/assets/object/tool/kinova_gripper.py` | 3-finger 그리퍼, `scale=(4.2, 4.2, 4.2)` |

- 팔과 그리퍼는 **별개의 Articulation** 이며, `env_cfg.py` 의
  `joint_assemblies` (`RobotAssembler`) 가 `canadarm3_large_7` (flange) ↔
  그리퍼 `base` 를 **fixed joint** 로 붙입니다 (`mask_all_collisions=True`).
- Scene entity 이름: `robot`(팔), `end_effector`(그리퍼), `debris`(MEP), `satellite`.

Canadarm3 초기 joint 자세 (`canadarm3.py:48-58`):

| Joint | 값 |
|---|---|
| `canadarm3_large_joint_1` | 50° |
| `canadarm3_large_joint_2` | 0° |
| `canadarm3_large_joint_3` | 55° |
| `canadarm3_large_joint_4` | 75° |
| `canadarm3_large_joint_5` | -30° |
| `canadarm3_large_joint_6` | 0° |
| `canadarm3_large_joint_7` | 0° |

Kinova300Large 초기 joint 자세 (`kinova_gripper.py:42-47`): 모든 finger /
finger_tip 관절 `0.2 rad` (= OPEN).
`close = 1.2 rad`, `open = 0.2 rad` (`kinova_gripper.py:66-74`).

### 3-3. Action 구성

| 대상 | Action group | 의미 |
|---|---|---|
| Canadarm3 | `InverseKinematicsActionGroup` (`DifferentialInverseKinematicsActionCfg`, `use_relative_mode=True`, `ik_method="dls"`, `scale=0.1`) | 6-DOF **상대** twist |
| Kinova300Large | `JointPositionBinaryActionGroup` (`BinaryJointPositionActionCfg`) | 1-DOF 이진 open/close |

→ **현재 action space 로는 관절 하나하나를 직접 지정할 수 없습니다.**
팔은 엔드이펙터 twist(IK)로만, 그리퍼는 open/close 이진값으로만 제어됩니다.
요구사항 7(관절별 수동 제어)을 충족하려면 action manager를 우회하는
별도의 제어 경로가 필요합니다. → Step 6 에서 처리.

### 3-4. MEP(debris) / satellite 에셋 구조

`task.py:143-157` 에서 debris 를 다음과 같이 spawn 합니다.

```python
self.scene.debris = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/debris",
    spawn=UsdFileCfg(
        usd_path=SRB_ASSETS_DIR_SPACE.joinpath("debris_v3.usd").as_posix(),
        scale=(1.5, 1.5, 1.5),
        collision_props=CollisionPropertiesCfg(),
        rigid_props=RigidBodyPropertiesCfg(),
        mass_props=MassPropertiesCfg(density=1000.0),
        activate_contact_sensors=True,
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(-0.105728, 13.90236, 11.82852),
        rot=(0.095277, -0.700659, 0.095277, -0.700659),
    ),
)
```

USD 참조 체인 (`assets/space_asset/`, `SRB_ASSETS_DIR_SPACE`):

```
debris_v3.usd      (1.4 KB)  ── references ./3asset_v3.usd  →  /World/envs/env_0/debris
satellite_v3.usd   (716 B)   ── references ./3asset_v3.usd  →  /World/envs/env_0/satellite
3asset_v3.usd      (14 KB)   ── 저장된 "스테이지 스냅샷"
                                 (debris / satellite / robot / end_effector /
                                  kinova300 / canadarm3 / skydome / 카메라 /
                                  FixedJoint / physics 설정까지 포함)
   └ debris ── references ./mep_combined.usd
mep_combined.usd   (3.5 KB)  ── references /home/rokey/space_asset/mep.usd
                                 + PhysicsRigidBodyAPI / PhysicsMassAPI /
                                   PhysicsCollisionAPI 부여
mep.usd            (11 MB)   ── 실제 지오메트리
                                 (_Ares1..., Fermi_Gamma_ray_Large_Area_Space_Telescope,
                                  gripper_fixture, imagetostl_mesh2, DiscMount,
                                  Cylinder_01, env_light)
```

확인된 사실 (binary USDC 에 대해 `strings` 로 확인):

1. `mep_combined.usd` 의 참조가 **절대경로 `/home/rokey/space_asset/mep.usd`** 입니다.
   저장소 안의 `assets/space_asset/mep.usd` 를 가리키지 않습니다.
   Isaac Sim 머신에서 그 절대경로가 유효한 동안에는 동작하지만,
   경로가 바뀌면 조용히 깨집니다. → **확인 필요 항목**.
2. `mep_combined.usd` / `mep.usd` / `3asset_v3.usd` 어디에도
   **`UsdPhysicsMeshCollisionAPI` 의 approximation 이 author 되어 있지 않습니다.**
   (`approximation`, `convexHull`, `convexDecomposition`, `sdf`,
   `meshSimplification` 문자열 모두 미검출)
   → PhysX 기본값인 **convexHull** 이 적용됩니다.
   → `gripper_fixture` 손잡이가 **속이 꽉 찬 볼록 덩어리**가 되어
   그리퍼 손가락이 손잡이 안쪽으로 들어갈 수 없습니다. → 파지 불가의 유력한 원인.
3. `3asset_v3.usd` 에는 `physxRigidBody:sleepThreshold` 가 author 되어 있습니다.
   (정지 시 sleep → 접촉 시 wake 라는 원하는 거동과 부합)

비교: Canadarm3 는 `mesh_collision_props=MeshCollisionPropertiesCfg(
mesh_approximation="convexDecomposition")` 를 명시하고 있습니다
(`canadarm3.py:35-37`). debris 에는 같은 설정이 없습니다.

---

## 4. Step 2 — 자동 움직임 원인 분석 (코드 레벨, 완료)

> 아래는 **코드를 읽고 특정한 원인**입니다. 각 항목의 실제 재현/해소 여부는
> Isaac Sim 실행 후 "테스트 결과" 절에 기록합니다.

### 원인 A — `srb agent zero` 는 구조적으로 즉시 Play 후 무한 step

`project/srb/__main__.py`:

- `run_agent_with_env()` 의 `hydra_main()` 안에서 `env.reset()` 이 호출됩니다 (`__main__.py:211`).
- 이어서 `zero_agent()` (`__main__.py:354-375`) 가
  `while sim_app.is_running(): env.step(action)` 루프를 즉시 돕니다.

→ 프로그램이 뜨자마자 물리가 진행됩니다. 일시정지 지점이 없습니다.
→ "실행 후 사용자가 직접 Play" 를 만들려면 **step 루프를 돌지 않는 별도의
agent 모드**가 필요합니다. `zero`/`rand`/`teleop` 중 어느 것도 이 동작을 하지 않습니다.

### 원인 B — zero action 이 그리퍼를 **CLOSE** 로 명령함

`Kinova300Large` 의 action 은 `BinaryJointPositionActionCfg` 입니다.
Isaac Lab 의 `BinaryJointAction.process_actions()` 는
`binary_mask = actions < 0` 로 판정하고 `torch.where(binary_mask, open, close)` 를 씁니다.

`zero_agent` 의 action 은 **정확히 0.0** 이므로 `0 < 0 == False` → **close 명령**입니다.

→ `srb agent zero` 로 띄우면 그리퍼가 `0.2 rad`(open) 에서 `1.2 rad`(close) 로
즉시 닫히기 시작합니다. stiffness 가 `1_200_000.0` 이므로 매우 강하게 움직입니다.
이것이 "실행 직후 로봇이 움직인다" 의 직접적인 원인 중 하나입니다.

(참고: SRB 의 `map_cmd_to_action` 은 `-1.0 if event else 1.0` 을 반환하므로
teleop 경로에서는 `-1 = open`, `+1 = close` 규약입니다. 즉 zero 는 close 쪽입니다.)

### 원인 C — reset 이벤트가 팔 관절을 ±5° 랜덤화

`project/srb/core/env/manipulation/env.py:53-62`:

```python
randomize_robot_joints: EventTermCfg = EventTermCfg(
    func=reset_joints_by_offset,
    mode="reset",
    params={
        "asset_cfg": SceneEntityCfg("robot"),
        "position_range": (-deg_to_rad(5.0), deg_to_rad(5.0)),
        "velocity_range": (0.0, 0.0),
    },
)
```

→ reset 마다 팔이 설정된 초기 자세에서 최대 ±5° 어긋난 위치로 배치됩니다.
"초기 joint position 을 명확하게 지정한다" 요구와 충돌합니다.

### 원인 D — reset 이벤트가 MEP 에 랜덤 pose + **랜덤 속도**를 부여

`project/srb/tasks/manipulation/debris_capture/task.py:82-104`:

```python
"pose_range": {"x": (-0.25, 0.25), "y": (-0.25, 0.25), "z": (-0.25, 0.25),
               "roll": (-pi, pi), "pitch": (-pi, pi), "yaw": (-pi, pi)},
"velocity_range": {"x": (-0.25, -0.15), "y": (-0.05, 0.05), "z": (-0.05, 0.05),
                   "roll/pitch/yaw": (-10°, 10°)},
```

→ MEP 가 매 reset 마다 **임의 위치 · 완전 임의 자세 · 초기 속도**를 갖습니다.
이것이 "MEP 가 계속 떠다닌다"의 원인입니다.
중력은 원인이 아닙니다 — `Domain.ORBIT` 의 `gravity_magnitude` 는 `0.0` 이며
(`srb/core/domain.py:40-41`), `sim.gravity = (0, 0, -0.0)` 으로 설정됩니다.
`randomize_gravity` 이벤트도 ORBIT 에서는 `None` 으로 비활성화됩니다.

### 원인 E — 예외가 삼켜지고 있음

`project/srb/core/env/common/base/direct/impl.py:342-349`:

```python
def _apply_action(self):
    try:
        ...
    except Exception as e:
        logging.error(f"Failed to apply action: {e}")
```

→ action 적용 중 예외가 발생해도 로그 한 줄만 남기고 계속 진행합니다.
사용자가 보고한 "오류를 출력한다" 가 이 경로일 가능성이 있습니다.
**실제 오류 메시지 원문이 필요합니다.** → 확인 필요 항목.

### 아직 확인하지 못한 것

- 사용자가 본 오류 메시지의 정확한 원문 (원인: 아직 확인 중)
- `RobotAssembler` fixed joint 생성 시 물리 스냅으로 인한 초기 충격 여부
  (`_update_assembly_fixed_joint_transforms` 가 완화하려는 대상) — 아직 확인 중
- `gripper_fixture` 의 실제 치수 대비 Kinova300Large(`scale=4.2`) 손가락 개폐 폭 — 아직 확인 중

---

## 5. 실행 명령어

### 기존 (문제 재현)

```bash
srb agent zero --env debris_capture_visual \
  --kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" \
  env.robot=canadarm3+kinova300_large
```

### 수정 후 (수동 파지 테스트용)

```bash
srb agent manual --env debris_capture_visual \
  --kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" \
  env.robot=canadarm3+kinova300_large
```

옵션:

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--joint_step <deg>` | `1.0` | 키 한 번당 선택된 관절이 움직이는 각도 |
| `--autoplay` | off | 붙이면 PAUSED 로 시작하지 않고 바로 물리 진행 |

`srb agent zero` 도 그대로 동작하지만, zero action 이 그리퍼 CLOSE 로 해석되는
문제(아래 원인 B)가 있으므로 **수동 파지 테스트에는 `manual` 을 사용**합니다.

---

## 6. 변경 파일

| 파일 | 변경 내용 | 이유 |
|---|---|---|
| `project/srb/tasks/manipulation/debris_capture/task.py` | `EventCfg.randomize_obj_state` 의 `pose_range` / `velocity_range` 를 전부 `(0.0, 0.0)` 으로 변경 | MEP 를 지정된 `init_state` 자세에 **정지 상태**로 배치. Kinematic 으로 만들지 않고 dynamic rigid body 로 유지 (원인 D) |
| `project/srb/tasks/manipulation/debris_capture/task.py` | `EventCfg.randomize_robot_joints = None` 추가 | `ManipulationEventCfg` 의 ±5° 관절 랜덤화 비활성화 → 팔이 항상 지정된 초기 자세에서 시작 (원인 C) |
| `project/srb/tasks/manipulation/debris_capture/task.py` | debris spawn 에 `mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="convexDecomposition")` 추가 | MEP USD 에 approximation 이 없어 PhysX 기본값 convexHull 로 떨어짐 → `gripper_fixture` 가 볼록 덩어리가 되어 파지 불가 (문제 4) |
| `project/srb/tasks/manipulation/debris_capture/task.py` | debris `init_state` 에 `lin_vel=(0,0,0)`, `ang_vel=(0,0,0)` 명시 | 요구사항(초기 속도 0)을 코드에 명시적으로 기록 |
| `project/srb/tasks/manipulation/debris_capture/task.py` | 미사용이 된 `deg_to_rad` import 제거 | 랜덤화 범위 제거로 미사용 |
| `project/srb/interfaces/manual.py` | **신규 파일**. `ManualJointControlInterface` — 키보드 기반 관절 단위 제어 + 그리퍼 단계 제어 + 파지 진단 출력 | 기존 action space (IK twist + binary gripper) 로는 관절 개별 지정이 불가능 (3-3 절) |
| `project/srb/__main__.py` | `manual_agent()` 신규 추가 | `env.step()` 을 호출하지 않고 sim 을 직접 step → action term / reward / termination / auto-reset 이 전혀 개입하지 않음 |
| `project/srb/__main__.py` | `srb agent manual` 서브커맨드 및 `--joint_step`, `--autoplay` 인자 등록 | CLI 노출 |
| `project/srb/__main__.py` | 기본 `L → env.reset` 키 바인딩 대상에서 `manual` 제외 | `manual` 이 자체적으로 `L` 을 바인딩 (reset + target 재동기화) |
| `docs/debris_capture_environment_setup.md` | **신규 파일**. 이 작업 로그 | 요구사항 12 |
| `project/srb/core/env/common/base/direct/impl.py` | `_reset_idx` 에서 `_update_assembly_fixed_joint_transforms` 호출 **직전**에 `physics_sim_view.update_articulations_kinematic()` 추가 | reset 이 쓴 관절 위치가 link transform 에 반영되기 전에 flange 포즈를 읽어 그리퍼를 엉뚱한 곳에 배치 → Play 시 fixed joint 가 폭발적으로 스냅 (문제 6) |
| `project/srb/core/env/common/base/direct/impl.py` | `_align_joint_assemblies()` 헬퍼 신설 (kinematics 전파 → 배치 → **root 속도 0**), `__post_init__()` 에서 1회 호출, 배치 거리 로그 추가 | 첫 reset 이전부터 fixed joint 위반이 0 이 되도록. 옮기면서 속도를 지우지 않으면 스냅 운동량이 다시 joint 로 들어감 (문제 8-A) |
| `project/srb/assets/object/tool/kinova_gripper.py` | `Kinova300Large` 게인 하향: `stiffness` 1.2e6→5000, `damping` 1e4→100, `effort_limit_sim` 2000→50, `velocity_limit_sim=5.0` 추가, `solver_velocity_iteration_count` 0→1, `mass_props(density=1000)` 제거 | author 된 `physics:mass` 가 density 를 이기므로 그리퍼는 74배 무거워진 적이 없음 → 게인이 실제 관성 대비 10⁶배 과함 (문제 8-B) |
| `project/srb/assets/object/tool/kinova_gripper.py` | `frame_tool_centre_point` `0.64` → `0.672` | 0.16 × **4.2** 이어야 함 (기존 값은 4배 기준). TCP 가 3.2 cm 짧게 잡혀 있었음 (문제 10) |
| `project/srb/assets/robot/manipulation/canadarm3.py` | `velocity_limit_sim=5.0` 추가, `fix_root_link` 관련 주석 명시 | 팔에도 PhysX 레벨 속도 상한 부여. 베이스는 USD `root_joint` 로 이미 고정 (가설 검증 요약 #1) |
| `project/srb/tasks/manipulation/debris_capture/task.py` | scenery(VenusExpress) 에 `collision_props=CollisionPropertiesCfg(collision_enabled=False)` | 붐(link 3)이 태양전지판을 1.4 mm 스쳐 Play 시 정적 충돌체가 팔을 밀어냄 (문제 9) |
| `project/srb/tasks/manipulation/debris_capture/task.py` | satellite spawn 에 `mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="convexHull")` 추가 | GOES_R 메시 25개가 approximation 없이 dynamic body 에 붙어 PhysX 에러를 25줄씩 뱉음 (문제 7) |
| `project/srb/assets/robot/manipulation/canadarm3.py`, `project/srb/assets/object/tool/kinova_gripper.py` | `effort_limit` → `effort_limit_sim`, `velocity_limit` 제거 | Isaac Lab deprecation 경고 제거. implicit actuator 는 `velocity_limit` 을 애초에 쓰지 않으므로 거동 변화 없음 (문제 7) |

---

## 7. 발생한 문제

### 문제 1 — Simulation 이 실행 직후 자동 Play 됨

- **현상**: 프로그램 실행 즉시 물리 시뮬레이션이 진행됨.
- **원인**: `zero_agent()` 가 `env.reset()` 직후 `while sim_app.is_running(): env.step(...)`
  루프를 돌기 때문. 일시정지 지점이 없음. (`srb/__main__.py:211, 354-375`)
- **해결**: `srb agent manual` 추가 (`srb/__main__.py`).
  `env.reset()` 직후 `sim.pause()` 를 호출하고, 다음 루프를 돕니다.

  ```python
  while sim_app.is_running():
      if not sim.is_playing():
          sim.render()          # UI 만 갱신, 물리 진행 없음
          continue
      controller.apply()        # joint position target 기록
      scene.write_data_to_sim()
      sim.step(render=False)
      ...
  ```

  `env.step()` 을 전혀 호출하지 않으므로 action term / reward / termination /
  auto-reset 이 개입하지 않습니다.
- **검증**: 미실행

### 문제 2 — 로봇팔/그리퍼가 실행 직후 움직임

- **현상**: 실행 직후 그리퍼·팔이 움직이고 오류가 출력됨.
- **원인 (코드 분석)**:
  - zero action → `BinaryJointAction` 이 CLOSE 로 해석 (원인 B)
  - reset 이벤트가 팔 관절을 ±5° 랜덤화 (원인 C)
  - 오류 원문은 **아직 확인 중** (원인 E 의 예외 삼킴 경로 의심)
- **해결**:
  - `randomize_robot_joints = None` 로 ±5° 랜덤화 제거 (원인 C)
  - `manual` 에이전트는 binary gripper action term 을 아예 사용하지 않고,
    그리퍼 관절 target 을 `open_command_expr` 값(= `0.2 rad`, OPEN)으로 초기화 (원인 B)
  - 팔의 joint position target 은 **측정값이 아니라 `default_joint_pos`** 로 초기화.
    랜덤화를 껐으므로 target == 실제값 → 오차 0 → 첫 스텝에서 움직임 없음.
- **검증**: 미실행
- **남은 확인**: 오류 메시지 원문. 실행 시 터미널 로그에서
  `Failed to apply action:` 로 시작하는 줄이 있으면 그 전문을 이 문서에 붙여 주세요.
  (`manual` 에이전트에서는 action 을 적용하지 않으므로 이 경로 자체가 실행되지 않습니다.
  즉 `manual` 에서 오류가 사라진다면 원인이 action term 쪽임이 확정됩니다.)

### 문제 3 — MEP 가 계속 떠다님

- **현상**: Simulation 시작 시 MEP 가 임의 자세로 배치되어 계속 이동/회전.
- **원인**: `EventCfg.randomize_obj_state` 가 매 reset 마다 랜덤 pose + 랜덤 초기
  속도를 부여 (원인 D). 중력은 무관 (ORBIT = 0 g).
- **해결**: `randomize_obj_state` 의 `pose_range` / `velocity_range` 를 모두
  `(0.0, 0.0)` 으로 변경. `reset_root_state_uniform` 은
  `default_root_state + 랜덤샘플` 을 쓰므로, 범위가 0 이면 결과는
  **`init_state` 의 pose + 속도 0** 입니다.
  `init_state` 에도 `lin_vel=(0,0,0)`, `ang_vel=(0,0,0)` 을 명시했습니다.
  **Kinematic 으로 만들거나 고정하지 않았습니다** — 여전히 완전한 dynamic
  rigid body 이므로 그리퍼가 밀면 움직입니다.
  (`3asset_v3.usd` 에 `physxRigidBody:sleepThreshold` 가 있어 정지 시 sleep,
  접촉 시 wake 하는 거동이 됩니다.)
- **검증**: 미실행

### 문제 4 — MEP collision 이 convexHull 로 근사될 가능성

- **현상**: (미검증) 그리퍼가 `gripper_fixture` 를 감싸지 못할 것으로 예상.
- **원인 (정적 분석)**: MEP 관련 USD 어디에도 mesh collision approximation 이
  author 되어 있지 않아 PhysX 기본값 convexHull 이 적용됨.
  spawn cfg 에도 `mesh_collision_props` 가 없음.
- **해결**: debris spawn 에
  `mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="convexDecomposition")`
  추가. Canadarm3 가 쓰는 것과 동일한 방식입니다.
- **검증**: 미실행
- **확인 방법**: Isaac Sim 에서 `Window > Simulation > Debug` →
  `Physics > Colliders > All` 을 켜고 `gripper_fixture` 의 collision wireframe 이
  손잡이 구멍을 따라 들어가는지 육안 확인.
  convexDecomposition 으로도 부족하면 `mesh_approximation="sdf"` 로 올릴 수 있으나,
  메시가 52개라 비용이 큽니다.

### 문제 5 — `mep_combined.usd` 가 저장소 밖 절대경로를 참조

- **현상**: (미검증) `/home/rokey/space_asset/mep.usd` 절대경로 참조.
- **원인**: USD 저장 시 절대경로로 기록됨. 저장소 내 `assets/space_asset/mep.usd`
  와 별개 경로.
- **해결**: 미적용 — Isaac Sim 머신에서 해당 경로 존재 여부 확인 후 판단.
- **검증**: 미실행

### 문제 6 — Play 를 누르면 로봇팔이 이리저리 움직이다 1초 안에 튕겨나감

- **현상**: `srb agent manual` 로 PAUSED 진입까지는 정상. 아무 키도 누르지 않고
  Play 를 누르면 팔+그리퍼가 잠깐 이리저리 떨리다가 1초도 안 되어 화면 밖으로
  날아감. 위성과 MEP 는 제자리에 그대로 있음.
- **기각된 가설**: "Canadarm3 베이스가 고정되어 있지 않다" 라고 추정해
  `fix_root_link = True` 를 넣어봤으나 변화 없었고, USD 를 직접 열어보니
  `canadarm3_large.usdz` 에는 이미
  `/canadarm3_large/canadarm3_large_0/root_joint [PhysicsFixedJoint] body0=[]`
  (= world 고정) 이 들어 있었습니다. **베이스는 원래부터 고정**이었고
  `fix_root_link` 는 이미 켜져 있는 joint 를 다시 켜는 no-op 였습니다.
  → 해당 설정은 되돌렸습니다.
- **결정적 단서 (터미널 로그)**:
  ```
  [Warning] [omni.physx.plugin] PhysicsUSD: CreateJoint - found a joint with
  disjointed body transforms, the simulation will most likely snap objects
  together: /World/envs/env_0/end_effector/base/AssemblerFixedJoint
  ```
  팔 flange 와 그리퍼를 잇는 fixed joint 의 양쪽 바디가 **서로 다른 위치**에
  있다는 뜻입니다. PhysX 는 Play 순간 이 위반을 해소하려고 두 바디를 강제로
  끌어당기고, 그 충격이 팔 전체를 튕겨냅니다.
- **원인**: `_update_assembly_fixed_joint_transforms` (`impl.py:482-540`) 는 바로
  이 스냅을 막으려고 reset 마다 그리퍼를 flange 위치로 옮깁니다. 그런데 이
  함수는 `base_asset.data.body_state_w` 로 flange 의 **현재 포즈**를 읽습니다.
  - reset 이벤트(`reset_scene`)는 관절 위치를 `write_joint_position_to_sim` →
    `root_physx_view.set_dof_positions()` 로 씁니다.
  - `set_dof_positions()` 는 DOF 값만 바꿀 뿐 **link transform 을 다시 계산하지
    않습니다.** 갱신은 `physics_sim_view.update_articulations_kinematic()` 이나
    다음 물리 스텝에서 일어납니다.
  - `DirectRLEnv.reset()` 은 `_reset_idx()` → `write_data_to_sim()` →
    `sim.forward()` 순서라, `sim.forward()` (= kinematic 갱신) 가
    `_update_assembly_fixed_joint_transforms` 보다 **나중**입니다.
  - 따라서 읽히는 flange 포즈는 관절이 모두 0 인 **spawn 시점의 값**입니다.
    Canadarm3 의 초기 자세는 50°/55°/75°/-30° 로 크고 팔 자체가 길어서,
    실제 flange 위치와 spawn 시점 위치의 차이가 수 m 단위입니다.
    그리퍼가 그만큼 엉뚱한 곳에 놓이고 → joint 위반 → Play 시 폭발.
  - 이 코드는 모든 매니퓰레이션 태스크가 공유하지만, 초기 관절 각도가 0 에
    가까운 다른 로봇들은 오차가 작아 조용히 넘어갔습니다. Canadarm3 만
    터진 이유입니다.
- **해결**: `impl.py` 의 `_reset_idx` 에서 `_update_assembly_fixed_joint_transforms`
  를 호출하기 **직전**에 kinematics 를 전파합니다.

  ```python
  if self.joint_assemblies:
      if self.sim.physics_sim_view is not None:
          self.sim.physics_sim_view.update_articulations_kinematic()
      self._update_assembly_fixed_joint_transforms(env_ids)
  ```

  이제 flange 포즈가 reset 된 관절 자세를 반영하므로 그리퍼가 정확한
  마운트 위치에 놓이고, fixed joint 위반이 0 이 되어 스냅이 사라집니다.
- **검증**: 미실행
- ⚠️ **이 수정만으로는 부족합니다.** 이후 USD 를 직접 열어 수치를 재어 본 결과
  (1) 이 코드는 reset 경로에서만 돌기 때문에 **첫 reset 이전**(= `sim.reset()`
  직후)에 이미 1.29 m 어긋난 joint 로 물리가 돌고, (2) 그것과 별개로 그리퍼
  게인이 실제 관성 대비 10⁶배 과하게 잡혀 있었습니다.
  → **문제 8** 에 정밀 측정 결과와 최종 수정을 정리했습니다.

### 문제 7 — 실행 시 PhysX 에러 / deprecation 경고 다수 출력

- **현상**:
  1. `PhysicsUSD: Parse collision - triangle mesh collision (approximation
     None/MeshSimplification) cannot be a part of a dynamic body, falling back to
     convexHull approximation: /World/envs/env_0/satellite/GOES_R/...` — 메시마다
     한 줄씩, 25줄 이상.
  2. `The <ImplicitActuatorCfg> object has a value for 'effort_limit' /
     'velocity_limit'. This parameter will be removed in the future...` — 4줄.
- **원인**:
  1. `satellite.usd` 의 GOES_R 메시들은 collision approximation 이 author 되어
     있지 않은데 `/GOES_R` 에 `PhysicsRigidBodyAPI` 가 있어 dynamic body 입니다.
     PhysX 는 dynamic body 의 triangle mesh 를 거부하고 convexHull 로 떨어뜨리며
     그때마다 에러를 찍습니다. task cfg 의 satellite spawn 에는
     `mesh_collision_props` 가 없었습니다 (debris 에만 있었음).
  2. Isaac Lab 이 `effort_limit` / `velocity_limit` 을 `*_sim` 으로 이관 중입니다.
     특히 `velocity_limit` 은 implicit actuator 에서 **원래 쓰이지 않습니다**.
- **해결**:
  1. satellite spawn 에 `mesh_collision_props=MeshCollisionPropertiesCfg(
     mesh_approximation="convexHull")` 추가. PhysX 가 어차피 쓰던 근사를 명시만
     한 것이라 **거동 변화 없이** 에러만 사라집니다. 배경 소품이므로
     convexDecomposition 까지 갈 필요는 없습니다.
  2. `effort_limit` → `effort_limit_sim` (implicit actuator 에서 동등),
     `velocity_limit` 은 제거 (미사용이므로 거동 변화 없음).
- **남은 경고 (무해)**:
  - `mep.usd` 의 `Cylinder_01.material:binding` 이 참조 범위 밖의
    `</Looks/OmniPBR_Disc_Cylinder>` 를 가리켜 무시됨 → 해당 실린더만 기본 머티리얼로
    보입니다. 물리와 무관.
  - `A prim already exists at prim path: ...` → Isaac Lab 이 기존 prim 을 재사용하며
    cfg 를 덮어씁니다. 정상 경로입니다.
- **검증**: 미실행

### 문제 8 — Play 폭발의 진짜 원인 (정밀 측정 결과)

문제 6 에서 넣은 `update_articulations_kinematic()` 수정은 **reset 경로만** 고친
것이었습니다. USD 에셋을 직접 열어 수치를 재어 본 결과, Play 시 폭발은 서로
독립적인 **두 개의 원인**이 겹친 것이었습니다. 둘 중 하나만 고치면 여전히
날아갑니다.

#### 8-A. Fixed joint 가 "이미 어긋난 채로" 생성된다 (기하 문제)

측정 방법: `assets/srb_assets/robot/manipulator/canadarm3_large.usdz` 의 joint
origin / axis 를 읽어 초기 관절각(50°, 0°, 55°, 75°, -30°, 0°, 0°)으로 FK 를
직접 계산했습니다.

| 시점 | flange (`canadarm3_large_7` + `(0,0,-0.44)`) 위치 | 그리퍼 root 위치 | 어긋남 |
|---|---|---|---|
| spawn 직후 (관절 전부 0, USD rest 자세) | `(1.045, -0.635, -0.029)` | `(0, 0, -0.44)` | **1.29 m** |
| reset 이후 (지정된 초기 자세) | `(0.567, 0.888, 4.850)` | `(0, 0, -0.44)` | **5.39 m** |

그리퍼 root 가 `(0, 0, -0.44)` 인 이유는 `env_cfg.py` 의 `_add_robot()` 이
end-effector 의 `init_state.pos` 를 **flange 기준 오프셋** 값(`frame_flange.offset`
∘ `frame_mount.offset` = `(0,0,-0.44)`)으로 그대로 써 버리기 때문입니다. 이 값은
env 원점 기준으로 해석되므로, 팔이 조금이라도 뻗어 있으면 그리퍼는 항상 엉뚱한
곳에 스폰됩니다. → PhysX 가 찍는
`found a joint with disjointed body transforms` 경고의 정체입니다.

`_update_assembly_fixed_joint_transforms()` 가 매 reset 마다 이걸 바로잡지만,

1. 그 함수는 **`_reset_idx()` 안에서만** 호출됩니다. 그런데 물리는 이미
   `DirectRLEnv.__init__` 안의 `sim.reset()` 시점부터 살아 있습니다. 즉
   *첫 reset 이전에* 1.29 m 어긋난 fixed joint 로 물리가 몇 스텝 돌아갑니다.
2. 옮기기만 하고 **속도를 지우지 않아서**, 그 사이에 스냅으로 얻은 속도를
   그대로 들고 제자리로 순간이동합니다. 그 운동량은 다시 fixed joint 로
   들어갑니다.

- **해결** (`impl.py`):
  - `_align_joint_assemblies()` 헬퍼로 분리 (kinematics 전파 → 배치 → **속도 0**).
  - `_reset_idx()` 뿐 아니라 **`__post_init__()` 에서도 1회 호출**. 이제 물리가
    처음 도는 순간부터 joint 위반이 0 입니다.
  - 초기화 시 `Assembly 'end_effector': attached body moved by up to X m onto its
    mount frame` 로그를 남겨, 실행 로그만 보고 배치가 실제로 일어났는지 확인할 수
    있게 했습니다. (예상 출력: `1.29 m`)
- **주의**: `disjointed body transforms` 경고 자체는 joint 가 *생성되는* 순간
  (`_setup_scene`) 에 찍히므로 **여전히 한 번 출력될 수 있습니다**. 중요한 것은
  경고의 유무가 아니라 Play 후에 스냅이 일어나지 않는 것입니다.

#### 8-B. 4.2배 스케일 그리퍼의 게인이 실제 관성보다 10⁶배 크다 (물리 문제)

`kinova300.usdz` 를 열어 실제로 author 된 값을 확인했습니다.

| 링크 | `physics:mass` | `physics:diagonalInertia` |
|---|---|---|
| `base` | **0.99 kg** | `(3.45e-4, 3.45e-4, 5.82e-4)` |
| `link_finger_[1-3]`, `link_finger_tip_[1-3]` (6개) | **0.01 kg** 씩 | `(7.9e-7, 7.9e-7, 8e-8)` |

핵심: **`mass_props=MassPropertiesCfg(density=1000.0)` 은 아무 일도 하지
않았습니다.** UsdPhysics 는 `physics:mass` 가 author 되어 있으면 density 를
무시합니다. `scale=(4.2,4.2,4.2)` 도 지오메트리만 키울 뿐 author 된 mass /
inertia 를 바꾸지 않습니다. 즉 **그리퍼는 74배 무거워진 적이 없습니다.**

그런데 게인은 "74배 무거워졌으니까" 라는 전제로 1000배가 되어 있었습니다.

```
stiffness = 1_200_000,  damping = 10_000,  effort_limit_sim = 2000
```

관성 `I = 7.9e-7 kg·m²` 인 손가락에 clamp 된 2000 N·m 이 한 번 걸리면, 물리 dt
(`env_rate = 1/150 s`) 한 스텝에

```
Δω = 2000 × (1/150) / 7.9e-7 ≈ 1.7 × 10⁷ rad/s
```

가 됩니다. 게다가 `solver_velocity_iteration_count = 0` 이라 이 속도를 보정할
기회조차 없고, 손가락의 반작용 토크는 assembler fixed joint 를 타고 팔 전체로
전달됩니다. **그리퍼를 전혀 조작하지 않아도** 8-A 의 스냅이 만든 아주 작은 관절
오차 하나면 폭발이 시작됩니다.

- **해결** (`kinova_gripper.py`, `Kinova300Large`):

  | 항목 | 이전 | 이후 | 근거 |
  |---|---|---|---|
  | `stiffness` | 1,200,000 | **5,000** | 실제 관성(1배 그리퍼와 동일)에 맞춤 |
  | `damping` | 10,000 | **100** | 위와 동일, 과감쇠 유지 |
  | `effort_limit_sim` | 2000 | **50** | 50 N·m ÷ (4.2 × 0.044 m) ≈ 270 N 파지력 |
  | `velocity_limit_sim` | (없음) | **5.0 rad/s** | PhysX 레벨 속도 상한 = 발산 불가능 |
  | `solver_velocity_iteration_count` | 0 | **1** | 속도 오차 보정 활성화 |
  | `mass_props` | `density=1000` | **제거** | no-op 인데 게인을 정당화하는 착시를 줌 |

  Canadarm3 에도 `velocity_limit_sim=5.0` 을 추가했습니다 (`effort_limit_sim` 만
  있고 속도 상한이 없었습니다).
- **파지력이 부족하면**: `effort_limit_sim` 을 먼저 올리고 (50 → 100 → 200),
  `stiffness`/`damping` 비(50:1)는 유지하세요. `velocity_limit_sim` 은 그대로
  두는 것이 안전합니다.

---

### 문제 9 — 위성(VenusExpress) 충돌체와 팔 붐(link 3)이 스친다

- **측정**: VenusExpress 메시 정점 271,478개를 `scale=3.4`, `yaw=90°`,
  `pos=(-1.65, 0, -1.05)` 로 변환한 뒤, 초기 자세의 각 링크 박스와 거리 계산.

  | 링크 | 위성까지 최소 거리 |
  |---|---|
  | `canadarm3_large_0` (베이스) | 0.268 m |
  | `canadarm3_large_1` | 0.386 m |
  | `canadarm3_large_2` | 0.473 m |
  | `canadarm3_large_3` (붐) | **0.000 m** (정점 6개가 최대 1.4 mm 침투) |
  | `canadarm3_large_4` 이상 | 2.3 m 이상 |

  즉 "**베이스가 위성 안에 파묻혀 있다**" 는 사실이 아닙니다 (27 cm 떠 있음).
  다만 **붐이 태양전지판을 스칩니다.**
- **왜 문제인가**: 위성은 `disable_rigid_body=True` 로 스폰되는 **정적 충돌체**
  입니다. 정적 충돌체가 dynamic 링크와 겹치면 PhysX 는 Play 즉시 밀어냅니다
  (`max_depenetration_velocity=5.0`). 깊이가 얕아 폭발의 주원인은 아니지만,
  수동 파지 테스트에서 팔이 알 수 없이 밀리는 원인이 됩니다.
- **해결** (`task.py`): 위성 scenery 의 충돌을 끕니다.

  ```python
  scenery.asset_cfg.spawn.collision_props = CollisionPropertiesCfg(
      collision_enabled=False
  )
  ```

  Canadarm3 베이스는 위성에 볼트로 붙은 것이 아니라 **자체 `root_joint` 로 월드에
  고정**되어 있으므로, 위성 충돌체는 이 태스크에서 아무 역할도 하지 않습니다.
  (충돌을 되살리려면 이 한 줄을 지우면 됩니다.)

---

### 문제 10 — 그리퍼가 플랜지 안으로 4.1 cm 들어가 있다 (시각 문제, 물리 무해)

- **측정**: `kinova300.usdz` 의 `base` 링크 지오메트리는 마운트 평면보다 **9.7 mm
  뒤쪽**까지 나와 있습니다. `scale=4.2` 를 곱하면 **41 mm**. `canadarm3_large_7`
  의 메시는 자기 프레임에서 `z ∈ [-0.441, 0]` 이고 마운트는 `z = -0.44` 이므로,
  그리퍼 뒷면이 플랜지 안으로 41 mm 들어갑니다.
- **물리적으로는 무해합니다**: assembler 가
  `mask_all_collisions=True` 로 `{ENV}/robot` ↔ `{ENV}/end_effector` **전체 서브트리**
  충돌을 필터링합니다 (`RobotAssembler.mask_collisions()` → `FilteredPairsAPI`).
  따라서 겹쳐도 척력이 발생하지 않고, "충돌 척력 vs fixed joint 상충" 은
  일어나지 않습니다. 실제 로그에도 접촉 관련 에러는 없었습니다.
- **시각적으로 거슬린다면**: `canadarm3.py` 의
  `frame_flange.offset.pos` 를 `(0, 0, -0.44)` → `(0, 0, -0.48)` 로 바꾸면 딱
  붙습니다. 단 TCP 도 4 cm 같이 밀리므로, 이미 맞춰 둔 파지 위치가 있으면
  그대로 두는 편이 낫습니다. (스케일을 키운 것은 그리퍼 지오메트리뿐이고 마운트
  오프셋 0.44 는 1배 기준 값이라 생긴 차이입니다.)
- **함께 수정**: `frame_tool_centre_point` 가 `0.64` (= 0.16 × **4**) 로 되어 있어
  실제 스케일 4.2 와 어긋났습니다 → `0.672` (= 0.16 × 4.2) 로 수정. TCP 가 3.2 cm
  짧게 잡혀 IK 목표와 `G` 진단의 TCP↔MEP 거리가 그만큼 틀어져 있었습니다.

---

### 가설 검증 요약 — 제기된 6가지 원인 진단

정밀 분석 요청으로 받은 6개 가설을 USD / IsaacLab 소스 기준으로 하나씩
검증했습니다. **3개는 사실이 아니어서 수정하지 않았습니다.**

| # | 가설 | 판정 | 근거 |
|---|---|---|---|
| 1 | 로봇 베이스 미고정 (`fix_root_link` 누락) | ❌ **사실 아님** | `canadarm3_large.usdz` 에 `canadarm3_large_0/root_joint` (`PhysicsFixedJoint`, `body0=[]` = world) 가 이미 있음. 이전에 직접 시도해 보고 "변화 없음"으로 되돌린 기록도 있음 (문제 6). 넣어도 이미 켜진 joint 를 다시 켜는 no-op |
| 2 | 베이스가 위성 충돌체에 파묻혀 스폰 | ⚠️ **부분적으로 사실** | 베이스는 27 cm 떠 있음. 대신 **붐(link 3)** 이 태양전지판을 1.4 mm 스침 → 위성 충돌 비활성화로 조치 (문제 9) |
| 3 | 그리퍼-플랜지 침투와 fixed joint 의 상충 | ❌ **사실 아님** (침투는 사실) | 41 mm 침투는 맞지만 `mask_all_collisions=True` 가 서브트리 전체 충돌을 필터링하므로 척력 자체가 없음 (문제 10) |
| 4 | `write_root_pose_to_sim` 후 `write_data_to_sim` 누락 | ❌ **사실 아님** | IsaacLab 의 `write_root_pose_to_sim()` 은 버퍼가 아니라 `root_physx_view.set_root_transforms()` 로 **즉시** PhysX 에 씀. `write_data_to_sim()` 은 actuator/force 버퍼용이고, `DirectRLEnv.reset()` 이 `_reset_idx()` 직후에 이미 호출함. 실제 어긋남도 15 m 가 아니라 **5.39 m** (문제 8-A) |
| 5 | 4.2배 그리퍼의 초고강성 + `solver_velocity_iteration_count=0` | ✅ **사실 (핵심 원인)** | 다만 이유가 다름: 질량이 74배가 된 적이 없음 (author 된 `physics:mass` 가 density 를 이김). 그래서 게인은 74배가 아니라 **10⁶배** 과한 상태였음 (문제 8-B) |
| 6 | 플랜지-마운트 쿼터니언 뒤틀림 | ❌ **사실 아님 (의도된 값)** | `Ry(180) @ Rx(180) = Rz(180)` → **Z 축은 그대로**. 그리퍼 지오메트리는 `base` 의 -Z 로 자라고 플랜지 면도 link7 의 -Z 이므로 이 합성이 있어야 그리퍼가 팔 바깥을 향함. 남는 180° 는 3개 손가락 중 어느 것이 어디로 갈지만 정함 (대칭이라 무의미). "정렬" 하면 오히려 방향이 틀어짐 |

---

## 8. 수동 조작 방법

### 8-1. 실행 직후 예상 상태

| 대상 | 상태 |
|---|---|
| Simulation | **PAUSED** (툴바 Play 를 누를 때까지 물리 진행 없음) |
| Robot Arm | `Canadarm3.asset_cfg.init_state.joint_pos` 자세 그대로 정지 (50°, 0°, 55°, 75°, -30°, 0°, 0°) |
| Joint velocity | 0 |
| Gripper | OPEN (모든 finger / finger_tip = `0.2 rad`) |
| MEP (debris) | `pos=(-0.105728, 13.90236, 11.82852)`, `rot=(0.095277, -0.700659, 0.095277, -0.700659)` 에서 정지 |
| MEP linear / angular velocity | 0 / 0 |
| 자동 trajectory / grasp / motion | 없음 (`env.step()` 미호출) |

Play 를 눌러도 아무것도 움직이지 않아야 합니다. 팔·그리퍼의 joint position
target 이 초기 자세와 동일하게 설정되어 있어 제어 오차가 0 이기 때문입니다.

### 8-2. 키 매핑

터미널에도 실행 시 동일한 표가 출력되고, `H` 로 다시 볼 수 있습니다.

| 키 | 동작 |
|---|---|
| `1` ~ `7` (상단 숫자 또는 NUMPAD) | 조작할 관절 선택 |
| `Z` / `X` | 이전 / 다음 관절 선택 |
| `↑` / `↓` | 선택된 관절 `+step` / `-step` (기본 1°) |
| `←` / `→` | step 크기 1/2배 / 2배 (0.05° ~ 15°) |
| `O` | 그리퍼 OPEN (0% closed) |
| `C` | 그리퍼 CLOSE (100% closed) |
| `K` / `M` | 그리퍼 10% 열기 / 10% 닫기 (0/25/50/75/100% 단계 제어 가능) |
| `G` | 파지 진단 출력 (접촉 body 수, 접촉력, MEP 속도, TCP↔MEP 거리) |
| `N` | 현재 측정 자세를 target 으로 고정 (움직임 정지) |
| `B` | 지정된 초기 관절 자세로 복귀 |
| `L` | 씬 전체 reset (MEP 도 초기 위치로) |
| `H` | 도움말 다시 출력 |

관절 각도는 `soft_joint_pos_limits` 로 클램프되므로 리밋을 넘지 않습니다.

그리퍼 open/close 값은 하드코딩이 아니라 `Kinova300Large` 의
`BinaryJointPositionActionCfg.open_command_expr` / `close_command_expr`
(`0.2` / `1.2 rad`) 에서 읽어옵니다. 따라서 로봇 설정을 바꾸면 자동으로 따라갑니다.

### 8-3. 파지 성공 여부 판정

`G` 키를 누르면 다음이 출력됩니다.

```
[manual] --- grasp state ---
  gripper joints: kinova300_joint_finger_1=..deg, ...
  gripper<->debris contact: N bodies, total |F| = ... N
    -> ...
  debris |v| = ... m/s, |w| = ... rad/s
  TCP <-> debris centre distance = ... m
```

접촉 판정은 `contacts_end_effector` ContactSensor 의 `force_matrix_w` 를 씁니다.
이 센서는 `TaskCfg.__post_init__` 에서 `filter_prim_paths_expr = [debris.prim_path]`
로 설정되어 있어 **그리퍼 ↔ MEP 사이의 접촉력만** 분리해서 봅니다.
즉 시각적 겹침이 아니라 실제 물리 접촉만 집계됩니다.

| 판정 | `G` 출력 | 의미 |
|---|---|---|
| **Case A — 단순 겹침** | `0 bodies` / `NO physical contact` | visual mesh 만 겹침. collision 미발생 → FAIL |
| **Case B — 충돌하지만 미파지** | `1 body`, MEP `\|v\|` 가 증가하며 멀어짐 | 정상 collision 이지만 밀어냄 → grasp 실패 |
| **Case C — 실제 파지** | `2 bodies 이상`, 팔을 움직이면 MEP 가 따라옴 | 파지 성공 |

Case C 최종 확인 절차:
1. `C` 로 그리퍼를 닫고 `G` 로 접촉 body 수 ≥ 2 확인
2. 관절 하나를 `↑`/`↓` 로 천천히 이동
3. MEP 가 그리퍼와 **함께** 이동하는지 확인 (`G` 의 TCP↔MEP 거리가 유지되면 파지)
4. `O` 로 열었을 때 MEP 가 release 되는지 확인

---

## 9. 테스트 결과

*(모든 항목 미실행 — Isaac Sim 머신에서 아래 명령으로 실행 후 결과를 채워 주세요)*

```bash
srb agent manual --env debris_capture_visual \
  --kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" \
  env.robot=canadarm3+kinova300_large
```

### Test 0 — Play 폭발 재발 여부 (문제 8/9 수정 후 가장 먼저 확인)

실행 로그에서 먼저 확인할 것:

```
[INFO] Assembly 'end_effector': attached body moved by up to 1.29 m onto its mount frame
```

- 이 줄이 보이지 않으면 배치가 아예 안 된 것입니다 (`joint_assemblies` 비었거나
  `physics_sim_view` 가 None). 그대로 보고해 주세요.
- 값이 `1.29 m` 근처면 계산과 일치합니다.
- `found a joint with disjointed body transforms` 경고는 joint 가 **생성되는**
  순간에 찍히므로 한 번 더 나올 수 있습니다. 판단 기준은 경고가 아니라 아래
  거동입니다.

| 확인 항목 | 기대 결과 | 결과 |
|---|---|---|
| Play 직후 5초 대기 (키 입력 없음) | 팔·그리퍼가 **전혀 움직이지 않음** | 미실행 |
| 그리퍼 손가락 | 자동으로 열리거나 닫히지 않음 | 미실행 |
| 팔이 위성에 밀리는지 | 밀림 없음 | 미실행 |
| `G` 진단의 debris `\|v\|` | 0.000 유지 | 미실행 |

### Test 1 — Simulation 초기 상태
- 자동 Play 안 함: 미실행
- Robot 초기 정지 (Play 전): 미실행
- Robot 초기 정지 (Play 후에도 유지): 미실행
- Gripper OPEN 상태: 미실행
- MEP 초기 정지 (Play 전): 미실행
- MEP 초기 정지 (Play 후에도 유지): 미실행

### Test 2 — Robot Joint Control
- `1`~`7` 관절 선택: 미실행
- `↑`/`↓` 관절 개별 이동: 미실행
- `←`/`→` step 크기 변경: 미실행

### Test 3 — Gripper
- `O` Open: 미실행
- `C` Close: 미실행
- `K`/`M` 단계 제어: 미실행

### Test 4 — Collision
- `gripper_fixture` collision wireframe 확인: 미실행
- `G` 출력에서 접촉 body 수 > 0: 미실행

### Test 5 — Manual Grasp
- Fixture 파지 (접촉 body ≥ 2): 미실행
- MEP 동반 이동: 미실행
- Release: 미실행

---

## 10. 다음 확인이 필요한 항목

실행 시 아래를 확인하고 이 문서에 기록합니다.

1. **`mep_combined.usd` 의 절대경로 참조**
   ```bash
   ls -l /home/rokey/space_asset/mep.usd
   ```
   존재하지 않으면 MEP 지오메트리가 조용히 비어 있게 됩니다.
   그 경우 `mep_combined.usd` 의 reference 를 저장소 내 상대경로
   (`../../assets/space_asset/mep.usd` 기준)로 다시 저장해야 합니다.
2. **`Failed to apply action:` 로그 원문** — 있으면 전문을 7절 문제 2 에 추가.
3. **MEP 가 팔의 도달 범위 밖에 있음 (실측)** — 폭발이 잡히고 나면 바로 마주칠
   문제입니다.

   | 항목 | 값 |
   |---|---|
   | Canadarm3 최대 도달 거리 (베이스 기준, USD joint origin 합산) | 약 **8.5 m** |
   | 초기 자세의 flange 위치 | `(0.567, 0.888, 4.850)` |
   | MEP `init_state.pos` | `(-0.106, 13.902, 11.829)` |
   | 베이스 ↔ MEP 거리 | **18.2 m** |
   | flange ↔ MEP 거리 | **14.8 m** |

   즉 현재 배치로는 관절을 어떻게 움직여도 MEP 에 닿을 수 없습니다.
   (`task.py` 의 `termination_debris_too_far` 임계값도 10 m 입니다.)
   `self.scene.debris` 의 `init_state.pos` 를 flange 근처 — 예: `y` 를 13.9 →
   4~5 m 대로 — 옮겨야 Step 8 수동 파지 테스트가 가능합니다. 원래 배치를
   유지하라는 요구가 있어 **값은 바꾸지 않았습니다.**
4. **`gripper_fixture` 실제 치수 대비 그리퍼 개폐 폭** —
   `G` 의 TCP↔MEP 거리와 collision 표시로 판단. 손가락이 손잡이를 감싸기에
   너무 크거나 작으면 `Kinova300Large.asset_cfg.spawn.scale` 또는
   debris 의 `scale` 조정이 필요하지만, 기존 배치를 유지하라는 요구에 따라
   **먼저 실측한 뒤에** 판단합니다.
5. **convexDecomposition 으로 충분한지** — 부족하면 `sdf` 로 상향.
