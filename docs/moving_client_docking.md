# 이동 Client 도킹 시나리오 (rendezvous → moving docking → stop → release → departure)

> 상태: **구현 완료 / offline 테스트 43개 통과 / Isaac Sim은 이송 단계까지만 확인**
> 이동 중 도킹(Test 3)을 막던 depth 보정 문제는 원인을 찾아 수정했지만, 전체 도킹 실행 검증은 아직입니다. Test 3~6은 **미검증**입니다.

관련 문서: [`mrv_probe_docking.md`](mrv_probe_docking.md), [`mrv_dynamic_approach_progress.md`](mrv_dynamic_approach_progress.md)

---

## 1. 분석 결과 (구현 전, 코드·USD 기준)

### Client가 움직이지 않던 이유

**고정된 것이 아닙니다.** 속도가 0으로 설정돼 있었을 뿐입니다.

| 확인 항목 | 결과 | 근거 |
|---|---|---|
| Client prim | `{ENV}/satellite/GOES_R` | `satellite_v3.usd` PXR traversal |
| RigidBodyAPI | 적용됨, `rigidBodyEnabled = True` | 동일 |
| Kinematic | `kinematicEnabled = False` (동적) | 동일 |
| Client에 걸린 joint | **0개** (world 고정 joint 없음, 내부 joint 없음) | 런타임 stage 전체 traversal (`client_constraints_report`) |
| 중력 | scene gravity `(0, 0, 0)` (`Domain.ORBIT`) | 런타임 `[MOVE0]` 체크 |
| 질량 | density 1000 → **1,792,594 kg** (convex hull 부피 기준) | `root_physx_view.get_masses()` |
| 초기 속도 | `docking.satellite_velocity_mps: 0.0` → `init_state.lin_vel = 0` | `vision_task.py` |

즉 요청서에 있던 후보 원인(FixedJoint, kinematic, rigidBodyEnabled=false, transform 고정, velocity overwrite, constraint)은 **모두 해당 없음**입니다. 제거한 constraint도 없습니다. 방어 코드로, 한쪽이 world인 joint가 발견되면 비활성화하고 Client 내부 joint는 유지하도록 만들어 두었습니다. 현재 에셋에서는 해당 joint가 없어 보고만 합니다.

### MRV / MEP / 결합 구조

| 대상 | 구조 |
|---|---|
| MRV | Canadarm3 articulation. `canadarm3_large_0/root_joint`는 body0가 비어 있는 world 고정 FixedJoint. 선체 `{ENV}/scenery`는 충돌이 꺼진 정적 XForm |
| MRV 이동 방식 | 기존 `MrvTransit`: 매 step `write_root_pose_to_sim` + 선체 `set_world_poses` (kinematic 평행이동) |
| MEP | `{ENV}/debris` 단일 강체, 3000 kg, 감쇠 0 |
| **Robot ↔ MEP** | `{ENV}/capture_joint`: `CaptureManager`가 만드는 FixedJoint (flange link ↔ MEP) |
| **MEP ↔ Client** | `{ENV}/docking_joint`: `DockingManager`가 만드는 FixedJoint (MEP ↔ GOES_R, probe tip 기준) |
| 도킹 완료 판정 | `docking.is_docked` = `docking_joint` prim 존재 여부 |
| 도킹 축 | world +X (`DockingPlacementCfg.approach_dir`) |

두 joint는 **서로 다른 prim**이라 따로 해제할 수 있습니다.

---

## 2. 변경된 파일

| 파일 | 변경 이유 |
|---|---|
| `project/srb/tasks/manipulation/debris_capture/moving_dock.py` | **신규.** 순수 numpy. config 4종, 상대 운동 계산, MRV 속도 제어기, 속도 추정기(LSQ), 안정 타이머, 정지 법칙, 이탈 방향/거리, CSV 컬럼 정의 |
| `.../vision_capture_demo.py` | 새 상태 10개 + 실패 상태 8개, 각 상태 핸들러, MRV 이동·추력·텔레메트리·debug draw, 결과 체크 `[MOVE0]~[MOVE6]`. 기존 코드는 아래 5곳만 수정 |
| `.../vision.py` | `client` / `rendezvous` / `post_docking` / `separation` config 섹션 등록 + 검증 |
| `project/config/vision_capture.yaml` | 위 4개 섹션 추가 (`client.release_enabled: false`가 기본값이라 기존 실행은 영향 없음) |
| `project/scripts/vision_capture.py` | `--moving_dock` 플래그 (docking + client release를 켬) |
| `project/tests/test_moving_dock.py` | **신규.** offline 테스트 14개 |

### 기존 코드에 대한 수정 (전부 moving 모드일 때만 동작이 달라짐)

1. `set_velocity_feedforward`: joint 속도 feed-forward에서 **MRV base 속도를 뺌**. MRV가 안 움직이면 0을 빼므로 기존과 동일합니다.
2. 도킹의 "tip 정지" 판정(`tip_settled`): moving 모드에서는 tip 위치를 **Client 좌표계**로 기록합니다.
3. `docking_tracking_valid`: "한 step에 target이 튐" 검사를 `prev + v_sat·Δt` 기준으로 변경. 정지 위성이면 기존과 동일합니다. **런타임 버그 수정**(아래 5절).
4. `step_dock_ready`: moving 모드에서 상대속도 게이트 추가(기존 조건은 그대로 유지).
5. `step_docked` / `after_capture_state` / skip_capture 경로: moving 모드일 때 다음 상태만 바뀜.

**건드리지 않은 것**: AprilTag, pose estimation, prediction, capture(`capture.py`), docking 기하·조인트(`docking.py`), `probe_dock.py`, MRV 접근 단계, ROS 인터페이스.

---

## 3. 상태 머신

도킹 전 대기 단계는 없습니다. 도킹 이송이 시작과 동시에 출발하고, Client 방출과 MRV 위치 유지는 이송과 **동시에** 진행됩니다.

```
(capture HOLDING 또는 --dock_only)
→ DOCK_TARGET_ACQUIRE → PRE_DOCK_APPROACH … ALIGNMENT_CHECK      (기존 도킹 상태)
      ├ 동시 진행: Client 방출 추력 (F = m·a) → 방출 완료 후 추력 없이 유영
      └ 동시 진행: MRV 위치 유지 v_cmd = v_client + Kp·e
→ [삽입 직전 판정: 방출 완료 + 상대속도 + MRV 위치 오차]
→ Z_APPROACH → FINAL_INSERTION → DOCK_READY → DOCKED           (기존 도킹 상태)
→ ROBOT_RELEASE (즉시 흡착 해제, 0.5 s 확인)
→ MRV_SEPARATION: MRV가 +X에서 감속·반전해 −X로 이탈
      └ 동시 진행: MEP + 위성(도킹 상태)은 추력으로 감속해 정지
→ SUCCESS (Mission complete: MRV 정지 + 스택 정지 + 거리 증가 + 도킹 유지)
```

`post_docking.immediate_release: false`이면 이전 흐름(STABILIZING → STOPPING → ROBOT_RELEASE → ARM_RETREAT → MRV_SEPARATION)을 사용합니다.

| 상태 / 단계 | 진입 조건 | 동작 | 종료 조건 | 타임아웃 → 실패 상태 |
|---|---|---|---|---|
| Client 방출 (도킹과 동시) | 시나리오 시작 | Client에 추력 F = m·a (a ≤ 0.004 m/s²) | \|v − v_cmd\| ≤ 0.5 mm/s → 추력 해제 | 30 s → CLIENT_RELEASE_FAILED |
| MRV 위치 유지 (도킹과 동시) | 시나리오 시작 | v_cmd = v_client + Kp·e, 가속 제한 | 정지 단계까지 계속 | - |
| 기존 도킹 상태 | 시작 즉시 | 기존 로직 (위성 속도 feed-forward) | DOCK_READY 조건 + 상대속도 게이트 | 기존 타임아웃 → DOCK_FAILED |
| 삽입 직전 판정 (ALIGNMENT_CHECK 안) | 정렬 + 정지 + depth 보정 완료 | 판정만 함 | 방출 완료, 상대속도 MRV·MEP ≤ 5 mm/s, MRV 위치 오차 ≤ 50 mm | 도킹 단계 타임아웃 → DOCK_FAILED |
| STABILIZING | docking_joint 생성 | 자세 유지, MEP–Client drift 감시 | 2 s 동안 drift < 5 mm / 0.5° | → DOCK_FAILED |
| STOPPING | 안정화 완료 | Client·MEP에 **같은 감속** F = m·a, MRV는 위치 유지 제어로 따라감 | 세 속도 ≤ 1 mm/s, Client 각속도 ≤ 0.0005 rad/s가 1 s 유지 | 90 s → STOP_FAILED |
| ROBOT_RELEASE | 전체 정지 | `capture.release(0)` → **capture_joint만 삭제** | 0.5 s 후 capture 해제됨 AND docking 유지 | 5 s → ROBOT_RELEASE_FAILED |
| ARM_RETREAT | 해제 확인 | EE를 MEP 면 법선 방향으로 0.6 m 후퇴 | 목표 도달 (< 10 mm) | 90 s → ARM_RETREAT_FAILED |
| MRV_SEPARATION | 후퇴 완료 | MRV 사다리꼴 이동 (가속, 5 s 순항, 감속) | 거리 +0.1 m 이상, 되돌아옴 ≤ 5 mm, 도킹 유지 | 90 s → SEPARATION_FAILED |

추가 실패 처리:
- ARM_RETREAT·MRV_SEPARATION 중 팔 접촉력이 50 N을 넘으면 → SEPARATION_COLLISION
- NaN, Client 속도 폭주, docking_joint 소실 → PHYSICS_ERROR (= PHYSICS_UNSTABLE)
- 실패 시에는 모든 추력을 해제합니다. MRV는 MEP를 잡고 있으면 Client 기준 위치를 유지하고, 잡고 있지 않으면 가속 제한을 지키며 정지합니다.

---

## 4. 구현 요점

### Client Release

- 제거하거나 비활성화한 constraint는 **없습니다**. 원래 고정이 아니었기 때문입니다.
- 속도는 한 번에 쓰지 않고 **추력 램프**로 올립니다: `set_external_force_and_torque(F = m·a, τ = I·α, is_global=True)`. Client 질량이 1.79×10⁶ kg이라 최대 힘은 약 7.2 kN입니다.
- 방향은 **+X**를 골랐습니다. 도킹 삽입 축이자 팔의 접근 방향이라, MRV가 자기 접근 축을 따라 추격하고 반대(−X)로 이탈하면 EE가 MEP 면 법선 방향으로 떨어집니다.

### 속도 제어

| 대상 | 방식 | 이유 |
|---|---|---|
| Client | 추력(외력) | 자유 강체라 힘으로 가속하는 것이 물리적으로 자연스러움 |
| MRV | 가속 제한 속도 명령 → 적분 → kinematic root pose 쓰기 | base가 world 고정 articulation이라 힘·속도 API가 없음. `write_root_pose_to_sim`이 유일한 방법(기존 `MrvTransit`와 동일). 매 step 변위는 v·dt ≈ 0.3 mm |
| MEP | **독립 제어 안 함** | 팔에 FixedJoint로 붙어 있어 MRV + 팔 움직임을 따라감. 도킹 중에는 기존 probe 추적 제어가 상대 운동을 담당 |
| 정지 | Client와 MEP에 **같은 가속도** a를 F = m·a로 적용 | joint에 추가 하중이 걸리지 않음. MRV는 위치 유지 제어로 따라옴 |

MRV 제어 법칙:

```text
e      = (p_client − p_mrv) − d_ref            # d_ref = 검증된 도킹 구성의 Client→MRV 오프셋
v_des  = v_client + clip(Kp·e, 0.02 m/s)       # Kp = 0.15 1/s
v_cmd  = v_prev + clip(v_des − v_prev, a_max·dt)   # a_max = 0.004 m/s²
```

`d_ref`를 유지하면 도킹 로직은 정지 위성 때와 같은 상대 기하를 봅니다.

### 상대 속도 계산

```text
v_rel_mrv = v_mrv − v_client
v_rel_mep = v_mep − v_client
```

- v_client, v_mep: 위치 이력 0.5 s 구간의 **최소자승 기울기**
- v_mrv: 명령 속도 (kinematic이라 명령 = 실제)
- PhysX 자체 속도도 CSV에 같이 기록합니다(`*_physx_velocity_*`). kinematic으로 끌려가는 MEP는 PhysX 속도가 부정확할 수 있어 게이트에는 쓰지 않습니다.
- 모든 rendezvous·도킹 게이트는 **상대값만** 봅니다. world 속도는 조건이 아닙니다.

### Docking 통합

RENDEZVOUS 완료 시 기존 `begin_docking()`을 호출하고 기존 도킹 상태를 그대로 탑니다. 기존 `track_probe`는 이미 위성 속도를 feed-forward로 쓰고 있었습니다. 추가한 것은 2절의 수정 1~4입니다.

### Robot Release

- 제거: `{ENV}/capture_joint` (Robot EE ↔ MEP)
- 유지: `{ENV}/docking_joint` (MEP ↔ Client)
- 해제 후 `release_wait_sec` 동안 기다린 뒤 두 prim의 존재 여부를 다시 확인합니다.

### MRV Separation

- 방향: `−normalize(client.linear_velocity_mps)` = **−X**. `separation.direction`으로 직접 지정할 수도 있습니다.
- 속도 0.02 m/s, 가속 0.004 m/s², 순항 5 s → 이동 거리 0.2 m
- 이탈 전에 ARM_RETREAT로 EE를 0.6 m 먼저 뺍니다.
- Thruster VFX(기존)는 MRV가 가속하는 동안에만 켜집니다.

### Telemetry / Debug draw

- `<tag>_moving.csv`: 요청된 컬럼 전부 + PhysX 속도, MRV 위치 오차, 적용된 힘, 팔 접촉력
- Debug draw: Client·MEP·MRV 속도 벡터(v × 100 s), MEP→Client 도킹점 벡터, 이탈 방향

---

### 강제 삽입 (forced insertion, `--moving_dock`에서만, 기본 켜짐)

pre-dock 지점에 도착하면(50 mm 이내) 흔들림이 가라앉기를 기다리지 않고, **MEP가 도킹축 방향으로 Client보다 항상 빠르게**(상대속도 ≥ 5 mm/s) 다가갑니다.

| 항목 | 기존 (검증된 삽입) | 강제 삽입 |
|---|---|---|
| pre-dock 흔들림 대기 | 3 s 동안 50 mm 이내 | 없음 |
| 삽입 시작 조건 | 횡오차 50 mm, 축 3°, roll 5°, depth 보정 완료 | 횡오차 ≤ `insertion_max_lateral_m`(0.20 m), 축 ≤ `insertion_max_axis_deg`(12°), 노즐 벽 여유 ≥ 20 mm |
| 삽입 중 재정렬 기준 | `approach_abort_*` | 위와 같은 넓은 기준 + 노즐 벽 여유 |
| depth 불일치 | 전진 보류 | 로그만 남기고 전진 (축 위에 오면 보정은 계속 시도) |
| 최종 FixedJoint 조건 | `dock_*` (40 mm, 2°) | **동일, 완화 안 함** |

끄려면 `--set rendezvous.forced_insertion=false`.

### Docking monitor 창 (GUI)

정지 위성 도킹과 이동 도킹 모두에서 열립니다(`logging.motion_hud`). 표시 항목: 상태, 속도, 상대속도, 프로브 축–노즐 축 각도와 기준값, 횡오차, roll, 프로브 끝–도킹점 거리, 결합 상태.

---

## 5. 테스트 결과

### Offline (Isaac Sim 없이)

```bash
cd project && ~/isaac-sim/python.sh -m pytest tests/test_moving_dock.py -q   # 14 passed
~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py tests/test_vision_math.py tests/test_six_dof_capture.py -q   # 59 passed (변경 전 기준선)
```

### Isaac Sim headless 실행

| 실행 | 명령 | 결과 |
|---|---|---|
| 1 `moving_smoke` | `--dock_only --moving_dock` | 18/25 PASS, **DOCK_FAILED** (sim 102 s): "target jumped 1662 mm". 원인은 아래 버그 1 |
| 2 `moving_smoke2` | 동일 (버그 1 수정 + rendezvous 게이트 0.05 m) | Z_APPROACH에서 sim 153~225 s 동안 전진 멈춤 → **사용자 요청으로 중단**. 결과 JSON 없음 |
| 3 `static_regress` / `moving_nodepth` | 정지 위성 회귀 테스트 / depth 게이트 끔 | 사용자 요청으로 **중단, 결과 없음** |

공통 실행 명령:

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --headless --tag <tag> --dock_only --moving_dock
```

| 4 `trail_smoke` / `concurrent_smoke` | `--dock_only --moving_dock` (약 5분 후 자동 종료) | 궤적 그리기 오류 없음. 동시 진행 모드에서 **sim 0 s에 이송 시작**, 이송 중 5.2 s에 Client 방출 완료, MRV 위치 오차 ≤ 4.5 mm → 0.0 mm |

### 요청서 30번 항목 기준 결과

실행 1·2는 이전의 순차 진행 설정(순항 5 s, MRV 0.015 m/s 추격)으로 측정한 값입니다.

| 테스트 | 결과 | 근거 (실행 1·2 동일 값) |
|---|---|---|
| Client Free Flight | **PASS** | 5.0 s 유영, 최대 속도 변화 0.000 mm/s, Z 속도 변화 0.000 mm/s, 변위 0.0971 m (예상 0.0974 m) |
| Velocity Matching | **PASS** | 시작: Client 19.5 / MRV 15.0 / MEP 14.5 mm/s → 1.2 s 만에 \|v_mrv−v_c\| 0.37, \|v_mep−v_c\| 2.08 mm/s. world 속도는 계속 약 19.5 mm/s |
| Moving Docking | **FAIL (미도달)** | 이송·정렬은 통과. Z_APPROACH에서 전진 멈춤 (아래 문제 2) |
| Post Docking Stop | **미검증** | 도달 못함 |
| Robot Release | **미검증** | 도달 못함 |
| MRV Separation | **미검증** | 도달 못함 |

Velocity Matching 참고: MEP는 팔에 붙어 있어 MRV와 거의 같은 속도로 움직입니다. 요청서의 "MEP 0.018 m/s로 따로 시작" 조건은 이 구조에서는 독립적으로 줄 수 없습니다.

### 런타임에서 찾은 문제

**버그 1 (수정함)**: `docking_tracking_valid`가 수십 초 전 값과 현재 위치를 비교해, Client가 85 s 동안 1.66 m 이동한 것을 "순간 점프"로 오판했습니다. 이동 예상치 `prev + v·Δt` 기준으로 바꿨습니다. 정지 위성이면 동작은 기존과 동일합니다.

**문제 2 (원인 확인·수정, 실행 검증 전)**: 실행 2의 Z_APPROACH에서 전진이 멈췄습니다.

- 관측 1: depth 1.83 m vs 기하 거리 1.51 m, **320 mm 불일치** → 기존 fail-safe가 전진을 보류합니다.
- 관측 2: 그 사이 팔+3 t payload 흔들림으로 횡오차가 약 76~80 mm까지 커져 재정렬이 2번 일어났습니다. 남은 거리는 1.494 m로 고정됐습니다.
- 관측 3: MRV 위치 오차는 0.0 mm, MRV 상대속도는 0으로, **MRV 추종은 정상**입니다. MEP 상대속도는 5~27 mm/s로 흔들립니다.
- 관측 4: depth 캘리브레이션 오프셋은 1.207 m로, 실행 1·2가 같은 값입니다(결정적).
- 비교: 예전 정지 위성 로그(`dock_docking.csv`)도 횡오차 최대 80 mm, Z_APPROACH에서 끝났습니다. 이동 런은 평균 45 mm vs 35 mm, 상대속도 22 vs 16 mm/s로 조금 더 나쁩니다.
- **원인 (로그로 확인)**: depth 보정이 **축에서 41 mm 벗어난 위치**에서 잡혔습니다(정렬 게이트 50 mm라 통과됨). 그 결과 축에 정렬되면(횡오차 0~40 mm) 항상 320 mm가 어긋나 전진이 막혔고, 축에서 벗어나 있을 때만(40~90 mm) depth가 맞았습니다. 정지 위성 로그에도 똑같은 패턴이 있어, 이동 때문이 아닌 기존 문제입니다.
- **수정**: 보정을 축 위(횡오차 ≤ 15 mm, 축 기울기 ≤ 1°)에서만 10개 샘플의 중앙값으로 하고, 보정 전에는 삽입을 시작하지 않게 했습니다. 안전 게이트는 그대로입니다. 전체 도킹으로는 아직 실행 검증하지 않았습니다.

---

## 6. 남은 작업 (우선순위 순)

1. **정지 위성 회귀 런** (`--dock_only`): depth 보정 수정 후에도 기존 도킹이 성공하는지 확인 (약 15분).
2. **이동 도킹 전체 런** (`--dock_only --moving_dock`): 도킹 성공과 이후 단계(Test 3~6) 확인.
3. ROS 모드(`--ros`)와 moving_dock 조합 실행 (미실행).
4. 포획부터 시작하는 전체 파이프라인(`--moving_dock`, `--dock_only` 없이) 실행 (미실행).
5. GUI에서 궤적·속도 창 확인.

---

## 7. 설정 요약 (`project/config/vision_capture.yaml`)

| 섹션 | 주요 키 (기본값) |
|---|---|
| `client` | `release_enabled` (false, `--moving_dock`으로 켬), `linear_velocity_mps` [0.02, 0, 0], `angular_velocity_rad_s` [0, 0, 0], `release_accel_mps2` 0.004 |
| `rendezvous` | `concurrent_docking` true, `mrv_max_accel_mps2` 0.004, `position_gain_hz` 0.25, `max_*_relative_velocity_mps` 0.005, `max_relative_position_m` 0.05, `dock_max_*` |
| `post_docking` | `stabilization_sec` 2.0, `stop_accel_mps2` 0.002, `stop_velocity_tolerance_mps` 0.001, `release_wait_sec` 0.5 |
| `separation` | `arm_retreat_distance_m` 0.6, `direction` [] (자동: −v_client), `velocity_mps` 0.02, `duration_sec` 5.0, `max_contact_force_n` 50 |
| `docking` (기존 섹션, 변경분) | `settle_time_s` 1.0 → **0.0**, `depth_calibration_max_lateral_m` 0.015, `depth_calibration_max_axis_deg` 1.0, `depth_calibration_samples` 10 |
| `logging` (추가분) | `motion_trail` true, `motion_trail_period_s` 1.0, `motion_trail_max_points` 600, `motion_hud` true |

기존 `docking.max_relative_velocity_mps`(0.05)는 검증된 도킹 조건이라 바꾸지 않았습니다. 이동 모드용 게이트는 `rendezvous.dock_max_*`에 따로 두었습니다.
