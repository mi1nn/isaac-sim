# MRV 접근 → 로봇팔 전개 → AprilTag 탐색 → 기존 Capture/Transport/Docking (진행 로그)

요청 문서: [`dynamic_mep_capture_docking_pipeline_prompt.md`](dynamic_mep_capture_docking_pipeline_prompt.md)
관련 문서: [`mrv_six_dof_capture.md`](mrv_six_dof_capture.md), [`mrv_phase1_vision_capture.md`](mrv_phase1_vision_capture.md), [`mrv_probe_docking.md`](mrv_probe_docking.md)

> 상태: **구현 완료 / 정적 검토 완료 / headless 런타임 검증 완료**
> (아래 "런타임 검증" 의 실행 결과만 검증된 것입니다. GUI 시연 확인은 사용자 실행 대상입니다.)

---

## Progress

### Existing Pipeline (이번 작업 이전에 검증된 것, 변경하지 않음)

- [x] MEP Capture verified
- [x] Dynamic MEP verified (`mep.motion_mode: six_dof`)
- [x] Transport verified
- [x] Docking verified

### New Pipeline

- [x] Initial MRV position (MEP 에서 충분히 떨어진 시작 위치)
- [x] Robot Arm folded pose
- [x] MRV translation step 1 (수평, world +X)
- [x] MRV translation step 2 (수직, world +Z)
- [x] Robot Arm deployment (joint interpolation)
- [x] Camera configuration (기존 `cam_wrist` 재사용, 변경 없음)
- [x] AprilTag detection (기존 탐색 로직에 연결)
- [x] AprilTag pose estimation (기존)
- [x] MEP relative pose estimation (기존)
- [x] gripper_fixture target calculation (기존 `Cylinder_01` 기준, 변경 없음)
- [x] Existing MEP Capture integration
- [x] Existing Transport integration
- [x] Existing Docking integration
- [x] 상태별 로그 출력
- [x] progress 문서 (이 파일)

### Thruster VFX

- [x] MRV 원본 USD 를 수정하지 않는 별도 Prim 레이어
- [x] 이동 단계와 ON/OFF 동기화
- [x] 실제 이동 벡터 기준 plume 방향
- [x] 실패 격리 (VFX 실패 시 파이프라인 계속)

---

## 최종 상태 머신

```
INIT
  ↓  (mrv.enabled: true)
MRV_MOVE_STEP_1      ── Thruster VFX ON  (plume = −이동방향)
  ↓
MRV_STEP_1_REACHED   ── VFX OFF
  ↓
MRV_MOVE_STEP_2      ── VFX ON
  ↓
MRV_STEP_2_REACHED   ── VFX OFF
  ↓
ARM_DEPLOY           ── VFX OFF, folded → 기존 파이프라인 시작 자세 (joint 보간)
  ↓
SEARCH → TAG_DETECTED → POSE_ESTIMATED → PREDICTING        ← 여기부터 전부 기존 코드
  → APPROACHING → SLOW_APPROACH → CAPTURE_ATTEMPT → CAPTURED
  → HOLDING → (RETREAT) 
  → DOCK_TARGET_ACQUIRE → PRE_DOCK_APPROACH → XY_ALIGN → ORIENTATION_ALIGN
  → ALIGNMENT_CHECK → Z_APPROACH → FINAL_INSERTION → DOCK_READY → DOCKED
  → DOCK_HOLDING → SUCCESS
```

실패 상태 `MRV_APPROACH_FAILED` 가 추가되었습니다 (기존 실패 상태는 변경 없음).

`mrv.enabled: false` 이면 `INIT → SEARCH` 로 바로 넘어가고, 이번 작업 이전과 완전히 동일하게 동작합니다.

---

## 실행 명령

검증된 Dynamic 조건 그대로입니다. 기존 CLI 옵션의 의미는 바꾸지 않았습니다.

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag full_6dof --dock \
  --start_yaw_deg 15 \
  --set mep.motion_mode=six_dof \
  --set "mep.angular_velocity_rad_s=[0.005,-0.004,0.006]"
```

추가된 옵션:

| 옵션 | 의미 |
|---|---|
| `--no_mrv_approach` | 접근 단계를 건너뛰고 예전처럼 관측 자세에서 시작 (회귀 확인용) |
| `--mrv_approach` | 설정에서 꺼져 있어도 접근 단계를 강제로 켬 |

`--scenario static` (Test 1, 자세 정확도 측정) 과 `--dock_only` 는 접근 단계가 의미가 없으므로 자동으로 꺼집니다.

---

## 좌표계 확인 결과 (축을 가정하지 않고 실제 값에서 결정)

| 항목 | 값 | 근거 |
|---|---|---|
| 접근 축 | world **+X** | `DockingPlacementCfg.approach_dir = (1, 0, 0)` |
| MEP 공칭 포획 위치 | `grasp_pos = (5.0, −2.75, 3.0)` | `DockingPlacementCfg` |
| MRV (팔 베이스) | world 원점 부근, 고정 베이스 | `canadarm3_large_0/root_joint` (world 고정 조인트) |
| MRV 선체 | `{ENV}/scenery` (Venus Express, scale 3.4, yaw 90°) | `TaskCfg.scenery` |
| 위성 | MEP 에서 `dock_offset = (0, 3.2, 0)` | `VISION_DOCK_OFFSET_M` |

따라서 **1차 이동 = +X (수평, MEP 쪽으로)**, **2차 이동 = +Z (수직, 작업 고도까지)** 로 결정했습니다.
MRV 는 공칭 위치에서 `start_offset_m = (−5, 0, −3)` 만큼 떨어진 곳에서 시작하고, 두 단계가 이 오프셋을 정확히 상쇄합니다.

---

## 구현 요점

### 1. MRV 는 무엇인가

이 씬에서 MRV 는 **Canadarm3 의 고정 베이스 (`{ENV}/robot`, articulation) + 그것이 얹힌 선체 (`{ENV}/scenery`, 정적 XForm)** 입니다.
자유비행 강체가 아니므로 추력으로 움직이지 않고, 두 prim 을 같은 오프셋으로 **강체 평행이동** 시킵니다.

- articulation: `write_root_pose_to_sim` (관절 위치는 건드리지 않고 전체가 같이 이동)
- 선체: `scene.extras["scenery"].set_world_poses`
- 이동할 때마다 `ArmKinematics.refresh_base()` 로 IK 기준 베이스 프레임 갱신

회전은 전혀 주지 않습니다 (순수 평행이동).

### 2. 기존 파이프라인 보호 — "오프셋을 정확히 상쇄"

두 이동 단계가 끝나면 MRV 는 **검증된 공칭 위치와 정확히 같은 곳**에 있습니다.
즉 `ARM_DEPLOY` 이후의 씬은 기존에 성공한 구성과 동일합니다.
런타임 측정값: articulation root 가 공칭 위치에서 **0.0 mm** (허용 20 mm).

팔의 전개 목표는 새로 만들지 않고, **기존 파이프라인이 시작하던 바로 그 자세** (관측 자세, `--start_yaw_deg` 가 있으면 그 yaw 자세) 의 IK 해를 그대로 씁니다.

### 3. MEP Dynamic motion 유지 (정지시키지 않음)

MEP 는 `six_dof` 로 계속 유영 + 회전합니다. 속도는 한 번도 건드리지 않습니다.
다만 접근 단계(약 42 s)만큼 MEP 가 더 움직이므로, **자유비행을 그만큼 뒤로 되감아** t=0 에 배치합니다
(`back_propagate_free_body`: COM 은 직선, 자세는 COM 기준 상수 각속도 — PhysX 의 실제 모델).
그 결과 **capture 가 시작되는 순간의 MEP 상태 = 기존 검증 런의 t=0 상태** 가 됩니다.

> 처음에는 `mep.rendezvous_time_s` 를 접근 시간만큼 늘리는 방식을 썼는데 실패했습니다.
> `rendezvous_start_pose` 는 역회전을 `Cylinder_01` 기준으로 하지만 PhysX 는 **질량중심**(Cylinder_01 에서 6.6 m) 기준으로 돌리기 때문에,
> 역회전 각이 3.6배가 되면서 Cylinder_01 이 공칭 위치에서 ~0.3 m 벗어났고 인계 시점에 태그가 4개 중 2개만 보여 `TAG_LOST` 가 났습니다
> (`mrv_smoke_result.json`). 되감기 방식으로 바꾼 뒤 4/4 태그가 잡힙니다.

### 4. Folded arm — 추측이 아니라 에셋에서 확인한 값

`canadarm3_large.usdz` 의 조인트 프레임을 직접 읽어서 결정했습니다.

| 확인 항목 | 값 |
|---|---|
| 조인트 이름 | `canadarm3_large_joint_1..7` |
| 조인트 축 | Z, Y, X, X, X, Y, Z |
| 조인트 limit | 전부 `(-inf, +inf)` — 연속 회전 조인트 |
| 기존 home pose | `Canadarm3.asset_cfg.init_state.joint_pos` = (50, 0, 55, 75, −30, 0, 0)° |
| 링크 3 → 4 오프셋 | +3.658 m (Z) |
| 링크 4 → 5 오프셋 | −3.663 m (Z) |

즉 **모든 조인트가 0 이면 3.66 m 붐 두 개가 서로 맞접혀 있는 = 접힌 자세**입니다 (에셋의 rest pose).
여기에 두 가지만 더했습니다.

- `joint_1 = 50°`: 기존 `init_state` 와 같은 방위각. 전개가 "붐을 펴는" 동작만 하면 되도록.
- `joint_6 = −90°`: 손목을 수평으로 접어 어떤 링크도 선체 아래로 내려가지 않게 (joint_6=0 이면 EE 가 베이스 평면 −0.43 m, −90° 면 +0.59 m).

기본값 `folded_joint_deg: [50, 0, 0, 0, 0, −90, 0]`.
런타임 측정: 접힘 상태 EE 가 베이스에서 **2.24 m**, 전개 상태 **6.02 m**.

전개는 `q(t) = q_folded + smoothstep(t/T)·(q_deployed − q_folded)` 로 보간합니다 (급격한 조인트 변경 없음).

> 조인트가 연속(limit 없음)이라 IK 가 반환하는 해는 몇 바퀴 감긴 값일 수 있습니다.
> 그대로 보간하면 조인트가 수천 도를 회전하므로, 전개 목표를 folded 기준 ±180° 로 되감아서 씁니다 (`wrap_joint_target`).

### 5. 이동 완료 판정 — `sleep()` 아님

각 단계는 사다리꼴 속도 프로파일(가속 → 순항 → 감속)로 목표까지 가고,
완료는 **시뮬레이션에서 읽은 articulation root 위치와 목표의 거리**로 판정합니다.

```python
err = |root_pos_w − (nominal + target_offset)|
if leg.remaining <= 0 and err <= mrv.position_tolerance_m:  # 20 mm
    leg complete
```

`stage_timeout_s` (120 s) 안에 도달하지 못하면 `MRV_APPROACH_FAILED` 로 명확히 실패합니다.
전개 역시 시간이 아니라 **전 조인트 오차 < `deploy_tolerance_deg`** 로 종료합니다 (`deploy_settle_s` 는 상한).

### 6. Thruster VFX — 기존 파이프라인과 분리

| 요구사항 | 구현 |
|---|---|
| MRV 원본 USD 수정 금지 | `{ENV}/mrv_thruster_vfx` 아래에 런타임 생성. 선체/로봇 prim 은 손대지 않음 |
| 실제 thruster 위치 확인 | `VenusExpress.actions.thrust.thrusters` 의 17개 `ThrusterCfg.offset` 을 읽어 (spawn scale 적용) 클러스터 박스를 만듦. 좌표 하드코딩 없음 |
| 실제 추력 금지 | 메시만 생성. Collision/RigidBody/Physics API 를 일절 적용하지 않음 |
| 이동 방향과 반대 | plume 방향 = `−leg.dir` (설정 축이 아니라 **실제 이동 벡터**) |
| 단계 동기화 | `MRV_MOVE_STEP_1/2` 이고 참조 속도 > 0 일 때만 ON, 그 외 전부 OFF |
| 실패 격리 | 생성/갱신 실패를 `try/except` 로 잡고 경고만 남긴 뒤 계속 진행 |
| 인식 방해 금지 | plume 은 **진행 반대쪽 면**에만 (1차: −X, 2차: −Z). 카메라/MEP 는 +X/+Z 쪽. 또 탐색·포획 중에는 항상 OFF |

표현은 투명도가 다른 원뿔 3겹(노즐 쪽 밝음 → 바깥쪽 투명) + 길이 flicker 로, 파티클 시스템은 쓰지 않습니다.
런타임 측정 thruster 클러스터 half extent: `[2.92, 2.54, 3.19] m` (선체 body frame).

---

## 변경한 파일

| 파일 | 변경 |
|---|---|
| `project/srb/tasks/manipulation/debris_capture/mrv_approach.py` | **신규.** 설정, 이동 프로파일, MRV 평행이동, 자유비행 되감기, Thruster VFX, 전개 보간 |
| `.../vision_capture_demo.py` | 상태 5개 + 실패 상태 1개 추가, `begin_mrv_approach` / 단계 핸들러 / 상태 로그, capture 시계(`capture_clock`) 분리 |
| `.../vision.py` | `VisionCaptureConfig.mrv` 추가 + 검증 호출 |
| `.../vision_task.py` | 주석만 (MEP 배치 로직은 그대로) |
| `.../docking_demo.py` | `ArmKinematics.refresh_base()` 추가 (베이스가 움직이지 않는 기존 시나리오에서는 같은 값을 다시 읽을 뿐) |
| `project/config/vision_capture.yaml` | `mrv:` 섹션 추가 |
| `project/scripts/vision_capture.py` | `--mrv_approach` / `--no_mrv_approach` |

**건드리지 않은 것**: Capture mechanism(`capture.py`, `CaptureManager`), Transport, Satellite Approach, Docking(`docking.py`, `probe_dock.py`), MEP Dynamic motion 설정, AprilTag detection / pose estimation, `gripper_fixture`/`Cylinder_01` 기준.

기존 capture 단계의 제한 시간(`test.dynamic_timeout_sec`)은 **capture 가 시작된 시점부터** 재므로 의미가 바뀌지 않습니다
(접근 단계가 없으면 `capture_clock() == sim_time` 으로 예전과 동일).

---

## Configuration

`project/config/vision_capture.yaml` 의 `mrv:` 섹션 (전부 `--set mrv.key=value` 로 덮어쓰기 가능).

| 키 | 기본값 | 의미 |
|---|---|---|
| `enabled` | `true` | 접근 단계 on/off |
| `start_offset_m` | `[-5.0, 0.0, -3.0]` | 공칭 위치 대비 MRV 시작 오프셋 (= `MRV_INITIAL_OFFSET`) |
| `step1_axis_mask` | `[1, 1, 0]` | 1차 이동이 상쇄하는 성분 (= `MRV_APPROACH_STEP_1` / `STEP_2` 분할) |
| `speed_mps` / `accel_mps2` | `0.4` / `0.15` | 이동 속도 / 가속도 |
| `position_tolerance_m` | `0.02` | `POSITION_TOLERANCE` |
| `settle_time_s` | `1.0` | 단계 사이 정지 (두 단계가 눈에 구분되도록) |
| `folded_joint_deg` | `[50,0,0,0,0,-90,0]` | 접힌 자세 |
| `deploy_duration_s` | `10.0` | `ARM_DEPLOY_DURATION` |
| `deploy_settle_s` / `deploy_tolerance_deg` | `4.0` / `0.1` | 전개 후 정착 상한 / 종료 조건 |
| `stage_timeout_s` | `120.0` | 단계별 타임아웃 |
| `mep_rendezvous_delay_s` | `-1.0` (auto) | 자유비행 되감기 시간 |
| `vfx_enabled`, `plume_*` | — | Thruster VFX |

`APRILTAG_SEARCH_TIMEOUT`, `APRILTAG_ID`, `CAPTURE_DISTANCE` 는 **새로 만들지 않고 기존 값을 그대로 씁니다**:
`approach.stage_timeout_s`, `apriltag.ids: [0,1,2,3]`, `capture.max_distance_m` / `approach.final_gap_m`.

시연 길이: 1차 15.2 s + 2차 10.2 s + 정지 2 s + 전개 10 s + 정착 ≈ **약 40 s**.
더 길거나 짧게 하려면 `start_offset_m` 과 `speed_mps` 만 조정하면 됩니다.

---

## Notes

### Existing verified behavior

`--no_mrv_approach` 를 주면 `mrv.enabled=false` 가 되어 `INIT → SEARCH` 로 바로 갑니다.
이 경로에서는 `MrvTransit` / `ThrusterVfx` 가 아예 생성되지 않고, MEP 되감기도 0 s 이며, `capture_clock() == sim_time` 입니다.
즉 이번 작업 이전 코드와 동일한 실행 경로입니다.

### New implementation

위 "구현 요점" 참조.

### 정적 검토 (VFX 체크리스트, 요청 문서 10.11)

- [x] MRV 원본 USD 를 직접 수정하지 않았는가? → 런타임에 `{ENV}/mrv_thruster_vfx` 생성만
- [x] VFX 가 별도 Prim/레이어로 분리되어 있는가? → 선체/로봇의 자식이 아님
- [x] 실제 MRV 이동 로직과 VFX 로직이 분리되어 있는가? → `MrvTransit` (이동) / `ThrusterVfx` (표시), 호출도 분리
- [x] Thruster VFX 가 실제 Physics Force 를 발생시키지 않는가? → 메시 + 머티리얼만, 물리 API 없음
- [x] Step 1 / Step 2 이동 시 VFX ON/OFF 가 연결되어 있는가? → `MRV_MOTION` 상태 + 참조 속도 조건
- [x] 이동 종료 후 / Arm Deploy 이후 VFX 가 OFF 인가? → 상태가 `MRV_MOTION` 이 아니면 항상 OFF
- [x] AprilTag Detection 에 방해되지 않도록 배치되었는가? → 진행 반대쪽 면, 탐색 중에는 OFF
- [x] VFX 생성 실패가 전체 파이프라인 실패로 이어지지 않는가? → `try/except` + 경고 후 계속
- [x] 기존 Dynamic MEP / Capture / Transport / Docking 로직이 변경되지 않았는가? → 위 "변경한 파일" 참조

---

## 런타임 검증

### 실행 1 — 전체 파이프라인 (접근 + 포획 + 도킹), 2026-09-21 headless

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --headless --tag mrv_full_final --dock \
  --start_yaw_deg 15 \
  --set mep.motion_mode=six_dof \
  --set "mep.angular_velocity_rad_s=[0.005,-0.004,0.006]"
```

**결과: 37/38 checks PASS, final state `SUCCESS`** (`project/logs/vision_capture/mrv_full_final_result.json`)

| 단계 | sim 시각 | 측정값 |
|---|---|---|
| 자유비행 되감기 | t=0 | 41.8 s (MEP 1.54 m / 21.0° 뒤로) |
| `MRV_MOVE_STEP_1` (+X 5.00 m) | 0.5 → 15.64 s | 종료 시 베이스 오차 **0.00 mm** |
| `MRV_STEP_1_REACHED` | 15.64 → 16.64 s | plume OFF |
| `MRV_MOVE_STEP_2` (+Z 3.00 m) | 16.64 → 26.78 s | 종료 시 베이스 오차 **0.00 mm** |
| `MRV_STEP_2_REACHED` | 26.78 → 27.78 s | plume OFF |
| `ARM_DEPLOY` | 27.78 → 39.92 s | 최대 조인트 오차 **0.099°**, EE 12.2 mm |
| **capture 파이프라인 인계** | 39.92 s | MRV 공칭 위치 오차 0.0 mm |
| `SEARCH` → 4/4 태그 검출 | 46.37 s | PnP RMS **0.294 px**, inlier 16/16 |
| `CAPTURE_ATTEMPT` → `CAPTURED` | 72.12 s (capture 시계 **32.2 s** / 제한 60 s) | GT gap 52.5 mm, lateral 5.3 mm, angle 0.12°, 상대속도 8.4 mm/s, 상대각속도 0.0004 rad/s |
| `HOLDING` | 72.13 → 74.13 s | MEP-EE 상대 drift 최대 0.62 mm / 0.022° |
| Transport → 도킹 정렬 → 삽입 | 74.13 → 225.07 s | — |
| `DOCKED` → `DOCK_HOLDING` → `SUCCESS` | 225.07 → 235.08 s | 10 s 유지, MEP-위성 상대 drift 0.001 mm, 최소 벽 간격 320 mm |

총 sim 235.6 s / wall 957 s.

접힌 팔: EE 접촉점이 베이스에서 **2.24 m** (전개 시 6.02 m).
전개 목표 조인트 (IK 해를 folded 기준 ±180° 로 되감은 값):
`[-31.0, 47.8, 37.5, 89.6, -1.5, -27.8, 146.9]°`, 최대 조인트 이동 146.9°.

### 실행 2 — 회귀 확인 (접근 단계 없음), 동일 조건

```bash
... --tag base_nomrv --dock --no_mrv_approach --start_yaw_deg 15 --set mep.motion_mode=six_dof ...
```

**결과: 33/34 checks PASS, final state `SUCCESS`** (`base_nomrv_result.json`).
접근 단계 관련 4개 체크만 없고 나머지는 동일하게 통과 — 기존 경로가 그대로 동작합니다.

### 유일한 FAIL — `[DOCK3] Depth agrees with the geometric docking distance` (기존 문제)

| 런 | 평균 오차 | 최대 | 상관계수 | 최종 상태 |
|---|---|---|---|---|
| 접근 단계 포함 (`mrv_full_final`) | 117 mm | 407 mm | 0.9742 | SUCCESS |
| 접근 단계 없음 (`base_nomrv`) | 80 mm | 451 mm | 0.9748 | SUCCESS |

**두 런 모두 동일하게 실패**하므로 이번 작업이 만든 문제가 아닙니다.
`cam_probe` 깊이값이 노즐 내부에서 광선이 닿는 위치에 민감해 생기는 기존 센서 검사 항목이고,
깊이가 밴드를 벗어난 프레임에서는 전진이 보류될 뿐 도킹 자체는 양쪽 다 성공합니다.

### 실행 3 — 접근 + 포획만 (도킹 제외), 최종 코드 재확인

`--tag mrv_recheck` (도킹 없이 동일 조건): **27/27 PASS, `SUCCESS`**.
실행 1 이후의 마지막 두 수정(import 정렬, 쓰이지 않던 인자 제거)이 동작을 바꾸지 않았음을 확인한 것입니다.
접근 단계 측정값은 실행 1과 동일 (베이스 오차 0.0 mm, 전개 조인트 오차 0.099°).

### 실행 4 — Thruster VFX 오프스크린 렌더 확인

데모 루프를 그대로 돌리면서 광각 카메라 프레임을 저장해 확인했습니다
(스크립트는 저장소에 넣지 않은 scratch 도구입니다).

- 1차 이동: plume 4개가 **−X**(진행 반대) 방향으로 분출, 팔은 접힌 상태
- 단계 사이: plume **OFF**
- 2차 이동: plume이 **−Z**(아래) 방향, MRV는 +Z 로 상승
- `ARM_DEPLOY` / `SEARCH`: plume **OFF**, 팔이 전개되어 MEP 를 향함

렌더 중 발견해 고친 것:
- cone 면 winding 이 안쪽을 향해 밝은 배경에서 내부가 검게 보이던 문제 → winding 반전
- thruster 박스 모서리에 plume 을 배치해 선체에서 떨어져 보이던 문제 → `plume_spread` 0.8 → 0.45, `plume_inset` 추가

### 검증하지 않은 것

- **GUI 시연** (`--headless` 없이): 실행하지 않았습니다. 카메라 워크/시연 길이는 사용자 확인 대상입니다.
- `--scenario static` (Test 1) 은 접근 단계가 자동으로 꺼지므로 이번에 다시 돌리지 않았습니다.
- ROS 인터페이스(`--ros`)와 접근 단계의 조합은 실행하지 않았습니다 (접근 단계 중에는 vision/ROS telemetry 가 갱신되지 않습니다).

