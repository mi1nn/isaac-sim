# MEP Ares1 Probe → Client Satellite 자동 도킹 — 작업 보고서

작업일: 2026-09-20 · 브랜치: `feature/apriltag` · 커밋: 아직 안 함
설계/사용 문서: [`mrv_probe_docking.md`](mrv_probe_docking.md)

> **결과 요약: 도킹 성공.** `--dock_only` headless 실행에서 **20/20 checks PASS, 최종 상태 SUCCESS**.
> 전송 → 정렬 → 도킹축 접근 → 삽입 → FixedJoint → 10 s 유지까지 통과했습니다.
> AI/ML 은 이번 단계에서 구현하지 않았습니다(데이터 수집만 준비).

---

## [CURRENT SYSTEM ANALYSIS]

코드를 고치기 전에 조사한 내용입니다. **Prim path 와 규약은 추측하지 않고 코드/USD 에서 확인**했습니다.

### 기존 코드 구조 (이미 있던 것, 재사용)

| 자산 | 위치 | 내용 |
|---|---|---|
| `DockingGeometry` | `docking.py` | USD 메시에서 프레임을 **실측** (하드코딩 없음) |
| `DockingManager.dock()` | `docking.py` | 도킹 FixedJoint (MEP ↔ 위성, 프로브 팁 anchor) |
| `build_thruster_collider` | `docking.py` | 중공 노즐 콜라이더 + 백플레이트 |
| `ArmKinematics` | `docking_demo.py` | DLS IK, `set_tool()` 로 제어 프레임 전환 |
| `CaptureManager` | `capture.py` | 포획 FixedJoint |
| 비전 포획 체인 | `vision_capture_demo.py`, `vision_task.py` | AprilTag → PnP → 예측 → 포획 |

**중요한 발견:** `VisionCaptureTask` 가 이미 `DockingTask` 를 상속하고 있어서, 비전 포획 씬에
위성·노즐 콜라이더·`DockingManager` 가 **이미 들어 있었습니다.** 새로 만들 필요가 없었습니다.

### 실제 Prim Path

```
MEP            /World/envs/env_0/debris                (scale 1.1)
Ares1 Probe    /World/envs/env_0/debris/Xform_Ares1
MEP 본체       .../debris/Fermi_Gamma_ray_Large_Area_Space_Telescope
위성           /World/envs/env_0/satellite
위성 바디      .../satellite/GOES_R
추력기         .../satellite/GOES_R/Thruster
팔 마지막 링크 /World/envs/env_0/robot/canadarm3_large_7
```

### Reference Point (시작 시 실측값)

| 항목 | 값 |
|---|---|
| `PROBE_DOCK_POINT` 팁 (MEP 바디) | `[-1.8104, -2.4356, 0.1065]` m |
| 프로브 축 (MEP 바디) | `[0.0, -0.9997, 0.0236]` |
| 프로브 팁 반경 | 0.080 m |
| `SAT_DOCK_POINT` (위성 바디) | `[-0.1657, -8.7247, -0.0194]` m |
| 도킹축 (위성 바디) | `[-0.0001, 1.0, 0.0008]` |
| `dock_depth` / `backstop_gap` | 0.8 m / 0.1 m |
| 도킹 지점 노즐 내경 | 0.466 m (프로브 대비 여유 386 mm) |

### 좌표계 · 규약 (코드에서 확인)

| 항목 | 규약 |
|---|---|
| 쿼터니언 | (w, x, y, z) |
| `Frame` | 회전행렬은 축이 열, `A @ B` = T_A_B 합성 |
| 중력 | 무중력 (`Domain.ORBIT`) |
| 시간 | physics dt 1/150 s, render interval 3 |
| 카메라 | `cam_wrist` 하나뿐. **프로브에는 카메라 prim 없음 → 신규 생성 필요** |

### 기존 속도 결정 요소

- 관절 드라이브 stiffness 40000 / damping 25000 → 관절 속도 ≈ 1.6 × lead
- `max_joint_step_rad` (lead) 가 실제 팔 속도를 결정
- 기존 도킹 데모 값: 운반 0.10 m/s, 정밀 구간 0.04 m/s, 단계 제한 240 s
- 팔 + 3 t 페이로드 모드: 주기 약 20 s, 감쇠비 ζ ≈ 0.10 (실측 진동과 일치)

---

## [IMPLEMENTED]

### 신규 파일

| 파일 | 내용 |
|---|---|
| `srb/tasks/manipulation/debris_capture/probe_dock.py` | 상대 pose / 깊이 / 조건 판정 (순수 numpy, 오프라인 테스트 가능) |
| `project/tests/test_probe_dock.py` | 오프라인 테스트 25개 |
| `docs/mrv_probe_docking.md` | 설계·사용 문서 |
| `docs/mrv_probe_docking_report.md` | 이 보고서 |

### 수정 파일

| 파일 | 수정 내용 | 이유 |
|---|---|---|
| `vision_capture_demo.py` | 기존 상태머신에 도킹 상태 11개 + 처리 메서드 추가 | 병렬 구조를 만들지 말라는 요구 |
| `vision_task.py` | `cam_probe` RGB-D 카메라, 위성 자유유영, 도킹 표시 디스크 숨김 | 깊이 센서와 자유유영 조건 |
| `vision.py` | `docking:` / `probe_camera:` 설정 + 검증 | 설정 노출 |
| `config/vision_capture.yaml` | 두 섹션 추가 | 값 조정 가능하게 |
| `scripts/vision_capture.py` | `--dock`, `--dock_only` | 실행 진입점 |

### 로직 — 좌표 규약 (핵심)

오차를 **world XYZ 로 재지 않습니다.** 전부 위성 도킹 프레임 D 에서 봅니다.

```
T_W_P  프로브 : 원점 = 팁,             +Z = 삽입 방향
T_W_D  도킹부 : 원점 = SAT_DOCK_POINT, +Z = 노즐 안쪽

e = T_W_D⁻¹ · T_W_P
e.pos = [e_x, e_y, e_z]     e_z < 0 → 아직 바깥
남은 삽입 거리 = -e_z
```

- Euler 각을 **누적하지 않습니다.** 게이트는 회전행렬에서 뽑은 `axis_deg`(축 기울기), `roll_deg`(축 둘레)
- `roll/pitch/yaw` 는 사람이 보는 표시·로그 전용
- Pre-dock 위치도 world Z offset 이 아니라 `P_dock - d · n_dock` (도킹축 기준)

**명시:** 이 단계의 상대 pose 는 **시뮬레이션 바디 pose 에서 계산**합니다. 즉 transform(= Ground Truth)
기반이며 **비전 추정이 아닙니다.** 위성을 보는 카메라는 없습니다. 깊이 카메라는 독립적인 교차검증
으로만 씁니다. GT 를 비전 측정이라고 표현하지 않았습니다.

### 로직 — 상태 머신 (기존 머신에 통합)

```
(포획 SUCCESS 또는 --dock_only)
 → DOCK_TARGET_ACQUIRE → PRE_DOCK_APPROACH → XY_ALIGN → ORIENTATION_ALIGN
 → ALIGNMENT_CHECK → Z_APPROACH ⇄ (정렬 이탈 시 XY_ALIGN 복귀) → FINAL_INSERTION
 → DOCK_READY → DOCKED → DOCK_HOLDING → SUCCESS        (실패: DOCK_FAILED)
```

- `XY_ALIGN` / `ORIENTATION_ALIGN` / `ALIGNMENT_CHECK` 는 **축 거리를 고정**한 채 횡/자세만 줄입니다
- `Z_APPROACH` 는 매 스텝 두 바디 pose 를 다시 읽는 closed loop. 정렬이 복도를 벗어나면 전진을
  멈추고 정렬로 복귀
- 속도 3구간: 먼 곳 `approach_speed` → `slow_zone` 램프 → `insertion_zone` 저속

### 로직 — Fail-safe (전진하지 않는 조건)

- 깊이가 무효/불일치 → **전진 정지** (무조건 전진하는 fallback 없음)
- 정렬이 `approach_abort_*` 이탈 → 전진 정지 + 정렬 복귀
- 도킹 타깃이 한 스텝에 250 mm 이상 점프 → `DOCK_FAILED`
- MEP 가 팔에서 떨어짐 / pose 가 유한하지 않음 → `DOCK_FAILED`
- 물리 불안정(NaN, 속도 폭주) → `PHYSICS_ERROR`
- 단계 제한 240 s, 전체 phase 제한 600 s (상태 왕복이 단계 제한을 계속 리셋하므로 별도 필요)

### 로직 — 도킹 조건 (모두 만족해야 FixedJoint 생성)

축 오차 · 횡 오차 · 축 기울기 · roll · 상대 선속도 · 추적 유효 · 깊이 유효 · 벽 여유.
**근접만으로는 절대 joint 를 만들지 않습니다.**

### 로직 — RGB-D 카메라 (`cam_probe`)

- 프로브 축 위, 팁보다 20 mm **앞** → 로드가 주광선을 가리지 않음
- 위치/자세는 시작 시 실측한 `probe_dock` 에서 계산 (하드코딩 없음)
- RGB = 운용자 확인용 (`<tag>_probe_rgb/`), Depth = `distance_to_image_plane`
- 깊이 → 거리: `남은 삽입 거리 = depth_raw - camera_to_tip - surface_offset`
- `surface_offset` 은 도킹축에 정렬된 pre-dock 자세에서 **한 번 보정**.
  측정값 **1.527 m** (설정 기본 0.1 m). 백플레이트가 충돌 전용이라 렌더링되지 않고 광선이 그 뒤
  추력기 메시에 닿기 때문입니다. `[-0.5, 3.0] m` 밖이면 거부.

---

## [SPEED OPTIMIZATION]

### 기존에 느렸던 원인과 조치

| 바꾼 것 | 전 → 후 | 근거 |
|---|---|---|
| `camera.supersample` | 2 → 1 | 시뮬 10 s 구간 벽시계 116 s → 39 s (약 3배) |
| 도킹 중 wrist 비전 처리 | 매 프레임 → 생략 | 도킹은 AprilTag 를 쓰지 않음 |
| `--dock_only` | 포획 30 s 포함 → 생략 | 도킹 반복 검증용 |
| `camera.render_on_vision_only` (headless 전용, 기본 off) | — | 시뮬 10 s 구간 116 s → 35 s |

### 안정성을 위해 **유지한** 제한

| 항목 | 값 | 이유 |
|---|---|---|
| 전송 속도 | 0.10 m/s | 기존 데모의 carry 속도. 0.25 m/s 로 올렸더니 ±150 mm 진동 잔류 → 되돌림 |
| 가속 제한 | 0.005 m/s² | 계단 입력이 3 t 페이로드를 옆으로 참 |
| 정렬 속도 | 0.008 m/s | 20 s 주기 진동을 다시 일으키지 않도록 |
| 삽입 속도 | 0.01 m/s | 충돌/관통/overshoot 방지 |
| 단계 제한 | 240 s | 기존 도킹 데모와 동일 |

**임계값은 하나도 느슨하게 만들지 않았습니다.** 기존 값(`pre_dock_radial` 10 mm, `dock_radial` 10 mm,
`dock_axis` 0.5°)을 그대로 재사용했습니다.

---

## Demo Time Optimization

### 작업 상태

- Isaac Sim 실행: **미실행**
- 실제 시뮬레이션 검증: **미실행**
- 코드 및 설정 수정: **완료** (아래 값만 변경)
- 설정 로드/순서 검증: `load_vision_config()` 가 새 값으로 오류 없이 로드됨 (Isaac 없이 확인). 이것은 시뮬레이션 검증이 아님

### 변경 목표

MEP 포획부터 도킹까지의 이동 거리를 줄이고, 불필요하게 느린 이동 속도를 소폭 증가시켜
향후 시연 시간을 단축할 수 있도록 설정을 조정한다. 기존 로직은 수정하지 않았다.

### MEP Position (로봇팔 ↔ MEP 상대 거리)

MEP 와 팔의 관측 자세는 모두 `DockingPlacementCfg.grasp_pos` `(5.0, -2.75, 3.0)` 에서 파생된다
(MEP 명목 자세 = 팔이 파지하는 자세, 팔 시작 자세 = 그 자세에서 `observe_distance_m` 만큼 후퇴).
그래서 `grasp_pos` 를 옮기면 MEP 와 팔이 **같이** 움직여 상대 거리는 그대로이고, 검증된 도달성만 깨진다.
상대 거리를 실제로 정하는 파라미터는 `approach.observe_distance_m` / `approach.approach_standoff_m` 이다.

Before:
- `approach.observe_distance_m` = 0.7 m (팔 시작 자세의 접촉면 ↔ MEP 면 간격)
- `approach.approach_standoff_m` = 0.6 m (Stage A/B 추종 간격, 이 뒤부터 저속으로 좁힘)
- MEP body 명목 pos (문서 실측값 인용) = (12.8046, -2.805, 4.8084) — 변경 없음
- 팔 시작 자세 접촉면 X ≈ 5.0 − 0.7 = 4.3 (기하 계산, 미실측)

After:
- `approach.observe_distance_m` = 0.6 m
- `approach.approach_standoff_m` = 0.5 m
- MEP body 명목 pos = 변경 없음 (world 좌표계는 그대로)
- 팔 시작 자세 접촉면 X ≈ 5.0 − 0.6 = 4.4 (기하 계산, 미실측)

- 변경 방향: 팔 시작 자세 쪽에서 MEP 방향(접근축 world +X)으로 0.1 m, 즉 MEP 가 팔에 0.1 m 더 가까움
- 변경량: observe −0.1 m (−14.3 %), standoff −0.1 m (−16.7 %). 두 값의 간격 0.1 m 는 그대로
- 새 관측 자세와 접근 구간은 기존 접근 구간(0.7 → 0.05 m) 안에 포함됨
- `mep.rendezvous_time_s`(15 s), `mep.linear_velocity_mps`(0.01 m/s), 드리프트 방향은 **변경 안 함**
  (MEP 시작 상류 오프셋 0.15 m 는 접근축과 수직 방향이라 팔 쪽으로의 거리가 아님)
- 태그 크기: 카메라–태그 거리가 약 1.09 m → 약 0.99 m 로 줄어 영상에서 태그가 커짐(계산상 ~34 px → ~37 px).
  판독에는 유리한 방향이지만 실행으로 확인하지 않음

파일: `project/config/vision_capture.yaml` (`approach:`), 동일 기본값 `project/srb/tasks/manipulation/debris_capture/vision.py` (`ApproachConfig`)

### Satellite Position

Before:
- `DockingPlacementCfg.dock_offset` = (0.0, 5.5, 0.0) m
- Satellite body(GOES_R) pos (문서 실측값 인용) = (23.9627, 2.3949, 3.1644)

After:
- `VISION_DOCK_OFFSET_M` = (0.0, 4.8, 0.0) m (`vision_task.py`, `VisionCaptureTaskCfg.__post_init__` 에서 적용)
- Satellite body pos = (23.9627, 1.6949, 3.1644) — Y 만 −0.7 이동한 **계산값**, 미실측. 자세(quat) 는 그대로
- 변경 방향: Probe(MEP) 쪽. MEP 가 +Y 로 이송되는 방향은 그대로이고, 위성 도킹 지점이 MEP 시작 쪽(−Y)으로 0.7 m 이동
- 변경량: −0.7 m (−12.7 %)
- 유지한 것: 도킹축(+X 접근), 노즐 방향, `pre_dock_distance_m`(1.0), `dock_depth`(0.8), 위성 자세, `roll_hint`
- 위성 배치는 `probe_docked @ sat_dock⁻¹` 로 역산되므로 프로브 팁이 SAT_DOCK_POINT 에 동축으로 오는 관계는 유지됨
  (오프셋만 평행이동)
- 기하 추정: pre-dock 까지 프로브 팁 이송 거리 ≈ √(5.5² + 1.8²) = 5.79 m → √(4.8² + 1.8²) = 5.13 m (약 −11 %). 시간이 아님
- 적용 범위: 비전 데모(`VisionCaptureTaskCfg`)에만 적용. GT 도킹 데모(`satellite_docking_demo.py`)의 `DockingPlacementCfg` 기본값
  (0.0, 5.5, 0.0)은 그대로이며 `satellite_docking_demo.md` 의 "초기 배치" 표도 유효

### Movement Speed

| Parameter | Before | After | 변경률 | 구간 |
|---|---|---|---|---|
| `approach.speed_far_mps` | 0.10 m/s | 0.11 m/s | +10 % | 포획 far 접근 (관측 → standoff) |
| `docking.transport_speed_mps` | 0.10 m/s | 0.11 m/s | +10 % | 도킹 pre-dock 자유공간 이송 |
| `docking.approach_speed_mps` | 0.10 m/s | 0.11 m/s | +10 % | 도킹축 접근의 far 구간 (`slow_zone_m` 0.30 m 밖) |

파일: `project/config/vision_capture.yaml`, 동일 기본값 `vision.py`(`speed_far_mps`), `probe_dock.py`(`transport_speed_mps`, `approach_speed_mps`)

- `transport ≥ approach ≥ near ≥ insertion` 순서 제약은 유지됨 (0.11 / 0.11 / 0.03 / 0.01)
- 위 "안정성을 위해 유지한 제한" 표의 **전송 속도 0.10 m/s 는 이번에 0.11 m/s 로 바뀜.**
  0.10 이 실행으로 검증된 값이고 0.11 은 검증되지 않음. 0.25 m/s 에서 ±150 mm 진동이 남은 이력이 있어,
  실행 시 pre-dock 도달 시점의 진동 크기를 가장 먼저 확인해야 함

### 변경하지 않은 항목

- 정밀 구간 속도: `speed_near_mps`(0.04), `speed_capture_range_mps`(0.01), `speed_ang_deg_s`, `near_speed_mps`(0.03),
  `insertion_speed_mps`(0.01), `align_speed_mps`(0.008), `align_speed_deg_s`, `creep_speed_mps`
- 진동 억제 관련: `accel_mps2`(0.005), `decel_gain_hz`(0.05), `swing_damping`, `settle_window_s/m`
- `search_speed_mps` 등 SEARCH 파라미터 (기본 시작 자세에서는 사용되지 않고, 과거 더 공격적인 값이 IK 를 다른 해로 보낸 기록이 있음)
- MEP Capture / Attachment / Compliance / Docking / AprilTag 로직, ROS 2 구조, State Machine, Robot control logic
- Physics 설정, Collision 설정, MEP mass, simulation timestep
- 게이트·타임아웃 값(`gate_*`, `align_*`, `dock_*`, `stage_timeout_s`, `phase_timeout_s`)
- `grasp_pos`, `approach_dir`, `roll_hint`, `mep.rendezvous_time_s`, 드리프트 속도·방향

### 이전 검증 결과의 적용 범위

이 보고서의 `[RUNTIME_VALIDATED]` (20/20 checks PASS, 도킹 SUCCESS) 는 **변경 전 값(dock_offset 5.5 m, 속도 0.10 m/s)** 으로
얻은 결과이다. 위 변경 후의 값으로는 다시 실행하지 않았으므로 같은 결과가 유지되는지는 알 수 없다.

### 실행 검증

현재 Isaac Sim 실행 환경이 없어 실제 실행 검증은 수행하지 않음.
향후 Isaac Sim 실행 환경이 확보되면 다음 항목을 별도로 검증해야 함.

- MEP Capture (관측 0.6 m / standoff 0.5 m 에서 태그 검출·PnP·예측이 유지되는지)
- Attachment
- Robot movement (특히 이동한 pre-dock / 도킹 자세 `(3.2, 2.05, 3.0)` 부근의 도달성·관절 한계. 비전 데모는 PLAN 도달성 검사를 하지 않음)
- AprilTag tracking
- Compliance
- Docking (위성이 −Y 로 0.7 m 이동한 배치에서 MEP 시작 자세·팔과의 간섭 여부)
- Physics stability (전송 0.11 m/s 에서 3 t 페이로드 진동)
- 전체 시연 시간 (변경 전후 비교. 이번 변경이 시간을 줄이는지는 측정 전까지 알 수 없음)

---

## [STATIC_CHECKED]

- Python 문법 / import / 클래스·함수 참조
- transform 곱 순서, 쿼터니언 순서 (w,x,y,z), 좌표 프레임, 단위
- 설정 로드와 검증 (불가능한 값 조합 거부)
- 상태 전이 경로, 깊이 검증 규칙, 도킹 조건
- 오프라인 테스트 **54개 통과** (`test_probe_dock.py` 25 + `test_vision_math.py` 29)

---

## [RUNTIME_VALIDATED]

`--dock_only` headless 실행. **20/20 checks PASS, 최종 상태 SUCCESS.**

### 도킹 순간 (모두 GT 실측)

| 항목 | 측정값 | 한계 |
|---|---|---|
| 축 오차 (axial) | **-9.98 mm** | 10 mm |
| 횡 오차 (radial) | **6.53 mm** | 10 mm |
| 축 기울기 | **0.025°** | 0.5° |
| 축 둘레 roll | **0.013°** | 1.0° |
| 상대 선속도 | **4.6 mm/s** | 20 mm/s |
| 삽입 깊이 | 노즐 안쪽 0.790 m | — |
| 프로브-벽 최소 여유 | **381 mm** | 20 mm |

### 도킹 후

| 항목 | 결과 |
|---|---|
| 10 s 유지 | MEP-위성 상대 드리프트 **0.000 mm / 0.0000°**, joint 계속 존재 |
| 관통 | 없음 (최소 여유 381 mm) |
| 재정렬 복귀 | 3 회 (closed loop 이 설계대로 동작) |
| 소요 | 시뮬 424 s (벽시계 약 31 분) |

### 깊이 vs 기하 거리 (TEST 6)

| 항목 | 값 |
|---|---|
| 샘플 (보정 이후) | 2649 |
| 평균 절대 차이 | **4 mm** |
| 상관 | **0.9982** |
| 시작 → 종료 | 깊이 1.800 → 0.010 m, 기하 1.800 → 0.010 m (**함께 감소**) |
| 밴드(150 mm) 이탈 | 33 프레임 (1.2 %) — 그 동안 프로브는 설계대로 **정지** |

---

## 실행으로 찾아 고친 문제

| 증상 | 원인 | 조치 |
|---|---|---|
| 팁이 100 mm/s 지령에 20 mm/s 로만 기어감 | 관절 속도 목표를 0 으로 줘서 PD 드라이브가 **자기 위치 목표에 제동** | 기준 궤적의 속도를 피드포워드 |
| 깊이가 전부 빈 값 | **USD 카메라는 자기 -Z 를 보는데** +Z 로 배치 → 뒤를 봄 | prim 에 USD 규약 직접 적용 |
| 깊이가 항상 0.93 m (출구 거리) | 노즐 출구의 **반투명 도킹 표시 디스크**가 광선 차단 (깊이는 불투명도 무관하게 첫 충돌 기록) | 프로브 카메라 사용 시 디스크 숨김 |
| 횡오차 ±150 mm 진동이 안 줄어듦 | 지연 게이트의 on/off 가 진동을 **정류**해 공진을 펌핑 | 기준 궤적을 연속 전진으로 변경 |
| 정렬이 자기 진동을 따라감 | 목표가 **실측 축거리**를 따라감 | 정렬 중 축 거리 고정 |
| Z 접근 시작 1 s 만에 횡오차 20 mm | 속도 0→0.1 m/s 계단 입력이 3 t 페이로드를 옆으로 참 | 가속 제한 `accel_mps2` |
| 목표 도달 후 진동이 안 잦아듦 | `ik_joint_target` 이 매 스텝 **측정 관절값**에서 목표를 만들어 진동을 따라감 → 복원력 0 | 기준이 멈추면 관절 목표 래치(`hold`) |
| PRE_DOCK 120 s 제한 초과 | 수렴 중인데 제한이 짧았음 | 240 s (기존 데모와 동일) |
| 깊이 검증이 전송 구간까지 포함 | 그 구간엔 광선이 노즐을 안 봄 | 보정 이후 구간만 판정 |
| `--dock_only` 에서 포획 check 가 FAIL | 실행되지 않은 단계의 check | 해당 모드에서 기록하지 않음 |

### 시도했다가 되돌린 것

| 시도 | 결과 | 현재 |
|---|---|---|
| 관절 속도 목표에 감쇠 외란 (gain 1.0) | **0.1 s 만에 불안정** (MEP 5 deg/s) | 기본값 0.0, 설정으로 남김 |
| 관절 목표 누적 적분 | 강성 40000 에서 **과도 토크로 불안정** | 되돌림, 래치 방식 채택 |

---

## [NEEDS_ISAAC_VALIDATION]

실제 Isaac Sim 에서 **아직 확인하지 않은** 항목입니다. 성공했다고 표현하지 않습니다.

- **GUI 실행** — headless 로만 확인
- **포획 → 도킹 전체 체인** (`--dock`) — `--dock_only` 로만 확인.
  포획 지점이 명목 자세와 다르고 MEP 가 표류한 상태로 넘어가므로 전송 거리·자세가 달라집니다
- **위성 표류** (`satellite_velocity_mps > 0`) — 0 으로만 확인.
  코드에 위성 속도 피드포워드는 있지만 실행 확인 안 함
- **고의 X/Y 오프셋 주입 시 전진 정지** (TEST 7 후반) — 자연 발생 재정렬 3 회만 확인
- **접촉력 기반 충돌 감지** — MEP 부착 중에는 기존 접촉 검사가 동작하지 않음.
  현재는 기하 여유(`wall_clearance`)로만 판정
- **되돌린 두 제어 옵션**(`swing_damping`, 목표 적분)의 안정적인 이득 값

---

## 실행 명령

```bash
cd ~/space_robotics_bench

# 도킹만 빠르게 (검증된 경로)
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag dock --headless --dock_only

# GUI 로 보기
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag dock_gui --dock_only

# 포획(AprilTag) → 도킹 전체 체인  [미검증]
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag full --dock

# 위성도 표류시키기  [미검증]
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag drift --headless --dock_only \
  --set docking.satellite_velocity_mps=0.004 --set "docking.satellite_drift_direction=[0.0,-1.0,0.0]"

# 오프라인 수학 테스트 (시뮬레이터 불필요)
cd project && ~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py tests/test_vision_math.py -q
```

출력: `project/logs/vision_capture/<tag>_{result.json,docking.csv,probe_rgb/}`

---

## 로그 (`<tag>_docking.csv`) — 향후 AI 학습 데이터용

`run_id`, `timestamp`, `state`,
`probe_tip_{x,y,z}` + `probe_tip_q{w,x,y,z}`, `sat_dock_{x,y,z}` + `sat_dock_q{w,x,y,z}`,
`relative_{x,y,z}` (도킹 프레임), `roll/pitch/yaw_error_deg`, `orientation_error_deg`,
`axis_error_deg`, `roll_about_axis_deg`, `lateral_error_m`,
`geometry_distance_m`, `depth_distance_m`, `depth_raw_m`, `depth_pixels`,
`insertion_depth_m`, `wall_clearance_m`,
`relative_v{x,y,z}`, `relative_speed_mps`, `commanded_speed_mps`,
`alignment_valid`, `depth_valid`, `dock_ready`, `dock_success`

이번 단계에서 AI 는 구현하지 않았습니다. 데이터 수집만 준비했습니다.
