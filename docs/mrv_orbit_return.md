# MRV — Client 정상 궤도(Reference Orbit) 시각화 · 무회전 유영 · 도킹 후 궤도 복귀

기존 문서: 포획 [`mrv_phase1_vision_capture.md`](mrv_phase1_vision_capture.md) · 도킹 [`mrv_probe_docking.md`](mrv_probe_docking.md)
코드: `srb/tasks/manipulation/debris_capture/orbit_return.py` (순수 numpy), `vision_task.py` (빨간 궤도 prim, Client 배치), `vision_capture_demo.py` (상태 머신)

> 상태: **`[NEEDS_ISAAC_VALIDATION]`** — 오프라인 테스트와 `--orbit-only` 실행 결과는 맨 아래 "검증 상태"에 있습니다.
> 궤도 복귀 전체 체인(`--orbit-return`)의 실행 결과는 그곳에 실행한 만큼만 적었습니다.

## 1. "정상 궤도"의 정의 — 지구(하늘 배경)와 실제 중력 궤도와의 차이

**지구는 3D 오브젝트가 아니라 하늘 배경(skydome)입니다.** `Domain.ORBIT` 은 `assets/srb_assets/skydome/low_res/low_earth_orbit.exr`
(equirectangular)을 쓰고, 이 이미지에서 지구는 **월드 −Z (nadir) 축을 중심으로 한 캡**입니다. 텍스처에서 측정한 값:

| 항목 | 값 |
|---|---|
| 지평선(limb) | 이미지 높이의 60.2 % 행 → **수평선 아래 18.3°** 의 일정한 고도 원 |
| 지구 각반경 | 90° − 18.3° = **71.7°** (nadir 중심) |
| 대응하는 궤도 고도 | cos 18.3° = R/(R+h) → R+h ≈ 6710 km (h ≈ 340 km, 저궤도) |
| 축 | 월드 Z. 캡은 Z 축 대칭이라 하늘 돔의 yaw 회전(스크립트가 고정함)은 그림에 영향이 없음 |

그래서 정상 궤도는 **지구 축과 동심인 수평 링(normal = +Z)** 이고, 시뮬레이션에서 지구가 보이는 쪽의 **호(arc)만** 그립니다
(`arc_span_deg`, 기본 80°, Client 의 명목 지점 중심). 링 전체를 그리지 않는 이유는 지구도 뒷면까지 다 보이지 않기 때문입니다.

이 환경은 `gravity = (0, 0, 0)` 자유유영이라 **케플러 궤도(중력, 이심률, 주기)를 계산하지 않습니다.**
빨간 호는 Client 위성이 돌아가 있어야 하는 **월드 좌표계의 참조 궤적(reference trajectory)** 입니다.

| | 이 환경의 reference orbit | 실제 궤도 |
|---|---|---|
| 형태 | 지구 축과 동심인 수평 원의 호 | 중력이 만드는 케플러 타원 |
| 크기 | **축소 그림**: 반지름 `radius_m` = 50 m (실제 ≈ 6 700 km) | 6 700 km |
| 시간 | 정적인 선. 속도·주기 없음 | 궤도 속도, 주기 |
| 유영 방향 | 호의 접선 (월드 Y, 수평) | 접선 (비행 방향) |
| Client 가 벗어나는 원인 | 초기 고도 오프셋 0.5 m | 섭동 |
| 복귀 수단 | Canadarm3 가 MEP–Client 결합체를 IK 로 이동 | 추력기 기동 |

**축소 그림의 한계 (시차)**: 지구는 무한히 먼 배경이고 호는 유한한 지오메트리라서, 둘이 동심으로 보이는 것은
**카메라가 링 축 위에 있을 때**입니다 (링은 그 눈에서 일정한 고도 −`view_elevation_deg` 의 원으로 보이므로 limb 과 평행).
기본 GUI 시작 시점과 `--orbit-only` 의 저장 영상이 이 위치(`OrbitSetup.view()`)를 씁니다.
카메라를 다른 곳으로 옮기면 호는 월드에 고정된 채 남고 지구 그림은 따라 움직이므로 어긋나 보입니다 (실제 지구가 무한히 멀기 때문).

궤도선 위에 "있다"는 것은 Client 기준점(아래 §2)이 호까지 최단거리 ≤ `arrival_tolerance_m` 이라는 뜻입니다.

## 2. 좌표계와 Client 기준점

```
궤도 평면   : 원점 center, 법선 normal_w = +Z (= −nadir), 평면 축 e1, e2 (e1 × e2 = normal)
              e1 = 월드 +X 를 평면에 투영 (법선이 X 에 가까우면 +Y),  e2 = normal × e1  → 기본은 e1 = +X, e2 = +Y
궤도 점     : P(θ) = center + a·cosθ·e1 + b·sinθ·e2,  θ ∈ [θ_nom − span/2, θ_nom + span/2]  (그리는/쓰는 호)
              원: a = b = radius_m       타원: a = semi_major_m, b = semi_minor_m
접선        : θ 증가 방향 (−a sinθ e1 + b cosθ e2). θ_nom = 0 에서 월드 +Y
Client 기준점: SAT_DOCK_POINT (기본)   ← 선택 근거 아래
```

**기준점으로 `SAT_DOCK_POINT` 를 선택했습니다** (`orbit_reference.client_reference: sat_dock_point`).

1. 기존 도킹 프레임과 일관됩니다. 모든 도킹 오차와 FixedJoint 앵커가 이 프레임 기준입니다.
2. 도킹 후에는 프로브 팁(= IK 제어점)과 같은 강체 점(오차 ≤ `dock_axial_m`)이라서, "Client 점이 목표에 도달하는 팔 목표"가 **자세 유지 + 병진 `(목표 − Client 점)`** 로 정확히 역산됩니다.
3. 위성 body origin 은 dock point 에서 약 8.7 m 떨어져 있어 팔 도달성이 도킹 프레임과 무관한 점에 묶입니다.

`client_reference: body_origin` 으로 바꿀 수 있습니다(같은 코드 경로).

## 3. Client 초기 오프셋

```
nominal 점  P_nom = P(θ_nom)                      (호 위의 명목 지점, 기본 θ_nom = 0)
Client 시작 = P_nom + client_initial_offset_m × dir
dir         = [0, 0, 1] (기본: 위쪽) | radial_outward | radial_inward | [x, y, z]
```

기본은 **고도 오차**입니다: Client 가 궤도 평면보다 0.5 m 위에서 시작합니다. 이렇게 한 데에는 이유가 하나 더 있습니다 —
도킹축은 수평(월드 ±X)이라서 궤도 평면(0.5 m 아래)과 어디에서도 0.5 m 보다 가까워지지 않습니다. 선이 깊이 카메라 주광선에 들어가지 않습니다 (§7).

기존에 검증된 위성 배치(도달성·도킹축)는 그대로 둡니다. **`center_w: null` (기본)** 이면 위 식을 거꾸로 풀어 궤도 중심을 유도합니다:
`center = Client 시작 − offset·dir − P_nom 의 중심 기준 위치`. 기본값에서는 중심이 Client 보다 반지름 50 m 만큼 팔 쪽(−X)에 놓이고 궤도 평면은 Client 의 0.5 m 아래입니다.
Client 는 정확히 `client_initial_offset_m` 만큼 궤도에서 벗어나 시작하고 위성은 움직이지 않습니다.
`center_w: [x, y, z]` 를 직접 주면 위성이 (nominal 점 + 오프셋)으로 옮겨집니다 — 이 경우 도킹 자세들의 도달성을 다시 확인하세요.

시작 시 다음을 검증합니다.

- Client 기준점 ↔ 호 최단거리 = `client_initial_offset_m` ± `initial_offset_tolerance_m` (5 mm)
- 그 거리가 `arrival_tolerance_m` 보다 큼 (궤도 위에서 시작하면 **실패**)

## 4. 회전 없는 등속 유영

| | 설정 |
|---|---|
| MEP | 기존 `translation_only` 경로. `drift.mep_*` |
| Client | `docking.satellite_*` 를 `drift.client_*` 로 덮어써 초기 선속도 지정 |
| 회전 | 둘 다 `angular velocity = 0` 강제 (`drift.angular_velocity_rad_s` 가 0 이 아니면 로드 시 오류) |
| 감쇠 | linear / angular damping 0 (시작 시 USD 값을 읽어 확인) |
| six_dof | 이 시나리오에서는 금지 (`mep.motion_mode` 가 `translation_only` 가 아니면 오류) |

`orbit_reference.enabled` 가 켜진 동안에만 `drift:` 가 `mep.*` / `docking.satellite_*` 를 덮어씁니다. 기본값(꺼짐)에서는 기존 시나리오가 그대로입니다.

**자유유영 감시** — 아직 포획되지 않은 MEP 와 아직 도킹되지 않은 Client 는 매 스텝 `|v − v_cmd| ≤ drift.velocity_tolerance_mps` (2 mm/s),
`|w| ≤ drift.angular_tolerance_rad_s` (0.002 rad/s) 여야 합니다. 넘으면 `PHYSICS_ERROR` 로 실패합니다.
시작, 도킹 직전, 도킹 직후, 홀딩 종료의 속도는 결과 JSON `metrics.orbit.motion_*` 에 남습니다.

**유영 방향 = 궤도 접선.** 기본 `(0, −1, 0)` 은 호의 접선(월드 Y, 수평)입니다. 그래서 MEP 와 Client 는 궤도를 따라 흐르고 (호가 반지름 50 m 라서
1.5 m 를 흘러도 호에서 2 cm 밖에 벗어나지 않음) 초기 오프셋이 유지됩니다. 팔의 접근축(+X) 에 수직이라 포획의 리드 예측 조건도 이전과 같습니다.
시작 시 `[ORBIT0] MEP and Client drift along the orbit (tangent at the nominal point)` 로 접선과의 각도를 확인합니다 (≤ 10°).
순수한 오프셋만 보려면 `--set drift.client_velocity_mps=0.0`.

## 5. 빨간 궤도선

- `/World/reference_orbit` (설정 `prim_path`, `/World/envs` 밖). 호 위 `segments`(≥128) 점을 따라가는 **8각 튜브 Mesh**, 지름 `line_width_m` (기본 0.25 m — 50 m 거리에서 약 4 px), `UsdPreviewSurface` emissive 빨강.
- `UsdGeom.BasisCurves` 대신 Mesh 를 쓴 이유: RTX 는 BasisCurves 의 굵기를 안정적으로 그리지 않아서, GUI 에서 항상 보이는 지오메트리를 택했습니다.
- **시각 전용**: rigid body, collider, contact sensor, 어떤 Physics/PhysX 스키마도 없습니다. 시작 시 이를 검사합니다 (`[ORBIT0] Red reference orbit prim, visual only`).
- 시작 로그에 중심, 법선, 반지름(반축), 호 범위와 양 끝점, segment 수, nominal 목표점 world pose, 접선을 출력합니다.
- `--orbit-only` 는 GUI 시작 시점과 같은 위치의 카메라(`cam_orbit_view`)로 `<tag>_orbit_view.png` 를 저장합니다. 빨간 호가 하늘의 지구 limb 과 나란히 지나가는지 GUI 없이 확인할 수 있습니다.

## 6. 상태 머신

기존 도킹 머신 뒤에 붙습니다 (앞부분은 바뀌지 않았습니다).

```
… → DOCK_READY → DOCKED
  → ORBIT_TARGET_ACQUIRE → ORBIT_TRANSFER → ORBIT_ARRIVAL_CHECK → ORBIT_HOLDING → SUCCESS
                                ↑______________ 보정 (최대 max_retargets) ______|
실패: ORBIT_FAILED   (기존: TAG_LOST … DOCK_FAILED, PHYSICS_ERROR)
```

`--orbit-return` 에서는 기존 `DOCK_HOLDING` (10 s) 대신 `ORBIT_HOLDING` 이 그 역할을 합니다.

| 상태 | 동작 |
|---|---|
| `ORBIT_TARGET_ACQUIRE` | Client 기준점에서 가장 가까운 궤도 점(정확한 최단점)부터 후보를 만들고, **EE 접촉점** 이동량 `(목표 − Client 점)` 이 팔 도달 범위 `[arm_reach_min_m, arm_reach_max_m]`(베이스 기준) 안인 첫 후보를 고릅니다. 궤도 파라미터와 목표점을 로그로 남기고 GUI 에서는 뷰포트를 옮깁니다 |
| `ORBIT_TRANSFER` | MEP–Client FixedJoint 를 유지한 채 기존 `track_probe` (DLS IK 관절 위치 목표, 가속 제한 `docking.accel_mps2`, 연속 감속 `docking.decel_gain_hz`) 로 이동. 속도 `transfer_speed_mps`. 위성 속도 피드포워드는 끔(위성이 팔에 붙어 있으므로). **`root pose` 덮어쓰기와 FixedJoint 재생성은 없습니다** |
| `ORBIT_ARRIVAL_CHECK` | 아래 PASS 기준을 `arrival_hold_s` 동안 동시에 만족. 팁이 정지한 뒤 오차가 남아 있으면 가장 가까운 궤도 점으로 보정 이동 |
| `ORBIT_HOLDING` | `hold_duration_s` (10 s) 동안 유지하며 오차·드리프트·Client 각속도를 기록 |

**목표를 MEP 가 아니라 Client 점 기준으로 역산하는 방법**: Client 는 MEP 에 FixedJoint 로 붙고, MEP 는 EE 에 붙어 있으므로 셋은 하나의 강체입니다.
자세를 유지하는 한 Client 점의 이동은 제어점(프로브 팁) 이동과 같아서 팁 목표 = 현재 팁 pose + `(목표 − Client 점)` (병진만). 이 목표는 시작 시점에 한 번 래치합니다 —
매 스텝 측정값에서 다시 만들면 팔의 진동을 따라가기 때문입니다 (도킹 정렬 단계와 같은 이유).

**충돌·관절 한계**: 팔 도달성은 위 구면 필터, 관절 한계는 이동 중 감시(`joint_limit_margin_rad` 안으로 들어가면 안전 정지)입니다.
링크 충돌 여유와 접촉력 기반 충돌 감지(MEP 부착 중에는 기존 접촉 검사가 동작하지 않음)는 계산하지 않습니다 `[NEEDS_ISAAC_VALIDATION]`.

## 7. 센서 간섭 방지

궤도선은 지속적인 USD 지오메트리라서 카메라에 그대로 찍힙니다. 깊이 카메라(`cam_probe`)는 첫 충돌을 기록하므로(기존 도킹 표시 디스크에서 겪은 문제)
선이 주광선을 지나면 도킹 거리 측정이 어긋나고, 손목 카메라 앞을 지나면 AprilTag 를 가립니다.
그래서 시작 시 궤도와 다음 두 선분의 최소 거리가 `sensor_keepout_m` (0.4 m) 이상인지 검사합니다. 위반하면 실행 전에 `ValueError` 로 멈춥니다.

- 도킹축: pre-dock 진입 앞 2 m … 노즐 뒤 3 m (depth 주광선)
- 손목 카메라 통로: MEP 부착면에서 법선 방향으로 관측 거리 + 1 m

기본 설정에서는 궤도 평면이 Client 의 0.5 m 아래(수직)라서 수평인 도킹축과의 거리가 최소 0.5 m 입니다 (측정: 로그의 `docking_axis 0.50 m`).
손목 카메라 통로와는 약 10 m 떨어집니다.

## 8. 설정 키 (`config/vision_capture.yaml`)

### `orbit_reference:`

| 키 | 기본 | 단위 | 의미 |
|---|---|---|---|
| `enabled` | false | | 켜면 궤도선 + 표류 + 검사. CLI 가 켬 |
| `observe_only` | false | | `--orbit-only`: 장면 검사만 |
| `observe_only_duration_s` | 20 | s | 위 검사 시간 |
| `shape` | circle | | `circle` / `ellipse` |
| `center_w` | null | m | null = Client 시작점에서 유도. `[x, y, z]` = 고정 |
| `normal_w` | [0,0,1] | | 궤도 평면 법선 (정규화됨) |
| `radius_m` | 50.0 | m | 원 (축소 그림, §1) |
| `semi_major_m`, `semi_minor_m` | 50.0, 40.0 | m | 타원 (e1 방향이 장축) |
| `arc_span_deg` | 80 | deg | 그리는/쓰는 호의 각도 길이 (nominal 각 중심). 360 = 링 전체 |
| `view_elevation_deg` | 12 | deg | GUI 시작 시점: 링 축 위의 눈에서 링이 수평선 아래 이 각도로 보임 (limb 은 18.3°) |
| `segments` | 128 | | ≥ 128 |
| `line_width_m` | 0.25 | m | 튜브 지름 |
| `color_rgb` | [1,0,0] | | |
| `prim_path` | /World/reference_orbit | | |
| `nominal_angle_deg` | 0 | deg | Client 가 있던 호 위 명목 각 (e1 = +X 기준). 호의 중심. null = 자동 |
| `client_reference` | sat_dock_point | | `sat_dock_point` / `body_origin` |
| `client_initial_offset_m` | 0.5 | m | 시작 시 궤도에서 벗어나는 거리 |
| `client_offset_direction` | [0,0,1] | | `radial_outward` / `radial_inward` / `[x,y,z]` (기본: 위쪽 = 고도 오차) |
| `initial_offset_tolerance_m` | 0.005 | m | 시작 오프셋 검증 허용치 |
| `target_mode` | nearest_feasible_point | | |
| `arrival_tolerance_m` | 0.05 | m | 궤도 도달 판정 |
| `hold_duration_s` | 10 | s | |
| `arm_reach_min_m`, `arm_reach_max_m` | 1.0, 8.0 | m | EE 접촉점의 베이스 기준 도달 범위 (Canadarm3 ≈ 8.5 m) |
| `max_target_candidates` | 64 | | |
| `transfer_speed_mps` | 0.05 | m/s | |
| `transfer_timeout_s`, `arrival_hold_s`, `arrival_timeout_s`, `max_retargets` | 400, 0.5, 60, 3 | s | |
| `joint_limit_margin_rad` | 0.02 | rad | |
| `max_relative_drift_mm`, `_deg` | 5.0, 0.5 | mm, deg | 기존 DOCK6 홀딩 기준 |
| `max_client_angular_velocity_rad_s` | 0.005 | rad/s | |
| `sensor_keepout_m` | 0.4 | m | §7 |
| `max_step_jump_m` | 0.005 | m | 제어 1스텝당 Client 점 이동 한계(순간이동 감지) |

### `drift:`

| 키 | 기본 | 의미 |
|---|---|---|
| `mep_velocity_mps`, `mep_direction_w` | 0.01, [0,−1,0] | MEP 표류 (호의 접선) |
| `client_velocity_mps`, `client_direction_w` | 0.01, [0,−1,0] | Client 표류 (기본은 MEP 와 같은 벡터) |
| `angular_velocity_rad_s` | [0,0,0] | 0 이어야 함 |
| `velocity_tolerance_mps`, `angular_tolerance_rad_s` | 0.002, 0.002 | 자유유영 감시 허용치 |

## 9. 실행

```bash
cd /home/rokey/isaac_space

# 장면 검사만 (궤도선, 무회전 등속, 접선 유영, 초기 오프셋) — 시뮬레이션 20 s. 저장 영상: <tag>_orbit_view.png
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag orbit_only --headless --orbit-only

# 전체 파이프라인: 흡착(AprilTag) 포획 → 도킹 → Client orbit return   (GUI)
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag orbit_return --dock --orbit-return

# 빠른 반복: 포획 생략 → 도킹 → orbit return
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag orbit_return_quick --headless --dock_only --orbit-return

# 순수 오프셋만 (Client 는 표류하지 않음)
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag orbit_return_quick --headless --dock_only --orbit-return --set drift.client_velocity_mps=0.0

# 오프라인 테스트 (시뮬레이터 불필요)
cd project && python3 -m pytest tests/test_orbit_return.py -q
```

`--orbit-return` 은 `--dock` 을 포함합니다. `--orbit-return` 과 `--orbit-only` 는 함께 못 씁니다. 두 옵션 모두 `--scenario dynamic` 에서만 동작합니다.
`--dock` / `--dock_only` (기존) 는 이 옵션 없이 이전과 같게 동작합니다.
**Isaac Sim 은 한 번에 하나만 실행하세요.** 두 개를 동시에 띄우면 (실측) 먼저 뜬 쪽이 Carb 뮤텍스 assertion 으로 죽었습니다.

출력: `project/logs/vision_capture/<tag>_{result.json,orbit.csv,docking.csv,metrics.csv,probe_rgb/}`. `orbit.csv` 에는 Client 점, 궤도 오차, 목표점, 두 물체의 v/w, MEP–Client 드리프트, 두 FixedJoint 유효, 관절 여유가 10 Hz 로 기록됩니다.

### ROS 2 토픽 확인 (포획 → 도킹 → 궤도 복귀 전체)

터미널 1 — 시뮬레이터 (`--ros` 추가):

```bash
cd /home/rokey/isaac_space
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag pipeline_ros --dock --orbit-return --ros
```

터미널 2 — 모니터 (시스템 Python + ROS 2 Jazzy). 상태 전이, `captured`/`docked`, 궤도 오차를 출력하고 끝에 PASS/FAIL 표를 냅니다:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp        # 시뮬레이터와 같은 RMW
python3 /home/rokey/isaac_space/project/scripts/mrv_ros_monitor.py
```

직접 볼 때:

```bash
ros2 topic echo /mrv/state                 # 상태 머신 (래치)
ros2 topic echo /mrv/captured              # EE <-> MEP FixedJoint
ros2 topic echo /mrv/docked                # MEP <-> Client FixedJoint
ros2 topic echo /mrv/orbit/client_error    # Client 점 -> 궤도 거리 [m]
ros2 topic echo /mrv/client/pose           # Client 점 (SAT_DOCK_POINT)
ros2 topic echo /mrv/status                # JSON
ros2 topic hz /mrv/ee/pose
```

시작 명령을 ROS 로 주려면 (`--ros_wait_start`, 예측이 준비되면 팔이 대기):

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --dock --orbit-return --ros_wait_start
python3 /home/rokey/isaac_space/project/scripts/mrv_ros_monitor.py --send-start
# 또는: ros2 topic pub --once /mrv/cmd/start std_msgs/msg/Empty "{}"
```

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/mrv/state` | String (latched) | 상태 머신 (`ORBIT_*` 포함) |
| `/mrv/captured` | Bool (latched) | 포획 FixedJoint |
| `/mrv/docked` | Bool (latched) | 도킹 FixedJoint |
| `/mrv/client/pose` | PoseStamped | Client 점 (SAT_DOCK_POINT), 도킹 단계 |
| `/mrv/orbit/client_error` | Float64 | Client 점 → 호 최단거리 [m], 궤도 시나리오 |
| `/mrv/status` | String | JSON (`docked`, `orbit_error_m` 포함) |
| 기존 | | `ee/pose`, `estimate/*`, `gt/*`(평가 전용), `cam_wrist/*`, `/tf`, `cmd/{start,abort,capture_enable}` — [`mrv_ros_interface.md`](mrv_ros_interface.md) |

## 10. PASS 기준 (`<tag>_result.json` 의 checks, exit code 0 = 전부 통과)

| Check | 기준 |
|---|---|
| `[ORBIT0] Red reference orbit prim, visual only` | prim 존재, 보임, 빨강, 정점 ≥ segments, 물리 스키마 없음 |
| `[ORBIT0] Client starts off the orbit by the configured offset` | 최단거리 = 오프셋 ± 5 mm, 궤도 위가 아님 |
| `[ORBIT0] Orbit clear of the depth ray and the AprilTag corridor` | 두 선분과 ≥ `sensor_keepout_m` |
| `[ORBIT0] MEP and Client: no rotation, commanded constant velocity` | t = 0 에서 `|w|` ≈ 0, `|v − v_cmd|` ≤ 허용치 |
| `[ORBIT0] Linear and angular damping are 0 …` | USD damping 값 0 |
| `[ORBIT0] Free flight held …` | 포획/도킹 전 전 구간에서 위 허용치 유지 |
| `[DOCK…]` | 기존 도킹 checks 그대로 (`--orbit-return` 에서는 DOCK6/7 홀딩 대신 아래 ORBIT4) |
| `[ORBIT1] Orbit target acquired` | 도달 가능한 궤도 점 존재 |
| `[ORBIT2] Transfer with both FixedJoints kept, no jump` | 이동 중 두 FixedJoint 유지, 한 스텝 Client 점 이동 ≤ 5 mm |
| `[ORBIT3] Client point on the orbit` | 최단거리 ≤ 50 mm, FixedJoint 유효, MEP–Client 드리프트 ≤ 5 mm / 0.5°, Client `|w|` ≤ 0.005 rad/s, 비정상 물리 없음 |
| `[ORBIT4] Held on the orbit` | 10 s 동안 위 기준 유지 (오차 최대값 기준) |

실패 시 로그에 `[ORBIT] failure context: …` 한 줄이 남습니다 (직전 상태, Client 점, 궤도 오차, 목표점, 두 joint 유효 여부, 두 물체 각속도, Client 속도).

## 11. Isaac Sim GUI 에서 확인하는 방법

GUI 는 `--headless` 없이 실행합니다 (시작 시 뷰포트가 궤도 전체를 보도록 맞춰집니다).

1. **빨간 궤도선**: 시작 화면에서 굵은 빨간 원. Stage 창에서 `/World/reference_orbit` 를 고르면 선택 윤곽이 뜹니다.
2. **초기 오프셋**: 위성(Client) 노즐 근처에서 빨간 선까지 약 0.5 m. 콘솔의 `[ORBIT] Client start: distance to the orbit 500.0 mm …` 와 비교하세요.
3. **도킹**: 기존과 같습니다 (노즐 링이 초록으로 바뀜).
4. **궤도 도달**: `ORBIT_TARGET_ACQUIRE` 에서 뷰포트가 목표 쪽으로 이동하고, 흰 점(궤도 목표)과 청록 점(Client 점)을 잇는 선이 보입니다. 선이 점점 짧아져 0 에 가까워지면 도달입니다.
   콘솔에 2 초마다 `[ORBIT] … orbit error … mm` 가 찍힙니다.

## 12. 알려진 한계 / 남은 검증 `[NEEDS_ISAAC_VALIDATION]`

- 아래 "검증 상태" 에 적힌 실행 이외는 실행하지 않았습니다.
- 도킹 후 3 t MEP + Client 를 팔로 옮기는 동역학(스윙, 정착 시간)은 실측 전입니다. `transfer_speed_mps` 와 `arrival_tolerance_m` 는 실행으로 조정해야 합니다.
- 팔 도달성은 구면 근사이고 링크 충돌 여유는 계산하지 않습니다.
- Client 표류 (`client_velocity_mps > 0`) 로 도킹하는 실행은 기존에 검증된 적이 없습니다. (이번 작업에서 위성 초기 속도가 씬에 적용되지 않던 문제를 고쳤습니다: `vision_task.py`.)
