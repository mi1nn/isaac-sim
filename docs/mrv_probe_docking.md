# MEP Ares1 Probe → Client Satellite 자동 도킹 (Phase 1: 무회전 자유유영)

Phase 1 포획 문서: [`mrv_phase1_vision_capture.md`](mrv_phase1_vision_capture.md) · 6-DoF: [`mrv_six_dof_capture.md`](mrv_six_dof_capture.md) · ROS: [`mrv_ros_interface.md`](mrv_ros_interface.md)

> 상태: **2026-09-20 headless 실행 검증 — 도킹 성공.**
> `--dock_only` (포획 생략) 로 전송 → 정렬 → 축 접근 → 삽입 → FixedJoint → 10 s 유지까지 통과했습니다.
> 아직 실행하지 못한 항목은 맨 아래 "남은 검증"에 있습니다.

## 무엇을 붙였나

기존 코드를 새로 쓰지 않고 이미 있던 것을 그대로 씁니다.

| 재사용한 기존 자산 | 위치 |
|---|---|
| PROBE_DOCK_POINT (프로브 팁 + 축), SAT_DOCK_POINT, 노즐 내경 프로파일 | `docking.py` `DockingGeometry` (USD 실측) |
| 도킹 FixedJoint | `docking.py` `DockingManager.dock()` |
| 중공 노즐 콜라이더 + 백플레이트 | `docking.py` `build_thruster_collider` |
| DLS IK, 도구 프레임 전환 | `docking_demo.py` `ArmKinematics.set_tool()` |
| 포획 FixedJoint | `capture.py` `CaptureManager` |

새로 만든 것은 순수 numpy 모듈 `probe_dock.py` (상대 pose / 깊이 / 조건, 오프라인 테스트 가능) 와
`vision_capture_demo.py` 의 상태 추가, `vision_task.py` 의 프로브 RGB-D 카메라입니다.

## 좌표 규약 — 핵심

오차는 **절대 world XYZ 로 재지 않습니다.** 전부 위성 도킹 프레임 D 에서 봅니다.

```
T_W_P  프로브 : 원점 = 팁, +Z = 삽입 방향
T_W_D  도킹부 : 원점 = SAT_DOCK_POINT (노즐 출구 안쪽 dock_depth=0.8 m), +Z = 노즐 안쪽
e = T_W_D⁻¹ · T_W_P     →  e_z < 0 이면 아직 바깥. 남은 삽입 거리 = -e_z
```

Euler 각은 누적하지 않습니다. 게이트는 회전행렬에서 뽑은 `axis_deg`(축 기울기) / `roll_deg`(축 둘레) 를 쓰고,
`roll/pitch/yaw` 는 사람이 보는 로그 표시용입니다.

이 단계의 상대 pose 는 **시뮬레이션 바디 pose 에서 계산**합니다. 즉 transform(=Ground Truth) 기반이며
**비전 추정이 아닙니다.** 위성을 보는 카메라는 없습니다. 깊이 카메라는 독립적인 교차검증으로만 씁니다.

## 상태 머신 (기존 머신에 이어붙임)

```
(포획 SUCCESS 또는 --dock_only)
 → DOCK_TARGET_ACQUIRE → PRE_DOCK_APPROACH → XY_ALIGN → ORIENTATION_ALIGN
 → ALIGNMENT_CHECK → Z_APPROACH ⇄ (정렬 이탈 시 XY_ALIGN 복귀) → FINAL_INSERTION
 → DOCK_READY → DOCKED → DOCK_HOLDING → SUCCESS      (실패: DOCK_FAILED)
```

- `XY_ALIGN` / `ORIENTATION_ALIGN` / `ALIGNMENT_CHECK` 는 축 거리를 **고정**한 채 횡/자세만 줄입니다.
  실측 축 거리를 따라가면 목표가 팔의 진동을 따라가며 진동을 계속 살려둡니다(실측으로 확인).
- `Z_APPROACH` 는 매 스텝 두 바디 pose 를 다시 읽는 closed loop 입니다. 정렬이 복도(`approach_abort_*`)를
  벗어나면 전진을 멈추고 정렬 단계로 되돌아갑니다.
- 깊이가 유효하지 않으면 **전진하지 않습니다.** 무조건 전진하는 fallback 은 없습니다.

## RGB-D 카메라 (`cam_probe`)

- 프로브 축 위, 팁보다 `offset_from_tip_m`(20 mm) **앞**. 로드가 주광선을 가리지 않습니다.
- 위치/자세는 시작 시 USD 에서 실측한 `probe_dock` 에서 계산합니다(하드코딩 없음).
- RGB 는 운용자 확인용으로 `<tag>_probe_rgb/` 에 저장, Depth 는 `distance_to_image_plane`.
- **USD 카메라는 자기 -Z 를 봅니다.** prim 변환을 직접 쓰므로 `convention="ros"` 변환을 거치지 않아
  여기서 USD 규약을 직접 적용합니다. (+Z 로 두었을 때 카메라가 뒤를 봐서 깊이가 전부 비었습니다.)
- **도킹 표시 디스크를 숨깁니다.** 노즐 출구를 가로지르는 반투명 디스크인데, 깊이 영상은 불투명도와
  무관하게 첫 충돌을 기록하므로 이 디스크만 계속 측정됐습니다(실측 원시 0.93 m = 출구 평면).

### 깊이 → 도킹축 거리

```
남은 삽입 거리 = depth_raw - camera_to_tip - surface_offset
```

`surface_offset` 은 도킹축에 정렬된 pre-dock 자세에서 **한 번 보정**합니다(`auto_calibrate`).
`build_thruster_collider` 의 백플레이트는 충돌 전용이라 렌더링되지 않고, 광선은 그 뒤 추력기 메시에
닿기 때문입니다. 측정값 **1.527 m** (설정 기본값 0.1 m, 백플레이트 0.1 m) 가 실제 면의 위치입니다.
보정값이 `[-0.5, 3.0] m` 밖이면 거부합니다(광선이 노즐을 벗어난 경우).

## 실행

```bash
cd ~/space_robotics_bench

# 도킹만 빠르게 (포획 생략, MEP 를 명목 파지 자세에 부착하고 시작)
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag dock --headless --dock_only

# 포획(AprilTag) → 도킹 전체 체인
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag full --dock

# 위성도 표류시키기
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag drift --headless --dock_only \
  --set docking.satellite_velocity_mps=0.004 --set "docking.satellite_drift_direction=[0.0,-1.0,0.0]"

# 오프라인 수학 테스트 (시뮬레이터 불필요)
cd project && ~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py tests/test_vision_math.py -q
```

출력: `project/logs/vision_capture/<tag>_{result.json,docking.csv,probe_rgb/}`.

## 로그 (`<tag>_docking.csv`)

`run_id, timestamp, state`, probe tip / sat dock 의 pose(위치 + 쿼터니언 wxyz),
`relative_x/y/z`(도킹 프레임), `roll/pitch/yaw_error_deg`, `orientation_error_deg`,
`axis_error_deg`, `roll_about_axis_deg`, `lateral_error_m`,
`geometry_distance_m`, `depth_distance_m`, `depth_raw_m`, `depth_pixels`,
`insertion_depth_m`, `wall_clearance_m`, `relative_vx/vy/vz`, `relative_speed_mps`,
`commanded_speed_mps`, `alignment_valid`, `depth_valid`, `dock_ready`, `dock_success`.

AI 는 이번 단계에서 구현하지 않았습니다. 데이터 수집만 준비했습니다.

## 실행 검증 결과 (2026-09-20, headless, `--dock_only`)

**도킹 성공.** 최종 상태 SUCCESS.

| 항목 | 측정값 |
|---|---|
| 축 오차 (axial) | -9.98 mm (한계 10 mm) |
| 횡 오차 (radial) | 6.53 mm (한계 10 mm) |
| 축 기울기 | 0.025° (한계 0.5°) |
| 축 둘레 roll | 0.013° (한계 1.0°) |
| 상대 속도 | 4.6 mm/s (한계 20 mm/s) |
| 삽입 깊이 | 노즐 안쪽 0.790 m |
| 프로브-벽 최소 여유 | 381 mm |
| 도킹 후 10 s 유지 | MEP-위성 상대 드리프트 0.000 mm / 0.0000° |
| 관통 | 없음 (최소 여유 381 mm) |
| 재정렬 복귀 횟수 | 3 (closed loop 이 설계대로 동작) |

**깊이 vs 기하 거리** (보정 이후 2649 샘플): 평균 차이 4 mm, 상관 0.9982,
시작 1.800 m ↔ 1.800 m → 종료 0.010 m ↔ 0.010 m. 두 값이 함께 감소합니다.
2649 프레임 중 33 프레임(1.2 %)이 일치 밴드(150 mm)를 벗어났고, 그 동안 프로브는 설계대로
전진하지 않고 정지했습니다.

소요: 시뮬 424 s (벽시계 약 31 분). 최종 확인 실행: **20/20 checks PASS, 최종 상태 SUCCESS**.

### 실행으로 찾아 고친 문제

| 증상 | 원인 | 조치 |
|---|---|---|
| 프로브 팁이 100 mm/s 지령에 20 mm/s 로만 기어감 | 관절 속도 목표를 0 으로 줘서 PD 드라이브가 자기 위치 목표에 제동 | 기준 궤적의 속도를 피드포워드 |
| 깊이가 전부 빈 값 | USD 카메라는 -Z 를 보는데 +Z 로 배치 → 뒤를 봄 | prim 에 USD 규약 적용 |
| 깊이가 항상 출구 거리(0.93 m) | 반투명 도킹 표시 디스크가 광선 차단 | 프로브 카메라 사용 시 디스크 숨김 |
| 횡오차 ±150 mm 진동이 안 줄어듦 | 지연 게이트의 on/off 가 진동을 정류해 공진을 펌핑 | 기준 궤적을 연속 전진으로 변경 |
| 정렬이 자기 진동을 따라감 | 목표가 실측 축거리를 따라감 | 정렬 중 축 거리 고정 |
| Z 접근 시작 1 s 만에 횡오차 20 mm | 속도를 0→0.1 m/s 계단 입력 → 3 t 페이로드를 옆으로 참 | 가속 제한 `accel_mps2` |
| 목표 도달 후 진동이 안 잦아듦 | `ik_joint_target` 이 매 스텝 **측정 관절값**에서 목표를 만들어 진동을 따라감(복원력 0) | 기준이 멈추면 관절 목표 래치(`hold`) |
| 시도했다가 되돌린 것 | 관절 속도 목표에 감쇠 외란(gain 1.0) → 0.1 s 만에 불안정 / 목표 누적 적분 → 강성 40000 에서 과도 토크 | 둘 다 기본값 off, 설정으로 남김 |

## 속도 최적화

| 바꾼 것 | 전 → 후 | 근거 |
|---|---|---|
| `camera.supersample` | 2 → 1 | 10 s 구간 116 s → 39 s (약 3배). 별도 확인 |
| 도킹 중 wrist 카메라 비전 처리 | 매 프레임 → 생략 | 도킹은 AprilTag 를 쓰지 않음 |
| `--dock_only` | 포획 30 s 포함 → 생략 | 도킹 반복 검증용 |
| `camera.render_on_vision_only` (headless) | off (기본) | 10 s 구간 116 s → 35 s |

안정성을 위해 **유지한 제한**: 전송 0.10 m/s(기존 데모의 carry 속도), 가속 0.005 m/s²,
정렬 0.008 m/s, 삽입 0.01 m/s, 단계 제한 240 s. 전송을 0.25 m/s 로 올렸더니 ±150 mm 진동이 남아
되돌렸습니다.

## 남은 검증 `[NEEDS_ISAAC_VALIDATION]`

- **GUI 실행** — headless 로만 확인했습니다.
- **포획 → 도킹 전체 체인** (`--dock`) — `--dock_only` 로만 확인했습니다. 포획 지점이 명목 자세와
  다르고 MEP 가 표류한 상태로 넘어가므로 전송 거리와 자세가 달라집니다.
- **위성 표류** (`satellite_velocity_mps > 0`) — 0 으로만 확인했습니다. 코드에는 위성 속도
  피드포워드가 있지만 실행으로 확인하지 않았습니다.
- **고의 X/Y 오프셋 주입 시 전진 정지** (TEST 7 후반부) — 자연 발생한 재정렬 3 회는 확인했지만
  고의 주입 시험은 하지 않았습니다.
- **접촉력 기반 충돌 감지** — MEP 부착 중에는 기존 접촉 검사가 동작하지 않습니다. 현재는 기하
  여유(`wall_clearance`)로만 판정합니다.
- **되돌린 두 제어 옵션**(`swing_damping`, 목표 적분)의 안정적인 이득 값.
