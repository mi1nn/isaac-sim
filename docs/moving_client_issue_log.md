# 이동 Client 도킹 이슈 트래킹

위성(Client)에 속도를 주는 시나리오(`--moving_dock`)를 구현·실행하면서 생긴 이슈를 기록합니다. 이슈마다 **증상 → 원인 → 수정 → 결과** 순서이고, 결과는 실행 로그로 확인된 것만 적었습니다. 확인하지 못한 것은 "미검증"으로 표시합니다.

관련 문서: [`moving_client_docking.md`](moving_client_docking.md) (구현 설명·설정)
로그 위치: `project/logs/vision_capture/<tag>_{docking,moving,metrics}.csv`, `<tag>_result.json`

---

## 요약

| # | 이슈 | 분류 | 상태 |
|---|---|---|---|
| 1 | Client(위성)가 움직이지 않음 | 분석 | 원인 확인 (고정이 아니라 속도 0) |
| 2 | 도킹 중 "target jumped 1662 mm" → DOCK_FAILED | 버그 | 수정·확인 |
| 3 | rendezvous 위치 게이트가 너무 느슨함 (196 mm 오차로 통과) | 설정 | 수정·확인 |
| 4 | Z_APPROACH에서 전진이 멈추고 재정렬 반복 | 버그 (기존 코드) | 수정, 전체 검증 전 |
| 5 | 도킹 시작 전 대기가 너무 김 (sim 32 s, GUI 약 4분) | 설계 | 수정·확인 |
| 6 | 느린 이동이 화면에서 안 보임 | 시각화 | 기능 추가 |
| 7 | pre-dock 흔들림 대기 + 이송이 느림 | 설계 (기존 코드) | 강제 삽입 추가·부분 확인 |
| 8 | 강제 삽입 기준이 사용자 설정보다 좁았음 | 설정 충돌 | 수정 |
| 9 | 도킹 직후 `AttributeError: update_moving_visuals` 로 중단 | 버그 (제 실수) | 수정, 실행 검증 전 |
| 10 | "external wrench frame has been changed" 경고 | 경고 | 수정, 실행 검증 전 |
| 11 | 위성이 흡착 이후에야 움직이기 시작함 | 요구 변경 | 수정, 실행 검증 전 |
| 12 | 도킹 후 STOPPING에서 속도가 0이 안 되는 것처럼 보임 / 불필요한 대기 | 설계 | 흐름 변경, 실행 검증 전 |

---

## 실행 기록

| tag | 명령 (공통: `vision_capture.py --scenario dynamic`) | 결과 |
|---|---|---|
| `moving_smoke` | `--headless --dock_only --moving_dock` | 18/25 PASS, sim 102 s에 DOCK_FAILED (이슈 #2) |
| `moving_smoke2` | 동일 (#2·#3 수정 후) | Z_APPROACH에서 sim 153~225 s 정체 → 사용자 요청으로 중단 (이슈 #4) |
| `static_regress`, `moving_nodepth`, `static_calib`, `moving_calib` | 진단·회귀 런 | 사용자 요청으로 초반에 중단, 결과 없음 |
| `moving_vis` (GUI, 사용자) | `--dock_only --moving_dock` | RENDEZVOUS에서 멈춘 것처럼 보임 (이슈 #5) |
| `moving_fast` (GUI, 사용자) | 빠른 데모 기본값 | 도킹 시작 sim 7.2 s |
| `trail_smoke`, `concurrent_smoke` | `--headless --dock_only --moving_dock`, 약 5분 | 궤적 정상, 동시 진행 모드에서 도킹 sim 0 s 시작 |
| `forced_smoke` | 동일, 약 10분 | 이송 중간(sim 20 s)까지만 진행, 강제 삽입 구간 미도달 |
| `moving_now` (GUI, 사용자) | `--dock_only --moving_dock` (강제 삽입) | sim 97 s DOCK_READY 도달 → 도킹 직후 크래시 (이슈 #9) |
| `full_moving` (GUI, 사용자) | 흡착부터 전체 (`--moving_dock`, six_dof) | 흡착 → 이송 → sim 156 s Z_APPROACH → 180 s DOCK_READY → **sim 184 s 도킹 성공** (첫 전체 파이프라인 도킹) → STABILIZING → STOPPING (19.5 → 1.15 mm/s, 마지막 기록 sim 197.5 s) |

---

## 이슈 상세

### #1 Client(위성)가 움직이지 않음

- **증상**: 기존 도킹에서 위성이 항상 정지해 있음.
- **원인**: 고정된 것이 아니었습니다. USD와 런타임 stage를 모두 확인했습니다(PXR traversal).
  - `/satellite/GOES_R`: RigidBodyAPI 적용, `kinematic = False`, `rigidBodyEnabled = True`
  - 위성에 걸린 joint 0개, scene gravity 0
  - 초기 속도 `docking.satellite_velocity_mps: 0.0`
  - 질량은 약 1,792,594 kg (density 1000 × convex hull 부피)
- **수정**: 해제할 constraint가 없어서 제거한 것은 없습니다. 추력(F = m·a)으로 속도를 주는 방출 단계를 추가했습니다(`client_release_thrust`).
- **결과**: 방출 5.2 s 만에 0.0195 m/s 도달. 5 s 유영 동안 속도 변화 0.000 mm/s, 위치 오차 0.25 mm (`moving_smoke`, `moving_smoke2`).

### #2 "docking target jumped 1662 mm in one step" → DOCK_FAILED

- **증상**: `moving_smoke`에서 ALIGNMENT_CHECK → Z_APPROACH로 넘어가는 순간(sim 102.6 s) DOCK_FAILED.
- **원인**: `docking_tracking_valid()`가 직전 호출 때 저장한 목표 위치와 지금 위치를 비교해 0.25 m 이상 차이 나면 "순간 점프"로 판정합니다. 그런데 이 함수는 일부 상태에서만 호출되어, 직전 값이 **sim 17 s**(DOCK_TARGET_ACQUIRE) 시점이었습니다. 그 사이 85 s 동안 위성이 0.0195 m/s × 85 s ≈ **1.66 m** 이동했습니다. 정지 위성에서는 드러나지 않던 문제입니다.
- **수정**: 기준을 `이전 위치 + 위성 속도 × 경과 시간`으로 변경(`vision_capture_demo.py`, `docking_tracking_valid`). 정지 위성은 속도가 0이라 동작이 기존과 같습니다.
- **결과**: `moving_smoke2`에서 같은 지점(Z_APPROACH 진입)을 오류 없이 통과.

### #3 rendezvous 위치 게이트가 너무 느슨함

- **증상**: `moving_smoke`에서 MRV 위치 오차 195.8 mm인 상태로 rendezvous를 통과했고, 상대속도(4.4 mm/s)는 오히려 올라가는 중이었음.
- **원인**: `rendezvous.max_relative_position_m`을 요청서 예시값 0.5 m로 두었습니다. 도킹은 MRV가 정확히 공칭 위치에 있는 상태로 검증돼 있어, 0.5 m는 의미 없는 값이었습니다.
- **수정**: `max_relative_position_m` 0.5 → **0.05 m**, `dock_max_relative_position_m` 0.5 → **0.1 m**.
- **결과**: `moving_smoke2`에서 위치 오차 28.7 mm로 통과.

### #4 Z_APPROACH에서 전진이 멈추고 재정렬 반복

- **증상**: `moving_smoke2`에서 남은 삽입 거리가 **1.494 m로 고정**(sim 153~225 s). 로그에 `holding: depth 1.832 m disagrees with geometry 1.511 m by 320 mm`와 `approach stopped, re-aligning: lateral 80.01 mm > 80 mm`가 반복됨.
- **처음 가설 (틀림)**: MRV가 kinematic으로 움직이며 흔들림을 키웠다. MRV 위치 오차가 0.0 mm로 정상이어서 기각했습니다.
- **원인 (로그로 확인)**: depth 보정(캘리브레이션)이 **축에서 41 mm 벗어난 위치**에서 한 번 잡혔습니다. 정렬 게이트 50 mm라 통과됐습니다. depth 광선이 원뿔형 노즐 내벽에 맞기 때문에, 축에서 벗어난 정도에 따라 다른 면에 닿습니다.

  | 횡오차 | 이동 런 \|depth − 기하 거리\| | 예전 정지 위성 런 (`dock_docking.csv`) |
  |---|---|---|
  | 0~40 mm (축 위) | **항상 320 mm** | **항상 320 mm** |
  | 40~90 mm (축에서 벗어남) | 평균 65 mm | 평균 58 mm |

  축에 잘 맞으면 depth가 어긋나 전진이 보류되고, 축에서 벗어나 있으면 전진하다가 80 mm를 넘어 재정렬되는 반복이었습니다. **정지 위성 로그에도 같은 패턴이 있어, 위성 이동과 무관한 기존 문제**입니다.
- **수정**: `docking.depth_calibration_max_lateral_m` 0.015, `depth_calibration_max_axis_deg` 1.0, `depth_calibration_samples` 10을 추가했습니다. 축 위에서만 샘플을 모아 중앙값으로 보정하고, 보정 전에는 삽입을 시작하지 않습니다. 오프라인 테스트를 추가했습니다(`test_probe_dock.py`).
- **결과**: 전체 도킹 실행으로는 **미검증**입니다. 검증 런(`static_calib`, `moving_calib`)은 보정 단계 전에 중단됐습니다. 이동 모드에서는 이후 강제 삽입(#7)이 depth를 참고용으로만 씁니다.

### #5 도킹 시작 전 대기가 너무 김

- **증상**: `moving_vis`(GUI)에서 실행 후 약 2분 뒤 시점이 바뀌고, 다시 2~3분 기다려야 프로브가 움직임. 사용자에게는 "도킹이 진행되지 않는" 것으로 보였음.
- **원인**: 요청서의 상태 목록과 Test 1·2 조건을 **매 실행마다 차례로 기다리는 단계**로 구현했습니다. GUI는 sim 1 s당 실제 약 8 s가 걸려 대기가 크게 늘어났습니다.

  | 대기 (sim) | 출처 |
  |---|---|
  | 방출 램프 5.2 s | 요청서 29번 (급격한 속도 변경 금지) + 가속 0.004 m/s² 선택 |
  | 순항 확인 5 s | 요청서 Test 1 (5~10 s 유영 확인) |
  | 추격 3.8 s + 따라잡기 15.9 s | 요청서 Test 2 (MRV 0.015 vs Client 0.020) → 약 200 mm 간격 발생 |
  | 조건 유지 1 s씩 | 요청서 10번 (`stable_duration_sec`) |
  | 도킹 시작 hold 1 s | 기존 `docking.settle_time_s` |

- **수정**:
  1. 순항 확인·추격을 기본으로 생략(`cruise_check_s: 0`, `mrv_initial_velocity_mps: []`)하고, MRV가 방출 순간부터 따라가게 함(`mrv_follow_during_release: true`).
  2. `rendezvous.concurrent_docking: true`: 방출과 MRV 위치 유지가 **도킹 이송과 동시에** 진행되고, 속도·위치 판정은 **삽입 직전**으로 옮김(`insertion_gates`).
  3. `docking.settle_time_s` 1.0 → 0.0.
- **결과**:
  - 1 적용 후 (`moving_fast`): 도킹 시작 sim 32 s → **7.2 s**
  - 2·3 적용 후 (`concurrent_smoke`): 도킹 시작 **sim 0 s**. 이송 중 5.2 s에 방출 완료, MRV 위치 오차 최대 4.5 mm → 0.0 mm.

### #6 느린 이동이 화면에서 안 보임

- **증상**: 0.02 m/s(초당 2 cm)라 위성과 MEP가 움직이는지 눈으로 확인할 수 없음.
- **수정**:
  - 이동 궤적: 출발점 고정 십자 + 1 s 간격 점과 선. `logging.motion_trail*` 설정.
  - "Docking monitor" 창(GUI): 속도, 상대속도, 이동 거리, 결합 상태, 프로브 축–노즐 축 각도·횡오차·roll과 기준값.
  - 이후 정지 위성 도킹과 흡착 단계에서도 창이 뜨도록 확장.
- **결과**: 궤적은 headless에서 오류 없이 그려지는 것을 확인했습니다(`trail_smoke`). 창 표시는 GUI 전용이라 사용자 화면에서만 확인 가능합니다.

### #7 pre-dock 흔들림 대기 + 느린 이송

- **증상**: 이송 단계(sim 약 85 s) 중 약 40 s가 pre-dock 근처에서 대기. 이송 초반 약 6 s는 프로브가 거의 안 움직임.
- **원인 (기존 도킹 코드의 안전 조정값)**:
  - `docking.accel_mps2: 0.01`: 0.15 m/s까지 15 s 램프
  - `decel_gain_hz: 0.08`: 도착 1.9 m 전부터 계속 감속
  - `settle_window_s: 3.0`, `settle_window_m: 0.05`: 팔 + 3 t 짐의 흔들림(주기 약 20 s)이 가라앉을 때까지 대기
- **수정**: 이동 모드 전용 **강제 삽입**(`rendezvous.forced_insertion: true`).
  - pre-dock 50 mm 이내 도착 즉시 삽입 시작 (흔들림 대기 없음)
  - depth 불일치는 로그만 남기고 전진
  - MEP가 도킹축 방향으로 Client보다 항상 ≥ 5 mm/s 빠르게 접근
  - 재정렬 기준을 넓힘(#8)
  - 노즐 벽 여유 20 mm와 최종 FixedJoint 조건(40 mm, 2°)은 그대로
- **결과 (`moving_now`)**:
  - pre-dock 도착 sim **72 s** (이전 `moving_smoke2`는 117 s)
  - 0.5 s 뒤 Z_APPROACH 진입
  - sim 95 s FINAL_INSERTION, **97 s DOCK_READY**
  - 마지막 기록(101.6 s): 도킹점 오차 0.060 m / 0.21°, MEP–Client 상대속도 16.8 mm/s(결합 게이트 10 mm/s 대기 중), MRV 상대속도 0.00 mm/s, MRV 위치 오차 0.0 mm
  - 삽입 중 횡오차가 37 → 95 mm까지 커지는 것이 관찰됨 (넓힌 기준 안)

### #8 강제 삽입 기준이 사용자 설정보다 좁았음

- **증상**: 오프라인 테스트 실패 `assert 8.0 > 12.0`.
- **원인**: 사용자가 YAML에서 `approach_abort_axis_deg`를 12°, `approach_abort_lateral_m`를 0.20 m로 넓혀 두었는데, 강제 삽입 기준을 8°, 0.15 m로 잡아 오히려 더 좁았습니다.
- **수정**: `insertion_max_axis_deg` 12.0, `insertion_max_lateral_m` 0.20으로 맞춤. 테스트는 "강제 기준 ≥ 기존 기준"으로 변경.
- **결과**: 오프라인 테스트 통과.

### #9 도킹 직후 `AttributeError: 'VisionCaptureDemo' object has no attribute 'update_moving_visuals'`

- **증상**: 도킹이 거의 끝난 시점에 프로그램이 중단되고 창이 멈춰 강제종료가 필요했음. `moving_now`로 추정되며, CSV가 sim 101.6 s DOCK_READY에서 끝남.
- **원인**: **제 실수입니다.** 모니터 창 함수(`update_motion_hud`)를 다시 쓰면서 바로 뒤에 있던 `update_moving_visuals`를 함께 지웠습니다. 이 함수는 도킹 상태가 아닌 이동 상태에서 불리는데, 도킹 직후 STABILIZING에서 처음 호출되어 크래시가 났습니다.
  - 크래시 위치가 STABILIZING이라는 것은 `docking_joint`가 생성된 뒤라는 뜻입니다. 즉 **이동 중 결합 자체는 된 것으로 보입니다**(추정). CSV 버퍼가 비워지기 전에 중단돼 결합 순간의 행은 없습니다.
- **수정**: 함수 복구.
- **결과**: 컴파일 확인. 실행 검증은 **미완료**.

### #10 "The external wrench frame has been changed from False to True" 경고

- **증상**: 크래시 직전 로그에 출력됨.
- **원인**: Isaac Lab은 어떤 물체에 **처음으로 world 기준(`is_global=True`) 외력**을 줄 때 이 경고를 한 번 출력합니다. 방출·정지 추력을 world 기준으로 넣고 있었습니다. 크래시(#9)와 같은 시점에 찍혔지만 원인은 아닙니다.
- **수정**: world 기준 힘을 물체 좌표계로 변환해 기본 모드(`is_global=False`)로 넣도록 변경(`apply_wrench`). 물리적으로 주는 힘은 동일합니다.
- **결과**: 실행 검증 **미완료**.

### #11 위성이 흡착 이후에야 움직이기 시작함

- **증상/요청**: 전체 시뮬레이션(흡착부터)에서 위성이 처음부터 움직이길 원함. 이전 동작은 흡착이 끝나고 도킹으로 넘어갈 때 방출.
- **수정**: `client.release_at_start: true` (기본).
  - t = 0에 방출 추력을 시작합니다. MRV 접근·흡착 동안 MRV는 기존처럼 정지합니다(검증된 흡착 보존).
  - MRV 위치 목표 `d_ref`는 t = 0의 **공칭 배치**로 고정합니다. 도킹이 시작되면 MRV가 앞서간 위성(약 1.5 m 예상)을 이송과 동시에 따라잡고, 삽입은 위치 오차 ≤ 50 mm까지 대기합니다.
  - 위성은 흡착 동안 +X(MEP 반대쪽)로 멀어져 충돌 위험을 키우지 않습니다.
  - 이전 방식은 `--set client.release_at_start=false`.
- **결과**: 실행 검증 **미완료**. MRV 따라잡기에 약 70 s가 걸리고 이송 시간과 겹친다는 것은 추정입니다.

### #12 도킹 후 STOPPING에서 속도가 0이 안 되는 것처럼 보임

- **증상**: `full_moving`에서 도킹 후 STOPPING이 끝나지 않는 것처럼 보임.
- **원인 (로그 확인)**: 감속 자체는 설계대로 동작했습니다. 속도가 19.45 → 1.15 mm/s(sim 186.1 → 197.5 s)로 줄었습니다. 다만 감속 법칙 `a = min(v / 3 s, 0.002)`의 끝부분이 지수형이라 정지 기준(1 mm/s)에 천천히 다가가고, 그 뒤 1 s 유지 조건까지 있어 GUI(×8)에서는 멈추지 않는 것처럼 보였습니다. 흡착 해제는 세 물체가 **모두** 멈춘 뒤에야 가능했습니다.
- **수정**: `post_docking.immediate_release: true` (기본).
  - DOCKED → **즉시 흡착 해제**(ROBOT_RELEASE, 0.5 s 확인). STABILIZING·STOPPING·ARM_RETREAT는 거치지 않습니다.
  - 도킹된 MEP + 위성은 흡착 해제 후 **백그라운드로 감속**합니다(같은 추력 법칙, 팔과 분리된 상태). 두 속도가 1 mm/s 이하로 1 s 유지되면 추력을 끕니다.
  - MRV는 해제 직후 **지금 움직이던 속도(+X, 약 19.5 mm/s)에서 바로 감속·반전**해 −X로 0.02 m/s, 5 s 순항 후 정지합니다(가속 0.004 m/s²).
  - MRV가 스택보다 빨리 감속하므로 EE는 MEP 면에서 법선 방향으로 멀어집니다(팔 후퇴가 따로 필요 없음).
  - Mission complete 조건: MRV 정지, 스택 정지 완료, 거리 +0.1 m 이상, 도킹 유지.
  - 성공·실패 등 종료 상태에서는 남은 추력을 모두 끕니다. 외력 버퍼가 GUI 대기 루프에서 계속 적용되는 것을 막기 위해서입니다.
  - 이전 흐름은 `--set post_docking.immediate_release=false`.
- **결과**: 실행 검증 **미완료**. 예상(계산): 도킹 → 해제 0.5 s → 스택 정지 약 12 s, MRV 반전·순항·정지 약 20 s.

---

## 미해결 / 확인 필요

1. **#9·#10·#11 실행 검증**: 흡착부터 이탈까지 전체 런 1회. 확인할 것: 크래시 없음, 경고 없음, 위성이 t = 0부터 이동, 도킹 성공, 이후 단계(정지·해제·후퇴·이탈).
2. **도킹 이후 단계 (Test 4~6)**: `full_moving`에서 STOPPING까지는 실행됐습니다(#12). 새 흐름(즉시 해제 → 스택 정지 → MRV 반전·이탈)은 아직 실행되지 않았습니다.
3. **정지 위성 회귀**: #4의 depth 보정 수정이 기존 정지 위성 도킹(`--dock_only`)에 주는 영향을 확인하지 못했습니다.
4. **`full_moving`의 ALIGNMENT_CHECK 33 s 체류** (sim 122.5 → 155.8 s): 강제 삽입 모드인데도 삽입 시작까지 33 s가 걸렸습니다. 원인은 조사하지 않았습니다. 후보: 강제 삽입 기준(축·횡오차), MRV 상대속도·위치 게이트.
5. **삽입 중 횡오차 증가** (#7, 37 → 95 mm): 최종 결합 조건 40 mm를 넘으면 DOCK_READY에서 실패할 수 있습니다. 흔들림 감쇠(`swing_damping`)나 삽입 가속 조정이 후보입니다.
6. **MEP 속도 측정**: MEP는 kinematic으로 움직이는 팔에 끌려가 PhysX 속도가 부정확할 수 있어, 게이트에는 위치 기반 최소자승 속도를 씁니다. 두 값의 차이는 CSV(`mep_velocity_*` vs `mep_physx_velocity_*`)에 기록만 하고 분석하지 않았습니다.
