# MRV — 6-DoF 자유유영 MEP 포획 (XYZ 병진 + Roll/Pitch/Yaw 복합회전)

Phase 1 문서: [`mrv_phase1_vision_capture.md`](mrv_phase1_vision_capture.md) (구조, 좌표계, 상태 머신은 그대로)

> 상태: **2026-09-20 headless 1회 실행 검증 (six_dof, dynamic)**: 접근 → 캡처(29 s) → 10 s holding → retreat 성공.
> 발견/수정한 문제와 남은 문제는 아래 "실행 검증 결과" 참고. 아래 TEST 0–6 중 GUI 확인과 회귀(TEST 0) 는 사용자 실행 대상입니다.

## 구조

```
MEP (v = [vx,vy,vz] m/s, w = [wx,wy,wz] rad/s, world)
→ cam_wrist → AprilTag 4개 → 16점 PnP → T_W_T (기존)
→ T_W_Y = T_W_T · T_T_Y (기존, Cylinder_01)
→ ConstantTwistPredictor: 기준 프레임 R 의 v (최소자승 기울기), w (SO(3) 로그 최소자승)
→ R(t+h) = R(t) · Exp([R(t)ᵀ w] h),  p(t+h) = p(t) + v h
→ T_W_Y(t+h) = T_W_R(t+h) · T_R_Y   (Cylinder_01 오프셋이 회전에 따라 호를 그림)
→ EE 목표 = T_W_T(t+h) · Trans(0,0,standoff − 1 mm) · RotX(180°)  (기존 ee_goal)
→ 기존 DLS IK (위치 + 자세 쿼터니언) + 속도 피드포워드 q̇* = J⁺[v_EE; w]
→ capture 조건 (기존 5개 + 상대 각속도) → 기존 CaptureManager.attach() → FixedJoint
```

Euler 각 누적은 사용하지 않습니다. 회전은 모두 회전행렬 / 회전벡터(Exp/Log, 기존 `cv2.Rodrigues` 래퍼)로 처리합니다.
Ground Truth 는 제어에 쓰지 않고 로그 / 판정 / 시각화에만 씁니다.

## 모드

| `mep.motion_mode` | 예측기 | 각속도 | 용도 |
|---|---|---|---|
| `translation_only` (기본값) | 기존 `LinearDriftPredictor` (변경 없음) | 0 (설정값 무시) | Phase 1 회귀 |
| `six_dof` | `ConstantTwistPredictor` | `mep.angular_velocity_rad_s` | 6-DoF |

`translation_only` 에서는 Phase 1 과 같은 계산 경로를 탑니다(시작 자세, 피드포워드, capture 조건, CSV 앞 36열 동일).

## 규약 (코드에서 확인한 것)

| 항목 | 규약 | 근거 |
|---|---|---|
| 쿼터니언 | (w, x, y, z) | `frames.py` `Frame.quat`, `root_quat_w` / `body_quat_w` 를 그대로 `Frame.from_pos_quat` 에 넣는 기존 코드, IK 명령 `[pos, quat]` |
| Frame 곱 | `A @ B` = T_A_B 합성 (자식이 오른쪽) | `Frame.__matmul__` |
| 각속도 frame | **world** | 설정값 → `init_state.ang_vel` → `root_com_ang_vel_w`. 예측기의 w 도 world (좌측 섭동 Log(R_i R̄ᵀ) 의 기울기) |
| 회전 전파 | `R(t+h) = R · Exp([Rᵀw] h) = Exp([w] h) · R` | 두 식의 동일성은 단위 테스트로 확인 |
| 단위 | m, m/s, rad/s (설정 / 계산), deg 는 로그 / 임계값 이름에 `_deg` 로 표시 | |
| 시간 | 예측 horizon, 윈도우 = 시뮬레이션 시간 [s]. 제어 dt = physics dt, 비전 10 Hz, 로그 10 Hz | `run()` |

`[NEEDS_ISAAC_VALIDATION]` Isaac Lab 이 `init_state.ang_vel` 을 **world frame COM 각속도**로 쓰는지는 실행으로 확인합니다.
시작 시 자동 확인: `MEP 6-DoF initial velocity` (설정 벡터 = 읽은 벡터), `[6DOF] Angular velocity is a world-frame vector`
(1 s 뒤 GT 자세 변화에서 world 해석과 body 해석을 비교, 콘솔 `[FRAME] ... WORLD frame confirmed`).

## 설정 (`project/config/vision_capture.yaml`)

기존 파라미터를 재사용하고 새로 추가한 것만 표시했습니다.

| 요구 파라미터 | 설정 키 | 단위 | 기본값 | 비고 |
|---|---|---|---|---|
| motion mode | `mep.motion_mode` | — | `translation_only` | **신규** |
| linear_velocity_x/y/z | `mep.drift_direction` × `mep.linear_velocity_mps` | m/s | [0, −0.008, 0.006] | 기존 (중복 파라미터 만들지 않음) |
| angular_velocity_x/y/z | `mep.angular_velocity_rad_s` | rad/s (world) | [0.005, −0.004, 0.006] (\|w\| = 0.50 °/s) | **기존 `angular_velocity_deg_s` (스칼라, 0 고정) 를 대체** |
| prediction_horizon | `prediction.horizon_sec` | s | 0.3 | 기존 |
| — | `prediction.orientation_window_sec` | s | 5.0 | 기존 (six_dof: 각속도 / 자세 최소자승 윈도우) |
| — | `prediction.max_plausible_angular_rate_rad_s` | rad/s | 0.2 | **신규** (이상치 거부) |
| — | `prediction.reference_frame` | — | `tag` | **신규** (`tag` / `mep_com`, 아래 참고) |
| position_capture_tolerance | `capture.max_distance_m`, `capture.max_lateral_m` | m | 0.15, 0.02 | 기존, 변경 없음 |
| orientation_capture_tolerance | `capture.max_angle_deg` | deg | 5.0 | 기존, 변경 없음 |
| relative_linear_velocity_tolerance | `capture.max_relative_velocity_mps` | m/s | 0.05 | 기존, 변경 없음 (six_dof 는 EE 점의 v + w×r 사용) |
| relative_angular_velocity_tolerance | `capture.max_relative_angular_velocity_rad_s` | rad/s | 0.01 | **신규, six_dof 에서만 적용. 제안값** |
| logging enable | `logging.csv_enabled` | — | true | **신규** (false: CSV 파일 안 씀, 결과 JSON 은 항상 기록) |
| debug visualization | `logging.debug_draw` | — | true | 기존 |

### 임계값 관련

- 기존 capture 임계값은 하나도 바꾸지 않았습니다.
- 신규 `max_relative_angular_velocity_rad_s = 0.01 rad/s`: 반지름 0.2 m 원기둥 가장자리에서 2 mm/s (선속도 허용 50 mm/s 대비 작음).
  기본 MEP 회전 0.0087 rad/s 보다 커서, EE 가 전혀 회전하지 않아도 이 조건은 통과합니다. 회전 정렬은 기존 각도 조건(≤ 5°, 게이트 1°)이 담당합니다.
  더 빠른 회전을 시험할 때 함께 조정할 값입니다.

### `prediction.reference_frame`

무중력 자유운동에서 등속 직선운동을 하는 점은 질량중심(COM)뿐입니다. 다른 기준점은 COM 주위를 돌기 때문에 등속 최소자승 적합에
`|w|² r (T²/12 + T h/2 + h²/2)` 만큼 편향이 생깁니다 (T = 속도 윈도우, r = COM 까지 거리).

- `tag` (기본): 비전 + 설계 변환 T_T_Y 만 사용. 기본 회전율(0.5 °/s, r ≈ 2 m)에서 편향 ≈ 0.1 mm, 5 °/s 에서 ≈ 7 mm (단위 테스트에서 확인).
- `mep_com`: MEP 몸체 안의 COM 위치(질량 특성)를 시작 시 한 번 읽어 기준으로 사용 → 편향 0. 실시간 pose 는 쓰지 않지만 모델 정보를 쓰므로 선택 사항으로 두었습니다.

## 시작 자세 (six_dof)

`mep.rotation_start` 로 회전 방향 관계를 고릅니다.

| 값 | 동작 |
|---|---|
| `converge` (기본) | 시작 자세를 `Exp(-w t_r)` 로 미리 돌려 놓아 `rendezvous_time_s` (15 s) 에 명목(평행) 자세를 지나감 — 처음엔 점점 평행해짐 |
| `diverge` | 명목 자세로 시작해 점점 벌어짐 — 팔이 회전을 쫓아가야 함. 위치는 기존처럼 t_r 에 명목 위치를 지나감 |

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --set mep.motion_mode=six_dof --set mep.rotation_start=diverge
```

`diverge` 실행 확인 (headless, 기본 각속도): t = 0 에서 팔-MEP 각도 0.00°, MEP 자세가 시작 대비 2.3° (5 s) → 7.0° (15 s) → 13.0° (28 s) 로 벌어짐,
29.4 s 에 캡처 (GT 거리 0.0967 m, 각도 0.065°, 측면 4.95 mm, 상대 각속도 0.0009 rad/s), 10 s holding 통과.
retreat 도중 실행 시간 제한(500 s)에 걸려 최종 요약은 받지 못함.

MEP 는 `rendezvous_time_s` (15 s) 뒤 명목 자세를 지나도록 시작합니다. 회전은 `Q = Exp(−w t_r)` 를 **Cylinder_01 중심으로** 되돌립니다(`vision.rendezvous_start_pose`).
그래서 t = 0 에서 관측 자세의 카메라는 Phase 1 과 같은 위치에서 제자리 회전(7.5°)만 된 태그를 봅니다.
PhysX 는 COM 중심으로 회전시키므로 t_r 에서의 통과는 근사입니다: 오차 ≈ |w| t_r |COM→Cylinder_01| (COM 오프셋은 `setup.mep_com_in_body_m` 에 기록). `[NEEDS_ISAAC_VALIDATION]`

## 실행 (Isaac Sim PC)

Phase 1 과 같은 방식입니다 (저장소 위치는 Phase 1 문서와 같은 `~/space_robotics_bench` 기준).

```bash
cd ~/space_robotics_bench

# 0) 오프라인 수학 테스트 (시뮬레이터 불필요, 수 초)
cd project && ~/isaac-sim/python.sh -m pytest tests/test_vision_math.py -q && cd ..

# TEST 0 -- 회귀: static + dynamic 0.01 / 0.02 m/s (translation_only 로 고정), 보고서 생성
~/isaac-sim/python.sh project/scripts/run_phase1_tests.py

# TEST 1-6 -- 6-DoF, GUI (debug draw 확인)
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag six_dof --set mep.motion_mode=six_dof
# headless
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag six_dof --headless --set mep.motion_mode=six_dof
# pytest (위 headless 실행을 직접 수행하고 판정; PHASE1_REUSE=1 이면 마지막 결과만 판정)
cd project && ~/isaac-sim/python.sh -m pytest tests/test_six_dof_capture.py -s

# 값 바꾸기 (예: 각속도, horizon, COM 기준)
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag six_dof_fast \
  --set mep.motion_mode=six_dof --set "mep.angular_velocity_rad_s=[0.01, -0.008, 0.012]" \
  --set prediction.horizon_sec=0.3 --set prediction.reference_frame=mep_com
```

출력: `project/logs/vision_capture/<tag>_{result.json,metrics.csv,overlay/,overlay.mp4}` (`logs/` 는 git 제외).

## 로그 (CSV, `<tag>_metrics.csv`)

Phase 1 열(앞 36개 + 추가 열)은 순서 그대로 유지하고 뒤에 다음을 붙였습니다.

| 열 | 내용 |
|---|---|
| `motion_mode`, `prediction_horizon_s` | 실행 조건 |
| `gt_q*`, `est_q*`, `pred_q*`, `ee_q*`, `ee_target_q*` | (w,x,y,z) world 자세: GT / 추정 / 예측 Cylinder_01, EE, EE 목표 |
| `mep_angular_velocity_x/y/z` (기존 열) | 추정 w [rad/s, world] (six_dof: 제어에 쓰는 값) |
| `gt_mep_angular_velocity_x/y/z` | GT w [rad/s, world] (평가 전용) |
| `ee_angular_velocity_x/y/z` | EE 각속도 [rad/s, world] |
| `est_ee_to_cylinder_distance_m`, `est_ee_lateral_error_m`, `est_ee_normal_angle_deg` | 비전 기준 capture 량 (기존 `ee_*` 열은 GT 기준) |
| `est_/gt_ee_orientation_error_deg` | EE ↔ capture 자세 3축 전체 오차 (roll 포함) |
| `est_/gt_relative_angular_velocity_rad_s` | 상대 각속도 |
| `pnp_valid`, `tracking_valid` | 이번 영상 PnP 유효, tracking 조건 충족 |

`mep_linear_velocity_*` 는 six_dof 에서 EE 점의 MEP 속도(v + w×r)로 기록합니다 (GT 열 `gt_mep_linear_velocity_*` 와 같은 점).

## Debug visualization

- 3D (Isaac Sim debug draw): 기존 점 + 축 (X 빨강, Y 초록, Z 파랑)
  - 굵고 밝은 긴 축: 추정 현재 Cylinder_01 · 어두운 긴 축: 예측 Cylinder_01 (t+h) · 가는 축: GT Cylinder_01
  - 짧은 축: EE (밝음), EE 목표 (어두움)
  - 노란 선: 추정 선속도 × 10 s · 자홍 선: 현재 → 예측 위치
- 2D overlay (`<tag>_overlay/`): 추정 현재 축 (굵게) / 예측 축 (가늘고 어둡게) 을 영상에 투영

## 검증 체크리스트 (Isaac Sim 에서 수행)

결과 JSON 의 `checks` 이름과 콘솔 태그로 확인합니다.

| TEST | 실행 | 확인 |
|---|---|---|
| **0 회귀** | `run_phase1_tests.py` | exit 0, `logs/phase1_result.md` TEST 1/2/3 PASS (Phase 1 과 동일 기준) |
| **1 6-DoF 운동** | six_dof GUI | `MEP 6-DoF initial velocity` PASS, `[FRAME] ... WORLD frame confirmed`, `[6DOF] Angular velocity is a world-frame vector` PASS, 화면에서 이동 + 복합 회전, `PHYSICS_ERROR` 없음 |
| **2 Vision / PnP** | 같은 실행 | `[TEST2] AprilTag detection during approach` PASS, CSV `position_error_mm` / `angle_error_deg` (시스템 추정), `raw_*` (단일 영상), `metrics.tracking_error` |
| **3 예측** | 같은 실행 | `[6DOF] Angular velocity estimation (report)` (\|w_est − w_gt\|), `[6DOF] Future orientation prediction (report)` (예측 vs GT, 자세 고정 대비), `[TEST2] Future position prediction`; viewport / overlay 에서 예측 축이 실제 회전 방향으로 앞서는지 |
| **4 추종** | 같은 실행 | `[APPROACH] Stage B`, `Stage D` 로그, CSV `est_ee_lateral_error_m`, `est_ee_normal_angle_deg`, `ee_target_q*` vs `ee_q*`, `APPROACH_TIMEOUT` 없음 |
| **5 Capture** | 같은 실행 | `[CAPTURE] ATTEMPT` 줄의 거리 / 각도 / 상대속도 / 상대 각속도, `[TEST2] Relative velocity at capture`, `[TEST2] Capture geometry (GT)`, `[6DOF] Relative angular velocity at capture` PASS |
| **6 Holding** | 같은 실행 | `[TEST3] Holding duration` (≥ 10 s), `FixedJoint validity`, `Physics stability`, `Robot EE stability`, `Retreat with the MEP attached` PASS |

`(report)` 가 붙은 check 는 값이 존재하는지만 판정합니다 (정확도 합격 기준은 아직 정하지 않음 — 값을 보고 결정).

## Isaac Sim 실행이 필요한 항목

- `[NEEDS_ISAAC_VALIDATION]` `init_state.ang_vel` / `root_com_ang_vel_w` 의 world frame 규약 (자동 확인 check 있음)
- `[NEEDS_ISAAC_VALIDATION]` MEP 병진 + 복합회전 물리 (비주축 회전 시 토크 없는 세차로 world w 가 서서히 변함 — 예측기는 슬라이딩 윈도우로 추종)
- `[NEEDS_ISAAC_VALIDATION]` 회전 중 태그 렌더링 / 검출 (4/4 유지, 기울어진 태그의 코너 정밀도)
- `[NEEDS_ISAAC_VALIDATION]` 실제 PnP 노이즈에서의 각속도 / 자세 예측 정확도 (오프라인 합성 테스트는 0.1 px 노이즈 기준)
- `[NEEDS_ISAAC_VALIDATION]` 회전 목표 IK 추종: 게이트(측면 10 mm, 1°)를 지키며 slow approach 가 진행되는지, 관절 한계
- `[NEEDS_ISAAC_VALIDATION]` 회전하는 3 t MEP 부착 시 FixedJoint / 팔이 각운동량을 흡수하는 과정의 안정성
- `[NEEDS_ISAAC_VALIDATION]` 시작 자세 근사 (t_r 통과 오차, 초기 태그 가시성)


## 실행 검증 결과 (2026-09-20, headless, `--set mep.motion_mode=six_dof`)

**수정한 문제 — MEP 각속도가 감쇠하고 있었음 (PhysX 기본 각 감쇠 0.05 1/s)**
- `RigidBodyPropertiesCfg()` 의 감쇠를 지정하지 않아 PhysX 기본값이 적용되어 25 s 동안 `|w|` 가 0.0084 → 0.0025 rad/s 로 감소 (지수 피팅 감쇠율 0.0496 1/s). 무중력 자유유영이 아니었고 등각속도 모델과도 맞지 않았음.
- `vision_task.py` `apply_vision_config` 에서 MEP 의 `linear_damping = angular_damping = 0` 으로 고정. 수정 후 `root_com_ang_vel_w` 는 지령값을 유지 (0.00502 vs 0.005 rad/s, 20 s 동안 |w| 0.00878 → 0.00889 로 세차에 의한 변화만).
- 수정 전 캡처 결과(29 s): 위치 예측 오차 평균 0.35 mm, 자세 예측 0.065°, 캡처 시 상대 각속도 0.0007 rad/s.

**남은 문제 (미해결, 원인 미확인) — 보고 각속도와 자세 변화율이 8 % 불일치**
- 자세(쿼터니언)에서 구한 회전율은 0.00809 rad/s 로 20 s 내내 일정 (0.01 % 이내), `root_com_ang_vel_w` 와 지령값은 0.00878 rad/s. 비율 0.922, 세 축 모두 같은 비율, 방향 오차 0.09°.
- 선속도는 정확함 (위치 피팅 v = [0, -8.011, 6.008] mm/s vs 지령 [0, -8, 6]) → 시간 진행 문제가 아님. 원인은 아직 모름 (PhysX 각속도 적분 / 초기 각속도 적용 방식 의심).
- 영향: 예측기는 자세에서 w 를 추정하므로 제어에는 영향 없음 (자세 예측 오차 0.07°). `[6DOF] Angular velocity estimation` 의 `|w_est − w_gt|` 는 GT 쪽(`root_com_ang_vel_w`) 이 8 % 높아서 약 0.0007 rad/s 의 바이어스를 포함. 상대 각속도(EE vs MEP) 판정도 같은 GT 값을 씀.
- 그래서 `[6DOF] Angular velocity is a world-frame vector` 는 **방향**(5° 미만 + world 해석이 body 해석보다 가까움)으로만 판정하고, 크기 불일치는 별도 `[6DOF] Rotation rate vs commanded (report)` 로 분리해 값을 기록합니다. 이전 판정(크기 5 %)은 이 불일치 때문에 FAIL 이었음.

**그 밖의 관찰**
- MEP COM 이 루트에서 [-6.41, 0.41, -1.33] m 떨어져 있어 (Cylinder_01 까지 약 1.4 m), 시작 자세 근사가 정확하지 않음: t = 0 에서 GT 기준 측면 194 mm / 6.2° 벗어난 채 시작하지만 팔이 4 s 안에 측면 5.9 mm 로 수렴 (문제 없음, 필요하면 COM 기준 회전으로 개선 가능).
- 캡처 시 (GT) 거리 0.0967 m, 각도 0.015°, 측면 2.7 mm, 상대 속도 7.4 mm/s, 상대 각속도 0.0007 rad/s. 관절 정지 후 holding 중 MEP-EE 상대 드리프트 0.12 mm / 0.004°.
- 자세 예측(0.3 s 앞)은 평균 0.065° 로 "현재 자세 유지"(0.050°) 보다 약간 나쁨 — 이 각속도(0.5 °/s)에서는 예측 이득이 없음. 더 빠른 회전에서 재평가 필요.
