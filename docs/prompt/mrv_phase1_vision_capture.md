# MRV Phase 1 — AprilTag 비전 포획 (선형 유영 3 t MEP)

요구사항: [`mrv_phase1_apriltag_capture_prompt_linear_drift.md`](mrv_phase1_apriltag_capture_prompt_linear_drift.md)
6-DoF 확장 (XYZ 병진 + 복합회전, `mep.motion_mode: six_dof`): [`mrv_six_dof_capture.md`](mrv_six_dof_capture.md)
결과 보고서: `project/logs/phase1_result.md` · 작업 로그: `project/logs/phase1_progress.md` (둘 다 `logs/`라 git 에서 제외됨)

## 개요

```
cam_wrist 영상 → AprilTag 4개 (tag36h11, ID 0..3) → 16점 PnP → 태그 constellation pose
→ 고정 변환 T(constellation→Cylinder_01) → Cylinder_01 6-DoF 추정 (world / robot base)
→ 윈도우 최소자승 선속도 → 0.3 s 뒤 Cylinder_01 위치 예측 → EE 목표 (부착면 법선 정렬)
→ Canadarm3 DLS 미분 IK (관절 위치 목표 + 속도 피드포워드) → 단계별 감속 접근
→ capture 조건 5개 확인 → 기존 CaptureManager.attach() → UsdPhysics.FixedJoint
→ 10 s holding → 법선 방향 0.2 m 저속 후퇴 (MEP 동반 확인)
```

- 제어기는 Ground Truth 를 쓰지 않습니다. GT(`Cylinder_01` 실제 pose, MEP 속도)는 로그와 테스트 판정에만 쓰입니다.
- IBVS 가 아닙니다. 픽셀 오차가 아니라 추정한 6-DoF pose 를 로봇 좌표계 목표로 바꿔 IK 로 접근합니다.
- 기존 자석형 capture(`capture.py`)와 도킹 태스크(`docking.py`)는 그대로 재사용합니다.

## 실행

```bash
cd ~/space_robotics_bench
# 전체 검증: Test 1 (정지) + Test 2/3 (0.01, 0.02 m/s) → 보고서/CSV 생성, 모두 PASS 면 exit 0
~/isaac-sim/python.sh project/scripts/run_phase1_tests.py
# 개별 시나리오 (GUI: debug draw 확인용). 성공하면 창을 닫을 때까지 MEP 를 잡은 채 계속 실행
# (바로 끝내려면 --exit_when_done)
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario static --headless
# 설정 값 바꾸기
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --headless --set mep.linear_velocity_mps=0.02
# pytest (각 run 을 새로 시뮬레이션; PHASE1_REUSE=1 이면 마지막 결과만 판정)
cd project && ~/isaac-sim/python.sh -m pytest tests/test_vision_math.py tests/test_static_accuracy.py tests/test_dynamic_intercept.py tests/test_holding_stability.py -s
```

System `python3` 로는 실행되지 않습니다(Isaac Sim 번들 Python 필요). `test_vision_math.py` 는 시뮬레이터 없이 수 초 안에 끝납니다.

## 파일

| 파일 | 내용 |
|---|---|
| `project/config/vision_capture.yaml` | 모든 파라미터 (MEP 질량/속도, 카메라, 태그, 예측, 접근, capture, 테스트 기준) |
| `srb/tasks/manipulation/debris_capture/vision.py` | 설정 로더, 태그 텍스처/배치, 검출기, 16점 PnP, 선형 예측기 (Isaac Sim 불필요) |
| `srb/tasks/manipulation/debris_capture/vision_task.py` | 태스크 `srb/debris_capture_vision` (`DockingTask` 상속): 태그, `cam_wrist`, 유영 초기조건 |
| `srb/tasks/manipulation/debris_capture/vision_capture_demo.py` | 상태 머신, 예측 접근, capture/holding, CSV, debug draw, 2D overlay, 판정 |
| `srb/tasks/manipulation/debris_capture/frames.py` | `docking.py` 에서 옮긴 순수 numpy 좌표계 헬퍼 (`docking.py` 가 그대로 re-export) |
| `project/scripts/vision_capture.py` | 시나리오 1개 실행 |
| `project/scripts/run_phase1_tests.py` | 전체 검증 + `logs/vision_capture_metrics.csv` 병합 + `logs/phase1_result.md` |
| `project/tests/test_*.py` | 오프라인 단위 테스트 + Test 1/2/3 |

출력: `project/logs/vision_capture/<run>_{result.json,metrics.csv,console.log,overlay/,overlay.mp4}`

## 좌표계

| 기호 | Frame |
|---|---|
| W | world |
| B | robot base `canadarm3_large_0` (world 원점 고정, Phase 1 의 MRV frame) |
| L | Canadarm3 마지막 링크 `canadarm3_large_7` |
| C | `cam_wrist` 광학 frame (OpenCV: +X 오른쪽, +Y 아래, +Z 전방) — L 기준 (0, 0, −0.45) m, X축 180° |
| E | EE 접촉면 (capture cylinder 바깥 끝면 중심, +Z = 접근 방향) |
| M | MEP 강체 |
| T | 태그 constellation: 원점 = Cylinder_01 중심을 부착면에 투영(+1 mm), +Z = 부착면 바깥 법선 |
| Y | `Cylinder_01` (GT 결합점) |

`T_W_Y(추정) = T_W_L · T_L_C · T_C_T(PnP) · T_T_Y`, `T_T_Y` 는 시작 시 USD 에서 한 번 계산하는 고정값 (0, 0, −45.2 mm).
EE 목표: `T_W_T(예측) · Trans(0, 0, gap − 1 mm) · RotX(180°)` (EE +Z = −법선, EE +X = 부착면 +X — 기존 도킹 데모와 같은 정렬).

## 상태 머신

`INIT → SEARCH → TAG_DETECTED → POSE_ESTIMATED → PREDICTING → APPROACHING → SLOW_APPROACH → CAPTURE_ATTEMPT → CAPTURED → HOLDING → RETREAT → SUCCESS`
(정지 시나리오: `PREDICTING → STATIC_MEASURE → SUCCESS`)
실패: `TAG_LOST`, `POSE_INVALID`, `PREDICTION_INVALID`, `APPROACH_TIMEOUT`, `CAPTURE_FAILED`, `PHYSICS_ERROR` — 팔은 현재 관절을 유지(안전 정지).

| 단계 | 동작 |
|---|---|
| Stage A/B (APPROACHING) | 예측 Cylinder_01 앞 0.6 m 로 0.10 m/s 접근 후 추종 |
| Stage C (SLOW_APPROACH) | 간격을 0.04 m/s → (0.30~0.12 m 에서 선형 감소) → 0.01 m/s 로 줄임. EE 가 추정 축 위(측면 < 10 mm, 각도 < 1°)이고 상대속도 < 0.04 m/s 일 때만 전진 |
| Stage D | 간격 0.12 m 이하 = capture range, 최종 간격 50 mm 에서 0.3 s 유지 |
| Stage E/F (CAPTURE_ATTEMPT) | 거리 ≤ 0.15 m, 각도 ≤ 5°, 상대속도 ≤ 0.05 m/s, 부착면 바깥 + 측면 ≤ 20 mm, tracking 정상(유효 pose 나이 ≤ 1 s, 4/4 태그, 속도 추정 존재) → `[CAPTURE] ATTEMPT` → `CaptureManager.attach()` → `[CAPTURE] CAPTURED` |

Tag loss: 마지막 유효 pose + 예측으로 계속, 1 s 초과 시 `TAG_LOST` (안전 정지).

## 구현하면서 실측으로 정한 것

| 항목 | 결정 | 근거 (측정) |
|---|---|---|
| 카메라 위치 | EE 축 위, flange 바로 앞 | 최종 간격 50 mm 에서도 4 태그가 화면 안에 들어가야 함 |
| EE capture cylinder | 반투명 유리(opacity 0.35, ior 1.0), 반지름 0.20 m (flange 둘레 0.17~0.19 m 에 맞춤), 길이 0.4 m, 128분할 매끈한 메쉬로 표시 | 기본 implicit Cylinder 는 반투명 렌더에서 윤곽이 울퉁불퉁 → 표시만 메쉬로 교체(충돌체·접촉면 기준은 implicit 유지). 카메라가 원기둥을 투과해 봄: 보임/숨김 A/B 에서 코너 잔차 차이 ≤ 0.05 px. 속 빈 파이프는 마커와 크기를 맞추는 방식에서 구멍이 태그를 가려 제외 |
| MEP 마커 `Cylinder_01` | 반지름을 EE 원기둥과 같은 0.20 m 로 확대 (위치·자세·높이 110 mm 유지) | 정렬되면 두 원기둥이 정확히 겹쳐 오차가 눈으로 보임 |
| 카메라 화각 | 수평 90° (세로 ±29.4°) | 0.20 m 마커 바깥의 태그가 최종 간격에서도 화면에 들어오려면 필요 (70° 로는 불가) |
| 태그 크기/배치 | 60 mm, d = 190 mm | 마커 가장자리에서 16 mm 밖 (가장 가까운 여백 모서리가 축에서 216 mm) |
| 코너 정제 | `CORNER_REFINE_APRILTAG` | GT 코너 투영 대비 편향 ≈ 0 (Isaac K 는 연속 좌표 규약, SUBPIX 는 −0.5 px 편향) |
| 관측 거리 | 0.7 m | 90° 화각에서 태그 ~34 px 유지 (30 px 이하에서 디코딩 실패 관찰) |
| 렌더링 | AA Off + 2× 슈퍼샘플 → 면적 평균 | 기본 DLSS 는 저해상도 업스케일. 단일 영상 재투영 RMS 0.63 → 0.42 px |
| 태그 재질 | 순수 발광(diffuse 0) | 조명 방향 무관 |
| 자세 추정 | 윈도우(5 s) chordal 평균 + IPPE 2중 해 선택 | 단일 영상 오차는 거의 전부 면 밖 기울기이고 평균 ≈ 0 (편향 < 0.07°), 정면 근처에서 두 해가 번갈아 선택됨 |
| debug draw | 비전용 렌더 직전에만 지움 | debug draw 는 `cam_wrist` 영상에도 그려져 검출을 망가뜨림 |
| 속도 피드포워드 | q̇* = J⁺[v̂_MEP; 0] | 위치 목표만 주면 관절 PD 가 이동 목표를 damping/stiffness = 0.625 s 만큼 뒤따름 (0.02 m/s 에서 slow approach 가 진행 안 됨) |

## 알려진 한계

- 정지 테스트에서 팔이 완전히 멈추면 같은 영상이 반복되어 평균 효과가 없고, 결과가 그 자세 한 장의 오차로 정해집니다. 태그 배치를 키운 뒤(폭 0.46 m) 최대 0.26° 로 여유가 생겼지만, 단일 영상 기울기 노이즈 자체가 없어진 것은 아닙니다.
- 반투명 원기둥이 카메라 화면을 덮으면 RTX 렌더 비용이 커져 wall time 이 약 2.7배 늘어납니다(시뮬레이션 결과에는 영향 없음).
- Holding 중 팔+3 t 결합계에 주기 ~8 s, 수 mm/s 의 잔류 진동이 남습니다(발산 없음, MEP–EE 상대 드리프트 < 0.1 mm).
- 관측 시작 자세는 설정된 명목 랑데부 자세에서 계산합니다(Phase 1 은 사각지대 탐색 제외).
- `SimulationApp.close()` 가 exit code 0 으로 프로세스를 끝내므로, `vision_capture.py` 는 결과 기록 후 `os._exit(code)` 로 종료합니다. 기존 `satellite_docking_demo.py` 는 같은 이유로 실패해도 exit code 0 을 반환합니다(수정하지 않음).
