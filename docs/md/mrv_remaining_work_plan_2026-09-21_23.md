# MRV 1차 시연 준비 및 2026-09-21~23 잔여 개발·진척률 관리 계획

## 1. 문서 목적과 기준 시각

- 목적: 현재 `feature/apriltag` 체크아웃을 기준으로 2026-09-21(월) 1차 시연의 성공 가능성을 높이고, 9/21~9/23 잔여 개발을 증거 기반으로 관리한다.
- 기준 시각: **2026-09-20 16:49 KST**
- 기준 커밋: `923e379924ca427a4972dbb9e70a12ec7c659a4c` (`apriltag 방식 부착`)
- 일정 가정: 기존 요구사항 문서의 9/21 10:00 시연 일정을 임시 기준으로 삼는다. 실제 시연 시간이 다르면 월요일 블록을 같은 순서로 평행 이동한다.
- 중요한 제한: 이 문서는 저장소와 Git 이력의 **정적 조사 결과**다. 현재 머신에서 `~/isaac-sim/python.sh`와 실행 로그를 확인하지 못했으므로, 이번 조사 자체가 Isaac Sim 실행 성공을 새로 증명하지 않는다.

증거 수준은 다음처럼 고정한다.

| 수준 | 진척률 | 이 문서에서의 판정 규칙 |
|---|---:|---|
| 미착수 | 0% | 설계 아이디어만 있거나 대상 코드가 없음 |
| 작업 중 | 30% | 일부 구성요소만 존재하거나 통합 경로가 없음 |
| 구현 및 정적 확인 | 60% | 실행 코드가 있고 구문·호출 경로를 정적으로 확인함 |
| headless/자동 시험 통과 | 85% | 현재 기준 커밋의 명령·원시 결과 로그·exit code가 보존됨 |
| GUI E2E 반복 및 기록 | 100% | 같은 기준 커밋에서 GUI 전체 흐름을 반복 성공하고 영상·로그를 보존함 |

> 과거 문서의 “PASS”와 커밋 메시지의 “성공”은 참고 근거다. 현재 체크아웃에 원시 로그가 없으므로 현재 증거 수준을 85%로 올리는 근거로 쓰지 않는다.

## 2. 현재 저장소·브랜치·변경 상태

| 항목 | 현재 값 | 판정 |
|---|---|---|
| 저장소 | `/Users/min/orca/projects/isaac-sim` | 확인 |
| 브랜치 | `feature/apriltag` | 전환하지 않음 |
| HEAD | `923e379` | `origin/feature/apriltag`와 동일 |
| 추적 파일 변경 | 없음 | clean |
| 추적되지 않은 사용자 작업 | `docs/images/`, `docs/md/`, `docs/mrv_phase2_ml_pose_estimation.md` | 보존, 수정·삭제하지 않음 |
| `AGENTS.md` | 파일은 있으나 내용 없음 | 추가 저장소 지침 없음 |
| `.codegraph/` | 없음 | 대상 파일·Git 이력 직접 조사 |
| 현재 결과 로그 | `project/logs/`에 결과 JSON/CSV/console log 없음 | 현재 런타임 결과 미검증 |
| 로그 추적 정책 | `project/.gitignore:31-32`, 루트 `.gitignore`가 `*.log`, `logs/` 제외 | 시연 증거는 별도 artifact 경로로 복사 필요 |
| 정적 확인 | 핵심 Python 9개 파일 `ast.parse` 통과, `git diff --check` 통과 | 실행 검증 아님 |

최근 관련 커밋은 다음과 같다.

| 커밋 | 일시(KST) | 의미 | 주의 |
|---|---|---|---|
| `923e379` | 2026-09-19 17:46 | AprilTag/PnP, 선형 예측, 비전 포획 상태 머신·시험 코드 추가 | 도킹 커밋 위에 존재하지만 단일 E2E 통합을 뜻하지 않음 |
| `5b7e55b` | 2026-09-18 18:38 | Canadarm3의 MEP 포획·운반·Client 도킹 데모 추가 | 문서상 headless 25/25 PASS이나 현재 원시 로그 없음 |
| `13884f3` | 2026-09-18 14:27 | MEP 3 t 관련 변경 | 현재 질량 설정의 선행 이력 |
| `6e86135` | 2026-09-18 12:20 | 그리퍼 대신 흡착/FixedJoint 시험 환경 | 현재 `CaptureManager`의 선행 이력 |

## 3. 현재 구현 현황과 근거

### 3.1 AprilTag 기반 MEP 인식·예측·포획

현재 코드 경로는 다음과 같다.

```text
project/scripts/vision_capture.py
  -> srb/debris_capture_vision
  -> VisionCaptureTask(DockingTask)
  -> VisionCaptureDemo
  -> cam_wrist RGB
  -> AprilTag 4개 / 16점 PnP
  -> T_W_Y = T_W_L · T_L_C · T_C_T · T_T_Y
  -> LinearDriftPredictor
  -> Canadarm3 DLS IK + velocity feed-forward
  -> CaptureManager.attach()
  -> EE↔MEP FixedJoint
  -> 10 s hold + 0.2 m retreat
```

근거:

- 실행 진입점과 결과 JSON/CSV 생성: `project/scripts/vision_capture.py:29-135`
- MEP 3,000 kg, 0.01 m/s, 회전 0, 0.3 s 예측, 태그 소실 1 s: `project/config/vision_capture.yaml:9-19,64-80`
- capture 게이트: 거리 0.15 m, 자세 5°, 상대속도 0.05 m/s, 횡오차 0.02 m: `project/config/vision_capture.yaml:108-123`
- 태그·카메라·MEP 초기 선속도 적용: `project/srb/tasks/manipulation/debris_capture/vision_task.py:39-89,149-160`
- 규칙 기반 baseline은 최근 2 s 위치의 최소제곱 선형 적합과 등속 외삽: `project/srb/tasks/manipulation/debris_capture/vision.py:581-675`
- 상태 머신과 실패 상태: `project/srb/tasks/manipulation/debris_capture/vision_capture_demo.py:59-84`
- NaN/Inf, MEP 속도 0.5 m/s, 각속도 5°/s, 관절 속도 1 rad/s, 예상 밖 접촉 200 N 감시: `vision_capture_demo.py:527-547`
- 자동 시험 runner와 보고서 생성 코드: `project/scripts/run_phase1_tests.py`

현재 판정은 **구현 및 정적 확인 60%**다. 결과 보고서가 있어야 할 `project/logs/phase1_result.md`와 per-run JSON/CSV가 현재 체크아웃에 없다.

### 3.2 MEP 운반·Client 위성 정렬·도킹

별도 실행 경로가 구현되어 있다.

```text
project/scripts/satellite_docking_demo.py
  -> srb/debris_capture_docking
  -> DockingDemo
  -> INIT / PLAN
  -> MEP pre-grasp / align / attach
  -> lift / transport
  -> Client pre-dock / align / probe insert
  -> MEP↔Client FixedJoint
  -> DOCKED hold
```

근거:

- 단일 실행 스크립트: `project/scripts/satellite_docking_demo.py:22-103`
- 상태 머신: `project/srb/tasks/manipulation/debris_capture/docking_demo.py:121-136,720-885`
- MEP 부착 기준: 면 간격 ±4 mm, 횡오차 5 mm, 법선 0.5°, roll 0.5°, 접근 방향 3°: `docking_demo.py:92-100`
- 도킹 기준: 축방향·반경 10 mm, 축 0.5°, roll 1°, 접근 방향 3°: `docking_demo.py:101-109`
- MEP↔Client FixedJoint 생성/해제: `project/srb/tasks/manipulation/debris_capture/docking.py:418-468`
- 문서상 과거 headless 명령 2회 25/25 PASS: `docs/satellite_docking_demo.md:208-224`
- 문서상 GUI는 INIT/PLAN smoke까지만 확인됐고 전체 GUI는 남은 항목: `docs/satellite_docking_demo.md:227-234`

현재 판정은 **구현 및 정적 확인 60%**다. 과거 headless 결과는 유의미하지만 현재 기준 커밋의 원시 로그가 없고, 전체 GUI E2E는 문서 자체가 미완료로 표시한다.

### 3.3 두 경로의 통합 여부

두 기능은 같은 `DockingTask` 계열과 `CaptureManager`, frame helper를 재사용하지만 실행 스크립트와 상태 머신은 분리되어 있다.

- AprilTag 포획: `vision_capture.py` + `VisionCaptureDemo`
- 기존 도킹: `satellite_docking_demo.py` + `DockingDemo`
- `VisionCaptureDemo`는 `RETREAT -> SUCCESS`에서 종료한다.
- `DockingDemo`는 처음부터 GT 기반 배치와 MEP 접근·부착 단계를 다시 수행한다.

따라서 현재 저장소에는 **“AprilTag로 유영 MEP 포획 후 같은 실행에서 운반·Client 도킹”을 수행하는 단일 진입점이나 hand-off 코드가 없다.** 이것이 월요일 시연의 최우선 차단 요소다.

### 3.4 현재 고정·동역학 상태

| 대상 | 정확한 현재 상태 | 파일·설정 | 해석 |
|---|---|---|---|
| 월드 | `Domain.ORBIT`, gravity magnitude 0 | `task.py:146-149`, `core/domain.py:24-41`, `env_cfg.py:171-182` | 무중력일 뿐 실제 중심중력/궤도역학은 없음 |
| MRV 외형 | `VenusExpress` scenery를 `AssetBaseCfg`로 바꾸며 `articulation_enabled=False`, `rigid_body_enabled=False`; collision도 task에서 끔 | `env_cfg.py:475-486`, `asset.py:289-323`, `task.py:151-166` | 정적 배경이며 자유 유영 MRV 강체가 아님 |
| Canadarm3 base | USD의 `root_joint`가 body0 없는 world-fixed `PhysicsFixedJoint` | `assets/robot/manipulation/canadarm3.py:23-49` | 로봇 base는 world 원점 고정 |
| MEP | dynamic `RigidObject`, explicit mass 3000 kg, default v=0/w=0 | `task.py:219-252` | kinematic은 아니며 동적 강체 |
| Vision MEP | 위 MEP에 0.01 m/s, direction `[0,-0.8,0.6]`, w=0 적용 | `vision_capture.yaml:9-19`, `vision_task.py:52-59,149-160` | 선형 자유 유영만 구현 |
| Client 위성 | dynamic `RigidObject`, density 1000 kg/m³, v/w 미지정(기본 0) | `task.py:67-104` | 실제 총질량·관성은 런타임 측정 기록 없음 |
| EE↔MEP | 조건 통과 후 `/capture_joint` FixedJoint, articulation에서 제외 | `capture.py:299-369` | MEP는 팔에 물리 구속됨 |
| MEP↔Client | 조건 통과 후 `/docking_joint` FixedJoint | `docking.py:418-468` | 두 동적 body가 물리 구속됨 |

결론: MEP는 이미 선형 자유 유영이지만 MRV는 자유 유영이 아니며, Client는 동적이지만 궤도 이탈 운동·명시적 질량/관성/속도 설정이 없다.

### 3.5 ML 상태

현재 규칙 기반 baseline은 실제 코드다.

1. AprilTag/PnP로 `Cylinder_01` 6-DoF 추정
2. 최근 2 s 위치에 최소제곱 직선 적합
3. `p(t+h)=p(t)+v·h`, `h=0.3 s`
4. 회전은 5 s 창 평균이며 미래 회전을 외삽하지 않음

`docs/mrv_phase2_ml_pose_estimation.md`는 다음을 설계했지만 추적되지 않은 기술 문서일 뿐이다.

- 관측 조건별 PnP 불확실성 추정
- 태그 소실 구간 시계열 예측
- 회전·비선형 유영 예측
- NPZ 데이터, EKF, GRU, 3-way 평가

저장소에는 `collect_dataset.py`, `build_dataset.py`, `train_predictor.py`, `eval_predictors.py`, `vision_predictors.py`, 모델 `.pt`, 데이터 `.npz`가 없다. 따라서 ML 데이터 수집·학습·런타임 적용·비교 평가는 **미착수 0%**다.

## 4. 완료 / 구현됐지만 미검증 / 미구현 / 불명확 구분

| 구분 | 항목 | 근거 또는 이유 |
|---|---|---|
| 완료(조사 기준) | 브랜치·HEAD·작업트리 확인, 사용자 untracked 파일 보존 대상 식별, 핵심 Python AST 확인 | 현재 조사 명령 |
| 구현됐지만 미검증 | AprilTag/PnP, 선형 속도 추정, 예측 접근, FixedJoint capture, hold/retreat | 코드와 `923e379`; 현재 실행 로그 없음 |
| 구현됐지만 미검증 | 고정 배치 MEP 포획·운반·Client 정렬·도킹·3 s hold·undock | 코드와 `5b7e55b`; 문서상 headless 결과만 있고 현재 원시 로그 없음 |
| 미구현 | AprilTag 포획에서 운반/Client 도킹으로 이어지는 한 실행 E2E hand-off | 서로 다른 스크립트·상태 머신 |
| 미구현 | 자유 유영 MRV, 명시적 Client 이탈 운동, 공통 seed 설정 | MRV static/fixed, Client v=0 |
| 미구현 | 빨간 기준 원궤도, 최근접점, 이탈 판정 | 대상 경로에 관련 구현 없음 |
| 미구현 | 이탈 Client 자동 선택 | 현재 Client 하나를 고정 배치 |
| 미구현 | 도킹 후 궤도 복귀 제어·유지 판정 | `DOCKED` 이후 hold/undock만 존재 |
| 미구현 | ML 데이터·학습·추론·비교 평가 | 기술서만 존재 |
| 불명확 | 현재 GPU PhysX에서 런타임 FixedJoint가 동일하게 반영되는지 | 과거 이슈 문서가 별도 확인 필요를 명시 |
| 불명확 | Client 실제 총질량·관성, MEP 관성 텐서 | Client는 density 기반, 런타임 측정 로그 없음 |
| 불명확 | 전체 GUI 흐름 wall time과 발표 시간 적합성 | 도킹 문서상 약 516 simulated s; 전체 GUI 완주 기록 없음 |

사용자에게 보여 줄 표현은 다음처럼 제한한다.

- 허용: “AprilTag 포획 코드와 별도 도킹 코드가 구현되어 있다.”
- 금지: “AprilTag 포획부터 Client 도킹까지 통합 검증됐다.”
- 허용: “도킹 문서에 과거 headless 25/25 PASS 기록이 있다.”
- 금지: “현재 HEAD에서 GUI 전체 시연이 검증됐다.”

## 5. 월요일 1차 시연 범위와 제외 범위

### 5.1 Must — 시연 필수 경로

```text
씬 시작
 -> AprilTag 인식
 -> 유영 MEP 포즈/속도 추정 및 예측
 -> Canadarm3 접근
 -> capture 조건 통과
 -> EE↔MEP FixedJoint
 -> MEP 운반
 -> Client dock frame 정렬
 -> probe 삽입 조건 통과
 -> MEP↔Client FixedJoint
 -> 도킹 3 s 이상 유지
```

완료 게이트:

1. 하나의 명령과 하나의 프로세스로 실행된다.
2. 태그 인식부터 `DOCKED`까지 상태가 화면과 console에 보인다.
3. 성공을 만들기 위한 MEP/Client pose 순간이동이나 매 프레임 pose 강제가 없다.
4. capture·dock FixedJoint는 기존 위치·자세·상대속도·접근방향 조건을 통과한 뒤에만 생성된다.
5. 실패하면 관절 hold + 추진 명령 0으로 안전 정지하고 원인을 출력한다.
6. 같은 commit/config/seed로 GUI **2회 연속 성공**하고 영상·console·result JSON을 보존한다.
7. 2회 연속 성공이 안 되면 준비율을 100%라 부르지 않는다. 성공 영상 1개와 실패 로그를 준비하고 라이브 위험을 명시한다.

### 5.2 월요일 제외 범위

- ML predictor, 학습·추론
- 회전/tumbling MEP
- 실제 중심중력 기반 궤도역학
- MRV 전체 자유 유영 articulation 전환
- 도킹 후 실제 force/thrust 궤도 복귀
- 다수 Client 중 자동 선택
- 신규 센서·신규 의존성·대규모 추상화

### 5.3 범위 동결 기준

- **9/20 20:00 KST 또는 시연 14시간 전 중 빠른 시점**에 기능 범위를 동결한다.
- 그때 단일 E2E가 한 번도 성공하지 않았다면, 월요일 라이브에서 “통합 완료”를 주장하지 않는다.
- 차선책은 AprilTag 포획과 기존 도킹을 **두 개의 별도 검증 시나리오**로 보여 주고, 둘 사이 hand-off가 남았다고 밝히는 것이다. 두 영상을 편집해 한 E2E인 것처럼 보이게 하지 않는다.

## 6. 월·화·수 시간대별 일정

담당자는 이름이 정해지지 않았으므로 역할/대상으로 표기한다. 각 블록 시작 전에 실제 담당자 한 명을 지정한다.

### 6.1 9/21(월) — 1차 시연 안정화

| 시간(KST) | 우선순위 | 작업 | 선행조건 | 담당 대상 | 산출물 | 완료 조건 | 실패 시 대안 |
|---|---|---|---|---|---|---|---|
| 09:20-09:35 | Must | 로그·영상 경로, 디스크, 화면 overlay, 실패 메시지 확인 | smoke 결과 | 운영 | artifact index | 상태/오차/실패원인이 읽힘 | console tail 창을 별도 화면에 배치 |
| 09:35-09:50 | Must | 범위 동결, GUI 최종 리허설 또는 녹화본 재생 확인 | 통합 smoke PASS | 발표/운영 | 승인 체크리스트 | 새 기능 변경 0, 재현 명령 복사 완료 | live 대신 성공 녹화본 사용 |
| 10:00-10:15 | Must | 1차 시연 | freeze 승인 | 발표자 | live/recording | 필수 흐름 또는 정직한 분리 시연 | ABORT 후 녹화 전환 |
| 10:30-11:30 | Must | 실패·경고·wall time 분석 | 시연 로그 | 통합 | issue list | 각 이슈 owner/재현/심각도 있음 | 재현 불가 이슈는 관측 부족으로 분류 |
| 13:00-14:00 | Should | 2회 연속 GUI 회귀 또는 실패 재현 | 오전 환경 유지 | QA/운영 | run A/B artifact | 동일 commit/config/seed 2회 결과 | 1회 성공+1회 실패면 85% 상한 |
| 14:00-15:30 | Should | 화·수 이슈를 Must/Should/Could로 재분류 | 오전 결과 | 리드 | backlog | 차단관계·완료조건 존재 | 궤도 복귀보다 자유 유영/측정 우선 |
| 15:30-17:30 | Should | 화요일 fixed/free-flight 조사 스파이크 계획 확정 | backlog | 물리/제어 | object matrix | MRV 표현 방식 결정 필요사항 목록화 | 상대운동 대체안 채택 |

월요일에는 신규 ML·tumbling·궤도 복귀를 넣지 않는다. 오전 시연 직전 수정은 재현 명령, 로그 경로, 명백한 안전 guard로 제한한다.

### 6.2 9/22(화) — 자유 유영과 궤도 시각화

| 시간(KST) | 우선순위 | 작업 | 선행조건 | 담당 대상 | 산출물 | 완료 조건 | 실패 시 대안 |
|---|---|---|---|---|---|---|---|
| 09:00-10:00 | Must | MRV/MEP/Client prim, rigid/articulation API, root joint, mass/inertia/collider 런타임 덤프 | 월요일 기준 commit | 물리 | `body_inventory.json` | 모든 대상의 body path·동적 여부·mass·inertia·v/w 기록 | 값 누락 시 INIT 실패 처리 |
| 10:00-11:30 | Must | MRV 표현 결정: full free-floating vs 상대운동 시연 | inventory | 물리/제어 | ADR 1장 | 제어 주체·base frame·FixedJoint 영향 합의 | **공통 궤도는 시각화, 상대 운동만 물리 적용** 채택 |
| 11:30-12:30 | Must | MEP/Client 초기 v/w, damping, mass/inertia, seed를 config로 이동 | 표현 결정 | 물리 | config diff 계획/구현 | reset 2회 동일 초기 상태 | explicit velocity만 주고 inertia는 측정값 고정 |
| 13:30-14:30 | Must | 빨간 기준 원궤도 생성 | orbit center/normal/radius 결정 | 시각화 | red polyline | 128개 이상 sample, scene reset 후 동일 | viewport overlay로 임시 표시 |
| 14:30-15:30 | Must | 최근접점·반경/평면/속도 오차 계산 | orbit frame | 궤도 수학 | telemetry | analytic unit test + 화면 수치 일치 | offline 계산 CSV만 우선 |
| 15:30-16:30 | Must | 이탈 Client 판정과 deterministic seed | metrics | 임무 상태 | selection log | 임계값+유지시간으로 동일 Client 선택 | Client 1개 고정, “자동 선택 미구현” 표시 |
| 16:30-17:30 | Should | FixedJoint 전후 안정성, collision/NaN/각속도 safety smoke | free-flight config | QA/물리 | headless logs | 발산 0, reset 재현 | MRV 고정 유지, MEP/Client 상대운동만 적용 |
| 17:30-18:00 | Must | 일일 진행률·리스크 갱신 | 당일 artifacts | 리드 | daily log | 근거 링크와 점수 재계산 | 실패 항목 30% 유지 |

### 6.3 9/23(수) — 도킹 후 궤도 복귀와 다음 단계

| 시간(KST) | 우선순위 | 작업 | 선행조건 | 담당 대상 | 산출물 | 완료 조건 | 실패 시 대안 |
|---|---|---|---|---|---|---|---|
| 09:00-09:45 | Must | 이탈 Client 선택 결과를 접근 대상에 연결 | 화요일 detector | 임무/통합 | target binding log | 선택된 client ID와 dock frame 일치 | 단일 Client 명시 선택 |
| 09:45-11:30 | Must | MEP–Client 정렬·도킹을 자유 유영 조건에서 재검증 | target binding, 안정 물리 | 제어 | docking artifact | 기존 모든 gate 통과, teleport 0 | 상대 운동 정지 seed로 축소 |
| 11:30-12:30 | Must | 도킹 후 복합체 mass/inertia 확인과 recovery 제어 주체 확정 | dock PASS | 물리/제어 | composite telemetry | joint 유효, 질량·COM·속도 finite | MRV fixed + Client velocity controller 데모로 축소·명시 |
| 13:30-15:00 | Must | 정상 궤도 복귀 제어 | orbit metrics | 제어 | recovery log | 위치/속도 오차 감소, overshoot 제한 | 속도 목표 램프만 적용, force controller는 후속 |
| 15:00-15:30 | Must | `VERIFY_ORBIT` 유지 시험 | recovery | QA | verdict JSON | 모든 임계값 5 s 연속 만족 | hold 실패 시 recovery 60% 상한 |
| 15:30-16:30 | Must | 전체 headless 회귀 | 모든 Must | QA | regression bundle | seed 3개, crash/NaN 0 | 각 단계 분리 회귀 + 실패 state 저장 |
| 16:30-17:00 | Should | ML 수집 접합점·baseline 비교 명령 확정 | 안정 baseline | ML/비전 | schema + eval matrix | raw PnP/GT/seed schema 승인 | 구현은 다음 sprint로 이월 |
| 17:00-17:30 | Should | 추가 로봇팔 기능 Must/Should/Could 분류 | 회귀 결과 | 로봇/리드 | backlog | owner/조건/목표일 있음 | 비필수 기능 전부 Could |
| 17:30-18:00 | Must | 최종 진척률·기술부채·rollback 기록 | artifacts | 리드 | daily/final log | 증거 수준별 점수 갱신 | 미검증 항목은 60% 이하 유지 |

## 7. 목표 아키텍처와 상태 머신

### 7.1 좌표계 관계

```text
W: demo inertial world
└─ O: orbit reference {center C, plane normal n, radius R, direction sign}
   ├─ B: MRV/Canadarm3 base
   │  └─ L: Canadarm3 flange link
   │     ├─ C: cam_wrist optical frame
   │     └─ E: EE contact frame
   ├─ M: MEP body
   │  ├─ T: AprilTag constellation
   │  │  └─ Y: Cylinder_01 physical capture point
   │  └─ P: probe tip/dock frame
   └─ S: Client body
      └─ D: Client thruster dock frame
```

현재 재사용 가능한 변환:

- 비전: `T_W_Y = T_W_L · T_L_C · T_C_T(PnP) · T_T_Y`
- 포획 목표: 예측한 `T_W_T`에서 face normal 반대 방향으로 EE frame 구성
- 도킹: `T_W_P`와 `T_W_D`의 축방향·반경·축/roll 오차 계산
- 순수 frame 연산: `project/srb/tasks/manipulation/debris_capture/frames.py`

새 `O` frame은 실제 지구 중력 모델이 아니라 **축척 시연용 기준 원**이다. W에서의 강체 물리와 O 기준 궤도 오차를 섞지 않는다.

### 7.2 MRV·MEP·Client별 변경 방식

| 대상 | 월요일 | 화·수 목표 | 성공 판정에 금지할 방식 |
|---|---|---|---|
| MRV/Canadarm3 | 현재 world-fixed base를 유지해 검증된 IK·도킹을 보호 | 먼저 static scenery와 arm root를 하나의 MRV 동역학 모델로 바꿀 수 있는지 inventory/스파이크. 불안정하면 MRV는 기준 frame으로 유지하고 공통 궤도 성분은 시각화만 함 | world-fixed arm을 자유 유영 MRV라고 부르기, 매 프레임 arm root pose를 움직여 물리 복귀라고 부르기 |
| MEP | 현재 dynamic 3,000 kg, 0.01 m/s, w=0 선형 유영 유지 | mass뿐 아니라 양의 inertia tensor, damping, v/w, seed를 명시하고 reset에서 1회 적용. 포획 후 기존 EE↔MEP FixedJoint 유지 | capture gate 전에 pose snap, 포획 뒤 joint 없이 EE pose에 종속 |
| Client 위성 | 현재 고정 배치·v=0의 동적 body 유지 | density 의존을 없애고 runtime 측정으로 확정한 mass/inertia를 config화. 접선/반경/법선 초기속도로 이탈 상태를 만들고 선택·도킹·recovery 동안 rigid dynamics 유지 | 도킹 직전 dock frame에 순간이동, recovery 중 매 프레임 기준원 위 pose 강제 |

full free-floating MRV를 선택할 때는 Canadarm3 내부 7개 관절은 articulation으로 유지하고, world-fixed `root_joint`만 제거/대체한다. 이 변경은 base motion을 반영하도록 IK 기준 `B`를 매 step 갱신하고, 기존 PLAN/reachability를 다시 통과하는 것이 선행조건이다.

### 7.3 빨간 궤도선

최소 구현은 debug draw polyline이다.

1. config에서 `C`, 단위 normal `n`, `R`, 진행 방향을 읽는다.
2. `n`에 수직인 정규직교 기저 `u`, `v`를 한 번 만든다.
3. `p_i=C+R(cosθ_i·u+sinθ_i·v)`, `i=0..127`을 계산한다.
4. 인접점을 빨강 `(1,0,0)` 선으로 연결하고 마지막과 처음을 닫는다.
5. orbit config가 바뀔 때만 점을 다시 계산한다. 매 프레임 다시 만드는 것은 필요 없다.

### 7.4 현재 위치의 기준 궤도 최근접점

물체 위치를 `p`, 속도를 `vel`이라 하면:

```text
q        = p - C
h        = dot(q, n)                     # 궤도 평면 밖 오차
q_plane  = q - h*n
r_hat    = q_plane / ||q_plane||
p_near   = C + R*r_hat
e_radius = ||q_plane|| - R
e_pos    = sqrt(e_radius^2 + h^2)
t_hat    = direction * cross(n, r_hat)
e_v_tan  = dot(vel, t_hat) - v_ref
v_radial = dot(vel, r_hat)
v_normal = dot(vel, n)
```

`||q_plane||`가 epsilon보다 작으면 최근접점이 유일하지 않으므로 `INVALID_ORBIT_GEOMETRY`로 ABORT한다. 이 계산은 위치 순간이동이 아니라 측정이다.

### 7.5 정상 궤도 판정 기준

아래 값은 **축척 시연용 초기 제안값**이며 실제 궤도역학 상수가 아니다. config knob로 유지하고 화요일 로그로 조정한다.

| 지표 | 초기 합격 기준 | 비고 |
|---|---:|---|
| 최근접점 위치 오차 `e_pos` | ≤ 0.10 m | 평면 밖·반경 오차 합성 |
| 접선 속도 오차 `|e_v_tan|` | ≤ 0.02 m/s | `v_ref`는 config 값 |
| 반경 방향 속도 `|v_radial|` | ≤ 0.01 m/s | 발산 방지 |
| 평면 법선 속도 `|v_normal|` | ≤ 0.01 m/s | 3D 이탈 방지 |
| 연속 유지 시간 | ≥ 5.0 s | 중간 한 프레임이라도 실패하면 timer reset |

### 7.6 임무 상태 머신 초안

```text
INIT
  -> FREE_FLIGHT
  -> DETECT_OFF_ORBIT_CLIENT
  -> TRACK_MEP
  -> CAPTURE_MEP
  -> APPROACH_CLIENT
  -> DOCK
  -> RECOVER_ORBIT
  -> VERIFY_ORBIT
  -> SUCCESS

모든 비종단 상태 -> ABORT
```

| 상태 | 입력 | 출력/전이 조건 | timeout·실패 |
|---|---|---|---|
| INIT | prim/API inventory, config, seed | finite mass/inertia, gravity 0, 필요한 frame 존재 | 값 누락 시 시작 금지 |
| FREE_FLIGHT | 초기 pose/v/w | 지정 seed와 허용 오차 내 초기 상태 | 고정 joint/kinematic 잔존 시 ABORT |
| DETECT_OFF_ORBIT_CLIENT | 각 Client `e_pos`, velocity errors | 유지시간을 넘겨 이탈한 Client 중 최대 score 선택 | 후보 없으면 safe wait |
| TRACK_MEP | AprilTag/PnP history | pose age ≤1 s, 속도 추정 유효 | tag loss >1 s면 hold/ABORT |
| CAPTURE_MEP | 예측 pose, 상대속도 | 기존 5개 capture gate 통과 + FixedJoint valid | timeout/접촉/발산 시 ABORT |
| APPROACH_CLIENT | P, D frames | 기존 pre-dock/insert gate 통과 | collision/축이탈 시 후퇴 후 1회 재시도 |
| DOCK | 도킹 metrics | MEP↔Client FixedJoint valid, 3 s 유지 | joint 미생성/드리프트 시 ABORT |
| RECOVER_ORBIT | composite state, orbit target | 오차가 단조 감소하거나 제한 내 | NaN/각속도/속도 상한 시 thrust 0 |
| VERIFY_ORBIT | four orbit metrics | 모두 5 s 유지 | 실패 시 RECOVER_ORBIT 1회, 이후 ABORT |

### 7.7 도킹 전후 질량·관성·제어 주체

- 현재 MEP mass는 3,000 kg로 명시됐지만 관성은 asset/collision에서 유도된다.
- Client는 density 1000 kg/m³만 명시되어 총질량·관성이 asset 형상에 의존한다.
- FixedJoint는 body를 하나로 합쳐 재작성하지 않는다. PhysX는 두 body와 joint constraint를 함께 푼다.
- 도킹 직후에는 MEP/Client 각각의 mass, inertia, COM, v/w와 joint 유효성을 다시 기록한다.
- recovery controller는 **한 제어 주체만** 가져야 한다. 3일 계획의 기본은 MRV/MEP 측 추진 주체 하나이며, Client와 동시에 힘을 주지 않는다.
- full free-floating MRV를 채택하지 못하면 “Client를 실제 MRV thrust로 복귀”라고 주장하지 않고, “상대 운동 복귀 controller 시연”으로 명시한다.

### 7.8 FixedJoint 생성·해제

- EE↔MEP 생성: pose age, 4/4 tags, velocity estimate, distance, angle, relative velocity, outside-face/lateral 조건을 모두 통과한 뒤 `CaptureManager.attach()` 한 곳에서 생성한다.
- MEP↔Client 생성: axial/radial/axis/roll/depth/approach 조건 통과 후 `DockingManager.dock()` 한 곳에서 생성한다.
- reset 전에 docking joint를 먼저, capture joint를 나중에 해제한다.
- ABORT에서 joint를 무조건 해제하지 않는다. 충돌 중 release는 물체를 튕길 수 있으므로 먼저 관절 hold·thrust 0, 접촉과 상대속도가 안전 범위일 때만 해제한다.

### 7.9 안전 처리

| 감시 항목 | 현재 재사용 | 추가 최소 조치 |
|---|---|---|
| NaN/Inf | vision demo의 body/joint finite check | orbit metrics와 controller output 포함 |
| 과속/과도 각속도 | MEP 0.5 m/s, 5°/s; joint 1 rad/s | Client/composite에도 동일 구조 적용, 임계값 config화 |
| 충돌 | arm contact 200 N, probe wall clearance | recovery 중 Client/MEP contact impulse 기록 |
| 태그 소실 | 1 s 후 `TAG_LOST`, 관절 hold | 1 s 이내는 마지막 유효 상태 예측, 초과 시 접근 금지 |
| FixedJoint 소실 | capture 후 joint validity 확인 | dock joint도 매 step 확인 |
| 발산 | 속도 기반 physics error | orbit error 연속 증가 N step 감시 |
| timeout | vision 60 s, docking state 240 s | mission state별 timeout과 원인 코드 |

### 7.10 구현 방식의 구분

| 방식 | 용도 | 장점 | 단점/금지선 |
|---|---|---|---|
| reset 시 1회 pose 배치 | 재현 가능한 초기조건 | 단순·결정론적 | 성공 직전 위치 보정에 쓰면 안 됨 |
| 매 프레임 pose 강제 | 시각화 전용 궤도 reference | 빠르고 안정 | 동적 물체 성공 판정에 쓰면 물리 시연 아님 |
| 초기 velocity 1회 설정 | 자유 유영 초기조건 | 실제 rigid dynamics 유지 | 제어가 아니므로 오차 복귀 불가 |
| velocity target/ramp | 축척 시연용 상대운동 복귀 | 안정·구현 작음 | 실제 추력/궤도역학으로 표현하면 안 됨 |
| force/torque/thruster 제어 | 최종 물리 복귀 | 운동량·질량 효과 반영 | MRV free base와 actuator 모델 필요, 3일 위험 큼 |

화요일 full free-floating MRV가 불안정하면 **공통 궤도 성분은 빨간 기준선과 reference transform으로만 표현하고, MEP/Client의 상대 운동만 동적 rigid body로 계산**한다. 이는 차선책이며 실제 궤도 복귀라고 부르지 않는다.

## 8. 최소 파일 변경 계획

이번 요청에서는 아래 파일을 **수정하지 않는다**. 실제 구현 시에도 기존 클래스를 재사용하고 새 의존성을 추가하지 않는다.

### 8.1 월요일 E2E 최소안

| 파일 | 최소 변경 | 재사용 이유 |
|---|---|---|
| `project/scripts/mrv_e2e_demo.py` 신규 | 같은 `srb/debris_capture_vision` env에서 비전 포획과 도킹 hand-off 실행 | 실행 명령 하나 제공 |
| `vision_capture_demo.py` | `HOLDING` 후 retreat 대신 hand-off 가능한 종료점/step 결과 노출 | 기존 PnP·접근·safety 유지 |
| `docking_demo.py` | 이미 capture된 MEP의 실제 `EE^-1·MEP` 관계로 `LIFT_MEP`부터 시작하는 진입 함수 추가 | 운반·정렬·삽입 로직 복사 방지 |
| `vision_capture.yaml` | demo seed, retry, artifact dir 등 꼭 필요한 값만 추가 | 새 설정 파일 확산 방지 |

새 base class, predictor factory, plugin, 외부 패키지는 월요일에 만들지 않는다.

### 8.2 화·수 최소안

| 파일 | 계획 | 비고 |
|---|---|---|
| `project/config/mrv_mission.yaml` 신규 | orbit C/n/R/v_ref, body v/w, seed, recovery thresholds | 여러 하드코딩을 막는 최소 config |
| `debris_capture/mrv_mission.py` 신규 | 상위 상태 머신, orbit math, selection/recovery | 임무 전용 책임이므로 분리 가치 있음 |
| `mrv_e2e_demo.py` | mission config와 artifact CLI 연결 | 단일 명령 유지 |
| 기존 `capture.py`, `docking.py`, `frames.py` | 원칙적으로 변경 없음 | 검증된 joint/frame 재사용 |

## 9. 1차 시연 준비율

산식: `Σ(가중치 × 증거수준)`이며, 가중치는 합계 100%다.

| 기능 | 가중치 | 상태 | 증거 수준 | 획득 점수 | 근거 파일/커밋/실행 로그 | 남은 작업 | 목표일 | 위험 |
|---|---:|---|---:|---:|---|---|---|---|
| 실행 환경 및 재현 명령 | 10% | 구현됐지만 현재 PC 미검증 | 60% | 6.0 | 두 실행 script, docs; 현재 로그 없음 | 데모 PC preflight와 artifact 명령 고정 | 9/21 | checkout/import 경로 불일치 |
| AprilTag 인식·추적·예측 | 25% | 구현 및 정적 확인 | 60% | 15.0 | `vision.py`, `vision_task.py`, `923e379` | 현재 HEAD headless + GUI 반복 | 9/21 | 카메라/PnP 재현성 |
| MEP 접근·포획·유지 | 25% | 구현 및 정적 확인 | 60% | 15.0 | `vision_capture_demo.py`, config | 실제 10 s hold, joint artifact | 9/21 | 3 t payload 진동, GPU joint |
| MEP 운반·Client 정렬·도킹 | 25% | 별도 시나리오 구현, hand-off 없음 | 60% | 15.0 | `docking_demo.py`, `5b7e55b`, 과거 결과 문서 | 비전 capture에서 same-run hand-off | 9/21 | 통합 시작상태/IK 경로 |
| GUI 리허설·로그·녹화·발표 체크리스트 | 15% | 작업 중 | 30% | 4.5 | GUI smoke 기록과 문서만 있음 | GUI 2회, 영상·로그·checklist | 9/21 | 약 9분 경로, 시간 초과 |
| **합계** | **100%** |  |  | **55.5%** |  |  |  |  |

목표:

| 시점 | 목표 준비율 | 달성 조건 |
|---|---:|---|
| 현재 | 55.5% | 정적 근거만 반영 |
| 9/21 시연 전 | 100% | 단일 GUI E2E 2회 연속 + artifact 보존 |
| 통합 headless만 성공한 경우 | 최대 85% | GUI 반복이 없으므로 100% 금지 |
| separate demo만 성공한 경우 | 60% 이하 유지 | 필수 hand-off 미완료를 명시 |

## 10. 전체 프로젝트 진행률

| 기능 | 가중치 | 상태 | 증거 수준 | 획득 점수 | 근거 파일/커밋/실행 로그 | 남은 작업 | 목표일 | 위험 |
|---|---:|---|---:|---:|---|---|---|---|
| AprilTag 기반 MEP 포획 | 15% | 구현됐지만 현재 미검증 | 60% | 9.0 | `vision*.py`, config, `923e379`; 원시 로그 없음 | 현재 HEAD GUI 반복 | 9/21 | PnP/렌더 차이 |
| MEP 운반 및 Client 도킹 | 15% | 별도 시나리오 구현 | 60% | 9.0 | `docking*.py`, `5b7e55b`, 과거 문서 | same-run hand-off·GUI 반복 | 9/21 | carry 진동/긴 실행시간 |
| MRV/MEP/Client 자유 유영 전환 | 15% | MEP만 부분 구현 | 30% | 4.5 | MEP dynamic drift; MRV static/fixed, Client v=0 | MRV 표현 결정, 명시 mass/inertia/v/w | 9/22 | robot root/articulation 불안정 |
| 빨간 궤도 시각화 및 이탈 판정 | 10% | 미착수 | 0% | 0.0 | 관련 구현 없음 | orbit frame/line/metrics/threshold | 9/22 | 축척과 실제 궤도 혼동 |
| 이탈 Client 자동 선택·접근·도킹 | 15% | 고정 Client 접근만 존재 | 30% | 4.5 | 기존 dock frame/접근 로직 | off-orbit selection과 target binding | 9/23 | moving target IK |
| 도킹 후 정상 궤도 복귀 | 15% | 미착수 | 0% | 0.0 | `DOCKED` 후 hold/undock뿐 | controller + verify hold | 9/23 | 복합체 발산/제어 주체 불명 |
| ML 데이터·학습·추론·비교 평가 | 10% | 기술서만 존재 | 0% | 0.0 | untracked ML 문서, 구현 파일 없음 | 데이터 schema부터 구현 | 다음 sprint | baseline보다 악화 가능 |
| 추가 로봇팔 기능 및 전체 회귀 | 5% | 부분 시험 코드만 존재 | 30% | 1.5 | phase1 tests, docking checks | 통합 회귀와 artifact 정책 | 9/23 | 테스트 시간 |
| **합계** | **100%** |  |  | **28.5%** |  |  |  |  |

일별 목표는 낙관적 “코드 완료”가 아니라 해당 날짜에 확보할 증거 수준으로 계산한다.

| 시점 | 전체 목표 | 주요 상승 근거 |
|---|---:|---|
| 현재 | 28.5% | 정적 구현 근거 |
| 9/21 종료 | 42.0% | AprilTag·도킹 GUI E2E 100%, 통합 회귀 60% |
| 9/22 종료 | 58.75% | 자유 유영·orbit 시각화/판정 headless 85% |
| 9/23 종료 | 80.25% | 자동 선택/도킹 85%, recovery 구현·정적 확인 60%, 전체 회귀 85%, ML 접합점 30% |

9/23의 100%는 현실적 약속이 아니다. recovery GUI 반복과 ML 학습/평가, 추가 로봇팔 범위까지 끝나야 100%다.

## 11. 남은 퍼센티지별 작업 계획

현재 전체 진행률 28.5%이므로 남은 점수는 71.5점이다.

| 우선도 | 기능 | 현재→최종 증거 | 남은 점수 | 구체 작업 |
|---|---|---:|---:|---|
| Must | AprilTag 포획 | 60→100 | 6.0 | 현재 HEAD GUI 반복·artifact |
| Must | 운반·Client 도킹 | 60→100 | 6.0 | same-run hand-off·GUI 반복 |
| Must | 자유 유영 | 30→100 | 10.5 | MRV 표현, 명시 물성/v/w, seed, GUI |
| Must | 궤도 시각화/판정 | 0→100 | 10.0 | red line, nearest point, 4개 지표, 유지시간 |
| Must | 이탈 Client 선택·접근·도킹 | 30→100 | 10.5 | selector, moving target binding, E2E |
| Should | 궤도 복귀 | 0→100 | 15.0 | controller, composite safety, verify hold |
| Should | ML | 0→100 | 10.0 | 수집, split, EKF/ML, inference, 3-way 평가 |
| Should/Could | 추가 팔 기능·전체 회귀 | 30→100 | 3.5 | seed matrix, GUI 회귀, backlog 기능 |
| **합계** |  |  | **71.5** |  |

범위 분류:

- **Must:** 월요일 single-run E2E, 화요일 물성/자유유영·orbit 측정, 수요일 target binding·dock·기본 recovery verdict.
- **Should:** force/velocity recovery 안정화, seed 3개 회귀, 데이터 수집 schema와 baseline 비교 harness.
- **Could:** GRU 학습/배포, 복잡한 tumbling, 실제 중심중력/궤도역학, full free-floating arm base, 다중 Client, 추가 로봇팔 동작.

추가 로봇팔 기능은 다음처럼 분류한다.

| 등급 | 기능 | 완료 조건 |
|---|---|---|
| Must | 비전 포획 후 실제 상대 pose를 유지한 운반 hand-off | joint를 재생성/pose snap하지 않고 `LIFT_MEP` 진입 |
| Must | 상태별 safe hold와 1회 제한 재시도 | 실패 원인 출력, 속도 명령 0, 동일 실패 반복 시 ABORT |
| Must | dock 후 하중을 든 자세 유지 | 3 s 이상 joint·축 오차 기준 통과 |
| Should | 충돌 없는 후퇴·undock·home 복귀 | 순차 joint 해제 후 접촉/발산 없이 지정 자세 도달 |
| Should | moving Client에 대한 predock 목표 갱신 | stale target 거부, 속도/축 오차 gate 유지 |
| Could | tumbling MEP 회전 동기화 포획 | 1~3°/s 조건의 반복 GUI 성공 |
| Could | 자유 유영 base–arm 협조 제어 | base momentum과 arm motion을 함께 제한하는 물리 검증 |
| Could | 다중 팔/도구 교환/복수 Client | 별도 요구사항과 독립 acceptance test가 생길 때만 착수 |

## 12. 테스트 및 합격 기준

### 12.1 월요일 E2E

| 구간 | 합격 기준 | 필수 증거 |
|---|---|---|
| 시작 | commit/config/seed/import path 기록 | preflight text |
| AprilTag | 4/4 tag, reprojection RMS ≤1.5 px, pose age ≤1 s | overlay + CSV |
| 예측 | velocity 존재, plausible speed ≤0.2 m/s | telemetry |
| 포획 | distance ≤0.15 m, angle ≤5°, rel speed ≤0.05 m/s, lateral ≤0.02 m, outside-face | gate log |
| 유지 | FixedJoint 10 s, relative drift ≤5 mm/0.5° | result JSON |
| 운반 | MEP–EE 상대 pose 변화 ≤5 mm/0.5° | transport log |
| 도킹 | axial/radial ≤10 mm, axis ≤0.5°, roll ≤1°, depth >0, approach ≤3° | gate log |
| 유지 | dock joint valid, 3 s 뒤 axial/radial ≤10 mm | result JSON |
| 반복 | 동일 commit/config/seed GUI 2회 연속 | run A/B video+console |

### 12.2 화요일 자유 유영/궤도

1. INIT inventory에 MRV/MEP/Client의 rigid/articulation/kinematic/fixed-joint 상태와 mass/inertia가 모두 기록된다.
2. 선택한 동적 물체는 성공 판정 구간에서 per-frame pose write를 사용하지 않는다.
3. reset 직후 초기 v/w가 config와 허용 오차 내 일치한다.
4. 동일 seed 두 번의 초기 상태와 첫 5 s telemetry가 허용 오차 내 재현된다.
5. analytic circle point 8개에서 nearest-point 오차가 수치 epsilon 이하다.
6. 빨간 원, 최근접점, `e_pos/e_v_tan/v_radial/v_normal`이 화면과 CSV에 동시에 나온다.
7. threshold 밖 Client만 `OFF_ORBIT`로 판정된다.

### 12.3 수요일 recovery

1. 선택된 Client의 `D` frame으로만 접근한다.
2. 도킹 전후 body pose를 성공시키기 위해 순간이동하지 않는다.
3. recovery command는 acceleration/velocity/force 상한과 rate limit을 가진다.
4. collision, NaN, joint loss, speed/angle limit 초과 시 command 0 + ABORT가 된다.
5. `e_pos≤0.10 m`, `|e_v_tan|≤0.02 m/s`, `|v_radial|≤0.01 m/s`, `|v_normal|≤0.01 m/s`를 5 s 유지한다.
6. seed 3개 headless regression에서 crash/NaN이 없고 결과가 artifact로 남는다.

### 12.4 ML 데이터·비교 기준

월요일 필수 경로와 분리한다.

| 항목 | 정의 |
|---|---|
| 입력 | camera-relative PnP pose, rot6d, reprojection RMS, inlier/tag 수, ambiguity, 거리/시선각, dt, valid mask |
| GT | `Cylinder_01` world pose, MEP linear/angular velocity, 0.1/0.3/1.0 s 미래 pose |
| 시나리오 | linear speed, angular speed, drift/rotation axis, tag dropout, distance, noise, seed |
| split | scenario/seed 단위 70/15/15; 같은 궤적 window가 split을 넘지 않음 |
| 비교 | 동일 seed·동일 image/GT에서 B0 raw PnP, B1 LinearDriftPredictor, B2 EKF, ML |
| 지표 | position/angle median·p95, horizon error, dropout error, latency p95, capture success rate |
| 채택 | B2 대비 angle p95 20% 이상 개선, capture 성공률 비저하, latency <10 ms, fallback <1% |
| 중단 | 하나라도 못 맞추면 ML을 사용하지 않고 EKF/규칙 기반 유지 |

## 13. 리스크, 차선책, 롤백 기준

| 위험 | 가능성/영향 | 조기 신호 | 차선책 | 롤백/중단 기준 |
|---|---|---|---|---|
| AprilTag capture와 dock 상태 머신 hand-off 부재 | 매우 높음/치명 | 단일 명령 없음 | 이미 capture된 pose를 DockingDemo `LIFT_MEP`에 전달 | 9/20 freeze까지 1회 성공 없으면 분리 시연 |
| MRV가 static scenery + world-fixed arm | 높음/치명 | root joint 제거 후 IK/base 불안정 | 공통 궤도 시각화 + 상대 운동 물리 | base 발산/충돌이면 full free-floating 중단 |
| 현재 로그가 gitignored·소실 | 높음/높음 | PASS 주장만 있고 파일 없음 | `artifacts/demo_YYYYMMDD/`로 복사·manifest | artifact 없는 run은 85/100 인정 금지 |
| GPU PhysX 런타임 joint 반영 차이 | 중간/높음 | joint prim은 있으나 body가 안 따라옴 | 검증된 CPU pipeline | GPU에서 joint hold 실패 즉시 CPU 전환 |
| 3 t payload 진동·긴 wall time | 높음/높음 | tracking lag, 약 9분 simulated sequence | 고정 seed, 검증된 속도 유지, 녹화 | 시연 직전 속도 상향 금지 |
| Client mass/inertia 불명확 | 높음/높음 | recovery gain 민감·폭주 | runtime 측정값을 명시 config로 고정 | finite/positive 검사 실패 시 INIT 중단 |
| per-frame pose로 가짜 성공 | 중간/치명 | physics telemetry와 pose jump 불일치 | pose write audit log | 성공 구간 pose 강제 발견 시 결과 무효 |
| tag loss/렌더 차이 | 중간/높음 | pose age 증가, PnP RMS 급증 | 마지막 유효 예측 ≤1 s 후 hold | 1 s 초과 접근 금지 |
| scope 폭증 | 높음/높음 | ML/tumbling/orbit gravity 동시 변경 | Must만 유지 | 월요일 freeze 후 신규 기능 금지 |

롤백은 브랜치 전환이나 사용자 파일 삭제가 아니다. 기능 flag/config로 마지막 검증 경로를 선택하고 실패 코드는 보존한다.

## 14. 시연 당일 체크리스트

### 14.1 시작 전

- [ ] `git branch --show-current`가 `feature/apriltag`
- [ ] `git rev-parse HEAD`가 승인된 commit
- [ ] `git status --short`를 artifact에 저장하고 사용자 untracked 파일을 건드리지 않음
- [ ] `python -c "import srb; print(srb.__file__)"`가 이 checkout을 가리킴
- [ ] Isaac Sim/Isaac Lab/GPU/driver 기록
- [ ] config·seed·명령을 clipboard와 로컬 text에 보존
- [ ] 디스크 여유, 영상 저장, console tee, result dir 확인
- [ ] live run 전에 성공 녹화본 재생 가능 확인
- [ ] 시연 중 수정 금지와 ABORT 키/절차 공유

### 14.2 발표자가 읽을 3~5분 순서

1. **0:00-0:30 — 범위 선언**: “오늘은 축척 무중력 씬에서 AprilTag 기반 유영 MEP 포획부터 Client 도킹 유지까지 보여 줍니다. 실제 중심중력 궤도 복귀와 ML은 후속입니다.”
2. **0:30-1:15 — 인식/예측**: `SEARCH -> TAG_DETECTED -> POSE_ESTIMATED -> PREDICTING`과 4 tag, PnP RMS, 예측 marker를 가리킨다.
3. **1:15-2:00 — 접근/포획**: `APPROACHING -> SLOW_APPROACH -> CAPTURED`; 거리·자세·상대속도 gate가 모두 통과해야 FixedJoint가 생성됨을 말한다.
4. **2:00-3:15 — 운반/정렬**: MEP가 EE에 붙어 물리적으로 따라가며 `MOVE_TO_SAT_PREDOCK -> ALIGN_SAT`로 이동하는 것을 보여 준다.
5. **3:15-4:15 — 삽입/도킹**: probe axis/radial/roll/depth 수치가 기준을 통과한 뒤에만 MEP↔Client FixedJoint가 생김을 보여 준다.
6. **4:15-5:00 — 유지/증거**: `DOCKED`, hold timer, result 경로를 보여 주고 “순간이동이나 매 프레임 pose 강제 없이 성공했다”고 로그 근거와 함께 마무리한다.

> 실제 motion이 5분보다 길면 이 순서는 발표 멘트 순서다. 검증 없이 arm 속도를 올리지 않는다. 시간 제한이 엄격하면 검증된 전체 녹화본을 재생하고 live는 핵심 상태만 보여 준다.

### 14.3 실패 시

- [ ] 즉시 관절 hold, thrust/velocity command 0
- [ ] 마지막 state와 reason을 읽고 재시도는 최대 1회
- [ ] 같은 실패가 반복되면 live 종료, 녹화본 재생
- [ ] separate demo인 경우 통합 성공처럼 설명하지 않음
- [ ] 실패 console/result/영상도 삭제하지 않고 artifact에 보존

## 15. 매일 갱신할 진행 로그 템플릿

````markdown
# MRV Daily Progress — YYYY-MM-DD

## 기준
- 기준 시각(KST):
- 브랜치 / commit:
- config hash / seed:
- 실행 환경(Isaac Sim / Isaac Lab / GPU):

## 진행률
- 1차 시연 준비율: 시작 __% -> 종료 __%
- 전체 프로젝트 진행률: 시작 __% -> 종료 __%
- 점수 변경 근거:

## 완료 항목과 검증 증거
| 항목 | 계획/구현/자동시험/GUI 실측 | 결과 | artifact 경로 | evidence level |
|---|---|---|---|---:|
| | | | | |

## 실패 항목과 원인
| 명령/상태 | 관측 증상 | 직접 원인 | 근본 원인 가설 | 재현 여부 |
|---|---|---|---|---|
| | | | | |

## 새로 발견된 리스크
| 위험 | 가능성 | 영향 | owner | 완화/중단 조건 |
|---|---|---|---|---|
| | | | | |

## 다음 작업
1.
2.
3.

## 차단 요소와 필요한 결정
- 차단 요소:
- 결정권자:
- 결정 시한:
- 기본 대안:

## 실행 명령과 결과 로그
```bash
# exact command
```
- console:
- result JSON:
- metrics CSV:
- video:
- exit code:
- 예상 시간:
- 실제 wall time:
- simulated time:
````

## 16. 즉시 착수할 작업 상위 5개

1. **데모 PC preflight와 현재 HEAD의 두 기존 시나리오 재실행**: `srb.__file__`, 버전, GPU부터 기록하고 AprilTag dynamic과 docking headless의 원시 로그를 확보한다.
2. **비전 포획→도킹 hand-off 구현**: 같은 env에서 `VisionCaptureDemo`의 실제 EE–MEP 상대 pose를 `DockingDemo`의 `LIFT_MEP` 시작 상태에 전달한다. 새 IK/도킹 알고리즘은 만들지 않는다.
3. **단일 명령 E2E headless 1회 성공**: 상태 전이, capture/dock gate, exit code, result JSON을 한 artifact bundle에 남긴다.
4. **GUI 2회 연속 리허설과 녹화**: 같은 commit/config/seed로 실행하고, 성공 2회가 아니면 준비율 100%를 선언하지 않는다.
5. **MRV 표현 결정용 body inventory 생성**: VenusExpress static scenery, fixed Canadarm3 base, dynamic MEP/Client의 실제 mass/inertia/API를 덤프해 화요일 full free-floating 또는 상대운동 대체안을 결정한다.

가장 큰 위험은 “두 기능이 각각 성공했다”는 기록을 “단일 자유 유영 E2E가 성공했다”로 오인하는 것이다. 월요일의 성공 여부는 오직 같은 프로세스·같은 물리 씬·같은 artifact bundle의 `AprilTag -> CAPTURED -> DOCKED` 연속 기록으로 판정한다.
