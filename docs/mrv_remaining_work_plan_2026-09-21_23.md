# MRV 1차 시연 준비 및 2026-09-21~23 잔여 개발·진척률 관리 계획

## 1. 문서 목적과 기준 시각

- 목적: 2026-09-21(월) 1차 시연을 안정화하고, 9/21~9/23 잔여 개발을 파일·커밋·실행 결과에 따라 관리한다.
- 기준 시각: **2026-09-20 18:12 KST**
- 저장소: `/home/rokey/isaac_space`
- 기준 브랜치/커밋: `feature/apriltag` / `923e379924ca427a4972dbb9e70a12ec7c659a4c`
- 범위: 현재 저장소 정적 조사와 이번에 추가한 축척 기준 궤도선의 오프라인 시험까지다. Isaac Sim GUI E2E를 새로 실행한 결과가 아니다.

증거 수준은 다음처럼 고정한다.

| 증거 수준 | 값 | 판정 기준 |
|---|---:|---|
| 미착수 | 0% | 실행 코드 없음 |
| 작업 중 | 30% | 일부 구성요소만 존재하거나 통합 경로 없음 |
| 구현 및 정적 확인 | 60% | 구현과 구문·단위시험 확인, Isaac 실행 미확인 |
| headless/자동 시험 통과 | 85% | 현재 커밋·설정의 원시 로그와 exit code 보존 |
| GUI E2E 반복 및 결과 기록 | 100% | 같은 commit/config/seed로 반복 성공, 영상·로그 보존 |

과거 문서의 PASS와 커밋 제목의 “성공”은 참고 근거다. 현재 체크아웃의 원시 결과가 없으면 85%로 올리지 않는다.

## 2. 현재 저장소·브랜치·변경 상태

| 항목 | 확인 결과 | 처리 원칙 |
|---|---|---|
| 현재 브랜치 | `feature/apriltag` | 다른 브랜치로 전환하지 않음 |
| HEAD | `923e379` — `apriltag 방식 부착` | 진행률 기준점 |
| 관련 선행 커밋 | `5b7e55b` 도킹, `6e86135` FixedJoint 흡착 | 기능 존재 근거, E2E 증거는 아님 |
| 사용자 수정 파일 | `.gitignore`, `.vscode/settings.json`, Phase 1 문서와 vision 관련 파일 | 기존 변경을 보존하고 관련 위치에 최소 추가 |
| 사용자 미추적 작업 | `probe_dock*`, `gps_link.py`, `docs/md/` | 덮어쓰기·삭제하지 않음 |
| 이번 추가 파일 | `orbit_reference.py`, `test_orbit_reference.py`, 설계·구현 계획 문서 | 아직 커밋되지 않음 |
| 현재 Isaac 결과 로그 | 현재 체크아웃에서 GUI E2E 원시 로그 미확인 | 미검증으로 표기 |

최근 관련 커밋:

| 커밋 | 일시(KST) | 확인 가능한 의미 |
|---|---|---|
| `923e379` | 9/19 17:46 | AprilTag/PnP·선형 예측·비전 포획 코드 |
| `5b7e55b` | 9/18 18:38 | 별도 MEP 운반·Client 도킹 데모 |
| `13884f3` | 9/18 14:27 | MEP 3 t 설정 선행 이력 |
| `6e86135` | 9/18 12:20 | 흡착식 FixedJoint 시험 환경 선행 이력 |

## 3. 현재 구현 현황과 근거

### 3.1 AprilTag 기반 MEP 포획

실행 경로는 `project/scripts/vision_capture.py → VisionCaptureTask → VisionCaptureDemo`다. `vision.py`가 4개 AprilTag/16점 PnP, 최근 2초 최소제곱 속도, 0.3초 등속 위치 예측을 수행한다. `CaptureManager`는 위치·자세·상대속도·접근방향 gate 뒤 EE–MEP FixedJoint를 만든다.

- 설정: `project/config/vision_capture.yaml`
- 태그·카메라·초기 drift: `vision_task.py`
- 추정·baseline: `vision.py`
- 상태·안전·포획: `vision_capture_demo.py`, `capture.py`
- 판정: **구현 및 정적 확인 60%**, 현재 GUI 반복 로그 없음

### 3.2 MEP 운반·Client 도킹

`project/scripts/satellite_docking_demo.py → DockingDemo`에 고정 배치 MEP 접근, 포획, 운반, Client predock 정렬, probe 삽입, MEP–Client FixedJoint, 도킹 유지가 있다. `docs/satellite_docking_demo.md`에 과거 headless 25/25 PASS가 기록됐지만 현재 HEAD의 원시 로그와 전체 GUI 반복 기록은 없다.

- 도킹 frame·joint: `docking.py`
- 상태 머신·gate: `docking_demo.py`
- 판정: **구현 및 정적 확인 60%**

### 3.3 통합·자유 유영·궤도

- AprilTag 포획과 기존 도킹은 서로 다른 script/state machine이다. 같은 실행에서 `CAPTURED → 운반 → DOCKED`로 이어지는 hand-off는 확인되지 않았다.
- 월드는 `Domain.ORBIT`·중력 0이지만 실제 중심중력은 없다.
- MRV 외형은 static scenery이고 Canadarm3 root는 world-fixed다.
- MEP는 3,000 kg 동적 강체이며 비전 시나리오에서 `0.01 m/s`, 방향 `[0,-0.8,0.6]`, 각속도 0이다.
- Client는 동적 강체지만 density 기반이며 초기 속도·각속도는 0이다.
- 이번 변경으로 `center=(0,0,0)`, normal `+X`, radius `14 m`, 128점의 빨간 `UsdGeom.BasisCurves`가 추가됐다. 이는 YZ 평면의 **축척 시연용 시각 기준**이고 물체 제어가 아니다.
- 최근접점·속도 오차·OFF_ORBIT 판정은 아직 없다.

### 3.4 ML 범위

| 구분 | 현재 상태 | 근거/남은 일 |
|---|---|---|
| 규칙 baseline | 구현 | AprilTag/PnP, 선형 최소제곱 속도, 등속 미래 위치 |
| ML 후보 | 설계만 존재 | PnP 불확실성, tag-loss 시계열, 회전·비선형 예측 |
| 데이터 수집 | 미착수 | image/PnP/GT/seed schema와 수집 runner 필요 |
| 학습·추론 | 미착수 | dataset split, model artifact, runtime adapter 없음 |
| 비교 평가 | 미착수 | 같은 seed에서 baseline/EKF/ML 비교 필요 |

## 4. 완료 / 구현됐지만 미검증 / 미구현 / 불명확 구분

| 구분 | 항목 | 근거 |
|---|---|---|
| 완료(정적) | 빨간 기준 원의 수학·설정·USD authoring 코드 | `test_orbit_reference.py` 9개 통과, compileall 통과 |
| 구현됐지만 미검증 | AprilTag/PnP·선형 예측·EE–MEP FixedJoint | 코드/커밋 존재, 현재 GUI artifact 없음 |
| 구현됐지만 미검증 | MEP 운반·Client 정렬·도킹 | 별도 코드/과거 문서, current GUI E2E 없음 |
| 미구현 | AprilTag 포획→운반→Client 도킹 same-run hand-off | 분리된 두 상태 머신 |
| 미구현 | 자유 유영 MRV와 명시적 Client 이탈 운동 | MRV fixed, Client v/w 0 |
| 미구현 | 궤도 최근접점·이탈 판정·복귀 제어 | 시각선만 구현 |
| 미구현 | ML 데이터·학습·런타임·비교 | 기술 문서만 존재 |
| 불명확 | GPU PhysX FixedJoint 재현성과 Client 실제 mass/inertia | 현재 런타임 dump 없음 |

## 5. 월요일 1차 시연 범위와 제외 범위

필수 흐름:

```text
씬 시작 → AprilTag 인식 → 유영 MEP 추적/예측 → Canadarm3 포획
→ MEP 운반 → Client 정렬 → 도킹 → 도킹 유지
```

완료 조건:

1. 한 명령·한 프로세스로 재현하고 commit/config/seed를 기록한다.
2. `SEARCH`부터 `DOCKED`까지 상태와 gate 값을 화면·로그에 표시한다.
3. reset 초기 배치 외 성공 구간에서 pose 순간이동·매 프레임 pose write를 사용하지 않는다.
4. 위치·자세·상대속도·접근방향 조건 통과 뒤에만 FixedJoint를 만든다.
5. GUI 2회 연속 성공 또는 성공 녹화본 1개와 실패 원인 로그를 확보한다.

월요일 제외: ML, 실제 궤도 복귀, tumbling, full free-floating MRV, 실제 중심중력, 다중 Client 자동 선택. 시연 직전에는 범위를 동결한다.

## 6. 월·화·수 시간대별 일정

### 9/21(월) — 1차 시연 안정화

| 시간 | 우선 | 작업/선행조건 | 담당 대상 | 산출물·완료 조건 | 실패 시 대안 |
|---|---|---|---|---|---|
| 08:00–09:00 | Must | 환경 preflight, 두 기존 demo smoke | 운영/QA | 정확한 명령·버전·원시 로그 | import 실패면 검증된 PC/환경 사용 |
| 09:00–10:30 | Must | capture→dock hand-off, 기존 개별 PASS 선행 | 통합 | same-run `CAPTURED→DOCKED` 1회 | 두 기능을 별도 시연하고 통합 미완료 명시 |
| 10:30–12:00 | Must | headless smoke·실패 안전 정지 | QA/제어 | result JSON, exit code, ABORT reason | 고정 seed·저속 설정 |
| 13:00–15:00 | Must | GUI 리허설 2회·로그·녹화 | 발표/운영 | 2회 성공 또는 영상+실패 로그 | live 실패 즉시 녹화 전환 |
| 15:00–17:30 | Should | 시연 결과를 화·수 backlog로 재분류 | 리드 | owner·완료조건·목표일 | 신규 기능 금지, 범위 동결 |

### 9/22(화) — 자유 유영과 궤도 시각화

| 시간 | 우선 | 작업/선행조건 | 담당 대상 | 산출물·완료 조건 | 실패 시 대안 |
|---|---|---|---|---|---|
| 09:00–10:00 | Must | MRV/MEP/Client prim·fixed·mass/inertia dump | 물리 | `body_inventory.json`, finite/positive 값 | 누락 시 INIT 실패 |
| 10:00–11:30 | Must | full free base 여부 결정 | 물리/제어 | ADR, IK 기준 frame 결정 | 공통 궤도 시각화+상대운동 물리 |
| 11:30–13:00 | Must | v/w·damping·mass/inertia·seed config화 | 물리 | reset 2회 동일 초기 상태 | MRV fixed 유지, MEP/Client만 동적 |
| 14:00–15:00 | Must | 빨간 궤도 GUI smoke | 시각화 | reset 후 14 m YZ 원 유지 | debug overlay가 아닌 현재 USD curve 유지 |
| 15:00–17:00 | Must | 최근접점·4개 속도 지표·OFF_ORBIT 판정 | 임무/QA | 단위시험+CSV+화면 일치 | offline CSV 판정 우선 |

### 9/23(수) — 도킹 후 복귀와 다음 단계

| 시간 | 우선 | 작업/선행조건 | 담당 대상 | 산출물·완료 조건 | 실패 시 대안 |
|---|---|---|---|---|---|
| 09:00–10:00 | Must | 이탈 Client ID를 dock frame에 binding | 임무 | 선택 ID와 D frame 일치 | 단일 Client 명시 선택 |
| 10:00–12:00 | Must | moving Client 정렬·도킹 재검증 | 제어/QA | 기존 gate 통과, teleport 0 | 상대속도 낮은 seed로 축소 |
| 13:00–14:00 | Must | 도킹 후 mass/COM/inertia·joint 확인 | 물리 | finite composite telemetry | 복귀 시작 금지 |
| 14:00–15:30 | Must | 축척 상대운동 복귀 controller | 제어 | 제한 내 오차 감소 | velocity ramp만 사용, force는 후속 |
| 15:30–17:30 | Must | 5초 유지 판정·seed 3개 회귀·백로그 | QA/리드 | verdict와 regression bundle | 단계별 회귀, 실패 상태 보존 |

## 7. 목표 아키텍처와 상태 머신

### 7.1 좌표계

```text
W world
└─ O orbit reference: C, n, R, direction
   ├─ B MRV/Canadarm3 base
   │  └─ L flange/EE
   │     └─ M MEP (capture FixedJoint 이후 구속)
   │        └─ P probe
   └─ S Client body
      └─ D Client dock frame
```

월요일은 B가 world-fixed다. full free-floating으로 바꾸면 root joint 제거뿐 아니라 매 step 움직이는 B를 IK와 모든 frame 계산에 반영해야 한다.

### 7.2 빨간 기준 궤도와 최근접점

- 구현: `p_i=C+R(cosθ_i·u+sinθ_i·v)`, 128점, linear periodic `BasisCurves`, red RGBA.
- 현재 값: `C=(0,0,0) m`, `n=(1,0,0)`, `R=14 m`; 실제 지구 반경/중력을 뜻하지 않는다.
- 물체 위치 `p`, 속도 `vel`에 대해 `q=p-C`, `h=q·n`, `q_plane=q-hn`, `r_hat=q_plane/||q_plane||`다.
- 최근접점은 `p_near=C+R·r_hat`, 위치 오차는 `sqrt((||q_plane||-R)^2+h^2)`다.
- 접선은 `t_hat=direction·cross(n,r_hat)`이며 `e_v_tan=vel·t_hat-v_ref`, `v_radial=vel·r_hat`, `v_normal=vel·n`이다.
- `||q_plane||≤1e-9`이면 최근접점이 유일하지 않아 `INVALID_ORBIT_GEOMETRY`로 중단한다.

정상 궤도 초기 기준:

| 지표 | 기준 | 연속 조건 |
|---|---:|---:|
| 위치 오차 | ≤0.10 m | 모두 동시에 5.0 s |
| 접선 속도 오차 | ≤0.02 m/s | 모두 동시에 5.0 s |
| 반경 속도 | ≤0.01 m/s | 모두 동시에 5.0 s |
| 법선 속도 | ≤0.01 m/s | 모두 동시에 5.0 s |

### 7.3 상태 머신

```text
INIT → FREE_FLIGHT → DETECT_OFF_ORBIT_CLIENT → TRACK_MEP
→ CAPTURE_MEP → APPROACH_CLIENT → DOCK → RECOVER_ORBIT
→ VERIFY_ORBIT → SUCCESS
모든 비종단 상태 → ABORT
```

- `INIT`: prim/API/mass/inertia/frame 검증.
- `FREE_FLIGHT`: config v/w 적용 후 per-frame pose write 금지.
- `TRACK/CAPTURE`: 기존 AprilTag/PnP·LinearDriftPredictor·CaptureManager 재사용.
- `APPROACH/DOCK`: 기존 docking frame 계산·gate·DockingManager 재사용.
- `RECOVER/VERIFY`: 단일 제어 주체만 명령하고 4개 지표를 5초 유지한다.

### 7.4 도킹 전후 물리와 안전

- EE–MEP joint는 tag pose age, 거리, 자세, 상대속도, lateral/outside-face 조건 뒤 생성한다.
- MEP–Client joint는 axial/radial/axis/roll/depth/approach gate 뒤 생성한다.
- FixedJoint는 두 body를 하나로 재작성하지 않는다. 도킹 후 각 body의 mass/inertia/COM/v/w와 joint 유효성을 다시 기록한다.
- reset은 docking joint 후 capture joint 순서로 해제한다. 충돌 중 ABORT에서는 즉시 해제하지 않고 hold·명령 0 후 안전할 때 해제한다.
- NaN/Inf, collision impulse, joint loss, tag loss 1초 초과, 속도·각속도 상한, 오차 연속 발산 시 command 0과 원인 코드로 ABORT한다.

### 7.5 구현 방식 구분

| 방식 | 허용 용도 | 장점 | 제한 |
|---|---|---|---|
| reset 1회 pose 배치 | 재현 가능한 초기조건 | 결정론적 | 성공 직전 보정 금지 |
| 매 프레임 pose 강제 | 시각 reference만 | 안정적 | 동적 성공 판정에 사용 금지 |
| 초기 velocity 1회 | 자유 유영 초기조건 | rigid dynamics 유지 | 복귀 제어가 아님 |
| velocity target/ramp | 축척 상대운동 복귀 | 3일 내 안정화 가능 | 실제 궤도역학이라 부르지 않음 |
| force/torque | 후속 물리 복귀 | 질량·운동량 반영 | free base/actuator 모델 선행 |

## 8. 최소 파일 변경 계획

| 파일 | 최소 변경 | 상태 |
|---|---|---|
| `orbit_reference.py` | 궤도 config·점 생성·USD curve | 이번 변경 구현 |
| `vision.py`, `vision_capture.yaml` | orbit 설정 로드·검증 | 이번 변경 구현 |
| `vision_task.py` | scene setup에서 curve 1회 생성 | 이번 변경 구현 |
| `test_orbit_reference.py` | 오프라인 수학·검증 시험 | 이번 변경 구현 |
| `mrv_e2e_demo.py`, `mrv_mission.py` | hand-off와 상위 상태 머신 | 후속 최소안 |

기존 `CaptureManager`, `DockingManager`, frame helper, AprilTag estimator는 변경보다 재사용을 우선한다. 새 외부 패키지나 범용 base class는 제안하지 않는다.

## 9. 1차 시연 준비율

`획득 점수 = 가중치 × 증거 수준`이다.

| 기능 | 가중치 | 상태 | 증거 수준 | 획득 점수 | 근거 파일/커밋/실행 로그 | 남은 작업 | 목표일 | 위험 |
|---|---:|---|---:|---:|---|---|---|---|
| 실행 환경 및 재현 명령 | 10% | script 존재, 현재 PC 미검증 | 60% | 6.0 | 두 demo script, 원시 로그 없음 | preflight·단일 명령 | 9/21 | import/GPU 차이 |
| AprilTag 인식·추적·예측 | 25% | 구현·정적 확인 | 60% | 15.0 | `vision.py`, `923e379` | GUI 반복·artifact | 9/21 | 렌더/PnP 변동 |
| MEP 접근·포획·유지 | 25% | 구현·정적 확인 | 60% | 15.0 | `vision_capture_demo.py` | 10초 hold 실측 | 9/21 | 3 t 진동 |
| 운반·Client 정렬·도킹 | 25% | 별도 demo, hand-off 없음 | 60% | 15.0 | `docking_demo.py`, `5b7e55b` | same-run 연결 | 9/21 | IK 시작 상태 |
| GUI·로그·녹화·체크리스트 | 15% | 문서/부분 기록 | 30% | 4.5 | 과거 문서, current artifact 없음 | GUI 2회·영상 | 9/21 | 실행시간 |
| **현재 합계** | **100%** |  |  | **55.5%** |  |  |  |  |

목표: 9/21 시연 전 GUI single-run 2회면 100%, headless만이면 최대 85%, 분리 demo만이면 60% 이하로 유지한다.

## 10. 전체 프로젝트 진행률

| 기능 | 가중치 | 상태 | 증거 수준 | 획득 점수 | 근거 파일/커밋/실행 로그 | 남은 작업 | 목표일 | 위험 |
|---|---:|---|---:|---:|---|---|---|---|
| AprilTag 기반 MEP 포획 | 15% | 구현·현재 미검증 | 60% | 9.0 | vision 코드/`923e379` | GUI 반복 | 9/21 | PnP 차이 |
| MEP 운반·Client 도킹 | 15% | 별도 구현 | 60% | 9.0 | docking 코드/`5b7e55b` | hand-off | 9/21 | 긴 경로 |
| MRV/MEP/Client 자유 유영 | 15% | MEP만 부분 | 30% | 4.5 | MEP drift, MRV fixed | 물성/v/w/base 결정 | 9/22 | articulation 불안정 |
| 빨간 궤도 시각화·이탈 판정 | 10% | 선 구현, 판정 없음 | 30% | 3.0 | orbit 코드, 9 unit tests | GUI·nearest/off-orbit | 9/22 | 축척 혼동 |
| 이탈 Client 선택·접근·도킹 | 15% | 고정 Client 접근만 | 30% | 4.5 | 기존 docking frame | selector/binding | 9/23 | moving target |
| 도킹 후 정상 궤도 복귀 | 15% | 미착수 | 0% | 0.0 | 코드 없음 | controller/verdict | 9/23 | 복합체 발산 |
| ML 수집·학습·추론·평가 | 10% | 기술서만 | 0% | 0.0 | 구현 artifact 없음 | 전체 pipeline | 다음 sprint | baseline 악화 |
| 추가 팔 기능·전체 회귀 | 5% | 부분 시험 | 30% | 1.5 | 기존 tests | 통합 회귀 | 9/23 | 시험시간 |
| **현재 합계** | **100%** |  |  | **31.5%** |  |  |  |  |

| 시점 | 목표 진행률 | 상승 근거 |
|---|---:|---|
| 현재 | 31.5% | 궤도선은 부분 기능 30%만 반영 |
| 9/21 종료 | 45.0% | 포획·도킹 same-run GUI와 회귀 |
| 9/22 종료 | 58.75% | 자유 유영·궤도 판정 headless |
| 9/23 종료 | 80.25% | 자동 선택/도킹 85%, 복귀 60%, 회귀 85% |

## 11. 남은 퍼센티지별 작업 계획

현재 31.5%이므로 남은 점수는 68.5점이다.

| 등급 | 기능 | 현재→최종 | 남은 점수 | 구체 출력 |
|---|---|---:|---:|---|
| Must | AprilTag 포획 | 60→100 | 6.0 | GUI 2회 artifact |
| Must | 운반·도킹 | 60→100 | 6.0 | same-run hand-off |
| Must | 자유 유영 | 30→100 | 10.5 | body inventory·config·GUI |
| Must | 궤도선·이탈 판정 | 30→100 | 7.0 | GUI line·nearest·4 metrics |
| Must | 자동 선택·접근·도킹 | 30→100 | 10.5 | selector·target binding |
| Should | 정상 궤도 복귀 | 0→100 | 15.0 | 제한 controller·5초 verdict |
| Should | ML | 0→100 | 10.0 | dataset·split·모델·비교 |
| Should/Could | 팔 기능·회귀 | 30→100 | 3.5 | seed matrix·후퇴/undock |

ML 채택 기준은 동일 seed/image/GT에서 위치·각도·0.1/0.3/1.0초 예측 median/p95, tag-loss 구간, 지연 p95, 포획 성공률로 비교한다. baseline보다 성공률이 낮거나 지연이 10 ms를 넘거나 각도 p95 개선이 20% 미만이면 ML을 runtime 경로에 넣지 않는다.

## 12. 테스트 및 합격 기준

| 구간 | 합격 기준 | 증거 |
|---|---|---|
| 궤도 수학 | 128점 반경·평면 오차 `1e-10` 이하 | pytest |
| 궤도 GUI | 빨간 YZ 원, reset 후 유지, 물리 API 없음 | screenshot/video+stage dump |
| AprilTag | 4/4, RMS≤1.5 px, pose age≤1 s | overlay+CSV |
| 포획 | 거리≤0.15 m, angle≤5°, rel speed≤0.05 m/s, lateral≤0.02 m | gate log |
| 포획 유지 | 10 s, 상대 drift≤5 mm/0.5° | result JSON |
| 도킹 | axial/radial≤10 mm, axis≤0.5°, roll≤1°, approach≤3° | gate log |
| 복귀 | 4개 궤도 기준을 5 s 연속 만족 | verdict JSON |
| 전체 | seed 3개 headless, crash/NaN 0; 월요일 GUI 2회 | artifact bundle |

현재 실행 증거: `uv run pytest tests/test_orbit_reference.py -q`는 **9 passed**, 관련 `compileall`은 exit 0이다. PXR·PyYAML이 없는 일반 `uv` 환경이라 실제 USD stage/GUI는 미검증이다.

## 13. 리스크, 차선책, 롤백 기준

| 위험 | 영향 | 조기 신호 | 차선책 | 롤백/중단 기준 |
|---|---|---|---|---|
| 두 상태 머신 hand-off 부재 | 치명 | 단일 명령 없음 | 두 정직한 분리 demo | freeze 전 1회 성공 없으면 통합 주장 금지 |
| MRV static/world-fixed | 치명 | root 해제 후 IK 발산 | 공통 궤도 시각화+상대운동 | 충돌/발산 시 fixed 복귀 |
| USD curve 렌더 차이 | 중간 | GUI에서 선 미표시 | width 조정 또는 전용 viewport debug line | AprilTag 영상 오염 시 debug line 금지 |
| joint/3 t payload 불안정 | 높음 | drift·진동·NaN | 검증된 저속/CPU pipeline | gate 위반 시 attach/dock 금지 |
| 로그 소실·범위 폭증 | 높음 | PASS 주장만 남음 | artifact manifest·Must 동결 | artifact 없는 run 점수 상향 금지 |

기능 롤백은 branch 전환이나 사용자 파일 삭제가 아니다. `orbit_reference.enabled: false` 또는 검증된 config로 경로를 선택하며 실패 로그는 보존한다.

## 14. 시연 당일 체크리스트

시작 전:

- [ ] branch/commit/status와 `srb.__file__` 저장
- [ ] config hash/seed/명령/Isaac·GPU 버전 저장
- [ ] console·CSV·JSON·video 경로와 디스크 확인
- [ ] 빨간 선을 “축척 시각 기준”으로 설명할 문구 확인
- [ ] ABORT와 녹화본 전환 절차 확인

발표자 3~5분 순서:

1. 0:00–0:30 — 축척 무중력 씬과 빨간 기준 원의 의미를 설명한다.
2. 0:30–1:20 — 4개 tag, PnP RMS, 추정·예측 상태를 보여 준다.
3. 1:20–2:10 — capture gate 통과 후에만 EE–MEP joint가 생김을 보여 준다.
4. 2:10–3:30 — 운반·Client frame 정렬·probe 삽입을 보여 준다.
5. 3:30–5:00 — dock gate·FixedJoint·3초 유지와 artifact 경로를 보여 준다.

## 15. 매일 갱신할 진행 로그 템플릿

```markdown
# MRV Daily Progress — YYYY-MM-DD

## 기준
- 기준 시각(KST):
- 브랜치 / commit:
- config hash / seed:
- Isaac Sim / Isaac Lab / GPU:

## 진행률
- 1차 시연 준비율: 시작 __% → 종료 __%
- 전체 프로젝트: 시작 __% → 종료 __%
- 점수 변경 근거:

## 완료와 검증 증거
| 항목 | 계획/구현/자동시험/GUI | 결과 | artifact | 증거 수준 |
|---|---|---|---|---:|

## 실패와 원인
| 명령/상태 | 증상 | 직접 원인 | 재현 여부 | 다음 조치 |
|---|---|---|---|---|

## 새 리스크·다음 작업·결정
- 새 리스크:
- 다음 작업:
- 차단 요소/결정권자/시한:

## 실행 기록
- exact command:
- console / JSON / CSV / video:
- exit code:
- 예상 시간 / 실제 wall time / simulated time:
```

## 16. 즉시 착수할 작업 상위 5개

1. 데모 PC에서 현재 HEAD·config로 AprilTag와 docking 두 기존 경로의 원시 로그를 확보한다.
2. 실제 EE–MEP 상대 pose를 유지한 채 `VisionCaptureDemo`에서 `DockingDemo` 운반 단계로 hand-off한다.
3. 한 명령 headless E2E 1회와 GUI 2회 리허설을 artifact bundle로 남긴다.
4. 새 빨간 USD 궤도선의 GUI 표시·reset 유지·카메라 비오염을 확인한다.
5. MRV/MEP/Client의 fixed/kinematic/mass/inertia/v/w inventory를 저장한다.

가장 큰 위험은 각 기능의 별도 성공을 하나의 자유 유영 E2E 성공으로 오인하는 것이다. 성공 판정은 같은 프로세스·물리 씬·artifact의 연속 `AprilTag → CAPTURED → DOCKED` 기록으로만 한다.
