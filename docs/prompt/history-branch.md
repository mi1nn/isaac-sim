# 브랜치 개발 이력과 README 기준

> 기준: 2026-09-21 `git fetch --prune origin` 이후의 원격 브랜치와 로컬 브랜치.
>
> 이 문서는 브랜치를 병합하기 위한 변경 목록이 아니라, 각 브랜치가 구현한 기능을
> 식별하기 위한 이력표다. `main`의 README를 작성할 때 이 문서를 기능 설명의 근거로 쓴다.

## 1. 문서 운영 원칙

- 각 브랜치의 루트 `README.md`를 각각 따로 발전시키지 않는다. 같은 파일이 여러 브랜치에서
  충돌하고, 병합 뒤에는 어느 설명이 최신인지 알기 어려워진다.
- 브랜치 단위 설명은 이 파일과 향후 `docs/branches/<branch>.md`에 보관한다.
- 루트 `README.md`는 `main`에서 실행 가능한 통합 기능, 실행 방법, 검증 상태만 설명한다.
- 테스트·시연이 확인되지 않은 항목은 구현 여부와 별도로 `미검증`으로 표시한다.
- 현재 미커밋 작업 트리 변경은 이 문서의 브랜치 이력에 포함하지 않는다.

## 2. 권장 README 구조

| 위치 | 목적 | 포함할 내용 |
|---|---|---|
| `README.md` | main의 통합 안내 | 프로젝트 목표, 지원 시나리오, 설치, 빠른 실행, 검증 상태, 핵심 문서 링크 |
| `docs/branch-history.md` | 전체 이력 색인 | 브랜치 계보, 구현 기능, 상태, 병합·보관 판단 |
| `docs/branches/<branch>.md` | 브랜치별 상세 README | 목표, 변경 파일, 실행 명령, 검증 결과, 제약, 후속 브랜치 |
| `mep_monitor/README.md` | 독립 ROS2 패키지 안내 | 패키지 설치, topic contract, mock/실측 데이터 구분 |

## 3. 개발 계보

```text
feature/asset → feature/asset_v2 → feature/grip
                                      └→ capture / docking 기반 (5b7e55b)
                                           ├→ feature/magnet (= feature/debris, local alias)
                                           ├→ feature/apriltag → feature/rotation
                                           │                      └→ feature/docking_param
                                           │                           └→ feature/docking
                                           │                                └→ feature/debris_satellite (merge)
                                           └→ feature/redline (별도 정상궤도 시각화 실험)

feature/grip 이후 별도 경로: feature/suction
별도 저장소 구조: feature/space, feature/arm
별도 ROS2 패키지: feature/mep-monitoring
```

`feature/docking_param`과 `feature/docking`은 `bb53327`에서 갈라진 형제 브랜치다.
`feature/debris_satellite`는 두 브랜치의 변경을 합친 로컬 통합 브랜치다.

## 4. 기반·초기 환경 브랜치

| 브랜치 | 기준 커밋 | 구현 기능 | 상태와 README에 남길 내용 |
|---|---:|---|---|
| `main` | `6a9de82` | 초기 standalone Isaac Sim 기반 | 현재 위성 개발 계열과 공통 조상이 없어 직접 병합 기준으로 쓰면 안 됨. main README는 통합 검증 뒤 별도 작성 필요. |
| `feature/space` | `dc31194` | M0609·Kinova 관련 USD/에셋 실험 | `space_robotics_bench/` 하위의 이전 저장소 구조다. 현 `project/` 기반 기능과 분리 보관. |
| `feature/arm` | `b8af492` | `satellite_mission` task, MEP USDA, contact sensor 보강, smoke script | 루트 `srb/` 구조를 사용한다. 현 `project/srb/` 계열에 바로 병합하지 않는다. |
| `feature/asset` | `2ba9e70` | MEP·Satellite USD를 저장소로 이동하고 reference path를 수정, debris capture task의 에셋 경로 갱신 | 현 시나리오의 에셋 기반이다. 에셋 위치와 USD 참조 규칙을 문서화한다. |
| `feature/asset_v2` | `d22e253` | IDE 설정 파일을 추적 해제하고 `.gitignore` 정리 | 기능 브랜치가 아닌 저장소 위생 이력이다. main README가 아니라 개발 규칙에 짧게 기록한다. |

## 5. 수동 파지·흡착 실험 브랜치

| 브랜치 | 기준 커밋 | 구현 기능 | 상태와 README에 남길 내용 |
|---|---:|---|---|
| `feature/grip` | `82a0930` | Kinova 그리퍼/Canadarm3 수동 제어 실험, debris capture 초기화·물리 설정, 작업 로그 | 수동 파지 환경의 출발점. `docs/debris_capture_environment_setup.md`를 실행·한계 설명의 원본으로 사용한다. |
| `feature/magnet` | `8a5e8ef` | 상위 capture·docking 코드를 상속하며 tip에서는 IDE ignore만 수정 | 독자 기능은 거의 없다. `feature/debris`와 동일 커밋을 가리키는 로컬 별칭이다. 하나만 유지 대상으로 정한다. |
| `feature/debris` (local) | `8a5e8ef` | `feature/magnet`과 동일 | 중복 별칭이다. 별도 README를 만들지 않는다. |
| `feature/suction` | `3ac25bc` | ROS2 흡착 포획, RGB-D pose, pose controller, vision servo, smoke/geometry 도구, 테스트 | AprilTag 파이프라인과는 별도 접근이다. ROS2 의존성·실행 명령·검증 범위를 전용 문서에 기록한다. |

## 6. AprilTag 포획·유영·도킹 주 개발 계열

| 브랜치 | 기준 커밋 | 구현 기능 | 상태와 README에 남길 내용 |
|---|---:|---|---|
| `feature/apriltag` | `923e379` | 3 t MEP 무중력 선형 유영, 4 AprilTag PnP, `Cylinder_01` pose 추정, 미래 위치 예측, IK 접근, FixedJoint 포획, Test 1/2/3 | Phase 1의 핵심 기반. `docs/mrv_phase1_vision_capture.md`와 `vision_capture.py` 실행법을 main README에 반영한다. |
| `feature/rotation` | `3166171` | `six_dof` 모드, MEP XYZ 병진과 roll/pitch/yaw 회전, ConstantTwist 예측, 6-DoF 테스트 | Phase 1 확장 기능. 문서상 일부 Isaac Sim 검증 항목은 아직 남아 있다. |
| `feature/docking_param` | `d8bf300` | probe/thruster 도킹 1차 구현과 객체 재배치·속도 조정 이력을 포함 | tip 자체는 `.vscode/settings.json` 삭제뿐이다. 독립 파라미터 기능으로 소개하지 말고 도킹 중간 이력으로 표시한다. |
| `feature/docking` | `cff239a` | MEP 포획 → Ares1 probe → Client Satellite thruster 도킹 파라미터 조정, 전체 파이프라인 성공 기록 | 무회전·무유영 Client 기준 성공 상태다. `docs/mrv_probe_docking.md`의 실행 결과와 제한을 main README에 반영한다. |
| `feature/debris_satellite` (local) | `7cdaaaf` | `feature/docking_param`과 `feature/docking`을 merge한 현재 통합 브랜치 | 현재 개발 기준 브랜치. 앞으로 정상궤도 복귀 기능과 통합 테스트는 이 브랜치에서 이어간다. |

## 7. 병렬 기능 브랜치

| 브랜치 | 기준 커밋 | 구현 기능 | 상태와 README에 남길 내용 |
|---|---:|---|---|
| `feature/redline` | `2b68dd9` | 빨간 원형 reference orbit 수학, visual-only USD `BasisCurves`, 단위 테스트, 정상궤도 시각화 설계 문서 | 현재 `vision_capture.yaml`, `vision.py`, `vision_task.py`에 연결되지 않았다. `graphify-out/` 생성 파일과 별도 `probe_dock` 파일이 섞여 있어 직접 merge하지 않는다. 필요한 orbit 모듈만 선별 통합한다. |
| `feature/mep-monitoring` | `17cd94b` | ROS2 telemetry, PlotJuggler 설정, CSV logger, monitoring node, 실 Isaac capture metrics bridge | 독립 `mep_monitor` 패키지다. 기존 README가 mock-only라고 설명하므로 실제 bridge 추가 내용으로 갱신해야 한다. |

## 8. 현재 통합 기능 기준

`feature/debris_satellite`에서 확인할 기능 묶음은 다음과 같다.

1. AprilTag 기반 MEP pose 추정과 무회전 선형 유영 포획
2. 선택적 6-DoF MEP 유영·자세 예측
3. FixedJoint 기반 MEP capture와 probe/thruster docking
4. 도킹 geometry, depth camera, alignment gate, holding 검증
5. 다음 작업: Client Satellite의 등속 유영·빨간 reference orbit·도킹 후 orbit return

## 9. main README 작성 전 확인 목록

1. 기준 브랜치를 `feature/debris_satellite`로 확정하고 main에 병합할 커밋 범위를 정한다.
2. `feature/redline`에서 필요한 reference orbit 코드만 선별 통합하고 생성 산출물은 제외한다.
3. capture, 6-DoF, docking, orbit return 각각의 Isaac Sim 실행 결과를 최신 상태로 다시 기록한다.
4. `mep_monitor` README에서 mock telemetry와 Isaac metrics bridge의 지원 범위를 분리한다.
5. 위 확인이 끝난 뒤 main `README.md`를 이 문서의 6~8절을 기반으로 교체한다.

## 10. 브랜치별 상세 README 템플릿

향후 `docs/branches/<branch>.md`를 만들 때 아래 구조를 사용한다.

```md
# <branch> — <기능 이름>

## 목적

## 구현 범위

## 주요 파일

## 실행

## 검증 결과

## 알려진 제약

## 후속 브랜치 또는 병합 대상
```
