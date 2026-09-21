# Isaac Sim Astrobee 정찰 로봇 추가 및 포획 가능성 판정 시스템 수정 프롬프트

## 1. 프로젝트 목표

현재 Isaac Sim 기반 우주 위성 임무 연장 시스템에 NASA Astrobee 계열의 자유비행 정찰 로봇을 추가한다.

기존 Canadarm3 로봇팔의 역할은 유지한다.

- Astrobee: 대상 위성을 사전 정찰하고 포획 가능 여부를 판단
- Canadarm3: 포획 가능하다고 판단된 대상에 대해 기존 MEP 포획 작업 수행
- MEP: 기존 프로젝트의 포획 대상
- Target Satellite: 정찰 및 이후 도킹 대상

핵심 목표는 Astrobee가 실제 우주에서 사용하는 자유비행 로봇이라는 개념을 활용하여, 단순히 드론 모양의 물체를 움직이는 것이 아니라 `자유비행 정찰 로봇`으로 시스템에 통합하는 것이다.

---

## 2. 반드시 지켜야 할 방향

### 2.1 Quadcopter를 우주용 로봇으로 개조하지 않는다

기존 Isaac Sim Quadcopter, Crazyflie, Ingenuity 등을 억지로 우주용 드론으로 변경하지 않는다.

Astrobee를 우선 사용한다.

Astrobee는 실제 ISS에서 운용되는 자유비행 로봇이며, 기존 Gazebo 기반 시뮬레이터/ROS 생태계가 존재한다.

### 2.2 Astrobee의 전체 기능을 복제하지 않는다

프로젝트의 목적은 Astrobee 자체를 완벽하게 시뮬레이션하는 것이 아니다.

필요한 최소 기능만 구현한다.

- 자유비행 6-DOF 이동
- 카메라
- 대상 위성 접근
- AprilTag 또는 기존 프로젝트의 마커 인식
- 상대 위치/자세 확인
- 포획 가능성 판정
- 판정 결과를 다른 시스템에 전달

### 2.3 실제 물리적으로 말이 되는 우주 이동을 사용한다

지상용 프로펠러 드론처럼 공기 중에서 비행하는 방식은 사용하지 않는다.

Astrobee는 우주 자유비행 로봇이므로 최소한 다음 6-DOF 개념을 유지한다.

- X
- Y
- Z
- Roll
- Pitch
- Yaw

단, 프로젝트 기간과 시연 목적을 고려하여 복잡한 실제 추진기/궤도역학/자세제어까지 구현할 필요는 없다.

초기 구현에서는 안정적인 목표점 접근 방식 또는 간단한 6-DOF 위치/자세 제어를 사용한다.

---

# 3. 전체 작업 흐름

다음 순서로 시스템을 구성한다.

```text
Astrobee Spawn
    ↓
초기 위치 설정
    ↓
Target Satellite 접근
    ↓
Inspection Position 도달
    ↓
Camera로 Target Satellite 관측
    ↓
AprilTag / 기존 마커 검출
    ↓
상대 거리 계산
    ↓
상대 자세 계산
    ↓
Capture Point 확인
    ↓
접근 경로 장애물 확인
    ↓
Capture Feasibility 판정
    ↓
┌──────────────────────┐
│                      │
│   CAPTURE AVAILABLE   │
│          또는          │
│   CAPTURE UNAVAILABLE │
│                      │
└──────────────────────┘
    ↓
가능한 경우
    ↓
Canadarm3 기존 포획 파이프라인으로 전달
```

---

# 4. Astrobee 추가

## 4.1 모델 확보

먼저 NASA Astrobee 공식 소프트웨어/로봇 모델을 조사한다.

우선 확인할 내용:

- Astrobee 공식 GitHub repository
- robot description
- URDF
- mesh 파일
- sensor description
- camera 관련 description
- ROS 2/ROS 기반 인터페이스
- 기존 simulator 구성

Astrobee 모델을 Isaac Sim에 직접 사용할 수 있는 공식 USD가 없다면 URDF 기반 import를 우선 검토한다.

가능한 경우:

```text
Astrobee URDF
    ↓
Isaac Sim URDF Importer
    ↓
USD
    ↓
Isaac Sim
```

모델 파일이 URDF가 아닌 경우에는 해당 파일 형식을 분석하고 Isaac Sim에서 사용할 수 있는 가장 안전한 변환 방법을 선택한다.

임의로 새로운 3D 모델을 만들어 Astrobee라고 가정하지 않는다.

---

# 5. Isaac Sim Asset 변환

Isaac Sim의 URDF importer를 활용한다.

확인해야 하는 항목:

- visual mesh 정상 표시
- collision 정상 생성
- rigid body 설정
- mass
- inertia
- articulation/joint 설정
- transform
- scale
- material
- camera 위치

변환 후 별도의 USD Asset으로 저장하여 기존 프로젝트에서 재사용할 수 있도록 한다.

예:

```text
assets/
└── astrobee/
    ├── astrobee.usd
    ├── meshes/
    └── config/
```

실제 프로젝트 구조가 이미 존재한다면 기존 구조를 우선 따르고 불필요한 디렉터리를 새로 만들지 않는다.

---

# 6. 자유비행 구현

Astrobee는 지상 이동 로봇이 아니므로 wheel-based navigation을 사용하지 않는다.

최소한의 자유비행 제어를 구현한다.

## 6.1 1차 목표

우선 다음 정도만 안정적으로 구현한다.

```text
Start Position
      ↓
Approach Target Position
      ↓
Inspection Position
      ↓
Stop
```

예:

```python
target_position = [x, y, z]
```

와 같은 목표점을 지정하고 Astrobee가 해당 위치로 이동하도록 한다.

## 6.2 6-DOF

필요하면 다음을 독립적으로 제어할 수 있도록 구조를 만든다.

```text
position:
    x
    y
    z

orientation:
    roll
    pitch
    yaw
```

초기 시연에서는 Roll/Pitch/Yaw를 크게 변화시키지 않아도 된다.

단, 구조 자체는 향후 자세 제어를 추가할 수 있도록 작성한다.

---

# 7. Astrobee Camera 추가

Astrobee에 카메라를 추가한다.

카메라의 위치와 방향은 Astrobee 전면을 바라보도록 설정한다.

필요한 경우 프로젝트에서 이미 사용 중인 카메라/센서 설정을 재사용한다.

카메라가 실제 화면을 생성하는지 확인한다.

최종 목표:

```text
Astrobee
   │
   └── Camera
          ↓
      Target Satellite
```

---

# 8. Target Satellite 정찰

Astrobee가 Target Satellite에 접근하면 자동으로 Inspection Position에서 정지한다.

이 위치에서:

1. 카메라 활성화
2. Target Satellite 촬영
3. AprilTag 또는 기존 프로젝트의 마커 검출
4. 상대 위치 계산
5. 상대 자세 계산
6. Capture Point 확인

을 수행한다.

---

# 9. Capture Point 정의

Target Satellite에 실제 작업을 수행할 수 있는 기준 위치를 정의한다.

예:

```text
Target Satellite
┌──────────────────────┐
│                      │
│      [AprilTag]      │
│           ●          │
│      Capture Point   │
│                      │
└──────────────────────┘
```

Capture Point는 실제 Canadarm3가 접근하여 작업할 수 있는 기준점으로 사용한다.

기존 프로젝트에서 이미 정의된 포획 관련 위치/마커가 있다면 그것을 우선 재사용한다.

---

# 10. Capture Feasibility 판정

Astrobee가 단순히 AprilTag를 찾았다는 이유만으로 포획 가능하다고 판단하지 않는다.

다음 조건을 사용하여 판정한다.

```text
Capture Feasibility =
    Marker Detected
    AND Distance Valid
    AND Relative Pose Valid
    AND Capture Point Visible
    AND Approach Path Clear
```

최소 구현에서는 다음 조건부터 적용한다.

### 조건 1. Marker Detection

AprilTag 또는 기존 마커가 정상적으로 검출되어야 한다.

### 조건 2. Distance

Astrobee와 Target Satellite 사이의 상대 거리가 허용 범위 안이어야 한다.

예:

```text
distance <= MAX_INSPECTION_DISTANCE
```

실제 값은 기존 환경 크기에 맞게 설정한다.

### 조건 3. Relative Pose

대상의 상대 위치/자세가 허용 범위 안이어야 한다.

### 조건 4. Capture Point Visibility

Canadarm3가 접근해야 하는 Capture Point가 카메라에서 확인 가능해야 한다.

### 조건 5. Approach Path

포획 위치까지 이동하는 경로에 명백한 장애물이 없어야 한다.

처음부터 복잡한 충돌회피 알고리즘을 구현하지 않는다.

---

# 11. 판정 결과

판정 결과는 명확하게 두 상태로 관리한다.

```text
CAPTURE_AVAILABLE
CAPTURE_UNAVAILABLE
```

가능한 경우:

```text
Astrobee
   ↓
Inspection Complete
   ↓
CAPTURE_AVAILABLE
   ↓
Canadarm3
   ↓
MEP Capture
```

불가능한 경우:

```text
Astrobee
   ↓
Inspection Complete
   ↓
CAPTURE_UNAVAILABLE
   ↓
작업 중단 또는 다른 대상 탐색
```

---

# 12. 기존 Canadarm3 시스템과 연결

기존 Canadarm3와 MEP의 포획 로직은 가능한 한 변경하지 않는다.

기존 시스템이 다음 구조라면:

```text
Canadarm3
    ↓
gripper_fixture
    ↓
MEP
    ↓
Attachment
```

이 구조를 그대로 유지한다.

Astrobee는 이 파이프라인 앞단에 추가한다.

최종 구조:

```text
Astrobee
    ↓
Target Inspection
    ↓
Capture Feasibility
    ↓
CAPTURE_AVAILABLE
    ↓
Canadarm3
    ↓
MEP Capture
    ↓
기존 후속 파이프라인
```

---

# 13. 기존 MEP Capture 기능을 깨뜨리지 않는다

이번 작업에서 특히 중요한 사항이다.

다음 기존 기능은 변경을 최소화한다.

- Canadarm3
- gripper_fixture
- MEP
- MEP physics
- 기존 attachment 방식
- 기존 MEP 포획 위치
- 기존 docking 로직
- 기존 웹/ROS2 인터페이스

Astrobee 추가로 인해 기존 기능이 동작하지 않으면 먼저 원인을 확인하고 기존 기능을 보존하는 방향으로 수정한다.

---

# 14. 시작 상태

Isaac Sim은 자동 Play 상태로 시작하지 않는다.

기본 상태:

```text
Simulation: PAUSED
Astrobee: 초기 위치에서 정지
Canadarm3: 기존 초기 위치
MEP: 기존 초기 위치
Target Satellite: 기존 초기 위치
```

Play 이후 Astrobee의 inspection sequence를 시작한다.

---

# 15. 우주환경 조건

중력은 기존 프로젝트의 우주환경 설정을 유지한다.

Astrobee 이동은 중력에 의존하지 않는다.

지상용 드론의 다음 요소는 사용하지 않는다.

- 프로펠러 양력
- 공기저항 기반 비행
- 지상용 path following
- 바퀴 이동

자유비행 rigid body의 위치/자세 제어를 사용한다.

---

# 16. 웹 시스템과의 연결

현재 프로젝트의 웹 구조가 존재한다면 Astrobee 상태도 웹에서 확인할 수 있도록 확장한다.

최소 표시 항목:

```text
ASTROBEE
----------------
Status: INSPECTING
Target: SAT-001

Distance: 4.2 m
Marker: DETECTED
Pose: VALID
Capture Point: CLEAR

Result:
CAPTURE AVAILABLE
```

웹에서는 Isaac Sim의 실제 시뮬레이션 화면과 함께 Astrobee 상태를 확인할 수 있도록 한다.

---

# 17. 다중 PC 구조

다중 PC 환경을 고려하여 코드 구조를 작성한다.

권장 개념:

```text
PC A
┌────────────────────────┐
│ Isaac Sim              │
│                        │
│ MRV                    │
│ Canadarm3              │
│ MEP                    │
│ Target Satellite       │
└───────────┬────────────┘
            │
          ROS 2
            │
            ▼
PC B
┌────────────────────────┐
│ Astrobee               │
│                        │
│ Camera                 │
│ Inspection             │
│ Feasibility            │
└────────────────────────┘
```

단, 두 PC에서 독립적인 Isaac Sim physics world를 실행할 경우 simulation time과 object state 동기화 문제가 발생할 수 있으므로, 실제 구현에서는 기존 프로젝트 구조를 먼저 확인한다.

가능하다면 하나의 simulation authority를 두고 Astrobee와 Canadarm3의 상태를 ROS 2 등으로 전달하는 방식을 우선 검토한다.

다중 PC가 반드시 필요한 기능과 단순 분산 실행을 구분한다.

---

# 18. 구현 우선순위

다음 순서대로 구현한다.

## Phase 1 — Astrobee 모델

- [ ] Astrobee 공식 모델 확보
- [ ] URDF/description 구조 확인
- [ ] Isaac Sim import 가능 여부 확인
- [ ] USD 생성
- [ ] Isaac Sim에서 정상 표시

## Phase 2 — 자유비행

- [ ] Rigid Body 설정
- [ ] 질량/관성 확인
- [ ] 6-DOF 상태 확인
- [ ] 목표 위치 이동
- [ ] Inspection Position 정지

## Phase 3 — Camera

- [ ] Camera 추가
- [ ] Target Satellite 촬영
- [ ] Camera orientation 확인

## Phase 4 — Marker

- [ ] AprilTag 또는 기존 마커 배치
- [ ] Detection 구현
- [ ] 상대 위치 계산
- [ ] 상대 자세 계산

## Phase 5 — Feasibility

- [ ] Distance 조건
- [ ] Pose 조건
- [ ] Capture Point 조건
- [ ] 접근 경로 조건
- [ ] CAPTURE_AVAILABLE / UNAVAILABLE 상태 생성

## Phase 6 — Canadarm3 연결

- [ ] 판정 결과 전달
- [ ] CAPTURE_AVAILABLE일 때 기존 Canadarm3 pipeline 실행
- [ ] 기존 MEP attachment 기능 유지

## Phase 7 — Web

- [ ] Astrobee 상태 표시
- [ ] 검사 결과 표시
- [ ] 기존 Isaac Sim 화면 표시
- [ ] 필요 시 Play/Pause 및 기본 화면 조작 유지

---

# 19. 코드 작성 원칙

1. 기존 코드를 전부 갈아엎지 않는다.
2. 기존 Canadarm3/MEP 기능을 최대한 재사용한다.
3. Astrobee 관련 코드는 별도의 모듈/파일로 분리한다.
4. 하드코딩된 경로는 최소화한다.
5. 기존 프로젝트의 설정/환경 변수를 우선 재사용한다.
6. 테스트 가능한 작은 함수 단위로 구현한다.
7. 실제로 실행할 수 없는 기능을 구현 완료로 표시하지 않는다.
8. Isaac Sim에서 확인하지 못한 동작은 성공했다고 가정하지 않는다.
9. 우주환경과 맞지 않는 지상용 드론 물리 모델을 사용하지 않는다.
10. Astrobee를 단순 시각적 장식물로 추가하지 않는다. 반드시 Inspection이라는 기능적 역할을 갖도록 한다.

---

# 20. 실행 기록 Markdown 파일

이번 작업에서 수행한 모든 작업을 별도의 Markdown 파일에 기록한다.

예:

```text
docs/astrobee_progress.md
```

기록 항목:

```markdown
# Astrobee Integration Progress

## Current Phase

## Completed

## In Progress

## Pending

## Modified Files

## Commands Executed

## Verification

## Issues

## Next Step
```

실제로 실행한 명령과 수정한 파일을 정확히 기록한다.

실행하지 않은 명령은 실행한 것처럼 기록하지 않는다.

---

# 21. 최종 시연 시나리오

최종적으로 다음 장면이 자연스럽게 이어져야 한다.

```text
[1] Mission Start
        ↓
[2] Astrobee Launch
        ↓
[3] Target Satellite Approach
        ↓
[4] Inspection Position
        ↓
[5] Camera / AprilTag Detection
        ↓
[6] Relative Pose Calculation
        ↓
[7] Capture Feasibility Check
        ↓
     ┌───────────────┐
     │               │
     ▼               ▼
 AVAILABLE       UNAVAILABLE
     │
     ▼
Canadarm3 Activated
     ↓
MEP Capture
     ↓
Existing Mission Pipeline
```

핵심 메시지는 다음과 같다.

> Astrobee가 먼저 대상 위성을 정찰하여 작업 가능 여부를 판단하고, 작업 가능한 경우에만 Canadarm3가 실제 포획 작업을 수행한다.

---

# 22. 구현 완료 기준

다음 조건을 모두 만족해야 구현 완료로 판단한다.

- [ ] Isaac Sim에 Astrobee가 정상적으로 표시된다.
- [ ] Astrobee가 자유비행 방식으로 이동한다.
- [ ] Astrobee가 Target Satellite의 Inspection Position까지 이동한다.
- [ ] Astrobee Camera가 Target Satellite를 바라본다.
- [ ] AprilTag 또는 지정 마커를 검출할 수 있다.
- [ ] 상대 거리/자세를 계산할 수 있다.
- [ ] Capture Feasibility를 판정할 수 있다.
- [ ] CAPTURE_AVAILABLE 결과가 생성된다.
- [ ] CAPTURE_UNAVAILABLE 결과가 생성된다.
- [ ] CAPTURE_AVAILABLE일 때 기존 Canadarm3 포획 파이프라인과 연결된다.
- [ ] 기존 MEP attachment 기능이 유지된다.
- [ ] 기존 docking 기능이 깨지지 않는다.
- [ ] 웹에서 Astrobee 상태와 판정 결과를 확인할 수 있다.
- [ ] 작업 과정이 `docs/astrobee_progress.md`에 기록된다.

---

# 23. 중요

이번 수정의 핵심은 "드론 하나를 추가하는 것"이 아니다.

다음과 같은 다중 로봇 협업 시스템을 만드는 것이다.

```text
Astrobee
= Scout / Inspection Robot
        ↓
"이 위성을 지금 포획할 수 있는가?"

        ↓

Canadarm3
= Manipulation Robot
        ↓
"가능한 경우 실제 MEP 포획"
```

따라서 Astrobee가 실제 작업을 직접 수행할 필요는 없다.

Astrobee의 역할은 정찰과 판단이다.

Canadarm3의 역할은 실제 조작이다.

두 로봇의 역할을 명확하게 분리하고, 기존 프로젝트의 `debris_capture → docking` 파이프라인을 최대한 보존하면서 Astrobee를 앞단의 지능형 정찰 단계로 추가한다.
