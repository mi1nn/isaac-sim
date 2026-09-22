# Isaac Sim Astrobee 위성 정찰 및 MEP 도킹 가능성 판정 시스템 구현 프롬프트

## 1. 프로젝트 목표

현재 Isaac Sim 기반의 Automated Satellite Mission Extension System에 NASA Astrobee 계열의 자유비행 정찰 로봇 기능을 추가한다.

기존 시스템의 주요 구성은 유지한다.

- Canadarm3
- MEP
- Client / Target Satellite
- 기존 MEP Capture 기능
- 기존 MEP Attachment 기능
- 기존 Satellite Docking 기능
- 기존 ROS2 / Web Monitoring 구조

이번 작업에서 Astrobee의 역할을 명확하게 구분한다.

### Astrobee의 역할

Astrobee는 MEP를 찾거나 포획하기 위한 로봇이 아니다.

Astrobee는 Target Satellite 주변을 자유비행하면서 Satellite와 주변 상황이 카메라에 계속 보이도록 관찰하고 모니터링하는 정찰 로봇이다.

Astrobee 자체가 어떤 값을 계산하거나 상태를 판단하지 않는다.

즉 역할은 다음과 같다.

- Astrobee = Satellite Observation / Monitoring Robot
- Canadarm3 = MEP Manipulation Robot
- MEP = Mission Extension Pod
- Target Satellite = MEP가 최종적으로 도킹하는 대상

---

# 2. 반드시 지켜야 할 방향

## 2.1 Astrobee의 역할을 MEP Capture와 혼동하지 않는다

Astrobee는 다음 작업을 하지 않는다.

- MEP 탐색
- MEP AprilTag 검출
- MEP Capture Point 검출
- Canadarm3 포획 위치 계산
- MEP 직접 포획
- MEP 직접 도킹
- Satellite 상태 계산
- Satellite 회전 속도 계산
- Docking Feasibility 계산 또는 판단
- MEP 접근 가능 여부 판단
- Docking 방향 안정성 판단
- Mission 진행 여부 결정

Astrobee가 관찰하는 대상은 Target Satellite와 그 주변 상황이다.

## 2.2 Satellite에 AprilTag / Marker Asset을 추가하지 않는다

이번 Astrobee 정찰 기능을 위해 Target Satellite에 다음과 같은 추가 Asset을 붙이지 않는다.

- AprilTag
- ArUco Marker
- 별도의 Marker Board
- Capture Point Marker
- 검사 전용 Mesh
- 검사 전용 Visual Target

Satellite는 현재 사용하고 있는 기존 모델을 그대로 유지한다.

Astrobee는 Satellite 주변을 자유비행하면서 기존 Satellite의 움직임과 자세 상태를 관찰한다.

## 2.3 기존 MEP Capture 시스템을 변경하지 않는다

현재 동작하고 있는 다음 기능을 최대한 그대로 유지한다.

- Canadarm3
- MEP Capture
- MEP attachment
- gripper / fixture
- MEP 이동
- Satellite docking
- Docking Probe
- Dock Target
- 기존 ROS2 인터페이스
- 기존 Monitoring 기능

Astrobee는 기존 파이프라인을 대체하지 않는다.

Astrobee는 Satellite Docking 전에 수행되는 별도의 Inspection 단계로 추가한다.

---

# 3. 전체 시스템 역할

```text
Astrobee
    ↓
Target Satellite 접근
    ↓
Satellite가 카메라에 보이도록 위치 조정
    ↓
Satellite 및 주변 상황 지속 관찰
    ↓
Camera Observation Feed 제공
    ↓
ROS2 / Web Monitoring으로 전달
```

중요:

Astrobee는 위성의 상태를 계산하거나 판단하지 않는다.

Astrobee의 역할은 "보이게 하는 것"과 "계속 관찰하는 것"이다.

판정이 필요한 경우에는 Astrobee와 분리된 별도의 시스템에서 수행할 수 있다.

# 4. 기존 잘못된 흐름 제거

기존 문서에 존재했던 다음 흐름은 사용하지 않는다.

```text
Astrobee
→ AprilTag Detection
→ Capture Point Detection
→ Capture Feasibility
→ CAPTURE_AVAILABLE
→ Canadarm3 MEP Capture
```

이 구조는 현재 프로젝트 목적과 맞지 않으므로 제거한다.

Astrobee는 Canadarm3의 MEP Capture 가능성을 검사하는 로봇이 아니다.

---

# 5. 수정된 전체 Mission 흐름

```text
[1] Mission Start
        ↓
[2] Astrobee Observation Start
        ↓
[3] Astrobee가 Target Satellite 접근
        ↓
[4] Satellite 주변 Observation Position 이동
        ↓
[5] Satellite 주변 자유비행
        ↓
[6] Satellite 및 주변 상황 지속 관찰
        ↓
[7] Camera Observation Feed 제공
        ↓
[8] ROS2 / Web Monitoring으로 전달
```

이후 기존 Mission Pipeline은 Astrobee와 독립적으로 별도의 로직에 따라 진행한다.

Astrobee는 위 과정에서 어떤 계산이나 판정도 수행하지 않는다.

# 6. Astrobee Inspection 방식

Astrobee는 Satellite의 한 점만 보는 것이 아니라 Satellite 주변을 이동하면서 상태를 검사한다.

예시:

```text
                 Astrobee Position 2

                        ●
                        |
                        |
                        |

Astrobee 1 ● ---- Satellite ---- ● Astrobee 3

                        |
                        |
                        ●

                 Astrobee Position 4
```

정확히 원형 궤도역학을 구현할 필요는 없다.

이번 프로젝트에서 필요한 것은 Satellite 주변의 여러 Inspection Position을 이동하면서 Satellite의 자세와 회전 상태를 관찰하는 것이다.

예:

```text
INSPECTION_POINT_1
        ↓
INSPECTION_POINT_2
        ↓
INSPECTION_POINT_3
        ↓
INSPECTION_POINT_4
```

또는 Satellite 주변을 부드러운 경로로 이동할 수 있다.

이 이동은 실제 궤도 계산을 구현하려는 목적이 아니다.

Astrobee 자유비행 정찰 시나리오를 구현하기 위한 단순화된 6-DOF 이동이다.

---

# 7. Astrobee 자유비행

Astrobee는 지상용 Quadcopter 방식으로 구현하지 않는다.

사용하지 않을 개념:

- Propeller Lift
- 지상용 Drone Physics
- Wheel Navigation
- Gravity 기반 비행
- 공기저항 기반 Flight Model

Astrobee는 우주 자유비행 로봇이므로 최소한 다음 6-DOF 구조를 유지한다.

Position:
- X
- Y
- Z

Orientation:
- Roll
- Pitch
- Yaw

프로젝트 기간을 고려하여 실제 Astrobee propulsion system 전체를 구현할 필요는 없다.

초기 구현에서는 안정적인 Target Position 기반 이동 또는 간단한 Position / Orientation Controller를 사용할 수 있다.

---

# 8. Satellite / 주변 상황 관찰

Astrobee의 핵심 기능은 카메라를 이용해 Target Satellite와 주변 상황을 계속 보여주는 것이다.

Astrobee는 다음을 직접 계산하거나 판단하지 않는다.

- Satellite Orientation
- Satellite Angular Velocity
- Angular Speed
- Docking Direction
- Relative Orientation
- Relative Angular Velocity
- Docking Feasibility
- Mission 진행 여부

위 값이 필요하다면 Astrobee의 Camera Feed와 Simulation State를 별도의 외부 시스템에서 사용할 수 있도록 구조를 열어둔다.

이번 구현에서는 Astrobee에 계산 및 판정 로직을 추가하지 않는다.

# 9. Camera 구성

Astrobee에 Satellite가 보이는 카메라가 없다면 카메라를 추가한다.

목표:

```text
Astrobee
   ↓
Camera
   ↓
Target Satellite + 주변 상황
   ↓
Continuous Observation
   ↓
ROS2 / Web Monitoring
```

카메라는 특정 Marker나 특정 지점을 검출하기 위한 용도로 사용하지 않는다.

Astrobee의 카메라는 "위성과 주변 상황을 계속 볼 수 있는 감시 카메라"로 취급한다.

가능하면 Astrobee 이동 중에도 Target Satellite가 화면에서 유지되도록 카메라 방향과 Astrobee의 위치를 구성한다.

단, 이를 위해 별도의 Vision 기반 추적/판단 알고리즘을 구현하지 않는다.

---

# 10. Camera 기반 확장

이번 단계에서는 Camera Feed를 제공하는 것까지만 구현한다.

향후 별도의 외부 시스템에서 영상 분석이 필요할 경우 확장할 수 있지만, 그 분석 로직을 Astrobee에 직접 넣지 않는다.

사용하지 않는 기능:

- AprilTag Detection
- ArUco Detection
- Capture Point Detection
- Pose Estimation
- Optical Flow
- Natural Feature Tracking
- Docking Feasibility 판단

1차 목표:

```text
Astrobee 자유비행
+
Satellite 및 주변 상황 Camera Observation
+
Continuous Monitoring
```

---

# 11. Docking Feasibility는 Astrobee의 책임에서 제외

Astrobee는 Docking Feasibility를 판단하지 않는다.

Astrobee는 다음과 같은 판정 결과를 생성하지 않는다.

```text
DOCKING_AVAILABLE
DOCKING_UNAVAILABLE
```

Docking Feasibility 판단이 필요한 경우에는 Astrobee와 분리된 Mission Controller 또는 별도의 판단 시스템에서 수행할 수 있다.

Astrobee는 판단에 활용할 수 있는 Camera Observation Feed를 제공하는 감시/관찰 플랫폼으로만 동작한다.

# 12. Astrobee Inspection 관점

Astrobee가 특정 Docking Axis를 계산하거나 추적할 필요는 없다.

다만 카메라 화면에 Target Satellite와 Docking 관련 영역이 가능한 한 함께 보이도록 Astrobee의 위치와 카메라 방향을 구성한다.

기존 Docking Frame이나 Docking Port를 변경하지 않는다.

---

# 13. Astrobee Observation 단계

Astrobee의 내부 구현에서는 필요에 따라 다음과 같은 단계 구분을 사용할 수 있다.

```text
ASTROBEE_IDLE
ASTROBEE_APPROACH
ASTROBEE_OBSERVATION_START
ASTROBEE_OBSERVING
ASTROBEE_OBSERVATION_COMPLETE
```

단, 이 단계값을 Web UI에 Astrobee 상태 정보로 표시할 필요는 없다.

Astrobee는 Docking Feasibility 상태를 생성하지 않는다.

# 14. 예시 State Flow

```text
ASTROBEE_IDLE
        ↓
ASTROBEE_APPROACH
        ↓
ASTROBEE_OBSERVATION_START
        ↓
ASTROBEE_OBSERVING
        ↓
Satellite + 주변 상황 지속 관찰
        ↓
Camera Feed / Monitoring
        ↓
ASTROBEE_OBSERVATION_COMPLETE
```

Astrobee는 이 흐름에서 어떤 Mission 진행 여부도 결정하지 않는다.

---

# 15. Astrobee 관찰 테스트 환경 구성

Astrobee 카메라에 Target Satellite와 주변 상황이 실제로 보이는지를 확인한다.

## CASE 1

```text
Satellite 정지
        ↓
Astrobee가 Satellite를 향함
        ↓
Camera Feed에서 Satellite 확인
```

## CASE 2

```text
Satellite 저속 회전
        ↓
Astrobee Camera Feed에서 지속 관찰
```

## CASE 3

```text
Satellite가 회전하거나 위치가 변함
        ↓
Astrobee가 주변에서 관찰
        ↓
Camera Feed에 Satellite가 계속 보이도록 구성
```

이 테스트에서 Astrobee가 Angular Velocity나 Docking Feasibility를 계산하지 않는다.

---

# 16. Astrobee 모델 관련

먼저 현재 프로젝트 안에 사용할 수 있는 Astrobee 모델이 존재하는지 확인한다.

이미 Astrobee Asset / USD / URDF가 존재한다면 그것을 우선 재사용한다.

불필요하게 새로운 Astrobee Asset을 다시 만들지 않는다.

Astrobee 모델이 전혀 없는 경우에만 NASA Astrobee 공식 Robot Description / URDF / Mesh 사용을 검토한다.

임의로 다른 Quadcopter Asset을 Astrobee라고 이름만 변경해서 사용하지 않는다.

---

# 17. 기존 Satellite Asset 수정 금지

현재 프로젝트의 Satellite Asset은 그대로 유지한다.

특히 다음을 추가하지 않는다.

- AprilTag
- ArUco
- Marker Mesh
- Capture Marker
- Inspection Target
- 시각적 기준점

Astrobee 정찰 기능 때문에 Satellite 외형을 변경하지 않는다.

---

# 18. 기존 Canadarm3 / MEP 시스템 연결

Astrobee와 Canadarm3의 역할을 명확히 분리한다.

Astrobee:
- Target Satellite 주변 관찰
- Camera Observation Feed 제공
- Web Monitoring으로 영상 전달

Canadarm3:
- MEP Capture
- MEP 이동

MEP:
- Satellite Approach
- Docking

Astrobee는 Canadarm3의 Capture Point를 계산하거나 Mission 진행 여부를 결정하지 않는다.

# 19. 기존 Docking Pipeline 유지

현재 구현된 Docking state는 최대한 유지한다.

예:

```text
DOCK_TARGET_ACQUIRE
PRE_DOCK_APPROACH
XY_ALIGN
ORIENTATION_ALIGN
ALIGNMENT_CHECK
Z_APPROACH
FINAL_INSERTION
DOCK_READY
DOCKED
```

Astrobee 기능은 이 Docking Pipeline 자체를 다시 작성하는 것이 아니다.

Astrobee는 Docking Pipeline에 직접 개입하지 않으며, Docking을 위한 판정도 수행하지 않는다.

# 20. Web UI 표시

기존 Web Monitoring UI가 있다면 Astrobee의 Camera Feed만 추가한다.

Astrobee의 상태값, 판정 결과, Angular Velocity, Rotation Status 등의 정보를 Web UI에 표시하지 않는다.

기존 Web UI 하단에 Camera 1, Camera 2가 있다면 그 오른쪽에 Astrobee Camera를 Camera 3으로 배치한다.

예:

```text
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│    Camera 1     │ │    Camera 2     │ │    Camera 3     │
│                 │ │                 │ │                 │
│ Existing Feed   │ │ Existing Feed   │ │ Astrobee Camera │
│                 │ │                 │ │ Observation Feed│
└─────────────────┘ └─────────────────┘ └─────────────────┘
```

Camera 3은 Astrobee가 바라보는 Target Satellite 및 주변 상황을 보여주는 관찰 영상이다.

Web UI에서 Astrobee가 별도의 판정을 수행하는 것처럼 보이게 하는 UI를 추가하지 않는다.

표시하지 않는 항목:

- Astrobee Status
- Satellite Status
- Angular Velocity
- Rotation Status
- Docking Feasibility
- DOCKING_AVAILABLE
- DOCKING_UNAVAILABLE
- 기타 Astrobee 자체 계산 결과

Web UI에서 Astrobee와 관련하여 필요한 것은 Camera 3 영상 표시뿐이다.

# 21. ROS2 구조

Astrobee의 Camera Feed를 다중 PC에서도 볼 수 있도록 ROS2로 전달하는 구조를 만든다.

Astrobee의 상태값, 판정 결과, 관찰 계산값은 ROS2로 발행하지 않는다. 발행하는 것은 Camera Image뿐이다.

예시 Topic 개념:

```text
/astrobee/camera/image
```

단, 실제 프로젝트에 이미 Naming Convention이 존재하면 기존 규칙을 우선 따른다.

---

# 22. 다중 PC 구조

하나의 Simulation Authority를 유지하는 것을 우선한다.

권장 구조:

```text
GPU PC

Isaac Sim

├── Satellite
├── MEP
├── Canadarm3
├── Astrobee
├── Satellite Rotation State
└── Astrobee Inspection

           │
           │ ROS2 DDS
           ▼

Monitoring PC

├── ROS2 Subscriber
├── FastAPI
└── Web UI
```

두 PC에서 각각 독립적인 Physics World를 만들지 않는다.

Satellite와 Astrobee의 Simulation State는 하나의 Isaac Sim World에서 관리한다.

---

# 22-1. DB 구조 수정 금지 (Astrobee 관련)

Astrobee 관련 기능은 DB를 만들지 않는다. 기존 DB 구조를 어떤 형태로든 수정하지 않는다.

- 기존 DB(Firebase Firestore의 `simulation_sessions`, `simulation_sessions/{session_id}/session_telemetry`)의 컬렉션, 문서 구조, 필드(컬럼)를 추가, 삭제, 변경하지 않는다.
- Astrobee 전용 컬렉션, 테이블, 필드를 만들지 않는다.
- Astrobee의 Camera Feed, 위치, 자세, 이동 상태, 관찰 단계값(`ASTROBEE_*`)을 DB에 저장하지 않는다.
- 기존 DB 브리지(`project/scripts/firebase_bridge.py`)에 Astrobee 토픽(`/astrobee/*`)을 구독하거나 기록하는 코드를 추가하지 않는다.
- Astrobee가 필요로 하는 데이터는 ROS2 Camera Feed(Web Camera 3 표시용)뿐이며, DB 저장 대상이 아니다.
- 기존 시뮬레이터 → ROS2 → DB 기록 경로(포획, 도킹 텔레메트리)는 Astrobee 작업 때문에 변경하지 않는다.

---

# 23. 구현 우선순위

## Phase 1 — 현재 프로젝트 확인

- [ ] 기존 Satellite Prim 확인
- [ ] 기존 Docking Port 확인
- [ ] 기존 Dock Frame 확인
- [ ] 기존 MEP Docking 코드 확인
- [ ] Astrobee 모델 존재 여부 확인

## Phase 2 — Astrobee 이동

- [ ] Astrobee 정상 표시
- [ ] 6-DOF 이동 가능
- [ ] Satellite 접근
- [ ] Observation Position 이동
- [ ] Satellite 주변 이동

## Phase 3 — Camera Observation

- [ ] Astrobee Camera 확인
- [ ] Camera가 Satellite를 볼 수 있도록 구성
- [ ] Satellite 및 주변 상황 지속 관찰
- [ ] Camera Feed 확인
- [ ] 필요 시 ROS2 Image Publish

## Phase 4 — Web Monitoring 연결

- [ ] Astrobee Camera Feed Publish
- [ ] Web에서 Astrobee Camera Feed 확인
- [ ] 기존 Camera 1, Camera 2 오른쪽에 Camera 3 배치
- [ ] Astrobee 상태/판정 UI는 추가하지 않음

## Phase 5 — 기존 Mission 유지

- [ ] 기존 MEP Capture 유지
- [ ] 기존 MEP Attachment 유지
- [ ] 기존 Satellite Docking 유지
- [ ] Astrobee가 기존 Mission Pipeline을 변경하지 않는지 확인

## Phase 6 — ROS2 / Web

- [ ] Astrobee Camera Image Publish
- [ ] 기존 Naming Convention에 맞게 Topic 구성
- [ ] Camera 3 Web Feed 확인

# 24. 구현 시 가장 중요한 제한

1. AprilTag를 추가하지 않는다.
2. 기존 Marker를 Astrobee 검사 기준으로 사용하지 않는다.
3. Satellite에 새로운 Inspection Asset을 추가하지 않는다.
4. Astrobee를 MEP Capture Robot으로 사용하지 않는다.
5. Astrobee는 Satellite와 주변 상황을 카메라로 지속 관찰한다.
6. Astrobee는 어떤 값도 계산하거나 판단하지 않는다.
7. Astrobee의 결과는 Camera Observation / Monitoring Feed뿐이다.
8. Web UI에는 Astrobee Camera Feed만 추가한다.
9. 기존 Web UI의 Camera 1, Camera 2 오른쪽에 Astrobee Camera를 Camera 3으로 배치한다.
10. Web UI에 Astrobee Status나 판정 결과를 추가하지 않는다.
11. DOCKING_AVAILABLE / DOCKING_UNAVAILABLE을 Astrobee의 결과로 생성하지 않는다.
12. 기존 Canadarm3 MEP Capture 코드는 최대한 유지한다.
13. 기존 MEP Attachment 기능을 유지한다.
14. 기존 Satellite Docking 코드를 다시 작성하지 않는다.
15. 실제 구현하지 않은 Vision 기능을 완료했다고 표시하지 않는다.
16. Ground Truth와 Camera Measurement를 구분한다.
17. 기존 코드가 정상 동작한다면 불필요하게 리팩터링하거나 전체 구조를 변경하지 않는다.
18. Astrobee 관련 부분은 DB를 만들지 않는다. 기존 DB 구조(컬렉션, 문서, 필드)를 수정하지 않고, DB 브리지에도 Astrobee 항목을 추가하지 않는다. (22-1 참고)

# 25. 실행 기록

모든 작업 내용을 Markdown에 기록한다.

예:

```text
docs/astrobee_progress.md
```

내용:

```markdown
# Astrobee Satellite Inspection Progress

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

실제로 수행하지 않은 명령이나 테스트를 수행했다고 기록하지 않는다.

---

# 26. 최종 시연 시나리오

```text
[1] Mission Start
        ↓
[2] Astrobee Observation Start
        ↓
[3] Astrobee Satellite Approach
        ↓
[4] Satellite가 카메라에 보이도록 위치 조정
        ↓
[5] Astrobee가 Satellite 주변을 자유비행
        ↓
[6] Satellite + 주변 상황 지속 관찰
        ↓
[7] Astrobee Camera Feed를 Web의 Camera 3으로 표시
        ↓
[8] 기존 Mission Pipeline은 Astrobee와 독립적으로 진행
```

중요:

Astrobee는 위 과정에서 어떤 계산이나 판단도 수행하지 않는다.

# 27. 최종 핵심 메시지

이번 기능의 핵심은 다음과 같다.

```text
Astrobee
=
Satellite Observation / Monitoring Robot
```

Astrobee가 Target Satellite 주변을 자유비행하면서 Satellite와 주변 상황이 카메라에 계속 보이도록 한다.

카메라가 없다면 Astrobee에 카메라를 추가한다.

Astrobee는 다음 역할을 하지 않는다.

```text
계산 ❌
판단 ❌
Docking Feasibility 판단 ❌
MEP Capture 판단 ❌
Canadarm3 제어 ❌
Mission 결정 ❌
```

Astrobee의 역할은 단순하다.

```text
Astrobee

"위성과 주변 상황을 계속 보여준다."
```

↓

```text
Astrobee Camera
+
Continuous Monitoring
```

↓

```text
ROS2 Camera Feed
+
Web Camera 3
```

판정이나 Mission 제어가 필요한 경우에는 Astrobee와 분리된 별도의 시스템에서 수행한다.

이 역할 분리를 반드시 유지한다.
