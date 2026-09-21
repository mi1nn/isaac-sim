# Isaac Sim — Dynamic MEP Capture & Docking Pipeline 확장 구현

## 1. 현재 프로젝트 상태

현재 프로젝트에서는 다음 파이프라인의 실행과 성공을 이미 확인했다.

```text
MEP 부착
    ↓
MRV 이동
    ↓
Satellite 접근
    ↓
도킹
```

또한 MEP가 정지된 상태가 아니라 회전 및 유영하는 Dynamic 환경에서도 전체 파이프라인을 테스트했으며 성공을 확인했다.

현재 Dynamic 테스트에 사용한 실행 명령은 다음과 같다.

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag full_6dof --dock \
  --start_yaw_deg 15 \
  --set mep.motion_mode=six_dof \
  --set "mep.angular_velocity_rad_s=[0.005,-0.004,0.006]"
```

따라서 이번 구현에서는 MEP의 Dynamic motion을 제거하거나 정적인 상태로 변경하지 않는다.

---

# 2. 이번 작업의 목표

현재 성공적으로 동작하는 MEP Capture → Transport → Docking 파이프라인의 앞부분에 다음 과정을 추가한다.

```text
[초기 상태]
MRV + 접힌 Robot Arm
        ↓
[1] MEP 접근
MRV가 MEP와 충분한 거리를 두고 배치
        ↓
MRV가 X축 방향으로 1차 평행이동
        ↓
MRV가 Z축 방향으로 2차 평행이동
        ↓
[2] 로봇팔 전개
MRV 이동 완료 후 접힌 Robot Arm 전개
        ↓
[3] AprilTag 탐색
카메라로 움직이는 MEP의 AprilTag 탐색
        ↓
[4] AprilTag 기반 상대 위치 추정
MEP의 현재 위치/자세 확인
        ↓
[5] MEP 접근 및 Capture
기존 Capture 로직 연결
        ↓
[6] 기존 Transport
MRV + MEP 이동
        ↓
[7] 기존 Docking
Satellite 접근 및 도킹
```

최종적으로 다음과 같은 하나의 자동화 Pipeline을 만든다.

```text
START
  ↓
INITIALIZE
  ↓
ROBOT ARM FOLDED
  ↓
MRV MOVE — STEP 1
  ↓
MRV MOVE — STEP 2
  ↓
ARM DEPLOY
  ↓
APRILTAG SEARCH
  ↓
APRILTAG DETECTED
  ↓
MEP RELATIVE POSE ESTIMATION
  ↓
MEP CAPTURE
  ↓
EXISTING TRANSPORT
  ↓
EXISTING DOCKING
  ↓
COMPLETE
```

---

# 3. 가장 중요한 조건

이번 작업에서 반드시 다음 조건을 유지한다.

## MEP

MEP는 정지된 물체가 아니다.

다음 Dynamic motion을 유지한다.

```text
motion_mode = six_dof
```

검증된 테스트 조건은 다음과 같다.

```text
start_yaw_deg = 15

angular_velocity_rad_s =
[0.005, -0.004, 0.006]
```

즉 MEP는 현재 프로젝트에서 검증된 Dynamic 상태 그대로 유지한다.

- 평행이동
- Roll
- Pitch
- Yaw

등의 6-DoF 운동을 고려할 수 있는 상태를 유지한다.

현재 이미 검증된 Dynamic MEP 환경을 이번 구현에서 Static 환경으로 변경하지 않는다.

---

# 4. MRV 초기 위치

이번 단계에서 중요한 요구사항이다.

MRV를 처음부터 MEP 바로 옆에 배치하지 않는다.

MRV가 실제로 MEP를 향해 이동하는 모습이 시뮬레이션 화면에서 명확하게 보이도록 적당한 초기 거리를 확보한다.

특히 단순히 한 번의 직선 이동으로 MEP에 접근하지 않는다.

이번 시연에서는 MRV가 최소 2개의 translation 단계로 이동하는 모습이 보이도록 구성한다.

예:

```text
초기

MRV
 │
 │
 │
 │
 │
 │                         MEP
 │
 └─────────────────────────


Step 1 — X축 이동

MRV ──────────────────────→
                            MEP


Step 2 — Z축 이동

                            MEP
                             ↑
                             │
                             │
                        MRV ─┘
```

실제 축 방향은 현재 USD/World 좌표계를 먼저 확인한 후 결정한다.

중요한 것은 다음과 같은 "2단계 이동"이 시각적으로 보이는 것이다.

```text
1차 이동
X축 또는 적절한 수평 방향으로 이동
        ↓
2차 이동
Z축 또는 적절한 수직 방향으로 이동
        ↓
MEP 접근 위치
```

단순히 하나의 목표 좌표로 한 번에 직선 이동하지 않는다.

---

# 5. MRV 2단계 접근 동작

MRV 접근은 최소 2개의 명확한 Translation Step으로 구성한다.

권장 구조:

```text
MOVE_MRV_STEP_1
        ↓
STEP_1_POSITION_REACHED
        ↓
MOVE_MRV_STEP_2
        ↓
STEP_2_POSITION_REACHED
        ↓
ARM_DEPLOY
```

예를 들어 World Frame 기준으로:

```text
Step 1:
X 방향으로 이동

Step 2:
Z 방향으로 이동
```

한다.

단, 현재 프로젝트의 World 좌표계를 확인한 후 실제 이동 축을 결정한다.

예:

```python
MRV_APPROACH_STEP_1 = [x1, y1, z0]
MRV_APPROACH_STEP_2 = [x1, y1, z1]
```

처럼 단계별 target을 별도로 관리할 수 있다.

또는 기존 프로젝트의 좌표계와 이동 구조에 맞는 방식으로 구현한다.

---

# 6. 2단계 이동의 목적

이번 2단계 이동은 단순한 코드상의 구분이 아니라 시연 화면에서 실제 접근 동작이 명확하게 보이도록 하기 위한 것이다.

따라서 다음과 같은 구현은 피한다.

```text
MRV
  └──────────────→ MEP
```

한 번의 직선 이동으로 바로 접근하는 방식.

대신:

```text
MRV
  └──────────────→
                   │
                   │
                   ↓
                  MEP
```

처럼 최소 2개의 이동 단계가 명확하게 나타나도록 한다.

예를 들어:

```text
1. X축으로 수평 이동
2. Z축으로 수직 이동
3. 최종 MEP 접근 위치에서 정지
```

이후 로봇팔을 전개한다.

---

# 7. 이동 중 MEP Dynamic Motion 유지

MRV가 1차 이동하는 동안에도 MEP는 계속 움직인다.

MRV가 2차 이동하는 동안에도 MEP는 계속 움직인다.

즉:

```text
MRV Step 1
    +
MEP Dynamic Motion

        ↓

MRV Step 2
    +
MEP Dynamic Motion

        ↓

Arm Deploy
    +
MEP Dynamic Motion
```

상태가 되어야 한다.

MRV 이동을 위해 MEP를 정지시키지 않는다.

다음과 같은 방식은 사용하지 않는다.

```python
mep_velocity = 0
mep_angular_velocity = 0
```

또는

```python
freeze_mep()
```

현재 검증된 `six_dof` motion을 유지한다.

---

# 8. Step 1 — 초기화

Simulation 시작 시 다음 상태가 되어야 한다.

```text
MRV
    ↓
MEP와 충분한 거리

Robot Arm
    ↓
Folded

MEP
    ↓
Dynamic six_dof
```

초기화 단계에서 다음을 설정한다.

- MRV 초기 위치
- Robot Arm folded pose
- Camera
- AprilTag
- MEP Dynamic motion
- Capture 관련 설정
- 기존 Docking 관련 설정

---

# 9. Step 2 — Robot Arm Folded

Simulation 시작 시 Robot Arm은 펼쳐져 있으면 안 된다.

처음에는 접힌 상태여야 한다.

```text
START

MRV
 └── Robot Arm = FOLDED
```

현재 프로젝트에서 사용 중인 Canadarm3 모델의 실제 joint 구조를 확인한다.

반드시 다음을 확인한다.

- 실제 joint 이름
- 실제 joint index
- joint limit
- 기존 home pose
- 기존 arm control 방식

이미 folded/home pose가 구현되어 있다면 반드시 재사용한다.

임의의 joint 값을 추측하여 작성하지 않는다.

---

# 10. Step 3 — MRV 1차 이동

로봇팔이 접힌 상태에서 MRV가 MEP 방향으로 1차 이동한다.

예:

```text
초기

MRV ------------------------------ MEP


1차 이동

        MRV --------------------→ MEP
```

1차 이동은 X축 또는 현재 World Frame에서 적절한 수평축을 사용한다.

이동 완료 후 반드시 위치 도달 여부를 확인한다.

```text
MOVE_MRV_STEP_1
        ↓
STEP_1_POSITION_REACHED
```

가능하면 기존 프로젝트의 이동 완료 판정 방식을 재사용한다.

---

# 11. Step 4 — MRV 2차 이동

1차 이동이 완료되면 2차 이동을 수행한다.

예:

```text
Step 1
X축 이동
        ↓
Step 1 완료
        ↓
Step 2
Z축 이동
        ↓
Step 2 완료
```

개념:

```text
MRV
  ─────────────────→
                    │
                    │
                    ↓
                   Target
```

이 단계 역시 실제 이동 과정이 시뮬레이션 화면에서 보이도록 한다.

2차 이동 완료 후에만 Robot Arm을 전개한다.

---

# 12. MRV 이동 완료 조건

가능하면 단순한 `sleep()`이나 고정 시간으로 이동 완료를 판단하지 않는다.

목표 위치와 현재 위치의 차이를 기준으로 판단한다.

예:

```python
distance = np.linalg.norm(
    current_position - target_position
)

if distance < POSITION_TOLERANCE:
    move_complete = True
```

기존 프로젝트에 이미 이동 완료 판단 로직이 있다면 해당 로직을 재사용한다.

---

# 13. Step 5 — MRV 도착 후 Robot Arm 전개

MRV가 2단계 이동을 모두 완료한 후에만 Robot Arm을 전개한다.

순서는 반드시 다음과 같다.

```text
Robot Arm Folded
        ↓
MRV Translation Step 1
        ↓
MRV Translation Step 2
        ↓
MRV Position Reached
        ↓
Robot Arm Deploy
```

MRV가 이동하는 도중 Robot Arm을 전개하지 않는다.

---

# 14. Robot Arm Deployment

Robot Arm은 접힌 상태에서 Capture 작업이 가능한 자세로 전개한다.

가능하면 joint interpolation을 사용한다.

```text
FOLDED
   ↓
DEPLOYING
   ↓
DEPLOYED
```

갑작스럽게 joint 값을 한 번에 변경하지 않는다.

가능하다면:

```python
q(t) = q_start + alpha * (q_target - q_start)
```

형태로 부드럽게 전개한다.

전개가 완료된 이후에만 AprilTag 탐색을 시작한다.

---

# 15. Step 6 — AprilTag 탐색

Robot Arm 전개가 완료되면 카메라를 통해 MEP의 AprilTag를 탐색한다.

현재 MEP가 움직이고 있기 때문에 AprilTag 역시 화면에서 계속 움직인다.

따라서 AprilTag 탐색은 한 번만 수행하는 것이 아니라 일정 시간 동안 계속 detection을 수행한다.

개념:

```text
Camera Image
     ↓
AprilTag Detector
     ↓
Detected?
 ┌───┴────┐
No       Yes
 ↓        ↓
Search   Pose
again
```

---

# 16. AprilTag ID

현재 프로젝트에 이미 정의된 AprilTag ID가 있다면 해당 설정을 재사용한다.

새로운 ID를 임의로 지정하지 않는다.

현재 프로젝트의 Tag configuration을 먼저 확인한다.

---

# 17. AprilTag → MEP 상대 위치

AprilTag를 찾으면 Camera Frame에서 Tag의 pose를 얻는다.

그 다음 필요한 좌표계 변환을 수행한다.

```text
Camera Frame
      ↓
AprilTag Pose
      ↓
Transform
      ↓
Robot / MRV Frame
      ↓
MEP Relative Pose
```

반드시 현재 USD hierarchy와 실제 frame 관계를 확인한다.

확인해야 할 좌표계:

```text
World
MRV
Robot Base
Robot End Effector
Camera
AprilTag
MEP
```

좌표축 방향을 임의로 가정하지 않는다.

---

# 18. Dynamic MEP의 Pose 추정

MEP가 회전하고 유영하기 때문에 AprilTag에서 얻은 pose를 이용해 현재 MEP의 상태를 계산할 수 있어야 한다.

최소한 다음 정보를 얻는다.

```text
MEP relative position
MEP relative orientation
```

이번 구현에서는 이를 통해:

```text
현재 MEP가 어디에 있는가?
현재 MEP가 어떤 방향을 향하고 있는가?
```

를 확인할 수 있어야 한다.

---

# 19. AprilTag 탐색 중 MEP가 움직이는 경우

AprilTag를 탐색하는 동안 MEP가 움직이는 것은 정상이다.

따라서 다음과 같이 동작한다.

```text
MEP 움직임
    ↓
Camera frame
    ↓
AprilTag detection
    ↓
Current tag pose
    ↓
Current MEP relative pose
```

Tag를 한 번 검출했다고 해서 과거의 위치를 계속 사용하는 방식으로 구현하지 않는다.

필요하다면 detection 결과를 계속 갱신한다.

---

# 20. Step 7 — MEP Capture 위치 접근

AprilTag 기반으로 MEP의 현재 위치를 확인한 후 Capture 위치로 접근한다.

현재 프로젝트에서 MEP의 Capture 대상은:

```text
gripper_fixture
```

이다.

따라서 기존 Capture 시스템이 `gripper_fixture`를 기준으로 동작한다면 해당 기준을 그대로 유지한다.

구조:

```text
AprilTag
   ↓
MEP Pose
   ↓
gripper_fixture 위치 계산
   ↓
Capture position
   ↓
Robot End Effector 접근
```

---

# 21. 기존 Capture 로직 재사용

현재 MEP Capture는 이미 정상 동작하는 것으로 검증되어 있다.

따라서 기존 Capture 로직을 새로 만들지 않는다.

이번 작업에서는:

```text
새로운 접근/인식 Pipeline
        ↓
기존 Capture
```

형태로 연결한다.

예:

```python
if april_tag_detected:
    mep_pose = estimate_mep_pose()
    move_to_capture_position(mep_pose)
    existing_mep_capture()
```

실제 함수명은 현재 프로젝트 구조에 맞게 사용한다.

---

# 22. Capture 조건

기존 프로젝트에서 사용하는 Capture 조건을 우선 유지한다.

현재 사용 중인 attachment / magnetic-like capture 방식이 있다면 이를 그대로 사용한다.

이번 작업에서 Capture 방식 자체를 변경하지 않는다.

변경 대상:

```text
접근 과정
```

유지 대상:

```text
Capture mechanism
```

---

# 23. Capture 이후

MEP Capture가 성공하면 기존에 검증된 Pipeline으로 연결한다.

```text
MEP Capture
    ↓
Transport
    ↓
Satellite Approach
    ↓
Docking
```

기존 도킹 로직은 가능한 한 수정하지 않는다.

---

# 24. 최종 Pipeline

최종 Pipeline은 다음과 같아야 한다.

```text
START
  │
  ▼
INITIALIZE
  │
  ├── MRV 초기 위치 설정
  ├── Robot Arm Folded
  ├── MEP Dynamic six_dof
  ├── Camera / AprilTag 준비
  │
  ▼
MOVE_MRV_STEP_1
  │
  │  X축 또는 적절한 수평 방향 이동
  │  MEP는 계속 유영 + 회전
  │
  ▼
STEP_1_POSITION_REACHED
  │
  ▼
MOVE_MRV_STEP_2
  │
  │  Z축 또는 적절한 수직 방향 이동
  │  MEP는 계속 유영 + 회전
  │
  ▼
STEP_2_POSITION_REACHED
  │
  ▼
ARM_DEPLOY
  │
  ▼
APRILTAG_SEARCH
  │
  ├── NOT DETECTED
  │       ↓
  │   계속 탐색
  │
  └── DETECTED
          ↓
    APRILTAG POSE
          ↓
    MEP RELATIVE POSE
          ↓
    gripper_fixture 위치 계산
          ↓
    CAPTURE APPROACH
          ↓
    MEP CAPTURE
          ↓
    EXISTING TRANSPORT
          ↓
    EXISTING DOCKING
          ↓
       COMPLETE
```

---

# 25. 이번 구현에서 하지 않을 것

이번 단계에서는 다음 기능을 새로 구현하지 않는다.

- 새로운 도킹 알고리즘
- 새로운 MEP Capture mechanism
- 궤도역학
- 실제 위성 자세제어
- 새로운 6-DoF controller
- 불필요한 동역학 재설계
- 기존 Docking 로직 재작성
- 기존 Capture 로직 재작성

MEP의 Dynamic motion은 유지하되, 이번 단계의 핵심은:

```text
MRV 접근
→ 1차 Translation
→ 2차 Translation
→ Robot Arm 전개
→ AprilTag 탐색
→ 상대 위치 추정
→ 기존 Capture
→ 기존 Transport
→ 기존 Docking
```

이다.

---

# 26. 코드 분석 우선

코드를 수정하기 전에 현재 프로젝트 구조를 먼저 분석한다.

반드시 확인할 것:

```text
project/
├── scripts/
│   └── vision_capture.py
│
├── ...
```

그리고 다음 기능의 실제 구현 위치를 찾는다.

```text
- scenario dynamic
- six_dof MEP motion
- MRV movement
- robot arm control
- robot joint configuration
- camera
- AprilTag
- MEP capture
- gripper_fixture
- transport
- docking
```

기존 구현이 있다면 최대한 재사용한다.

---

# 27. Dynamic 실행 설정 유지

현재 검증된 Dynamic 테스트 환경은 다음과 같다.

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag full_6dof --dock \
  --start_yaw_deg 15 \
  --set mep.motion_mode=six_dof \
  --set "mep.angular_velocity_rad_s=[0.005,-0.004,0.006]"
```

새로운 Pipeline에서도 이 Dynamic 조건을 유지할 수 있도록 구현한다.

가능하면 기존 CLI 옵션을 유지하고 새로운 옵션이 필요할 경우 기존 구조를 확장한다.

기존 옵션의 의미를 변경하지 않는다.

---

# 28. Configuration

다음 값은 configuration으로 분리하여 쉽게 조정할 수 있도록 한다.

```python
MRV_INITIAL_OFFSET
MRV_APPROACH_STEP_1
MRV_APPROACH_STEP_2
POSITION_TOLERANCE
ARM_DEPLOY_DURATION
APRILTAG_SEARCH_TIMEOUT
APRILTAG_ID
CAPTURE_DISTANCE
```

특히:

```text
MRV_INITIAL_OFFSET
MRV_APPROACH_STEP_1
MRV_APPROACH_STEP_2
```

는 시연 화면에서 MRV가 실제로 MEP까지 접근하는 모습이 충분히 보이도록 조정할 수 있어야 한다.

너무 멀어서 불필요하게 긴 시연이 되지 않도록 하고, 너무 가까워서 이동이 보이지 않는 상태도 피한다.

---

# 29. 시연 관점의 배치

이번 구현은 단순히 기능만 작동하는 것이 아니라 시뮬레이션 화면에서 Pipeline이 명확하게 보이는 것이 중요하다.

따라서 초기 배치는 다음과 같은 시각적 흐름이 보이도록 한다.

```text
[초기]

MRV
 └─ 접힌 Robot Arm


                                MEP
                           ↻ 유영 + 회전


        ↓ 1차 X축 이동


                MRV ─────────────→
                                  MEP


        ↓ 2차 Z축 이동


                                  MEP
                                   ↻
                                   ↑
                                   │
                                  MRV


        ↓ Robot Arm 전개


[Capture]

                                  MEP
                                   ↻
                                  AprilTag
                                     ↓
MRV ───────── Robot Arm ─────────► gripper_fixture


        ↓ Capture


[Transport]

MRV + MEP
        ↓


[Docking]

MRV + MEP
        ↓
Satellite
        ↓
Docking
```

초기 MRV와 MEP 사이의 거리는 2단계 이동이 실제 화면에서 명확하게 보이면서도 전체 시연 시간이 지나치게 길어지지 않도록 설정한다.

---

# 30. 진행 로그

이번 작업도 반드시 Markdown 진행 로그를 생성하거나 기존 진행 로그를 업데이트한다.

예:

```text
progress.md
```

내용:

```md
# Progress

## Existing Pipeline

- [x] MEP Capture verified
- [x] Dynamic MEP verified
- [x] Transport verified
- [x] Docking verified

## New Pipeline

- [ ] Initial MRV position
- [ ] Robot Arm folded pose
- [ ] MRV translation step 1
- [ ] MRV translation step 2
- [ ] Robot Arm deployment
- [ ] Camera configuration
- [ ] AprilTag detection
- [ ] AprilTag pose estimation
- [ ] MEP relative pose estimation
- [ ] gripper_fixture target calculation
- [ ] Existing MEP Capture integration
- [ ] Existing Transport integration
- [ ] Existing Docking integration

## Dynamic Configuration

```bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --tag full_6dof --dock \
  --start_yaw_deg 15 \
  --set mep.motion_mode=six_dof \
  --set "mep.angular_velocity_rad_s=[0.005,-0.004,0.006]"
```

## Notes

### Existing verified behavior

...

### New implementation

...

### Modified files

...

### Runtime verification

...
```

실제로 작업한 항목만 `[x]`로 변경한다.

실행하지 않은 항목을 성공했다고 기록하지 않는다.

---

# 31. 현재 Isaac Sim을 실행할 수 없는 경우

현재 환경에서 Isaac Sim 실행이 불가능하다면 실제 Runtime 테스트를 수행하지 않는다.

대신 다음 작업까지 수행한다.

```text
1. 현재 코드 구조 분석
2. 기존 Dynamic MEP 구현 확인
3. 기존 Capture 구현 확인
4. 기존 Docking 구현 확인
5. Folded Arm 상태 구현
6. MRV 초기 위치 설정
7. MRV 1차 이동 구현
8. MRV 2차 이동 구현
9. Arm Deployment 단계 구현
10. AprilTag Detection 단계 구현
11. Pose transformation 구현
12. 기존 Capture 연결
13. 기존 Transport 연결
14. 기존 Docking 연결
15. 코드 정적 검토
16. progress.md 업데이트
```

실행하지 않은 것을 검증 완료라고 표현하지 않는다.

예:

```text
Implementation: Complete
Static Review: Complete
Runtime Verification: Pending
```

---

# 32. 완료 조건

다음 조건을 만족하면 이번 코드 구현 작업을 완료한다.

## 필수 구현

- [ ] MEP Dynamic six_dof motion 유지
- [ ] MRV가 MEP와 충분히 떨어진 위치에서 시작
- [ ] MRV 1차 Translation 구현
- [ ] MRV 2차 Translation 구현
- [ ] 두 이동 단계가 시뮬레이션 화면에서 명확하게 보이는 초기 거리 확보
- [ ] 1차 이동 완료 후 2차 이동 시작
- [ ] 2차 이동 완료 후 Robot Arm 전개
- [ ] Robot Arm 초기 상태가 Folded
- [ ] Camera 기반 AprilTag detection
- [ ] Dynamic MEP의 현재 AprilTag pose 추정
- [ ] AprilTag → MEP relative pose 변환
- [ ] `gripper_fixture` 위치 계산
- [ ] 기존 MEP Capture 로직 연결
- [ ] 기존 Transport 로직 연결
- [ ] 기존 Docking 로직 연결
- [ ] 상태별 로그 출력
- [ ] progress.md 업데이트

## 기존 기능 보호

다음 기존 기능은 최대한 변경하지 않는다.

```text
MEP Capture mechanism
Transport
Satellite Approach
Docking
Dynamic MEP motion
```

---

# 33. 최종 작업 원칙

이번 작업에서 가장 중요한 것은 기존에 성공한 시스템을 망가뜨리지 않는 것이다.

현재:

```text
MEP Capture
    ↓
Transport
    ↓
Docking
```

은 이미 검증되어 있다.

따라서 이번 구현은 다음과 같이 앞부분을 확장한다.

```text
기존

MEP Capture
    ↓
Transport
    ↓
Docking
```

↓

```text
변경 후

Robot Arm Folded
    ↓
MRV 초기 위치
    ↓
MRV 1차 평행이동
    ↓
MRV 2차 평행이동
    ↓
Robot Arm Deploy
    ↓
AprilTag Search
    ↓
AprilTag Pose
    ↓
MEP Relative Pose
    ↓
기존 MEP Capture
    ↓
기존 Transport
    ↓
기존 Docking
```

특히 MRV 접근은 한 번의 직선 이동이 아니라 최소 2개의 translation 단계로 구성하여 시연 화면에서 "MRV가 MEP를 향해 접근하고 있다"는 동작이 명확하게 보이도록 한다.

즉:

```text
1차: X축 방향 이동
        ↓
2차: Z축 방향 이동
        ↓
Robot Arm 전개
        ↓
AprilTag 탐색
        ↓
MEP Capture
        ↓
Docking
```

단, X/Z 축은 실제 USD World Frame을 확인한 뒤 적절한 축으로 결정한다.

기존에 성공한 Dynamic MEP 및 Capture/Docking 로직은 최대한 그대로 보존하고, 이번 단계에서는 "접근 → 전개 → 인식 → 위치 추정 → 기존 부착"의 앞단 확장에 집중한다.

## 10. MRV Thruster Visual Effect (VFX) — 반드시 기존 파이프라인과 분리

### 10.1 구현 목적

MRV가 MEP를 향해 평행이동하는 장면에서 실제 우주선 추진기가 작동하는 것처럼 보이는 시각적 효과를 추가한다.

단, 현재 MRV 에셋은 하나의 통합 에셋으로 구성되어 있고 기존 `MRV 이동 → MEP Capture → Transport → Docking` 파이프라인은 이미 검증되었으므로 MRV 원본 USD/에셋 자체를 수정하지 않는다.

핵심 원칙:

```text
기존 MRV Asset
    │
    ├── 기존 Transform / Physics / Robot Arm
    ├── 기존 MRV 이동 로직
    └── 기존 Capture / Transport / Docking
              │
              │ 변경하지 않음
              ▼
      별도 Thruster VFX Layer
              │
              ├── MOVE_MRV_STEP_1 → VFX ON
              ├── STEP_1 종료 → VFX OFF
              ├── MOVE_MRV_STEP_2 → VFX ON
              └── STEP_2 종료 → VFX OFF
```

### 10.2 절대 변경하지 말아야 할 것

다음 기존 기능은 VFX 구현 때문에 수정하거나 재작성하지 않는다.

- MRV 원본 USD 에셋
- MRV의 기존 Physics 설정
- MRV의 실제 이동/평행이동 로직
- MEP의 Dynamic Motion (`six_dof`)
- AprilTag Detection
- AprilTag Pose Estimation
- `gripper_fixture` 기반 기존 MEP Capture
- 기존 MEP Transport 로직
- 기존 Satellite Docking 로직

특히 Thruster VFX를 실제 추력(force/thrust)으로 사용하지 않는다.

MRV가 이동하는 실제 원인은 기존 이동 로직이며, 플룸은 오직 시각 효과로만 동작해야 한다.

### 10.3 VFX 구현 방식

가장 안전한 방식은 MRV와 별도의 Prim 또는 별도 USD 기반 VFX를 런타임에 생성/연결하는 것이다.

권장 개념:

```text
/World/MRV
/World/MRV_Thruster_VFX
```

또는 런타임 생성 시:

```text
MRV
 └── runtime-generated ThrusterVFX
      ├── Plume_1
      ├── Plume_2
      └── ...
```

단, 기존 MRV USD 파일을 직접 수정해서 VFX를 저장하는 방식은 피한다.

가능하다면 VFX의 기준 Transform을 MRV의 현재 Transform에 따라 갱신하거나, 안전하게 MRV를 따라 움직이는 별도 VFX Transform을 사용한다.

### 10.4 Thruster 위치 결정

Thruster 위치와 방향을 코드에서 임의로 추측하지 않는다.

먼저 현재 USD의 MRV 구조를 확인하여 다음을 판단한다.

1. 실제 thruster/nozzle 관련 Prim이 존재하는지 확인
2. 존재한다면 위치와 방향을 확인
3. 실제 thruster Prim을 변경하지 않고 VFX의 기준점으로만 활용
4. 실제 thruster 정보가 없거나 찾기 어려우면 MRV 외형을 기준으로 합리적인 위치에 별도 VFX 기준점을 배치
5. VFX 기준점이 MRV의 외부에 노출되어 보이는지 확인
6. 카메라가 AprilTag를 바라볼 때 VFX가 Tag/MEP를 가리지 않는지 확인

실제 MRV 에셋의 구조를 확인하기 전에는 특정 Prim 이름이나 특정 nozzle 위치를 하드코딩하지 않는다.

### 10.5 VFX 표현

처음부터 복잡한 물리 기반 추진기 시뮬레이션을 구현하지 않는다.

데모에서 추진기 작동이 명확하게 보이는 정도의 가벼운 시각 효과를 우선한다.

가능한 구현:

- Particle System / GPU Particle 기반 plume
- 반투명 plume mesh
- 간단한 emissive/translucent material
- 기존 Isaac Sim에서 사용 가능한 VFX 방식

권장 시각적 특성:

- nozzle 근처는 상대적으로 밝게 표현
- nozzle에서 멀어질수록 투명해짐
- 너무 길거나 크게 만들지 않음
- 약간의 불규칙한 flicker 또는 길이 변화는 선택 사항
- MRV 전체를 가릴 정도의 크기는 사용하지 않음

### 10.6 이동 단계와 VFX 동기화

현재 MRV 접근은 최소 두 단계의 평행이동으로 구성한다.

```text
MRV Initial Position
        ↓
MOVE_MRV_STEP_1
        ↓
MOVE_MRV_STEP_1 완료
        ↓
MOVE_MRV_STEP_2
        ↓
MOVE_MRV_STEP_2 완료
        ↓
Arm Deploy
```

VFX는 다음과 같이 연결한다.

```text
IDLE
→ Plume OFF

MOVE_MRV_STEP_1
→ Plume ON
→ Step 1 이동 방향에 맞는 VFX 방향 사용

STEP_1 완료
→ Plume OFF

MOVE_MRV_STEP_2
→ Plume ON
→ Step 2 이동 방향에 맞는 VFX 방향 사용

STEP_2 완료
→ Plume OFF

ARM_DEPLOY
→ Plume OFF

APRILTAG_SEARCH
→ Plume OFF

CAPTURE
→ Plume OFF
```

중요:

- 실제 MRV 이동 방향과 반대 방향으로 plume이 분출되는 것처럼 보여야 한다.
- Step 1과 Step 2의 이동 방향이 다르면 각각 적절한 VFX 방향을 사용한다.
- World Frame의 실제 축 방향을 먼저 확인한 뒤 구현한다.
- 단순히 `+X`, `+Z`라고 가정하지 않는다.
- MRV의 실제 이동 벡터를 기준으로 plume 방향을 계산할 수 있다면 이를 우선 사용한다.

### 10.7 상태 기반 VFX 제어

기존 이동 상태를 활용하여 VFX를 독립적으로 제어한다.

예시 개념:

```python
if state == "MOVE_MRV_STEP_1":
    plume.set_visible(True)

elif state == "MOVE_MRV_STEP_2":
    plume.set_visible(True)

else:
    plume.set_visible(False)
```

단, 위 코드는 개념 예시이며 현재 프로젝트의 실제 state 관리 구조에 맞게 구현한다.

기존 이동 코드를 VFX 코드에 맞추기 위해 재작성하지 않는다.

가능하면 다음처럼 분리한다.

```text
MRV Motion Controller
        │
        ├── 실제 MRV 이동
        │
        └── movement state
                 │
                 ▼
          Thruster VFX Controller
```

### 10.8 실패 격리

Thruster VFX가 생성되지 않거나 렌더링 문제가 발생하더라도 전체 파이프라인이 중단되면 안 된다.

즉:

```text
VFX 성공
→ 정상 진행

VFX 실패
→ 경고 기록
→ VFX 없이 기존 MRV 이동 계속
→ Capture / Transport / Docking 계속
```

가능하면 VFX 초기화/생성 실패를 예외 처리하여 기존 파이프라인의 핵심 제어 흐름에 영향을 주지 않도록 한다.

### 10.9 성능 및 인식 영향

VFX는 데모용 시각 효과이므로 과도한 Particle 수나 복잡한 Shader를 사용하지 않는다.

특히 다음을 확인한다.

- Simulation FPS가 불필요하게 떨어지지 않는지
- AprilTag 카메라 영상에 plume이 과도하게 들어가지 않는지
- MEP의 AprilTag가 가려지지 않는지
- MRV/Arm의 시각적 구조가 plume 때문에 식별하기 어려워지지 않는지
- VFX가 Physics collision이나 rigid body에 영향을 주지 않는지

VFX는 collision geometry를 갖지 않는 순수 시각 효과로 유지하는 것을 우선한다.

### 10.10 최종 파이프라인

최종 시연 흐름은 다음과 같이 구성한다.

```text
MRV Arm Folded
      ↓
MRV Initial Position
      ↓
MRV MOVE STEP 1
      └── Thruster VFX ON
      ↓
MOVE STEP 1 Complete
      └── Thruster VFX OFF
      ↓
MRV MOVE STEP 2
      └── Thruster VFX ON
      ↓
MOVE STEP 2 Complete
      └── Thruster VFX OFF
      ↓
Robot Arm Deploy
      ↓
AprilTag Search
      ↓
AprilTag Detection
      ↓
MEP Relative Pose Estimation
      ↓
Existing MEP Capture (`gripper_fixture`)
      ↓
Existing MEP Transport
      ↓
Existing Satellite Docking
```

### 10.11 검증 기준

현재 Isaac Sim을 실행할 수 없는 상태라면 런타임 동작을 완료했다고 표시하지 않는다.

구현 후 정적 검토에서 확인할 항목:

- [ ] MRV 원본 USD를 직접 수정하지 않았는가?
- [ ] VFX가 별도 Prim/레이어로 분리되어 있는가?
- [ ] 실제 MRV 이동 로직과 VFX 로직이 분리되어 있는가?
- [ ] Thruster VFX가 실제 Physics Force를 발생시키지 않는가?
- [ ] Step 1 이동 시 VFX ON/OFF 상태가 연결되어 있는가?
- [ ] Step 2 이동 시 VFX ON/OFF 상태가 연결되어 있는가?
- [ ] 이동 종료 후 VFX가 OFF 되는가?
- [ ] Arm Deploy 이후 VFX가 OFF 상태인가?
- [ ] AprilTag Detection에 VFX가 방해되지 않도록 배치되었는가?
- [ ] VFX 생성 실패가 전체 파이프라인 실패로 이어지지 않는가?
- [ ] 기존 Dynamic MEP / Capture / Transport / Docking 로직이 변경되지 않았는가?

런타임 검증이 가능한 시점에는 다음을 추가 확인한다.

- [ ] MRV 이동 시 실제로 plume이 표시되는가?
- [ ] Step 1과 Step 2에서 plume 방향이 실제 이동 방향과 일치하는가?
- [ ] MRV 정지 시 plume이 사라지는가?
- [ ] Arm Deploy 시 plume이 꺼져 있는가?
- [ ] AprilTag Detection 성공률에 문제가 없는가?
- [ ] 기존 Capture → Transport → Docking 성공 여부가 유지되는가?
