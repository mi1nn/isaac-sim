# Task Prompt: MRV 기반 무중력 유영 MEP(3톤) AprilTag 비전 포획 시스템 구현 및 검증 — Phase 1

## 0. 최우선 개발 원칙

Isaac Sim / Space Robotics Bench 환경에서 MRV(Mission Robotic Vehicle)에 탑재된 Canadarm3(7-DoF) 로봇팔을 이용하여 무중력 상태에서 회전 없이 단순 선형 유영하는 3,000 kg MEP(Mission Extension Pod)를 포획하는 시스템을 우선 구현한다. 초기 단계에서는 MEP에 roll / pitch / yaw 회전을 주지 않으며, 해당 단순 유영 환경에서 Magnetic Attachment / FixedJoint 기반 흡착·고정과 이후 도킹 동작을 안정적으로 구현·검증하는 것을 우선한다.

이번 Phase 1의 핵심 목표는 다음과 같다.

> **개발 우선순위 변경:** 우선은 MEP에 roll / pitch / yaw 회전을 주지 않고 단순 선형 유영만 진행한다. 이 환경에서 Canadarm3의 MEP 흡착(Magnetic Attachment / FixedJoint)과 이후 Client Satellite 도킹을 먼저 구현·검증한다. 회전 유영(tumbling)과 각속도 기반 자세 예측은 해당 기능이 안정화된 이후 확장 단계에서 추가한다.

> 손목 카메라로 4개의 AprilTag를 검출하고,
> 4개 AprilTag의 기하학적 관계를 이용하여
> MEP의 실제 물리 결합점인 `Cylinder_01`의 6-DoF Pose를 추정한 뒤,
> 현재 위치와 움직임을 기반으로 미래의 결합점 위치를 예측하고,
> Canadarm3의 EE를 예측 위치로 접근시킨 후,
> 기존에 구현되어 있는 Magnetic Attachment / FixedJoint 방식으로 MEP를 포획한다.

중요:

- Image-Based Visual Servoing(IBVS)을 주 제어 방식으로 사용하지 않는다.
- 이미지 픽셀 오차를 직접 최소화하는 방식으로 로봇팔을 제어하지 않는다.
- AprilTag는 MEP의 위치와 자세를 추정하기 위한 시각적 기준점으로 사용한다.
- 최종적으로 로봇팔이 추적해야 하는 대상은 AprilTag 자체가 아니라 `Cylinder_01`이다.
- `Cylinder_01`은 실제 물리적 결합점(Ground-Truth Docking Point)이다.
- 기존에 정상 동작했던 Magnetic Attachment / FixedJoint 결합 방식은 유지한다.
- AprilTag 기반 위치 추정 및 예측은 기존 Magnetic Attachment가 언제 활성화되어야 하는지를 결정하기 위한 상위 제어 계층으로 사용한다.

---

# 1. 개발 범위

## Phase 1에서 반드시 구현할 기능

1. 기존 USD 환경 확인
2. MEP 질량을 3,000 kg으로 설정
3. 무중력 환경 구성
4. MEP에 4개의 AprilTag 생성 및 배치
5. 기존 `Cylinder_01`을 물리적 결합점으로 유지
6. Canadarm3에 `cam_wrist` RGB 카메라 생성
7. 카메라 intrinsic parameter 설정 및 기록
8. AprilTag detection 구현
9. 4개 AprilTag의 corner 좌표 검출
10. 4개 Tag constellation을 이용한 PnP 기반 Pose 추정
11. Tag constellation 중심과 `Cylinder_01`의 관계를 이용하여 물리 결합점 Pose 계산
12. Camera Frame → Robot/MRV Frame → World Frame 좌표 변환
13. MEP 중심점의 현재 위치 및 자세 추적
14. MEP의 선속도 추정 (Phase 1에서는 roll / pitch / yaw 회전을 주지 않으므로 각속도 기반 운동 예측은 제외)
15. 미래 결합점 위치 예측
16. 예측된 결합점으로 Canadarm3 EE 접근
17. 접근 중 상대 거리 및 상대 속도 제어
18. 기존 Magnetic Attachment 방식으로 capture
19. 조건 만족 시 기존 `UsdPhysics.FixedJoint` 생성
20. Capture 이후 10초간 holding 테스트
21. CSV telemetry 기록
22. Debug visualization
23. 자동화 Test 1 / Test 2 / Test 3 수행
24. 최종 결과 Markdown 리포트 생성

---

# 2. Phase 1에서 구현하지 않는 기능

다음 기능은 이번 Phase 1 범위에서 제외한다.

- MRV 본체의 위치 재조정(Repositioning)
- MRV 전체의 translation maneuver
- MEP가 카메라 시야에서 완전히 사라지는 사각지대 상황
- 복수 카메라 기반 인식
- LiDAR 기반 MEP 인식
- 딥러닝 기반 객체 검출
- YOLO 등 객체 검출 모델
- Image-Based Visual Servoing(IBVS)
- end-to-end neural network control
- 복잡한 6-DoF 최적 제어기
- 실제 우주선의 docking mechanism 모델링
- MEP의 의도적인 roll / pitch / yaw 회전(tumbling)
- 각속도 기반 미래 orientation 예측

Phase 1에서는 AprilTag → Pose 추정 → 미래 중심점 예측 → EE 접근 → Magnetic Capture라는 단순하고 검증 가능한 구조를 우선 완성한다.

---

# 3. MEP 물리 설정

## 3.1 질량

MEP의 물리 질량은 반드시:

```text
3000 kg
```

으로 설정한다.

테스트 편의를 위해 1톤이나 그 이하로 낮추지 않는다.

3톤 질량에서 Canadarm3의 접근 및 capture가 안정적으로 수행되는지를 검증하는 것이 이번 Phase 1의 목적이다.

---

## 3.2 무중력

시뮬레이션 환경의 중력은:

```python
gravity = (0, 0, 0)
```

으로 설정한다.

MEP가 중력에 의해 떨어지지 않아야 한다.

---

## 3.3 MEP 초기 운동

MEP는 정지 상태가 아니라 천천히 유영하는 상태에서 테스트할 수 있어야 한다.

초기 선속도:

```text
0.01 ~ 0.02 m/s
```

초기 각속도:

```text
0 deg/s
```

Phase 1에서는 roll / pitch / yaw 회전을 주지 않는다. 회전(tumbling)은 단순 유영 상태에서 흡착 및 도킹이 안정적으로 구현된 이후의 확장 단계에서 적용한다.

단, 테스트를 재현할 수 있도록 다음 값을 configuration parameter로 분리한다.

```yaml
mep:
  mass_kg: 3000
  linear_velocity_mps: 0.01
  angular_velocity_deg_s: 0.0
```

---

# 4. Ground-Truth 물리 결합점

기존 USD에 존재하는:

```text
debris/gripper_fixture/Cylinder_01
```

을 실제 물리 결합점으로 사용한다.

이 Prim은:

```text
Ground-Truth Docking Point
```

이다.

중요:

- `Cylinder_01`은 삭제하지 않는다.
- 위치를 임의로 변경하지 않는다.
- AprilTag의 위치 계산 기준으로 사용한다.
- 최종 로봇팔 EE가 접근해야 하는 목표점이다.
- Magnetic Attachment의 실제 물리 결합 기준점으로 유지한다.

AprilTag가 인식하는 대상과 `Cylinder_01`은 서로 다른 개념이다.

---

# 5. AprilTag 배치

## 5.1 목적

MEP 부착면에 4개의 AprilTag를 정사각형 형태로 배치한다.

4개 Tag의 기하학적 중심이 정확하게:

```text
debris/gripper_fixture/Cylinder_01
```

의 중심과 일치해야 한다.

개념적으로 다음 구조를 사용한다.

```text
AprilTag 0 ───────────── AprilTag 1
      │                       │
      │                       │
      │        CENTER         │
      │     Cylinder_01       │
      │                       │
      │                       │
AprilTag 2 ───────────── AprilTag 3
```

즉:

```text
Tag constellation center == Cylinder_01 center
```

가 되도록 한다.

---

## 5.2 Tag 위치

부착면의 local coordinate 기준으로 4개 Tag를 대칭 배치한다.

예:

```text
Tag 0 = (-d, +d, 0)
Tag 1 = (+d, +d, 0)
Tag 2 = (+d, -d, 0)
Tag 3 = (-d, -d, 0)
```

단, 실제 `d`는 MEP의 기존 `Cylinder_01` 위치 및 부착면 크기를 먼저 확인한 후 충돌이나 geometry overlap이 발생하지 않는 범위에서 결정한다.

Tag는 `Cylinder_01`을 가리지 않아야 한다.

---

## 5.3 Tag ID

Tag ID는 반드시 고정값으로 사용한다.

예:

```text
Tag 0 → ID 0
Tag 1 → ID 1
Tag 2 → ID 2
Tag 3 → ID 3
```

단, 이미 프로젝트에서 사용 중인 AprilTag ID가 있다면 충돌하지 않도록 변경하고 configuration 파일에서 관리한다.

---

## 5.4 Tag Family

AprilTag family는 configuration으로 관리한다.

기본값은 다음과 같이 설정한다.

```yaml
apriltag:
  family: tag36h11
```

이미 환경에 설치되어 있거나 지원되는 다른 family가 있다면 해당 환경과 호환되는 family를 사용한다.

중요한 것은 family와 ID가 실행마다 변경되지 않는 것이다.

---

## 5.5 Tag 크기

Tag physical size 역시 configuration으로 분리한다.

예:

```yaml
apriltag:
  tag_size_m: 0.10
```

실제 MEP geometry와 카메라 거리에서 충분히 검출될 수 있도록 조정 가능해야 한다.

---

# 6. AprilTag는 물리 결합점이 아니다

매우 중요하다.

AprilTag는 단순한 optical marker이다.

AprilTag 자체를 로봇팔의 최종 물리 목표로 사용하지 않는다.

전체 시스템은 다음 관계를 사용한다.

```text
4 AprilTags
      ↓
Tag Constellation Pose
      ↓
Constellation Center
      ↓
Cylinder_01 Pose
      ↓
Predicted Cylinder_01 Pose
      ↓
Robot EE Target
      ↓
Magnetic Attachment
      ↓
FixedJoint
```

---

# 7. Canadarm3 손목 카메라 생성

현재 USD에 `cam_wrist`가 존재한다고 가정하지 않는다.

이번 구현에서 Canadarm3의 손목에 RGB Pinhole camera를 새로 생성한다.

카메라 위치는:

```text
Canadarm3 EE / Wrist
```

에 고정한다.

카메라는 로봇팔 EE와 함께 움직여야 한다.

---

## 7.1 카메라 요구사항

카메라:

```text
Type: RGB
Model: Pinhole
Resolution: configuration
FOV: configuration
```

기본 예:

```yaml
camera:
  name: cam_wrist
  width: 1280
  height: 720
  horizontal_fov_deg: 70
```

실제 Isaac Sim에서 지원되는 camera API를 사용한다.

---

## 7.2 Camera intrinsic

카메라의 intrinsic matrix:

```text
K =
[ fx  0  cx ]
[ 0  fy  cy ]
[ 0   0   1 ]
```

를 정확히 확보한다.

`fx`, `fy`, `cx`, `cy`는 실제 생성된 Isaac Sim 카메라의 설정값에서 가져온다.

임의의 값으로 하드코딩하지 않는다.

CSV 또는 별도의 configuration/log 파일에 기록한다.

---

# 8. AprilTag Detection

카메라 이미지에서 4개의 AprilTag를 검출한다.

각 Tag에 대해 최소한 다음 정보를 확보한다.

```text
tag_id
detected
corner_0
corner_1
corner_2
corner_3
center_2d
```

---

# 9. 4개 Tag를 이용한 Pose 추정

단순히 4개 Tag의 중심 좌표를 평균내는 방식으로 Pose를 계산하지 않는다.

각 AprilTag의 4개 corner를 사용한다.

총:

```text
4 tags × 4 corners
= 16개의 2D image points
```

를 사용한다.

각각에 대응되는 3D object points를 미리 정의한다.

구조:

```text
3D Object Points
        +
2D Image Points
        +
Camera Intrinsic
        ↓
solvePnP / solvePnPRansac
        ↓
Rotation + Translation
        ↓
Tag Constellation Pose
```

가능하면 `solvePnPRansac`를 사용할 수 있도록 구성한다.

단, 모든 Tag가 안정적으로 검출되는 정상적인 Phase 1 상황에서는 일반 `solvePnP`도 사용할 수 있다.

---

# 10. 좌표계 관리

좌표계 혼동을 절대 허용하지 않는다.

최소한 다음 frame을 명확히 구분한다.

```text
World
MRV / Robot Base
Canadarm3 EE
cam_wrist
MEP
Tag Constellation
Cylinder_01
```

Pose는 각 frame 사이의 변환을 명확히 기록한다.

기본 흐름:

```text
AprilTag Detection
        ↓
Camera Frame
        ↓
Robot / MRV Frame
        ↓
World Frame
```

최종 로봇 제어 목표는 반드시 Robot/MRV 또는 World 기준으로 변환하여 사용한다.

---

# 11. Cylinder_01 Pose 계산

Tag constellation의 중심과 `Cylinder_01`의 상대 위치는 초기 USD 구성 단계에서 정확히 정의한다.

이 관계를 이용하여:

```text
Tag Constellation Pose
+
Known Transform(TagCenter → Cylinder_01)
=
Estimated Cylinder_01 Pose
```

를 계산한다.

즉 AprilTag detection 결과로 직접 `Cylinder_01`을 검출하는 것이 아니다.

AprilTag constellation을 기준으로 `Cylinder_01`의 위치와 자세를 역산한다.

---

# 12. Ground-Truth와 Vision Estimate 비교

시뮬레이션에서는 실제 `Cylinder_01`의 World Pose를 직접 읽을 수 있다.

이를:

```text
GT Pose
```

로 기록한다.

AprilTag + PnP로 계산한 결과는:

```text
Estimated Pose
```

로 기록한다.

두 값을 매 simulation step 또는 지정된 logging frequency로 비교한다.

---

# 13. Pose Error 계산

## 위치 오차

```text
position_error =
||GT_position - Estimated_position||
```

단위:

```text
meter
```

CSV에는:

```text
position_error_mm
```

로 저장한다.

---

## 각도 오차

회전 오차는 단순 Euler angle 차이를 기본 방식으로 사용하지 않는다.

가능하면 rotation matrix 또는 quaternion을 이용해 두 orientation 사이의 최소 회전각을 계산한다.

개념:

```text
R_error = R_gt^-1 * R_est
```

그리고:

```text
angle_error_deg
```

를 계산한다.

Euler angle을 별도로 기록해야 한다면 wrap-around를 고려한다.

예를 들어:

```text
179°
-179°
```

를 358° 오차로 계산하지 않는다.

---

# 14. MEP 움직임 추정

AprilTag 기반으로 추정된 `Cylinder_01` pose를 시간에 따라 기록한다.

연속된 두 시점 `t0`, `t1`에서 위치를 비교하여 선속도를 추정한다.

```text
linear_velocity =
(position_t - position_t-dt) / dt
```

Phase 1에서는 MEP에 roll / pitch / yaw 회전을 주지 않으므로 미래 운동 예측에 각속도를 사용하지 않는다. Orientation은 초기 정렬 상태가 유지되는지 모니터링하고, 비의도적 회전이 발생하는지 검증하는 용도로 기록할 수 있다. 각속도 추정 및 회전 예측은 후속 확장 단계로 둔다.

---

# 15. 미래 위치 예측

이번 프로젝트에서 가장 중요한 제어 원칙이다.

현재 위치를 그대로 따라가는 것이 아니라:

```text
현재 MEP 위치
+
현재 MEP 운동 상태
+
prediction horizon
```

을 이용해 미래의 `Cylinder_01` 위치를 예측한다.

기본 prediction horizon:

```text
0.3 sec
```

단, 반드시 configuration parameter로 만든다.

예:

```yaml
prediction:
  horizon_sec: 0.3
```

---

## 15.1 기본 선형 예측

초기 구현은 다음 모델을 사용한다.

```text
p_future =
p_current + v_current * Δt
```

---

## 15.2 Phase 1 자세 처리

Phase 1에서는 MEP에 의도적인 roll / pitch / yaw 회전을 주지 않는다. 따라서 미래 orientation을 각속도로 외삽하는 회전 예측은 구현하지 않는다.

Orientation은 AprilTag/PnP로 계속 추정하여 초기 자세가 유지되는지 확인하고, 비의도적인 회전이나 물리 불안정이 발생하는지 검증하는 용도로 사용한다.

목표는:

```text
현재 Tag Pose
→ 현재 Cylinder_01 Pose
→ 선속도 기반 미래 Cylinder_01 위치
→ EE 접근
→ Magnetic Capture / FixedJoint
→ 도킹 구현
```

을 안정적으로 완성하는 것이다. 회전 유영 및 미래 orientation 예측은 이 단계가 성공한 이후 확장한다.

---

# 16. Robot EE 접근 전략

이번 프로젝트에서는 Image-Based Visual Servoing을 구현하지 않는다.

즉 다음과 같은 방식은 사용하지 않는다.

```text
image center error
→ pixel error
→ directly control joint velocity
```

대신:

```text
AprilTag
   ↓
PnP
   ↓
Cylinder_01 6-DoF Pose
   ↓
Future Cylinder_01 Pose
   ↓
Robot/MRV Frame
   ↓
EE Target Pose
   ↓
Canadarm3 IK / Cartesian Motion
```

구조를 사용한다.

이것을 Phase 1의 marker-based pose tracking / predictive Cartesian approach로 정의한다.

---

# 17. EE Target Pose

EE가 단순히 `Cylinder_01` 위치에 도달하는 것이 아니라 적절한 접근 방향을 유지하도록 한다.

기본 목표:

```text
EE position
→ predicted Cylinder_01 position
```

EE orientation:

```text
→ MEP attachment surface normal
```

을 기준으로 정렬한다.

단, 최종 Magnetic Attachment가 기존 방식과 호환되어야 한다.

따라서 기존에 사용하던 EE Cylinder의 orientation과 attachment 방향을 먼저 확인하고 그 방향을 우선한다.

---

# 18. 접근 단계

로봇팔이 한 번에 목표점으로 이동하여 충돌하지 않도록 접근 단계를 나눈다.

권장 구조:

```text
Stage A
Far approach
↓
Stage B
Prediction target tracking
↓
Stage C
Slow approach
↓
Stage D
Capture range entry
↓
Stage E
Magnetic Attachment
↓
Stage F
FixedJoint
```

---

# 19. 상대 속도 제어

3톤 MEP를 다루기 때문에 최종 접촉 시 상대 속도가 중요하다.

따라서 capture 직전에는 EE와 `Cylinder_01` 사이의 상대 속도를 계산한다.

목표:

```text
relative_velocity < 0.05 m/s
```

가 되도록 한다.

만약 상대 속도가 너무 높으면 즉시 capture하지 않고 접근 속도를 낮춘다.

---

# 20. 기존 Magnetic Attachment 유지

현재 프로젝트에서 이미 정상적으로 동작하고 있는 Magnetic Attachment 방식을 삭제하거나 새 방식으로 교체하지 않는다.

기존 로직을 최대한 재사용한다.

다만 capture가 너무 어려운 경우 configuration으로 tolerance/range를 조정할 수 있도록 한다.

예:

```yaml
capture:
  max_distance_m: 0.30
  max_angle_deg: 5.0
  max_relative_velocity_mps: 0.05
```

단순히 capture range만 지나치게 확대하여 물리적으로 말이 안 되는 상황에서 자동으로 붙도록 만들지 않는다.

---

# 21. Capture 조건

최종 capture는 다음 조건을 모두 만족할 때만 허용한다.

```text
1. EE와 Cylinder_01의 거리 <= capture_distance
2. EE와 docking surface의 orientation error <= capture_angle
3. 상대 접근 속도 <= capture_velocity
4. 접근 방향이 유효한 방향
5. AprilTag 기반 target tracking이 정상 상태
```

조건 만족 시:

```text
[CAPTURE] ATTEMPT
```

로그를 출력한다.

기존 Magnetic Attachment를 실행한다.

FixedJoint 생성 성공 시:

```text
[CAPTURE] CAPTURED
```

를 출력한다.

---

# 22. FixedJoint

capture 성공 이후 기존 방식과 동일하게 `UsdPhysics.FixedJoint`를 생성한다.

FixedJoint가:

```text
Robot EE / magnetic fixture
        ↕
MEP / gripper_fixture
```

를 연결하도록 한다.

FixedJoint 생성 이후 Physics 안정성을 확인한다.

---

# 23. Debug Visualization

## 23.1 3D Debug

가능하면 Isaac Sim의 debug draw 기능을 우선 사용한다.

표시 대상:

```text
AprilTag 4개 위치
AprilTag 연결선
Tag constellation center
Cylinder_01 estimated position
Cylinder_01 ground-truth position
Predicted future position
EE current position
EE target position
```

가능하면 색상은 다음 의미를 구분할 수 있도록 구성한다.

```text
Tag constellation
Ground Truth
Estimated
Prediction
EE
```

---

## 23.2 2D Camera Overlay

3D debug draw가 불안정하거나 확인하기 어려우면 `cam_wrist` 영상에 OpenCV overlay를 추가한다.

표시:

```text
Tag 0
Tag 1
Tag 2
Tag 3

4개 Tag를 연결하는 사각형

중앙 Crosshair

Estimated Cylinder_01

Prediction Point
```

예:

```text
┌──────────────────────────┐
│  □────────────────□      │
│  │                │      │
│  │       +        │      │
│  │   Cylinder     │      │
│  │                │      │
│  □────────────────□      │
└──────────────────────────┘
```

---

# 24. Logging

다음 파일을 생성한다.

```text
project/logs/vision_capture_metrics.csv
```

필요하면:

```text
project/logs/vision_capture_metrics.json
```

도 함께 생성할 수 있다.

---

# 25. CSV 필수 항목

다음 항목을 반드시 기록한다.

```text
timestamp

gt_x
gt_y
gt_z

gt_roll
gt_pitch
gt_yaw

est_x
est_y
est_z

est_roll
est_pitch
est_yaw

position_error_mm
angle_error_deg

pred_x
pred_y
pred_z

ee_x
ee_y
ee_z

ee_target_x
ee_target_y
ee_target_z

ee_target_distance_m

relative_velocity_mps

mep_linear_velocity_x
mep_linear_velocity_y
mep_linear_velocity_z

mep_angular_velocity_x
mep_angular_velocity_y
mep_angular_velocity_z

num_detected_tags

capture_state
capture_success

total_elapsed_time
```

---

# 26. Capture State

다음 상태 머신을 사용한다.

```text
INIT
  ↓
SEARCH
  ↓
TAG_DETECTED
  ↓
POSE_ESTIMATED
  ↓
PREDICTING
  ↓
APPROACHING
  ↓
SLOW_APPROACH
  ↓
CAPTURE_ATTEMPT
  ↓
CAPTURED
  ↓
HOLDING
  ↓
SUCCESS
```

실패 상황:

```text
TAG_LOST
POSE_INVALID
PREDICTION_INVALID
APPROACH_TIMEOUT
CAPTURE_FAILED
PHYSICS_ERROR
```

등을 별도로 기록한다.

---

# 27. Tag Loss 처리

Phase 1에서는 MEP의 4개 Tag가 계속 보이는 상황을 기본 시나리오로 한다.

단, 일시적으로 Tag detection이 실패할 수 있으므로 즉시 로봇팔을 무작정 이동시키지 않는다.

예:

```text
Tag detection lost
        ↓
last valid pose 유지
        ↓
짧은 timeout
        ↓
재검출 시 tracking 복귀
```

timeout을 초과하면 안전 정지한다.

Phase 1에서는 MEP repositioning을 수행하지 않는다.

---

# 28. Test 1 — Static Accuracy Test

MEP를 정지시킨다.

조건:

```text
linear velocity = 0
angular velocity = 0
```

4개 AprilTag를 카메라에서 충분히 볼 수 있도록 한다.

AprilTag → PnP → Cylinder_01 pose를 계산한다.

Ground Truth와 비교한다.

PASS 조건:

```text
position_error < 5 mm
angle_error < 0.5 deg
```

그리고 다음을 확인한다.

```text
[PASS] AprilTag detection
[PASS] PnP pose estimation
[PASS] Cylinder_01 center estimation
[PASS] Debug visualization
[PASS] CSV generated
```

---

# 29. Test 2 — Linear Drift Intercept Test

MEP에는 tumbling을 적용하지 않고 회전 없는 단순 선형 유영만 적용한다.

테스트 범위:

```text
roll / pitch / yaw angular velocity = 0 deg/s
```

선속도:

```text
0.01 ~ 0.02 m/s
```

AprilTag를 실시간 검출한다.

다음 순서로 동작해야 한다.

```text
AprilTag detection
      ↓
Pose estimation
      ↓
Cylinder_01 tracking
      ↓
Velocity estimation
      ↓
Future position prediction
      ↓
EE target generation
      ↓
Predictive approach
      ↓
Slow approach
      ↓
Capture
```

PASS 조건:

```text
relative_velocity_at_capture < 0.05 m/s
```

그리고:

```text
60 sec 이내 FixedJoint 생성
```

성공해야 한다.

로그:

```text
[CAPTURE] CAPTURED
```

가 출력되어야 한다.

---

# 30. Test 3 — Holding & Physics Stability Test

capture 성공 후 최소:

```text
10 sec
```

동안 FixedJoint를 유지한다.

확인 항목:

```text
MEP position
MEP orientation
FixedJoint validity
Physics stability
Robot EE stability
```

PhysX가 발산하거나 물체가 폭발하지 않아야 한다.

3톤 MEP가 FixedJoint를 통해 정상적으로 유지되어야 한다.

가능하다면 holding 이후 로봇팔을 아주 천천히 retreat하여 FixedJoint가 실제로 MEP를 함께 유지하는지도 확인한다.

---

# 31. 자동화 테스트 실행

가능하면 다음과 같은 실행 구조를 만든다.

```text
tests/
├── test_static_accuracy.py
├── test_dynamic_intercept.py
└── test_holding_stability.py
```

또는 프로젝트 구조에 적합한 방식으로 구성한다.

각 테스트는 결과를 명확하게 반환한다.

예:

```text
TEST 1: PASS
TEST 2: PASS
TEST 3: PASS
```

실패하면:

```text
TEST 2: FAIL
Reason:
- tag detection lost
- pose error exceeded threshold
- relative velocity too high
- capture timeout
```

처럼 원인을 출력한다.

---

# 32. Headless / GUI

개발 및 디버깅 단계에서는 GUI 실행을 우선한다.

AprilTag overlay와 3D debug visualization을 직접 확인할 수 있어야 한다.

안정화 이후 자동 테스트에서는 Headless 실행이 가능하도록 구조를 만든다.

가능하면 동일한 controller를 GUI/Headless에서 공통으로 사용한다.

---

# 33. Configuration 파일

하드코딩을 최소화하고 다음 값을 configuration 파일로 분리한다.

예:

```text
config/vision_capture.yaml
```

포함 항목:

```yaml
mep:
  mass_kg: 3000
  linear_velocity_mps: 0.01
  angular_velocity_deg_s: 0.0

camera:
  name: cam_wrist
  width: 1280
  height: 720
  horizontal_fov_deg: 70

apriltag:
  family: tag36h11
  ids: [0, 1, 2, 3]
  tag_size_m: 0.10
  constellation_half_width_m: ...

prediction:
  horizon_sec: 0.3

capture:
  max_distance_m: 0.30
  max_angle_deg: 5.0
  max_relative_velocity_mps: 0.05

test:
  static_position_threshold_mm: 5.0
  static_angle_threshold_deg: 0.5
  dynamic_timeout_sec: 60.0
  holding_duration_sec: 10.0
```

실제 값은 USD geometry와 기존 환경을 확인하여 합리적으로 결정한다.

---

# 34. 기존 프로젝트 코드 보호

이번 작업을 시작하기 전에 현재 프로젝트의 기존 코드를 먼저 확인한다.

특히 다음 기능을 검색한다.

```text
MEP
debris
gripper_fixture
Cylinder_01
magnet
magnetic
FixedJoint
capture
Canadarm3
robot
```

이미 구현되어 정상 동작하는 Magnetic Attachment 코드는 최대한 재사용한다.

중복 구현하지 않는다.

기존 기능을 삭제하거나 깨뜨리지 않는다.

---

# 35. 구현 전 환경 조사

코드를 수정하기 전에 반드시 다음을 확인한다.

1. 현재 프로젝트 디렉터리 구조
2. 현재 USD 파일 위치
3. MEP Prim 경로
4. `debris/gripper_fixture/Cylinder_01` 존재 여부
5. Canadarm3 Prim 경로
6. Canadarm3의 EE / wrist 위치
7. 현재 robot control 방식
8. 현재 Magnetic Attachment 구현
9. 현재 FixedJoint 생성 방식
10. 현재 무중력 설정 방식
11. 현재 MEP physics 설정
12. 현재 실행 command
13. Isaac Sim 버전
14. Space Robotics Bench 버전/API
15. 사용 가능한 AprilTag/OpenCV 관련 Python package

환경을 확인한 후 현재 프로젝트 구조와 맞지 않는 부분은 임의로 덮어쓰지 말고 기존 구조에 맞춰 통합한다.

---

# 36. Isaac Sim API 사용 원칙

현재 설치된 Isaac Sim 버전에 실제로 존재하는 API를 사용한다.

예전 버전 API를 추측해서 작성하지 않는다.

특히:

```text
Camera API
Physics API
UsdPhysics
Debug Draw
Robot articulation API
IK / controller API
Simulation callbacks
```

는 현재 설치 환경에서 확인한 후 사용한다.

존재하지 않는 API를 가정해서 구현하지 않는다.

---

# 37. Physics 안정성

3톤 MEP이므로 physics timestep 및 solver stability를 고려한다.

FixedJoint 생성 순간 큰 impulse가 발생하지 않도록 한다.

필요하다면 capture 직전에 상대 속도를 충분히 낮춘다.

다음 현상이 발생하면 실패로 기록한다.

```text
MEP 폭발
robot joint explosion
NaN
Inf
PhysX instability
FixedJoint invalid
```

---

# 38. 안전한 접근

로봇팔이 MEP를 직접 빠른 속도로 관통하지 않도록 한다.

목표점 접근 속도는 단계적으로 감소시킨다.

개념:

```text
far:
normal approach

near:
reduced velocity

capture range:
very slow approach

capture:
relative velocity threshold 확인
```

---

# 39. 성공 판정

최종적인 Phase 1 성공 조건은 다음과 같다.

### Vision

```text
4개 AprilTag 정상 검출
```

### Pose

```text
Cylinder_01 위치 오차 < 5 mm
각도 오차 < 0.5 deg
```

### Prediction

```text
MEP의 움직임을 기반으로 미래 Cylinder_01 위치 계산 가능
```

### Motion

```text
Canadarm3가 예측 위치를 추종
```

### Capture

```text
relative velocity < 0.05 m/s
distance <= capture threshold
angle <= capture threshold
```

### Physics

```text
FixedJoint 정상 생성
3톤 MEP 안정적으로 유지
10초 holding 성공
```

### Logging

```text
vision_capture_metrics.csv 정상 생성
```

### Report

```text
Phase 1 테스트 결과 Markdown 파일 생성
```

---

# 40. 최종 결과 파일

최종적으로 최소한 다음 구조를 만든다.

```text
project/
├── config/
│   └── vision_capture.yaml
│
├── logs/
│   ├── vision_capture_metrics.csv
│   └── phase1_result.md
│
├── scripts/
│   └── ...
│
├── tests/
│   ├── test_static_accuracy.py
│   ├── test_dynamic_intercept.py
│   └── test_holding_stability.py
│
└── ...
```

실제 프로젝트 구조가 다르다면 기존 구조에 맞춰 통합한다.

---

# 41. 작업 진행상황 Markdown 기록

구현을 진행하면서 반드시 작업 내용을 Markdown으로 기록한다.

예:

```text
project/logs/phase1_progress.md
```

파일에는 다음을 기록한다.

```markdown
# Phase 1 Progress Log

## Step 0 — Environment Inspection
- [ ] Project structure checked
- [ ] USD checked
- [ ] Canadarm3 checked
- [ ] MEP checked
- [ ] Existing magnetic capture checked

## Step 1 — MEP Physics
- [ ] 3000 kg mass
- [ ] Zero gravity
- [ ] Linear drift motion (roll / pitch / yaw = 0)

## Step 2 — AprilTag
- [ ] Tag 0
- [ ] Tag 1
- [ ] Tag 2
- [ ] Tag 3
- [ ] Constellation center aligned with Cylinder_01

## Step 3 — cam_wrist
- [ ] Camera created
- [ ] Camera attached to wrist
- [ ] Intrinsics verified

## Step 4 — Pose Estimation
- [ ] AprilTag detection
- [ ] PnP
- [ ] Cylinder_01 pose estimation

## Step 5 — Prediction
- [ ] Velocity estimation
- [ ] Future pose prediction

## Step 6 — Robot Control
- [ ] EE target generation
- [ ] Cartesian approach
- [ ] Slow approach

## Step 7 — Capture
- [ ] Magnetic attachment
- [ ] FixedJoint
- [ ] Holding

## Step 8 — Verification
- [ ] Test 1
- [ ] Test 2
- [ ] Test 3

## Final
- [ ] CSV
- [ ] Result report
```

각 단계가 완료될 때마다 실제 수행한 작업과 결과를 기록한다.

실패한 경우에도 삭제하지 말고:

```text
FAIL
Cause
Attempted fix
Result
```

형태로 기록한다.

---

# 42. 최종 결과 Markdown Report

모든 테스트 종료 후:

```text
project/logs/phase1_result.md
```

를 생성한다.

다음 내용을 포함한다.

```markdown
# Phase 1 Verification Report

## Environment

## MEP Configuration

## Camera Configuration

## AprilTag Configuration

## Pose Estimation Accuracy

| Metric | Result | Threshold | PASS/FAIL |
|---|---:|---:|---|
| Position Error | | < 5 mm | |
| Angle Error | | < 0.5 deg | |

## Dynamic Intercept

| Metric | Result | Threshold | PASS/FAIL |
|---|---:|---:|---|
| Capture Time | | < 60 sec | |
| Relative Velocity | | < 0.05 m/s | |

## Holding Stability

| Metric | Result | Threshold | PASS/FAIL |
|---|---:|---:|---|
| Holding Duration | | >= 10 sec | |
| Physics Stability | | Stable | |

## Final Result

TEST 1:
TEST 2:
TEST 3:

Overall:
```

실제 측정값을 기반으로 작성한다.

값을 임의로 PASS 처리하지 않는다.

---

# 43. 구현 중 문제가 발생할 경우

문제가 발생하면 무조건 코드를 계속 수정하지 말고 다음 순서로 원인을 분리한다.

```text
1. USD 문제인가?
2. Physics 문제인가?
3. Camera 문제인가?
4. AprilTag Detection 문제인가?
5. PnP 문제인가?
6. Coordinate Transform 문제인가?
7. Prediction 문제인가?
8. Robot IK / Motion 문제인가?
9. Magnetic Attachment 문제인가?
10. FixedJoint / PhysX 문제인가?
```

각 단계별로 독립 테스트가 가능하도록 구현한다.

---

# 44. 절대 금지사항

다음 방식으로 결과를 만들어서는 안 된다.

1. Ground Truth `Cylinder_01` 위치를 직접 robot controller에 전달하여 비전 시스템을 우회하는 것
2. AprilTag detection 결과가 없어도 Ground Truth를 사용해 자동으로 capture하는 것
3. 카메라 화면의 중앙 좌표만 보고 MEP 위치를 추정하는 것
4. AprilTag 하나만 보고 4-point constellation을 무시하는 것
5. `Cylinder_01`을 AprilTag로 대체하는 것
6. 기존 Magnetic Attachment를 삭제하는 것
7. capture range를 지나치게 크게 만들어 실제 접근 없이 자동 capture되는 것
8. 3톤 질량을 임의로 1톤 이하로 변경하는 것
9. Test 결과를 실제 측정 없이 PASS 처리하는 것
10. 존재하지 않는 Isaac Sim API를 추측해서 사용하는 것

---

# 45. 최종 구현 순서

반드시 다음 순서로 진행한다.

```text
STEP 0
현재 프로젝트 / USD / 코드 구조 조사
        ↓
STEP 1
3톤 MEP + 무중력 + 회전 없는 단순 선형 유영 설정 확인
        ↓
STEP 2
Cylinder_01 위치 확인
        ↓
STEP 3
Cylinder_01 중심 기준으로 4개 AprilTag 배치
        ↓
STEP 4
Canadarm3 wrist에 cam_wrist 생성
        ↓
STEP 5
Camera intrinsic 확인
        ↓
STEP 6
AprilTag detection 구현
        ↓
STEP 7
16-point PnP 구현
        ↓
STEP 8
Tag constellation → Cylinder_01 Pose 변환
        ↓
STEP 9
Ground Truth와 Estimated Pose 비교
        ↓
STEP 10
Static Accuracy Test
        ↓
STEP 11
MEP linear velocity 추정 (angular velocity prediction 제외)
        ↓
STEP 12
0.3 sec 미래 Cylinder_01 Pose 예측
        ↓
STEP 13
Robot/MRV Frame으로 target 변환
        ↓
STEP 14
Canadarm3 EE Cartesian / IK 접근
        ↓
STEP 15
상대 속도 감소
        ↓
STEP 16
기존 Magnetic Attachment 실행
        ↓
STEP 17
FixedJoint 생성
        ↓
STEP 18
10 sec Holding
        ↓
STEP 19
Dynamic Intercept Test
        ↓
STEP 20
CSV / Debug Visualization 확인
        ↓
STEP 21
최종 Phase 1 Report 작성
```

---

# 46. 최종 목표

최종적으로 다음 상황이 자동으로 재현되어야 한다.

```text
무중력
  ↓
3톤 MEP 단순 선형 유영 (roll / pitch / yaw 회전 없음)
  ↓
MEP 부착면에 있는 4개 AprilTag를 cam_wrist가 인식
  ↓
4개 Tag constellation의 6-DoF pose 계산
  ↓
Cylinder_01의 실제 물리 결합점 위치 계산
  ↓
현재 위치 + 운동 상태 계산
  ↓
0.3초 후 Cylinder_01 미래 위치 예측
  ↓
Canadarm3 EE가 예측 위치를 추종
  ↓
접근 속도 감소
  ↓
capture distance / angle / velocity 조건 만족
  ↓
기존 Magnetic Attachment
  ↓
FixedJoint 생성
  ↓
3톤 MEP 포획
  ↓
10초 holding
  ↓
CSV telemetry 저장
  ↓
Test 1 / Test 2 / Test 3 결과 생성
  ↓
Phase 1 완료
```

이번 Phase 1의 핵심은 **"카메라 영상 자체를 보고 로봇팔을 조종하는 시스템"이 아니라, "AprilTag constellation을 통해 실제 물리 결합점 `Cylinder_01`의 6-DoF 위치와 움직임을 추정하고 그 미래 위치를 예측하여 로봇팔이 접근하는 시스템"**을 만드는 것이다.

구현 과정에서 기존 프로젝트의 정상 동작하는 기능은 최대한 보존하고, 변경이 필요한 경우 기존 코드와 신규 코드를 명확하게 분리하여 통합한다.
