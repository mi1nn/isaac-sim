# 우주 비협조 물체 자율 포획 및 정밀 삽입 시스템

## 한 줄 결론

이 프로젝트의 강점은 Isaac Sim 환경 자체를 만든 것이 아니라, 오픈소스 Space Robotics Bench 위에서 **우주 물체 포획과 정밀 삽입에 필요한 perception–control–physics 폐루프를 설계하고 통합한 것**이다.

독자적인 원천 알고리즘으로 표현하기보다는 다음과 같은 기반·코어 기술로 설명하는 것이 정확하다.

> RGB-D 인식, 좌표계 변환, differential IK 제어, 물리적 포획 및 peg-in-hole 충돌 모델링을 연결한 우주로봇 조작 시스템

## 포트폴리오 제목

**Isaac Sim·ROS 2 기반 우주 비협조 물체 자율 포획 및 정밀 삽입 시스템**

## 핵심 기술

### 1. RGB-D 기반 폐루프 비전 서보

미리 입력한 목표 좌표로 로봇을 단순 이동시키는 방식이 아니라, 카메라 관측으로 목표를 계속 갱신하며 접근하도록 구성했다.

- RGB-D 영상의 마스크 픽셀을 카메라 내부 파라미터로 3차원 점군에 역투영한다.
- RANSAC 평면 추정으로 MEP 포획 마커의 중심과 표면 법선을 구한다.
- 3차원 원 피팅으로 thruster nozzle의 중심, 축 및 반지름을 추정한다.
- 부분 원호만 보이는 상황과 outlier가 포함된 점군을 처리한다.
- 추정 결과를 ROS TF 좌표계로 변환하여 differential IK 속도 명령을 생성한다.
- `SEARCH → APPROACH → DOCK → SETTLE → VERIFY → HOLD → DONE` 상태기계로 탐색, 접근, 포획 검증 및 실패 복구를 수행한다.

구현 근거:

- `project/scripts/ros2/vision_servo.py`
- `project/scripts/ros2/mep_suction_capture.py`
- `project/scripts/ros2/grasp_geometry.py`
- `project/scripts/ros2/mep_rgbd_pose.py`
- `project/scripts/ros2/mep_pose_controller.py`

### 2. 물리적으로 일관된 우주 물체 포획 모델

물체를 로봇에 순간이동시키거나 단순 parenting하는 대신, 실시간 물리 상태를 이용해 포획을 모델링했다.

- 로봇 말단의 capture point와 MEP marker의 world pose를 매 physics step마다 계산한다.
- 거리 기반 `IDLE → APPROACH → CAPTURED` 상태기계를 사용한다.
- 포획 순간의 상대 pose를 유지하도록 `UsdPhysics.FixedJoint`를 생성한다.
- USD scale과 physics body frame 차이를 보정하여 joint local frame을 계산한다.
- release 직후 즉시 재포획되지 않도록 rearm 조건을 둔다.
- 후퇴 동작에서 MEP marker가 로봇을 따라오는 비율을 측정하여 실제 포획 여부를 확인한다.
- 포획면 사이의 기울기를 각도로 계산하여 삽입 가능성을 정량화한다.

구현 근거:

- `project/srb/tasks/manipulation/debris_capture/capture.py`
- `project/srb/tasks/manipulation/debris_capture/task.py`

### 3. Peg-in-hole 시뮬레이션의 물리 정확도 개선

초기 satellite collision은 mesh별 convex hull을 사용했다. 이 방식은 오목한 thruster nozzle 내부를 채워진 물체로 근사하므로, 시각적으로 구멍이 있어도 peg가 내부로 진입할 수 없다.

이를 다음과 같이 수정했다.

- thruster와 rim처럼 실제 구멍을 구성하는 mesh에만 SDF collision을 적용한다.
- 나머지 satellite mesh는 계산 비용이 낮은 convex hull을 유지한다.
- SDF collision이 GPU PhysX에서만 정상 동작한다는 조건을 확인하고 CUDA device 설정을 보존한다.
- asset frame에서 peg tip과 nozzle mouth 위치 및 축을 계산하여 두 물체의 초기 pose를 정렬한다.
- MEP camera를 peg 축 근처로 이동하고 hole이 시야에 들어오도록 camera extrinsic을 계산한다.

구현 근거:

- `project/srb/tasks/manipulation/debris_capture/task.py`
- `project/srb/core/env/common/base/env_cfg.py`

### 4. Isaac Sim–ROS 2 통합 기반

Isaac Sim 내부의 ROS 2 환경 차이를 흡수하고 외부 인식·제어 노드가 simulator와 통신할 수 있도록 구성했다.

- Isaac Sim에 포함된 ROS 2 distro를 자동으로 선택한다.
- Python path와 shared library path를 설정하고 bridge 초기화 실패를 명확히 보고한다.
- Isaac Sim 배포 환경에 `tf2_ros`가 없을 때 사용할 `/tf` 및 `/tf_static` broadcaster를 구현한다.
- static transform을 parent–child 쌍별로 유지하고 transient-local QoS로 전달한다.
- 오래된 pose, 미래 timestamp, 잘못된 frame 및 비정상 quaternion을 제어 입력에서 거부한다.

구현 근거:

- `project/srb/utils/ros.py`
- `project/srb/interfaces/interface/ros.py`
- `project/scripts/ros2/test_mep_pipeline.py`

## 포트폴리오 설명 예시

> 오픈소스 Space Robotics Bench를 기반으로 RGB-D 인식부터 좌표계 변환, differential IK 제어, 물리적 포획, peg-in-hole 충돌 모델링까지 이어지는 폐루프 우주로봇 조작 파이프라인을 개발했습니다. Convex-hull 충돌이 오목한 노즐을 막아 삽입이 불가능한 문제를 분석하고, 필요한 mesh에만 SDF collision을 적용했습니다. 또한 실시간 물리 pose 기반 fixed-joint 포획 상태기계와 포획 추종 검증 로직을 구현했습니다.

## 면접 답변 예시

### 30초 답변

> Isaac Sim 환경 자체는 오픈소스를 활용했습니다. 제가 집중한 부분은 그 위에서 우주 비협조 물체를 실제로 포획하고 삽입할 수 있도록 perception, control, physics를 연결하는 일이었습니다. RGB-D 점군에서 포획면과 구멍의 pose를 추정하고 differential IK로 접근하도록 했으며, fixed-joint 기반 포획과 SDF collision 기반 peg-in-hole 환경을 구현했습니다.

### 오픈소스 수정에 불과하지 않느냐는 질문

> 프레임워크와 기존 asset은 오픈소스를 사용했지만, 목표 작업에 그대로 적용할 수는 없었습니다. 예를 들어 기존 convex hull은 노즐 구멍을 막아 삽입 자체가 불가능했고, asset의 camera와 좌표계도 제어 목적에 맞지 않았습니다. 이 원인을 물리·좌표계 수준에서 분석하고 collision, camera extrinsic, 포획 상태기계 및 ROS 2 제어 파이프라인을 직접 설계했습니다.

### 가장 어려웠던 문제

> 시각적으로는 구멍이 보이는데 peg가 입구에서 멈추는 문제가 있었습니다. 위치 제어나 로봇 reach 문제가 아니라 convex hull이 오목한 nozzle을 solid로 근사한 것이 원인이었습니다. 구멍을 구성하는 mesh에만 SDF collision을 적용하고 GPU PhysX 설정을 보장하여, 계산 비용을 제한하면서 실제 cavity collision을 표현하도록 수정했습니다.

## 과장하지 않아야 할 표현

다음 내용은 프로젝트 기여로 주장하지 않는다.

- Isaac Sim 또는 Space Robotics Bench 프레임워크 자체 개발
- RANSAC, Kasa circle fitting 또는 differential IK 알고리즘 발명
- 정량 실험 없이 강건한 자율 삽입이나 sim-to-real 완성 주장
- 외부 오픈소스 asset을 직접 모델링했다는 주장

대신 다음과 같이 표현한다.

> 검증된 알고리즘과 오픈소스 프레임워크를 특정 우주 포획 문제에 맞게 통합하고, 물리 모델과 좌표계의 실패 원인을 분석하여 실제 실행 가능한 조작 파이프라인으로 구현했다.

## 현재 검증 상태

비전 서보의 simulator 비의존 기하 로직은 다음 명령으로 검증했다.

```bash
cd project
uv run pytest tests/vision_servo_test.py -q
```

확인 결과:

```text
12 passed in 0.20s
```

테스트 범위:

- 기울어진 평면의 중심과 법선 복원
- outlier가 포함된 평면 추정
- annulus 및 부분 원호의 3차원 원 피팅
- 카메라 방향을 이용한 법선 부호 결정
- 속도 제한 및 벡터 각도 계산

아직 별도로 확보해야 하는 증거는 Isaac Sim 전체 파이프라인의 반복 성공률이다.

## 다음 정량화 목표

포트폴리오의 설득력을 높이려면 동일 조건에서 반복 실험하여 다음 지표를 기록한다.

1. 초기 위치·자세 오차별 포획 성공률
2. 포획 시 capture tilt의 평균과 최대값
3. peg tip과 hole center의 최종 위치·각도 오차
4. 포획 및 삽입 완료 시간
5. detection loss 또는 depth noise 조건에서의 복구 성공률

