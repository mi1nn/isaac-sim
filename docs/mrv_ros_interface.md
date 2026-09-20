# MRV — ROS 2 인터페이스 (Jazzy, rclpy)

`vision_capture.py --ros` 로 켭니다 (기본은 꺼짐). 시뮬레이터의 Isaac Sim ROS 2 브리지 `rclpy` 를 직접 사용하고,
제어 로직(비전 → 예측 → IK → capture)은 그대로입니다. 코드: `srb/tasks/manipulation/debris_capture/ros_interface.py`.

> 상태: **실행 검증 완료** (headless, six_dof, 별도 프로세스의 ROS 2 Jazzy 노드에서 수신/명령 확인).
> 검증하지 못한 것: GUI 실행, 다른 머신 간 DDS 통신, `mep_com` 기준 프레임과의 조합.

## 실행

```bash
cd ~/space_robotics_bench
# 텔레메트리만 (그대로 자동 진행)
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --ros --set mep.motion_mode=six_dof
# 예측이 준비되면 cmd/start 를 받을 때까지 팔이 관측 자세에서 대기
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic --ros_wait_start --set mep.motion_mode=six_dof
```

`--ros` 는 시작 시 `LD_LIBRARY_PATH` 에 브리지 라이브러리(`isaacsim.ros2.bridge/<ROS_DISTRO>/lib`)를 넣고 한 번 재실행합니다
(프로세스 시작 후에는 추가해도 로더가 못 봅니다). `ROS_DISTRO` (기본 `jazzy`), `RMW_IMPLEMENTATION` (기본 `rmw_fastrtps_cpp`) 를 씁니다.
수신 측 터미널:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp        # 시뮬레이터와 같은 RMW
ros2 topic echo /mrv/state
ros2 topic pub --once /mrv/cmd/start std_msgs/msg/Empty "{}"
```

## 토픽 (`/<namespace>/`, 기본 `mrv`)

| 방향 | 토픽 | 타입 | 내용 |
|---|---|---|---|
| out | `cam_wrist/image_raw` | sensor_msgs/Image | rgb8 1280×720, 비전이 쓴 영상 (best effort, 10 Hz) |
| out | `cam_wrist/camera_info` | sensor_msgs/CameraInfo | 그 영상의 K (왜곡 0) |
| out | `estimate/cylinder_pose` | PoseStamped | 필터링된 Cylinder_01 (비전만) |
| out | `predicted/cylinder_pose` | PoseStamped | t + `prediction.horizon_sec` 예측 |
| out | `estimate/mep_twist` | TwistStamped | 추정 v, w (world) |
| out | `ee/pose`, `ee/target_pose` | PoseStamped | EE 접촉 프레임 / 목표 |
| out | `gt/cylinder_pose`, `gt/mep_twist` | PoseStamped / TwistStamped | **Ground Truth, 평가 전용** (`ros.publish_ground_truth`) |
| out | `state` | std_msgs/String | 상태머신 상태 (latched, 변경 시) |
| out | `captured` | std_msgs/Bool | FixedJoint 부착 여부 (latched) |
| out | `status` | std_msgs/String | JSON: 상태, 태그 수, standoff, 추정 기준 capture 량(`est_distance/gap/lateral/angle/rel_vel/rel_ang_vel`), 실패 사유 |
| out | `/tf` | tf2_msgs/TFMessage | `world → mrv/{ee, cylinder_est, cylinder_pred, cam_wrist, cylinder_gt}` |
| in | `cmd/start` | std_msgs/Empty | `--ros_wait_start` 일 때 접근 시작 허가 |
| in | `cmd/abort` | std_msgs/Empty | 중단: 관절 정지, 최종 상태 `ABORTED` (결과 JSON 에 실패로 기록) |
| in | `cmd/capture_enable` | std_msgs/Bool | `false`: 추종은 하되 부착하지 않음 (기본 true) |

시각은 시뮬레이션 시간, 쿼터니언은 ROS 순서 (x, y, z, w). 제어는 `gt/` 토픽을 읽지 않습니다.
`cam_wrist` 프레임은 OpenCV 광학 좌표(Z 전방)입니다.

설정은 `project/config/vision_capture.yaml` 의 `ros:` 섹션 (`--set ros.namespace=robot1` 등).

## 주의

- MEP 는 명령을 기다리는 동안에도 흘러갑니다 (`test.dynamic_timeout_sec` 60 s 도 계속 셈). `cmd/start` / `capture_enable` 은 이 안에 보내야 합니다.
- 영상은 best effort + 2.7 MB/장이라 DDS 에서 일부가 유실될 수 있습니다 (테스트에서 상태·포즈는 전부, 영상은 일부 수신).
- `cmd/abort` 는 포획 후에도 동작하며 관절을 그 자리에 고정합니다 (FixedJoint 는 유지).

## 실행으로 확인한 것 (2026-09-20)

| 시험 | 결과 |
|---|---|
| `--ros_wait_start`, 별도 rclpy 노드가 8 s 뒤 `cmd/start` 전송 | PREDICTING 에서 대기 → 수신 즉시 APPROACHING → 캡처 성공 (21/22 checks, 실패 1건은 아래 각속도 항목) |
| 수신 토픽 | state, captured, status, estimate/predicted/ee/gt pose, twist, camera_info, tf, image (1280×720 rgb8) 확인, est ↔ gt 위치 일치 (수 mm) |
| `cmd/abort` (t = 2 s) | `APPROACHING -> ABORTED`, 관절 정지, `state` 토픽에 ABORTED |
