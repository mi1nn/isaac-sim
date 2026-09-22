# Isaac Sim Freeze 해결 인수인계 프롬프트

현재 상태에서 그대로 이어서 작업한다. 전체 프로젝트를 재구성하지 말고, 아래 원인과 다음 작업만 최소 범위로 진행한다.

## 프로젝트

- Isaac/SRB: `/home/rokey/space_robotics_bench`
- Web Dashboard: `/home/rokey/MRV_MEP_Project/mep_web_db_integration/mep_dashboard`
- ROS 2: `/opt/ros/jazzy`
- `ROS_DOMAIN_ID=144`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- FastDDS 후보: `/home/rokey/.ros/fastdds_whitelist.xml`
- Firebase: `/home/rokey/space_robotics_bench/serviceAccount.json`

## 지금까지 확인된 근본 원인

처음 Isaac이 멈춘 직접 원인은 ROS Jazzy `rclpy` ABI 불일치였다.

- Isaac Sim Python: 3.11.13
- `/opt/ros/jazzy` 기본 `rclpy`: Python 3.12용
- 오류: `No module named rclpy._rclpy_pybind11`

이 문제는 Isaac Python 3.11용 `rclpy` overlay를 빌드해 해결했다.

현재 overlay:

```text
/home/rokey/ros_jazzy_py311
```

빌드 결과:

```text
/home/rokey/ros_jazzy_py311/lib/python3.11/site-packages/rclpy/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so
```

`source /opt/ros/jazzy/setup.bash` 후 `source /home/rokey/ros_jazzy_py311/local_setup.bash`를 적용하면 Isaac Python에서 `rclpy` import가 PASS했다.

## 마지막 Isaac 실행 결과

`rclpy` overlay 적용 후 Isaac은 GUI/app startup, scene 생성, simulation setup, capture task 초기화까지 정상 진행했다.

그 후 다음 assertion으로 종료됐다.

```text
rcl_interfaces__msg__parameter_event__convert_from_py:
Assertion `strncmp("rcl_interfaces.msg._parameter_event.ParameterEvent", full_classname_dest, 50) == 0' failed
Aborted (core dumped)
```

남은 문제는 Jazzy `rclpy`와 Isaac Sim에 포함된 ROS 메시지 모듈이 섞이는 것이다. Isaac에는 Humble/Python 3.11 모듈이 있고, 외부 Jazzy Python 3.12 패키지도 있어 `rcl_interfaces` 같은 generated message가 혼용되었다.

## 다음 해결 방향

외부 `/opt/ros/jazzy` Python 3.12 패키지와 새로 빌드한 `rclpy`를 섞지 않는다.

Isaac Sim에 이미 Python 3.11용 Jazzy bridge가 포함되어 있다.

```text
/home/rokey/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy
```

이 디렉터리에는 Python 3.11용 Jazzy `rclpy`와 generated ROS message 패키지가 함께 있다.

```text
rclpy/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so
rclpy-7.1.4-py3.11.egg-info
```

다음 작업은 이 Isaac 내장 Jazzy Python 3.11 bridge 전체를 우선 사용하도록 실행 환경을 구성하는 것이다. `rclpy`만 외부 overlay에서 가져오고 `rcl_interfaces` 등 메시지는 Isaac 경로에서 가져오는 혼합을 만들지 않는다.

권장 순서:

1. Isaac 내장 Jazzy bridge 경로에서 `rclpy`, `rcl_interfaces`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `builtin_interfaces`, `rosidl_*`, `rmw_*` import 경로를 확인한다.
2. `/opt/ros/jazzy/lib/python3.12/site-packages`가 Isaac Python 3.11 ROS import보다 앞서지 않도록 한다.
3. 가능하면 Isaac 내장 Jazzy bridge의 전체 Python 3.11 패키지를 우선 사용한다.
4. 필요한 C/C++ ROS shared library 경로만 `/opt/ros/jazzy/lib`에서 제공한다.
5. 먼저 다음 최소 테스트를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=144
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/rokey/isaac-sim/python.sh - <<'PY'
import rclpy
from rcl_interfaces.msg import ParameterEvent
print(rclpy.__file__)
print(ParameterEvent.__module__)
PY
```

6. 위 테스트가 PASS한 뒤에만 Isaac 전체를 실행한다.

## 이미 반영된 Isaac 코드 수정

수정 파일:

```text
project/srb/tasks/manipulation/debris_capture/vision_capture_demo.py
project/srb/tasks/manipulation/debris_capture/astrobee.py
```

반영 내용:

- `ENABLE_EXTRA_CAMERA_WINDOWS = False`
- `cam_wrist`, `cam_probe`, Astrobee extra GUI window 호출은 flag 안에서만 실행
- Main Isaac viewport는 유지
- 최초 10 rendered frame warm-up
- warm-up 전 Camera 1/2/3 및 Viewport image read/publish 금지
- Camera 1/2 최대 5 FPS
- Astrobee Camera 3 최대 5 FPS
- Main Viewport 10 FPS
- Viewport capture는 `mission_started = self.ros is None or self.ros.start_received` 이후에만 실행
- ROS status/control polling과 `/mrv/status` 흐름은 유지
- physics timestep, camera pose/intrinsics, mission state machine은 변경하지 않음

두 파일 `python3 -m py_compile` 검증은 PASS했다. 이 두 파일을 되돌리거나 웹 파일을 수정하지 않는다.

## Web Dashboard 현재 상태

웹 UI 수정은 완료되어 있다. 다음 파일은 이번 해결에서 수정하지 않는다.

```text
frontend/index.html
frontend/css/style.css
frontend/js/live.js
frontend/js/validation.js
backend/app.py
```

마지막 Web health 확인은 PASS였으나, 단순 `nohup ... &` 프로세스가 셸 종료 후 정리되는 환경 특성이 있었다. 다음 작업 시 Web은 PTY/지속 세션으로 실행하고 `/health`를 확인한다.

## 다음 실행 검증 순서

1. 내장 Jazzy bridge의 `rclpy + ParameterEvent` 최소 import 테스트
2. 두 Isaac 파일 `py_compile`
3. Web `/health`
4. Isaac을 다음 환경으로 실행

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=144
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ -f /home/rokey/.ros/fastdds_whitelist.xml ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
fi
```

5. Isaac app/scene startup가 완료되고 traceback/assertion이 없는지 확인
6. `/mrv/status` 실제 message 1개 확인
7. 정상일 때만 Web endpoint로 START를 정확히 한 번 전송
8. START 전송 전 실패하면 즉시 중단하고 로그 마지막 20줄만 보고

중단 조건:

- Isaac process 종료
- traceback/assertion/core dump
- `/mrv/status` topic은 있으나 실제 message가 없음
- GUI not responding

이 경우 START를 보내지 않는다. 전체 회귀 테스트나 mission 장시간 실행도 하지 않는다.

## 목표 결과

```text
rclpy import: PASS
ParameterEvent import: PASS
Isaac startup/scene/simulation loop: PASS
/mrv/status actual message: PASS
extra camera windows: 없음
Camera 1/2/3 ROS feed: 유지
Viewport ROS feed: START 이후 유지
```
