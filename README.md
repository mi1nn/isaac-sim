## 13. `feature/mep-monitor`

### 이슈와 수정

- capture/docking 진행률과 오차를 ROS 2에서 관찰하고 CSV/PlotJuggler로 분석할 경로가 없었다.
- capture와 docking metric bridge, active monitor, CSV logger, mock telemetry, PlotJuggler 설정을 추가했다.
- 7단계 mission state와 실제 Isaac metric 연결을 보정했다. README의 `mock only` 설명은 최신 코드보다 오래됐다.

### 실행 명령

```bash
git switch feature/mep-monitor
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/mep_monitor_ws/src
ln -s "$PWD/mep_monitor" /tmp/mep_monitor_ws/src/mep_monitor
cd /tmp/mep_monitor_ws
colcon build --packages-select mep_monitor
source install/setup.bash
ros2 run mep_monitor mock_telemetry
```

테스트만 실행할 때:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/mep_monitor:$PYTHONPATH" python3 -m pytest mep_monitor/test -q
```

### 결과

- 현재: Python 컴파일 통과, 단위 테스트 14개 중 13개 통과.
- 실패 위치: `test_ros_transport_and_shutdown`.
- 원인: 임시 검사 환경에서 package를 `colcon build` 후 `install/setup.bash`로 source하지 않아 subprocess의 `ros2 run`이 `Package 'mep_monitor' not found`로 종료됐다.
- 수정/재검증: 위 workspace 명령으로 설치한 뒤 테스트를 다시 실행해야 한다. 세 차례 환경 조정 후에도 설치 가정이 충족되지 않아 추가 반복은 중단했다.
