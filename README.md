## 7. `feature/suction`

### 이슈와 수정

- 그리퍼 대신 비접촉 접근 후 흡착으로 MEP를 고정해야 했다.
- ROS 2 흡착 제어, RGB-D pose 계산, pose control, vision servo, smoke/geometry 도구와 테스트를 추가했다.
- AprilTag 계보와는 별도의 접근 방식이라 동일한 실행 옵션으로 보장되지 않는다.

### 실행 명령

```bash
git switch feature/suction
python3 -m compileall -q project/srb project/scripts
PYTHONPATH="$PWD/project" ~/isaac-sim/python.sh -m srb ls env -a

source /opt/ros/jazzy/setup.bash
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario static --ros
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 48개 등록.
- 과거 기록: `파이썬 코드로 흡착 성공`, `위치 정렬` 커밋이 존재한다.
- 제한: ROS graph, sensor topic, 실제 흡착 성공을 현재 다시 실행하지 않았다.
