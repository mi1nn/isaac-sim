## 8. `feature/apriltag`

### 이슈와 수정

- 3톤 MEP가 무중력에서 이동할 때 단순 현재 pose만 사용하면 접근 중 목표가 달라진다.
- 4개 AprilTag PnP, `Cylinder_01` pose, 미래 위치 예측, IK 접근, `FixedJoint` 캡처를 추가했다.
- 단계별 Test 1/2/3과 정적/동적 시나리오를 분리했다.

### 실행 명령

```bash
git switch feature/apriltag
~/isaac-sim/python.sh project/scripts/run_phase1_tests.py
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario dynamic
~/isaac-sim/python.sh project/scripts/vision_capture.py --scenario static --headless

cd project
~/isaac-sim/python.sh -m pytest \
  tests/test_vision_math.py tests/test_static_accuracy.py \
  tests/test_dynamic_intercept.py tests/test_holding_stability.py -s
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 기록: 3톤 MEP, Python 도킹, AprilTag 부착 성공 커밋이 있다.
- 제한: 위 명령의 전체 테스트 결과는 현재 세션에서 재측정하지 않았다.
