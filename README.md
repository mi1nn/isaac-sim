## 11. `feature/docking_param`

### 이슈와 수정

- 초기 docking의 접근 속도와 재배치 위치가 느리거나 불안정했다.
- docking 속도, reposition 조건, probe 접근 파라미터를 조정했다.
- tip의 고유 변경은 `.gitignore`이며 핵심 기능은 중간 docking 이력에 있다.

### 실행 명령

```bash
git switch feature/docking_param
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag dock_param --headless --dock_only

cd project
~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py -q
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 제한: 속도 증가와 재배치 변경 후 전체 파이프라인 재검증 근거가 없다.
