## 15. `feature/debris_satellite`

### 이슈와 수정

- docking과 docking parameter 계보를 하나의 debris/satellite 임무 브랜치로 합칠 필요가 있었다.
- 두 docking 계보를 병합하고 정지 client의 capture→docking 성공 상태를 가져왔다.
- tip 커밋에 `궤도 수정 필요`가 명시돼 있다.

### 실행 명령

```bash
git switch feature/debris_satellite
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag debris_satellite --dock

cd project
~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py tests/test_vision_math.py -q
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 기록: 정지 client 전체 도킹 성공 이력을 포함한다.
- 남은 이슈: 궤도 수정 후 전체 임무 재검증이 필요하다.
