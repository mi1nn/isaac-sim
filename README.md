## 10. `feature/docking`

### 이슈와 수정

- MEP 캡처 뒤 Ares1 probe를 Client Satellite에 삽입하는 연속 임무가 필요했다.
- probe/thruster 도킹, 접근 상태기계, 재배치와 속도 조정을 추가했다.
- 정지·비회전 client 조건에서 capture부터 docking까지 전체 파이프라인이 성공한 기록이 있다.

### 실행 명령

```bash
git switch feature/docking
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag dock --headless --dock_only
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag full --dock

cd project
~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py tests/test_vision_math.py -q
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 기록: 정지 client에서 전체 capture→docking 성공.
- 제한: 이동/회전 client는 이 브랜치의 성공 범위에 포함되지 않는다.
