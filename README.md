## 9. `feature/rotation`

### 이슈와 수정

- 선형 속도만 예측하면 회전하는 tag/MEP의 도킹 방향을 따라갈 수 없다.
- `six_dof` motion mode, XYZ와 roll/pitch/yaw, constant-twist 예측, 6-DoF 캡처 테스트를 추가했다.
- 오프라인 수학 테스트와 실제 Isaac 물리 검증이 분리돼 있으며 일부 항목은 `NEEDS_ISAAC_VALIDATION` 상태다.

### 실행 명령

```bash
git switch feature/rotation
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag six_dof --set mep.motion_mode=six_dof

cd project
~/isaac-sim/python.sh -m pytest tests/test_six_dof_capture.py -s
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 기록: 부유 상태 MEP 부착 테스트 커밋이 있다.
- 제한: 회전 표적의 전체 capture 성공과 안정성은 현재 미검증이다.
