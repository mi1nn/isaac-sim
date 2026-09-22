## 18. `feature/integration`

### 이슈와 수정

- 캡처, MRV 이동, Satellite 도킹, gripper release/retreat, DB 저장, MEP 이동·회전이 분산돼 있었다.
- 위 단계를 하나의 파이프라인으로 통합하고 docking hold를 제거했다.
- Astrobee 자산/observer와 probe camera 각도 조정을 추가했다.

### 실행 명령

전체 임무:

```bash
git switch feature/integration
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag full_6dof --dock \
  --set mep.motion_mode=six_dof
```

Astrobee smoke:

```bash
~/isaac-sim/python.sh project/scripts/build_astrobee_usd.py
cd project
~/isaac-sim/python.sh -m pytest tests/test_astrobee_observer.py -q

ROS_DOMAIN_ID=77 ~/isaac-sim/python.sh scripts/vision_capture.py \
  --headless --dock_only --tag astrobee_smoke \
  --set "astrobee.camera={save_every_s: 5.0}"
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 기록: 파이프라인 테스트 완료, MRV 접근→MEP 부착→도킹→release/retreat→DB 단계 통합.
- 제외 범위: 현재 작업 디렉터리의 커밋되지 않은 Astrobee/docking 수정은 이 브랜치 설명에 포함하지 않았다.
- 제한: 현재 tip의 전체 6-DoF 물리 실행은 이번 조사에서 재실행하지 않았다.
