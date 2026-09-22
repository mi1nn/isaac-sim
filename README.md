## 4. `feature/grip`

### 이슈와 수정

- `srb agent zero`는 모든 action을 0으로 보내 Kinova 그리퍼가 닫히는 동작을 만들었다.
- 조작 입력이 필요한 시험은 `srb agent manual`로 실행하도록 정리했다.
- Canadarm3+Kinova300 조작, debris capture 초기화, 물리/기하 검사 스크립트와 작업 로그가 추가됐다.

### 실행 명령

```bash
git switch feature/grip
cd project
srb agent manual --env debris_capture_visual \
  --kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" \
  env.robot=canadarm3+kinova300_large

~/isaac-sim/python.sh ../docs/debris_capture_tests/capture_test.py
~/isaac-sim/python.sh ../docs/debris_capture_tests/manual_test.py
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 48개 등록.
- 과거 기록: `capture_test.py` 19/19 통과, `manual_test.py` 5/5 통과.
- 제한: 수동 GUI 조작과 실제 grasp 성공은 현재 세션에서 재실행하지 않았다.
