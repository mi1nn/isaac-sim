## 5. `feature/arm`

### 이슈와 수정

- 위성 임무를 위한 팔/MEP 환경과 로봇 조종 코드가 없었다.
- `satellite_mission` task, MEP USDA, contact sensor, smoke script를 추가했다.
- 커밋 기록에 로봇 조종 Python이 미완성이라고 명시돼 있다.

### 실행 명령

```bash
git switch feature/arm
python3 -m compileall -q srb
PYTHONPATH="$PWD" ~/isaac-sim/python.sh -m srb ls env -a
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 제한: 구형 루트 `srb/` 구조이며, arm 조종과 접촉 성공의 현재 재현 결과는 없다.
