## 1. `feature/space`

### 이슈와 수정

- 초기 우주 로봇 자산과 임무 코드가 현재 `project/srb/`가 아닌 구형 `space_robotics_bench/` 구조에 있다.
- M0609, Kinova 300, cube grab, patrol robot 관련 USD와 설정을 추가했다.
- 현재 SRB 환경 등록 명령은 종료 코드 0이지만 환경 행이 0개라 등록 성공은 입증되지 않았다.

### 실행 명령

```bash
git switch feature/space
python3 -m compileall -q space_robotics_bench
PYTHONPATH="$PWD" ~/isaac-sim/python.sh -m srb ls env -a
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 초기화/종료 성공, 등록 환경 미표시.
- 판단: 자산 원형 보존용 브랜치로는 유효하지만 현재 구조에서 바로 임무를 실행할 수 있는지는 미검증이다.
