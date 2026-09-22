## 6. `feature/magnet`

### 이슈와 수정

- 캡처/도킹 계보에서 자기식 부착 실험을 분리하려는 브랜치다.
- 브랜치 tip의 고유 변경은 ignore 설정 수준이고, 주요 캡처 코드는 선행 브랜치에서 상속됐다.
- 따라서 magnet 고유 구현과 성공 조건을 커밋만으로 확정하기 어렵다.

### 실행 명령

```bash
git switch feature/magnet
python3 -m compileall -q project/srb project/scripts
PYTHONPATH="$PWD/project" ~/isaac-sim/python.sh -m srb ls env -a
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 49개 등록.
- 제한: magnet 전용 실행 진입점과 독립 성공 기록이 없어 기능 결과는 미검증이다.
