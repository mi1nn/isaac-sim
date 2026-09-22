## 2. `feature/asset`

### 이슈와 수정

- MEP와 Satellite USD가 외부 또는 이동 전 경로를 참조해 로딩이 깨질 수 있었다.
- 자산을 저장소로 이동하고 USD 내부 참조 경로를 저장소 구조에 맞게 고쳤다.
- stage 설정을 추가해 이후 캡처 브랜치의 자산 기반을 만들었다.

### 실행 명령

```bash
git switch feature/asset
python3 -m compileall -q project/srb
PYTHONPATH="$PWD/project" ~/isaac-sim/python.sh -m srb ls env -a
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 48개 등록.
- 제한: MEP/Satellite USD를 실제 stage에 열어 prim 경로와 texture까지 확인하는 자산 smoke test는 미실행이다.
