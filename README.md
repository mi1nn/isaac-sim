## 3. `feature/asset_v2`

### 이슈와 수정

- 개인 IDE 설정인 `.vscode`가 Git에 추적되고 있었다.
- `.vscode` 추적을 제거하고 `.gitignore`를 정리했다.
- 시뮬레이션 기능 변경은 없고 `feature/asset`의 자산 상태를 유지한다.

### 실행 명령

```bash
git switch feature/asset_v2
git check-ignore -v .vscode/settings.json
python3 -m compileall -q project/srb
PYTHONPATH="$PWD/project" ~/isaac-sim/python.sh -m srb ls env -a
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 48개 등록.
- 판단: 기능 브랜치보다 저장소 위생 정리 브랜치다.
