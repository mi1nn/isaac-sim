## 12. `feature/redline`

### 이슈와 수정

- client 주변 목표 궤도를 사람이 GUI에서 확인할 기준선이 없었다.
- 붉은 원형 reference orbit 계산과 USD `BasisCurves` 시각화를 추가했다.
- `vision_capture.yaml`, `vision.py`, `vision_task.py`에는 아직 연결되지 않았다.

### 실행 명령

```bash
git switch feature/redline
python3 -m compileall -q project/srb project/scripts
PYTHONPATH="$PWD/project" ~/isaac-sim/python.sh -m srb ls env -a
cd project && ~/isaac-sim/python.sh -m pytest tests -q
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 문제: graph 분석 산출물 등 기능 외 파일이 브랜치에 포함돼 있어 선택 병합이 적합하다.
- 제한: redline이 실제 viewport에 나타나는 런타임 경로는 연결되지 않아 미검증이다.
