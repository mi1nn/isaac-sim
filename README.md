## 14. `feature/linear-movement`

### 이슈와 수정

- MEP 캡처 이후 MRV가 정지한 채 다음 단계로 넘어가 실제 평행 이동 임무가 반영되지 않았다.
- MRV 평행 이동을 파이프라인에 연결하고 mission dashboard UI를 추가했다.
- 커밋 기록상 MRV 평행 이동 후 파이프라인을 통과했다.

### 실행 명령

```bash
git switch feature/linear-movement
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --tag linear --dock

cd mep_dashboard
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m uvicorn backend.app:app --host 127.0.0.1 --port 8000
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 기록: MRV 평행 이동 후 파이프라인 통과.
- 제한: dashboard와 simulator를 동시에 연결하는 통합 실행은 현재 재검증하지 않았다.
