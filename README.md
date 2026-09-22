## 19. `feature/web`

### 이슈와 수정

- 웹 대시보드가 mock state만 표시하고 실제 simulator ground-truth와 제어 명령을 연결하지 못했다.
- GT capture metrics, pause/resume/reset ROS commands, probe camera를 추가했다.
- mock state label을 실제 state machine과 맞췄다. 기존 README의 `mock only` 설명은 최신 tip과 불일치한다.

### 실행 명령

대시보드:

```bash
git switch feature/web
cd mep_dashboard
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m uvicorn backend.app:app --host 127.0.0.1 --port 8000
```

브라우저: `http://127.0.0.1:8000`

브라우저 자동 검사:

```bash
cd /home/rokey/isaac_space/mep_dashboard
PLAYWRIGHT_BROWSERS_PATH="$PWD/.browsers" \
  .venv/bin/python checks/browser_check.py
```

Simulator/ROS 연결:

```bash
cd /home/rokey/isaac_space
source /opt/ros/jazzy/setup.bash
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --ros --dock --tag web_pipeline
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 과거 mock UI 검사: Chromium HTTP/static 로딩 성공, JavaScript console 오류 0건, resource 실패 0건.
- 제한: 실제 ROS control, GT metric, probe camera, WebRTC/영상 경로를 함께 연결한 end-to-end 브라우저 시험은 미검증이다.
