# MEP Mission Extension Dashboard

Isaac Sim 기반 Automated Satellite Mission Extension System의 **UI 전용 mock prototype**.
`/home/rokey/Downloads/UI메인화면.png`, `/home/rokey/Downloads/UI 기술검증.png`를 확인하여 레이아웃과 색상을 반영했습니다.
HTML / CSS / Vanilla JavaScript / Chart.js / Python FastAPI로 작성했습니다.

## 실행 (Ubuntu 24.04)

```bash
cd /home/rokey/mep_dashboard
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m uvicorn backend.app:app --host 127.0.0.1 --port 8000
```

브라우저에서 **http://127.0.0.1:8000** 접속. 종료는 서버 터미널에서 `Ctrl+C`.
이미 `.venv`에 패키지를 설치해 두었으므로 재실행 시 마지막 명령만 실행하면 됩니다.
8000번 포트를 사용하는 다른 서버가 있으면 `--port 8001`로 변경하고 해당 주소로 접속하세요.
`venv` 생성 시 ensurepip 오류가 있으면 Ubuntu의 `python3-venv` 패키지를 설치하세요.

필수 Python 패키지: FastAPI 0.141.1, Uvicorn 0.53.0.
Chart.js 4.4.8은 `frontend/js/vendor/chart.umd.js`에 포함되어 실행 시 CDN이나 외부 인터넷이 필요하지 않습니다.
Playwright 1.63.0은 브라우저 검증용이며 앱 실행에는 필요하지 않습니다.

## 화면과 동작

- 상단 LIVE MISSION / TECHNOLOGY VALIDATION 탭: 같은 페이지에서 전환. 방향키/Home/End 지원.
- LIVE: 큰 Isaac Sim viewport, Mission State, 3/7 progress, Current State, 속도/각속도/거리,
  로그 스케일 Position Error 차트, 하단 두 카메라 패널.
- 모든 영상 패널은 STREAM OFFLINE placeholder입니다.
- 최초 로드부터 1초 간격으로 mock Position Error가 갱신됩니다. 초기값은 0.018 m입니다.
- PLAY: mock 시간 및 그래프 재개. STOP: 시간 및 그래프 정지.
- RESET: 정지 상태로 시간 0, Position Error 0.018 m, 초기 그래프 복원.
- 제어 버튼은 브라우저 내부 상태만 변경합니다. 버튼 클릭에 따른 서버 요청은 없습니다.
- Mission Progress 3/7, SLOW_APPROACH, MEP CAPTURE PHASE, 속도 0.021 m/s,
  각속도 0.35 deg/s, 남은 거리 0.118 m는 이번 단계에서 고정입니다.
- 차트의 초기 -60~0초는 가상의 사전 이력이며 현재 이후 최대 121개 표본을 유지합니다.
- VALIDATION: 상단 KPI 3개, XY 산점도 2개, 원점과 반경 0.05 m tolerance,
  오차 추이 차트 2개, 성공률 KPI 2개, RUN HISTORY 및 SELECTED RUN DETAILS.
- RUN HISTORY의 행 또는 Run ID 버튼을 선택하면 상태/오차/시간/상세가 갱신됩니다.
- 48개 실행 기록은 고정된 함수로 생성합니다. 새로고침 시 같은 값이 표시됩니다.
- 모든 KPI/산점도/추이/상세는 같은 mock 기록에서 계산합니다.
  Overall Success는 capture와 docking 모두 성공한 비율(41/48, 85.4%)입니다.
  Capture는 45/48 (93.8%), Docking은 43/48 (89.6%)입니다.
- Mean Error는 XY 거리의 평균, Std Dev는 거리의 모집단 표준편차,
  Within Tolerance는 XY 거리가 0.05 m 이하인 비율입니다.
  단계 성공 여부는 별도의 mock 시나리오 결과이므로 tolerance 비율과 다릅니다.
- 평균 임무 시간 개선율은 가상 baseline 770초와 비교합니다.
- 데스크톱 1920×1080, 브라우저 확대율 100%에서 주요 요소가 한 화면에 들어옵니다.
  실행 이력 48개는 표 안에서 스크롤합니다. 작은 화면은 문서 세로 스크롤을 허용합니다.

## 파일 구조

```text
mep_dashboard/
├── backend/
│   └── app.py
├── frontend/
│   ├── index.html
│   ├── css/
│   │   └── style.css
│   └── js/
│       ├── app.js
│       ├── live.js
│       ├── validation.js
│       └── vendor/
│           └── chart.umd.js
├── checks/
│   ├── browser_check.py
│   ├── report.json
│   ├── live-1920.png
│   └── validation-1920.png
├── requirements.txt
├── .gitignore
└── README.md
```

`.venv/`, `.browsers/`는 이 폴더 내부에 설치된 실행/검증 도구입니다.
`app.py`는 `/` 화면, `/static` 정적 파일, `/health` mock 상태만 제공합니다.
`app.js`는 공통 차트 설정과 탭, `live.js`는 로컬 telemetry,
`validation.js`는 mock 실행 기록과 통계를 담당합니다.

## 검증

Chromium headless에서 실제 HTTP 서버에 접속하여 검증했습니다.

- HTTP 화면/정적 파일 로딩 성공
- JavaScript 문법 검사 통과, 브라우저 JavaScript/console 오류 0건
- 실패한 리소스 요청 0건, 제어 명령 요청 0건
- mock 자동 갱신, STOP 정지, RESET 초기화, PLAY 재개
- 탭 반복 전환, 방향키 전환, Chart.js 5개 정상 렌더링
- 실행 기록 48개 및 실패 기록 상세 선택
- 양쪽 화면 1920×1080에서 문서 overflow 없음
- 390px 모바일 너비에서 가로 overflow 없음

결과: `checks/report.json`. 화면 캡처: `checks/live-1920.png`, `checks/validation-1920.png`.

검증 재실행 (서버가 8000번 포트에서 실행 중이어야 함):

```bash
.venv/bin/python -m pip install playwright==1.63.0
PLAYWRIGHT_BROWSERS_PATH="$PWD/.browsers" .venv/bin/python -m playwright install chromium
PLAYWRIGHT_BROWSERS_PATH="$PWD/.browsers" .venv/bin/python checks/browser_check.py
```

## 이번 단계의 범위

PostgreSQL, ROS2 Subscriber/Publisher, Isaac Sim 연결, WebRTC, 카메라 스트림,
실제 DB 데이터는 구현하지 않았습니다. 기존 프로젝트 파일은 수정하지 않았습니다.

향후 예정 구조 (현재 미구현):
`Web UI → FastAPI → ROS2 Command → GPU PC → Isaac Sim`.

참고한 공식 문서:
[FastAPI Static Files](https://fastapi.tiangolo.com/tutorial/static-files/),
[Chart.js Scatter Chart](https://www.chartjs.org/docs/latest/charts/scatter.html).
