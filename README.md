## 16. `feature/db`

### 이슈와 수정

- 기존 `is_success` 하나로는 capture 성공, docking 성공, 전체 mission 성공을 구분할 수 없었다.
- `capture_success`, `docking_success`, `mission_success`, `failure_stage`, `failure_reason`으로 분리했다.
- Firebase Admin bridge, batch 400건, 최대 5회 retry, dry-run/emulator 경로를 추가했고 불필요한 `docking_ready` 속성을 제거했다.

### 실행 명령

터미널 1:

```bash
git switch feature/db
source /opt/ros/jazzy/setup.bash
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --ros --dock
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install firebase-admin
python3 project/scripts/firebase_bridge.py --dry_run
# 실제 DB 사용 시 서비스 계정 파일을 Git 밖에 둔다.
python3 project/scripts/firebase_bridge.py --credentials /path/to/serviceAccount.json
```

에뮬레이터 사용:

```bash
FIRESTORE_EMULATOR_HOST=localhost:8080 \
  python3 project/scripts/firebase_bridge.py --project_id demo-mrv
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 커밋 기록: MRV 접근→MEP 부착→Satellite 도킹→DB 저장 확인 완료.
- 문서 기록: 실제 Firestore/ROS 연결은 미실행으로 남아 있다.
- 판단: 커밋과 문서의 상태가 상충하므로 새 자격증명으로 end-to-end 재검증 전에는 실제 DB 성공을 확정하지 않는다.
