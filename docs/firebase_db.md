# Firebase (Firestore) 저장 구조

시뮬레이터(`vision_capture.py --ros`)가 내보내는 ROS 2 토픽을 `project/scripts/firebase_bridge.py` 가 받아 Firestore 에 기록합니다.
웹 대시보드는 Firestore 를 `onSnapshot` 으로 구독합니다 (웹은 미구현). 브리지는 별도 프로세스라 시뮬레이터 루프를 막지 않습니다.

> 상태: 레코더 로직은 가짜 메시지로 검증. **실제 Firestore / ROS 연동은 아직 실행 검증 전.**

## 실행

```bash
source /opt/ros/jazzy/setup.bash
pip install firebase-admin
python3 project/scripts/firebase_bridge.py --credentials serviceAccount.json   # 실제 기록
python3 project/scripts/firebase_bridge.py --dry_run                           # 쓰기 내용만 출력
```

에뮬레이터: `FIRESTORE_EMULATOR_HOST=localhost:8080 python3 project/scripts/firebase_bridge.py --project_id demo-mrv`.
옵션: `--namespace`(기본 `mrv`), `--rate_hz`(텔레메트리 행/초, 기본 5), `--session_id`, `--idle_timeout`.

## 컬렉션

```
simulation_sessions/{session_id}                        Table 1
simulation_sessions/{session_id}/session_telemetry/{id} Table 2 (id = 0000000, 0000001, ...)
```

Table 2 를 서브컬렉션으로 둔 이유: `session_id` 인덱스 없이 세션별 조회·구독이 되고, 세션 삭제/보존 단위가 명확합니다 (`session_id` 필드도 함께 저장).
정렬은 `orderBy("sim_time")` 또는 `id`.

### simulation_sessions
`session_id, created_at(서버 시각), duration_sec, final_distance_m, final_angle_deg, contact_vel_mps`
+ `is_running`(진행 중 true), `final_state`, `is_docked`(= `docking_success`, 하위 호환용). 도킹 단계에서 끝난 세션은 `final_distance_m`/`final_angle_deg` 가 도킹 오차(남은 삽입 거리 / 축 각도). 시작 시 문서 생성(KPI 는 null) → 종료 시 KPI 1회 기록.

**성공 여부 (기존 `is_success` 는 제거됨 — Capture 성공만으로 Mission 성공이 되던 문제)**

| 필드 | 의미 |
|---|---|
| `capture_success` | 포획 성공 (마지막 `captured`) |
| `docking_success` | 도킹 조인트 생성 성공. 도킹을 하지 않은 실행(`--dock` 없음)은 `null` |
| `mission_success` | `capture_success AND docking_success`. 도킹을 하지 않은 실행은 `capture_success` 와 같음 |
| `failure_stage` | 실패 단계: `VISION`(TAG_LOST, POSE_INVALID, PREDICTION_INVALID) / `APPROACH`(APPROACH_TIMEOUT, MRV_APPROACH_FAILED) / `CAPTURE`(CAPTURE_FAILED) / `DOCKING`(DOCK_FAILED) / `PHYSICS` / `ABORTED`. 실패 없으면 null |
| `failure_reason` | `/status.failure` 원문 (없으면 null) |

**Capture KPI** (접촉 순간의 `est_*` 값, 포획 못 하면 null): `capture_position_error_m`(EE↔실린더 거리), `capture_lateral_error_m`, `capture_orientation_error_deg`, `capture_contact_velocity_mps`, `capture_time_s`(포획 시각, 시뮬레이션 시계).

**Docking KPI** (도킹 조인트 생성 시점의 값. 도킹 전에 실패하면 마지막 값. 도킹 안 했으면 null): `docking_position_error_m`(프로브 팁↔도킹점 상대 위치 벡터 크기), `docking_lateral_error_m`, `docking_orientation_error_deg`, `docking_insertion_depth_m`, `docking_relative_velocity_mps`, `docking_time_s`(도킹 단계 시작 → 도킹 성공/종료).

**성공률 계산**: Capture = `capture_success` 인 Run / 전체 Run, Docking = `docking_success` 인 Run / 전체 Run, Overall = `mission_success` 인 Run / 전체 Run. Overall 을 앞의 두 비율의 평균으로 구하지 않는다.

### session_telemetry
`id, session_id, sim_time, state, phase, ee_x/y/z, target_x/y/z, goal_x/y/z, est_x/y/z, distance_m, lateral_error_mm, angle_error_deg, rel_vel_mps, rel_ang_vel_rad_s, is_captured`
+ `phase`: `CAPTURE`(기본) / `DOCKING`(도킹 단계) / `MISSION`(MRV 랑데부·`ARM_DEPLOY`, 포획도 도킹도 아님).
+ 도킹용 추가 컬럼: `probe_x/y/z`(프로브 팁), `dock_x/y/z`(위성 도킹점), `dock_roll_deg`, `insertion_depth_m`, `wall_clearance_mm`, `is_docked` (포획 단계에서는 null / false).
+ 도킹 텔레메트리 (`phase = DOCKING` 일 때만 값, 아니면 null / false): `dock_relative_x_m/y_m/z_m`(도킹 프레임 상대 위치), `dock_lateral_error_m`, `dock_orientation_error_deg`(전체 자세 오차), `dock_geometry_distance_m`(남은 삽입 거리), `dock_insertion_depth_m`, `dock_relative_speed_mps`, `dock_ready`(`DOCK_READY` 상태), `dock_success`(도킹 조인트 생성).

### 단계별 의미 (같은 컬럼, 단계에 따라 기준이 바뀜)

| 컬럼 | 포획 단계 | 도킹 단계 (`DOCK_*` ~ `DOCK_HOLDING`) |
|---|---|---|
| `distance_m` | EE ↔ 실린더 거리 | 남은 삽입 거리 (도킹점까지) |
| `lateral_error_mm` | 접촉면 횡방향 이탈 | 도킹축 횡방향 이탈 |
| `angle_error_deg` | EE 목표 자세 오차 | 프로브 축 ↔ 도킹축 기울기 |
| `rel_vel_mps` | EE ↔ 타깃 상대 속도 | 프로브 ↔ 위성 상대 속도 |
| `est_*`, `goal_*` | 비전 추정 / 접근 목표 | null (비전이 꺼져 포획 때 값에서 멈춤) |
| `rel_ang_vel_rad_s` | `six_dof` 모드에서만 값 있음 | null |

## 동작 규칙

- `/mrv/status` (10 Hz) 1건당 텔레메트리 1행 (`--rate_hz` 로 솎음). 위치는 그 시점의 마지막 `ee/pose`, `gt/cylinder_pose`(target), `ee/target_pose`(goal), `estimate/cylinder_pose`(est).
- 세션 시작: 종료 상태가 아닌 첫 `/status`. 종료: `SUCCESS`, `DOCK_HOLDING`(도킹 완료 후 유지) 또는 실패 상태(`TAG_LOST`, `CAPTURE_FAILED`, `ABORTED`, `DOCK_FAILED` 등), Ctrl-C, 또는 `--idle_timeout` 동안 `/status` 없음.
- `capture_success` = 마지막 `captured` 값, `docking_success` = 한 번이라도 `dock_docked` 가 true 였는지, `failure_reason` = `/status.failure` (없으면 null), `contact_vel_mps` = `capture_contact_velocity_mps` = `captured` 가 false→true 로 바뀐 `/status` 의 `est_rel_vel`.
- 도킹 포함 여부는 `/status.dock_enabled` (시뮬레이터의 `docking.enabled`) 로 판단. Ctrl-C / idle timeout 처럼 실패 상태 없이 끝난 세션은 `failure_stage`/`failure_reason` 이 null 이고 `mission_success` 는 false 일 수 있다.
- 시뮬레이터 시계가 되돌아가면(재실행) 이전 세션을 닫고 새 세션 시작. `session_id` 기본값 `run_YYYYMMDD_HHMMSS`.
- NaN/inf 는 null 로 저장. 쓰기는 백그라운드 스레드에서 400건씩 배치, 실패 시 5회 재시도.
- 서비스 계정 키는 커밋하지 마세요 (`.gitignore` 에 추가됨).
