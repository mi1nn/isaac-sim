# Satellite Docking Demo

## 목적

「위성 임무 연장 자동화 시스템」 1차 시연입니다. Python 프로그램 하나를 실행하면 Canadarm3가
다음을 **물리 시뮬레이션으로** 자동 수행합니다.

```
Canadarm3 초기화 → MEP 접근 (PRE-GRASP) → EE 접촉면과 MEP 부착면 6-DoF 정렬
→ 법선 방향 직선 접근 → 평면 대 평면 접촉 → MEP 부착 (자석형 FixedJoint)
→ MEP 들어올리기 (LIFT) → Satellite 방향 운반 → Thruster PRE-DOCK
→ Probe(Ares1) ↔ Thruster 중심축 정렬 → 축을 따라 Probe 삽입 → 자동 도킹 → 도킹 유지
```

- MEP와 Satellite는 순간이동시키지 않습니다. 팔은 관절 위치 목표로만 움직이고, MEP는 팔에
  부착된 상태로, Satellite는 도킹 joint로 연결됩니다.
- 부착과 도킹은 **위치 + 자세 + 접근 방향 조건을 모두 실측으로 만족할 때만** 생성됩니다.

## 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 |
| Isaac Sim | 5.0.0 (`~/isaac-sim`) |
| Isaac Lab | v2.2.1 (`~/isaaclab`) |
| Space Robotics Bench | `~/space_robotics_bench/project` (`srb` 패키지) |
| Python | Isaac Sim 번들 Python (`~/isaac-sim/python.sh`) — **system `python3`로는 실행 불가** |
| 태스크 | `srb/debris_capture_docking` (`env.robot=canadarm3`, 그리퍼 없음) |

## 실행 전 준비

- `srb` 명령이 동작하는 환경이면 추가 설치는 없습니다. (`srb` 자체가 `~/isaac-sim/python.sh`로 실행됩니다.)
- 에셋: `assets/space_asset/debris_v3.usd`(MEP), `assets/space_asset/satellite_v3.usd`(Satellite),
  `assets/srb_assets/robot/manipulator/canadarm3_large.usdz`.
- 같은 PC에서 다른 Isaac Sim이 떠 있어도 실행은 되지만 GPU 메모리를 나눠 씁니다. 실행 시작 시
  멈춰 있거나(Ctrl+Z) 고아가 된 `srb` 프로세스가 있으면 경고가 출력됩니다.

## 실행 명령어

GUI (시연용):

```bash
cd ~/space_robotics_bench
~/isaac-sim/python.sh project/scripts/satellite_docking_demo.py
```

Play를 누른 뒤 시작하고 싶으면:

```bash
cd ~/space_robotics_bench
~/isaac-sim/python.sh project/scripts/satellite_docking_demo.py --start_paused
```

창 없이 자동 검증 (도킹 5초 뒤 R 키 동작을 자동으로 시험하고 종료, 모든 검사 PASS면 exit code 0):

```bash
cd ~/space_robotics_bench
~/isaac-sim/python.sh project/scripts/satellite_docking_demo.py --headless --test_undock 5 --exit_after 1500
```

| 옵션 | 설명 |
|---|---|
| `--headless` | Isaac Sim 창 없이 실행 |
| `--start_paused` | PAUSED로 시작, 툴바 Play를 누르면 시퀀스 시작 |
| `--test_undock SEC` | 도킹 SEC초 뒤 R 키 동작을 자동 실행 (자동 검증용) |
| `--exit_after SEC` | 시뮬레이션 SEC초 뒤 종료 (자동 검증에서는 R 검증 후 바로 종료) |
| `--kit_args "..."` | Kit 인자. GUI 기본값은 `srb agent manual`과 같은 `--ext-folder ~/isaac-sim/apps --enable isaacsim.exp.base` |
| 위치 인자 | Hydra override. 예: `env.docking.dock_depth=0.7` |

로그: 터미널 출력 + `project/logs/satellite_docking_demo/<시각>/`(Hydra 설정).

## 자동 시연 순서

```
INIT → PLAN → MOVE_TO_MEP_PREGRASP → ALIGN_MEP → MOVE_TO_MEP_GRASP → ATTACH_MEP
→ LIFT_MEP → MOVE_TO_SAT_PREDOCK → ALIGN_SAT → INSERT_PROBE → DOCK → DOCKED
```

| 상태 | 하는 일 | 다음 단계로 가는 조건 (실측) |
|---|---|---|
| INIT | USD에서 frame 측정, 배치 일관성 검사 | 1초 후 |
| PLAN | 5개 핵심 pose + 직선 경로 waypoint를 kinematic IK로 검증 (팔은 아직 안 움직임) | 모든 pose 도달 가능, 관절 점프 < 20°, 추정 clearance > 5 cm |
| MOVE_TO_MEP_PREGRASP | MEP 부착면 법선 방향 1.0 m 앞(PRE-GRASP)으로 이동 | 정착 (3 mm / 0.2°) |
| ALIGN_MEP | PRE-GRASP에서 6-DoF 자세 재확인 | 위치·회전·법선·roll·횡오차 모두 허용치 이내 |
| MOVE_TO_MEP_GRASP | 법선 방향 직선 접근 (최대 0.04 m/s, 가속 제한 + 남은 거리 비례 감속, 추종 오차가 1 cm에 가까워지면 연속적으로 감속) | 정착 |
| ATTACH_MEP | 부착 조건 검사 → 기존 자석형 FixedJoint 생성 | 조건 만족 시에만 부착 |
| LIFT_MEP | 이후 IK 제어점을 **probe 선단**으로 전환, 뒤로 1.8 m + 위로 0.5 m | MEP–EE 상대 pose 유지 |
| MOVE_TO_SAT_PREDOCK | 선단을 thruster 축 위, 노즐 출구 1.0 m 앞으로 운반 | 정착 + 선단 정지 |
| ALIGN_SAT | Probe 축과 Thruster 축 정렬 확인 | 반경 1 cm, 축 0.5°, roll 1° 이내 |
| INSERT_PROBE | Thruster 축을 따라 직선 삽입 (MEP 최종 접근과 같은 속도 프로파일) | 정착 + 선단 정지 |
| DOCK | 도킹 조건 검사 → MEP↔Satellite FixedJoint 생성 | 조건 만족 시에만 도킹 |
| DOCKED | 도킹 유지 | — |
| FAILED | 실패 원인 출력 후 현재 자세 유지 | — |

### 기준 frame (모두 실행 시 USD mesh에서 측정)

| 이름 | 정의 |
|---|---|
| `EE_ATTACH_POINT` | Canadarm3 `canadarm3_large_7` 링크의 capture cylinder **바깥 끝면 중심**, +Z = 바깥쪽 법선 |
| `MEP_GRASP_POINT` | MEP(Fermi 망원경 본체) 중 probe 반대편을 향하는 **가장 바깥 평면**(상단면)의 중심, +Z = 바깥쪽 법선 |
| `PROBE_DOCK_POINT` | Ares1 막대(probe)의 **선단**, +Z = 삽입 방향(선단 밖으로) |
| `SAT_DOCK_POINT` | GOES_R `Thruster` 노즐 축 위, 출구에서 `dock_depth`(0.8 m) 안쪽, +Z = 노즐 안쪽 |

부착 시 두 면은 마주봅니다: `EE +Z = −MEP +Z`, `EE +X = MEP +X` (X축 기준 반바퀴 관계).

### 부착 조건 (모두 만족해야 초록/부착)

| 항목 | 허용치 |
|---|---|
| 면 간격 (MEP 법선 방향) | −4 ~ +4 mm (목표 +1 mm) |
| 면 중심 횡오차 | ≤ 5 mm |
| 법선 반평행 오차 | ≤ 0.5° |
| roll (면 내 X축) 오차 | ≤ 0.5° |
| 접근 방향 (마지막 직선 이동 vs −법선) | ≤ 3° |

### 도킹 조건 (모두 만족해야 초록/도킹)

| 항목 | 허용치 |
|---|---|
| 선단 ↔ `SAT_DOCK_POINT` 축방향 | ≤ 10 mm |
| 선단의 thruster 축 이탈 (반경) | ≤ 10 mm |
| probe 축 ↔ thruster 축 각도 | ≤ 0.5° |
| roll | ≤ 1° |
| 선단이 노즐 안쪽 | depth > 0 |
| 삽입 방향 (삽입 이동 vs thruster 축) | ≤ 3° |

## 색상 의미

| 색 | capture cylinder (EE) | 노즐 입구 원판 (Satellite) |
|---|---|---|
| 파랑 | EE ↔ MEP 부착점 2 m 초과 | probe 선단 ↔ 도킹점 2 m 초과 |
| 노랑 | 2 m 이내, 접근/정렬 중 | 2 m 이내, 접근/삽입 중 |
| 초록 | **부착 조건 전부 만족** 또는 부착됨 | **도킹 조건 전부 만족** 또는 도킹됨 |

초록은 거리만으로 켜지지 않습니다.

## 키 조작

| 키 | 동작 |
|---|---|
| `R` | **가장 최근 연결 하나만** 해제. 도킹 상태면 도킹(MEP↔Satellite)만 해제하고 MEP는 팔에 붙어 있습니다. 한 번 더 누르면 EE↔MEP 부착을 해제합니다. |

## 성공 확인

터미널에서 다음 순서로 나오면 정상입니다.

```
[CHECK] ... reachable ...: PASS            (PLAN, 5개 pose)
[STATE] ... -> ALIGN_MEP
[CHECK] MEP pre-grasp alignment: PASS
[CHECK] MEP grasp alignment: PASS
[CAPTURE] MEP attached to Canadarm3
[CHECK] MEP attachment: PASS
[CHECK] MEP lift (MEP follows EE, relative pose held): PASS
[CHECK] Probe/Thruster alignment (pre-dock): PASS
[CHECK] Probe insertion stays on the thruster axis (while inserting): PASS
[CHECK] Probe never touches the thruster wall: PASS
[CHECK] Probe insertion / docking conditions: PASS
[DOCK] MEP docked to satellite thruster
[CHECK] Satellite docking: PASS
[CHECK] Docked state held: PASS
[SUMMARY] N/N checks passed, final state DOCKED
```

화면에서는 EE 원기둥이 파랑 → 노랑 → 초록(부착)으로, 노즐 입구 원판이 파랑 → 노랑 → 초록(도킹)으로
바뀌고, R을 누르면 원판이 노랑으로 돌아가며 `[DOCK] MEP undocked from satellite`가 출력됩니다.

## 문제 해결

| 증상 | 확인 방법 |
|---|---|
| `ModuleNotFoundError: omni / isaaclab` | system `python3`로 실행한 경우입니다. 반드시 `~/isaac-sim/python.sh`로 실행하세요. |
| `[CHECK] ... reachable ...: FAIL` (IK 실패) | 로그의 IK 오차·관절 점프를 확인. 배치는 `DockingPlacementCfg`(`grasp_pos`, `dock_offset`)에서 바꾸며, Canadarm3 자체는 수정하지 않습니다. |
| `Prim not found: ...` | USD 경로 변경. MEP: `/World/envs/env_0/debris/{Xform_Ares1,Fermi_Gamma_ray_Large_Area_Space_Telescope}`, Satellite: `/World/envs/env_0/satellite/GOES_R/Thruster` 를 확인. |
| `MEP grasp alignment: FAIL` / 부착 안 됨 | 로그의 gap/lateral/normal/roll/approach 값 중 허용치를 넘은 항목 확인. 허용치는 `DemoCfg`(`docking_demo.py`). |
| `unexpected contact on arm link` | 팔이 MEP/Satellite에 닿음. 로그의 링크 이름과 PLAN 단계의 estimated clearance 확인. |
| `Probe/Thruster alignment: FAIL` 또는 도킹 안 됨 | radial/axis/roll/depth 값 확인. 노즐 collider는 실행 시 `GOES_R/DockingThrusterCollider`로 재구성됩니다(아래). |
| Probe가 노즐 입구에 막힘 | 원래 thruster mesh는 convexHull(꽉 찬 원뿔) collider라 막힙니다. 도킹 태스크는 이 collider를 끄고 내벽 16개 박스 + back plate로 바꿉니다. Stage 창에서 `/World/envs/env_0/satellite/GOES_R/DockingThrusterCollider` (wall_00~15, back_plate) 존재와 `Thruster/Mesh_1372`의 Collision Enabled = off 를 확인하세요. |
| `PhysicsUSD: CreateJoint - found a joint with disjointed body transforms` 경고 | 부착/도킹 순간 1번씩 나오는 **예상된 경고**입니다. Isaac Lab은 시뮬레이션 중 물리 pose를 USD에 다시 쓰지 않아 PhysX가 오래된 USD pose로 검사해서 나옵니다. joint는 실측 물리 pose로 만들며, 도킹 3초 후에도 축방향 0.07 mm로 snap이 없음을 확인했습니다. |
| timeout | 로그 `[STATUS]`의 `err`(추종 오차)와 `v`(선단 속도) 확인. 3 t MEP를 들고 있을 때는 느리게 움직이는 것이 정상입니다. |

## 초기 배치 (실행 시 계산, 실측)

| 대상 | 값 |
|---|---|
| Canadarm3 base | world 원점, 고정 (`root_joint`) |
| EE 파지 자세 (`DockingPlacementCfg`) | 접촉면 중심 `(5.0, -2.75, 3.0)`, 접근축 world `+X`(수평) |
| MEP body pose | pos `(12.8046, -2.805, 4.8084)`, quat(wxyz) `(0.5, 0.5, -0.5, 0.5)` → 부착면이 −X(팔 쪽)를 향하고 probe가 +X |
| 도킹 시 MEP 이동량 (`dock_offset`) | `(0, 5.5, 0)` m (수평 이동) |
| Satellite body(GOES_R) pose | pos `(23.9627, 2.3949, 3.1644)`, quat(wxyz) `(0.4938, -0.506, -0.4939, -0.5061)` → 노즐 입구가 −X(MEP 쪽)를 향함 |

- 두 pose는 USD에서 측정한 frame으로부터 역산합니다: "EE가 파지 자세에서 MEP 면과 면접촉 → 그 MEP를
  `dock_offset`만큼 평행이동하면 probe 선단이 `SAT_DOCK_POINT`와 정확히 일치(동축)". (배치 일관성 오차 0.002 mm)
- probe 축이 MEP 부착면 법선과 **1.35° 어긋난 에셋**이라 satellite도 그만큼 기울어진 자세로 배치됩니다.
- Satellite는 노즐 축 아래로 약 4 m 뻗어 있어서, MEP 시작 위치(노즐 축에서 −5.5 m)와 겹치지 않도록
  `dock_offset`을 노즐 축 방향이 아닌 +Y로 잡았습니다.

## Thruster collider 구성 (도킹 태스크에서만)

| 항목 | 값 |
|---|---|
| 원래 collider | GOES_R 모든 mesh `convexHull` → `Thruster/Mesh_1372`가 꽉 찬 원뿔, 입구(깊이 0)부터 축을 막음 |
| 변경 | `Thruster/Mesh_1372`의 collider 비활성 (렌더링 형상은 그대로) |
| 대체 collider | `GOES_R/DockingThrusterCollider`: 내벽을 따라 박스 16개(두께 0.03 m), 출구 반지름 0.542 m → 0.444 m, 출구 안쪽 **0.90 m에 back plate** |
| 결과 | 입구는 열려 있고(도킹 지점 내벽 반지름 0.466 m, probe 선단 반지름 0.08 m), back plate 때문에 위성 본체 관통 불가 |
| 원본 파일 | `satellite_v3.usd` 등 USD 파일은 수정하지 않음. 기존 `debris_capture(_visual)` 태스크는 그대로 convexHull |

## 검증 결과 (실제 실행)

`~/isaac-sim/python.sh project/scripts/satellite_docking_demo.py --headless --test_undock 5 --exit_after 1500`
→ **25/25 PASS, exit code 0** (같은 코드로 2회 실행, 동일 결과). 시뮬레이션 약 516 s.

| 단계 | 결과 (실측) |
|---|---|
| 5개 핵심 pose 도달성 | 모두 PASS, IK 오차 ≤ 0.09 mm, waypoint 간 최대 관절 변화 6.1° |
| 추정 clearance (mesh 정점 기준) | 팔–MEP ≥ 0.90 m, 팔–Satellite ≥ 9.43 m, 도킹 시 probe–노즐 0.45 m |
| MEP PRE-GRASP 정렬 | 위치 1.69 mm, 회전 0.032°, 법선 0.026°, roll 0.019° |
| MEP 부착 직전 | 면 간격 1.84 mm, 횡오차 0.57 mm, 법선 0.007°, roll 0.005°, 접근 방향 0.09°, 접촉으로 MEP 밀림 0.00 mm |
| LIFT (3 t MEP) | MEP–EE 상대 pose 변화 0.08 mm / 0.002° |
| Satellite PRE-DOCK 정렬 | 선단 반경 오차 1.24 mm, 축 0.007°, roll 0.005°, 출구 1.000 m 앞 |
| 삽입 중 (노즐 안, 이동 중) | 최대 축 이탈 2.33 mm, 최대 축 각도 0.009°, 벽까지 최소 여유 384 mm |
| 도킹 직전 | 축방향 0.08 mm, 반경 1.92 mm, 축 0.007°, roll 0.004°, 깊이 0.800 m, 삽입 방향 0.10°, satellite 밀림 0.00 mm |
| 도킹 유지 (3 s 후) | 축방향 0.07 mm, 반경 1.92 mm |
| R (자동 시험) | 도킹 joint만 제거, EE 부착 유지 |
| 팔 링크 비정상 접촉 | 없음 (접촉력 200 N 초과 감시) |

GUI 실행(`--exit_after 3`)으로 창 생성, `isaacsim.exp.base` 로드, INIT/PLAN, 이동 시작까지 오류 없음을 확인했습니다.

### GUI에서 사람이 확인해야 하는 것 (미실행)

- 전체 시연을 GUI로 끝까지 보기 (headless에서는 끝까지 검증됨)
- EE 원기둥 / 노즐 입구 원판의 **색상 변화**가 화면에 보이는지
- Isaac Sim 창에서 **실제 R 키**를 눌렀을 때 도킹 → 부착 순서로 하나씩 해제되는지
  (같은 콜백은 `--test_undock`으로 검증함)
- 3 t MEP를 들고 있는 동안 팔이 **느리게** 움직이는 것은 정상입니다 (전체 약 9분, 시뮬레이션 시간 기준).
