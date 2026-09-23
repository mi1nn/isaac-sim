# 아이디어 회의록

- **일시**: 2026-09-18 ~ 2026-09-19
- **브랜치 / 환경**: `feature/magnet` / Isaac Sim & Isaac Lab Space Robotics Bench
- **참석 / 기록**: 사용자 & AI 어시스턴트

---

## 1. 프로젝트 배경 및 회의 목적
- **배경**: MRV(Mission Robotic Vehicle, 서비싱 위성)의 로봇팔(Canadarm3)로 MEP(Mission Extension Pod)를 포획하여 대상 위성 thruster에 도킹하는 시스템.
- **현 상태의 한계**: 기존 구현은 정지된(고정된) 환경에서 Hardcoded 절대 좌표 기반 IK 경로를 추종하여 실제 우주 환경(유영/회전)에 부적합함.
- **핵심 목표**: 
  1. 무중력에서 유영(Tumbling/Drifting)하는 MEP 포획 환경 구축.
  2. 사전 주입된 고정 좌표 대신, **단일 손목 카메라 기반 비전 인식 및 실시간 추종 제어(PBVS)**로 고도화.
- **개발 전략 (단계별 점진 적용)**:
  - **[Phase 1 - 현재 MVP]**: 마커가 시야에 보이는 환경에서 유영(1~3 deg/s)하는 MEP를 로봇팔로 추종하여 자석 결합 완료. (MRV는 Station-Keeping 고정)
  - **[Phase 2 - 차후 확장]**: 회전축 사각지대(마커 미노출) 환경 및 MRV 본체 평행이동(Repositioning) 기동 연동.

---

## 2. 센서 및 부착점 인식 방식 결정
- **단일 카메라(Single Camera) 시스템 확정: 로봇 손목 카메라(`cam_wrist`, Eye-in-Hand)**
  - **단 1대로 가능한 이유**: AprilTag는 4개 모서리 기하 규격을 알고 있어 단안(Monocular) 카메라 1대만으로 6-DoF(3D 위치 X,Y,Z + 3축 회전) 완전 계산(solvePnP) 가능.
  - **이점**: 로봇팔의 7축 관절을 이용한 능동 짐벌 효과, 시야 가림(Occlusion) 제로, 근접할수록 밀리미터 단위 정밀도 극대화, Isaac Sim 렌더링 부하 최소화.
- **AI(딥러닝) 기반 제외**: 프로젝트 기간, 파이프라인 복잡도, 실시간 추론 지연(Latency) 문제로 배제.
- **광학 마커(AprilTag) 기반 채택**: 초저지연(100+ FPS) 및 실제 우주 협조 타깃(MEV/ISS) 표준 방식.

---

## 3. 마커 용어 및 물리/시각 인터페이스 명시적 구분
구현 시 오해와 혼동을 방지하기 위해 두 마커의 역할을 엄격히 분리:

1. **[기존 USD 중심 기하 마커] (물리 체결 대상 / Ground-Truth)**:
   - **프림 경로**: `debris/gripper_fixture/Cylinder_01` (또는 MEP 부착면 정중앙 기준점)
   - **역할**: 로봇팔 말단 자석 실린더(EE)가 물리적으로 맞닿아 결합되는 **'물리적 결합점(Ground-Truth Docking Point)'**.
   - **주의**: 비전 카메라 인식 대상이 아니며, 물리 체결 인터페이스임.
2. **[신규 광학 AprilTag 마커군] (시각 인식 전용 / 2D 비전 타깃)**:
   - **부착면 4-마커 컨스텔레이션**: 위 1번 '중심 기하 마커'를 비워두고, 부착면 외곽 **4개 모서리에 정사각형 형태(Tag Constellation)**로 배치.
     - 카메라가 이 4개 태그를 인식하여, 그 기하학적 중심인 **1번 물리 체결점의 6-DoF 위치를 역산(solvePnP)**.
   - **측면 다면 마커 (Phase 2용)**: 사각지대 해소를 위해 MEP 옆면에 배치 (어느 각도에서든 부착면 6-DoF 역산).

---

## 4. 제어 및 도킹 알고리즘 (Phase 1 기준)
- **센서 중심 상대 제어 (Sensor-Centric)**: 가상의 우주선 절대 (0,0,0) 원점을 거치지 않고, 손목 카메라(`cam_wrist`) 프레임 기준의 상대 시선 벡터(LOS) 및 마커 6-DoF 상대 오차를 직접 제어 입력으로 활용.
- **무중력 등각속도 회전 선제 요격 (Kinematic Intercept)**:
  - 타깃이 완만하게 회전(1~3 deg/s)하므로, 현재 위치가 아닌 **0.3~0.5초 뒤의 미래 도킹점 좌표를 단순 외삽 계산하여 선제 요격**.
  - 스핀 매칭(손목 지속 회전 동기화)은 관절 꼬임 위험으로 배제하고, 직진 접근 PBVS로 접촉 유도.
- **체결 조건**: 상대 거리 < 30cm, 각도 오차 < 5° 진입 시 자석 결합(`UsdPhysics.FixedJoint`) 체결.

---

## 5. 단계별 로드맵 (Phased Roadmap)
- **[Phase 1] 마커 가시 영역 내 유영 포획 (현재 구현 목표)**
  1. MRV 본체는 고정(Station-Keeping).
  2. 부착면 마커가 손목 카메라 시야 내에 들어온 상태로 MEP 유영(선속도 ~0.02 m/s, 각속도 1~3 deg/s).
  3. 4-마커 컨스텔레이션 검출 → 가상 중심점 역산 → 미래 예측 선제 요격 → PBVS 자석 결합 완성.
- **[Phase 2] 회전축 사각지대 및 MRV 평행이동 연동 (차후 고도화)**
  1. 부착면이 반대편을 향하는 사각지대 환경 구성.
  2. 측면 마커 인식 → MRV 본체 2~3m 평행이동(Translation)으로 정면 진입로 확보 후 Phase 1 시퀀스로 연결.

---

## 6. 최종 시스템 구현용 프롬프트 (System Implementation Prompt)
*(아래 프롬프트를 복사하여 차기 구현 작업 지시에 사용할 수 있습니다.)*

```markdown
# Task Prompt: MRV 기반 무중력 유영 MEP(3톤) 비전 포획 시스템 구현 및 검증 [Phase 1]

## 1. 프로젝트 배경 및 목표
Isaac Sim / Space Robotics Bench 환경에서, 서비싱 위성(MRV)에 탑재된 Canadarm3(7-DoF 로봇팔)가 무중력 상태에서 천천히 회전·유영(Tumbling)하는 3,000 kg 질량의 MEP(Mission Extension Pod)를 단일 손목 카메라(cam_wrist)와 AprilTag를 이용해 시각 추종(PBVS)하여 안전하게 자석 결합(FixedJoint)하는 시스템을 구현하고 자동으로 검증한다.

> **단계별 개발 방침**:
> - **[Phase 1 - 본 구현 범위]**: MRV 본체는 Station-Keeping(고정) 상태로 두고, 부착면 마커가 카메라 시야각 내에 보이는 상태로 유영하는 MEP를 포획하는 핵심 비전 제어 루프를 우선 완성한다.
> - **[Phase 2 - 차후 확장 과제]**: 사각지대(마커 미노출) 환경 및 MRV 본체 평행이동(Repositioning) 기동은 Phase 1 안정화 후 차후 테스트 환경으로 연동한다.

---

## 2. [중요] 마커 용어 및 물리/시각 인터페이스 명시적 구분
1. **[기존 USD 중심 기하 마커] (물리 체결 대상 / Ground-Truth)**:
   - **프림 경로**: `debris/gripper_fixture/Cylinder_01` (또는 MEP 부착면 정중앙 기준점)
   - **역할**: 로봇팔 말단 자석 실린더(EE)가 물리적으로 맞닿아 결합되는 **'물리적 결합점(Ground-Truth Docking Point)'**.
   - **주의**: 비전 카메라로 인식하는 대상이 아니며, 광학 태그에 의해 물리적으로 가려지거나 방해받아서는 안 됨.
2. **[신규 광학 AprilTag 마커군] (시각 인식 전용 / 2D 비전 타깃)**:
   - **부착면 4-마커 컨스텔레이션**: 위 1번 '중심 기하 마커'를 비워두고, 부착면 외곽 **4개 모서리에 정사각형 형태(Tag Constellation)**로 배치하는 2D 광학 텍스처 마커.
     - 카메라가 이 4개 태그를 인식하여, 그 기하학적 중심인 **1번 물리 체결점의 6-DoF 위치를 역산(solvePnP)**한다.

---

## 3. 물리 및 시뮬레이션 환경 사양
- **중력 환경**: 무중력 궤도 환경 (`sim.cfg.gravity = (0, 0, 0)`, `Domain.ORBIT`).
- **MEP 물리 설정**:
  - 질량: 3,000 kg (`PhysicsMassAPI` 적용).
  - 유영 속도: 우주 랑데부 표준 준수 (선속도 0.01~0.02 m/s, 각속도 1~3 deg/s / 약 0.02~0.05 rad/s 완만한 텀블링).
  - 초기 자세: 부착면의 4개 AprilTag가 로봇 손목 카메라 시야각(FOV) 내에 들어오는 상태로 스폰.
- **MRV (서비싱 위성)**: Station-Keeping 상태 유지 (본체 고정).
- **로봇팔**: Canadarm3 (7-DOF) + 말단 손목 카메라(`cam_wrist`, 단일 RGB Pinhole 센서) + 원통형 자석 체결 인터페이스.
- **센서 중심 제어(Sensor-Centric)**: 가상의 우주선 절대 (0,0,0) 원점을 거치지 않고, 손목 카메라(`cam_wrist`) 프레임 기준의 상대 시선 벡터(LOS) 및 마커 6-DoF 상대 오차를 직접 제어 입력으로 활용.

---

## 4. 단계별 제어 및 포획 알고리즘 (Phase 1)
1. **[1차 거시 탐색]**: 
   - 손목 카메라 시야 기준 MEP 시선 벡터(방위각) 계산 → 로봇팔 끝단을 MEP 방향으로 1차 조준(Aiming).
2. **[부착면 4-마커 포착 및 중심점 추정]**:
   - 부착면 4개 코너 태그 검출 → PnP 역산으로 정중앙의 [기존 USD 중심 기하 마커] 6-DoF 상대 포즈 실시간 계산.
1. **[1차 거시 탐색: 센서 좌표 기반 로봇팔 방위각 지향(Aiming)]**:
   - 탑재 센서가 감지한 MEP의 상대 3D 좌표값을 수신.
   - 로봇팔 베이스 기준으로 타깃의 방위각(Azimuth)과 고도각(Elevation)을 계산하여, 로봇팔 끝단(손목 카메라)의 시선 축(Look-at Vector)을 타깃 방향으로 정렬하며 팔을 전개.
   - 결과: 손목 카메라(`cam_wrist`) 화면 정중앙에 MEP 부착면이 안정적으로 포착됨.
2. **[2차 미시 도킹: 부착면 4-마커 포착 및 중심점 역산]**:
   - 부착면 4개 코너 AprilTag 검출 → PnP 역산으로 정중앙의 [기존 USD 중심 기하 마커(물리 결합점)] 6-DoF 상대 포즈 실시간 계산.
3. **[미래 위치 선제 요격(Intercept) 및 PBVS 도킹]**:
   - 1~3 deg/s의 무중력 등각속도 회전 특성을 이용해, 현재 위치가 아닌 t + 0.3초 후의 미래 결합점 좌표 선제 외삽.
   - 로봇팔 끝단을 예상 위치로 직진 접근(PBVS)시켜 부드러운 접촉 유도.
   - 부착 조건(거리 < 30cm, 각도 오차 < 5°) 만족 시 자석 결합(`UsdPhysics.FixedJoint`) 체결.

---

## 5. 단계별 검증 및 테스트 계획 (Verification Plan)
## 5. 시각화 및 디버그 오버레이 (Visualization & Debug Display)
- **마커 검출 시 시각 피드백**:
  - 손목 카메라에 부착면 4개 AprilTag가 인식되면, **4개 마커의 중심을 잇는 정사각형 외곽선(Box Line)**과 **그 정중앙에 목표 도킹점(Point / Crosshair)**이 실시간으로 표시되어야 함.
- **구현 방식 가이드**:
  1. **1순위 (Isaac Sim 기본 3D 디버그 뷰)**: Isaac Sim 내장 `omni.isaac.debug_draw` 인터페이스를 우선 활용하여 3D 월드 상에 4개 마커 연결선과 중심점을 실시간 렌더링.
  2. **2순위 (2D 비전 피드 오버레이)**: Isaac Sim 내장 3D 디버그 드로우 사용이 어렵거나 불안정할 경우, `cam_wrist`의 2D 카메라 영상에 OpenCV(`cv2.polylines`, `cv2.circle`)로 오버레이를 직접 구현하여 화면에 출력/저장.

---

## 6. 테스트 데이터 로깅 및 파일 저장 (Telemetry & Metrics Logging)
- **목적**: 제어 및 도킹 과정에서 발생하는 시각 추정 오차와 궤적 데이터를 파일로 기록하여 분석 및 검증 자료로 활용.
- **저장 파일 형식 및 경로**: `project/logs/vision_capture_metrics.csv` (또는 JSON)
- **필수 기록 항목**:
  1. 시뮬레이션 타임스탬프 (`timestamp`)
  2. 실제 물리 체결점(Ground-Truth `Cylinder_01`)의 6-DoF 좌표 (`gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw`)
  3. 비전(AprilTag PnP)으로 추정한 6-DoF 좌표 (`est_x, est_y, est_z, est_roll, est_pitch, est_yaw`)
  4. 3D 위치 오차 (`position_error_mm`) 및 각도 오차 (`angle_error_deg`)
  5. 로봇팔 끝단(EE)과 목표 도킹점 간의 상대 거리 및 상대 접근 속도
  6. 최종 도킹 성공 여부 (`capture_success: True/False`) 및 총 소요 시간

---

## 7. 단계별 검증 및 테스트 계획 (Verification Plan)
구현 완료 후 다음 3단계 자동화 테스트(Headless/GUI)를 수행하여 성공 여부를 정량 검증한다:

- [ ] **Test 1: 중심점 비전 추정 정밀도 검증 (Static Accuracy Test)**
  - 정지 상태에서 4개 AprilTag로 추정한 중심점 좌표 vs 실제 `Cylinder_01`(Ground-Truth) 좌표 비교.
  - **PASS 기준**: 위치 오차 < 5 mm, 각도 오차 < 0.5°.
- [ ] **Test 1: 중심점 비전 추정 정밀도 및 시각화 검증 (Static Accuracy & Visual Test)**
  - 정지 상태에서 4개 AprilTag로 추정한 중심점 좌표 vs 실제 `Cylinder_01`(Ground-Truth) 좌표 비교 및 오차 데이터 CSV 기록 확인.
  - 마커를 잇는 정사각형 및 중앙 점 오버레이 정상 출력 확인.
  - **PASS 기준**: 위치 오차 < 5 mm, 각도 오차 < 0.5°, CSV 파일 정상 생성.
- [ ] **Test 2: 유영 타깃(1~3 deg/s) 선제 요격 및 자석 체결 검증 (Dynamic Intercept Test)**
  - 가시 영역 내 각속도 1~3 deg/s 텀블링 상태에서 PBVS 선제 요격 기동 실행.
  - 실시간 추종 오차 및 상대 속도 데이터가 CSV에 정상 기록되는지 확인.
  - **PASS 기준**: 접촉 순간 상대 충돌 속도 < 0.05 m/s (충격 완화), 60초 이내 `[CAPTURE] CAPTURED` (FixedJoint 생성) 성공.
- [ ] **Test 3: 체결 유지 및 안정성 검증 (Holding Stability Test)**
- [ ] **Test 3: 체결 유지 및 데이터 무결성 검증 (Holding & Data Integrity Test)**
  - 결합 성공 후 시뮬레이션 10초간 유지 및 로봇팔 리셋/회수 동작 수행.
  - **PASS 기준**: PhysX 발산/폭발 없이 MEP(3톤)가 팔 끝단에 안정적으로 고정 유지됨.
  - **PASS 기준**: PhysX 발산/폭발 없이 MEP(3톤) 고정 유지, 최종 결과 리포트 파일 생성 완료.
```

