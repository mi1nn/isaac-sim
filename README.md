## 17. `feature/merge`

### 이슈와 수정

- MRV 평행 이동과 Firebase 저장 기능이 서로 다른 브랜치에 있었다.
- `feature/linear-movement`와 `feature/db` 계보를 병합했다.
- tip의 직접 변경은 `.gitignore` 중심이다.

### 실행 명령

```bash
git switch feature/merge
source /opt/ros/jazzy/setup.bash
~/isaac-sim/python.sh project/scripts/vision_capture.py \
  --scenario dynamic --ros --dock --tag merged_pipeline

# 별도 터미널
python3 project/scripts/firebase_bridge.py --dry_run
```

### 결과

- 현재: 컴파일 통과, Isaac Sim 환경 50개 등록.
- 제한: 병합 브랜치 자체에서 이동+도킹+DB 전체를 다시 통과한 전용 회귀 기록은 확인되지 않았다.
