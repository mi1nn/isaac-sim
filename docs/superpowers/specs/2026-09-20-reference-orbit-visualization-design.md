# Reference Orbit Visualization Design

## 목적

`srb/debris_capture_vision` 단일 환경에 시연용 축척 기준 원궤도를 빨간 선으로 표시한다. 이 선은 궤도역학이나 물체 제어가 아니라 시각적 기준이며 MEP, Client 위성, Canadarm3의 pose 또는 속도를 변경하지 않는다.

## 승인된 기준값

- 중심 `C = (0.0, 0.0, 0.0) m`
- 평면 법선 `n = (1.0, 0.0, 0.0)` — YZ 평면
- 반경 `R = 14.0 m`
- 표본 수 `128`
- 색상 RGBA `(1.0, 0.0, 0.0, 1.0)`
- 선 폭 `0.05 m`

모든 값은 `project/config/vision_capture.yaml`의 `orbit_reference` 절에서 바꿀 수 있어야 한다.

## 설계

1. `orbit_reference.py`는 NumPy만 사용하는 원 표본 계산과 값 검증을 담당한다.
2. 같은 모듈의 USD 연결 함수가 `UsdGeom.BasisCurves` 선형·주기 곡선을 만든다.
3. 곡선은 환경 prim 아래 `reference_orbit`에 생성되며 collision, rigid body, mass API를 갖지 않는다.
4. `VisionCaptureTask._setup_scene()`은 설정을 읽은 뒤 곡선을 한 번 생성한다.
5. 기존 `isaacsim.util.debug_draw`를 사용하지 않는다. 비전 시각화가 감지용 렌더 직전에 모든 debug line을 지우기 때문이다.

## 공개 인터페이스

```python
@dataclass
class OrbitReferenceCfg:
    enabled: bool
    center_m: tuple[float, float, float]
    normal: tuple[float, float, float]
    radius_m: float
    samples: int
    color_rgba: tuple[float, float, float, float]
    width_m: float

def sample_reference_orbit(cfg: OrbitReferenceCfg) -> np.ndarray:
    """Return shape `(samples, 3)` world-frame points for a closed periodic circle."""

def spawn_reference_orbit(stage, prim_path: str, cfg: OrbitReferenceCfg):
    """Author a visual-only periodic BasisCurves prim and return it."""
```

## 오류 처리

- `radius_m <= 0`: `ValueError`
- `samples < 3`: `ValueError`
- 법선 크기가 `1e-9` 이하: `ValueError`
- RGBA가 4개가 아니거나 범위 `[0, 1]` 밖: `ValueError`
- `width_m <= 0`: `ValueError`

## 검증

- 순수 수학 시험에서 모든 점의 평면 오차와 반경 오차가 `1e-10` 이하인지 확인한다.
- 입력 normal이 정규화되지 않아도 같은 원이 생성되는지 확인한다.
- 잘못된 반경·표본 수·법선을 거부하는지 확인한다.
- Python compileall과 관련 pytest를 실행한다.
- Isaac Sim GUI 실측은 별도 smoke test로 분류하며 현재 정적 시험의 성공과 혼동하지 않는다.

