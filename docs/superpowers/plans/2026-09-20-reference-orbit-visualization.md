# Reference Orbit Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a configurable, visual-only red reference orbit to the current AprilTag vision-capture scene.

**Architecture:** A pure NumPy function validates the orbit configuration and samples a world-frame circle. A thin USD adapter authors those points as one periodic `UsdGeom.BasisCurves` prim so the line survives the existing debug-draw clear cycle without changing physics.

**Tech Stack:** Python 3.11, NumPy, pytest, OpenUSD `UsdGeom.BasisCurves`, existing YAML/dataclass configuration loader.

**Spec:** `docs/superpowers/specs/2026-09-20-reference-orbit-visualization-design.md`

## Global Constraints

- Work on the current `feature/apriltag` branch without switching branches.
- Preserve every existing modified or untracked user file; edit only the approved orbit integration areas.
- Treat the line as a scaled demonstration reference, not real orbital dynamics.
- Never write body poses or velocities to make an object follow the line.
- Add no external dependency.

---

### Task 1: Reference-orbit math

**Files:**
- Create: `project/srb/tasks/manipulation/debris_capture/orbit_reference.py`
- Create: `project/tests/test_orbit_reference.py`

**Interfaces:**
- Consumes: `OrbitReferenceCfg` numeric values.
- Produces: `sample_reference_orbit(cfg: OrbitReferenceCfg) -> np.ndarray` with shape `(samples, 3)`.

- [ ] **Step 1: Write failing geometry and validation tests**

```python
def test_samples_circle_in_requested_plane():
    cfg = OrbitReferenceCfg(center_m=(1, 2, 3), normal=(2, 0, 0), radius_m=14, samples=128)
    points = sample_reference_orbit(cfg)
    assert points.shape == (128, 3)
    assert np.max(np.abs(points[:, 0] - 1.0)) < 1e-10
    assert np.max(np.abs(np.linalg.norm(points[:, 1:] - (2, 3), axis=1) - 14.0)) < 1e-10
```

Also assert that non-positive radius, fewer than three samples, and a zero normal raise `ValueError`.

- [ ] **Step 2: Run the test and verify RED**

Run: `cd project && uv run pytest tests/test_orbit_reference.py -q`

Expected: collection fails because `orbit_reference` does not exist.

- [ ] **Step 3: Implement the minimal dataclass and sampler**

Use a robust orthonormal basis by selecting the Cartesian axis least aligned with the normalized normal, then compute `C + R(cos(theta)u + sin(theta)v)` for `endpoint=False` angles.

- [ ] **Step 4: Run the test and verify GREEN**

Run: `cd project && uv run pytest tests/test_orbit_reference.py -q`

Expected: all orbit-reference math tests pass.

### Task 2: USD visual and scene configuration

**Files:**
- Modify: `project/srb/tasks/manipulation/debris_capture/orbit_reference.py`
- Modify: `project/srb/tasks/manipulation/debris_capture/vision.py`
- Modify: `project/srb/tasks/manipulation/debris_capture/vision_task.py`
- Modify: `project/config/vision_capture.yaml`
- Modify: `project/tests/test_orbit_reference.py`

**Interfaces:**
- Consumes: `VisionCaptureConfig.orbit_reference: OrbitReferenceCfg`.
- Produces: `spawn_reference_orbit(stage, prim_path, cfg)` and one `/World/envs/env_0/reference_orbit` visual prim.

- [ ] **Step 1: Extend tests for display-value validation**

Assert that non-positive width and RGBA values outside `[0, 1]` raise `ValueError` through `cfg.validate()`.

- [ ] **Step 2: Run the new tests and verify RED**

Run: `cd project && uv run pytest tests/test_orbit_reference.py -q`

Expected: failures because display validation and the USD adapter are absent.

- [ ] **Step 3: Add the minimal USD adapter and configuration wiring**

Author a `linear`, `periodic` `BasisCurves` prim with `curveVertexCounts=[samples]`, sampled points, constant width, red `displayColor`, and opacity from the configured alpha. Call it once from `VisionCaptureTask._setup_scene()` after the environment path is known.

- [ ] **Step 4: Run focused tests and compilation**

Run: `cd project && uv run pytest tests/test_orbit_reference.py tests/test_vision_math.py -q`

Run: `cd project && python3 -m compileall -q srb scripts tests`

Expected: commands exit `0`.

### Task 3: Evidence-based project plan

**Files:**
- Create: `docs/mrv_remaining_work_plan_2026-09-21_23.md`

**Interfaces:**
- Consumes: current Git status, commits, repository code, existing requirements/technical documents, and verification results.
- Produces: the requested 16-section Korean planning document without modifying `docs/md/` user work.

- [ ] **Step 1: Write all required sections and progress tables**

Use the evidence scale and weighted formulas from the request. Mark the new red orbit as `60%` only after implementation and static verification; do not claim GUI validation.

- [ ] **Step 2: Verify document structure and readability**

Run: `test -r docs/mrv_remaining_work_plan_2026-09-21_23.md`

Run: `rg -n '^## (1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16)\.' docs/mrv_remaining_work_plan_2026-09-21_23.md`

Expected: one readable file and all 16 numbered sections.

### Task 4: Final verification

**Files:**
- Inspect: all files changed in Tasks 1–3.

**Interfaces:**
- Consumes: completed implementation and documentation.
- Produces: fresh test, compile, diff, and status evidence.

- [ ] **Step 1: Run focused and repository-safe checks**

Run: `cd project && uv run pytest tests/test_orbit_reference.py -q`

Run: `cd project && python3 -m compileall -q srb scripts tests`

Run: `git diff --check`

- [ ] **Step 2: Inspect scope and preserve user work**

Run: `git status --short --untracked-files=all`

Run: `git diff -- project/srb/tasks/manipulation/debris_capture/orbit_reference.py project/srb/tasks/manipulation/debris_capture/vision.py project/srb/tasks/manipulation/debris_capture/vision_task.py project/config/vision_capture.yaml project/tests/test_orbit_reference.py docs/mrv_remaining_work_plan_2026-09-21_23.md`

Expected: only approved additions overlap existing modified files; no untracked user file is deleted or overwritten.
