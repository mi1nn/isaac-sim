# Astrobee Satellite Inspection Progress

Spec: `docs/astrobee_satellite_inspection_updated_prompt_final.md`.
Role: Astrobee = satellite observation / monitoring camera platform. It computes nothing,
decides nothing, and its one output is the camera image (ROS 2 → web CAMERA 3).

## Current Phase

Phases 1–6 are implemented. Isaac Sim smoke test: see Verification.

## Completed

- Phase 1: checked the project.
  - Satellite: `{ENV}/satellite` (`satellite_v3.usd`, scale 3.5, body `GOES_R`). Docking port: `SAT_DOCK_POINT` (`docking.py`).
  - MEP docking code: `probe_dock.py`, `moving_dock.py`, `vision_capture_demo.py`.
  - No Astrobee asset existed in the project (`grep -ri astrobee`, `find -iname "*astrobee*"`).
- Model (spec §16): official NASA meshes from `nasa/astrobee_media/astrobee_freeflyer`
  (Apache-2.0 per its `package.xml`): body, pmc, pmc_bumper, pmc_skin_ + textures, stored in
  `assets/space_asset/astrobee/source/`. Converted with `omni.kit.asset_converter` and
  assembled like the `body` link of `nasa/astrobee` `model.urdf.xacro`
  (`project/scripts/build_astrobee_usd.py` → `assets/space_asset/astrobee/astrobee.usd`).
  Measured bbox 0.319 × 0.319 × 0.320 m; body 0.29 × 0.15 × 0.28 m (matches the URDF collision box).
  Visual only: no rigid body, no collider.
- Phase 2: kinematic 6-DoF flight (`srb/tasks/manipulation/debris_capture/astrobee.py`).
  Internal states `ASTROBEE_IDLE → APPROACH → OBSERVATION_START → OBSERVING → OBSERVATION_COMPLETE`
  (console log only). A ring around the satellite (horizontal AABB half-diagonal + 25 m,
  elevation 25°) with 4 inspection points (azimuth 45/135/225/315° from the docking-port
  direction), 5 s dwell, smooth arcs between them. The path follows the satellite's simulated
  pose (ground truth, used only to place the path; not measured or published).
- Phase 3: camera `cam_astrobee` 640×480, HFOV 70°, at the NASA SciCam mount
  (`sci_cam_transform` (0.118, 0, −0.096) m in the body frame, scaled with the model), looking
  along body +X. Aim point: midpoint between the satellite centre and its docking port
  (`look_at_dock_weight: 0.5`, spec §12). No vision processing.
- Phase 4/6: ROS 2 `/astrobee/camera/image_raw` (`sensor_msgs/Image` rgb8, BEST_EFFORT, 5 Hz sim time),
  same naming as `/mrv/cam_wrist/image_raw`. Image only: no state, TF, camera_info or status.
  Web: `mep_dashboard` CAMERA 3 to the right of CAMERA 1/2 (`backend/astrobee_feed.py` rclpy
  subscriber → `/api/camera/astrobee.jpg` → `frontend/js/camera3.js`). No Astrobee status UI.
- Phase 5: the mission is untouched. No new mission state, gate or check; `results.checks` and
  the exit code are unchanged; the Astrobee has no collider; `firebase_bridge.py` and the DB structure are unchanged.
- Config: `astrobee:` section in `project/config/vision_capture.yaml`; `--no_astrobee` turns it off.
- GUI: a `cam_astrobee` viewport window opens next to `cam_wrist`.
- GUI "Docking monitor" (`update_motion_hud`): per the user's request (2026-09-22), the second line shows
  `Astrobee |v| … m/s   rel. satellite … m/s` in every phase. It is computed from consecutive
  commanded poses (display only: not published, not stored, not used by the mission).

## In Progress

- (none)

## Pending

- Check that the full mission (capture → docking) still passes with the Astrobee on (not run this time).
- GUI (non-headless) visual check of the viewport window, the motion and the Astrobee speed line in the Docking monitor (headless only so far).
- Multi-PC check (dashboard on a second PC with the same `ROS_DOMAIN_ID`); tested on one PC only.

## Modified Files

- New: `project/srb/tasks/manipulation/debris_capture/astrobee.py`
- New: `project/scripts/build_astrobee_usd.py`
- New: `project/tests/test_astrobee_observer.py`
- New: `assets/space_asset/astrobee/` (`astrobee.usd`, `parts/*.usd`, `source/` NASA meshes + textures + `package.xml`)
- New: `mep_dashboard/backend/astrobee_feed.py`, `mep_dashboard/frontend/js/camera3.js`
- Changed: `project/config/vision_capture.yaml` (added the `astrobee:` section)
- Changed: `project/srb/tasks/manipulation/debris_capture/vision.py` (`astrobee` config field + validation)
- Changed: `project/srb/tasks/manipulation/debris_capture/vision_task.py` (Astrobee model + camera scene entities)
- Changed: `project/srb/tasks/manipulation/debris_capture/vision_capture_demo.py` (hooks in `run()` / `idle()` + `close_astrobee()`)
- Changed: `project/scripts/vision_capture.py` (`--no_astrobee`, `demo.close_astrobee()`)
- Changed: `mep_dashboard/backend/app.py`, `frontend/index.html`, `frontend/css/style.css`, `requirements.txt` (pillow), `README.md`

## Commands Executed

```bash
~/isaac-sim/python.sh project/scripts/build_astrobee_usd.py
cd project && ~/isaac-sim/python.sh -m pytest tests/test_astrobee_observer.py -q
cd project && ~/isaac-sim/python.sh -m pytest tests/test_vision_math.py tests/test_moving_dock.py tests/test_probe_dock.py tests/test_astrobee_observer.py -q
python3 -m compileall -q srb/tasks/manipulation/debris_capture scripts
ROS_DOMAIN_ID=77 ~/isaac-sim/python.sh project/scripts/vision_capture.py --headless --dock_only --tag astrobee_smoke --set "astrobee.camera={save_every_s: 5.0}"
# dashboard (ROS_DOMAIN_ID=77, port 8011) + headless Chrome screenshot at 1920x1080
```

`ROS_DOMAIN_ID=77` kept the test away from the Firebase bridge running on domain 143.

## Verification

- Offline: `test_astrobee_observer.py` 17 passed (path continuity/speed, ring radius, dwell,
  loop end, look-at rotation, camera mount axes, config validation, one Image publisher and
  no subscription, `firebase_bridge.py` has no Astrobee entries).
- Existing offline tests: the same 4 failures before and after this change
  (`test_vision_math::test_config_defaults`, `::test_config_six_dof`,
  `::test_ros_config_defaults_and_overrides`, `test_probe_dock::test_docking_config_defaults_and_overrides`:
  the config defaults (ROS on, etc.) no longer match the tests). Not caused by this work.
- Isaac Sim smoke test (headless, `num_envs=1`, `--dock_only`, first run with the old 10 m margin):
  - `ros2 topic list`: `/astrobee/camera/image_raw` is the only `/astrobee/*` topic.
  - Subscriber: 640×480 rgb8, frame_id `astrobee/cam_astrobee`.
  - Saved frames: the satellite is visible throughout, with the MEP during the approach. At the
    10 m margin the satellite overflowed the frame → margin raised to 25 m and the aim moved
    between the satellite centre and the docking port.
  - Dashboard: CAMERA 3 `● LIVE` with the image, CAMERA 1/2 unchanged (OFFLINE).
- Second run with the new config (margin 25 m, ring radius 49.4 m, 23.1 m above the satellite centre):
  - Log: `ASTROBEE_APPROACH → ASTROBEE_OBSERVATION_START → inspection point 1/4 (45°) → ASTROBEE_OBSERVING → inspection point 2/4 (135°)`.
  - Saved frames t=0…65 s (every 5 s): the satellite stays in view the whole time; the MEP and
    docking area are visible during the approach and at point 1, and the viewpoint changes along the arc.
  - Dashboard CAMERA 3 `● LIVE`, `/api/camera/astrobee.jpg` 200 (45 KB JPEG).
  - At the user's request the docking pipeline was not tested: the run was stopped after the
    Astrobee reached point 2 (sim ≈65 s). The mission result/checks with the Astrobee on were not checked.

## Issues

- The GPU was shared with another Isaac Sim instance during the first run, so the sim ran at
  ~0.1–0.18× real time (5 Hz sim ≈ 1 Hz wall). The dashboard therefore treats a frame as stale
  only after 10 s.
- `--set` supports only one level of keys: nested values need a mapping, e.g.
  `--set "astrobee.camera={save_every_s: 5.0}"`.
- In the NASA DAE files the "port" PMC visual sits on +Y (the URDF collision box is on −Y). The
  model is symmetric, so both sides look the same; the description was used as published.

## Next Step

- GUI run: `~/isaac-sim/python.sh project/scripts/vision_capture.py` and check the `cam_astrobee` window.
