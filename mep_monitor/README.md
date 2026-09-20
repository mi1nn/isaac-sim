# STEP 2 — MOCK ONLY telemetry

This package publishes synthetic display metrics directly. It does not simulate
physics, calculate pose-based metrics, determine actual capture success, or control
a robot. STEP 4 will introduce raw mock inputs and a Monitoring Node; do not run
both direct and computed publishers on the same output topics.

## Run

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rokey/MRV_MEP_Project/monitor_ws
colcon build --packages-select mep_monitor
source install/setup.bash
ros2 run mep_monitor mock_telemetry
```

Ctrl+C stops the publisher. Restart starts a new scenario. No automatic loop.
Default 20 Hz: 20 seconds capture, 3 seconds attached hold, 20 seconds docking,
then DOCKED continues. Capture, docking start and docking events last 1 second.
Success booleans remain true after their respective scripted events. Events are
not guaranteed for late subscribers; success signals persist while running.
Elapsed time uses a steady clock and does not require Isaac /clock.

Optional short verification scenario:

```bash
ros2 run mep_monitor mock_telemetry --ros-args -p capture_seconds:=2.0 -p attached_hold_seconds:=1.0 -p docking_seconds:=2.0 -p event_seconds:=0.5
```

## Mock topic contract (not actual Isaac topic names)

All topics below are under `/mep_monitor/mock/`. QoS: reliable, volatile, depth 10.

| Suffix | std_msgs type | Unit / meaning |
|---|---|---|
| active_position_error | Float64 | m, Active Target Position Error / Remaining Distance |
| error_x, error_y, error_z | Float64 | m, synthetic components |
| orientation_error | Float64 | degrees, synthetic scalar |
| relative_velocity | Float64 | m/s, synthetic scalar |
| scenario_elapsed | Float64 | seconds since publisher start, steady clock |
| mission_phase | String | MEP_CAPTURE / SATELLITE_DOCKING |
| mission_phase_code | Int32 | 1 = capture, 2 = docking |
| mission_state | String | state name |
| mission_state_code | Int32 | 0..8, mapping below |
| active_target_type | String | MEP_ATTACH_POINT / SATELLITE_DOCK_POINT |
| tracking_valid | Bool | always true in this nominal scripted scenario |
| capture_event, docking_start_event, docking_event | Int32 | timed 0/1 pulses |
| capture_success, docking_success | Bool | scripted latched success |
| mock_only | Bool | always true |

State codes: 0 MEP_SEARCH, 1 MEP_TRACKING, 2 MEP_APPROACH, 3 MEP_ALIGN,
4 MEP_ATTACHED, 5 DOCK_SEARCH, 6 DOCK_PRE_APPROACH, 7 DOCK_ALIGN, 8 DOCKED.
Codes are visualization metadata, not a controller state machine.

Position ramps start at synthetic 0.8 m and 1.2 m. Orientation ramps start at
35 and 25 degrees, velocity at 0.08 and 0.06 m/s. These are display test amplitudes,
not thresholds. Events depend only on elapsed time. There are no thresholds.
The active distance jumps upward when the target changes; do not smooth this away.
There is no duplicate distance main topic/graph.

STEP 2 emits no pose or quaternion. XYZ uses an arbitrary mock coordinate basis
with unit direction (0.6, 0, 0.8); it has no relationship to an actual project frame.
[NEEDS_FRAME_CONFIRMATION] Actual frames, quaternion ordering, point offsets,
orientation conventions and velocity frames remain unconfirmed.
[NEEDS_ISAAC_VALIDATION] Real topics/types, capture/docking state and thresholds
remain unconfirmed. No Isaac runtime test has been performed.

Separate scalar messages are not an atomic synchronized sample and have no header.
They are intended only for STEP 2 display validation. STEP 4/6 must establish
sample timestamps and coherent inputs before implementing scientific CSV logging.
Never remap these topics into controller inputs. Numeric signals are provided as
candidates for plotting; PlotJuggler parsing, strings, annotations and ROS plugin
support have NOT been verified. PlotJuggler is currently not installed.

## Tests and rollback

`python3 -m pytest src/mep_monitor/test` (after sourcing workspace).
Only new files are added under monitor_ws; existing GOES-R.glb and controllers
are untouched. To roll back, stop the node, open a fresh ROS shell, and remove only
the newly added monitor_ws directory after retaining any desired outputs.
Package maintainer and license fields are provisional, not publication metadata.
