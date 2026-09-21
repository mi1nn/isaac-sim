#!/usr/bin/env python3
"""ROS 2 -> Firebase (Firestore) bridge for the MRV capture / docking demo.

Subscribes to the telemetry topics of `vision_capture.py --ros` and stores them in Firestore
so that a web dashboard can read them in real time (`onSnapshot`). It is a separate process:
the simulator is not touched and never waits for the network.

Firestore layout (the two SQL-style tables of the spec, see docs/firebase_db.md):

    simulation_sessions/{session_id}                       session summary + final KPIs
    simulation_sessions/{session_id}/session_telemetry/{n} time series rows, ~5 Hz

Run (ROS 2 Jazzy sourced, same RMW as the simulator):

    source /opt/ros/jazzy/setup.bash
    pip install firebase-admin
    python3 project/scripts/firebase_bridge.py --credentials serviceAccount.json
    python3 project/scripts/firebase_bridge.py --dry_run        # print instead of writing
    FIRESTORE_EMULATOR_HOST=localhost:8080 python3 project/scripts/firebase_bridge.py --project_id demo-mrv
"""

import argparse
import json
import math
import os
import queue
import signal
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional

SESSIONS = "simulation_sessions"
TELEMETRY = "session_telemetry"

# Final states of the state machine (vision_capture_demo.py: FAILURES | SUCCESS)
TERMINAL_STATES = {
    "SUCCESS", "TAG_LOST", "POSE_INVALID", "PREDICTION_INVALID", "APPROACH_TIMEOUT", "CAPTURE_FAILED",
    "PHYSICS_ERROR", "ABORTED", "DOCK_FAILED", "MRV_APPROACH_FAILED",
}
# Docking done and held: the demo keeps running, the session is complete
FINISHED_STATES = {"DOCK_HOLDING"}
# MRV rendezvous / arm deployment: neither capture nor docking (telemetry phase "MISSION")
MISSION_STATES = {"MRV_MOVE_STEP_1", "MRV_STEP_1_REACHED", "MRV_MOVE_STEP_2", "MRV_STEP_2_REACHED", "ARM_DEPLOY"}
# Terminal failure state -> stage of the mission it belongs to (`failure_stage`)
FAILURE_STAGES = {
    "TAG_LOST": "VISION", "POSE_INVALID": "VISION", "PREDICTION_INVALID": "VISION",
    "APPROACH_TIMEOUT": "APPROACH", "MRV_APPROACH_FAILED": "APPROACH",
    "CAPTURE_FAILED": "CAPTURE", "DOCK_FAILED": "DOCKING",
    "PHYSICS_ERROR": "PHYSICS", "ABORTED": "ABORTED",
}


def _num(x) -> Optional[float]:
    """JSON / Firestore safe float (None, NaN, inf -> None)."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _scale(x, k: float) -> Optional[float]:
    x = _num(x)
    return None if x is None else x * k


###############
### Sinks ###
###############


class StdoutSink:
    """--dry_run: no network, prints what would be written."""

    def set(self, path: str, data: Dict[str, Any], merge: bool = False):
        print(f"[DB] {'MERGE' if merge else 'SET'} {path} {json.dumps(data, default=str)}", flush=True)

    def flush(self):
        pass

    def close(self):
        pass


class FirestoreSink:
    """Writes on a background thread: the ROS callbacks only enqueue (never block on the network)."""

    MAX_BATCH = 400  # Firestore limit is 500 writes per batch
    MAX_QUEUE = 20000  # ~1 h of telemetry; beyond that drop the oldest telemetry rows rather than grow

    def __init__(self, credentials: Optional[str], project_id: Optional[str]):
        import firebase_admin
        from firebase_admin import credentials as fb_cred
        from firebase_admin import firestore

        self._firestore = firestore
        if not firebase_admin._apps:
            opts = {"projectId": project_id} if project_id else None
            if credentials:
                cred = fb_cred.Certificate(credentials)
            elif os.environ.get("FIRESTORE_EMULATOR_HOST"):
                cred = None  # the emulator needs no credentials
            else:
                cred = fb_cred.ApplicationDefault()
            firebase_admin.initialize_app(cred, opts)
        self.db = firestore.client()
        self._q: "queue.Queue" = queue.Queue()
        self._idle = threading.Event()
        self._idle.set()
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="firestore-writer", daemon=True)
        self._thread.start()

    def set(self, path: str, data: Dict[str, Any], merge: bool = False):
        if self._q.qsize() >= self.MAX_QUEUE and TELEMETRY in path:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            print("[DB] write queue full, dropped the oldest telemetry row", flush=True)
        self._idle.clear()
        self._q.put((path, data, merge))

    def flush(self, timeout: float = 30.0):
        self._idle.wait(timeout)

    def close(self):
        self.flush()
        self._stop = True
        self._thread.join(timeout=5.0)

    def _run(self):
        while not (self._stop and self._q.empty()):
            try:
                items = [self._q.get(timeout=0.5)]
            except queue.Empty:
                self._idle.set()
                continue
            while len(items) < self.MAX_BATCH:
                try:
                    items.append(self._q.get_nowait())
                except queue.Empty:
                    break
            for attempt in range(5):
                try:
                    batch = self.db.batch()
                    for path, data, merge in items:
                        batch.set(self.db.document(path), self._encode(data), merge=merge)
                    batch.commit()
                    break
                except Exception as e:  # network / quota: retry, then drop the batch, never crash the bridge
                    print(f"[DB] write failed ({e}); retry {attempt + 1}/5", flush=True)
                    time.sleep(min(2.0 ** attempt, 10.0))
            else:
                print(f"[DB] dropped {len(items)} writes", flush=True)
            if self._q.empty():
                self._idle.set()

    def _encode(self, data: Dict[str, Any]) -> Dict[str, Any]:
        return {k: (self._firestore.SERVER_TIMESTAMP if v == "__SERVER_TIMESTAMP__" else v) for k, v in data.items()}


########################
### Session recorder ###
########################


class SessionRecorder:
    """Turns the ROS messages into rows. No ROS / Firebase imports: unit-testable with plain dicts.

    One /status message (sim time, ~10 Hz) triggers one telemetry row, throttled to `rate_hz`; the
    latest poses are attached. A session runs from the first non-terminal status to a terminal
    state (SUCCESS / failure) or `finish()`; the KPIs are written to the summary once.
    """

    def __init__(self, sink, rate_hz: float = 5.0, session_id: Optional[str] = None, idle_timeout_s: float = 30.0):
        self.sink = sink
        self.period = 1.0 / max(rate_hz, 1e-6)
        self.fixed_id = session_id
        self.idle_timeout_s = idle_timeout_s
        self.session_id: Optional[str] = None
        self._used_fixed_id = False
        self._reset_latest()

    def _reset_latest(self):
        self.poses: Dict[str, Optional[tuple]] = {"ee": None, "gt": None, "goal": None, "est": None, "probe": None, "dock": None}
        self.state: Optional[str] = None
        self.captured = False
        self._status_captured = False
        self.docked = False
        self.contact_vel: Optional[float] = None
        self.dock_enabled = False  # the run includes the docking phase (`--dock`)
        self.docking_started = False
        self.docking_success = False
        self.capture_metrics: Dict[str, Optional[float]] = {}  # est_* at the moment of contact
        self.capture_time: Optional[float] = None
        self.dock_t0: Optional[float] = None
        self.dock_t_end: Optional[float] = None  # sim time of the docking joint
        # Latest docking metrics (frozen once docked, so the holding phase does not overwrite the result)
        self.last_docking_metrics: Dict[str, Optional[float]] = {
            "relative_x": None, "relative_y": None, "relative_z": None, "lateral_error": None, "orientation_error": None,
            "insertion_depth": None, "relative_speed": None,
        }
        self.seq = 0
        self.last_row_t = -math.inf
        self.last_status: Dict[str, Any] = {}
        self.last_msg_wall = time.time()

    ## ROS callbacks (plain data in)
    def on_pose(self, key: str, xyz: tuple):
        self.poses[key] = xyz

    def on_state(self, state: str):
        self.state = state

    def on_captured(self, captured: bool):
        self.captured = bool(captured)

    def on_status(self, s: Dict[str, Any]):
        self.last_msg_wall = time.time()
        state = s.get("state") or self.state
        t = _num(s.get("sim_time_s"))
        if t is None:
            return
        terminal = state in TERMINAL_STATES or state in FINISHED_STATES
        # A new run (simulator restarted): the clock goes back, or a live state follows a finished session
        if self.session_id is not None and t < self.last_status.get("sim_time_s", 0.0) - 1.0:
            self.finish()
        if self.session_id is None:
            if terminal:
                return  # leftover of the previous run
            self._start()
        if "captured" in s:
            self.captured = bool(s["captured"])
        # Compare with the previous /status, not with /captured (that topic arrives first)
        if self.captured and not self._status_captured:
            self.contact_vel = _num(s.get("est_rel_vel"))  # impact speed at the moment of contact
            self.capture_time = t
            self.capture_metrics = {k: _num(s.get(f"est_{k}")) for k in ("distance", "lateral", "orientation", "rel_vel")}
        self._status_captured = self.captured
        self._track_docking(t, s)
        self.last_status = {**s, "sim_time_s": t, "state": state}
        if t - self.last_row_t >= self.period - 1e-9 or terminal:
            self.last_row_t = t
            self._write_row(t, state, s)
        if terminal:
            self.finish()

    def _track_docking(self, t: float, s: Dict[str, Any]):
        self.dock_enabled = self.dock_enabled or bool(s.get("dock_enabled"))
        docked = bool(s.get("dock_docked"))
        if s.get("dock_active"):
            self.dock_enabled = True
            if not self.docking_started:
                self.docking_started, self.dock_t0 = True, t
            if not self.docking_success:  # frozen from the moment the joint exists
                self.last_docking_metrics = {
                    "relative_x": _num(s.get("dock_relative_x")), "relative_y": _num(s.get("dock_relative_y")),
                    "relative_z": _num(s.get("dock_relative_z")), "lateral_error": _num(s.get("dock_lateral")),
                    "orientation_error": _num(s.get("dock_orientation_deg")),
                    "insertion_depth": _num(s.get("dock_insertion_depth")), "relative_speed": _num(s.get("dock_rel_speed")),
                }
                if docked:
                    self.dock_t_end = t
        self.docking_success = self.docking_success or docked
        self.docked = self.docking_success

    ## Session lifecycle
    def _new_id(self) -> str:
        if self.fixed_id and not self._used_fixed_id:
            self._used_fixed_id = True
            return self.fixed_id
        return "run_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    def _start(self):
        self._reset_latest_keep_clock()
        self.session_id = self._new_id()
        self.sink.set(f"{SESSIONS}/{self.session_id}", {
            "session_id": self.session_id,
            "created_at": "__SERVER_TIMESTAMP__",
            "is_running": True,
            "capture_success": None, "docking_success": None, "mission_success": None,
            "failure_stage": None, "failure_reason": None, "duration_sec": None,
            "final_distance_m": None, "final_angle_deg": None, "contact_vel_mps": None,
            "capture_position_error_m": None, "capture_lateral_error_m": None, "capture_orientation_error_deg": None,
            "capture_contact_velocity_mps": None, "capture_time_s": None,
            "docking_position_error_m": None, "docking_lateral_error_m": None, "docking_orientation_error_deg": None,
            "docking_insertion_depth_m": None, "docking_relative_velocity_mps": None, "docking_time_s": None,
        })
        print(f"[DB] session {self.session_id} started", flush=True)

    def _reset_latest_keep_clock(self):
        self.seq, self.last_row_t, self.contact_vel, self._status_captured, self.docked = 0, -math.inf, None, False, False
        self.dock_enabled = self.docking_started = self.docking_success = False
        self.capture_metrics, self.capture_time, self.dock_t0, self.dock_t_end = {}, None, None, None
        self.last_docking_metrics = dict.fromkeys(self.last_docking_metrics)

    def _write_row(self, t: float, state: Optional[str], s: Dict[str, Any]):
        docking = bool(s.get("dock_active"))
        if docking:
            # Docking phase: the est_* capture metrics (EE vs cylinder) no longer apply. The error columns carry the
            # probe tip -> SAT_DOCK_POINT errors, and the vision estimate / capture goal (frozen since capture) are null.
            errors = {
                "distance_m": _num(s.get("dock_distance")),  # remaining insertion distance
                "lateral_error_mm": _scale(s.get("dock_lateral"), 1000.0),
                "angle_error_deg": _num(s.get("dock_axis_deg")),  # probe axis vs docking axis
                "rel_vel_mps": _num(s.get("dock_rel_speed")),
            }
            poses = {"est": None, "goal": None}
        else:
            errors = {
                "distance_m": _num(s.get("est_distance")),
                "lateral_error_mm": _scale(s.get("est_lateral"), 1000.0),
                "angle_error_deg": _num(s.get("est_orientation")),
                "rel_vel_mps": _num(s.get("est_rel_vel")),
            }
            poses = {}
        pose = lambda key: None if key in poses else self.poses[key]  # noqa: E731

        def xyz(key, prefix):
            p = pose(key)
            return {f"{prefix}_{a}": (_num(v) if p else None) for a, v in zip("xyz", p or (None,) * 3)}

        phase = "DOCKING" if docking else "MISSION" if state in MISSION_STATES else "CAPTURE"
        dock = lambda key, k=1.0: _scale(s.get(key), k) if docking else None  # noqa: E731
        row = {
            "session_id": self.session_id, "sim_time": t, "state": state, "phase": phase,
            **xyz("ee", "ee"), **xyz("gt", "target"), **xyz("goal", "goal"), **xyz("est", "est"),
            **xyz("probe", "probe"), **xyz("dock", "dock"),
            **errors,
            "rel_ang_vel_rad_s": None if docking else _num(s.get("est_rel_ang_vel")),
            "is_captured": self.captured,
            # docking only (null in the capture phase)
            "dock_roll_deg": _num(s.get("dock_roll_deg")) if docking else None,
            "insertion_depth_m": _num(s.get("dock_insertion_depth")) if docking else None,
            "wall_clearance_mm": _scale(s.get("dock_clearance"), 1000.0) if docking else None,
            "is_docked": bool(s.get("dock_docked")) if docking else False,
            "dock_relative_x_m": dock("dock_relative_x"), "dock_relative_y_m": dock("dock_relative_y"),
            "dock_relative_z_m": dock("dock_relative_z"),
            "dock_lateral_error_m": dock("dock_lateral"), "dock_orientation_error_deg": dock("dock_orientation_deg"),
            "dock_geometry_distance_m": dock("dock_distance"), "dock_insertion_depth_m": dock("dock_insertion_depth"),
            "dock_relative_speed_mps": dock("dock_rel_speed"),
            "dock_ready": bool(s.get("dock_ready")) if docking else False,
            "dock_success": bool(s.get("dock_docked")) if docking else False,
        }
        self.sink.set(f"{SESSIONS}/{self.session_id}/{TELEMETRY}/{self.seq:07d}", {"id": self.seq, **row})
        self.seq += 1

    def finish(self):
        """Final KPIs into the summary (idempotent)."""
        if self.session_id is None:
            return
        s = self.last_status
        failure = s.get("failure")
        capture_success = bool(self.captured)
        docking_success = bool(self.docking_success)
        # A capture-only run (no `--dock`) has no docking to fail: docking_success stays null, mission = capture
        mission_success = capture_success and (docking_success if self.dock_enabled else True)
        cm, dm = self.capture_metrics, self.last_docking_metrics
        dock_xyz = [dm.get(k) for k in ("relative_x", "relative_y", "relative_z")]
        dock_pos = math.sqrt(sum(v * v for v in dock_xyz)) if all(v is not None for v in dock_xyz) else None
        dock_end = self.dock_t_end if self.dock_t_end is not None else _num(s.get("sim_time_s"))
        self.sink.set(f"{SESSIONS}/{self.session_id}", {
            "is_running": False,
            "duration_sec": _num(s.get("sim_time_s")),
            "capture_success": capture_success,
            "docking_success": docking_success if self.dock_enabled else None,
            "mission_success": mission_success,
            "failure_stage": FAILURE_STAGES.get(str(failure).split(":")[0]) if failure else None,
            "failure_reason": failure if failure else None,
            "capture_position_error_m": cm.get("distance"),
            "capture_lateral_error_m": cm.get("lateral"),
            "capture_orientation_error_deg": cm.get("orientation"),
            "capture_contact_velocity_mps": self.contact_vel,
            "capture_time_s": self.capture_time,
            "docking_position_error_m": dock_pos,
            "docking_lateral_error_m": dm.get("lateral_error"),
            "docking_orientation_error_deg": dm.get("orientation_error"),
            "docking_insertion_depth_m": dm.get("insertion_depth"),
            "docking_relative_velocity_mps": dm.get("relative_speed"),
            "docking_time_s": (dock_end - self.dock_t0) if self.dock_t0 is not None and dock_end is not None else None,
            "final_distance_m": _num(s.get("dock_distance" if s.get("dock_active") else "est_distance")),
            "final_angle_deg": _num(s.get("dock_axis_deg" if s.get("dock_active") else "est_orientation")),
            "contact_vel_mps": self.contact_vel,
            "final_state": s.get("state"),
            "is_docked": docking_success,
        }, merge=True)
        self.sink.flush()
        print(f"[DB] session {self.session_id} finished: capture={capture_success} docking={docking_success} mission={mission_success} failure={failure}", flush=True)
        self.session_id = None

    def check_idle(self):
        """The simulator died without a terminal state: close the session after `idle_timeout_s`."""
        if self.session_id is not None and time.time() - self.last_msg_wall > self.idle_timeout_s:
            print("[DB] no /status for a while, closing the session", flush=True)
            self.finish()


################
### ROS node ###
################


def run_ros(rec: SessionRecorder, ns: str):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    node = Node("firebase_bridge")
    lock = threading.Lock()  # the executor is single threaded, the lock guards the idle check from the main thread

    def pose_cb(key):
        def cb(m):
            with lock:
                p = m.pose.position
                rec.on_pose(key, (p.x, p.y, p.z))
        return cb

    def status_cb(m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        with lock:
            rec.on_status(s)

    def state_cb(m):
        with lock:
            rec.on_state(m.data)

    def captured_cb(m):
        with lock:
            rec.on_captured(m.data)

    for key, topic in (("ee", "ee/pose"), ("gt", "gt/cylinder_pose"), ("goal", "ee/target_pose"), ("est", "estimate/cylinder_pose"),
                       ("probe", "dock/probe_pose"), ("dock", "dock/target_pose")):
        node.create_subscription(PoseStamped, f"/{ns}/{topic}", pose_cb(key), reliable)
    node.create_subscription(String, f"/{ns}/state", state_cb, latched)
    node.create_subscription(Bool, f"/{ns}/captured", captured_cb, latched)
    node.create_subscription(String, f"/{ns}/status", status_cb, reliable)
    print(f"[DB] listening on /{ns}/ ...", flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            with lock:
                rec.check_idle()
    finally:
        with lock:
            rec.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="mrv", help="ROS namespace of the simulator topics (ros.namespace)")
    ap.add_argument("--credentials", default=os.environ.get("GOOGLE_APPLICATION_CREDENTIALS"),
                    help="service account JSON (default: $GOOGLE_APPLICATION_CREDENTIALS)")
    ap.add_argument("--project_id", default=None, help="Firebase project id (default: from the credentials)")
    ap.add_argument("--rate_hz", type=float, default=5.0, help="telemetry rows per second of simulation time (5-10)")
    ap.add_argument("--session_id", default=None, help="id of the first session (default: run_YYYYMMDD_HHMMSS)")
    ap.add_argument("--idle_timeout", type=float, default=30.0, help="close a session after this many wall seconds without /status")
    ap.add_argument("--dry_run", action="store_true", help="print the writes instead of sending them")
    args = ap.parse_args()

    sink = StdoutSink() if args.dry_run else FirestoreSink(args.credentials, args.project_id)
    rec = SessionRecorder(sink, args.rate_hz, args.session_id, args.idle_timeout)
    try:
        run_ros(rec, args.namespace.strip("/"))
    finally:
        sink.close()


if __name__ == "__main__":
    sys.exit(main())
