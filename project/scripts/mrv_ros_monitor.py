#!/usr/bin/env python3
"""ROS 2 monitor for the MRV pipeline (capture -> docking -> Client orbit return).

Run it in a SECOND terminal while `vision_capture.py --ros ...` is running (system Python + ROS 2 Jazzy,
not the Isaac Sim Python):

    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp        # same RMW as the simulator
    python3 ~/isaac_space/project/scripts/mrv_ros_monitor.py                # watch until the run ends
    python3 ~/isaac_space/project/scripts/mrv_ros_monitor.py --send-start   # with --ros_wait_start: release the arm

It subscribes to /<namespace>/{state, captured, docked, status, ee/pose, client/pose, orbit/client_error,
estimate/cylinder_pose, cam_wrist/image_raw}, prints every state change, a heartbeat, and at the end a
PASS/FAIL table: every expected topic delivered messages, and the state machine went through the
capture -> docking -> orbit-return chain. Exit code 0 only if all of that passed and the run ended in SUCCESS.
"""

import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, Float64, String

# states that end the run (the last one published stays latched on /state)
FINAL = {"SUCCESS", "TAG_LOST", "POSE_INVALID", "PREDICTION_INVALID", "APPROACH_TIMEOUT", "CAPTURE_FAILED",
         "PHYSICS_ERROR", "ABORTED", "DOCK_FAILED", "ORBIT_FAILED"}
# /state is published at `ros.publish_rate_hz` (10 Hz), so states that last only a few control steps are never
# seen (SEARCH before the monitor is up, CAPTURE_ATTEMPT / CAPTURED, DOCK_READY, DOCKED, ...). The chains below
# hold the states that last long enough; the two FixedJoints are judged from their own topics.
CAPTURE_CHAIN = ["PREDICTING", "APPROACHING", "SLOW_APPROACH", "HOLDING"]
DOCK_CHAIN = ["DOCK_TARGET_ACQUIRE", "PRE_DOCK_APPROACH", "ALIGNMENT_CHECK", "Z_APPROACH", "FINAL_INSERTION"]
ORBIT_CHAIN = ["ORBIT_TARGET_ACQUIRE", "ORBIT_TRANSFER", "ORBIT_ARRIVAL_CHECK", "ORBIT_HOLDING"]


class Monitor(Node):
    def __init__(self, ns: str, send_start: bool):
        super().__init__("mrv_ros_monitor")
        self.ns = ns.strip("/")
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        t = lambda name: f"/{self.ns}/{name}"  # noqa: E731
        self.count = {k: 0 for k in ("state", "captured", "docked", "status", "ee", "client", "orbit_error", "estimate", "image")}
        self.states = []          # state sequence, in order
        self.captured = None
        self.docked = None
        self.orbit_error = None
        self.min_orbit_error = None
        self.status = {}
        self.client = None
        self.t0 = time.time()
        self.create_subscription(String, t("state"), self.on_state, latched)
        self.create_subscription(Bool, t("captured"), self.on_captured, latched)
        self.create_subscription(Bool, t("docked"), self.on_docked, latched)
        self.create_subscription(String, t("status"), self.on_status, reliable)
        self.create_subscription(PoseStamped, t("ee/pose"), lambda m: self.bump("ee"), reliable)
        self.create_subscription(PoseStamped, t("client/pose"), self.on_client, reliable)
        self.create_subscription(Float64, t("orbit/client_error"), self.on_orbit_error, reliable)
        self.create_subscription(PoseStamped, t("estimate/cylinder_pose"), lambda m: self.bump("estimate"), reliable)
        self.create_subscription(Image, t("cam_wrist/image_raw"), lambda m: self.bump("image"), sensor)
        self.start_pub = self.create_publisher(Empty, t("cmd/start"), reliable) if send_start else None
        self.sent_start = False

    def bump(self, key):
        self.count[key] += 1

    def log(self, msg):
        print(f"[{time.time() - self.t0:7.1f}s] {msg}", flush=True)

    def on_state(self, m):
        self.bump("state")
        if not self.states or self.states[-1] != m.data:
            self.states.append(m.data)
            self.log(f"state -> {m.data}")

    def on_captured(self, m):
        self.bump("captured")
        if m.data != self.captured:
            self.captured = m.data
            self.log(f"captured (EE <-> MEP FixedJoint) = {m.data}")

    def on_docked(self, m):
        self.bump("docked")
        if m.data != self.docked:
            self.docked = m.data
            self.log(f"docked (MEP <-> Client FixedJoint) = {m.data}")

    def on_status(self, m):
        self.bump("status")
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def on_client(self, m):
        self.bump("client")
        p = m.pose.position
        self.client = (p.x, p.y, p.z)

    def on_orbit_error(self, m):
        self.bump("orbit_error")
        self.orbit_error = m.data
        self.min_orbit_error = m.data if self.min_orbit_error is None else min(self.min_orbit_error, m.data)

    def maybe_start(self):
        if self.start_pub is not None and not self.sent_start and self.states:
            self.start_pub.publish(Empty())
            self.sent_start = True
            self.log("cmd/start sent")

    def heartbeat(self):
        st = self.status
        err = f"{self.orbit_error * 1000:.0f} mm" if self.orbit_error is not None else "--"
        cl = f"({self.client[0]:.2f}, {self.client[1]:.2f}, {self.client[2]:.2f})" if self.client else "--"
        self.log(f"heartbeat: state {self.states[-1] if self.states else '--'}, sim_time {st.get('sim_time_s', '--')}, tags {st.get('tags', '--')}, "
                 f"captured {self.captured}, docked {self.docked}, client {cl}, orbit error {err} | msgs {self.count}")

    @property
    def final(self):
        return bool(self.states) and self.states[-1] in FINAL


def chain_seen(states, chain):
    """Every state of `chain` that the run must pass, in order (others may appear in between)."""
    it = iter(states)
    return all(any(s == want for s in it) for want in chain)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="mrv", help="ros.namespace of the simulator (default mrv)")
    ap.add_argument("--timeout", type=float, default=3600.0, help="give up after this many wall-clock seconds")
    ap.add_argument("--every", type=float, default=30.0, help="heartbeat period [s]")
    ap.add_argument("--send-start", action="store_true", help="publish cmd/start once the simulator is up (for --ros_wait_start)")
    ap.add_argument("--no-capture", action="store_true", help="the run skips the capture (--dock_only): do not require the capture states")
    ap.add_argument("--no-orbit", action="store_true", help="the run has no orbit return (--dock only): do not require the ORBIT states")
    args = ap.parse_args()

    rclpy.init()
    mon = Monitor(args.namespace, args.send_start)
    mon.log(f"waiting for /{mon.ns}/state (start the simulator with --ros)")
    next_beat = time.time() + args.every
    end_at = time.time() + args.timeout
    settle_until = None
    try:
        while rclpy.ok() and time.time() < end_at:
            rclpy.spin_once(mon, timeout_sec=0.2)
            mon.maybe_start()
            if time.time() >= next_beat:
                next_beat = time.time() + args.every
                mon.heartbeat()
            if mon.final:
                settle_until = settle_until or time.time() + 3.0  # let the last telemetry arrive
                if time.time() >= settle_until:
                    break
    except KeyboardInterrupt:
        pass

    # ---- verdict ----
    checks = []
    for key, needed in (("state", True), ("status", True), ("ee", True), ("captured", True), ("docked", True),
                        ("client", True), ("orbit_error", not args.no_orbit), ("estimate", not args.no_capture), ("image", not args.no_capture)):
        if needed:
            checks.append((f"topic {key}: {mon.count[key]} messages", mon.count[key] > 0))
    if not args.no_capture:
        checks.append(("capture chain " + " > ".join(CAPTURE_CHAIN), chain_seen(mon.states, CAPTURE_CHAIN)))
        checks.append(("EE <-> MEP FixedJoint (captured) became true", mon.captured is True or "CAPTURED" in mon.states))
    checks.append(("docking chain " + " > ".join(DOCK_CHAIN), chain_seen(mon.states, DOCK_CHAIN)))
    checks.append(("MEP <-> Client FixedJoint (docked) became true", mon.docked is True))
    if not args.no_orbit:
        checks.append(("orbit chain " + " > ".join(ORBIT_CHAIN), chain_seen(mon.states, ORBIT_CHAIN)))
        checks.append((f"orbit error reached <= 50 mm (min {mon.min_orbit_error * 1000:.1f} mm)" if mon.min_orbit_error is not None
                       else "orbit error reached <= 50 mm (no data)", mon.min_orbit_error is not None and mon.min_orbit_error <= 0.05))
    checks.append((f"run ended in SUCCESS (last state {mon.states[-1] if mon.states else 'none'})", bool(mon.states) and mon.states[-1] == "SUCCESS"))
    print("\n[MONITOR] ---------------- verdict ----------------")
    for name, ok in checks:
        print(f"[MONITOR] {'PASS' if ok else 'FAIL'}  {name}")
    print(f"[MONITOR] state sequence: {' > '.join(mon.states) or '(none)'}")
    print(f"[MONITOR] last status: {json.dumps(mon.status)[:400]}")
    ok_all = all(ok for _, ok in checks)
    mon.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
