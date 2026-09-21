#!/usr/bin/env python3
"""Unified monitoring output for MEP capture -> satellite docking."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import (
    Bool,
    Float64,
    Int32,
    String,
)


TOTAL_MISSION_STEPS = 7


DOCKING_STATES = {
    "DOCK_TARGET_ACQUIRE",
    "PRE_DOCK_APPROACH",
    "XY_ALIGN",
    "ORIENTATION_ALIGN",
    "ALIGNMENT_CHECK",
    "Z_APPROACH",
    "FINAL_INSERTION",
    "DOCK_READY",
    "DOCKED",
    "DOCK_HOLDING",
    "DOCK_FAILED",
}


def mission_progress(state, phase):
    """
    Final 7-stage mission progress mapping.

    1/7 MEP SEARCH
        INIT -> SEARCH -> TAG_DETECTED

    2/7 MEP CAPTURE
        POSE_ESTIMATED -> PREDICTING
        -> APPROACHING -> SLOW_APPROACH
        -> CAPTURE_ATTEMPT -> CAPTURED
        -> HOLDING -> RETREAT

    3/7 MEP ATTACHED / SATELLITE APPROACH
        DOCK_TARGET_ACQUIRE -> PRE_DOCK_APPROACH

    4/7 SATELLITE DOCKING
        XY_ALIGN -> ORIENTATION_ALIGN
        -> ALIGNMENT_CHECK -> Z_APPROACH
        -> FINAL_INSERTION -> DOCK_READY -> DOCKED

    5/7 DOCKED / ARM RELEASE
        ORBIT_TARGET_ACQUIRE

    6/7 ORBIT TRANSFER
        ORBIT_TRANSFER -> ORBIT_ARRIVAL_CHECK
        -> ORBIT_HOLDING

    7/7 ORBIT REACHED
        SUCCESS
    """

    stage_1 = {
        "INIT",
        "SEARCH",
        "TAG_DETECTED",
    }

    stage_2 = {
        "POSE_ESTIMATED",
        "PREDICTING",
        "APPROACHING",
        "SLOW_APPROACH",
        "CAPTURE_ATTEMPT",
        "CAPTURED",
        "HOLDING",
        "RETREAT",
    }

    stage_3 = {
        "DOCK_TARGET_ACQUIRE",
        "PRE_DOCK_APPROACH",
    }

    stage_4 = {
        "XY_ALIGN",
        "ORIENTATION_ALIGN",
        "ALIGNMENT_CHECK",
        "Z_APPROACH",
        "FINAL_INSERTION",
        "DOCK_READY",
        "DOCKED",
    }

    stage_5 = {
        "ORBIT_TARGET_ACQUIRE",
    }

    stage_6 = {
        "ORBIT_TRANSFER",
        "ORBIT_ARRIVAL_CHECK",
        "ORBIT_HOLDING",
    }

    stage_7 = {
        "SUCCESS",
    }

    if state in stage_1:
        return 1

    if state in stage_2:
        return 2

    if state in stage_3:
        return 3

    if state in stage_4:
        return 4

    if state in stage_5:
        return 5

    if state in stage_6:
        return 6

    if state in stage_7:
        return 7

    return 1


class ActiveMonitor(Node):

    def __init__(self):
        super().__init__(
            "active_monitor"
        )

        self.capture = self._blank()
        self.docking = self._blank()

        self.last_display_key = None

        self._create_inputs()
        self._create_outputs()

        self.create_timer(
            0.1,
            self.publish_active,
        )

        self.get_logger().info(
            "Unified Active Monitor ready."
        )

    @staticmethod
    def _blank():
        return {
            "position_error": None,
            "orientation_error": None,
            "total_velocity": None,
            "total_angular_velocity": None,
            "remaining_distance": None,
            "state": None,
            "success": False,
            "valid": False,
        }

    @staticmethod
    def _set(source, key, value):
        source[key] = value

    @staticmethod
    def _ready(source):
        required = (
            "position_error",
            "orientation_error",
            "total_velocity",
            "total_angular_velocity",
            "remaining_distance",
            "state",
        )

        return (
            source["valid"] is True
            and all(
                source[key] is not None
                for key in required
            )
        )

    def _subscribe_float(
        self,
        topic,
        source,
        key,
    ):
        self.create_subscription(
            Float64,
            topic,
            lambda msg: self._set(
                source,
                key,
                msg.data,
            ),
            10,
        )

    def _create_inputs(self):

        c = self.capture
        d = self.docking

        self._subscribe_float(
            "/mep_monitor/actual/position_error",
            c,
            "position_error",
        )

        self._subscribe_float(
            "/mep_monitor/actual/orientation_error",
            c,
            "orientation_error",
        )

        self._subscribe_float(
            "/mep_monitor/actual/relative_velocity",
            c,
            "total_velocity",
        )

        self._subscribe_float(
            "/mep_monitor/actual/relative_angular_velocity",
            c,
            "total_angular_velocity",
        )

        self._subscribe_float(
            "/mep_monitor/actual/remaining_distance",
            c,
            "remaining_distance",
        )

        self.create_subscription(
            String,
            "/mep_monitor/actual/capture_state",
            lambda msg: self._set(
                c,
                "state",
                msg.data,
            ),
            10,
        )

        self.create_subscription(
            Bool,
            "/mep_monitor/actual/capture_success",
            lambda msg: self._set(
                c,
                "success",
                msg.data,
            ),
            10,
        )

        self.create_subscription(
            Bool,
            "/mep_monitor/actual/metrics_valid",
            lambda msg: self._set(
                c,
                "valid",
                msg.data,
            ),
            10,
        )

        self._subscribe_float(
            "/mep_monitor/docking/position_error",
            d,
            "position_error",
        )

        self._subscribe_float(
            "/mep_monitor/docking/orientation_error",
            d,
            "orientation_error",
        )

        self._subscribe_float(
            "/mep_monitor/docking/total_velocity",
            d,
            "total_velocity",
        )

        self._subscribe_float(
            "/mep_monitor/docking/total_angular_velocity",
            d,
            "total_angular_velocity",
        )

        self._subscribe_float(
            "/mep_monitor/docking/remaining_distance",
            d,
            "remaining_distance",
        )

        self.create_subscription(
            String,
            "/mep_monitor/docking/current_state",
            lambda msg: self._set(
                d,
                "state",
                msg.data,
            ),
            10,
        )

        self.create_subscription(
            Bool,
            "/mep_monitor/docking/dock_success",
            lambda msg: self._set(
                d,
                "success",
                msg.data,
            ),
            10,
        )

        self.create_subscription(
            Bool,
            "/mep_monitor/docking/metrics_valid",
            lambda msg: self._set(
                d,
                "valid",
                msg.data,
            ),
            10,
        )

    def _create_outputs(self):

        base = "/mep_monitor/active"

        self.pub_position = self.create_publisher(
            Float64,
            f"{base}/position_error",
            10,
        )

        self.pub_orientation = self.create_publisher(
            Float64,
            f"{base}/orientation_error",
            10,
        )

        self.pub_velocity = self.create_publisher(
            Float64,
            f"{base}/total_velocity",
            10,
        )

        self.pub_angular_velocity = (
            self.create_publisher(
                Float64,
                f"{base}/total_angular_velocity",
                10,
            )
        )

        self.pub_remaining = self.create_publisher(
            Float64,
            f"{base}/remaining_distance",
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            f"{base}/current_state",
            10,
        )

        self.pub_phase = self.create_publisher(
            String,
            f"{base}/phase",
            10,
        )

        self.pub_target = self.create_publisher(
            String,
            f"{base}/active_target",
            10,
        )

        self.pub_progress = self.create_publisher(
            String,
            f"{base}/progress",
            10,
        )

        self.pub_progress_current = (
            self.create_publisher(
                Int32,
                f"{base}/progress_current",
                10,
            )
        )

        self.pub_progress_total = (
            self.create_publisher(
                Int32,
                f"{base}/progress_total",
                10,
            )
        )

        self.pub_valid = self.create_publisher(
            Bool,
            f"{base}/metrics_valid",
            10,
        )

    def publish_active(self):

        capture_ready = self._ready(
            self.capture
        )

        docking_ready = self._ready(
            self.docking
        )

        docking_state = str(
            self.docking["state"] or ""
        )

        use_docking = (
            docking_ready
            and self.capture["success"] is True
            and (
                docking_state in DOCKING_STATES
                or docking_state == "SUCCESS"
            )
        )

        if use_docking:
            source = self.docking
            phase = "SATELLITE_DOCKING"
            target = "SATELLITE_DOCK_POINT"

        elif capture_ready:
            source = self.capture
            phase = "MEP_CAPTURE"
            target = "MEP_ATTACH_POINT"

        else:
            self.pub_valid.publish(
                Bool(data=False)
            )
            return

        state = str(
            source["state"]
        )

        progress = mission_progress(
            state,
            phase,
        )

        self.pub_position.publish(
            Float64(
                data=float(
                    source["position_error"]
                )
            )
        )

        self.pub_orientation.publish(
            Float64(
                data=float(
                    source["orientation_error"]
                )
            )
        )

        self.pub_velocity.publish(
            Float64(
                data=float(
                    source["total_velocity"]
                )
            )
        )

        self.pub_angular_velocity.publish(
            Float64(
                data=float(
                    source[
                        "total_angular_velocity"
                    ]
                )
            )
        )

        self.pub_remaining.publish(
            Float64(
                data=float(
                    source["remaining_distance"]
                )
            )
        )

        self.pub_state.publish(
            String(data=state)
        )

        self.pub_phase.publish(
            String(data=phase)
        )

        self.pub_target.publish(
            String(data=target)
        )

        progress_text = (
            f"{progress}/"
            f"{TOTAL_MISSION_STEPS}"
        )

        self.pub_progress.publish(
            String(
                data=progress_text
            )
        )

        self.pub_progress_current.publish(
            Int32(data=progress)
        )

        self.pub_progress_total.publish(
            Int32(
                data=TOTAL_MISSION_STEPS
            )
        )

        self.pub_valid.publish(
            Bool(data=True)
        )

        key = (
            phase,
            state,
            progress,
        )

        if key != self.last_display_key:
            self.get_logger().info(
                f"{phase} | "
                f"{state} | "
                f"{progress_text} | "
                f"TARGET={target}"
            )

            self.last_display_key = key


def main(args=None):

    rclpy.init(args=args)

    node = ActiveMonitor()

    try:
        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        rclpy.executors.ExternalShutdownException,
    ):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
