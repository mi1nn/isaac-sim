"""Integration test: source the built workspace before running pytest."""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, String


def test_ros_transport_and_shutdown():
    rclpy.init()
    node = Node('mep_mock_step2_verifier')
    received = {}
    subscriptions = []
    contract = {
        Float64: ['active_position_error', 'error_x', 'error_y', 'error_z',
                  'orientation_error', 'relative_velocity', 'scenario_elapsed'],
        Int32: ['mission_phase_code', 'mission_state_code', 'capture_event',
                'docking_start_event', 'docking_event'],
        Bool: ['tracking_valid', 'capture_success', 'docking_success', 'mock_only'],
        String: ['mission_phase', 'mission_state', 'active_target_type'],
    }
    for msg_type, names in contract.items():
        for name in names:
            received[name] = []
            subscriptions.append(node.create_subscription(
                msg_type, '/mep_monitor/mock/' + name,
                lambda msg, key=name: received[key].append(msg.data), 10))
    process = subprocess.Popen([
        'ros2', 'run', 'mep_monitor', 'mock_telemetry', '--ros-args',
        '-p', 'capture_seconds:=3.0', '-p', 'attached_hold_seconds:=1.0',
        '-p', 'docking_seconds:=3.0', '-p', 'event_seconds:=0.5',
    ], start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert process.poll() is None, 'Publisher stopped unexpectedly'
        assert all(received.values()), {k: len(v) for k, v in received.items()}
        assert set(received['mission_phase']) == {'MEP_CAPTURE', 'SATELLITE_DOCKING'}
        assert set(received['mission_state_code']) == set(range(9))
        for name in ['capture_event', 'docking_start_event', 'docking_event']:
            assert 1 in received[name] and received[name][-1] == 0
        assert received['capture_success'][-1] and received['docking_success'][-1]
        assert received['mission_state'][-1] == 'DOCKED'
        assert all(received['mock_only']) and all(received['tracking_valid'])
        for name in ['active_position_error', 'orientation_error', 'relative_velocity']:
            values = received[name]
            jumps = [i for i in range(1, len(values)) if values[i] > values[i-1] + 1e-9]
            assert len(jumps) == 1, (name, jumps)
            cut = jumps[0]
            for segment in (values[:cut], values[cut:]):
                assert segment[0] > segment[-1]
                assert all(b <= a + 1e-9 for a, b in zip(segment, segment[1:]))
            assert values[-1] == 0.0
        timestamps = received['scenario_elapsed']
        rate = (len(timestamps)-1)/(timestamps[-1]-timestamps[0])
        assert 18.0 <= rate <= 22.0
        print(f'19 topics, 9 states, 3 events verified; observed {rate:.2f} Hz')
    finally:
        try:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                output, _ = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate(timeout=5)
                raise AssertionError('Mock process group did not stop on SIGINT')
            assert process.returncode == 0, output
        finally:
            node.destroy_node()
            rclpy.shutdown()
