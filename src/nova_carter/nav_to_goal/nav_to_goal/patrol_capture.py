import csv
import math
import os
import threading
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

CAMERA_TOPIC = '/front_stereo_camera/left/image_raw'
LOG_ROOT = os.path.expanduser('~/patrol_logs')


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def get_euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def print_final_pose(pose_msg):
    if not pose_msg:
        return
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = math.degrees(yaw_rad)

    print("-" * 50)
    print(f"📍 최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"🧭 최종 바라보는 방향 (Heading): {yaw_deg:.1f}°")
    print("-" * 50)

def create_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = get_quaternion_from_euler(0, 0, math.radians(yaw_deg))
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


# ==========================================================
# 📷 순찰 기록기 (카메라 구독 + 사진/CSV 저장)
# ==========================================================
class PatrolRecorder(Node):
    """카메라 프레임을 계속 받아두고, 요청 시점의 최신 프레임을 저장한다."""

    def __init__(self, session_dir):
        super().__init__('patrol_recorder')
        self.session_dir = session_dir
        self.bridge = CvBridge()
        self._frame = None
        self._lock = threading.Lock()

        self.create_subscription(
            Image, CAMERA_TOPIC, self._on_image, qos_profile_sensor_data)

        self.csv_path = os.path.join(session_dir, 'patrol_log.csv')
        self._csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ['idx', 'timestamp', 'x', 'y', 'yaw_deg', 'elapsed_s', 'image', 'note'])
        self._csv_file.flush()

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # 인코딩이 다를 때도 순찰은 계속되어야 한다
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return
        with self._lock:
            self._frame = frame

    def wait_for_camera(self, timeout_s=5.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                if self._frame is not None:
                    return True
            time.sleep(0.1)
        return False

    def record(self, idx, pose_msg, elapsed_s, note=''):
        """경유지 도착 시점의 사진 1장 + 위치/시간 1행을 남긴다."""
        now = datetime.now()
        stamp = now.strftime('%Y-%m-%d %H:%M:%S')
        filename = f'wp{idx:02d}_{now.strftime("%H%M%S")}.jpg'

        with self._lock:
            frame = None if self._frame is None else self._frame.copy()

        if frame is not None:
            cv2.imwrite(os.path.join(self.session_dir, filename), frame)
        else:
            filename = 'NO_IMAGE'
            self.get_logger().warn(f'경유지 {idx}: 카메라 프레임 없음')

        if pose_msg:
            pos = pose_msg.pose.position
            ori = pose_msg.pose.orientation
            _, _, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
            x, y, yaw_deg = pos.x, pos.y, math.degrees(yaw_rad)
        else:
            x = y = yaw_deg = float('nan')

        self._csv.writerow([
            idx, stamp, f'{x:.3f}', f'{y:.3f}', f'{yaw_deg:.1f}',
            f'{elapsed_s:.1f}', filename, note])
        self._csv_file.flush()

        print(f'📷 경유지 {idx} 기록 | {stamp} | '
              f'X={x:.3f} Y={y:.3f} Heading={yaw_deg:.1f}° | {filename}')

    def close(self):
        self._csv_file.close()


# ==========================================================
# 🚀 메인 주행 로직 (다중 경유지 순찰 + 포인트별 촬영/기록)
# ==========================================================
def main():
    rclpy.init()
    nav = BasicNavigator()

    # 0. 이번 순찰 세션 폴더 준비
    session_dir = os.path.join(
        LOG_ROOT, f'patrol_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(session_dir, exist_ok=True)
    print(f'🗂️  기록 저장 위치: {session_dir}')

    recorder = PatrolRecorder(session_dir)
    # BasicNavigator가 전역 executor를 쓰므로, recorder는 별도 executor로 spin 한다
    recorder_executor = SingleThreadedExecutor()
    recorder_executor.add_node(recorder)
    spin_thread = threading.Thread(target=recorder_executor.spin, daemon=True)
    spin_thread.start()

    # 1. 출발점 설정
    init_pose = create_pose(nav, 0.002, -0.024, 0.0)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()

    if not recorder.wait_for_camera():
        print(f'⚠️ {CAMERA_TOPIC} 프레임을 아직 못 받았습니다. 사진 없이 기록만 남을 수 있습니다.')

    # 2. 경유지(Waypoints) 리스트 생성
    waypoints = []

    # 예시: 로봇이 'ㄷ'자 형태로 이동하도록 설정
    waypoints.append(create_pose(nav, -2.898265838623047, 1.5972306728363037, 0.002758026123046875))   # 경유지 1
    waypoints.append(create_pose(nav, -1.2915763854980469, -1.0673840045928955, -0.000148773193359375))  # 경유지 2
    waypoints.append(create_pose(nav, -3.528575897216797, -2.9482085704803467 , -0.0028076171875))  # 경유지 3 (최종 목적지)

    # 3. Task 실행 (goThroughPoses -> goToPose 순차 방식)
    print("🚀 다중 경유지 주행을 시작합니다...")
    total = len(waypoints)
    start_time = time.time()

    last_pose = None
    aborted = False

    for idx, wp in enumerate(waypoints, start=1):
        print(f'\n➡️  경유지 {idx}/{total} 로 이동합니다...')
        wp.header.stamp = nav.get_clock().now().to_msg()
        nav.goToPose(wp)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                last_pose = feedback.current_pose
                print(f'경유지 {idx}/{total} | '
                      f'남은 거리: {feedback.distance_remaining:.2f} m')
            time.sleep(1.0)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            note = 'arrived' if idx == total else 'passed'
            recorder.record(idx, last_pose, time.time() - start_time, note=note)
            print(f'✅ 경유지 {idx}/{total} 도착')
        elif result == TaskResult.CANCELED:
            recorder.record(idx, last_pose, time.time() - start_time,
                            note='canceled')
            print(f'\n⚠️ 경유지 {idx}에서 주행이 취소되었습니다.')
            aborted = True
            break
        else:
            recorder.record(idx, last_pose, time.time() - start_time,
                            note='failed')
            print(f'\n❌ 경유지 {idx} 주행 실패 (장애물 등으로 경로를 찾을 수 없음).')
            aborted = True
            break

    # 4. 결과 처리
    if not aborted:
        print('\n🎉 모든 경유지를 거쳐 목적지에 도착 완료!')
        print_final_pose(last_pose)

    print(f'📝 순찰 로그: {recorder.csv_path}')
    recorder.close()
    recorder_executor.shutdown()
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
