#!/usr/bin/env python3
"""
SRB(`srb agent ros`)로 실행 중인 로봇팔이 ROS 2 명령으로 움직이는지 확인하는 테스트

동작
  1. /srb/env{N}/... 토픽이 보이는지 확인
  2. /srb/env{N}/action/cmd_vel 에 +X, -X, +Y, -Y, +Z, -Z, 회전 명령을 차례로 보냄
  3. 명령 전후 /srb/env{N}/robot/joint_states 를 비교해 관절이 움직였는지 판정
  4. /srb/env{N}/action/event (그리퍼 이벤트)로 엔드이펙터 관절이 움직이는지 확인

종료 코드: FAIL 없으면 0, 있으면 1
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def parse_args():
    p = argparse.ArgumentParser(description="SRB ROS 2 로봇팔 동작 테스트")
    p.add_argument("--env", type=int, default=0, help="환경 인덱스 (/srb/env{N})")
    p.add_argument("--cmd-topic", default=None, help="Twist 명령 토픽 (기본: /srb/env{N}/action/cmd_vel)")
    p.add_argument("--joint-topic", default=None, help="팔 관절 상태 토픽 (기본: /srb/env{N}/robot/joint_states)")
    p.add_argument("--speed", type=float, default=0.5, help="선속도 명령 크기")
    p.add_argument("--ang-speed", type=float, default=0.5, help="각속도 명령 크기")
    p.add_argument("--duration", type=float, default=1.5, help="방향별 명령 지속 시간 [s]")
    p.add_argument("--min-move", type=float, default=0.01, help="PASS 기준: 가장 많이 움직인 관절 변화량 [rad]")
    p.add_argument("--rate", type=float, default=30.0, help="명령 발행 주기 [Hz]")
    p.add_argument("--skip-gripper", action="store_true", help="그리퍼 이벤트 테스트 생략")
    args, _ = p.parse_known_args()
    return args


def twist(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z = lx, ly, lz
    t.angular.x, t.angular.y, t.angular.z = ax, ay, az
    return t


def to_dict(msg):
    return {n: p for n, p in zip(msg.name, msg.position)}


class SrbArmTester(Node):
    def __init__(self, a):
        super().__init__("srb_arm_motion_test")
        self.a = a
        self.latest = {}
        self.count = {}
        self.pubs = {}

    # ---------- ROS 유틸 ----------
    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def subscribe_js(self, topic):
        def cb(msg, topic=topic):
            self.latest[topic] = msg
            self.count[topic] = self.count.get(topic, 0) + 1
        self.create_subscription(JointState, topic, cb, qos_profile_sensor_data)

    def wait_msg(self, topic, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end and topic not in self.latest:
            rclpy.spin_once(self, timeout_sec=0.05)
        return topic in self.latest

    def publish_for(self, topic, msg_type, msg, sec):
        pub = self.pubs.get(topic)
        if pub is None:
            pub = self.pubs[topic] = self.create_publisher(msg_type, topic, 10)
        period = 1.0 / self.a.rate
        end = time.time() + sec
        while time.time() < end:
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=period)

    def moved(self, topic, action):
        before = to_dict(self.latest[topic])
        n_before = self.count.get(topic, 0)
        action()
        self.spin_for(0.3)
        after = to_dict(self.latest[topic])
        if any(not math.isfinite(v) for v in after.values()):
            return None, "NaN", 0
        diffs = {n: abs(after[n] - before[n]) for n in before if n in after}
        if not diffs:
            return 0.0, "-", self.count.get(topic, 0) - n_before
        name = max(diffs, key=diffs.get)
        return diffs[name], name, self.count.get(topic, 0) - n_before

    # ---------- 테스트 ----------
    def run(self):
        a = self.a
        ns = f"/srb/env{a.env}"
        cmd_topic = a.cmd_topic or f"{ns}/action/cmd_vel"
        js_topic = a.joint_topic or f"{ns}/robot/joint_states"
        ee_topic = f"{ns}/end_effector/joint_states"
        event_topic = f"{ns}/action/event"

        print("ROS 2 토픽 탐색 중...")
        self.spin_for(2.0)
        topics = dict(self.get_topic_names_and_types())
        srb_topics = sorted(t for t in topics if t.startswith("/srb/"))
        if not srb_topics:
            print("[FAIL] /srb/ 토픽이 없습니다. srb agent ros가 실행 중인지, "
                  "같은 컨테이너(또는 같은 ROS_DOMAIN_ID)에서 실행했는지 확인하세요.")
            return 1

        if js_topic not in topics:
            cands = [t for t in srb_topics if t.endswith("joint_states")]
            print(f"[FAIL] {js_topic} 없음. 후보: {cands}  → --joint-topic 으로 지정하세요.")
            return 1
        if cmd_topic not in topics:
            cands = [t for t in srb_topics if "geometry_msgs/msg/Twist" in topics[t]]
            print(f"[FAIL] {cmd_topic} 없음. Twist 토픽 후보: {cands}  → --cmd-topic 으로 지정하세요.")
            return 1

        self.subscribe_js(js_topic)
        has_ee = ee_topic in topics and event_topic in topics and not a.skip_gripper
        if has_ee:
            self.subscribe_js(ee_topic)

        if not self.wait_msg(js_topic):
            print(f"[FAIL] {js_topic} 메시지가 들어오지 않습니다 (시뮬레이션이 일시정지 상태인지 확인).")
            return 1

        print(f"\n명령 토픽 : {cmd_topic}")
        print(f"관절 토픽 : {js_topic}")
        print(f"관절 목록 : {list(self.latest[js_topic].name)}\n")

        s, w = a.speed, a.ang_speed
        motions = [
            ("+X", twist(lx=s)), ("-X", twist(lx=-s)),
            ("+Y", twist(ly=s)), ("-Y", twist(ly=-s)),
            ("+Z", twist(lz=s)), ("-Z", twist(lz=-s)),
            ("+Roll", twist(ax=w)), ("+Pitch", twist(ay=w)), ("+Yaw", twist(az=w)),
        ]

        results = []
        zero = twist()
        for label, cmd in motions:
            def act(cmd=cmd):
                self.publish_for(cmd_topic, Twist, cmd, a.duration)
                self.publish_for(cmd_topic, Twist, zero, 0.3)
            move, joint, n_msgs = self.moved(js_topic, act)
            if move is None:
                status = "FAIL"
            elif n_msgs == 0:
                status, joint = "FAIL", "joint_states 갱신 안 됨"
            else:
                status = "PASS" if move >= a.min_move else "FAIL"
            results.append((label, move, joint, status))
            print(f"  {label:<7} 최대 변화 {0 if move is None else move:7.4f} rad  ({joint})  {status}")

        if has_ee:
            def grip(val):
                return lambda: self.publish_for(event_topic, Bool, Bool(data=val), a.duration)
            for label, val in (("grip ON", True), ("grip OFF", False)):
                move, joint, _ = self.moved(ee_topic, grip(val))
                status = "PASS" if (move or 0) >= a.min_move else "WARN"
                results.append((label, move, joint, status))
                print(f"  {label:<7} 최대 변화 {0 if move is None else move:7.4f}      ({joint})  {status}")
        elif not a.skip_gripper:
            print("  (그리퍼 테스트 생략: end_effector/joint_states 또는 action/event 토픽 없음)")

        self.publish_for(cmd_topic, Twist, zero, 0.3)

        n_fail = sum(1 for r in results if r[3] == "FAIL")
        n_pass = sum(1 for r in results if r[3] == "PASS")
        print(f"\n요약: PASS {n_pass} / FAIL {n_fail} / 총 {len(results)}")
        print("최종 결과:", "PASS" if n_fail == 0 else "FAIL")
        return 0 if n_fail == 0 else 1


def main():
    args = parse_args()
    rclpy.init()
    node = SrbArmTester(args)
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        try:
            node.publish_for(args.cmd_topic or f"/srb/env{args.env}/action/cmd_vel", Twist, twist(), 0.2)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()