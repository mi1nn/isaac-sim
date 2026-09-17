#!/usr/bin/env python3
"""
SRB ROS 2 로봇팔 동작 테스트 v2 — 로봇 크기·자유도(6축/7축) 무관

v1과 차이
  - "관절 하나가 몇 rad 움직였나" 대신 "말단이 몇 mm / 몇 도 움직였나"(TF)로 판정
    → Canadarm3처럼 긴 팔이나 7축 여유자유도 팔에서도 기준이 일관됨
  - 전체 관절 변화량은 노름(모든 관절 합산)으로 참고 출력
  - +/- 명령이 서로 반대 방향으로 움직이는지 방향성 검사
  - TF 프레임을 못 찾으면 관절 노름 기준으로 자동 전환

실행 (컨테이너 안, 반드시 시스템 파이썬):
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 test_srb_ros_arm_v2.py
  /usr/bin/python3 test_srb_ros_arm_v2.py --list-frames      # TF 프레임 확인
  /usr/bin/python3 test_srb_ros_arm_v2.py --base-frame A --ee-frame B

종료 코드: FAIL 없으면 0, 있으면 1
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

EE_KEYS = ("end_effector", "tcp", "flange", "tool", "ee_link", "hand", "gripper")


def parse_args():
    p = argparse.ArgumentParser(description="SRB ROS 2 로봇팔 동작 테스트 v2 (말단 이동량 기준)")
    p.add_argument("--env", type=int, default=0, help="환경 인덱스 (/srb/env{N})")
    p.add_argument("--cmd-topic", default=None, help="Twist 명령 토픽 (기본: /srb/env{N}/action/cmd_vel)")
    p.add_argument("--joint-topic", default=None, help="팔 관절 토픽 (기본: /srb/env{N}/robot/joint_states)")
    p.add_argument("--base-frame", default=None, help="기준 TF 프레임 (기본: 자동 추정)")
    p.add_argument("--ee-frame", default=None, help="말단 TF 프레임 (기본: 자동 추정)")
    p.add_argument("--list-frames", action="store_true", help="TF 프레임 목록만 출력하고 종료")
    p.add_argument("--speed", type=float, default=0.5, help="선속도 명령 크기")
    p.add_argument("--ang-speed", type=float, default=0.5, help="각속도 명령 크기")
    p.add_argument("--duration", type=float, default=1.5, help="방향별 명령 지속 시간 [s]")
    p.add_argument("--min-dist", type=float, default=0.01, help="PASS 기준: 말단 이동거리 [m]")
    p.add_argument("--min-angle", type=float, default=1.0, help="PASS 기준: 말단 회전각 [deg]")
    p.add_argument("--min-joint-norm", type=float, default=0.005,
                   help="TF를 못 쓸 때 PASS 기준: 전체 관절 변화 노름 [rad]")
    p.add_argument("--rate", type=float, default=30.0, help="명령 발행 주기 [Hz]")
    p.add_argument("--skip-gripper", action="store_true", help="그리퍼 테스트 생략")
    args, _ = p.parse_known_args()
    return args


# ---------------- 수학 유틸 ----------------
def twist(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z = lx, ly, lz
    t.angular.x, t.angular.y, t.angular.z = ax, ay, az
    return t


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotvec(q0, q1):
    """q0 → q1 상대 회전을 회전벡터(축*각도)로 반환"""
    x, y, z, w = qmul(q1, (-q0[0], -q0[1], -q0[2], q0[3]))
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    ang = 2.0 * math.atan2(s, w)
    if s < 1e-12:
        return (0.0, 0.0, 0.0), 0.0
    return (x / s * ang, y / s * ang, z / s * ang), ang


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def axis_label(v):
    if v is None or norm(v) < 1e-9:
        return "-"
    i = max(range(3), key=lambda k: abs(v[k]))
    return ("+" if v[i] > 0 else "-") + "xyz"[i]


def guess_frames(frames, joint_names):
    names = set(frames)
    for info in frames.values():
        if isinstance(info, dict) and info.get("parent"):
            names.add(info["parent"])

    prefix = ""
    if joint_names and "_joint" in joint_names[0]:
        prefix = joint_names[0].split("_joint")[0].lower()

    def ok(f):
        f = f.lower()
        return "cam" not in f and "finger" not in f

    ee = None
    for key in EE_KEYS:
        c = sorted((f for f in names if key in f.lower() and ok(f)), key=len)
        if c:
            ee = c[0]
            break
    if ee is None and prefix:
        links = [f for f in names if prefix in f.lower() and ok(f) and "base" not in f.lower()]
        if links:
            ee = sorted(links, key=lambda s: (len(s), s))[-1]

    c = sorted((f for f in names if "base" in f.lower() and ok(f)
                and (not prefix or prefix in f.lower())), key=len)
    if not c:
        c = sorted((f for f in names if "base" in f.lower() and ok(f)), key=len)
    base = c[0] if c else None
    if base is None:
        roots = [i.get("parent") for i in frames.values()
                 if isinstance(i, dict) and i.get("parent") and i.get("parent") not in frames]
        base = roots[0] if roots else None
    return base, ee


# ---------------- 노드 ----------------
class SrbArmTesterV2(Node):
    def __init__(self, a):
        super().__init__("srb_arm_motion_test_v2")
        self.a = a
        self.latest, self.count, self.pubs = {}, {}, {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base = self.ee = None

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

    def get_frames(self):
        try:
            data = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}

    def ee_pose(self, base=None, ee=None):
        base, ee = base or self.base, ee or self.ee
        if not base or not ee:
            return None
        try:
            t = self.tf_buffer.lookup_transform(base, ee, Time())
        except Exception:
            return None
        tr, r = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w)

    def joints(self, topic):
        m = self.latest[topic]
        return {n: p for n, p in zip(m.name, m.position)}

    def measure(self, cmd_topic, js_topic, cmd):
        p0, j0 = self.ee_pose(), self.joints(js_topic)
        n0 = self.count.get(js_topic, 0)
        self.publish_for(cmd_topic, Twist, cmd, self.a.duration)
        self.publish_for(cmd_topic, Twist, twist(), 0.3)
        self.spin_for(0.3)
        p1, j1 = self.ee_pose(), self.joints(js_topic)

        r = {"n_msgs": self.count.get(js_topic, 0) - n0, "nan": False}
        if any(not math.isfinite(v) for v in j1.values()):
            r["nan"] = True
        common = [n for n in j0 if n in j1]
        r["jnorm"] = math.sqrt(sum((j1[n] - j0[n]) ** 2 for n in common)) if common else 0.0
        if p0 and p1:
            r["disp"] = tuple(b - a for a, b in zip(p0[0], p1[0]))
            r["dist"] = norm(r["disp"])
            r["rotv"], r["angle"] = rotvec(p0[1], p1[1])
        else:
            r["disp"] = r["rotv"] = None
            r["dist"] = r["angle"] = None
        return r

    def run(self):
        a = self.a
        ns = f"/srb/env{a.env}"
        cmd_topic = a.cmd_topic or f"{ns}/action/cmd_vel"
        js_topic = a.joint_topic or f"{ns}/robot/joint_states"
        ee_js_topic = f"{ns}/end_effector/joint_states"
        event_topic = f"{ns}/action/event"

        print("ROS 2 토픽 / TF 탐색 중...")
        self.spin_for(2.0)
        topics = dict(self.get_topic_names_and_types())
        srb_topics = sorted(t for t in topics if t.startswith("/srb/"))
        if not srb_topics:
            print("[FAIL] /srb/ 토픽이 없습니다. srb agent ros 실행 여부와 ROS_DOMAIN_ID를 확인하세요.")
            return 1
        for need, opt in ((js_topic, "--joint-topic"), (cmd_topic, "--cmd-topic")):
            if need not in topics:
                print(f"[FAIL] {need} 없음 → {opt} 로 지정하세요. /srb 토픽: {srb_topics}")
                return 1

        self.subscribe_js(js_topic)
        has_grip = (not a.skip_gripper) and ee_js_topic in topics and event_topic in topics
        if has_grip:
            self.subscribe_js(ee_js_topic)
        if not self.wait_msg(js_topic):
            print(f"[FAIL] {js_topic} 메시지가 들어오지 않습니다 (시뮬레이션 일시정지 여부 확인).")
            return 1

        joint_names = list(self.latest[js_topic].name)
        self.spin_for(1.0)  # TF 버퍼 채우기
        frames = self.get_frames()

        if a.list_frames:
            print("\n=== TF frames (child <- parent) ===")
            for f in sorted(frames):
                parent = frames[f].get("parent", "?") if isinstance(frames[f], dict) else "?"
                print(f"  {f}  <-  {parent}")
            return 0

        gb, ge = guess_frames(frames, joint_names)
        self.base, self.ee = a.base_frame or gb, a.ee_frame or ge
        use_tf = self.ee_pose() is not None

        print(f"\n명령 토픽 : {cmd_topic}")
        print(f"관절 토픽 : {js_topic}  (DOF={len(joint_names)})")
        if use_tf:
            print(f"판정 방식 : 말단 TF 이동량  [{self.base} → {self.ee}]")
            print(f"PASS 기준 : 이동 ≥ {a.min_dist * 1000:.0f} mm (선속도), 회전 ≥ {a.min_angle:.1f}° (각속도)")
        else:
            print(f"[WARN] TF 프레임을 찾지 못했습니다 (base={self.base}, ee={self.ee}). "
                  "--list-frames 로 확인 후 --base-frame/--ee-frame 지정을 권장합니다.")
            print(f"판정 방식 : 전체 관절 변화 노름 ≥ {a.min_joint_norm} rad")

        s, w = a.speed, a.ang_speed
        motions = [
            ("+X", "lin", twist(lx=s)), ("-X", "lin", twist(lx=-s)),
            ("+Y", "lin", twist(ly=s)), ("-Y", "lin", twist(ly=-s)),
            ("+Z", "lin", twist(lz=s)), ("-Z", "lin", twist(lz=-s)),
            ("+Roll", "ang", twist(ax=w)), ("-Roll", "ang", twist(ax=-w)),
            ("+Pitch", "ang", twist(ay=w)), ("-Pitch", "ang", twist(ay=-w)),
            ("+Yaw", "ang", twist(az=w)), ("-Yaw", "ang", twist(az=-w)),
        ]

        print(f"\n  {'동작':<7} {'이동[mm]':>9} {'회전[deg]':>10} {'관절노름[rad]':>13} {'주방향':>6}  결과")
        print("  " + "-" * 62)
        results = {}
        min_ang = math.radians(a.min_angle)
        for label, kind, cmd in motions:
            r = self.measure(cmd_topic, js_topic, cmd)
            if r["nan"]:
                status = "FAIL"
            elif r["n_msgs"] == 0:
                status = "FAIL"
            elif use_tf and r["dist"] is not None:
                ok = r["dist"] >= a.min_dist if kind == "lin" else r["angle"] >= min_ang
                status = "PASS" if ok else "FAIL"
            else:
                status = "PASS" if r["jnorm"] >= a.min_joint_norm else "FAIL"
            r["status"] = status
            results[label] = r

            dist = "-" if r["dist"] is None else f"{r['dist'] * 1000:.1f}"
            ang = "-" if r["angle"] is None else f"{math.degrees(r['angle']):.2f}"
            vec = r["disp"] if kind == "lin" else r["rotv"]
            note = " (NaN)" if r["nan"] else (" (joint_states 갱신 없음)" if r["n_msgs"] == 0 else "")
            print(f"  {label:<7} {dist:>9} {ang:>10} {r['jnorm']:>13.4f} {axis_label(vec):>6}  {status}{note}")

        warns = 0
        if use_tf:
            print("\n방향성 검사 (반대 명령 → 반대 방향으로 움직이는지)")
            for axis, kind in (("X", "lin"), ("Y", "lin"), ("Z", "lin"),
                               ("Roll", "ang"), ("Pitch", "ang"), ("Yaw", "ang")):
                rp, rm = results[f"+{axis}"], results[f"-{axis}"]
                key = "disp" if kind == "lin" else "rotv"
                if rp[key] is None or rm[key] is None:
                    continue
                d = dot(rp[key], rm[key])
                ok = d < 0
                warns += 0 if ok else 1
                print(f"  {axis:<6} {'OK' if ok else 'WARN (반대 방향이 아님)'}")

        if has_grip:
            print("\n그리퍼")
            for label, val in (("grip ON", True), ("grip OFF", False)):
                before = self.joints(ee_js_topic)
                self.publish_for(event_topic, Bool, Bool(data=val), a.duration)
                self.spin_for(0.3)
                after = self.joints(ee_js_topic)
                diffs = {n: abs(after[n] - before[n]) for n in before if n in after}
                mv = max(diffs.values()) if diffs else 0.0
                st = "PASS" if mv >= 0.01 else "WARN"
                warns += 0 if st == "PASS" else 1
                print(f"  {label:<8} 최대 관절 변화 {mv:.4f}  {st}")

        self.publish_for(cmd_topic, Twist, twist(), 0.3)

        n_fail = sum(1 for r in results.values() if r["status"] == "FAIL")
        n_pass = len(results) - n_fail
        print(f"\n요약: 동작 PASS {n_pass} / FAIL {n_fail} (총 {len(results)}), WARN {warns}")
        print("최종 결과:", "PASS" if n_fail == 0 else "FAIL")
        return 0 if n_fail == 0 else 1


def main():
    args = parse_args()
    rclpy.init()
    node = SrbArmTesterV2(args)
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        try:
            topic = args.cmd_topic or f"/srb/env{args.env}/action/cmd_vel"
            node.publish_for(topic, Twist, twist(), 0.2)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()