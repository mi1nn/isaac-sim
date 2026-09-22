"""Astrobee camera feed for CAMERA 3: ROS 2 sensor_msgs/Image -> latest JPEG.

The simulator (GPU PC) publishes the Astrobee observation camera on
`/astrobee/camera/image_raw` (rgb8). This subscriber runs on the monitoring PC in a
background thread and keeps only the newest frame as JPEG for the web UI. It carries the
image only: no Astrobee state, no analysis, and nothing is written to any database.

Environment:
    ASTROBEE_IMAGE_TOPIC   topic (default /astrobee/camera/image_raw)
    MEP_DASHBOARD_ROS      0 = do not start the subscriber (UI shows CAMERA 3 offline)
    ROS_DOMAIN_ID          must match the simulator PC
"""
import io
import os
import threading
import time

DEFAULT_TOPIC = '/astrobee/camera/image_raw'
# A frame older than this (wall clock) is not served: CAMERA 3 shows OFFLINE. Generous,
# because a loaded GPU PC runs the simulation well below real time (5 Hz sim ~ 1 Hz wall)
STALE_AFTER_S = 10.0
JPEG_QUALITY = 80


def _encode_jpeg(width: int, height: int, encoding: str, data: bytes) -> bytes:
    from PIL import Image

    mode = {'rgb8': 'RGB', 'bgr8': 'RGB', 'rgba8': 'RGBA', 'mono8': 'L'}.get(encoding)
    if mode is None:
        raise ValueError(f'unsupported image encoding {encoding}')
    image = Image.frombytes(mode, (width, height), data)
    if encoding == 'bgr8':
        image = Image.merge('RGB', image.split()[::-1])
    if image.mode == 'RGBA':
        image = image.convert('RGB')
    out = io.BytesIO()
    image.save(out, format='JPEG', quality=JPEG_QUALITY)
    return out.getvalue()


class AstrobeeFeed:
    def __init__(self, topic: str | None = None):
        self.topic = topic or os.environ.get('ASTROBEE_IMAGE_TOPIC', DEFAULT_TOPIC)
        self.error: str | None = None
        self._jpeg: bytes | None = None
        self._received = 0.0  # time.monotonic() of the newest frame
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._context = None

    def start(self) -> bool:
        if os.environ.get('MEP_DASHBOARD_ROS', '1') == '0':
            self.error = 'disabled (MEP_DASHBOARD_ROS=0)'
            return False
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image
        except ImportError as e:
            self.error = f'rclpy not available ({e}); source /opt/ros/<distro>/setup.bash'
            print(f'[CAMERA 3] {self.error}', flush=True)
            return False
        self._context = rclpy.Context()
        rclpy.init(context=self._context)
        node = Node('mep_dashboard_astrobee_feed', context=self._context, start_parameter_services=False)
        # Best effort, like the publisher (sensor data)
        node.create_subscription(Image, self.topic, self._on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        executor = SingleThreadedExecutor(context=self._context)
        executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except Exception as e:  # context shut down on exit
                if self._context is not None and self._context.ok():
                    self.error = f'ROS spin stopped: {e}'
            finally:
                node.destroy_node()

        self._thread = threading.Thread(target=spin, name='astrobee-feed', daemon=True)
        self._thread.start()
        print(f'[CAMERA 3] subscribed to {self.topic} (ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")})', flush=True)
        return True

    def _on_image(self, msg):
        try:
            jpeg = _encode_jpeg(int(msg.width), int(msg.height), msg.encoding, bytes(msg.data))
        except Exception as e:
            self.error = f'frame dropped: {e}'
            return
        with self._lock:
            self._jpeg, self._received = jpeg, time.monotonic()

    def latest(self) -> bytes | None:
        """Newest JPEG, or None if there is none or it is stale."""
        with self._lock:
            if self._jpeg is None or time.monotonic() - self._received > STALE_AFTER_S:
                return None
            return self._jpeg

    def stop(self):
        if self._context is not None:
            try:
                self._context.try_shutdown()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
