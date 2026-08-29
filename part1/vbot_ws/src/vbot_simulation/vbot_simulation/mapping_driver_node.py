import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from slam_msgs.srv import SaveMap


class MappingDriver(Node):
    """在建图冒烟测试中代替人工遥控，按固定路线驱动机器人。"""

    def __init__(self) -> None:
        super().__init__('mapping_driver')
        self.declare_parameter('map_id', 'hospital_sim')
        self.declare_parameter('report_path', '/vbot_ws/reports/mapping_acceptance.json')
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._save = self.create_client(SaveMap, '/lightning/save_map')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._have_scan = False
        self._start_wall = time.monotonic()
        self._started_sim = None
        self._save_future = None
        self._done = False
        self.create_timer(0.1, self._step)

    def _scan_cb(self, _message: LaserScan) -> None:
        self._have_scan = True

    def _publish(self, linear=0.0, angular=0.0) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._cmd.publish(message)

    def _write_report(self, success: bool, message: str) -> None:
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    'success': bool(success),
                    'laser_scan_received': self._have_scan,
                    'map_id': str(self.get_parameter('map_id').value),
                    'message': message,
                    'wall_duration_sec': round(time.monotonic() - self._start_wall, 3),
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

    def _step(self) -> None:
        if self._done:
            return
        now_sim = self.get_clock().now().nanoseconds / 1e9
        if not self._have_scan:
            if time.monotonic() - self._start_wall > 30.0:
                self._write_report(False, 'laser scan timeout')
                self._done = True
            return
        if self._started_sim is None:
            self._started_sim = now_sim
        elapsed = now_sim - self._started_sim

        # 绕开走廊中央的人体模型后扫描两侧，防止机器人卡住并造成东侧地图缺失。
        if elapsed < 7.5:
            self._publish(linear=0.42)
            return
        if elapsed < 10.8:
            self._publish(angular=0.48)
            return
        if elapsed < 13.3:
            self._publish(linear=0.42)
            return
        if elapsed < 16.6:
            self._publish(angular=-0.48)
            return
        if elapsed < 31.0:
            self._publish(linear=0.42)
            return
        if elapsed < 34.3:
            self._publish(angular=-0.48)
            return
        if elapsed < 36.8:
            self._publish(linear=0.42)
            return
        self._publish()

        if self._save_future is None:
            if not self._save.service_is_ready():
                if time.monotonic() - self._start_wall > 70.0:
                    self._write_report(False, 'VBot SaveMap service unavailable')
                    self._done = True
                return
            request = SaveMap.Request()
            request.map_id = str(self.get_parameter('map_id').value)
            self._save_future = self._save.call_async(request)
            return
        if not self._save_future.done():
            return
        response = self._save_future.result()
        success = response is not None and response.response == 0
        self._write_report(success, 'map saved' if success else f'save response={getattr(response, "response", -1)}')
        self._done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingDriver()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
