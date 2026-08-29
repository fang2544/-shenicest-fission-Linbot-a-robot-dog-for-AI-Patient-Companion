import json
import math
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2

from function_msgs.action import GoalNav


class SimulationAcceptance(Node):
    """验收传感器、定位、规划、避障和 VBot action 调用链。"""

    def __init__(self) -> None:
        super().__init__('simulation_acceptance')
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('start_x', -4.5)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('report_path', '/vbot_ws/reports/navigation_acceptance.json')
        self.declare_parameter('timeout_sec', 180.0)
        self._start_wall = time.monotonic()
        self._sensors = {'scan': False, 'imu': False, 'image': False, 'depth_points': False}
        self._cmd_vel_seen = False
        self._vel_cmd_seen = False
        self._plan_seen = False
        self._initial_pose_sent = 0
        self._last_initial_pose_wall = 0.0
        self._goal_future = None
        self._result_future = None
        self._goal_attempts = 0
        self._retry_not_before = 0.0
        self._odom_start = None
        self._odom_last = None
        self._distance = 0.0
        self._front_range = math.inf
        self._nearest_range = math.inf
        self._dynamic_distance = math.inf
        self._minimum_dynamic_distance = math.inf
        self._obstacle_observed = False
        self._avoidance_response = False
        self._max_angular = 0.0
        self._done = False

        self._initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._goal = ActionClient(self, GoalNav, '/goal_nav')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Imu, '/imu/data', lambda _m: self._mark('imu'), 10)
        self.create_subscription(
            Image, '/camera/depth/depth/image_raw', lambda _m: self._mark('image'), 10
        )
        self.create_subscription(PointCloud2, '/camera/depth/points', lambda _m: self._mark('depth_points'), 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._models_cb, 10)
        self.create_subscription(
            PoseStamped,
            '/simulation/dynamic_obstacle_pose',
            self._dynamic_pose_cb,
            10,
        )
        self.create_subscription(NavPath, '/plan', lambda _m: setattr(self, '_plan_seen', True), 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Twist, '/vel_cmd', lambda _m: setattr(self, '_vel_cmd_seen', True), 10)
        self.create_timer(0.25, self._step)

    def _mark(self, key: str) -> None:
        self._sensors[key] = True

    def _scan_cb(self, message: LaserScan) -> None:
        self._sensors['scan'] = True
        if not message.ranges:
            return
        all_values = [r for r in message.ranges if math.isfinite(r)]
        self._nearest_range = min(all_values) if all_values else math.inf
        center = len(message.ranges) // 2
        half_width = max(1, int(math.radians(15.0) / max(message.angle_increment, 1e-6)))
        values = [r for r in message.ranges[max(0, center-half_width):center+half_width+1] if math.isfinite(r)]
        self._front_range = min(values) if values else math.inf
        # 走廊墙面约距 1.45 米；直线路径上小于 1 米的回波视为横穿行人。
        if self._nearest_range < 1.0:
            self._obstacle_observed = True

    def _cmd_cb(self, message: Twist) -> None:
        self._cmd_vel_seen = True
        self._max_angular = max(self._max_angular, abs(message.angular.z))
        obstacle_close = self._nearest_range < 1.0 or self._dynamic_distance < 1.5
        if obstacle_close and (
            abs(message.angular.z) > 0.04 or abs(message.linear.x) < 0.10
        ):
            self._avoidance_response = True

    def _models_cb(self, message: ModelStates) -> None:
        try:
            robot = message.pose[message.name.index('vbot')].position
            obstacle = message.pose[message.name.index('dynamic_obstacle')].position
        except (ValueError, IndexError):
            return
        self._dynamic_distance = math.hypot(robot.x - obstacle.x, robot.y - obstacle.y)
        self._minimum_dynamic_distance = min(
            self._minimum_dynamic_distance, self._dynamic_distance
        )
        if self._dynamic_distance < 1.5:
            self._obstacle_observed = True

    def _dynamic_pose_cb(self, message: PoseStamped) -> None:
        if self._odom_last is None:
            return
        obstacle = message.pose.position
        self._dynamic_distance = math.hypot(
            self._odom_last[0] - obstacle.x, self._odom_last[1] - obstacle.y
        )
        self._minimum_dynamic_distance = min(
            self._minimum_dynamic_distance, self._dynamic_distance
        )
        if self._dynamic_distance < 1.5:
            self._obstacle_observed = True

    def _odom_cb(self, message: Odometry) -> None:
        xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self._odom_start is None:
            self._odom_start = xy
        if self._odom_last is not None:
            self._distance += math.hypot(xy[0] - self._odom_last[0], xy[1] - self._odom_last[1])
        self._odom_last = xy

    def _send_initial_pose(self) -> None:
        message = PoseWithCovarianceStamped()
        # 零时间戳要求 AMCL/TF 使用最新变换，避免 Gazebo 启动时产生外推错误。
        message.header.frame_id = 'map'
        message.pose.pose.position.x = float(self.get_parameter('start_x').value)
        message.pose.pose.position.y = float(self.get_parameter('start_y').value)
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.02
        self._initial_pose.publish(message)
        self._initial_pose_sent += 1
        self._last_initial_pose_wall = time.monotonic()

    def _write(self, nav_success: bool, message: str) -> None:
        final_error = None
        if self._odom_last is not None:
            final_error = math.hypot(
                self._odom_last[0] - float(self.get_parameter('goal_x').value),
                self._odom_last[1] - float(self.get_parameter('goal_y').value),
            )
        checks = {
            'laser_scan': self._sensors['scan'],
            'imu': self._sensors['imu'],
            'depth_image': self._sensors['image'],
            'depth_point_cloud': self._sensors['depth_points'],
            'nav2_plan': self._plan_seen,
            'cmd_vel_generated': self._cmd_vel_seen,
            'vbot_vel_cmd_relayed': self._vel_cmd_seen,
            'dynamic_obstacle_observed': self._obstacle_observed,
            'avoidance_or_safe_stop_observed': self._avoidance_response,
            'goal_reached': nav_success,
            'final_position_error_under_0_35m': final_error is not None and final_error < 0.35,
        }
        success = all(checks.values())
        report = {
            'success': success,
            'checks': checks,
            'metrics': {
                'travel_distance_m': round(self._distance, 3),
                'final_position_error_m': None if final_error is None else round(final_error, 3),
                'max_commanded_angular_speed_rad_s': round(self._max_angular, 3),
                'duration_sec': round(time.monotonic() - self._start_wall, 3),
                'goal_attempts': self._goal_attempts,
                'minimum_dynamic_obstacle_distance_m': (
                    None if not math.isfinite(self._minimum_dynamic_distance)
                    else round(self._minimum_dynamic_distance, 3)
                ),
            },
            'message': message,
            'limitations': [
                'Planar kinematic model does not validate quadruped gait or joint torque control.',
                'Synthetic sensors do not validate VBot calibration, firmware, or Zenoh transport.',
            ],
        }
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

    def _step(self) -> None:
        if self._done:
            return
        elapsed = time.monotonic() - self._start_wall
        if elapsed > float(self.get_parameter('timeout_sec').value):
            self._write(False, 'acceptance timeout')
            self._done = True
            return
        if self._odom_start is None or not self._sensors['scan']:
            return
        if elapsed < 40.0:
            if elapsed > 15.0 and time.monotonic() - self._last_initial_pose_wall > 3.0:
                self._send_initial_pose()
            return
        if elapsed < 45.0:
            return
        if self._goal_future is None:
            if time.monotonic() < self._retry_not_before:
                return
            if not self._goal.server_is_ready():
                return
            goal = GoalNav.Goal()
            goal.control = True
            goal.x = float(self.get_parameter('goal_x').value)
            goal.y = float(self.get_parameter('goal_y').value)
            goal.z = 0.0
            goal.yaw = float(self.get_parameter('goal_yaw').value)
            self._goal_attempts += 1
            self._goal_future = self._goal.send_goal_async(goal)
            return
        if self._result_future is None:
            if not self._goal_future.done():
                return
            handle = self._goal_future.result()
            if handle is None or not handle.accepted:
                self._write(False, 'GoalNav rejected')
                self._done = True
                return
            self._result_future = handle.get_result_async()
            return
        if not self._result_future.done():
            return
        wrapped = self._result_future.result()
        result = wrapped.result if wrapped is not None else None
        nav_success = bool(result is not None and result.success)
        # 兼容桥可能早于 Nav2 生命周期节点就绪；只重试短暂的未就绪错误，
        # 规划器或控制器错误仍直接判定验收失败。
        error_code = int(getattr(result, 'error_code', -1)) if result is not None else -1
        if not nav_success and error_code in (1001, 1002) and self._goal_attempts < 4:
            self.get_logger().warning(
                f'transient navigation readiness failure ({error_code}); retrying GoalNav'
            )
            self._goal_future = None
            self._result_future = None
            self._retry_not_before = time.monotonic() + 5.0
            return
        self._write(nav_success, result.message if result is not None else 'missing action result')
        self._done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationAcceptance()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
