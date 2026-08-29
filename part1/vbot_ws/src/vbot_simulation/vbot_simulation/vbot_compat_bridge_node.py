import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import SaveMap as Nav2SaveMap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from function_msgs.msg import NavigationStatus
from function_msgs.srv import ControlFollowing, SetSpeak
from slam_msgs.msg import SlamStatus
from slam_msgs.srv import LocCmd, SaveMap, SetSlamMode


class VBotCompatibilityBridge(Node):
    """在标准 Nav2 组件上提供 VBot 风格的速度和 SLAM 接口。"""

    def __init__(self) -> None:
        super().__init__('vbot_compat_bridge')
        self.declare_parameter('initial_mode', int(SlamStatus.MODE_ODOMETRY))
        self.declare_parameter('map_directory', '/vbot_ws/maps')
        self.declare_parameter('map_name', 'hospital_sim')
        self._group = ReentrantCallbackGroup()
        self._mode = int(self.get_parameter('initial_mode').value)
        self._map_name = str(self.get_parameter('map_name').value)
        self._have_odom = False
        self._have_map = False
        self._distance = 0.0
        self._last_xy = None
        self._last_cmd = Twist()
        self._last_cmd_at = 0.0

        self._vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)
        self._nav_status_pub = self.create_publisher(
            NavigationStatus, '/navigation/status', 10
        )
        self._slam_status_pub = self.create_publisher(SlamStatus, '/slam/status', 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        self._map_saver = self.create_client(
            Nav2SaveMap, '/map_saver/save_map', callback_group=self._group
        )
        self.create_service(
            SetSlamMode,
            '/slam/set_mode',
            self._set_mode,
            callback_group=self._group,
        )
        self.create_service(
            SaveMap,
            '/lightning/save_map',
            self._save_map,
            callback_group=self._group,
        )
        self.create_service(
            LocCmd,
            '/lightning/loc_cmd',
            self._loc_cmd,
            callback_group=self._group,
        )
        self.create_service(SetSpeak, '/set_speak', self._set_speak)
        self.create_service(ControlFollowing, '/control_following', self._control_following)
        self.create_timer(0.2, self._publish_status)

    def _cmd_cb(self, message: Twist) -> None:
        # 真机接入时由 VBot Zenoh 链路替换此转发层；Gazebo 固定订阅 /vel_cmd。
        self._last_cmd = message
        self._last_cmd_at = time.monotonic()
        self._vel_pub.publish(message)

    def _odom_cb(self, message: Odometry) -> None:
        self._have_odom = True
        xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self._last_xy is not None:
            self._distance += math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1])
        self._last_xy = xy

    def _map_cb(self, _message: OccupancyGrid) -> None:
        self._have_map = True

    def _set_mode(self, request, response):
        if request.mode not in (
            SlamStatus.MODE_ODOMETRY,
            SlamStatus.MODE_MAPPING,
            SlamStatus.MODE_LOCALIZATION,
        ):
            response.accepted = False
            response.message = f'unsupported SLAM mode: {request.mode}'
            response.server_task_id = 0
            return response
        self._mode = int(request.mode)
        if request.map_name:
            self._map_name = request.map_name
        response.accepted = True
        response.message = 'simulation mode accepted; launch topology owns SLAM lifecycle'
        response.server_task_id = int(time.time_ns() & 0xFFFFFFFFFFFFFFFF)
        return response

    def _save_map(self, request, response):
        if not self._map_saver.wait_for_service(timeout_sec=5.0):
            response.response = 3
            self.get_logger().error('Nav2 map_saver service is unavailable')
            return response
        map_id = request.map_id or self._map_name or 'hospital_sim'
        map_dir = Path(str(self.get_parameter('map_directory').value))
        map_dir.mkdir(parents=True, exist_ok=True)
        nav_request = Nav2SaveMap.Request()
        nav_request.map_topic = '/map'
        nav_request.map_url = str(map_dir / map_id)
        nav_request.image_format = 'pgm'
        nav_request.map_mode = 'trinary'
        nav_request.free_thresh = 0.25
        nav_request.occupied_thresh = 0.65
        future = self._map_saver.call_async(nav_request)
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None or not future.result().result:
            response.response = 3
            self.get_logger().error(f'failed to save map {map_id}')
            return response
        self._map_name = map_id
        self._have_map = True
        response.response = 0
        self.get_logger().info(f'saved map to {map_dir / map_id}.yaml')
        return response

    def _loc_cmd(self, request, response):
        if request.id == LocCmd.Request.CMD_LOC_INIT_POSE:
            pose = PoseWithCovarianceStamped()
            pose.header = request.header
            pose.header.frame_id = request.header.frame_id or 'map'
            pose.pose.pose.position.x = request.x
            pose.pose.pose.position.y = request.y
            pose.pose.pose.orientation.z = math.sin(request.z / 2.0)
            pose.pose.pose.orientation.w = math.cos(request.z / 2.0)
            pose.pose.covariance[0] = 0.04
            pose.pose.covariance[7] = 0.04
            pose.pose.covariance[35] = 0.02
            self._initial_pose_pub.publish(pose)
            response.result = 0
        else:
            response.result = 1
        return response

    def _set_speak(self, request, response):
        response.success = True
        response.error_code = 0
        response.message = request.human_language_text or request.machine_language_name or 'speech stopped'
        self.get_logger().info(f'Simulated speech: {response.message}')
        return response

    def _control_following(self, request, response):
        if request.mode == ControlFollowing.Request.STOP or request.target_state == 0:
            self._vel_pub.publish(Twist())
        response.success = True
        response.message = 'simulation follow/control request accepted'
        response.error_code = 0
        return response

    def _publish_status(self) -> None:
        now = self.get_clock().now().to_msg()
        nav = NavigationStatus()
        nav.modify_source = 'vbot_compat_bridge'
        moving = time.monotonic() - self._last_cmd_at < 0.5 and (
            abs(self._last_cmd.linear.x) + abs(self._last_cmd.linear.y)
            + abs(self._last_cmd.angular.z) > 0.001
        )
        nav.status = NavigationStatus.STATUS_RUNNING if moving else NavigationStatus.STATUS_IDLE
        nav.mode = NavigationStatus.MODE_GOAL_NAVIGATION
        nav.current_linear_velocity = float(self._last_cmd.linear.x)
        nav.current_angular_velocity = float(self._last_cmd.angular.z)
        self._nav_status_pub.publish(nav)

        slam = SlamStatus()
        slam.header.stamp = now
        slam.header.frame_id = 'map'
        slam.operation_mode = self._mode
        slam.odom_status = (
            SlamStatus.ODOM_STATUS_TRACKING if self._have_odom else SlamStatus.ODOM_STATUS_INIT
        )
        if self._mode == SlamStatus.MODE_MAPPING:
            slam.map_status = SlamStatus.MAP_STATUS_MAPPING
            slam.loc_status = SlamStatus.LOC_STATUS_TRACKING if self._have_odom else SlamStatus.LOC_STATUS_INIT
        elif self._mode == SlamStatus.MODE_LOCALIZATION:
            slam.map_status = SlamStatus.MAP_STATUS_READY if self._have_map else SlamStatus.MAP_STATUS_IDLE
            slam.loc_status = SlamStatus.LOC_STATUS_TRACKING if self._have_odom else SlamStatus.LOC_STATUS_INIT
        else:
            slam.map_status = SlamStatus.MAP_STATUS_IDLE
            slam.loc_status = SlamStatus.LOC_STATUS_UNKNOWN
        slam.tracking_quality = 1.0 if self._have_odom else 0.0
        slam.current_map_name = self._map_name
        slam.mileage = float(self._distance)
        slam.context = 'Gazebo/Nav2 compatibility layer'
        self._slam_status_pub.publish(slam)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VBotCompatibilityBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
