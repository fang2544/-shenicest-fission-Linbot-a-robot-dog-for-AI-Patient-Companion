import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_srvs.srv import SetBool


class DynamicObstacle(Node):
    """控制人形动态障碍物横穿主走廊。"""

    def __init__(self) -> None:
        super().__init__('dynamic_obstacle')
        self.declare_parameter('model_name', 'dynamic_obstacle')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('amplitude', 1.1)
        self.declare_parameter('period_sec', 8.0)
        self.declare_parameter('start_delay_sec', 65.0)
        self.declare_parameter('active_duration_sec', 25.0)
        self._enabled = True
        self._future = None
        self._pending_pose = None
        self._start_wall = time.monotonic()
        self._cleared = False
        self._client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self._pose_pub = self.create_publisher(
            PoseStamped, '/simulation/dynamic_obstacle_pose', 10
        )
        self.create_service(SetBool, '~/enable', self._enable)
        self.create_timer(0.25, self._update)

    def _enable(self, request, response):
        self._enabled = bool(request.data)
        response.success = True
        response.message = 'dynamic obstacle enabled' if self._enabled else 'dynamic obstacle disabled'
        return response

    def _update(self) -> None:
        if not self._enabled or not self._client.service_is_ready():
            return
        if self._future is not None:
            if not self._future.done():
                return
            try:
                response = self._future.result()
            except Exception as exc:  # 服务传输失败会写入日志。
                self.get_logger().error(f'failed to move dynamic obstacle: {exc}')
                response = None
            if response is not None and response.success and self._pending_pose is not None:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                pose.pose.position.x, pose.pose.position.y = self._pending_pose
                pose.pose.position.z = 0.65
                pose.pose.orientation.w = 1.0
                self._pose_pub.publish(pose)
            self._future = None
        seconds = self.get_clock().now().nanoseconds / 1e9
        period = max(1.0, float(self.get_parameter('period_sec').value))
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = str(self.get_parameter('model_name').value)
        request.state.reference_frame = 'world'
        request.state.pose.position.x = float(self.get_parameter('x').value)
        elapsed = time.monotonic() - self._start_wall
        start_delay = float(self.get_parameter('start_delay_sec').value)
        active_duration = float(self.get_parameter('active_duration_sec').value)
        if elapsed < start_delay:
            # 先让 Nav2 生成初始全局路径，再让行人进入走廊测试局部停车和避障。
            request.state.pose.position.y = 3.0
        elif elapsed >= start_delay + active_duration:
            # 模拟行人完成横穿后离开通道。
            request.state.pose.position.y = 3.0
            self._cleared = True
        else:
            request.state.pose.position.y = float(self.get_parameter('amplitude').value) * math.sin(2.0 * math.pi * seconds / period)
        request.state.pose.position.z = 0.65
        request.state.pose.orientation.w = 1.0
        self._pending_pose = (
            request.state.pose.position.x, request.state.pose.position.y
        )
        self._future = self._client.call_async(request)
        if self._cleared:
            self._enabled = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
