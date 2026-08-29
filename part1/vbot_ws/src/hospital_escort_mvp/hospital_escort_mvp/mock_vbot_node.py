import struct
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from function_msgs.action import GoalNav
from function_msgs.msg import NavigationStatus
from function_msgs.srv import ControlFollowing, NavigateToTarget, SemanticNavQuery, SetSpeak
from sensor_msgs.msg import Image, Imu, PointCloud2
from sensor_msgs.msg import PointField
from slam_msgs.msg import SlamStatus
from slam_msgs.srv import GetPathToTarget, LocCmd, QueryMap, SaveMap, SetSlamMode
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class MockVbot(Node):
    """在没有真机时提供行为确定、兼容 VITA 接口的模拟节点。"""

    def __init__(self) -> None:
        super().__init__('mock_vbot')
        self.declare_parameter('navigation_step_sec', 0.05)
        self.declare_parameter('fail_navigation', False)

        self._nav_status = NavigationStatus()
        self._nav_status.status = NavigationStatus.STATUS_IDLE
        self._nav_status.mode = NavigationStatus.MODE_GOAL_NAVIGATION
        self._nav_status.modify_source = 'mock_vbot'

        self._slam_status = SlamStatus()
        self._slam_status.operation_mode = SlamStatus.MODE_LOCALIZATION
        self._slam_status.loc_status = SlamStatus.LOC_STATUS_TRACKING
        self._slam_status.odom_status = SlamStatus.ODOM_STATUS_TRACKING
        self._slam_status.map_status = SlamStatus.MAP_STATUS_READY
        self._slam_status.tracking_quality = 0.99
        self._slam_status.current_map_name = 'hospital_F1'

        self.create_service(ControlFollowing, '/control_following', self._following)
        self.create_service(NavigateToTarget, '/navigate_to_target', self._navigate_target)
        self.create_service(SemanticNavQuery, '/semantic_nav_query', self._semantic_query)
        self.create_service(SetSpeak, '/set_speak', self._speak)
        self.create_service(SetSlamMode, '/slam/set_mode', self._set_slam_mode)
        self.create_service(SaveMap, '/lightning/save_map', self._save_map)
        self.create_service(LocCmd, '/lightning/loc_cmd', self._loc_cmd)
        self.create_service(QueryMap, '/slam/query_map', self._query_map)
        self.create_service(GetPathToTarget, '/slam/get_path_to_target', self._get_path)

        self._action_server = ActionServer(
            self,
            GoalNav,
            '/goal_nav',
            execute_callback=self._execute_nav,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )

        self._nav_pub = self.create_publisher(NavigationStatus, '/navigation/status', 10)
        self._slam_pub = self.create_publisher(SlamStatus, '/slam/status', 10)
        self._cloud_pub = self.create_publisher(PointCloud2, '/mock/lidar/points', 10)
        self._image_pub = self.create_publisher(Image, '/mock/camera/image', 10)
        self._imu_pub = self.create_publisher(Imu, '/mock/imu', 10)
        self._tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.create_timer(0.25, self._publish_status)
        self.get_logger().info('Mock VITA interfaces are ready')

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()

    def _accept_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_nav(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            f'Mock navigating to x={request.x:.2f}, y={request.y:.2f}, yaw={request.yaw:.2f}'
        )
        self._nav_status.status = NavigationStatus.STATUS_RUNNING
        step_sec = float(self.get_parameter('navigation_step_sec').value)
        for progress in (25, 50, 75, 100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._nav_status.status = NavigationStatus.STATUS_IDLE
                result = GoalNav.Result()
                result.success = False
                result.message = 'Mock navigation canceled'
                result.error_code = 2
                return result
            feedback = GoalNav.Feedback()
            feedback.progress = [progress]
            feedback.message = f'{progress}%'
            goal_handle.publish_feedback(feedback)
            time.sleep(step_sec)

        result = GoalNav.Result()
        if bool(self.get_parameter('fail_navigation').value):
            goal_handle.abort()
            result.success = False
            result.message = 'Injected mock navigation failure'
            result.error_code = 1001
        else:
            goal_handle.succeed()
            result.success = True
            result.message = 'Mock destination reached'
            result.error_code = 0
        self._nav_status.status = NavigationStatus.STATUS_IDLE
        return result

    def _following(self, request, response):
        response.success = request.mode in range(0, 9)
        response.error_code = 0 if response.success else 400
        response.message = f'Mock following mode={request.mode}, enabled={request.target_state}'
        return response

    def _navigate_target(self, request, response):
        response.success = bool(request.target_name)
        response.message = f'Mock semantic target: {request.target_name}'
        response.selected_x = 1.0
        response.selected_y = 1.0
        response.selected_distance = 1.4
        return response

    def _semantic_query(self, request, response):
        response.success = bool(request.prompt)
        response.message = 'Mock semantic query completed'
        response.confidence = 0.9
        response.estimated_distance_m = 2.0
        response.frame_id = request.camera_frame_id or 'stereo_left'
        return response

    def _speak(self, request, response):
        response.success = request.target_state in (0, 1)
        response.error_code = 0 if response.success else 400
        response.message = f'Mock speech: {request.human_language_text}'
        self.get_logger().info(response.message)
        return response

    def _set_slam_mode(self, request, response):
        self._slam_status.operation_mode = request.mode
        self._slam_status.current_map_name = request.map_name or self._slam_status.current_map_name
        response.accepted = True
        response.message = 'Mock SLAM mode accepted'
        response.server_task_id = 1
        return response

    def _save_map(self, _request, response):
        response.response = 0
        return response

    def _loc_cmd(self, _request, response):
        response.result = 0
        return response

    def _query_map(self, _request, response):
        response.success = True
        response.error_code = QueryMap.Response.ERROR_CODE_OK
        response.message = 'Mock empty local point cloud'
        response.point_count_raw = 0
        response.point_count_after_filter = 0
        return response

    def _get_path(self, request, response):
        response.success = bool(request.target_name)
        response.message = f'Mock path for {request.target_name}'
        return response

    def _publish_status(self) -> None:
        now = self.get_clock().now().to_msg()
        self._slam_pub.publish(self._slam_status)
        self._nav_pub.publish(self._nav_status)

        # 先发布同一时间戳的坐标变换，再发布传感器数据，避免下游外推 TF。
        transforms = []
        for child, z in (('mock_lidar', 0.5), ('base_link', 0.0)):
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = 'map'
            transform.child_frame_id = child
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._tf_pub.publish(TFMessage(transforms=transforms))

        # 生成带门洞墙体的地面点云，用于建图和路径规划演示。
        points = []
        for x_index in range(1, 17):
            for y_index in range(-8, 9):
                points.append((x_index * 0.25, y_index * 0.25, -0.5))
        for y_index in list(range(-8, -2)) + list(range(3, 9)):
            for z_index in range(1, 7):
                points.append((2.0, y_index * 0.25, z_index * 0.25 - 0.5))
        cloud = PointCloud2()
        cloud.header.stamp = now
        cloud.header.frame_id = 'mock_lidar'
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(('x', 'y', 'z'))
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        self._cloud_pub.publish(cloud)
        image = Image()
        image.header.stamp = now
        image.header.frame_id = 'mock_camera'
        self._image_pub.publish(image)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'mock_imu'
        self._imu_pub.publish(imu)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockVbot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
