"""使用点云建立地图并规划医院单层全局路径。"""

import math
import struct
from typing import Iterable, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from hospital_escort_mvp.grid_planner import PlanningError, plan_path
from hospital_escort_mvp.voxel_mapping import Costmap2D, VoxelOccupancyMap


def _quaternion_rotate(
    q: Quaternion,
    xyz: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    # 直接展开 q × v × q⁻¹，避免引入额外的非 ROS 数学库。
    x, y, z = xyz
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def _xyz_points(message: PointCloud2, limit: int) -> Iterable[Tuple[float, float, float]]:
    fields = {field.name: field for field in message.fields}
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('PointCloud2 must contain x/y/z fields')
    supported = {PointField.FLOAT32: 'f', PointField.FLOAT64: 'd'}
    formats = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in supported:
            raise ValueError(f'{name} must be FLOAT32 or FLOAT64')
        formats.append((field.offset, supported[field.datatype]))
    count = int(message.width) * int(message.height)
    stride = max(1, int(math.ceil(count / max(1, limit))))
    endian = '>' if message.is_bigendian else '<'
    data = memoryview(message.data)
    for index in range(0, count, stride):
        row, column = divmod(index, int(message.width))
        base = row * message.row_step + column * message.point_step
        yield tuple(
            struct.unpack_from(endian + fmt, data, base + offset)[0]
            for offset, fmt in formats
        )  # type: ignore


class MappingPlanner(Node):
    def __init__(self) -> None:
        super().__init__('hospital_mapping_planner')
        defaults = {
            'cloud_topic': '/points', 'goal_topic': '/goal_pose',
            'global_frame': 'map', 'base_frame': 'base_link',
            'voxel_resolution': 0.10, 'floor_z': 0.0,
            'min_obstacle_height': 0.15, 'max_obstacle_height': 1.80,
            'inflation_radius': 0.55, 'map_padding': 1.0,
            'sensor_max_range': 8.0, 'max_rays_per_cloud': 12000,
            'publish_period_sec': 1.0, 'allow_unknown_planning': False,
            'map_file': '/vbot_ws/maps/hospital_F1.voxels.json.gz',
            'load_map_on_start': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._global_frame = str(self.get_parameter('global_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        resolution = float(self.get_parameter('voxel_resolution').value)
        self._map = VoxelOccupancyMap(resolution)
        self._costmap = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_pub = self.create_publisher(
            OccupancyGrid, '/hospital_mapper/costmap', 1
        )
        self._voxel_pub = self.create_publisher(
            PointCloud2, '/hospital_mapper/occupied_voxels', 1
        )
        self._path_pub = self.create_publisher(Path, '/hospital_planner/path', 1)
        self._status_pub = self.create_publisher(String, '/hospital_planner/status', 10)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_cb,
            10,
        )
        self.create_service(Trigger, '~/save_map', self._save_map_cb)
        self.create_service(Trigger, '~/clear_map', self._clear_map_cb)
        period = float(self.get_parameter('publish_period_sec').value)
        self.create_timer(period, self._publish_map)
        if bool(self.get_parameter('load_map_on_start').value):
            self._map.load(str(self.get_parameter('map_file').value))
            self.get_logger().info('Loaded persisted 3D voxel map')

    def _status(self, text: str) -> None:
        message = String(data=text)
        self._status_pub.publish(message)
        self.get_logger().info(text)

    def _cloud_cb(self, message: PointCloud2) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.2),
            ).transform
            translation = transform.translation
            rotation = transform.rotation

            def transformed():
                limit = int(self.get_parameter('max_rays_per_cloud').value)
                for point in _xyz_points(message, limit):
                    px, py, pz = _quaternion_rotate(rotation, point)
                    yield px + translation.x, py + translation.y, pz + translation.z
            accepted = self._map.integrate(
                transformed(), (translation.x, translation.y, translation.z),
                max_range=float(self.get_parameter('sensor_max_range').value),
                max_rays=int(self.get_parameter('max_rays_per_cloud').value),
            )
            if accepted == 0:
                self.get_logger().warning(
                    'Point cloud contained no usable in-range points'
                )
        except (TransformException, ValueError, struct.error) as exc:
            self.get_logger().warning(f'Cloud skipped: {exc}')

    def _project(self) -> Costmap2D:
        return self._map.to_costmap(
            floor_z=float(self.get_parameter('floor_z').value),
            min_obstacle_height=float(self.get_parameter('min_obstacle_height').value),
            max_obstacle_height=float(self.get_parameter('max_obstacle_height').value),
            inflation_radius=float(self.get_parameter('inflation_radius').value),
            padding=float(self.get_parameter('map_padding').value),
        )

    def _publish_map(self) -> None:
        try:
            self._costmap = self._project()
        except ValueError:
            return
        now = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.stamp, message.header.frame_id = now, self._global_frame
        message.info.resolution = self._costmap.resolution
        message.info.width = self._costmap.width
        message.info.height = self._costmap.height
        message.info.origin.position.x = self._costmap.origin_x
        message.info.origin.position.y = self._costmap.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = list(self._costmap.data)
        self._map_pub.publish(message)
        self._publish_voxels(now)

    def _publish_voxels(self, stamp) -> None:
        points = [
            self._map.voxel_center(voxel)
            for voxel, _ in self._map.occupied_voxels()
        ]
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, self._global_frame
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [
            PointField(
                name=name,
                offset=i * 4,
                datatype=PointField.FLOAT32,
                count=1,
            )
            for i, name in enumerate(('x', 'y', 'z'))
        ]
        cloud.is_bigendian, cloud.point_step = False, 12
        cloud.row_step, cloud.is_dense = cloud.point_step * cloud.width, True
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        self._voxel_pub.publish(cloud)

    def _goal_cb(self, goal: PoseStamped) -> None:
        if goal.header.frame_id and goal.header.frame_id != self._global_frame:
            self._status(f'PLAN_FAILED goal frame must be {self._global_frame}')
            return
        try:
            base = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, rclpy.time.Time()
            ).transform.translation
            costmap = self._costmap or self._project()
            points = plan_path(
                costmap, (base.x, base.y), (goal.pose.position.x, goal.pose.position.y),
                allow_unknown=bool(self.get_parameter('allow_unknown_planning').value),
            )
        except (TransformException, PlanningError, ValueError) as exc:
            self._status(f'PLAN_FAILED {exc}')
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._global_frame
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            floor_z = float(self.get_parameter('floor_z').value)
            pose.pose.position = Point(x=x, y=y, z=floor_z)
            if index + 1 < len(points):
                yaw = math.atan2(points[index + 1][1] - y, points[index + 1][0] - x)
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose.pose.orientation = goal.pose.orientation
            path.poses.append(pose)
        self._path_pub.publish(path)
        self._status(f'PLAN_OK poses={len(path.poses)}')

    def _save_map_cb(self, _request, response):
        try:
            path = str(self.get_parameter('map_file').value)
            self._map.save(path)
            response.success, response.message = True, f'saved {path}'
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _clear_map_cb(self, _request, response):
        self._map.clear()
        self._costmap = None
        response.success, response.message = True, 'map cleared'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
