"""执行真实楼层建图、地图保存和基于 TF 的点位采集。"""

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from slam_msgs.msg import SlamStatus
from slam_msgs.srv import SaveMap, SetSlamMode
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from hospital_escort_mvp.hospital_graph import HospitalGraph
from hospital_escort_mvp.survey_store import SurveyStore


def _yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class FloorSurveyor(Node):
    def __init__(self) -> None:
        super().__init__('hospital_floor_surveyor')
        defaults = {
            'floor': 'F1',
            'map_name': 'surveyed_F1',
            'global_frame': 'map',
            'base_frame': 'base_link',
            'poi_id': 'poi_1',
            'poi_name': '现场点位',
            'poi_type': 'poi',
            'elevator_id': '',
            'edge_id': 'edge_1',
            'edge_from': '',
            'edge_to': '',
            'edge_type': 'walk',
            'edge_travel_time_sec': 10.0,
            'edge_distance_m': -1.0,
            'edge_elevator_id': '',
            'edge_bidirectional': True,
            'survey_file': '/vbot_ws/maps/surveyed_hospital_graph.yaml',
            'reference_walk_speed_mps': 0.4,
            'slam_mode_service': '/slam/set_mode',
            'save_map_service': '/lightning/save_map',
            'slam_status_topic': '/slam/status',
            'operation_timeout_sec': 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._group = ReentrantCallbackGroup()
        self._slam_mode = self.create_client(
            SetSlamMode,
            str(self.get_parameter('slam_mode_service').value),
            callback_group=self._group,
        )
        self._save_map = self.create_client(
            SaveMap,
            str(self.get_parameter('save_map_service').value),
            callback_group=self._group,
        )
        self._slam_status = None
        self.create_subscription(
            SlamStatus,
            str(self.get_parameter('slam_status_topic').value),
            self._slam_status_cb,
            10,
            callback_group=self._group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self.create_service(
            Trigger, '~/start_mapping', self._start_mapping, callback_group=self._group
        )
        self.create_service(
            Trigger,
            '~/finish_and_save_map',
            self._finish_and_save,
            callback_group=self._group,
        )
        self.create_service(
            Trigger, '~/load_map', self._load_map, callback_group=self._group
        )
        self.create_service(
            Trigger, '~/record_poi', self._record_poi, callback_group=self._group
        )
        self.create_service(
            Trigger, '~/connect_nodes', self._connect_nodes, callback_group=self._group
        )
        self.create_service(
            Trigger, '~/validate_graph', self._validate_graph, callback_group=self._group
        )

    @property
    def store(self) -> SurveyStore:
        return SurveyStore(
            str(self.get_parameter('survey_file').value),
            float(self.get_parameter('reference_walk_speed_mps').value),
        )

    def _slam_status_cb(self, message: SlamStatus) -> None:
        self._slam_status = message

    def _state(self, state: str) -> None:
        self._state_pub.publish(String(data=state))
        self.get_logger().info(state)

    def _request_mode(self, mode: int, map_name: str, request_reloc: bool):
        if not self._slam_mode.wait_for_service(timeout_sec=5.0):
            return None, 'SetSlamMode unavailable'
        request = SetSlamMode.Request()
        request.mode = mode
        request.request_reloc = request_reloc
        request.map_name = map_name
        request.client_id = 'hospital_floor_surveyor'
        request.client_req_id = int(time.time_ns() & 0xFFFFFFFFFFFFFFFF)
        future = self._slam_mode.call_async(request)
        timeout = float(self.get_parameter('operation_timeout_sec').value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None, 'SetSlamMode response timeout'
        response = future.result()
        if not response.accepted:
            return None, response.message
        return response, response.message

    def _wait_map_ready(self, map_name: str) -> bool:
        timeout = float(self.get_parameter('operation_timeout_sec').value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            status = self._slam_status
            if (
                status is not None
                and status.current_map_name == map_name
                and status.map_status == SlamStatus.MAP_STATUS_READY
            ):
                return True
            time.sleep(0.05)
        return False

    def _start_mapping(self, _request, response):
        result, message = self._request_mode(
            SlamStatus.MODE_MAPPING, map_name='', request_reloc=False
        )
        response.success = result is not None
        response.message = message if result is None else 'mapping started'
        if response.success:
            self._state('MAPPING_STARTED')
        return response

    def _finish_and_save(self, _request, response):
        floor = str(self.get_parameter('floor').value)
        map_name = str(self.get_parameter('map_name').value)
        if not self._save_map.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = 'SaveMap unavailable'
            return response
        save_request = SaveMap.Request()
        save_request.map_id = map_name
        save_future = self._save_map.call_async(save_request)
        timeout = float(self.get_parameter('operation_timeout_sec').value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not save_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not save_future.done() or save_future.result() is None:
            response.success = False
            response.message = 'SaveMap response timeout'
            return response
        if save_future.result().response != 0:
            response.success = False
            response.message = f'SaveMap failed code={save_future.result().response}'
            return response
        result, message = self._request_mode(
            SlamStatus.MODE_LOCALIZATION,
            map_name=map_name,
            request_reloc=False,
        )
        if result is None:
            response.success, response.message = False, message
            return response
        if not self._wait_map_ready(map_name):
            response.success = False
            response.message = f'save accepted but {map_name} did not become ready'
            return response
        self.store.add_floor(floor, map_name)
        self._state(f'MAP_SAVED floor={floor} map_name={map_name}')
        response.success = True
        response.message = f'saved {map_name} and registered floor {floor}'
        return response

    def _load_map(self, _request, response):
        map_name = str(self.get_parameter('map_name').value)
        result, message = self._request_mode(
            SlamStatus.MODE_LOCALIZATION, map_name=map_name, request_reloc=True
        )
        if result is None:
            response.success, response.message = False, message
            return response
        response.success = self._wait_map_ready(map_name)
        response.message = (
            f'loaded {map_name}' if response.success else f'{map_name} load timeout'
        )
        if response.success:
            self._state(f'MAP_LOADED map_name={map_name}')
        return response

    def _record_poi(self, _request, response):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('global_frame').value),
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time(),
            ).transform
        except TransformException as exc:
            response.success, response.message = False, str(exc)
            return response
        floor = str(self.get_parameter('floor').value)
        map_name = str(self.get_parameter('map_name').value)
        node_id = str(self.get_parameter('poi_id').value)
        self.store.add_node(
            floor=floor,
            map_name=map_name,
            node_id=node_id,
            name=str(self.get_parameter('poi_name').value),
            node_type=str(self.get_parameter('poi_type').value),
            x=transform.translation.x,
            y=transform.translation.y,
            yaw=_yaw_from_quaternion(transform.rotation),
            elevator_id=str(self.get_parameter('elevator_id').value) or None,
        )
        self._state(f'POI_RECORDED node={node_id} floor={floor}')
        response.success = True
        response.message = f'recorded {node_id} from TF'
        return response

    def _connect_nodes(self, _request, response):
        try:
            self.store.add_edge(
                edge_id=str(self.get_parameter('edge_id').value),
                from_node=str(self.get_parameter('edge_from').value),
                to_node=str(self.get_parameter('edge_to').value),
                edge_type=str(self.get_parameter('edge_type').value),
                travel_time_sec=float(
                    self.get_parameter('edge_travel_time_sec').value
                ),
                bidirectional=bool(self.get_parameter('edge_bidirectional').value),
                distance_m=float(self.get_parameter('edge_distance_m').value),
                elevator_id=(
                    str(self.get_parameter('edge_elevator_id').value) or None
                ),
            )
            self._state(f'EDGE_RECORDED edge={self.get_parameter("edge_id").value}')
            response.success, response.message = True, 'edge recorded'
        except (KeyError, TypeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _validate_graph(self, _request, response):
        try:
            graph = HospitalGraph.from_yaml(
                str(self.get_parameter('survey_file').value)
            )
            response.success = True
            response.message = (
                f'valid graph: {len(graph.nodes)} nodes, {len(graph.edges)} edges'
            )
        except (KeyError, OSError, TypeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloorSurveyor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
