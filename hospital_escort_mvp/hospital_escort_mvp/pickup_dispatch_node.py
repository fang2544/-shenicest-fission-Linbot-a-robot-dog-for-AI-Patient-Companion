"""接收经过认证且位于白名单内的医院接客请求。"""

import json
from pathlib import Path
import threading
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from function_msgs.srv import FunctionInput
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from hospital_escort_mvp.hospital_graph import HospitalGraph
from hospital_escort_mvp.multi_floor_nav_node import MultiFloorNavigator


class PickupDispatcher(Node):
    """接收可信网页或二维码网关的点位编号并下发导航任务。"""

    def __init__(self) -> None:
        super().__init__('hospital_pickup_dispatcher')
        default_graph = str(
            Path(get_package_share_directory('hospital_escort_mvp'))
            / 'config'
            / 'showroom_graph.yaml'
        )
        defaults = {
            'graph_path': default_graph,
            'home_node': 'cardiology_3f',
            'require_patient_token': True,
            'navigator_auto_confirm_elevator': False,
            'navigator_navigation_timeout_sec': 120.0,
            'navigator_report_path': '/vbot_ws/reports/pickup_route.json',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._graph_path = str(self.get_parameter('graph_path').value)
        self._graph = HospitalGraph.from_yaml(self._graph_path)
        with Path(self._graph_path).open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        policy = config.get('pickup_policy', {})
        self._allowed_pickups = set(policy.get('allowed_pickup_nodes', []))
        self._allowed_destinations = set(
            policy.get('allowed_destination_nodes', [])
        )
        self._allowed_sources = set(policy.get('accepted_signal_sources', []))
        self._home_node = str(self.get_parameter('home_node').value)
        self._current_node = self._home_node
        self._busy = False
        self._active_navigator = None
        self._preempt_requested = False
        self._lock = threading.Lock()
        self.create_service(
            FunctionInput, '/hospital/request_pickup', self._request_pickup
        )
        self.create_service(
            FunctionInput, '/hospital/request_mission', self._request_pickup
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, '/hospital/pickup/status', latched_qos
        )
        self._publish_status('READY', current_node=self._current_node)

    def _publish_status(self, state: str, **details) -> None:
        payload = {'timestamp': time.time(), 'state': state, **details}
        message = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _reject(self, response, request_id: str, message: str):
        response.success = [False]
        response.request_id = request_id
        response.response = json.dumps(
            {'accepted': False, 'message': message}, ensure_ascii=False
        )
        return response

    def _request_pickup(self, request, response):
        try:
            payload = json.loads(request.dag)
        except json.JSONDecodeError as exc:
            return self._reject(response, request.request_id, f'invalid JSON: {exc}')
        task_type = str(payload.get('task_type', 'pickup'))
        target_node = str(
            payload.get('pickup_node', '')
            if task_type == 'pickup'
            else payload.get('target_node', '')
        )
        patient_token = str(payload.get('patient_token', ''))
        if request.source not in self._allowed_sources:
            return self._reject(response, request.request_id, 'signal source not allowed')
        allowed_nodes = (
            self._allowed_pickups
            if task_type == 'pickup'
            else self._allowed_destinations
        )
        if task_type not in {'pickup', 'escort', 'priority_stop'}:
            return self._reject(response, request.request_id, 'task type not allowed')
        if target_node not in allowed_nodes:
            return self._reject(response, request.request_id, 'target node not allowed')
        if target_node not in self._graph.nodes:
            return self._reject(response, request.request_id, 'target node not in graph')
        if (
            bool(self.get_parameter('require_patient_token').value)
            and not patient_token
        ):
            return self._reject(response, request.request_id, 'patient token required')
        navigator_to_stop = None
        with self._lock:
            if self._busy and task_type != 'priority_stop':
                return self._reject(response, request.request_id, 'robot is busy')
            if self._busy:
                self._preempt_requested = True
                navigator_to_stop = self._active_navigator
        if navigator_to_stop is not None:
            self._publish_status(
                'PREEMPTING_FOR_PRIORITY', target_node=target_node,
                request_id=request.request_id,
            )
            navigator_to_stop.stop_motion()
            deadline = time.monotonic() + 12.0
            while time.monotonic() < deadline:
                with self._lock:
                    if not self._busy:
                        break
                time.sleep(0.05)
            else:
                return self._reject(
                    response, request.request_id, 'active navigation preemption timeout')
        with self._lock:
            if self._busy:
                return self._reject(response, request.request_id, 'robot is busy')
            try:
                route = self._graph.plan(self._current_node, target_node)
            except (ValueError, RuntimeError) as exc:
                return self._reject(response, request.request_id, str(exc))
            self._busy = True
            mission_id = str(uuid.uuid4())
            start_node = self._current_node
        worker = threading.Thread(
            target=self._run_pickup,
            args=(
                mission_id,
                request.request_id,
                start_node,
                target_node,
                task_type,
            ),
            daemon=True,
        )
        worker.start()
        response.success = [True]
        response.request_id = request.request_id
        response.response = json.dumps(
            {
                'accepted': True,
                'mission_id': mission_id,
                'task_type': task_type,
                'target_node': target_node,
                'estimated_arrival_sec': route.total_time_sec,
            },
            ensure_ascii=False,
        )
        self._publish_status(
            'REQUEST_ACCEPTED',
            mission_id=mission_id,
            request_id=request.request_id,
            task_type=task_type,
            target_node=target_node,
            estimated_arrival_sec=route.total_time_sec,
        )
        return response

    def _run_pickup(
        self,
        mission_id: str,
        request_id: str,
        start_node: str,
        target_node: str,
        task_type: str,
    ) -> None:
        self._publish_status(
            'DISPATCHING', mission_id=mission_id,
            task_type=task_type, target_node=target_node
        )
        overrides = [
            Parameter('graph_path', value=self._graph_path),
            Parameter('voice_prompts_enabled', value=True),
            Parameter(
                'auto_confirm_elevator',
                value=bool(
                    self.get_parameter('navigator_auto_confirm_elevator').value
                ),
            ),
            Parameter(
                'report_path',
                value=str(self.get_parameter('navigator_report_path').value),
            ),
            Parameter(
                'navigation_timeout_sec',
                value=float(
                    self.get_parameter('navigator_navigation_timeout_sec').value
                ),
            ),
        ]
        navigator = MultiFloorNavigator(
            parameter_overrides=overrides,
            external_spin=True,
            use_global_arguments=False,
        )
        self._mission_executor.add_node(navigator)
        with self._lock:
            self._active_navigator = navigator
        success = False
        try:
            success = navigator.run_route(start_id=start_node, goal_id=target_node)
        finally:
            self._mission_executor.remove_node(navigator)
            navigator.destroy_node()
        with self._lock:
            was_preempted = self._preempt_requested
            if success:
                self._current_node = target_node
            self._busy = False
            self._active_navigator = None
            self._preempt_requested = False
        self._publish_status(
            (
                'TARGET_REACHED' if success
                else 'MISSION_PREEMPTED' if was_preempted
                else 'MISSION_FAILED'
            ),
            mission_id=mission_id,
            request_id=request_id,
            task_type=task_type,
            target_node=target_node,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupDispatcher()
    executor = MultiThreadedExecutor(num_threads=4)
    node._mission_executor = executor
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
