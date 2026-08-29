"""在多张独立 VITA SLAM 地图之间执行分层跨楼层路线。"""

import json
from pathlib import Path
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_msgs.msg import SlamStatus
from slam_msgs.srv import LocCmd, SetSlamMode
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hospital_escort_mvp.hospital_graph import HospitalGraph, HospitalNode
from hospital_escort_mvp.vita_client import VitaInterfaceClient


class MultiFloorNavigator(VitaInterfaceClient):
    """步行路段调用 GoalNav，电梯路段切换楼层地图。"""

    def __init__(
        self,
        parameter_overrides=None,
        external_spin: bool = False,
        use_global_arguments: bool = True,
    ) -> None:
        super().__init__(
            'multi_floor_navigator',
            parameter_overrides=parameter_overrides,
            external_spin=external_spin,
            use_global_arguments=use_global_arguments,
        )
        default_graph = str(
            Path(get_package_share_directory('hospital_escort_mvp'))
            / 'config'
            / 'hospital_graph.yaml'
        )
        parameters = {
            'graph_path': default_graph,
            'start_node': 'parking',
            'goal_node': 'cardiology',
            'slam_mode_service': '/slam/set_mode',
            'localization_service': '/lightning/loc_cmd',
            'slam_status_topic': '/slam/status',
            'map_switch_timeout_sec': 30.0,
            'elevator_confirmation_timeout_sec': 180.0,
            'auto_confirm_elevator': False,
            'initialize_start_pose': False,
            'min_tracking_quality': 0.5,
            'voice_prompts_enabled': True,
            'report_path': '/vbot_ws/reports/multi_floor_route.json',
            'graph_export_path': '/vbot_ws/reports/hospital_graph.json',
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.graph = HospitalGraph.from_yaml(
            str(self.get_parameter('graph_path').value)
        )
        self._slam_mode = self.create_client(
            SetSlamMode, str(self.get_parameter('slam_mode_service').value)
        )
        self._localization = self.create_client(
            LocCmd, str(self.get_parameter('localization_service').value)
        )
        self._slam_status = None
        self._elevator_confirmed = False
        self.create_subscription(
            SlamStatus,
            str(self.get_parameter('slam_status_topic').value),
            self._slam_status_cb,
            10,
        )
        self.create_service(
            Trigger, '~/confirm_elevator_arrival', self._confirm_elevator_cb
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._graph_pub = self.create_publisher(
            String, '/hospital_graph/model', latched_qos
        )
        self._route_pub = self.create_publisher(
            String, '/hospital_graph/route', latched_qos
        )
        self._state_pub = self.create_publisher(
            String, '/multi_floor_navigator/state', 10
        )
        self._events = []
        graph_json = json.dumps(self.graph.as_dict(), ensure_ascii=False)
        self._graph_pub.publish(String(data=graph_json))
        export_path = Path(str(self.get_parameter('graph_export_path').value))
        export_path.parent.mkdir(parents=True, exist_ok=True)
        export_path.write_text(
            json.dumps(self.graph.as_dict(), ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

    def _slam_status_cb(self, message: SlamStatus) -> None:
        self._slam_status = message

    def _confirm_elevator_cb(self, _request, response):
        self._elevator_confirmed = True
        response.success = True
        response.message = 'elevator arrival confirmed'
        return response

    def _state(self, state: str, **details) -> None:
        event = {'timestamp': time.time(), 'state': state, **details}
        self._events.append(event)
        payload = json.dumps(event, ensure_ascii=False)
        self._state_pub.publish(String(data=payload))
        self.get_logger().info(payload)

    def _announce(self, text: str) -> None:
        if not bool(self.get_parameter('voice_prompts_enabled').value):
            return
        success, message = self.speak(text)
        if success:
            self._state('SPEECH_ANNOUNCED', text=text)
        else:
            self._state('SPEECH_WARNING', message=message, text=text)

    def _wait_map_status(
        self,
        map_name: str,
        require_tracking: bool,
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout
        min_quality = float(self.get_parameter('min_tracking_quality').value)
        while rclpy.ok() and time.monotonic() < deadline:
            if self._external_spin:
                time.sleep(0.05)
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
            status = self._slam_status
            if status is None or status.current_map_name != map_name:
                continue
            ready = status.map_status == SlamStatus.MAP_STATUS_READY
            tracking = (
                status.loc_status == SlamStatus.LOC_STATUS_TRACKING
                and status.tracking_quality >= min_quality
            )
            if ready and (tracking or not require_tracking):
                return True
        return False

    def _load_map(self, map_name: str) -> bool:
        if not self._slam_mode.wait_for_service(timeout_sec=self.interface_timeout):
            self._state('FAILED', reason='SetSlamMode unavailable')
            return False
        self._state('SWITCHING_MAP', map_name=map_name)
        self._announce(f'正在切换到{map_name}地图，请稍候。')
        request = SetSlamMode.Request()
        request.mode = SlamStatus.MODE_LOCALIZATION
        request.request_reloc = False
        request.map_name = map_name
        request.client_id = 'hospital_multi_floor_navigator'
        request.client_req_id = int(time.time_ns() & 0xFFFFFFFFFFFFFFFF)
        future = self._slam_mode.call_async(request)
        if not self._wait_future(future, self.interface_timeout):
            self._state('FAILED', reason='SetSlamMode timeout', map_name=map_name)
            return False
        response = future.result()
        if not response.accepted:
            self._state('FAILED', reason=response.message, map_name=map_name)
            return False
        timeout = float(self.get_parameter('map_switch_timeout_sec').value)
        if not self._wait_map_status(map_name, require_tracking=False, timeout=timeout):
            self._state('FAILED', reason='map load status timeout', map_name=map_name)
            return False
        return True

    def _set_initial_pose(self, node: HospitalNode) -> bool:
        self._state(
            'LOCALIZING', map_name=node.map_name, x=node.x, y=node.y, yaw=node.yaw
        )
        self._announce(f'已到达{node.floor}层，正在重新定位。')
        if not self._localization.wait_for_service(timeout_sec=self.interface_timeout):
            self._state('FAILED', reason='LocCmd unavailable')
            return False
        request = LocCmd.Request()
        request.id = LocCmd.Request.CMD_LOC_INIT_POSE
        request.x, request.y, request.z = node.x, node.y, node.yaw
        request.map_id = node.map_name
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        future = self._localization.call_async(request)
        if not self._wait_future(future, self.interface_timeout):
            self._state('FAILED', reason='LocCmd timeout')
            return False
        response = future.result()
        if response.result != 0:
            self._state('FAILED', reason=f'LocCmd result={response.result}')
            return False
        timeout = float(self.get_parameter('map_switch_timeout_sec').value)
        if not self._wait_map_status(
            node.map_name, require_tracking=True, timeout=timeout
        ):
            self._state(
                'FAILED', reason='localization tracking timeout', map_name=node.map_name
            )
            return False
        return True

    def _wait_elevator(
        self,
        elevator_id: str,
        source_floor: str,
        target_floor: str,
    ) -> bool:
        self._state(
            'WAITING_FOR_ELEVATOR',
            elevator_id=elevator_id,
            target_floor=target_floor,
        )
        self._announce(f'请乘坐{elevator_id}号电梯前往{target_floor}层。')
        self._state(
            'RIDING_ELEVATOR',
            elevator_id=elevator_id,
            from_floor=source_floor,
            to_floor=target_floor,
        )
        if bool(self.get_parameter('auto_confirm_elevator').value):
            self._state('ELEVATOR_CONFIRMED', source='simulation')
            return True
        self._elevator_confirmed = False
        timeout = float(
            self.get_parameter('elevator_confirmation_timeout_sec').value
        )
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._external_spin:
                time.sleep(0.05)
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._elevator_confirmed:
                self._state('ELEVATOR_CONFIRMED', source='operator')
                return True
        self._state('FAILED', reason='elevator confirmation timeout')
        return False

    def run_route(self, start_id=None, goal_id=None) -> bool:
        start_id = start_id or str(self.get_parameter('start_node').value)
        goal_id = goal_id or str(self.get_parameter('goal_node').value)
        self._state('PLAN_ROUTE', start=start_id, goal=goal_id)
        try:
            route = self.graph.plan(start_id, goal_id)
        except (ValueError, RuntimeError) as exc:
            self._state('FAILED', reason=str(exc))
            self._write_report(False, None)
            return False
        route_json = json.dumps(route.as_dict(), ensure_ascii=False)
        self._route_pub.publish(String(data=route_json))
        self._state(
            'ROUTE_READY',
            total_time_sec=route.total_time_sec,
            equivalent_distance_m=route.equivalent_distance_m,
        )
        self._announce(
            f'路径规划完成，共有{len(route.steps)}个步骤，'
            f'预计需要{round(route.total_time_sec)}秒。'
        )

        start = route.start_node
        current_map = (
            self._slam_status.current_map_name if self._slam_status is not None else ''
        )
        if current_map != start.map_name and not self._load_map(start.map_name):
            self._write_report(False, route)
            return False
        if bool(self.get_parameter('initialize_start_pose').value):
            if not self._set_initial_pose(start):
                self._write_report(False, route)
                return False

        for step_index, step in enumerate(route.steps, start=1):
            if step.edge.edge_type == 'walk':
                target = step.to_node
                self._state(
                    'NAVIGATING_ON_MAP',
                    map_name=target.map_name,
                    target=target.node_id,
                )
                self._announce(
                    f'第{step_index}步，共{len(route.steps)}步。'
                    f'正在前往{target.name}。'
                )
                ok, message, error_code = self.navigate_to_pose(
                    {'x': target.x, 'y': target.y, 'z': 0.0, 'yaw': target.yaw}
                )
                if not ok:
                    self.stop_motion()
                    self._announce('导航失败，机器人已经停止，请工作人员协助。')
                    self._state(
                        'FAILED',
                        reason=message,
                        error_code=error_code,
                        target=target.node_id,
                    )
                    self._write_report(False, route)
                    return False
                self._state('WAYPOINT_REACHED', target=target.node_id)
                self._announce(f'已到达{target.name}。')
                continue

            target = step.to_node
            elevator_id = step.edge.elevator_id or target.elevator_id or 'A'
            if not self._wait_elevator(
                elevator_id, step.from_node.floor, target.floor
            ):
                self._write_report(False, route)
                return False
            if not self._load_map(target.map_name):
                self._write_report(False, route)
                return False
            if not self._set_initial_pose(target):
                self._write_report(False, route)
                return False
            self._state('MAP_SWITCH_COMPLETE', map_name=target.map_name)
            self._announce(f'{target.floor}层定位成功，准备继续导航。')

        self._state('ARRIVED', goal=route.goal_node.node_id)
        self._announce(f'路线已经完成，已到达{route.goal_node.name}。')
        self._write_report(True, route)
        return True

    def _write_report(self, success, route) -> None:
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        report = {
            'success': bool(success),
            'route': route.as_dict() if route is not None else None,
            'events': self._events,
            'run_id': str(uuid.uuid4()),
        }
        path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiFloorNavigator()
    success = False
    try:
        success = node.run_route()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(2)
