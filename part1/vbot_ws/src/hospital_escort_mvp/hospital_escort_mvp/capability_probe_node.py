import json
import time
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.node import Node


EXPECTED = {
    'coordinate_navigation': ('action', 'function_msgs/action/GoalNav', True),
    'following_or_accompany': ('service', 'function_msgs/srv/ControlFollowing', True),
    'speech_output': ('service', 'function_msgs/srv/SetSpeak', True),
    'semantic_navigation': ('service', 'function_msgs/srv/NavigateToTarget', False),
    'semantic_query': ('service', 'function_msgs/srv/SemanticNavQuery', False),
    'slam_mode': ('service', 'slam_msgs/srv/SetSlamMode', False),
    'save_map': ('service', 'slam_msgs/srv/SaveMap', False),
    'localization_command': ('service', 'slam_msgs/srv/LocCmd', False),
    'local_map_query': ('service', 'slam_msgs/srv/QueryMap', False),
    'semantic_path': ('service', 'slam_msgs/srv/GetPathToTarget', False),
    'navigation_status': ('topic', 'function_msgs/msg/NavigationStatus', True),
    'slam_status': ('topic', 'slam_msgs/msg/SlamStatus', True),
    'point_cloud': ('topic', 'sensor_msgs/msg/PointCloud2', False),
    'camera_image': ('topic', 'sensor_msgs/msg/Image', False),
    'imu': ('topic', 'sensor_msgs/msg/Imu', False),
    'tf': ('topic', 'tf2_msgs/msg/TFMessage', True),
}


class CapabilityProbe(Node):
    def __init__(self) -> None:
        super().__init__('vbot_capability_probe')
        self.declare_parameter('discovery_seconds', 3.0)
        self.declare_parameter('report_path', '/vbot_ws/reports/vbot_capability_report.json')

    @staticmethod
    def _matching_names(entries: Iterable[Tuple[str, List[str]]], expected_type: str) -> List[str]:
        return sorted(name for name, types in entries if expected_type in types)

    def run(self) -> Dict[str, object]:
        deadline = time.monotonic() + float(self.get_parameter('discovery_seconds').value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        topics = self.get_topic_names_and_types()
        services = self.get_service_names_and_types()
        actions = get_action_names_and_types(self)
        graphs = {'topic': topics, 'service': services, 'action': actions}

        capabilities = {}
        for key, (kind, expected_type, required) in EXPECTED.items():
            names = self._matching_names(graphs[kind], expected_type)
            capabilities[key] = {
                'status': 'PASS' if names else 'NOT_FOUND',
                'required_for_single_floor_mvp': required,
                'interface_kind': kind,
                'expected_type': expected_type,
                'discovered_names': names,
            }

        required = [
            value for value in capabilities.values()
            if value['required_for_single_floor_mvp']
        ]
        passed_required = sum(value['status'] == 'PASS' for value in required)
        report = {
            'probe_scope': (
                'ROS graph presence and type matching; physical behavior is not validated'
            ),
            'summary': {
                'required_passed': passed_required,
                'required_total': len(required),
                'all_passed': sum(value['status'] == 'PASS' for value in capabilities.values()),
                'all_total': len(capabilities),
            },
            'capabilities': capabilities,
            'graph': {
                'topics': [{'name': name, 'types': types} for name, types in topics],
                'services': [{'name': name, 'types': types} for name, types in services],
                'actions': [{'name': name, 'types': types} for name, types in actions],
            },
        }
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

        for key, value in capabilities.items():
            self.get_logger().info(
                f"[{value['status']}] {key}: {value['expected_type']} "
                f"{value['discovered_names']}"
            )
        self.get_logger().info(f'Capability report written to {path}')
        return report


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapabilityProbe()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
