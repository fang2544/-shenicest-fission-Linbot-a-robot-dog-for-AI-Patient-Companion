import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory

from hospital_escort_mvp.config_loader import load_hospital_config
from hospital_escort_mvp.vita_client import VitaInterfaceClient


class EscortDemo(VitaInterfaceClient):
    def __init__(self) -> None:
        super().__init__('escort_demo')
        default_config = str(
            Path(get_package_share_directory('hospital_escort_mvp'))
            / 'config'
            / 'hospital_F1.yaml'
        )
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('arrival_wait_sec', 0.2)
        self.declare_parameter('log_path', '/vbot_ws/reports/escort_demo.jsonl')
        self._config = load_hospital_config(str(self.get_parameter('config_path').value))
        self._log_path = Path(str(self.get_parameter('log_path').value))
        self._log_path.parent.mkdir(parents=True, exist_ok=True)

    def _event(self, event: str, **details) -> None:
        record = {'timestamp': time.time(), 'event': event, **details}
        with self._log_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.get_logger().info(json.dumps(record, ensure_ascii=False))

    def run_route(self) -> bool:
        ready = self.wait_until_ready()
        self._event('interface_readiness', **ready)
        if not all(ready.values()):
            self._event('mission_blocked', reason='required_interface_unavailable')
            return False

        locations = self._config['locations']
        route = self._config['route']
        self._event('mission_started', map_id=self._config['map_id'], route=route)

        for index, poi_key in enumerate(route, start=1):
            poi = locations[poi_key]
            speech = f"下一站前往{poi['name']}，请跟随我。"
            speak_ok, speak_message = self.speak(speech)
            self._event(
                'speech_result', poi=poi_key, success=speak_ok, message=speak_message
            )
            if not speak_ok:
                self.stop_motion()
                self._event('mission_blocked', reason='speech_failed', poi=poi_key)
                return False

            self._event('navigation_started', poi=poi_key, sequence=index)
            nav_ok, nav_message, error_code = self.navigate_to_pose(poi)
            self._event(
                'navigation_result',
                poi=poi_key,
                success=nav_ok,
                message=nav_message,
                error_code=error_code,
            )
            if not nav_ok:
                self.stop_motion()
                self.speak('导航失败，已经停止，请工作人员协助。')
                self._event('mission_blocked', reason='navigation_failed', poi=poi_key)
                return False

            self.speak(f"已到达{poi['name']}。")
            time.sleep(float(self.get_parameter('arrival_wait_sec').value))

        self._event('mission_completed', stops=len(route))
        self.speak('本次陪诊路线已经完成。')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EscortDemo()
    success = False
    try:
        success = node.run_route()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(2)
