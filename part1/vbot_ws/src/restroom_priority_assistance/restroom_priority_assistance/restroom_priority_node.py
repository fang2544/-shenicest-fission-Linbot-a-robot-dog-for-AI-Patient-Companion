"""识别找厕所语音并发布独立的最高优先目的地请求。"""

import json
from pathlib import Path
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from function_msgs.msg import AsrResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


def match_destination(destinations, transcript: str):
    """按配置顺序返回首个匹配语音关键词的目的地。"""
    normalized = ''.join(str(transcript).lower().split())
    for target, config in destinations.items():
        for phrase in config.get('phrases', []):
            if ''.join(str(phrase).lower().split()) in normalized:
                return str(target), dict(config)
    return None


class RestroomPriorityNode(Node):
    def __init__(self) -> None:
        super().__init__('restroom_priority_node')
        share = Path(get_package_share_directory('restroom_priority_assistance'))
        defaults = {
            'config_path': str(share / 'config' / 'restroom_priority.yaml'),
            'asr_topic': '/asr/result',
            'priority_request_topic': '/hospital/priority_destination',
            'status_topic': '/restroom_priority/status',
            'report_path': '/vbot_ws/reports/restroom_priority_events.json',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        with Path(str(self.get_parameter('config_path').value)).open(
            'r', encoding='utf-8'
        ) as stream:
            self._config = yaml.safe_load(stream)
        self._events = []
        self._request_pub = self.create_publisher(
            String, str(self.get_parameter('priority_request_topic').value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self.create_subscription(
            AsrResult, str(self.get_parameter('asr_topic').value), self._asr_cb, 20
        )
        self._record('READY')

    def _asr_cb(self, message: AsrResult) -> None:
        transcript = str(message.transcript)
        confidence = float(message.confidence)
        if message.reject or confidence < float(self._config['minimum_confidence']):
            self._record(
                'ASR_IGNORED', transcript=transcript, confidence=confidence,
                reason='rejected_or_low_confidence',
            )
            return
        matched = match_destination(self._config['destinations'], transcript)
        if matched is None:
            return
        target, config = matched
        request = {
            'request_id': str(uuid.uuid4()),
            'priority': 'highest',
            'target': target,
            'label': str(config['label']),
            'depart_prompt': str(config['depart_prompt']),
            'arrival_prompt': str(config['arrival_prompt']),
            'source': 'restroom_voice_intent',
            'transcript': transcript,
        }
        self._request_pub.publish(
            String(data=json.dumps(request, ensure_ascii=False))
        )
        self._record('PRIORITY_REQUESTED', **request)

    def _record(self, event: str, **details) -> None:
        payload = {'timestamp': time.time(), 'event': event, **details}
        self._events.append(payload)
        encoded = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(String(data=encoded))
        self.get_logger().info(encoded)
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps({'events': self._events}, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RestroomPriorityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
