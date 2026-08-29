#!/usr/bin/env python3
"""驱动并可视化 VBot 二次开发功能。"""

import html
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from function_msgs.msg import AsrResult, FollowStatus
from function_msgs.srv import FunctionInput
from lowlevel_msg.msg import UwbState
from peripheral_msgs.srv import GetSupportedEmotions, PlayEmotion
from speech_msgs.msg import ChatTtsNotification, WakeupInfo
from speech_msgs.srv import SpeechTextData


class SecondDevDemo(Node):
    """提供模拟 VBot 外设和固定顺序的功能演示。"""

    def __init__(self) -> None:
        super().__init__('second_dev_demo')
        defaults = {
            'graph_path': '/vbot_ws/src/vbot_simulation/config/simulation_cardiology_graph.yaml',
            'report_path': '/vbot_ws/reports/second_dev_acceptance.json',
            'html_report_path': '/vbot_ws/reports/second_dev_visual_report.html',
            'auto_start_cardiology': True,
            'cardiology_start_delay_sec': 45.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._started_at = time.monotonic()
        self._events = []
        self._event_names = set()
        self._wakeup_sent = False
        self._asr_sent = False
        self._latest_itinerary = {}
        self._robot_xy = (-4.5, 0.0)

        self._wakeup_pub = self.create_publisher(WakeupInfo, '/speech/WakeupInfo', 10)
        self._follow_pub = self.create_publisher(FollowStatus, '/function/FollowStatus', 10)
        self._uwb_pub = self.create_publisher(UwbState, '/lowlevel/UwbState', 10)
        self._asr_pub = self.create_publisher(AsrResult, '/asr/result', 10)
        self._tts_notification_pub = self.create_publisher(
            ChatTtsNotification, '/speech/ChatTtsNotification', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/second_dev/markers', 10)
        self._status_pub = self.create_publisher(String, '/second_dev/status', 10)

        self.create_subscription(String, '/second_dev/events', self._event_cb, 20)
        self.create_subscription(
            String, '/cardiology_itinerary/state', self._itinerary_cb, 20)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._mission_client = self.create_client(
            FunctionInput, '/hospital/request_mission')

        self.create_service(
            SpeechTextData, '/speech/SpeechTextData', self._speech_text)
        self.create_service(PlayEmotion, '/peripheral/PlayEmotion', self._play_emotion)
        self.create_service(
            GetSupportedEmotions,
            '/peripheral/GetSupportedEmotions',
            self._supported_emotions,
        )

        self._stations = self._load_stations()
        self.create_timer(0.2, self._tick)
        self._record('demo_harness', 'SIMULATOR_READY')

    def _load_stations(self):
        path = Path(str(self.get_parameter('graph_path').value))
        with path.open('r', encoding='utf-8') as stream:
            graph = yaml.safe_load(stream)
        stations = []
        for floor in graph.get('floors', {}).values():
            for node_id, value in floor.get('nodes', {}).items():
                stations.append({
                    'id': node_id,
                    'name': str(value.get('name', node_id)),
                    'x': float(value['x']),
                    'y': float(value['y']),
                })
        return stations

    def _record(self, source: str, event: str, **details) -> None:
        entry = {
            'wall_time': time.time(),
            'elapsed_sec': round(time.monotonic() - self._started_at, 3),
            'source': source,
            'event': event,
            **details,
        }
        self._events.append(entry)
        self._event_names.add(event)
        self.get_logger().info(json.dumps(entry, ensure_ascii=False))
        self._write_reports()

    def _event_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {'event': 'UNPARSEABLE_EVENT', 'data': message.data}
        self._record(
            str(payload.pop('source', 'feature')),
            str(payload.pop('event', 'UNKNOWN')),
            **payload,
        )

    def _itinerary_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._latest_itinerary = payload
        details = dict(payload)
        event = str(details.pop('event', 'ITINERARY_STATE'))
        self._record('hospital_2_5d_navigation', event, **details)
        if payload.get('phase') == 'COMPLETED':
            self._record(
                'hospital_2_5d_navigation',
                'NAVIGATION_COMPLETED',
                phase='COMPLETED',
            )

    def _odom_cb(self, message: Odometry) -> None:
        self._robot_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _speech_text(self, request, response):
        response.success = True
        response.message = 'simulated TTS accepted'
        self._record('simulated_peripheral', 'TTS_REQUEST', text=request.text)
        notification = ChatTtsNotification()
        notification.notification_type = ChatTtsNotification.TTS_START_PLAYBACK
        self._tts_notification_pub.publish(notification)
        return response

    def _play_emotion(self, request, response):
        response.success = True
        response.message = 'simulated emotion accepted'
        response.error_code = 0
        self._record(
            'simulated_peripheral', 'EMOTION_REQUEST', mode=int(request.mode))
        return response

    @staticmethod
    def _supported_emotions(_request, response):
        response.success = True
        response.message = 'simulation emotions'
        response.supported_emotions = ['1:welcome', '2:lost-reminder']
        return response

    def _publish_follow_and_distance(self, distance: float) -> None:
        follow = FollowStatus()
        follow.status = FollowStatus.STATUS_RUNNING
        follow.mode = FollowStatus.FOLLOWING
        follow.modify_source = 'second_dev_demo'
        follow.follow_speed = 0.3
        self._follow_pub.publish(follow)
        uwb = UwbState()
        uwb.distance_est = float(distance)
        uwb.yaw_est = 0.0
        self._uwb_pub.publish(uwb)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._started_at
        if (
            not self._wakeup_sent
            and elapsed >= 3.0
            and self._wakeup_pub.get_subscription_count() > 0
        ):
            message = WakeupInfo()
            message.timestamp = self.get_clock().now().to_msg()
            message.wakeup_source = WakeupInfo.WAKEUP_SOURCE_DOG
            message.doa_angle = 25
            message.wakeup_word = '大头大头'
            self._wakeup_pub.publish(message)
            self._wakeup_sent = True
            self._record('demo_harness', 'WAKEUP_SENT')

        if elapsed >= 6.0 and 'LOST_PAUSE' not in self._event_names:
            self._publish_follow_and_distance(4.2)
        elif 'LOST_PAUSE' in self._event_names and 'LOST_RESUME' not in self._event_names:
            self._publish_follow_and_distance(2.0)
        elif (
            bool(self.get_parameter('auto_start_cardiology').value)
            and not self._asr_sent
            and 'LOST_RESUME' in self._event_names
            and elapsed >= float(self.get_parameter('cardiology_start_delay_sec').value)
            and self._asr_pub.get_subscription_count() > 0
            and self._mission_client.service_is_ready()
        ):
            message = AsrResult()
            message.header.stamp = self.get_clock().now().to_msg()
            message.transcript = '开始陪诊'
            message.source_type = AsrResult.DOG
            message.reject = False
            message.confidence = 0.99
            self._asr_pub.publish(message)
            self._asr_sent = True
            self._record('demo_harness', 'CARDIOLOGY_START_SENT')

        self._publish_visualization()
        self._publish_status()

    def _checks(self):
        names = self._event_names
        return {
            'wakeup_event_received': 'WAKEUP_DETECTED' in names,
            'welcome_tts_requested': 'WELCOME_TTS' in names,
            'lost_pause_triggered': 'LOST_PAUSE' in names,
            'lost_reminder_tts_requested': 'LOST_REMINDER_TTS' in names,
            'lost_resume_triggered': 'LOST_RESUME' in names,
            'follow_control_accepted': 'FOLLOW_CONTROL_ACCEPTED' in names,
            'hospital_navigation_completed': 'NAVIGATION_COMPLETED' in names,
        }

    def _publish_status(self) -> None:
        checks = self._checks()
        status = {
            'success': all(checks.values()),
            'checks': checks,
            'latest_itinerary_phase': self._latest_itinerary.get('phase', 'WAITING'),
            'robot_xy': [round(v, 3) for v in self._robot_xy],
        }
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _publish_visualization(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        route = Marker()
        route.header.frame_id = 'map'
        route.header.stamp = stamp
        route.ns = 'second_dev_route'
        route.id = 1
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.scale.x = 0.06
        route.color.r, route.color.g, route.color.b, route.color.a = 0.15, 0.75, 1.0, 0.9
        for station in self._stations:
            route.points.append(Point(x=station['x'], y=station['y'], z=0.08))
        route.points.append(Point(x=self._stations[0]['x'], y=self._stations[0]['y'], z=0.08))
        markers.markers.append(route)

        for index, station in enumerate(self._stations):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = stamp
            sphere.ns = 'second_dev_stations'
            sphere.id = 100 + index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = station['x']
            sphere.pose.position.y = station['y']
            sphere.pose.position.z = 0.18
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.24
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.8, 0.45, 0.95
            markers.markers.append(sphere)
            label = Marker()
            label.header = sphere.header
            label.ns = 'second_dev_station_labels'
            label.id = 200 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = station['x']
            label.pose.position.y = station['y']
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"{index + 1}. {station['name']}"
            markers.markers.append(label)

        checks = self._checks()
        rows = [
            ('唤醒欢迎', checks['welcome_tts_requested']),
            ('跟丢暂停/提醒/恢复', checks['lost_resume_triggered']),
            ('医院 2.5D Nav2 陪诊', checks['hospital_navigation_completed']),
        ]
        for index, (label_text, passed) in enumerate(rows):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'second_dev_status'
            marker.id = 300 + index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = -4.5 + index * 3.5
            marker.pose.position.y = -2.0
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.32
            marker.color.r = 0.2 if passed else 1.0
            marker.color.g = 1.0 if passed else 0.65
            marker.color.b = 0.25
            marker.color.a = 1.0
            marker.text = ('PASS  ' if passed else 'RUNNING  ') + label_text
            markers.markers.append(marker)
        self._marker_pub.publish(markers)

    def _write_reports(self) -> None:
        checks = self._checks()
        payload = {
            'success': all(checks.values()),
            'checks': checks,
            'events': self._events,
            'visualization_topics': {
                'rviz_markers': '/second_dev/markers',
                'status_json': '/second_dev/status',
            },
        }
        report = Path(str(self.get_parameter('report_path').value))
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')

        cards = ''.join(
            f'<article class="card {"pass" if value else "running"}">'
            f'<strong>{"PASS" if value else "RUNNING"}</strong>'
            f'<span>{html.escape(name)}</span></article>'
            for name, value in checks.items()
        )
        timeline = ''.join(
            '<li><time>+{:.1f}s</time><b>{}</b><span>{}</span></li>'.format(
                float(event['elapsed_sec']),
                html.escape(str(event['source'])),
                html.escape(str(event['event'])),
            )
            for event in self._events[-60:]
        )
        page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta http-equiv="refresh" content="3"><title>VBot 三功能仿真验收</title>
<style>body{{font:16px system-ui;margin:32px;background:#0c1220;color:#eef3ff}}
h1{{font-size:28px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:12px}}
.card{{padding:18px;border-radius:14px;background:#182238;display:flex;gap:12px;flex-direction:column}}
.pass strong{{color:#64e795}}.running strong{{color:#ffc766}}ul{{padding:0;list-style:none}}
li{{display:grid;grid-template-columns:80px 210px 1fr;padding:8px;border-bottom:1px solid #27344e}}
time{{color:#8ea0bf}}small{{color:#9db0d0}}</style><body>
<h1>VBot 二开三功能仿真验收</h1><small>页面每 3 秒刷新；Gazebo/RViz 中查看 /second_dev/markers</small>
<div class="grid">{cards}</div><h2>事件时间线</h2><ul>{timeline}</ul></body></html>'''
        Path(str(self.get_parameter('html_report_path').value)).write_text(
            page, encoding='utf-8')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SecondDevDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
