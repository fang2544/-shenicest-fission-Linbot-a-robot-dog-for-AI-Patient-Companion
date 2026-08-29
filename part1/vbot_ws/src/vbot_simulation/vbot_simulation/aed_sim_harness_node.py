#!/usr/bin/env python3
"""注入 AED 求助语音并生成可重复的 Gazebo/Nav2 验收报告。"""

import html
import json
import math
from pathlib import Path
import time

import rclpy
from function_msgs.msg import AsrResult
from nav_msgs.msg import Odometry
from rclpy.node import Node
from speech_msgs.srv import SpeechTextData
from std_msgs.msg import String


class AedSimHarness(Node):
    def __init__(self):
        super().__init__('aed_sim_harness')
        defaults = {
            'request_delay_sec': 45.0,
            'target_x': -0.9,
            'target_y': 0.0,
            'arrival_tolerance_m': 0.40,
            'report_path': '/vbot_ws/reports/aed_emergency_acceptance.json',
            'html_report_path': '/vbot_ws/reports/aed_emergency_visual_report.html',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.started = time.monotonic()
        self.request_sent = False
        self.events = []
        self.names = set()
        self.tts = []
        self.robot_xy = (-4.5, 0.0)
        self.pub = self.create_publisher(AsrResult, '/asr/result', 10)
        self.create_subscription(String, '/aed_emergency/status', self._status, 20)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_service(SpeechTextData, '/speech/SpeechTextData', self._speech)
        self.create_timer(0.2, self._tick)
        self._write_reports()

    def _status(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        event['observed_elapsed_sec'] = round(time.monotonic() - self.started, 3)
        self.events.append(event)
        self.names.add(str(event.get('event')))
        self.get_logger().info(json.dumps(event, ensure_ascii=False))
        self._write_reports()

    def _odom(self, message):
        self.robot_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _speech(self, request, response):
        self.tts.append(request.text)
        response.success = True
        response.message = 'simulated speaker completed'
        self._write_reports()
        return response

    def _tick(self):
        if (
            not self.request_sent
            and time.monotonic() - self.started
            >= float(self.get_parameter('request_delay_sec').value)
            and self.pub.get_subscription_count() > 0
        ):
            message = AsrResult()
            message.header.stamp = self.get_clock().now().to_msg()
            message.transcript = '心电图室有人倒地不醒没有正常呼吸，马上送AED来急救'
            message.source_type = AsrResult.DOG
            message.reject = False
            message.confidence = 0.99
            self.pub.publish(message)
            self.request_sent = True
            self.get_logger().info('AED emergency ASR request published')
        self._write_reports()

    def _checks(self):
        target_x = float(self.get_parameter('target_x').value)
        target_y = float(self.get_parameter('target_y').value)
        error = math.hypot(self.robot_xy[0] - target_x, self.robot_xy[1] - target_y)
        all_speech = ''.join(self.tts)
        guidance_events = [e for e in self.events if e.get('event') == 'GUIDANCE_STEP']
        checks = {
            'voice_intent_recognized': 'INTENT_ACCEPTED' in self.names,
            'previous_task_preempted': 'PREEMPT_COMPLETED' in self.names,
            'nav2_goal_accepted': 'NAVIGATION_ACCEPTED' in self.names,
            'nav2_progress_observed': 'NAVIGATION_PROGRESS' in self.names,
            'robot_arrived': 'ARRIVED' in self.names,
            'arrival_error_within_0_40m': error <= float(
                self.get_parameter('arrival_tolerance_m').value),
            'seven_guidance_steps': len(guidance_events) == 7,
            'nine_tts_requests_completed': len(self.tts) == 9,
            'guidance_calls_emergency_services': '120' in all_speech,
            'guidance_has_cpr_rate': '100到120' in all_speech,
            'guidance_has_clear_warning': '不要接触' in all_speech,
            'workflow_completed': 'COMPLETED' in self.names,
            'no_failed_state': 'FAILED' not in self.names,
        }
        return checks, error

    def _write_reports(self):
        checks, error = self._checks()
        payload = {
            'success': all(checks.values()),
            'scope': 'Gazebo physics + AMCL + real Nav2 action + simulated ASR/TTS',
            'clinical_validation': False,
            'checks': checks,
            'robot_xy': [round(v, 3) for v in self.robot_xy],
            'target_xy': [
                float(self.get_parameter('target_x').value),
                float(self.get_parameter('target_y').value),
            ],
            'arrival_error_m': round(error, 3),
            'tts_requests': self.tts,
            'events': self.events,
            'visualization': {
                'gazebo_model': 'vbot_simple with AED and sealed emergency_supply_case links',
                'rviz_marker_topic': '/aed_emergency/markers',
                'nav2_plan_topic': '/plan',
            },
        }
        report = Path(str(self.get_parameter('report_path').value))
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')

        cards = ''.join(
            f'<article class="card {"pass" if value else "running"}">'
            f'<strong>{"PASS" if value else "RUNNING"}</strong><span>{html.escape(key)}</span></article>'
            for key, value in checks.items()
        )
        timeline = ''.join(
            '<li><time>+{:.1f}s</time><b>{}</b><span>{}</span></li>'.format(
                float(e.get('observed_elapsed_sec', 0.0)),
                html.escape(str(e.get('event', 'UNKNOWN'))),
                html.escape(str(e.get('message', e.get('text', '')))),
            )
            for e in self.events[-50:]
        )
        status = 'PASS' if payload['success'] else 'RUNNING'
        page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta http-equiv="refresh" content="3"><title>VBot AED 紧急响应验收</title>
<style>body{{font:15px system-ui;margin:30px;background:#111827;color:#f8fafc}}h1{{margin-bottom:4px}}
.hero{{border-left:6px solid #ef3340;padding:12px 18px;background:#1f2937;margin:18px 0}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}}
.card{{padding:14px;border-radius:10px;background:#253248;display:flex;gap:9px;flex-direction:column}}
.pass strong{{color:#65e69a}}.running strong{{color:#ffc966}}ul{{padding:0;list-style:none}}
li{{display:grid;grid-template-columns:75px 220px 1fr;padding:7px;border-bottom:1px solid #334155}}
time,small{{color:#9fb0c9}}.warning{{color:#ffd0d3}}</style><body><h1>VBot AED 紧急医疗场景</h1>
<small>Gazebo + AMCL + Nav2 + VBot 接口端到端验收</small><div class="hero"><b>{status}</b>
　机器人位置 {self.robot_xy[0]:.2f}, {self.robot_xy[1]:.2f}　到达误差 {error:.2f} m</div>
<p class="warning">仿真通过不代表临床有效；真实事件必须立即启动院内急救并拨打 120。</p>
<div class="grid">{cards}</div><h2>事件时间线</h2><ul>{timeline}</ul></body></html>'''
        Path(str(self.get_parameter('html_report_path').value)).write_text(page, encoding='utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = AedSimHarness()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
