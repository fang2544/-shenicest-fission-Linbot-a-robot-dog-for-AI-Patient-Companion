#!/usr/bin/env python3
"""识别医疗求助，配送急救物资并向现场人员播报操作提示。"""

import json
from pathlib import Path
import time
import uuid

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from speech_msgs.srv import SpeechTextData
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from function_msgs.action import GoalNav
from function_msgs.msg import AsrResult


class AedEmergencyNode(Node):
    """基于 VBot 公开接口执行最高优先级急救任务。"""

    def __init__(self) -> None:
        super().__init__('aed_emergency_node')
        self._declare_parameters()
        self._locations = json.loads(str(self.get_parameter('locations_json').value))
        self._events = []
        self._event_names = set()
        self._state = 'IDLE'
        self._active_target = None
        self._emergency_type = 'cardiac'
        self._request_id = None
        self._action_deadline = None
        self._stop_send_future = None
        self._nav_send_future = None
        self._nav_result_future = None
        self._last_feedback_bucket = None
        self._guidance_index = 0
        self._next_guidance_at = None
        self._speech_queue = []
        self._speech_future = None

        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 20)
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.create_subscription(
            AsrResult,
            str(self.get_parameter('asr_topic').value),
            self._asr_cb,
            20,
        )
        self._speech_client = self.create_client(
            SpeechTextData,
            str(self.get_parameter('speech_text_data_service').value),
        )
        self._goal_client = ActionClient(
            self,
            GoalNav,
            str(self.get_parameter('goal_nav_action').value),
        )
        self.create_timer(0.1, self._tick)
        self._record('READY', message='medical emergency mode is ready')

    def _declare_parameters(self) -> None:
        defaults = {
            'asr_topic': '/asr/result',
            'goal_nav_action': '/goal_nav',
            'speech_text_data_service': '/speech/SpeechTextData',
            'status_topic': '/aed_emergency/status',
            'marker_topic': '/aed_emergency/markers',
            'minimum_confidence': 0.65,
            'default_location': 'ecg_2f',
            'action_server_timeout_sec': 15.0,
            'navigation_timeout_sec': 240.0,
            'guidance_interval_sec': 6.0,
            'report_path': '/vbot_ws/reports/aed_emergency_events.json',
            'aed_keywords': ['aed', '自动体外除颤器', '除颤器', '电除颤'],
            'cardiac_keywords': [
                '心脏骤停', '心跳停了', '没有心跳', '心脏起搏',
                '心源性猝死', '突然倒地', '倒地不醒', '没有呼吸',
            ],
            'bleeding_keywords': ['大出血', '严重出血', '流血不止', '止血'],
            'allergy_keywords': ['严重过敏', '过敏性休克', '喉咙肿', '过敏呼吸困难'],
            'hypoglycemia_keywords': ['低血糖', '血糖太低', '低血糖昏迷'],
            'medical_supply_keywords': ['急救药品', '急救箱', '急救物资'],
            'urgency_keywords': [
                '需要', '拿来', '送来', '送过去', '快来', '赶快', '马上',
                '急救', '救命', '紧急',
            ],
            'negative_phrases': [
                '不需要aed', '不要aed', '无需aed', '取消急救',
                '只是演练', '模拟演练',
            ],
            'locations_json': json.dumps({
                'ecg_2f': {
                    'name': '心电图室', 'x': -0.9, 'y': 0.0, 'yaw': 0.0,
                    'aliases': ['心电图室', '心电图'],
                }
            }, ensure_ascii=False),
            'supplies_json': json.dumps({
                'cardiac': ['AED', '一次性手套', '呼吸膜'],
                'bleeding': ['止血敷料', '无菌纱布', '弹性绷带', '一次性手套'],
                'allergy': ['急救箱', '肾上腺素自动注射器封存盒'],
                'hypoglycemia': ['血糖仪', '葡萄糖凝胶封存盒', '一次性手套'],
                'general': ['综合急救箱', '急救药品封存盒'],
            }, ensure_ascii=False),
            'cardiac_guidance_steps': [
                '我是聆灵。AED已送达。请确认现场安全，检查患者是否有反应和正常呼吸。',
                '请立即呼叫院内急救团队并拨打120。',
                '若患者无反应且没有正常呼吸或仅有濒死喘息，立即胸外按压，成人每分钟100到120次。',
                '打开AED，严格听从设备语音提示。',
                'AED分析心律时，所有人不要接触患者。',
                '若AED提示电击，确认无人接触后再按下电击键。',
                '电击后或不建议电击时，立即继续胸外按压约两分钟，直到急救人员接手。',
            ],
            'bleeding_guidance_steps': [
                '我是聆灵，止血急救物资已送达。请立即呼叫院内急救团队并拨打120。',
                '请戴上手套，用止血敷料或纱布直接持续按压出血处。',
                '敷料被血浸透时不要揭掉，在上面继续加敷料并持续按压。',
                '止血带等器材只由受过训练的人员按规范使用，等待急救人员接手。',
            ],
            'allergy_guidance_steps': [
                '我是聆灵，严重过敏急救物资已送达。请立即呼叫院内急救团队并拨打120。',
                '让患者保持便于呼吸的姿势，持续观察意识和呼吸。',
                '肾上腺素自动注射器仅由患者本人、医护人员或受过训练者按医嘱和产品说明使用。',
                '若患者无反应且没有正常呼吸，立即开始心肺复苏并等待急救人员接手。',
            ],
            'hypoglycemia_guidance_steps': [
                '我是聆灵，低血糖急救物资已送达。请立即通知医护人员。',
                '患者清醒且能安全吞咽时，葡萄糖凝胶只按医嘱或产品说明使用。',
                '患者意识不清或不能吞咽时不要喂食、喂水或口服药，立即拨打120。',
                '持续观察呼吸和意识，等待医护人员接手。',
            ],
            'general_guidance_steps': [
                '我是聆灵，综合急救箱和急救药品封存盒已送达。',
                '请立即通知院内急救团队，严重情况拨打120。',
                '药品仅由有资质医护人员、患者本人或受过训练者依医嘱和标签使用。',
            ],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    @staticmethod
    def _normalize(text: str) -> str:
        return ''.join(text.lower().split())

    def _classify(self, transcript: str):
        text = self._normalize(transcript)
        negatives = [self._normalize(v) for v in self.get_parameter('negative_phrases').value]
        if any(value and value in text for value in negatives):
            return False, 'negative_phrase', None
        aed = any(self._normalize(v) in text for v in self.get_parameter('aed_keywords').value)
        cardiac = any(
            self._normalize(v) in text
            for v in self.get_parameter('cardiac_keywords').value
        )
        urgent = any(
            self._normalize(v) in text
            for v in self.get_parameter('urgency_keywords').value
        )
        categories = {
            'bleeding': 'bleeding_keywords',
            'allergy': 'allergy_keywords',
            'hypoglycemia': 'hypoglycemia_keywords',
            'general': 'medical_supply_keywords',
        }
        matched = {
            name: any(self._normalize(v) in text for v in self.get_parameter(param).value)
            for name, param in categories.items()
        }
        emergency_type = None
        if (aed and (cardiac or urgent)) or (cardiac and urgent):
            emergency_type = 'cardiac'
        else:
            emergency_type = next(
                (name for name in ('bleeding', 'allergy', 'hypoglycemia', 'general')
                 if matched[name] and urgent),
                None,
            )
        reason = (
            f'aed={aed},cardiac={cardiac},urgent={urgent},categories={matched}'
        )
        return emergency_type is not None, reason, emergency_type

    def _guidance_steps(self):
        return list(
            self.get_parameter(f'{self._emergency_type}_guidance_steps').value
        )

    def _supplies(self):
        supplies = json.loads(str(self.get_parameter('supplies_json').value))
        return list(supplies.get(self._emergency_type, supplies['general']))

    def _resolve_location(self, transcript: str):
        text = self._normalize(transcript)
        for location_id, value in self._locations.items():
            aliases = list(value.get('aliases', [])) + [value.get('name', '')]
            if any(self._normalize(alias) in text for alias in aliases if alias):
                return location_id, value, False
        location_id = str(self.get_parameter('default_location').value)
        return location_id, self._locations[location_id], True

    def _asr_cb(self, message: AsrResult) -> None:
        transcript = str(message.transcript)
        if message.reject:
            self._record('ASR_IGNORED', reason='rejected', transcript=transcript)
            return
        if float(message.confidence) < float(self.get_parameter('minimum_confidence').value):
            self._record('ASR_IGNORED', reason='low_confidence', transcript=transcript)
            return
        accepted, reason, emergency_type = self._classify(transcript)
        if not accepted:
            self._record('ASR_IGNORED', reason=reason, transcript=transcript)
            return
        if self._state not in {'IDLE', 'COMPLETED', 'FAILED'}:
            self._record('ASR_IGNORED', reason='emergency_already_active', transcript=transcript)
            return

        location_id, target, used_default = self._resolve_location(transcript)
        self._active_target = dict(target)
        self._emergency_type = str(emergency_type)
        self._active_target['id'] = location_id
        self._request_id = str(uuid.uuid4())
        self._guidance_index = 0
        self._last_feedback_bucket = None
        self._state = 'WAITING_TO_PREEMPT'
        self._action_deadline = (
            time.monotonic()
            + float(self.get_parameter('action_server_timeout_sec').value)
        )
        self._record(
            'INTENT_ACCEPTED',
            request_id=self._request_id,
            transcript=transcript,
            confidence=float(message.confidence),
            classifier=reason,
            emergency_type=self._emergency_type,
            supplies=self._supplies(),
            target=location_id,
            target_name=target['name'],
            target_x=float(target['x']),
            target_y=float(target['y']),
            used_default_location=used_default,
        )
        supplies = '、'.join(self._supplies())
        self._queue_speech(
            f'收到急救请求，聆灵携带{supplies}，正在赶往{target["name"]}。'
            '请立即通知院内急救团队，严重情况拨打120。'
        )

    def _tick(self) -> None:
        self._try_send_speech()
        self._publish_markers()
        now = time.monotonic()
        if self._state == 'WAITING_TO_PREEMPT':
            if self._goal_client.server_is_ready():
                self._send_stop()
            elif now >= self._action_deadline:
                self._fail('GoalNav action server unavailable')
        elif self._state == 'NAVIGATING' and now >= self._action_deadline:
            self._fail('AED navigation timed out')
        elif self._state == 'GUIDING' and now >= self._next_guidance_at:
            self._speak_next_guidance()
        elif (
            self._state == 'FINISHING_GUIDANCE'
            and self._speech_future is None
            and not self._speech_queue
        ):
            self._state = 'COMPLETED'
            self._record(
                'COMPLETED',
                emergency_type=self._emergency_type,
                guidance_steps=len(self._guidance_steps()),
            )

    def _send_stop(self) -> None:
        self._state = 'PREEMPTING'
        goal = GoalNav.Goal()
        goal.control = False
        self._record('PREEMPT_REQUESTED')
        self._stop_send_future = self._goal_client.send_goal_async(goal)
        self._stop_send_future.add_done_callback(self._stop_sent)

    def _stop_sent(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._record('PREEMPT_FAILED', message=str(exc))
            self._send_navigation()
            return
        if not handle.accepted:
            self._record('PREEMPT_FAILED', message='stop goal rejected')
            self._send_navigation()
            return
        handle.get_result_async().add_done_callback(self._stop_result)

    def _stop_result(self, future) -> None:
        try:
            result = future.result().result
            self._record(
                'PREEMPT_COMPLETED', success=bool(result.success), message=result.message)
        except Exception as exc:  # noqa: BLE001
            self._record('PREEMPT_FAILED', message=str(exc))
        self._send_navigation()

    def _send_navigation(self) -> None:
        if self._state == 'NAVIGATING':
            return
        target = self._active_target
        goal = GoalNav.Goal()
        goal.control = True
        goal.x = float(target['x'])
        goal.y = float(target['y'])
        goal.z = 0.0
        goal.yaw = float(target.get('yaw', 0.0))
        self._state = 'SENDING_NAVIGATION'
        self._record(
            'NAV_GOAL_SENT', target=target['id'], x=goal.x, y=goal.y, yaw=goal.yaw)
        self._nav_send_future = self._goal_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        self._nav_send_future.add_done_callback(self._nav_sent)

    def _nav_sent(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._fail(f'GoalNav send failed: {exc}')
            return
        if not handle.accepted:
            self._fail('GoalNav emergency goal rejected')
            return
        self._state = 'NAVIGATING'
        self._action_deadline = (
            time.monotonic()
            + float(self.get_parameter('navigation_timeout_sec').value)
        )
        self._record('NAVIGATION_ACCEPTED')
        self._nav_result_future = handle.get_result_async()
        self._nav_result_future.add_done_callback(self._nav_result)

    def _nav_feedback(self, message) -> None:
        progress = int(message.feedback.progress[0]) if message.feedback.progress else 0
        bucket = max(0, min(100, progress)) // 10
        if bucket == self._last_feedback_bucket:
            return
        self._last_feedback_bucket = bucket
        self._record(
            'NAVIGATION_PROGRESS', progress=max(0, min(100, progress)),
            message=message.feedback.message,
        )

    def _nav_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self._fail(f'GoalNav result failed: {exc}')
            return
        if not result.success:
            self._fail(result.message or 'AED navigation failed')
            return
        # 先完整更新状态再发布，避免多线程执行器立即运行定时器时读到半成品状态。
        self._next_guidance_at = time.monotonic()
        self._state = 'GUIDING'
        self._record('ARRIVED', target=self._active_target['id'])
        self._queue_speech(
            f'急救物资已送达，请取下聆灵背部的{"、".join(self._supplies())}。'
        )

    def _speak_next_guidance(self) -> None:
        steps = self._guidance_steps()
        if self._guidance_index >= len(steps):
            # 所有指导语都收到语音服务响应后才算完成，不能只判断是否进入队列。
            self._state = 'FINISHING_GUIDANCE'
            return
        text = str(steps[self._guidance_index])
        self._guidance_index += 1
        self._queue_speech(text)
        self._record(
            'GUIDANCE_STEP', index=self._guidance_index, total=len(steps), text=text)
        self._next_guidance_at = (
            time.monotonic()
            + max(0.1, float(self.get_parameter('guidance_interval_sec').value))
        )

    def _queue_speech(self, text: str) -> None:
        self._speech_queue.append(text)
        self._try_send_speech()

    def _try_send_speech(self) -> None:
        if self._speech_future is not None or not self._speech_queue:
            return
        if not self._speech_client.service_is_ready():
            return
        text = self._speech_queue.pop(0)
        request = SpeechTextData.Request()
        request.data_type = SpeechTextData.Request.TYPE_TTS
        request.text = text
        request.sn = str(uuid.uuid4())
        request.timestamp = self.get_clock().now().to_msg()
        self._speech_future = self._speech_client.call_async(request)
        self._speech_future.add_done_callback(
            lambda future: self._speech_result(future, text))

    def _speech_result(self, future, text: str) -> None:
        self._speech_future = None
        try:
            response = future.result()
            self._record(
                'TTS_RESULT', success=bool(response.success), text=text,
                message=response.message,
            )
        except Exception as exc:  # noqa: BLE001
            self._record('TTS_RESULT', success=False, text=text, message=str(exc))
        self._try_send_speech()

    def _fail(self, message: str) -> None:
        if self._state == 'FAILED':
            return
        self._state = 'FAILED'
        self._record('FAILED', message=message)
        self._queue_speech(
            '急救物资送达任务失败，请立即由人员携带急救箱前往，'
            '并继续按急救人员指导进行处置。'
        )

    def _record(self, event: str, **details) -> None:
        payload = {
            'timestamp': time.time(),
            'state': self._state,
            'event': event,
            **details,
        }
        self._events.append(payload)
        self._event_names.add(event)
        encoded = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(String(data=encoded))
        self.get_logger().info(encoded)
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps({'events': self._events}, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

    def _publish_markers(self) -> None:
        if self._active_target is None:
            return
        stamp = self.get_clock().now().to_msg()
        target = self._active_target
        markers = MarkerArray()
        beacon = Marker()
        beacon.header.frame_id = 'map'
        beacon.header.stamp = stamp
        beacon.ns = 'aed_emergency'
        beacon.id = 1
        beacon.type = Marker.CYLINDER
        beacon.action = Marker.ADD
        beacon.pose.position = Point(
            x=float(target['x']), y=float(target['y']), z=0.45)
        beacon.pose.orientation.w = 1.0
        beacon.scale.x = beacon.scale.y = 0.45
        beacon.scale.z = 0.9
        beacon.color.r, beacon.color.g, beacon.color.b, beacon.color.a = 1.0, 0.05, 0.05, 0.65
        markers.markers.append(beacon)
        label = Marker()
        label.header = beacon.header
        label.ns = 'aed_emergency'
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=float(target['x']), y=float(target['y']), z=1.15)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.3
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'急救模式: {target["name"]}\n{self._emergency_type} / {self._state}'
        markers.markers.append(label)
        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AedEmergencyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
