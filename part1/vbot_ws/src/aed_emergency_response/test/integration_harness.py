#!/usr/bin/env python3
"""验证意图识别、任务抢占、导航和语音播报的黑盒集成测试。"""

import json
import sys
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from function_msgs.action import GoalNav
from function_msgs.msg import AsrResult
from speech_msgs.srv import SpeechTextData


class Harness(Node):
    def __init__(self):
        super().__init__('aed_emergency_integration_harness')
        self.events = []
        self.goals = []
        self.speech = []
        self.pub = self.create_publisher(AsrResult, '/asr/result', 10)
        self.create_subscription(String, '/aed_emergency/status', self._status, 20)
        self.create_service(SpeechTextData, '/speech/SpeechTextData', self._tts)
        self.action = ActionServer(self, GoalNav, '/goal_nav', self._goal)

    def _status(self, message):
        self.events.append(json.loads(message.data))

    def _tts(self, request, response):
        self.speech.append(request.text)
        response.success = True
        response.message = 'mock TTS completed'
        return response

    def _goal(self, handle):
        goal = handle.request
        self.goals.append((bool(goal.control), float(goal.x), float(goal.y)))
        feedback = GoalNav.Feedback()
        feedback.progress = [100]
        feedback.message = 'mock navigation complete'
        handle.publish_feedback(feedback)
        handle.succeed()
        result = GoalNav.Result()
        result.success = True
        result.message = 'mock success'
        return result

    def say(self, text, confidence=0.99):
        message = AsrResult()
        message.header.stamp = self.get_clock().now().to_msg()
        message.transcript = text
        message.source_type = AsrResult.DOG
        message.reject = False
        message.confidence = confidence
        self.pub.publish(message)


def wait_until(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def main():
    rclpy.init()
    harness = Harness()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(harness)
    executor_thread = __import__('threading').Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        assert wait_until(lambda: harness.pub.get_subscription_count() > 0), 'node unavailable'
        harness.say('AED在哪里')
        time.sleep(0.4)
        harness.say('只是演练，不需要AED')
        time.sleep(0.4)
        assert not harness.goals, 'non-emergency speech triggered navigation'

        harness.say('心电图室有人心脏骤停，马上送AED过来')
        assert wait_until(
            lambda: any(e.get('event') == 'COMPLETED' for e in harness.events), 15.0
        ), 'emergency workflow did not complete'

        harness.say('抽血窗口有人严重出血，马上送急救箱过来')
        assert wait_until(
            lambda: sum(
                e.get('event') == 'COMPLETED' for e in harness.events
            ) >= 2,
            15.0,
        ), 'bleeding emergency workflow did not complete'

        names = {e.get('event') for e in harness.events}
        required = {
            'INTENT_ACCEPTED', 'PREEMPT_COMPLETED', 'NAV_GOAL_SENT',
            'NAVIGATION_ACCEPTED', 'ARRIVED', 'GUIDANCE_STEP', 'COMPLETED',
        }
        assert required <= names, f'missing events: {required - names}'
        assert len(harness.goals) == 4, f'expected two stop + move pairs, got {harness.goals}'
        assert harness.goals[0][0] is False
        assert harness.goals[1][0] is True
        assert abs(harness.goals[1][1] - (-0.9)) < 0.001
        assert harness.goals[2][0] is False
        assert harness.goals[3][0] is True
        assert abs(harness.goals[3][1] - (-2.6)) < 0.001
        assert len(harness.speech) == 15, f'expected 15 TTS requests, got {len(harness.speech)}'
        all_speech = ''.join(harness.speech)
        for token in ('120', '100到120', '不要接触', 'AED', '止血敷料', '不要揭掉'):
            assert token in all_speech, f'missing guidance token: {token}'
        accepted = [e for e in harness.events if e.get('event') == 'INTENT_ACCEPTED']
        assert [e.get('emergency_type') for e in accepted] == ['cardiac', 'bleeding']
        print(json.dumps({
            'success': True,
            'goals': harness.goals,
            'tts_count': len(harness.speech),
            'event_count': len(harness.events),
        }, ensure_ascii=False))
        return 0
    except AssertionError as exc:
        print(f'AED integration FAILED: {exc}', file=sys.stderr)
        return 1
    finally:
        executor.shutdown()
        harness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
