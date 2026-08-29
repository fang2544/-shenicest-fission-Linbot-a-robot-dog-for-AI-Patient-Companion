#!/usr/bin/env python3
"""唤醒欢迎节点的 ROS 通信链路集成测试。"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from peripheral_msgs.srv import GetSupportedEmotions, PlayEmotion
from speech_msgs.msg import ChatTtsNotification, WakeupInfo
from speech_msgs.srv import SpeechTextData


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("wakeup_welcome_integration_harness")
        self.emotion_requests = []
        self.tts_requests = []
        self.wakeup_pub = self.create_publisher(WakeupInfo, "/speech/WakeupInfo", 10)
        self.notification_pub = self.create_publisher(
            ChatTtsNotification, "/speech/ChatTtsNotification", 10)

    def start_services(self) -> None:
        self.create_service(PlayEmotion, "/peripheral/PlayEmotion", self._emotion)
        self.create_service(GetSupportedEmotions,
                            "/peripheral/GetSupportedEmotions", self._emotions)
        self.create_service(SpeechTextData, "/speech/SpeechTextData", self._tts)

    def publish_wakeup(self) -> None:
        msg = WakeupInfo()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.wakeup_source = WakeupInfo.WAKEUP_SOURCE_DOG
        msg.doa_angle = 30
        msg.wakeup_word = "大头大头"
        self.wakeup_pub.publish(msg)

    def _emotion(self, request, response):
        self.emotion_requests.append(request)
        response.success = True
        response.message = "ok"
        return response

    @staticmethod
    def _emotions(request, response):
        response.success = True
        response.message = "ok"
        response.supported_emotions = ["1:welcome"]
        return response

    def _tts(self, request, response):
        self.tts_requests.append(request)
        response.success = True
        response.message = "ok"
        notification = ChatTtsNotification()
        notification.notification_type = ChatTtsNotification.TTS_START_PLAYBACK
        self.notification_pub.publish(notification)
        return response


def main() -> None:
    rclpy.init()
    node = Harness()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    time.sleep(1.0)

    # 先发布唤醒消息，再启动服务，验证服务发现后的重试逻辑。
    node.publish_wakeup()
    time.sleep(0.5)
    node.start_services()
    time.sleep(1.0)
    node.publish_wakeup()  # 冷却时间内的重复唤醒必须被忽略。
    time.sleep(0.4)

    assert len(node.emotion_requests) == 1
    assert len(node.tts_requests) == 1
    request = node.tts_requests[0]
    assert request.data_type == SpeechTextData.Request.TYPE_TTS
    assert request.text == "你好，我是你的陪诊伙伴聆灵，接下来我将全程陪伴你。"
    print("INTEGRATION_PASS: delayed services, emotion, TTS, notification, cooldown")

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
