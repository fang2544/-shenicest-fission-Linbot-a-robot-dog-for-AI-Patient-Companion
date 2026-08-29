#!/usr/bin/env python3
"""走失暂停提醒节点的 ROS 端到端集成测试。"""

import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from function_msgs.action import GoalNav
from function_msgs.msg import FollowStatus
from function_msgs.srv import ControlFollowing
from lowlevel_msg.msg import UwbState, WirelessController
from peripheral_msgs.srv import GetSupportedEmotions, PlayEmotion
from speech_msgs.msg import ChatTtsNotification
from speech_msgs.srv import SpeechTextData


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("lost_pause_reminder_harness")
        self.control_requests = []
        self.tts_requests = []
        self.emotion_requests = []
        self.goal_requests = []
        self.uwb_pub = self.create_publisher(UwbState, "/lowlevel/UwbState", 10)
        self.wireless_pub = self.create_publisher(
            WirelessController, "/lowlevel/WirelessController", 10)
        self.follow_pub = self.create_publisher(
            FollowStatus, "/function/FollowStatus", 10)
        self.tts_pub = self.create_publisher(
            ChatTtsNotification, "/speech/ChatTtsNotification", 10)
        self._servers = []
        self._action_server = None

    def start_servers(self) -> None:
        self._servers = [
            self.create_service(ControlFollowing, "/function/ControlFollowing",
                                self._control),
            self.create_service(SpeechTextData, "/speech/SpeechTextData", self._tts),
            self.create_service(PlayEmotion, "/peripheral/PlayEmotion", self._emotion),
            self.create_service(GetSupportedEmotions,
                                "/peripheral/GetSupportedEmotions", self._emotions),
        ]
        self._action_server = ActionServer(
            self, GoalNav, "/function/GoalNav", execute_callback=self._goal)

    def publish_following(self) -> None:
        msg = FollowStatus()
        msg.status = FollowStatus.STATUS_RUNNING
        msg.mode = FollowStatus.FOLLOWING
        self.follow_pub.publish(msg)

    def publish_uwb(self, distance: float, buttons: int = 0) -> None:
        msg = UwbState()
        msg.distance_est = distance
        msg.yaw_est = 0.5
        msg.buttons = buttons
        self.uwb_pub.publish(msg)

    def publish_wireless(self, keys: int) -> None:
        msg = WirelessController()
        msg.keys = keys
        self.wireless_pub.publish(msg)

    def _control(self, request, response):
        self.control_requests.append(request)
        response.success = True
        response.message = "ok"
        return response

    def _tts(self, request, response):
        self.tts_requests.append(request)
        response.success = True
        response.message = "ok"
        event = ChatTtsNotification()
        event.notification_type = ChatTtsNotification.TTS_START_PLAYBACK
        self.tts_pub.publish(event)
        return response

    def _emotion(self, request, response):
        self.emotion_requests.append(request)
        response.success = True
        response.message = "ok"
        return response

    @staticmethod
    def _emotions(request, response):
        response.success = True
        response.message = "ok"
        response.supported_emotions = ["2:reminder"]
        return response

    def _goal(self, goal_handle):
        self.goal_requests.append(goal_handle.request)
        goal_handle.succeed()
        result = GoalNav.Result()
        result.success = True
        result.message = "stopped"
        return result


def main() -> None:
    rclpy.init()
    node = Harness()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if (node.follow_pub.get_subscription_count() > 0 and
                node.uwb_pub.get_subscription_count() > 0):
            break
        time.sleep(0.1)
    assert node.follow_pub.get_subscription_count() > 0
    assert node.uwb_pub.get_subscription_count() > 0

    for _ in range(3):
        node.publish_following()
        time.sleep(0.1)
    node.publish_uwb(4.0)
    time.sleep(0.85)
    node.publish_uwb(4.0)  # 服务启动前触发经过防抖的距离暂停。
    time.sleep(0.3)
    node.start_servers()
    time.sleep(1.0)

    assert [r.mode for r in node.control_requests] == [ControlFollowing.Request.STOP]
    assert len(node.goal_requests) == 1 and node.goal_requests[0].control is False
    assert len(node.tts_requests) == 1
    assert node.tts_requests[0].text == "我是聆灵，我们好像走散了，请回到我身边"
    assert len(node.emotion_requests) == 1

    # 无效距离不能恢复跟随；连续低于回差阈值的有效距离才允许恢复。
    node.publish_uwb(float("nan"))
    time.sleep(0.2)
    assert len(node.control_requests) == 1
    node.publish_uwb(2.0)
    time.sleep(0.3)
    node.publish_uwb(2.0)
    time.sleep(0.5)
    assert [r.mode for r in node.control_requests] == [0, FollowStatus.FOLLOWING]

    # 同一次实体按键即使从两个话题上报，也只能切换一次状态。
    node.publish_wireless(1)
    node.publish_uwb(2.0, buttons=1)
    time.sleep(0.5)
    assert [r.mode for r in node.control_requests] == [0, 1, 0]
    node.publish_wireless(0)
    node.publish_uwb(2.0, buttons=0)
    time.sleep(0.35)
    node.publish_wireless(1)
    time.sleep(0.5)
    assert [r.mode for r in node.control_requests] == [0, 1, 0, 1]

    print("INTEGRATION_PASS: debounce, delayed services, pause/resume, "
          "invalid distance, action stop, button dedup")
    if self_action := node._action_server:
        self_action.destroy()
    executor.shutdown()
    thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
