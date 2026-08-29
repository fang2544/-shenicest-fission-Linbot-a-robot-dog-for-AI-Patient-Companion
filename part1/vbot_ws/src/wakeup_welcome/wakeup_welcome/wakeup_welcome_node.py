#!/usr/bin/env python3
"""接收 VBot 唤醒事件并播放欢迎表情和语音。"""

import json
import time
import uuid
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from peripheral_msgs.srv import GetSupportedEmotions, PlayEmotion
from speech_msgs.msg import ChatTtsNotification, CommandWord, WakeupInfo
from speech_msgs.srv import SpeechTextData
from std_msgs.msg import String


DEFAULT_WELCOME_TEXT = "你好，我是你的陪诊伙伴聆灵，接下来我将全程陪伴你。"
EMOTION_TARGET_STATE_ON = 1


class WakeupWelcomeNode(Node):
    """收到唤醒事件后执行可配置的欢迎流程。"""

    def __init__(self) -> None:
        super().__init__("wakeup_welcome_node")

        self.declare_parameter("wakeup_topic", "/speech/WakeupInfo")
        self.declare_parameter("tts_notification_topic", "/speech/ChatTtsNotification")
        self.declare_parameter("speech_text_data_service", "/speech/SpeechTextData")
        self.declare_parameter("play_emotion_service", "/peripheral/PlayEmotion")
        self.declare_parameter("get_supported_emotions_service", "/peripheral/GetSupportedEmotions")
        self.declare_parameter("welcome_text", DEFAULT_WELCOME_TEXT)
        self.declare_parameter("welcome_emotion_mode", 1)
        self.declare_parameter("welcome_emotion_duration_ms", -1)
        self.declare_parameter("rewelcome_cooldown_sec", 5.0)
        self.declare_parameter("service_ready_timeout_sec", 2.0)
        self.declare_parameter("service_retry_interval_sec", 0.1)
        self.declare_parameter("tts_notification_timeout_sec", 15.0)
        self.declare_parameter("enable_command_word_trigger", False)
        self.declare_parameter("command_word_topic", "/speech/CommandWord")
        self.declare_parameter("command_word_trigger_text", "你好聆灵")
        self.declare_parameter("topic_qos_depth", 10)
        self.declare_parameter("wakeup_qos_reliability", "reliable")
        self.declare_parameter("tts_notification_qos_reliability", "reliable")
        self.declare_parameter("event_topic", "/second_dev/events")

        retry_interval = max(0.05, float(self.get_parameter("service_retry_interval_sec").value))
        self._last_successful_trigger: Optional[float] = None
        self._trigger_stamp: Optional[Time] = None
        self._dispatch_deadline: Optional[float] = None
        self._notification_deadline: Optional[float] = None
        self._tts_future = None
        self._emotion_requested = False
        self._playback_started_for_request = False
        self._request_generation = 0
        self._supported_emotions_requested = False
        self._event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10)

        wakeup_qos = self._make_qos("wakeup_qos_reliability")
        notification_qos = self._make_qos("tts_notification_qos_reliability")
        self._subscriptions = [
            self.create_subscription(WakeupInfo, self.get_parameter("wakeup_topic").value,
                                     self._on_wakeup_info, wakeup_qos),
            self.create_subscription(ChatTtsNotification,
                                     self.get_parameter("tts_notification_topic").value,
                                     self._on_tts_notification, notification_qos),
        ]
        if self.get_parameter("enable_command_word_trigger").value:
            self._subscriptions.append(
                self.create_subscription(CommandWord,
                                         self.get_parameter("command_word_topic").value,
                                         self._on_command_word, wakeup_qos)
            )

        self._tts_client = self.create_client(
            SpeechTextData, self.get_parameter("speech_text_data_service").value)
        self._emotion_client = self.create_client(
            PlayEmotion, self.get_parameter("play_emotion_service").value)
        self._get_emotions_client = self.create_client(
            GetSupportedEmotions,
            self.get_parameter("get_supported_emotions_service").value)
        self._maintenance_timer = self.create_timer(retry_interval, self._maintenance)
        self.get_logger().info("wakeup_welcome_node 已启动，等待 WakeupInfo ...")

    def _make_qos(self, reliability_parameter: str) -> QoSProfile:
        depth = max(1, int(self.get_parameter("topic_qos_depth").value))
        value = str(self.get_parameter(reliability_parameter).value).strip().lower()
        if value == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            if value != "reliable":
                self.get_logger().warn(f"参数 {reliability_parameter}={value!r} 无效，使用 reliable")
            reliability = ReliabilityPolicy.RELIABLE
        return QoSProfile(depth=depth, reliability=reliability)

    def _on_wakeup_info(self, msg: WakeupInfo) -> None:
        sources = {WakeupInfo.WAKEUP_SOURCE_DOG: "本体(DOG)",
                   WakeupInfo.WAKEUP_SOURCE_PHONE: "手机(PHONE)"}
        source = sources.get(msg.wakeup_source, f"未知({msg.wakeup_source})")
        self.get_logger().info(
            f"收到唤醒事件: source={source}, doa_angle={msg.doa_angle}, "
            f"wakeup_word={msg.wakeup_word!r}")
        self._emit_event(
            "WAKEUP_DETECTED",
            wakeup_source=source,
            doa_angle=int(msg.doa_angle),
            wakeup_word=msg.wakeup_word,
        )
        self._trigger_welcome()

    def _emit_event(self, event: str, **details) -> None:
        payload = {"source": "wakeup_welcome", "event": event, **details}
        self._event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _on_command_word(self, msg: CommandWord) -> None:
        target = str(self.get_parameter("command_word_trigger_text").value)
        if msg.command_text == target:
            self.get_logger().info(f"收到命令词唤醒: {msg.command_text!r}")
            self._trigger_welcome()

    def _trigger_welcome(self) -> None:
        now = time.monotonic()
        cooldown = max(0.0, float(self.get_parameter("rewelcome_cooldown_sec").value))
        if self._dispatch_deadline is not None or self._tts_future is not None:
            self.get_logger().debug("欢迎语正在等待或下发中，忽略重复触发")
            return
        if self._last_successful_trigger is not None:
            elapsed = now - self._last_successful_trigger
            if elapsed < cooldown:
                self.get_logger().debug(
                    f"冷却中({elapsed:.2f}s < {cooldown:.2f}s)，忽略本次触发")
                return

        timeout = max(0.1, float(self.get_parameter("service_ready_timeout_sec").value))
        self._request_generation += 1
        self._trigger_stamp = self.get_clock().now()
        self._dispatch_deadline = now + timeout
        self._notification_deadline = None
        self._emotion_requested = False
        self._playback_started_for_request = False
        self._try_dispatch_welcome()

    def _maintenance(self) -> None:
        self._request_supported_emotions_once()
        now = time.monotonic()
        if self._notification_deadline is not None and now >= self._notification_deadline:
            self.get_logger().warn("等待欢迎语 TTS 播放通知超时")
            self._notification_deadline = None
            self._trigger_stamp = None
        if self._dispatch_deadline is None:
            return
        if now >= self._dispatch_deadline:
            # 使取消后延迟到达的回调失效，避免重复执行欢迎流程。
            self._request_generation += 1
            if self._tts_future is not None:
                self._tts_future.cancel()
                self._tts_future = None
            self.get_logger().error("等待或调用 SpeechTextData 服务超时，欢迎语未下发")
            self._dispatch_deadline = None
            self._trigger_stamp = None
            return
        self._try_dispatch_welcome()

    def _try_dispatch_welcome(self) -> None:
        if self._dispatch_deadline is None:
            return
        if not self._emotion_requested and self._emotion_client.service_is_ready():
            self._play_welcome_emotion()
        if self._tts_future is not None or not self._tts_client.service_is_ready():
            return
        req = SpeechTextData.Request()
        req.data_type = SpeechTextData.Request.TYPE_TTS
        req.text = str(self.get_parameter("welcome_text").value)
        req.sn = str(uuid.uuid4())
        req.timestamp = self.get_clock().now().to_msg()
        generation = self._request_generation
        self._tts_future = self._tts_client.call_async(req)
        self._tts_future.add_done_callback(
            lambda future: self._on_speak_welcome_response(future, generation))

    def _play_welcome_emotion(self) -> None:
        mode = int(self.get_parameter("welcome_emotion_mode").value)
        duration = int(self.get_parameter("welcome_emotion_duration_ms").value)
        self._emotion_requested = True
        if not 0 <= mode <= 255:
            self.get_logger().error(f"welcome_emotion_mode={mode} 超出 uint8 范围")
            return
        if not -(2**31) <= duration < 2**31:
            self.get_logger().error("welcome_emotion_duration_ms 超出 int32 范围")
            return
        req = PlayEmotion.Request()
        req.target_state = EMOTION_TARGET_STATE_ON
        req.req_id = str(uuid.uuid4())
        req.pre_check = False
        req.mode = mode
        req.duration_ms = duration
        future = self._emotion_client.call_async(req)
        future.add_done_callback(self._on_play_emotion_response)

    def _on_play_emotion_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"PlayEmotion 调用失败: {exc}")
            return
        if not resp.success:
            self.get_logger().warn(f"PlayEmotion 返回失败: {resp.message}")

    def _on_speak_welcome_response(self, future, generation: int) -> None:
        if generation != self._request_generation:
            return
        self._tts_future = None
        if future.cancelled():
            return
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"SpeechTextData 调用失败，将在超时前重试: {exc}")
            return
        if not resp.success:
            self.get_logger().warn(f"欢迎语下发失败，将在超时前重试: {resp.message}")
            return
        self._dispatch_deadline = None
        self._last_successful_trigger = time.monotonic()
        if not self._playback_started_for_request:
            timeout = max(
                0.1,
                float(self.get_parameter("tts_notification_timeout_sec").value),
            )
            self._notification_deadline = time.monotonic() + timeout
        self.get_logger().info("欢迎语已下发给 TTS")
        self._emit_event(
            "WELCOME_TTS",
            text=str(self.get_parameter("welcome_text").value),
        )

    def _on_tts_notification(self, msg: ChatTtsNotification) -> None:
        if msg.notification_type == ChatTtsNotification.TTS_START_PLAYBACK:
            if self._trigger_stamp is not None:
                self._playback_started_for_request = True
                latency = (self.get_clock().now() - self._trigger_stamp).nanoseconds / 1e9
                self.get_logger().info(f"TTS 开始播放，距唤醒约 {latency:.2f}s")
                self._trigger_stamp = None
                self._notification_deadline = None
            else:
                self.get_logger().info("TTS 开始播放")
        elif msg.notification_type == ChatTtsNotification.TTS_COMPLETED:
            self.get_logger().info("TTS 播放完成")
        elif msg.notification_type == ChatTtsNotification.TTS_INTERRUPTED:
            self.get_logger().info("TTS 播放被打断")
        else:
            self.get_logger().warn(f"未知 TTS 通知类型: {msg.notification_type}")

    def _request_supported_emotions_once(self) -> None:
        if self._supported_emotions_requested or not self._get_emotions_client.service_is_ready():
            return
        self._supported_emotions_requested = True
        future = self._get_emotions_client.call_async(GetSupportedEmotions.Request())
        future.add_done_callback(self._on_supported_emotions_response)

    def _on_supported_emotions_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"GetSupportedEmotions 调用失败: {exc}")
            return
        if resp.success:
            self.get_logger().info(f"支持的表情列表: {list(resp.supported_emotions)}")
        else:
            self.get_logger().warn(f"获取表情列表失败: {resp.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WakeupWelcomeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
