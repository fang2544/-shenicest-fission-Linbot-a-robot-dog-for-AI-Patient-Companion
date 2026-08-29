#!/usr/bin/env python3
"""根据 UWB 距离判断跟丢，暂停运动并提醒用户。"""

import json
import math
import time
import uuid
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from function_msgs.action import GoalNav
from function_msgs.msg import FollowStatus
from function_msgs.srv import ControlFollowing
from lowlevel_msg.msg import UwbState, WirelessController
from peripheral_msgs.srv import GetSupportedEmotions, PlayEmotion
from speech_msgs.msg import ChatTtsNotification
from speech_msgs.srv import SpeechTextData
from std_msgs.msg import String


DEFAULT_REMINDER_TEXT = "我是聆灵，我们好像走散了，请回到我身边"
TARGET_STATE_ON = 1
DEFAULT_ACTIVE_FOLLOW_MODES = [
    FollowStatus.FOLLOWING,
    FollowStatus.NAV_TO_POINT,
    FollowStatus.ACCOMPANY,
]


class LostPauseReminderNode(Node):
    """检测跟丢状态，下发安全暂停并播放提醒。"""

    def __init__(self) -> None:
        super().__init__("lost_pause_reminder_node")
        self._declare_parameters()

        self._exceed_since: Optional[Time] = None
        self._recovery_since: Optional[Time] = None
        self._is_paused = False
        self._pause_reason: Optional[str] = None
        self._pause_trigger_time: Optional[Time] = None
        self._mode_before_pause = int(self.get_parameter("resume_mode_fallback").value)
        self._last_follow_mode: Optional[int] = None
        self._last_follow_status: Optional[int] = None
        self._last_known_yaw: Optional[float] = None
        self._wireless_button_pressed = False
        self._uwb_button_pressed = False
        self._last_button_event = float("-inf")

        # 跟随控制属于安全指令；重试队列只保留最新请求。
        self._control_generation = 0
        self._pending_control: Optional[Tuple[int, int, str, float]] = None
        self._control_future = None
        self._last_control_wait_log = float("-inf")

        self._reminder_deadline: Optional[float] = None
        self._tts_future = None
        self._tts_completed = False
        self._emotion_requested = False
        self._tts_completed = False
        self._goal_cancel_pending = False
        self._goal_cancel_deadline: Optional[float] = None
        self._goal_send_future = None
        self._supported_emotions_requested = False

        self._event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10)

        sensor_qos = self._make_qos("sensor_qos_reliability")
        status_qos = self._make_qos("status_qos_reliability")
        # 使用独立变量保存订阅句柄，避免覆盖节点基类的内部属性。
        self.create_subscription(UwbState, self.get_parameter("uwb_state_topic").value,
                                 self._on_uwb_state, sensor_qos)
        self.create_subscription(FollowStatus,
                                 self.get_parameter("follow_status_topic").value,
                                 self._on_follow_status, status_qos)
        self.create_subscription(ChatTtsNotification,
                                 self.get_parameter("tts_notification_topic").value,
                                 self._on_tts_notification, status_qos)
        if self.get_parameter("enable_button_trigger").value:
            self.create_subscription(
                WirelessController,
                self.get_parameter("wireless_controller_topic").value,
                self._on_wireless_controller,
                sensor_qos,
            )

        self._control_client = self.create_client(
            ControlFollowing, self.get_parameter("control_following_service").value)
        self._tts_client = self.create_client(
            SpeechTextData, self.get_parameter("speech_text_data_service").value)
        self._emotion_client = self.create_client(
            PlayEmotion, self.get_parameter("play_emotion_service").value)
        self._emotions_client = self.create_client(
            GetSupportedEmotions,
            self.get_parameter("get_supported_emotions_service").value)
        self._goal_client = ActionClient(
            self, GoalNav, self.get_parameter("goal_nav_action").value)

        interval = max(0.05, float(self.get_parameter("retry_interval_sec").value))
        self._maintenance_timer = self.create_timer(interval, self._maintenance)
        self.get_logger().info("lost_pause_reminder_node 已启动，等待 UwbState/按键 ...")

    def _declare_parameters(self) -> None:
        defaults = {
            "uwb_state_topic": "/lowlevel/UwbState",
            "wireless_controller_topic": "/lowlevel/WirelessController",
            "follow_status_topic": "/function/FollowStatus",
            "tts_notification_topic": "/speech/ChatTtsNotification",
            "speech_text_data_service": "/speech/SpeechTextData",
            "play_emotion_service": "/peripheral/PlayEmotion",
            "get_supported_emotions_service": "/peripheral/GetSupportedEmotions",
            "control_following_service": "/function/ControlFollowing",
            "goal_nav_action": "/function/GoalNav",
            "distance_threshold_m": 3.0,
            "resume_distance_threshold_m": 2.8,
            "debounce_duration_sec": 0.75,
            "resume_debounce_duration_sec": 0.25,
            "enable_button_trigger": True,
            "wireless_controller_button_mask": 1,
            "uwb_buttons_mask": 1,
            "button_event_guard_sec": 0.3,
            "button_can_resume_distance_pause": False,
            "reminder_text": DEFAULT_REMINDER_TEXT,
            "reminder_emotion_mode": 2,
            "reminder_emotion_duration_ms": -1,
            "reminder_service_timeout_sec": 2.0,
            "goal_cancel_timeout_sec": 2.0,
            "retry_interval_sec": 0.1,
            "active_follow_modes": DEFAULT_ACTIVE_FOLLOW_MODES,
            "resume_mode_fallback": FollowStatus.FOLLOWING,
            "topic_qos_depth": 10,
            "sensor_qos_reliability": "reliable",
            "status_qos_reliability": "reliable",
            "event_topic": "/second_dev/events",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _make_qos(self, parameter: str) -> QoSProfile:
        depth = max(1, int(self.get_parameter("topic_qos_depth").value))
        value = str(self.get_parameter(parameter).value).strip().lower()
        if value == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            if value != "reliable":
                self.get_logger().warn(f"参数 {parameter}={value!r} 无效，使用 reliable")
            reliability = ReliabilityPolicy.RELIABLE
        return QoSProfile(depth=depth, reliability=reliability)

    def _on_follow_status(self, msg: FollowStatus) -> None:
        self._last_follow_mode = msg.mode
        self._last_follow_status = msg.status

    def _on_uwb_state(self, msg: UwbState) -> None:
        if math.isfinite(msg.yaw_est):
            self._last_known_yaw = msg.yaw_est
        self._evaluate_distance(msg.distance_est)
        if self.get_parameter("enable_button_trigger").value:
            mask = max(0, int(self.get_parameter("uwb_buttons_mask").value))
            pressed = bool(msg.buttons & mask) if mask else False
            if pressed and not self._uwb_button_pressed:
                self._handle_button_edge("UwbState.buttons", msg.buttons, mask)
            self._uwb_button_pressed = pressed

    def _on_wireless_controller(self, msg: WirelessController) -> None:
        mask = max(0, int(self.get_parameter("wireless_controller_button_mask").value))
        pressed = bool(msg.keys & mask) if mask else False
        if pressed and not self._wireless_button_pressed:
            self._handle_button_edge("WirelessController.keys", msg.keys, mask)
        self._wireless_button_pressed = pressed

    def _handle_button_edge(self, source: str, value: int, mask: int) -> None:
        now = time.monotonic()
        guard = max(0.0, float(self.get_parameter("button_event_guard_sec").value))
        if now - self._last_button_event < guard:
            self.get_logger().debug(f"忽略跨来源重复按键: {source}")
            return
        self._last_button_event = now
        self.get_logger().info(f"{source} 触发(mask={mask}, value={value})")
        if self._is_paused:
            if (self._pause_reason == "distance" and
                    not self.get_parameter("button_can_resume_distance_pause").value):
                self.get_logger().warn("当前为距离跟丢暂停，距离恢复前不允许按钮强制恢复")
                return
            self._trigger_resume()
        else:
            self._trigger_pause("button")

    def _evaluate_distance(self, distance: float) -> None:
        if not math.isfinite(distance) or distance < 0.0:
            self.get_logger().warn(f"忽略无效 UWB 距离: {distance!r}")
            return
        now = self.get_clock().now()
        pause_threshold = max(0.0, float(self.get_parameter("distance_threshold_m").value))
        resume_threshold = max(
            0.0, float(self.get_parameter("resume_distance_threshold_m").value))
        if resume_threshold > pause_threshold:
            self.get_logger().warn("恢复阈值大于暂停阈值，按暂停阈值处理")
            resume_threshold = pause_threshold

        if not self._is_paused:
            self._recovery_since = None
            if distance <= pause_threshold:
                self._exceed_since = None
                return
            if self._exceed_since is None:
                self._exceed_since = now
                return
            elapsed = (now - self._exceed_since).nanoseconds / 1e9
            debounce = max(0.0, float(self.get_parameter("debounce_duration_sec").value))
            if elapsed >= debounce and self._is_active_follow_context():
                self.get_logger().warn(
                    f"距离 {distance:.2f}m 超过 {pause_threshold:.2f}m "
                    f"持续 {elapsed:.2f}s，判定跟丢")
                self._trigger_pause("distance")
            return

        if self._pause_reason != "distance":
            return
        if distance > resume_threshold:
            self._recovery_since = None
            return
        if self._recovery_since is None:
            self._recovery_since = now
            return
        elapsed = (now - self._recovery_since).nanoseconds / 1e9
        debounce = max(
            0.0, float(self.get_parameter("resume_debounce_duration_sec").value))
        if elapsed >= debounce:
            self.get_logger().info(
                f"距离回到 {distance:.2f}m，稳定 {elapsed:.2f}s，恢复跟随")
            self._trigger_resume()

    def _is_active_follow_context(self) -> bool:
        active = {int(mode) for mode in self.get_parameter("active_follow_modes").value}
        return (self._last_follow_mode in active and
                self._last_follow_status == FollowStatus.STATUS_RUNNING)

    def _trigger_pause(self, reason: str) -> None:
        if self._is_paused:
            return
        self._is_paused = True
        self._pause_reason = reason
        self._pause_trigger_time = self.get_clock().now()
        self._recovery_since = None
        active = {int(mode) for mode in self.get_parameter("active_follow_modes").value}
        if self._last_follow_mode in active:
            self._mode_before_pause = int(self._last_follow_mode)
        else:
            self._mode_before_pause = int(self.get_parameter("resume_mode_fallback").value)
        self._queue_control(ControlFollowing.Request.STOP, "暂停")
        self._emit_event("LOST_PAUSE", reason=reason, mode=self._mode_before_pause)
        self._queue_goal_cancel()
        self._log_turn_todo()
        self._start_reminder()

    def _trigger_resume(self) -> None:
        if not self._is_paused:
            return
        self._is_paused = False
        self._pause_reason = None
        self._exceed_since = None
        self._recovery_since = None
        self._goal_cancel_pending = False
        self._goal_cancel_deadline = None
        self._queue_control(self._mode_before_pause, "恢复")
        self._emit_event("LOST_RESUME", mode=self._mode_before_pause)

    def _emit_event(self, event: str, **details) -> None:
        payload = {"source": "lost_pause_reminder", "event": event, **details}
        self._event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _queue_control(self, mode: int, label: str) -> None:
        self._control_generation += 1
        self._pending_control = (int(mode), self._control_generation, label, time.monotonic())
        self._try_send_control()

    def _try_send_control(self) -> None:
        if self._pending_control is None or self._control_future is not None:
            return
        if not self._control_client.service_is_ready():
            now = time.monotonic()
            if now - self._last_control_wait_log >= 2.0:
                self.get_logger().warn("ControlFollowing 服务未就绪，保留指令并继续重试")
                self._last_control_wait_log = now
            return
        mode, generation, label, queued_at = self._pending_control
        req = ControlFollowing.Request()
        req.target_state = TARGET_STATE_ON
        req.mode = mode
        req.req_id = str(uuid.uuid4())
        req.pre_check = False
        # 显式初始化新版本接口新增字段，保证不同固件版本行为一致。
        req.x = 0.0
        req.y = 0.0
        req.yaw = 0.0
        req.goal_frame = ""
        req.stop_distance = 0.0
        req.max_xvel = 0.0
        req.prompt = ""
        self._control_future = self._control_client.call_async(req)
        self._control_future.add_done_callback(
            lambda future: self._on_control_response(
                future, generation, label, queued_at))

    def _on_control_response(self, future, generation: int,
                             label: str, queued_at: float) -> None:
        self._control_future = None
        if generation != self._control_generation:
            self._try_send_control()
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"ControlFollowing {label}调用失败，将重试: {exc}")
            return
        if not response.success:
            self.get_logger().error(
                f"ControlFollowing {label}返回失败，将重试: {response.message}")
            return
        mode = self._pending_control[0] if self._pending_control is not None else -1
        self._pending_control = None
        self.get_logger().info(
            f"ControlFollowing {label}成功，排队耗时 {time.monotonic() - queued_at:.2f}s")
        self._emit_event("FOLLOW_CONTROL_ACCEPTED", label=label, mode=mode)

    def _queue_goal_cancel(self) -> None:
        timeout = max(0.1, float(self.get_parameter("goal_cancel_timeout_sec").value))
        self._goal_cancel_pending = True
        self._goal_cancel_deadline = time.monotonic() + timeout
        self._try_cancel_goal()

    def _try_cancel_goal(self) -> None:
        if not self._goal_cancel_pending or self._goal_send_future is not None:
            return
        if not self._goal_client.server_is_ready():
            return
        goal = GoalNav.Goal()
        goal.control = False
        goal.x = goal.y = goal.z = goal.yaw = 0.0
        self._goal_send_future = self._goal_client.send_goal_async(goal)
        self._goal_send_future.add_done_callback(self._on_goal_cancel_sent)

    def _on_goal_cancel_sent(self, future) -> None:
        self._goal_send_future = None
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"GoalNav 停止请求发送失败: {exc}")
            return
        if not handle.accepted:
            self.get_logger().warn("GoalNav 停止请求被拒绝")
            return
        self._goal_cancel_pending = False
        handle.get_result_async().add_done_callback(self._on_goal_cancel_result)

    def _on_goal_cancel_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"GoalNav 停止结果读取失败: {exc}")
            return
        if result.success:
            self.get_logger().info("导航目标已停止")
        else:
            self.get_logger().warn(f"停止导航目标失败: {result.message}")

    def _log_turn_todo(self) -> None:
        if self._last_known_yaw is None:
            self.get_logger().warn("未收到有效 yaw，且原地转向接口未确认，不下发转向")
        else:
            self.get_logger().warn(
                f"原地转向接口未确认，不下发转向；最后 yaw={self._last_known_yaw:.3f}rad")

    def _start_reminder(self) -> None:
        timeout = max(
            0.1, float(self.get_parameter("reminder_service_timeout_sec").value))
        self._reminder_deadline = time.monotonic() + timeout
        self._emotion_requested = False
        self._try_send_reminder()

    def _try_send_reminder(self) -> None:
        if self._reminder_deadline is None:
            return
        if not self._emotion_requested and self._emotion_client.service_is_ready():
            mode = int(self.get_parameter("reminder_emotion_mode").value)
            duration = int(self.get_parameter("reminder_emotion_duration_ms").value)
            self._emotion_requested = True
            if 0 <= mode <= 255 and -(2**31) <= duration < 2**31:
                req = PlayEmotion.Request()
                req.target_state = TARGET_STATE_ON
                req.req_id = str(uuid.uuid4())
                req.pre_check = False
                req.mode = mode
                req.duration_ms = duration
                self._emotion_client.call_async(req).add_done_callback(
                    self._on_emotion_response)
            else:
                self.get_logger().error("提醒表情参数超出接口数值范围")
        if (
            self._tts_completed
            or self._tts_future is not None
            or not self._tts_client.service_is_ready()
        ):
            return
        req = SpeechTextData.Request()
        req.data_type = SpeechTextData.Request.TYPE_TTS
        req.text = str(self.get_parameter("reminder_text").value)
        req.sn = str(uuid.uuid4())
        req.timestamp = self.get_clock().now().to_msg()
        self._tts_future = self._tts_client.call_async(req)
        self._tts_future.add_done_callback(self._on_tts_response)

    def _on_emotion_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"PlayEmotion 调用失败: {exc}")
            return
        if not response.success:
            self.get_logger().warn(f"PlayEmotion 返回失败: {response.message}")
        if self._tts_completed:
            self._reminder_deadline = None

    def _on_tts_response(self, future) -> None:
        self._tts_future = None
        if future.cancelled():
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"SpeechTextData 调用失败，将在超时前重试: {exc}")
            return
        if response.success:
            self._tts_completed = True
            if self._emotion_requested:
                self._reminder_deadline = None
            self.get_logger().info("提醒语已下发给 TTS")
            self._emit_event(
                "LOST_REMINDER_TTS",
                text=str(self.get_parameter("reminder_text").value),
            )
        else:
            self.get_logger().warn(f"提醒语下发失败，将在超时前重试: {response.message}")

    def _on_tts_notification(self, msg: ChatTtsNotification) -> None:
        names = {
            ChatTtsNotification.TTS_START_PLAYBACK: "开始播放",
            ChatTtsNotification.TTS_COMPLETED: "播放完成",
            ChatTtsNotification.TTS_INTERRUPTED: "播放被打断",
        }
        self.get_logger().info(f"TTS {names.get(msg.notification_type, '未知通知')}")

    def _maintenance(self) -> None:
        self._try_send_control()
        self._request_supported_emotions_once()
        now = time.monotonic()
        if self._goal_cancel_pending:
            if self._goal_cancel_deadline is not None and now >= self._goal_cancel_deadline:
                self.get_logger().error("GoalNav action server 等待超时，未能停止导航目标")
                self._goal_cancel_pending = False
            else:
                self._try_cancel_goal()
        if self._reminder_deadline is not None:
            if now >= self._reminder_deadline:
                if self._tts_future is not None:
                    self._tts_future.cancel()
                    self._tts_future = None
                self._reminder_deadline = None
                if self._tts_completed:
                    self.get_logger().warn("提醒语已播放，但表情服务在期限内不可用")
                else:
                    self.get_logger().error("提醒服务等待/调用超时")
            else:
                self._try_send_reminder()

    def _request_supported_emotions_once(self) -> None:
        if self._supported_emotions_requested or not self._emotions_client.service_is_ready():
            return
        self._supported_emotions_requested = True
        self._emotions_client.call_async(GetSupportedEmotions.Request()).add_done_callback(
            self._on_supported_emotions)

    def _on_supported_emotions(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"GetSupportedEmotions 调用失败: {exc}")
            return
        if response.success:
            self.get_logger().info(f"支持的表情列表: {list(response.supported_emotions)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LostPauseReminderNode()
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
