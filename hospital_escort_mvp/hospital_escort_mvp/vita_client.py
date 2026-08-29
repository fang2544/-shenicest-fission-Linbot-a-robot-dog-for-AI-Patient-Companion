import time
import uuid
from typing import Callable, Dict, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from function_msgs.action import GoalNav
from function_msgs.srv import ControlFollowing, SetSpeak


class VitaInterfaceClient(Node):
    """把 VITA 高层 ROS 2 接口封装为同步业务调用。"""

    def __init__(
        self,
        node_name: str = 'hospital_nav_adapter',
        parameter_overrides=None,
        external_spin: bool = False,
        use_global_arguments: bool = True,
    ) -> None:
        super().__init__(
            node_name,
            parameter_overrides=parameter_overrides,
            use_global_arguments=use_global_arguments,
        )
        self._external_spin = external_spin
        self._active_goal_handle = None
        self.declare_parameter('goal_nav_action', '/goal_nav')
        self.declare_parameter('following_service', '/control_following')
        self.declare_parameter('speak_service', '/set_speak')
        self.declare_parameter('interface_timeout_sec', 5.0)
        self.declare_parameter('navigation_timeout_sec', 120.0)

        self._goal_nav = ActionClient(
            self,
            GoalNav,
            self.get_parameter('goal_nav_action').value,
        )
        self._following = self.create_client(
            ControlFollowing,
            self.get_parameter('following_service').value,
        )
        self._speak = self.create_client(
            SetSpeak,
            self.get_parameter('speak_service').value,
        )

    @property
    def interface_timeout(self) -> float:
        return float(self.get_parameter('interface_timeout_sec').value)

    def wait_until_ready(self) -> Dict[str, bool]:
        timeout = self.interface_timeout
        return {
            'goal_nav': self._goal_nav.wait_for_server(timeout_sec=timeout),
            'control_following': self._following.wait_for_service(timeout_sec=timeout),
            'set_speak': self._speak.wait_for_service(timeout_sec=timeout),
        }

    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if self._external_spin:
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
            else:
                rclpy.spin_once(
                    self,
                    timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())),
                )
        return future.done()

    def speak(self, text: str, pre_check: bool = False) -> Tuple[bool, str]:
        if not self._speak.wait_for_service(timeout_sec=self.interface_timeout):
            return False, 'SetSpeak service unavailable'
        request = SetSpeak.Request()
        request.target_state = 1
        request.mode = SetSpeak.Request.HUMAN_VOICE
        request.req_id = str(uuid.uuid4())
        request.pre_check = pre_check
        request.human_language_text = text
        future = self._speak.call_async(request)
        if not self._wait_future(future, self.interface_timeout):
            return False, 'SetSpeak service timeout'
        response = future.result()
        return bool(response.success), response.message

    def set_follow_mode(
        self,
        mode: int,
        enabled: bool = True,
        stop_distance: float = 1.2,
        max_xvel: float = 0.3,
        pre_check: bool = False,
    ) -> Tuple[bool, str]:
        if not self._following.wait_for_service(timeout_sec=self.interface_timeout):
            return False, 'ControlFollowing service unavailable'
        request = ControlFollowing.Request()
        request.target_state = 1 if enabled else 0
        request.mode = int(mode)
        request.req_id = str(uuid.uuid4())
        request.pre_check = pre_check
        request.stop_distance = float(stop_distance)
        request.max_xvel = float(max_xvel)
        future = self._following.call_async(request)
        if not self._wait_future(future, self.interface_timeout):
            return False, 'ControlFollowing service timeout'
        response = future.result()
        return bool(response.success), response.message

    def stop_motion(self) -> Tuple[bool, str]:
        follow_ok, follow_message = self.set_follow_mode(
            ControlFollowing.Request.STOP, enabled=False)
        handle = self._active_goal_handle
        if handle is None:
            return follow_ok, follow_message
        cancel_future = handle.cancel_goal_async()
        if not self._wait_future(cancel_future, self.interface_timeout):
            return False, 'GoalNav cancel timeout'
        canceled = bool(cancel_future.result().goals_canceling)
        return (
            follow_ok and canceled,
            'GoalNav canceled' if canceled else 'GoalNav cancel rejected',
        )

    def navigate_to_pose(
        self,
        poi: Dict[str, float],
        feedback_callback: Optional[Callable[[object], None]] = None,
    ) -> Tuple[bool, str, int]:
        if not self._goal_nav.wait_for_server(timeout_sec=self.interface_timeout):
            return False, 'GoalNav action unavailable', -1

        goal = GoalNav.Goal()
        goal.control = True
        goal.x = float(poi['x'])
        goal.y = float(poi['y'])
        goal.z = float(poi.get('z', 0.0))
        goal.yaw = float(poi.get('yaw', 0.0))

        def default_feedback(message) -> None:
            progress = list(message.feedback.progress)
            self.get_logger().info(f'Navigation feedback: {progress} {message.feedback.message}')

        send_future = self._goal_nav.send_goal_async(
            goal,
            feedback_callback=feedback_callback or default_feedback,
        )
        if not self._wait_future(send_future, self.interface_timeout):
            return False, 'GoalNav goal request timeout', -2
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 'GoalNav goal rejected', -3

        self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        nav_timeout = float(self.get_parameter('navigation_timeout_sec').value)
        if not self._wait_future(result_future, nav_timeout):
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_future(cancel_future, self.interface_timeout)
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None
            return False, 'GoalNav execution timeout and cancel requested', -4

        wrapped_result = result_future.result()
        if self._active_goal_handle is goal_handle:
            self._active_goal_handle = None
        result = wrapped_result.result
        return bool(result.success), result.message, int(result.error_code)
