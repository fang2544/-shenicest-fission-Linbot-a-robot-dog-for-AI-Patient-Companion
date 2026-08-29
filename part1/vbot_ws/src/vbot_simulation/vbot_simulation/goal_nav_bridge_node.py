import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from function_msgs.action import GoalNav


class GoalNavBridge(Node):
    """把 VBot 的 GoalNav action 转换为 Nav2 的 NavigateToPose。"""

    def __init__(self) -> None:
        super().__init__('goal_nav_bridge')
        self.declare_parameter('nav2_action', '/navigate_to_pose')
        self.declare_parameter('goal_nav_action', '/goal_nav')
        self._group = ReentrantCallbackGroup()
        self._active_lock = threading.Lock()
        self._active_nav_handle = None
        self._nav2 = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav2_action').value),
            callback_group=self._group,
        )
        self._server = ActionServer(
            self,
            GoalNav,
            str(self.get_parameter('goal_nav_action').value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )

    def _goal(self, goal_request):
        # VBot 使用 control=false 表示停止当前导航任务。
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        if not goal_handle.request.control:
            return self._execute_stop(goal_handle)

        result = GoalNav.Result()
        if not self._nav2.wait_for_server(timeout_sec=20.0):
            result.success = False
            result.message = 'Nav2 NavigateToPose unavailable'
            result.error_code = 1001
            goal_handle.abort()
            return result

        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = goal_handle.request.x
        request.pose.pose.position.y = goal_handle.request.y
        request.pose.pose.position.z = goal_handle.request.z
        request.pose.pose.orientation.z = math.sin(goal_handle.request.yaw / 2.0)
        request.pose.pose.orientation.w = math.cos(goal_handle.request.yaw / 2.0)
        latest_distance = {'value': None}

        def nav_feedback(message):
            latest_distance['value'] = float(message.feedback.distance_remaining)

        send_future = self._nav2.send_goal_async(request, feedback_callback=nav_feedback)
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not send_future.done() or not send_future.result().accepted:
            result.success = False
            result.message = 'Nav2 rejected goal'
            result.error_code = 1002
            goal_handle.abort()
            return result
        nav_handle = send_future.result()
        with self._active_lock:
            self._active_nav_handle = nav_handle
        result_future = nav_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled'
                result.error_code = 1003
                with self._active_lock:
                    if self._active_nav_handle is nav_handle:
                        self._active_nav_handle = None
                return result
            feedback = GoalNav.Feedback()
            remaining = latest_distance['value']
            feedback.progress = [0 if remaining is None else max(0, int(100 - remaining * 12))]
            feedback.message = 'Nav2 executing' if remaining is None else f'{remaining:.2f} m remaining'
            goal_handle.publish_feedback(feedback)
            time.sleep(0.25)

        wrapped = result_future.result()
        with self._active_lock:
            if self._active_nav_handle is nav_handle:
                self._active_nav_handle = None
        if wrapped is not None and wrapped.status == 4:
            goal_handle.succeed()
            result.success = True
            result.message = 'Nav2 goal reached'
            result.error_code = 0
        else:
            goal_handle.abort()
            result.success = False
            result.message = f'Nav2 failed with status {getattr(wrapped, "status", -1)}'
            result.error_code = 1004
        return result

    def _execute_stop(self, goal_handle):
        result = GoalNav.Result()
        with self._active_lock:
            nav_handle = self._active_nav_handle
        if nav_handle is not None:
            future = nav_handle.cancel_goal_async()
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done():
                goal_handle.abort()
                result.success = False
                result.message = 'Timed out while stopping Nav2 goal'
                result.error_code = 1005
                return result
        goal_handle.succeed()
        result.success = True
        result.message = 'Nav2 goal stopped' if nav_handle is not None else 'No active Nav2 goal'
        result.error_code = 0
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalNavBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
