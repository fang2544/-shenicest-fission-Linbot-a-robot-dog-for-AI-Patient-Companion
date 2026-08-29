"""执行 4×3 米展厅中的心内科语音陪诊流程。"""

import json
from pathlib import Path
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from function_msgs.msg import AsrResult
from function_msgs.srv import FunctionInput, SetSpeak
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from hospital_escort_mvp.hospital_graph import HospitalGraph
from hospital_escort_mvp.wait_time_planner import WaitTimeEstimator, optimize_visit_groups


class CardiologyItinerary(Node):
    """依次执行检查、候诊计时和返回复诊。"""

    def __init__(self) -> None:
        super().__init__('cardiology_itinerary')
        share = Path(get_package_share_directory('hospital_escort_mvp'))
        defaults = {
            'scenario_path': str(share / 'config' / 'cardiology_itinerary.yaml'),
            'graph_path': str(share / 'config' / 'showroom_graph.yaml'),
            'asr_topic': '/asr/result',
            'priority_request_topic': '/hospital/priority_destination',
            'patient_token': 'showroom-cardiology-demo-token',
            'use_demo_timing': True,
            'demo_wait_duration_sec': 30.0,
            'report_path': '/vbot_ws/reports/cardiology_itinerary_dialogue.json',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        with Path(str(self.get_parameter('scenario_path').value)).open(
            'r', encoding='utf-8'
        ) as stream:
            self._scenario = yaml.safe_load(stream)

        self._steps = [dict(step) for step in self._scenario['itinerary']]
        self._graph = HospitalGraph.from_yaml(
            str(self.get_parameter('graph_path').value)
        )
        queue_config = self._scenario.get('queue_planning', {})
        self._queue_planning_enabled = bool(queue_config.get('enabled', True))
        self._wait_estimator = WaitTimeEstimator(
            queue_config.get('fallback_wait_times_sec', {}),
            stale_after_sec=float(queue_config.get('stale_after_sec', 120.0)),
            ewma_alpha=float(queue_config.get('ewma_alpha', 0.35)),
        )
        self._current_node = str(self._scenario['start_node'])
        self._step_index = 0
        self._phase = 'WAITING_START'
        self._active_target = None
        self._interrupted_target = None
        self._events = []
        self._initial_prompt_sent = False
        self._transition_at = None
        self._wait_started = None
        self._wait_duration = 0.0
        self._spoken_jokes = set()
        self._pending_unhandled_at = None

        self._mission = self.create_client(
            FunctionInput, '/hospital/request_mission'
        )
        self._speak_client = self.create_client(SetSpeak, '/set_speak')
        self.create_subscription(
            AsrResult,
            str(self.get_parameter('asr_topic').value),
            self._asr_cb,
            10,
        )
        self.create_subscription(
            String, '/hospital/pickup/status', self._mission_status_cb, 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter('priority_request_topic').value),
            self._priority_request_cb,
            10,
        )
        self.create_subscription(
            String,
            str(queue_config.get('topic', '/hospital/queue_status')),
            self._queue_status_cb,
            10,
        )
        self._state_pub = self.create_publisher(
            String, '/cardiology_itinerary/state', 10
        )
        self.create_timer(0.2, self._tick)
        self._optimize_remaining('startup')
        self._record('SYSTEM_READY', output=self._scenario['prompts']['ready'])

    def _record(self, event: str, **details) -> None:
        value = {
            'timestamp': time.time(),
            'phase': self._phase,
            'step_index': self._step_index,
            'event': event,
            **details,
        }
        self._events.append(value)
        message = json.dumps(value, ensure_ascii=False)
        self._state_pub.publish(String(data=message))
        self.get_logger().info(message)
        path = Path(str(self.get_parameter('report_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps({'events': self._events}, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

    def _speak(self, text: str, action: str) -> None:
        self._record('ROBOT_OUTPUT', output=text, action=action)
        if not self._speak_client.wait_for_service(timeout_sec=1.0):
            self._record('SPEECH_UNAVAILABLE', output=text)
            return
        request = SetSpeak.Request()
        request.target_state = 1
        request.mode = SetSpeak.Request.HUMAN_VOICE
        request.req_id = str(uuid.uuid4())
        request.human_language_text = text
        self._speak_client.call_async(request)

    def _tick(self) -> None:
        now = time.monotonic()
        if self._pending_unhandled_at is not None and now >= self._pending_unhandled_at:
            self._pending_unhandled_at = None
            self._speak(self._scenario['prompts']['busy'], 'busy_response')
        if not self._initial_prompt_sent:
            self._initial_prompt_sent = True
            self._speak(self._scenario['prompts']['ready'], 'present_itinerary')
            return
        if self._phase == 'TRANSITION' and now >= self._transition_at:
            self._dispatch_current_step()
            return
        if self._phase != 'WAITING_EXAM' or self._wait_started is None:
            return
        elapsed = now - self._wait_started
        jokes = self._scenario['wait']['jokes']
        for index, joke in enumerate(jokes):
            threshold = self._wait_duration * (index + 1) / (len(jokes) + 1)
            if elapsed >= threshold and index not in self._spoken_jokes:
                self._spoken_jokes.add(index)
                self._speak(joke, f'waiting_companion_{index + 1}')
        if elapsed >= self._wait_duration:
            self._speak(
                self._scenario['wait']['finished_prompt'], 'waiting_finished'
            )
            self._step_index += 1
            self._schedule_next_step()

    def _queue_status_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            values = self._wait_estimator.update(payload)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._record('QUEUE_STATUS_REJECTED', reason=str(exc))
            return
        self._record('QUEUE_STATUS_UPDATED', wait_times_sec=values)
        if self._phase in {'WAITING_START', 'TRANSITION'}:
            self._optimize_remaining('hospital_queue_update')

    def _optimize_remaining(self, reason: str) -> None:
        if not self._queue_planning_enabled or self._step_index >= len(self._steps):
            return
        remaining = self._steps[self._step_index:]
        # 用户临时插入的点位始终排在最前，只重排其后的未完成检查。
        leading = []
        while remaining and remaining[0].get('temporary_priority'):
            leading.append(remaining.pop(0))
        groups = {}
        order_seen = []
        fixed = []
        for step in remaining:
            group_id = step.get('optimization_group')
            if not group_id:
                fixed.append(step)
                continue
            if group_id not in groups:
                groups[group_id] = {
                    'visit_id': str(group_id), 'steps': [],
                    'wait_node': step.get('wait_node', step['target']),
                }
                order_seen.append(group_id)
            groups[group_id]['steps'].append(step)
        if len(groups) < 2:
            return
        try:
            optimized, report = optimize_visit_groups(
                self._graph,
                self._current_node,
                [groups[group_id] for group_id in order_seen],
                self._wait_estimator,
            )
        except (RuntimeError, ValueError) as exc:
            self._record('ITINERARY_REPLAN_FAILED', reason=str(exc))
            return
        reordered = leading + [
            step for visit in optimized for step in visit['steps']
        ] + fixed
        old_targets = [step['target'] for step in self._steps[self._step_index:]]
        self._steps[self._step_index:] = reordered
        self._record(
            'ITINERARY_REPLANNED', reason=reason,
            old_targets=old_targets,
            new_targets=[step['target'] for step in reordered],
            **report,
        )

    @staticmethod
    def _contains_any(text: str, phrases) -> bool:
        return any(phrase in text for phrase in phrases)

    def _asr_cb(self, message: AsrResult) -> None:
        self._record(
            'USER_INPUT',
            input=message.transcript,
            source_type=int(message.source_type),
            confidence=float(message.confidence),
            rejected=bool(message.reject),
        )
        if message.reject or message.confidence < float(
            self._scenario['minimum_confidence']
        ):
            self._speak(self._scenario['prompts']['low_confidence'], 'ask_repeat')
            return
        if self._contains_any(
            message.transcript, self._scenario['status_queries']['phrases']
        ):
            self._speak(self._status_reply(), 'answer_progress_query')
            return
        if self._phase == 'WAITING_START' and self._contains_any(
            message.transcript, self._scenario['start_phrases']
        ):
            self._speak(self._scenario['prompts']['accepted'], 'itinerary_accepted')
            self._schedule_next_step(delay_sec=0.3)
            return
        # 给独立意图节点预留短暂处理时间；收到高优先请求后取消通用回复。
        self._pending_unhandled_at = time.monotonic() + 0.25

    def _priority_request_cb(self, message: String) -> None:
        self._pending_unhandled_at = None
        try:
            payload = json.loads(message.data)
            target = str(payload['target'])
            config = {
                'label': str(payload['label']),
                'depart_prompt': str(payload['depart_prompt']),
                'arrival_prompt': str(payload['arrival_prompt']),
            }
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self._record('PRIORITY_REQUEST_REJECTED', reason=str(exc))
            return
        if payload.get('priority') != 'highest' or target not in self._graph.nodes:
            self._record(
                'PRIORITY_REQUEST_REJECTED', reason='invalid priority or target',
                target=target,
            )
            return
        self._record(
            'PRIORITY_REQUEST_RECEIVED', request_id=payload.get('request_id'),
            target=target, source=payload.get('source'),
        )
        self._insert_priority_destination((target, config))

    def _insert_priority_destination(self, destination) -> None:
        target, config = destination
        if self._phase in {'WAITING_START', 'COMPLETED', 'FAILED'}:
            self._speak('请先开始陪诊任务，再告诉聆灵需要临时前往哪里。', 'priority_not_active')
            return
        if self._active_target == target or any(
            step.get('temporary_priority') and step.get('target') == target
            for step in self._steps[self._step_index:]
        ):
            self._speak(f'已经把{config["label"]}设为当前最高优先级。', 'priority_duplicate')
            return

        if self._phase == 'WAITING_EXAM' and self._wait_started is not None:
            current = dict(self._steps[self._step_index])
            remaining_wait = max(
                0.0, self._wait_duration - (time.monotonic() - self._wait_started)
            )
            current['wait_duration_sec'] = remaining_wait
            current['depart_prompt'] = '临时行程结束，返回候诊区继续等待。'
            self._step_index += 1
            self._steps.insert(self._step_index, current)
            self._wait_started = None

        priority_step = {
            'target': target,
            'label': config['label'],
            'depart_prompt': config['depart_prompt'],
            'arrival_prompt': config['arrival_prompt'],
            'temporary_priority': True,
        }
        self._steps.insert(self._step_index, priority_step)
        interrupted = self._active_target
        self._interrupted_target = interrupted
        self._transition_at = None
        self._record(
            'PRIORITY_DESTINATION_INSERTED', target=target,
            interrupted_target=interrupted,
            remaining_targets=[s['target'] for s in self._steps[self._step_index:]],
        )
        self._dispatch_current_step()

    def _status_reply(self) -> str:
        if self._phase == 'WAITING_START':
            return self._scenario['prompts']['ready']
        if self._phase == 'WAITING_EXAM':
            remaining = max(
                0.0, self._wait_duration - (time.monotonic() - self._wait_started)
            )
            if bool(self.get_parameter('use_demo_timing').value):
                return f'正在彩超候诊区等待，演示倒计时约{int(remaining + 0.5)}秒。'
            return f'正在彩超候诊区等待，预计还需{int(remaining / 60 + 0.5)}分钟。'
        if self._phase == 'COMPLETED':
            return self._scenario['prompts']['completed']
        step = self._steps[min(self._step_index, len(self._steps) - 1)]
        return f'当前是第{self._step_index + 1}站，正在前往{step["label"]}。'

    def _schedule_next_step(self, delay_sec: float = 0.5) -> None:
        if self._step_index >= len(self._steps):
            self._phase = 'COMPLETED'
            self._active_target = None
            self._speak(self._scenario['prompts']['completed'], 'finish_itinerary')
            return
        self._optimize_remaining('remaining_route_changed')
        self._phase = 'TRANSITION'
        self._transition_at = time.monotonic() + delay_sec

    def _dispatch_current_step(self) -> None:
        step = self._steps[self._step_index]
        target = str(step['target'])
        self._phase = 'SUBMITTING'
        self._active_target = target
        self._speak(step['depart_prompt'], 'announce_next_exam')
        if not self._mission.wait_for_service(timeout_sec=1.0):
            self._fail('mission service unavailable')
            return
        request = FunctionInput.Request()
        request.source = 'voice_gateway'
        request.request_id = str(uuid.uuid4())
        request.dag = json.dumps(
            {
                'task_type': (
                    'priority_stop' if step.get('temporary_priority') else 'escort'
                ),
                'target_node': target,
                'patient_token': str(self.get_parameter('patient_token').value),
            }
        )
        future = self._mission.call_async(request)

        def completed(value):
            response = value.result()
            accepted = bool(response and response.success and response.success[0])
            self._record(
                'MISSION_RESPONSE',
                target=target,
                accepted=accepted,
                response=response.response if response else '',
            )
            if self._active_target != target:
                self._record('STALE_MISSION_RESPONSE_IGNORED', target=target)
                return
            if accepted:
                self._phase = 'NAVIGATING'
            else:
                self._fail('mission rejected')

        future.add_done_callback(completed)

    def _mission_status_cb(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._record('MISSION_STATUS', status=status)
        state = status.get('state')
        status_target = status.get('target_node')
        if (
            state == 'TARGET_REACHED'
            and self._interrupted_target
            and status_target == self._interrupted_target
            and status_target != self._active_target
        ):
            # 取消请求可能在切图期间或旧目标刚到达时生效，已完成点位不能重复访问。
            for index in range(self._step_index + 1, len(self._steps)):
                if self._steps[index].get('target') == status_target:
                    # 候诊点必须保留，临时绕行结束后患者还要返回并继续排队。
                    if not self._steps[index].get('wait_after_arrival'):
                        self._steps.pop(index)
                    break
            self._record(
                'INTERRUPTED_TARGET_COMPLETED_BEFORE_PREEMPT',
                target=status_target,
            )
            self._interrupted_target = None
            return
        if state == 'MISSION_PREEMPTED' and status_target == self._interrupted_target:
            self._record('INTERRUPTED_MISSION_PREEMPTED', target=status_target)
            self._interrupted_target = None
            return
        if state == 'MISSION_FAILED' and status.get('target_node') == self._active_target:
            self._fail('navigation failed')
            return
        if state != 'TARGET_REACHED' or status.get('target_node') != self._active_target:
            return
        step = self._steps[self._step_index]
        self._speak(step['arrival_prompt'], 'exam_point_reached')
        self._current_node = str(step['target'])
        self._active_target = None
        if bool(step.get('wait_after_arrival', False)):
            wait = self._scenario['wait']
            self._phase = 'WAITING_EXAM'
            if 'wait_duration_sec' in step:
                self._wait_duration = float(step['wait_duration_sec'])
                wait_source = 'resumed_wait'
            elif bool(self.get_parameter('use_demo_timing').value):
                self._wait_duration = float(
                    self.get_parameter('demo_wait_duration_sec').value)
                wait_source = 'demo_timing'
            else:
                estimate = self._wait_estimator.estimate(
                    str(step.get('wait_node', step['target'])))
                self._wait_duration = estimate.seconds
                wait_source = estimate.source
            self._wait_started = time.monotonic()
            self._spoken_jokes.clear()
            self._record(
                'WAIT_STARTED', duration_sec=self._wait_duration,
                source=wait_source,
            )
            self._speak(wait['start_prompt'], 'start_waiting_companion')
        else:
            self._step_index += 1
            self._schedule_next_step()

    def _fail(self, reason: str) -> None:
        self._phase = 'FAILED'
        self._active_target = None
        self._record('ITINERARY_FAILED', reason=reason)
        self._speak(self._scenario['prompts']['failed'], 'safe_stop_and_assist')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CardiologyItinerary()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
