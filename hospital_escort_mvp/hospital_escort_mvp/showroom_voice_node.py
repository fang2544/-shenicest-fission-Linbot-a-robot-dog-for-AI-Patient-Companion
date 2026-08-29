"""驱动 4×3 米展厅演示的确定性语音交互流程。"""

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


class ShowroomVoiceInteraction(Node):
    def __init__(self) -> None:
        super().__init__('showroom_voice_interaction')
        default_scenario = str(
            Path(get_package_share_directory('hospital_escort_mvp'))
            / 'config'
            / 'showroom_voice_scenario.yaml'
        )
        self.declare_parameter('scenario_path', default_scenario)
        self.declare_parameter('asr_topic', '/asr/result')
        self.declare_parameter('patient_token', 'showroom-demo-token')
        self.declare_parameter(
            'report_path', '/vbot_ws/reports/showroom_voice_dialogue.json'
        )
        with Path(str(self.get_parameter('scenario_path').value)).open(
            'r', encoding='utf-8'
        ) as stream:
            self._scenario = yaml.safe_load(stream)
        self._phase = 'WAITING_PICKUP_COMMAND'
        self._active_task = None
        self._events = []
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
        self._state_pub = self.create_publisher(
            String, '/showroom_voice/state', 10
        )
        self.create_timer(1.0, self._initial_prompt)
        self._initial_prompt_sent = False
        self._record('SYSTEM_READY', output=self._scenario['prompts']['ready'])

    def _record(self, event: str, **details) -> None:
        value = {
            'timestamp': time.time(),
            'phase': self._phase,
            'event': event,
            **details,
        }
        self._events.append(value)
        payload = json.dumps(value, ensure_ascii=False)
        self._state_pub.publish(String(data=payload))
        self.get_logger().info(payload)
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

    def _initial_prompt(self) -> None:
        if self._initial_prompt_sent:
            return
        self._initial_prompt_sent = True
        self._speak(self._scenario['prompts']['ready'], 'wait_for_remote_pickup')

    @staticmethod
    def _match(transcript: str, command_group):
        for target, command in command_group.items():
            if any(phrase in transcript for phrase in command['phrases']):
                return target, command
        return None, None

    def _asr_cb(self, message: AsrResult) -> None:
        self._record(
            'USER_INPUT',
            input=message.transcript,
            source_type=int(message.source_type),
            confidence=float(message.confidence),
            rejected=bool(message.reject),
        )
        minimum = float(self._scenario['minimum_confidence'])
        if message.reject or message.confidence < minimum:
            self._speak(self._scenario['prompts']['low_confidence'], 'ask_repeat')
            return
        if self._phase == 'WAITING_PICKUP_COMMAND':
            if message.source_type != AsrResult.PHONE:
                self._speak(
                    self._scenario['prompts']['invalid_pickup'], 'reject_local_pickup'
                )
                return
            target, command = self._match(
                message.transcript, self._scenario['pickup_commands']
            )
            if target is None:
                self._speak(
                    self._scenario['prompts']['invalid_pickup'], 'clarify_pickup'
                )
                return
            self._submit_mission('pickup', target, command['accepted_reply'])
            return
        if self._phase == 'WAITING_DESTINATION':
            if message.source_type != AsrResult.DOG:
                self._speak(
                    self._scenario['prompts']['invalid_destination'],
                    'reject_remote_destination',
                )
                return
            target, command = self._match(
                message.transcript, self._scenario['destination_commands']
            )
            if target is None:
                self._speak(
                    self._scenario['prompts']['invalid_destination'],
                    'clarify_destination',
                )
                return
            self._submit_mission('escort', target, command['accepted_reply'])
            return
        self._speak('当前任务正在执行，请稍候。', 'busy_response')

    def _submit_mission(self, task_type: str, target: str, reply: str) -> None:
        if not self._mission.wait_for_service(timeout_sec=1.0):
            self._speak('任务服务暂不可用，请工作人员协助。', 'mission_unavailable')
            return
        request = FunctionInput.Request()
        request.source = 'voice_gateway'
        request.request_id = str(uuid.uuid4())
        request.dag = json.dumps(
            {
                'task_type': task_type,
                'pickup_node': target if task_type == 'pickup' else '',
                'target_node': target if task_type == 'escort' else '',
                'patient_token': str(self.get_parameter('patient_token').value),
            }
        )
        future = self._mission.call_async(request)

        def completed(value):
            response = value.result()
            accepted = bool(response.success and response.success[0])
            self._record(
                'MISSION_RESPONSE',
                task_type=task_type,
                target=target,
                accepted=accepted,
                response=response.response,
            )
            if accepted:
                self._phase = (
                    'PICKUP_IN_PROGRESS'
                    if task_type == 'pickup'
                    else 'ESCORT_IN_PROGRESS'
                )
                self._active_task = task_type
                self._speak(reply, f'{task_type}_accepted')
            else:
                self._speak('任务未被接受，请工作人员协助。', 'mission_rejected')

        future.add_done_callback(completed)

    def _mission_status_cb(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._record('MISSION_STATUS', status=status)
        if status.get('state') != 'TARGET_REACHED':
            return
        task_type = status.get('task_type')
        if task_type == 'pickup':
            self._phase = 'WAITING_DESTINATION'
            self._active_task = None
            self._speak(
                self._scenario['prompts']['patient_met'], 'ask_destination'
            )
        elif task_type == 'escort':
            self._phase = 'COMPLETED'
            self._active_task = None
            self._speak(self._scenario['prompts']['completed'], 'finish_demo')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShowroomVoiceInteraction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
