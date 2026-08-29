#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash 2>/dev/null || source /vbot_ws/install/setup.bash
set -u

report=/vbot_ws/reports/cardiology_itinerary_dialogue.json
launch_log=/tmp/cardiology_launch.log
rm -f "$report"

setsid ros2 launch hospital_escort_mvp mock_cardiology_showroom.launch.py \
  demo_wait_duration_sec:=1.0 navigation_step_sec:=0.20 >"$launch_log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
ros2 topic pub --once /hospital/queue_status std_msgs/msg/String \
  '{data: "{\"wait_times_sec\":{\"blood_draw_1f\":900,\"ecg_2f\":5,\"cardiac_ultrasound_2f\":600}}"}'
ros2 topic pub --once /asr/result function_msgs/msg/AsrResult \
  '{transcript: 开始陪诊, source_type: 0, reject: false, confidence: 0.98}'
sleep 0.7
ros2 topic pub --once /asr/result function_msgs/msg/AsrResult \
  '{transcript: 我想先去厕所, source_type: 0, reject: false, confidence: 0.99}'

completed=0
for _ in $(seq 1 320); do
  if grep -q '"phase": "COMPLETED"' "$report" 2>/dev/null; then
    completed=1
    break
  fi
  sleep 0.25
done
if [[ "$completed" != 1 ]]; then
  tail -n 80 "$launch_log"
  echo 'cardiology itinerary did not complete' >&2
  exit 1
fi

python3 - <<'PY'
import json
from pathlib import Path

path = Path('/vbot_ws/reports/cardiology_itinerary_dialogue.json')
events = json.loads(path.read_text(encoding='utf-8'))['events']
outputs = [
    event['output'] for event in events if event['event'] == 'ROBOT_OUTPUT'
]
targets = [
    event['status'].get('target_node')
    for event in events
    if event['event'] == 'MISSION_STATUS'
    and event['status'].get('state') == 'TARGET_REACHED'
]
assert events[-1]['phase'] == 'COMPLETED', events[-1]
assert 'restroom_1f' in targets[:2], targets
assert set(targets) == {
    'restroom_1f',
    'blood_draw_1f', 'ecg_2f', 'echo_waiting_2f',
    'cardiac_ultrasound_2f', 'cardiology_3f',
}, targets
assert targets.count('restroom_1f') == 1, targets
assert targets.count('echo_waiting_2f') in (1, 2), targets
assert len(targets) in (6, 7), targets
assert any(e['event'] == 'PRIORITY_DESTINATION_INSERTED' for e in events)
assert any(e['event'] == 'ITINERARY_REPLANNED' for e in events)
assert any(
    e['event'] == 'QUEUE_STATUS_UPDATED'
    and e['wait_times_sec']['ecg_2f'] == 5
    for e in events
)
assert any('着急寄存' in text for text in outputs)
assert any('练习耐心' in text for text in outputs)
print('integration: COMPLETED')
print('reached:', ' -> '.join(targets))
print('spoken outputs:', len(outputs))
PY

tail -n 25 "$launch_log"
