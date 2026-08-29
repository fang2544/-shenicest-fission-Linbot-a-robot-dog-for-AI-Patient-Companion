#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash 2>/dev/null || source /vbot_ws/install/setup.bash

survey=/vbot_ws/reports/real_mapping_handoff_graph.yaml
report=/vbot_ws/reports/real_mapping_handoff.json
log=/vbot_ws/reports/real_mapping_handoff.log
rm -f "$survey" "$report" "$log"

setsid ros2 run hospital_escort_mvp mock_vbot >"$log" 2>&1 &
mock_pid=$!
setsid ros2 run hospital_escort_mvp floor_surveyor --ros-args \
  -p floor:=F1 -p map_name:=real_scan_F1 -p poi_id:=restroom_1f \
  -p poi_name:=一楼无障碍卫生间 -p poi_type:=restroom \
  -p survey_file:="$survey" >>"$log" 2>&1 &
survey_pid=$!
cleanup() {
  kill -INT -- "-$survey_pid" "-$mock_pid" 2>/dev/null || true
  sleep 0.5
  kill -TERM -- "-$survey_pid" "-$mock_pid" 2>/dev/null || true
  wait "$survey_pid" "$mock_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
for service in start_mapping finish_and_save_map record_poi validate_graph; do
  output=$(ros2 service call "/hospital_floor_surveyor/$service" std_srvs/srv/Trigger '{}')
  echo "$output" >>"$log"
  grep -q 'success=True' <<<"$output"
done

python3 - <<'PY'
import json
from pathlib import Path
from hospital_escort_mvp.hospital_graph import HospitalGraph

path = Path('/vbot_ws/reports/real_mapping_handoff_graph.yaml')
graph = HospitalGraph.from_yaml(str(path))
node = graph.nodes['restroom_1f']
checks = {
    'explicit_save_map_service_succeeded': True,
    'map_registered_for_localization': node.map_name == 'real_scan_F1',
    'tf_poi_recorded': node.node_type == 'restroom',
    'navigator_can_load_graph': len(graph.nodes) == 1,
}
payload = {
    'success': all(checks.values()),
    'checks': checks,
    'graph_path': str(path),
    'recorded_node': node.__dict__,
    'real_robot_boundary': (
        'Mock validates interface and handoff logic; sensor calibration, firmware '
        'map persistence and real TF must be verified on the physical VBot.'
    ),
}
Path('/vbot_ws/reports/real_mapping_handoff.json').write_text(
    json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
assert payload['success']
print(json.dumps(payload, ensure_ascii=False))
PY
