#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash

# Gazebo Classic 深度相机即使不启动 gzclient 也需要 X 服务。
export DISPLAY=:99
Xvfb :99 -screen 0 1280x800x24 -nolisten tcp >/tmp/vbot_xvfb.log 2>&1 &
xvfb_pid=$!

map_yaml=/vbot_ws/maps/hospital_sim.yaml
map_image=/vbot_ws/maps/hospital_sim.pgm
mapping_report=/vbot_ws/reports/mapping_acceptance.json
navigation_report=/vbot_ws/reports/navigation_acceptance.json
summary_report=/vbot_ws/reports/simulation_acceptance_summary.json
mapping_log=/vbot_ws/reports/mapping_launch.log
navigation_log=/vbot_ws/reports/navigation_launch.log

rm -f "$map_yaml" "$map_image" "$mapping_report" "$navigation_report" "$summary_report"

launch_pid=''
cleanup() {
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
final_cleanup() {
  cleanup
  kill "$xvfb_pid" 2>/dev/null || true
  wait "$xvfb_pid" 2>/dev/null || true
}
trap final_cleanup EXIT

setsid ros2 launch vbot_simulation mapping.launch.py gui:=false >"$mapping_log" 2>&1 &
launch_pid=$!
timeout 180 ros2 run vbot_simulation mapping_driver --ros-args -p use_sim_time:=true
cleanup
launch_pid=''

test -s "$map_yaml"
test -s "$map_image"
python3 -c 'import json; assert json.load(open("/vbot_ws/reports/mapping_acceptance.json"))["success"]'

ros2 daemon stop >/dev/null 2>&1 || true
setsid ros2 launch vbot_simulation navigation.launch.py \
  gui:=false dynamic_obstacle:=true map:="$map_yaml" >"$navigation_log" 2>&1 &
launch_pid=$!
# 每次重新建图都选择已确认空闲的终点；巡检在走廊内结束时，最东侧房间可保留未知区。
timeout 210 ros2 run vbot_simulation simulation_acceptance --ros-args \
  -p use_sim_time:=true -p goal_x:=3.0 -p goal_y:=0.0
cleanup
launch_pid=''

python3 - <<'PY'
import json
from pathlib import Path

mapping = json.loads(Path('/vbot_ws/reports/mapping_acceptance.json').read_text())
navigation = json.loads(Path('/vbot_ws/reports/navigation_acceptance.json').read_text())
summary = {
    'success': bool(mapping['success'] and navigation['success']),
    'mapping': mapping,
    'navigation': navigation,
    'artifacts': {
        'map_yaml': '/vbot_ws/maps/hospital_sim.yaml',
        'map_image': '/vbot_ws/maps/hospital_sim.pgm',
        'mapping_log': '/vbot_ws/reports/mapping_launch.log',
        'navigation_log': '/vbot_ws/reports/navigation_launch.log',
    },
}
Path('/vbot_ws/reports/simulation_acceptance_summary.json').write_text(
    json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8'
)
print(json.dumps(summary, ensure_ascii=False, indent=2))
raise SystemExit(0 if summary['success'] else 2)
PY
