#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash

report=/vbot_ws/reports/aed_emergency_acceptance.json
log=/vbot_ws/reports/aed_emergency_launch.log
rm -f "$report" /vbot_ws/reports/aed_emergency_visual_report.html
export DISPLAY=:99
Xvfb :99 -screen 0 1280x800x24 -nolisten tcp >/tmp/vbot_aed_xvfb.log 2>&1 &
xvfb_pid=$!
setsid ros2 launch vbot_simulation aed_emergency_demo.launch.py \
  gazebo_gui:=false rviz:=false dynamic_obstacle:=false >"$log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 40); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 0.1; done
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  kill "$xvfb_pid" 2>/dev/null || true
  wait "$xvfb_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 30
for _ in $(seq 1 3); do
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: map}, pose: {pose: {position: {x: -4.5, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02]}}'
  sleep 5
done

for _ in $(seq 1 700); do
  if python3 - <<'PY' 2>/dev/null
import json
data = json.load(open('/vbot_ws/reports/aed_emergency_acceptance.json'))
raise SystemExit(0 if data.get('success') else 1)
PY
  then
    echo 'AED Gazebo/Nav2 acceptance: PASSED'
    exit 0
  fi
  sleep 0.5
done
tail -n 180 "$log"
echo 'AED Gazebo/Nav2 acceptance did not complete' >&2
exit 2
