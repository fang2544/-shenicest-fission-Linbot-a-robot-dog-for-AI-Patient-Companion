#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash

report=/vbot_ws/reports/cardiology_nav2_dialogue.json
log=/vbot_ws/reports/cardiology_nav2_launch.log
rm -f "$report"

export DISPLAY=:99
Xvfb :99 -screen 0 1280x800x24 -nolisten tcp >/tmp/vbot_cardiology_xvfb.log 2>&1 &
xvfb_pid=$!

setsid ros2 launch vbot_simulation cardiology_nav.launch.py \
  gui:=false dynamic_obstacle:=true >"$log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$launch_pid" 2>/dev/null || break
    sleep 0.1
  done
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
ros2 topic pub --once /asr/result function_msgs/msg/AsrResult \
  '{transcript: 开始陪诊, source_type: 0, reject: false, confidence: 0.98}'

for _ in $(seq 1 900); do
  if grep -q '"phase": "COMPLETED"' "$report" 2>/dev/null; then
    python3 -c 'import json; data=json.load(open("/vbot_ws/reports/cardiology_nav2_dialogue.json")); assert data["events"][-1]["phase"] == "COMPLETED"'
    echo 'cardiology Nav2 itinerary: COMPLETED'
    exit 0
  fi
  sleep 0.5
done

tail -n 120 "$log"
echo 'cardiology Nav2 itinerary did not complete' >&2
exit 2
