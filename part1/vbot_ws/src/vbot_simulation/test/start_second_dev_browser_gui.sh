#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash

export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1

Xvfb :99 -screen 0 1600x1000x24 -ac +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
xvfb_pid=$!
fluxbox >/tmp/fluxbox.log 2>&1 &
fluxbox_pid=$!
x11vnc -display :99 -forever -shared -rfbport 5900 -nopw >/tmp/x11vnc.log 2>&1 &
vnc_pid=$!
websockify --web=/usr/share/novnc/ 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
web_pid=$!
python3 -m http.server 8080 --directory /vbot_ws/reports >/tmp/second_dev_http.log 2>&1 &
http_pid=$!

initialize_pose() {
  sleep 30
  for _ in $(seq 1 3); do
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      '{header: {frame_id: map}, pose: {pose: {position: {x: -4.5, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02]}}' \
      >/tmp/second_dev_initial_pose.log 2>&1
    sleep 5
  done
}
initialize_pose &
pose_pid=$!

cleanup() {
  kill "$pose_pid" "$http_pid" "$web_pid" "$vnc_pid" "$fluxbox_pid" "$xvfb_pid" 2>/dev/null || true
}
trap cleanup EXIT

echo 'Gazebo/RViz: http://localhost:6080/vnc.html?autoconnect=true'
echo 'Latest passed report: http://localhost:6081/second_dev_visual_report.html'
echo 'Current GUI session: http://localhost:6081/second_dev_gui_live.html'
ros2 launch vbot_simulation second_dev_demo.launch.py \
  gazebo_gui:=true rviz:=true auto_start_cardiology:=false \
  report_path:=/vbot_ws/reports/second_dev_gui_live.json \
  html_report_path:=/vbot_ws/reports/second_dev_gui_live.html
