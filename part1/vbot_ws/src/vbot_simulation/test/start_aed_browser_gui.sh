#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1

Xvfb :99 -screen 0 1600x1000x24 -ac +extension GLX +render -noreset >/tmp/aed_xvfb.log 2>&1 &
xvfb_pid=$!
fluxbox >/tmp/aed_fluxbox.log 2>&1 & fluxbox_pid=$!
x11vnc -display :99 -forever -shared -rfbport 5900 -nopw >/tmp/aed_x11vnc.log 2>&1 & vnc_pid=$!
websockify --web=/usr/share/novnc/ 6080 localhost:5900 >/tmp/aed_novnc.log 2>&1 & web_pid=$!
python3 -m http.server 8080 --directory /vbot_ws/reports >/tmp/aed_http.log 2>&1 & http_pid=$!

initialize_pose() {
  sleep 30
  for _ in $(seq 1 3); do
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      '{header: {frame_id: map}, pose: {pose: {position: {x: -4.5, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02]}}' \
      >/tmp/aed_initial_pose.log 2>&1
    sleep 5
  done
}
initialize_pose & pose_pid=$!
cleanup() {
  kill "$pose_pid" "$http_pid" "$web_pid" "$vnc_pid" "$fluxbox_pid" "$xvfb_pid" 2>/dev/null || true
}
trap cleanup EXIT

echo 'Gazebo/RViz: http://localhost:6080/vnc.html?autoconnect=true'
echo 'Live AED report: http://localhost:6081/aed_emergency_visual_report.html'
ros2 launch vbot_simulation aed_emergency_demo.launch.py \
  gazebo_gui:=true rviz:=true dynamic_obstacle:=true \
  request_delay_sec:=45.0 guidance_interval_sec:=4.0 navigation_timeout_sec:=900.0
