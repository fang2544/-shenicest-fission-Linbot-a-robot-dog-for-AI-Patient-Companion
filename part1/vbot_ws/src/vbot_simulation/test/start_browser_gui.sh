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

cleanup() {
  kill "$web_pid" "$vnc_pid" "$fluxbox_pid" "$xvfb_pid" 2>/dev/null || true
}
trap cleanup EXIT

echo 'VBot GUI is available at http://localhost:6080/vnc.html?autoconnect=true'
ros2 launch vbot_simulation navigation_rviz.launch.py gazebo_gui:=true
