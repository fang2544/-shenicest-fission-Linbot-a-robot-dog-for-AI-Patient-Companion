#!/usr/bin/env bash
set -eo pipefail

workspace="${VBOT_WS:-/vbot_ws}"
install_prefix="${VBOT_INSTALL_PREFIX:-${workspace}/install}"

source /opt/ros/humble/setup.bash
source "${install_prefix}/setup.bash"

log="${workspace}/reports/aed_node_integration.log"
mkdir -p "${workspace}/reports"
setsid ros2 run aed_emergency_response aed_emergency_node --ros-args \
  --params-file "${workspace}/src/aed_emergency_response/config/aed_emergency.yaml" \
  -p guidance_interval_sec:=0.12 \
  -p report_path:="${workspace}/reports/aed_node_integration_events.json" >"$log" 2>&1 &
node_pid=$!
cleanup() {
  kill -INT -- "-$node_pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "$node_pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -TERM -- "-$node_pid" 2>/dev/null || true
  sleep 0.3
  kill -KILL -- "-$node_pid" 2>/dev/null || true
  wait "$node_pid" 2>/dev/null || true
}
trap cleanup EXIT
python3 "${workspace}/src/aed_emergency_response/test/integration_harness.py"
