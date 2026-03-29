#!/bin/bash
set -euo pipefail

# shellcheck disable=SC1091
source /runtime_common.sh

source_runtime_environment
ensure_cyclonedds_uri
print_runtime_summary
validate_runtime_artifacts
validate_selected_interface UNITREE_NETWORK_INTERFACE "Unitree SDK"
validate_selected_interface ROS_NETWORK_INTERFACE "ROS 2"
validate_rmw_implementation

if ros2 pkg prefix go2w_d1_arm >/dev/null 2>&1; then
  echo "[go2w_d1_arm] package lookup succeeded"
else
  runtime_error "ros2 cannot resolve package 'go2w_d1_arm' after sourcing /ros2_ws/install/setup.bash"
fi

echo "[go2w_d1_arm] starting bridge process"

if [ -n "${UNITREE_NETWORK_INTERFACE:-}" ]; then
  exec ros2 launch go2w_d1_arm d1_arm_bridge.launch.py "network_interface:=${UNITREE_NETWORK_INTERFACE}"
fi

exec ros2 launch go2w_d1_arm d1_arm_bridge.launch.py
