#!/bin/bash
set -euo pipefail

# shellcheck disable=SC1091
source /setup_condition.sh

print_runtime_summary
validate_runtime_artifacts

if ros2 pkg prefix go2w_d1_arm >/dev/null 2>&1; then
  echo "[go2w_d1_arm] package lookup succeeded"
else
  runtime_error "ros2 cannot resolve package 'go2w_d1_arm' after sourcing /setup_condition.sh"
fi

echo "[go2w_d1_arm] starting bridge process"

if [ -n "${UNITREE_NETWORK_INTERFACE:-}" ]; then
  exec ros2 launch go2w_d1_arm d1_arm_bridge.launch.py "network_interface:=${UNITREE_NETWORK_INTERFACE}"
fi

exec ros2 launch go2w_d1_arm d1_arm_bridge.launch.py
