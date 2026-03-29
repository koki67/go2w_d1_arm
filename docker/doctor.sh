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

if package_prefix="$(ros2 pkg prefix go2w_d1_arm 2>/dev/null)"; then
  echo "[go2w_d1_arm] package prefix=${package_prefix}"
else
  runtime_error "ros2 cannot resolve package 'go2w_d1_arm' after sourcing /ros2_ws/install/setup.bash"
fi

if executables="$(ros2 pkg executables go2w_d1_arm 2>/dev/null)"; then
  echo "[go2w_d1_arm] package executables:"
  printf '%s\n' "$executables"
else
  runtime_error "ros2 cannot list executables for package 'go2w_d1_arm'"
fi

if command -v ip >/dev/null 2>&1; then
  echo "[go2w_d1_arm] ip -br addr:"
  ip -br addr
fi

echo "[go2w_d1_arm] doctor checks passed"
