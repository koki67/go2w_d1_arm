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

transport_binary="/ros2_ws/install/go2w_d1_arm/lib/go2w_d1_arm/d1_arm_transport"
bridge_launch_args=("transport_socket_path:=${D1_TRANSPORT_SOCKET_PATH}")

cleanup() {
  if [ -n "${transport_pid:-}" ] && kill -0 "${transport_pid}" >/dev/null 2>&1; then
    kill "${transport_pid}" >/dev/null 2>&1 || true
    wait "${transport_pid}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

echo "[go2w_d1_arm] starting raw D1 transport process"
(
  export CYCLONEDDS_URI="${D1_TRANSPORT_CYCLONEDDS_URI}"
  exec "${transport_binary}"
) &
transport_pid=$!

for _ in $(seq 1 50); do
  if [ -S "${D1_TRANSPORT_SOCKET_PATH}" ]; then
    break
  fi
  if ! kill -0 "${transport_pid}" >/dev/null 2>&1; then
    runtime_error "raw D1 transport process exited before opening ${D1_TRANSPORT_SOCKET_PATH}"
  fi
  sleep 0.1
done

if [ ! -S "${D1_TRANSPORT_SOCKET_PATH}" ]; then
  runtime_error "timed out waiting for raw D1 transport socket at ${D1_TRANSPORT_SOCKET_PATH}"
fi

echo "[go2w_d1_arm] starting ROS 2 bridge process"
export CYCLONEDDS_URI="${ROS_CYCLONEDDS_URI}"
ros2 launch go2w_d1_arm d1_arm_bridge.launch.py "${bridge_launch_args[@]}"
