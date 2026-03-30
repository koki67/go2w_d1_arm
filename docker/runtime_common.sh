#!/bin/bash

runtime_error() {
  echo "[go2w_d1_arm] ERROR: $*" >&2
  exit 1
}

require_file() {
  local path="$1"
  local label="$2"
  if [ ! -e "$path" ]; then
    runtime_error "missing ${label}: ${path}"
  fi
}

source_runtime_environment() {
  local restore_nounset=0

  require_file /opt/ros/humble/setup.bash "ROS 2 Humble setup"
  require_file /ros2_ws/install/setup.bash "workspace setup"

  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source /ros2_ws/install/setup.bash

  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi
}

readarray_network_interfaces() {
  local iface
  NETWORK_INTERFACES=()

  if [ -d /sys/class/net ]; then
    while IFS= read -r iface; do
      NETWORK_INTERFACES+=("$iface")
    done < <(find /sys/class/net -mindepth 1 -maxdepth 1 -printf '%f\n' | sort)
  fi
}

joined_network_interfaces() {
  if [ "${#NETWORK_INTERFACES[@]}" -eq 0 ]; then
    echo "(none detected)"
    return
  fi

  local joined=""
  local iface
  for iface in "${NETWORK_INTERFACES[@]}"; do
    if [ -n "$joined" ]; then
      joined+=", "
    fi
    joined+="$iface"
  done
  echo "$joined"
}

print_runtime_summary() {
  echo "[go2w_d1_arm] runtime preflight"
  echo "[go2w_d1_arm] ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  echo "[go2w_d1_arm] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
  echo "[go2w_d1_arm] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "[go2w_d1_arm] UNITREE_NETWORK_INTERFACE=${UNITREE_NETWORK_INTERFACE:-<unset>}"
  echo "[go2w_d1_arm] ROS_NETWORK_INTERFACE=${ROS_NETWORK_INTERFACE:-<unset>}"
  echo "[go2w_d1_arm] D1_TRANSPORT_SOCKET_PATH=${D1_TRANSPORT_SOCKET_PATH:-<unset>}"
  echo "[go2w_d1_arm] CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
  echo "[go2w_d1_arm] ROS_CYCLONEDDS_URI=${ROS_CYCLONEDDS_URI:-<unset>}"
  echo "[go2w_d1_arm] D1_TRANSPORT_CYCLONEDDS_URI=${D1_TRANSPORT_CYCLONEDDS_URI:-<unset>}"

  readarray_network_interfaces
  echo "[go2w_d1_arm] detected network interfaces: $(joined_network_interfaces)"
}

validate_runtime_artifacts() {
  require_file /ros2_ws/install/setup.bash "workspace setup"
  require_file /ros2_ws/install/go2w_d1_arm/lib/go2w_d1_arm/d1_arm_bridge_node "bridge executable"
  require_file /ros2_ws/install/go2w_d1_arm/lib/go2w_d1_arm/d1_arm_transport "transport executable"
}

validate_selected_interface() {
  local env_name="$1"
  local label="$2"
  local selected_interface="${!env_name:-}"

  if [ -z "$selected_interface" ]; then
    return
  fi

  if [ ! -d "/sys/class/net/${selected_interface}" ]; then
    runtime_error \
      "${label} interface '${selected_interface}' was not found. Detected interfaces: $(joined_network_interfaces)"
  fi
}

validate_rmw_implementation() {
  local rmw="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

  case "$rmw" in
    rmw_cyclonedds_cpp)
      ;;
    *)
      runtime_error \
        "unsupported RMW_IMPLEMENTATION=${rmw}. This repository now uses rmw_cyclonedds_cpp only."
      ;;
  esac
}
