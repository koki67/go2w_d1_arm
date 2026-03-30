#!/bin/bash

# shellcheck disable=SC1091
source /runtime_common.sh

source_runtime_environment

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export D1_TRANSPORT_SOCKET_PATH="${D1_TRANSPORT_SOCKET_PATH:-/tmp/go2w_d1_arm.sock}"

# This repository targets the standard GO2-W layout:
# - eth0: Unitree D1 arm DDS transport
# - wlan0: ROS 2 DDS path for desktop WiFi clients
export UNITREE_NETWORK_INTERFACE="${UNITREE_NETWORK_INTERFACE:-eth0}"
export ROS_NETWORK_INTERFACE="${ROS_NETWORK_INTERFACE:-wlan0}"

case "$(uname -m)" in
  aarch64|arm64)
    system_library_dir="/usr/lib/aarch64-linux-gnu"
    ;;
  *)
    system_library_dir="/usr/lib/x86_64-linux-gnu"
    ;;
esac

installed_transport_lib_dir="/ros2_ws/install/go2w_d1_arm/lib/go2w_d1_arm"

if [ -z "${D1_TRANSPORT_LIBRARY_DIR:-}" ]; then
  if [ -e "${installed_transport_lib_dir}/libddscxx.so.0" ]; then
    export D1_TRANSPORT_LIBRARY_DIR="${installed_transport_lib_dir}"
  elif [ -e "/usr/local/lib/libddscxx.so.0" ]; then
    export D1_TRANSPORT_LIBRARY_DIR="/usr/local/lib"
  elif [ -e "${system_library_dir}/libddscxx.so.0" ]; then
    export D1_TRANSPORT_LIBRARY_DIR="${system_library_dir}"
  else
    export D1_TRANSPORT_LIBRARY_DIR=""
  fi
fi

if [ -d "${D1_TRANSPORT_LIBRARY_DIR}" ]; then
  export LD_LIBRARY_PATH="${D1_TRANSPORT_LIBRARY_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

readarray_network_interfaces
validate_selected_interface UNITREE_NETWORK_INTERFACE "D1 transport"
validate_selected_interface ROS_NETWORK_INTERFACE "ROS 2"
validate_rmw_implementation

export ROS_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${ROS_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"true\" />
</Interfaces></General></Domain></CycloneDDS>"

export D1_TRANSPORT_CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${UNITREE_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"true\" />
</Interfaces></General></Domain></CycloneDDS>"

export CYCLONEDDS_URI="${ROS_CYCLONEDDS_URI}"
