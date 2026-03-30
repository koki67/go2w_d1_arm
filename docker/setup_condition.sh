#!/bin/bash

# shellcheck disable=SC1091
source /runtime_common.sh

source_runtime_environment

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# This repository targets the standard GO2-W layout:
# - eth0: Unitree D1 arm / internal robot-side DDS
# - wlan0: ROS 2 DDS path for desktop WiFi clients
#
# The bridge process hosts both the ROS 2 node and the Unitree SDK DDS channels,
# so CycloneDDS must allow both interfaces in the same process.
export UNITREE_NETWORK_INTERFACE="eth0"
export ROS_NETWORK_INTERFACE="wlan0"

readarray_network_interfaces
validate_selected_interface UNITREE_NETWORK_INTERFACE "Unitree SDK"
validate_selected_interface ROS_NETWORK_INTERFACE "ROS 2"
validate_rmw_implementation

if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="1" multicast="true" />
    <NetworkInterface name="wlan0" priority="2" multicast="true" />
  </Interfaces></General></Domain></CycloneDDS>'
else
  unset CYCLONEDDS_URI
fi
