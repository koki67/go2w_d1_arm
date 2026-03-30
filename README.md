# go2w_d1_arm

`go2w_d1_arm` is a standalone ROS 2 Humble package and Docker deployment target for the Unitree D1 arm mounted on a GO2-W. It bridges the arm's DDS JSON protocol on `eth0` into standard ROS 2 topics and services that can be consumed from a desktop PC over WiFi.

## Features

- Publishes `joint_states` from the D1 `current_servo_angle` DDS stream
- Publishes parsed `arm_status` and passthrough `raw_feedback`
- Accepts `joint_command` as a 7-joint ROS `sensor_msgs/msg/JointState`
- Exposes `enable`, `disable`, `power_on`, `power_off`, `zero`, and `single_joint_command`
- Uses raw CycloneDDS transport for the D1 arm plus a separate ROS 2 CycloneDDS bridge process
- Keeps the D1 transport process on `eth0` and the ROS 2 bridge process on `wlan0`

## Repository Setup

The `unitree_sdk2` submodule is still tracked at `third_party/unitree_sdk2`, but only for the vendored CycloneDDS headers, libraries, and upstream DDS examples used by this package.

```bash
git submodule update --init --recursive
```

## Build

Container build:

```bash
make build
```

Local ROS 2 build when Humble is installed on the host:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select go2w_d1_arm
```

## Run On The Robot

Standard GO2-W runtime contract:

- `eth0` is used for the D1 arm DDS transport side by default
- `wlan0` is used for ROS 2 DDS so a desktop PC can communicate over WiFi by default
- The container starts two processes:
  - a raw D1 CycloneDDS transport on `eth0`
  - a ROS 2 CycloneDDS bridge on `wlan0`

The container now sets this contract internally. For the normal robot workflow, no host-side DDS environment variables are required. If a nonstandard NIC name is needed, you can still override `UNITREE_NETWORK_INTERFACE` and `ROS_NETWORK_INTERFACE` when invoking `make`.

```bash
make up
```

Diagnostics without launching the bridge:

```bash
make doctor
```

Open a shell inside the running container:

```bash
make shell
```

## Desktop PC Usage

With CycloneDDS on the desktop PC:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp97s0"/></Interfaces></General></Domain></CycloneDDS>'
ros2 topic echo /joint_states
ros2 service call /enable std_srvs/srv/Trigger "{}"
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: ['d1_joint_0', 'd1_joint_1', 'd1_joint_2', 'd1_joint_3', 'd1_joint_4', 'd1_joint_5', 'd1_joint_6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## Topics And Services

Published topics:

- `/joint_states`
- `/arm_status`
- `/raw_feedback`

Subscribed topics:

- `/joint_command`

Services:

- `/enable`
- `/disable`
- `/power_on`
- `/power_off`
- `/zero`
- `/single_joint_command`

## Notes

- The bridge gates motion commands until `enable` succeeds when `require_enable_before_motion` is `true`.
- `make up`, `make doctor`, and `make shell` all source the same in-container runtime setup so the ROS 2 and DDS environment stays consistent.
- Startup fails fast if the robot-side `wlan0` interface is not present, because desktop-over-WiFi is the intended control path for this repository.
- The bridge no longer relies on `unitree_sdk2` runtime channel wrappers. The D1 arm transport uses raw CycloneDDS in its own process and exchanges data with the ROS bridge over a local Unix socket.
- The split-process design keeps this repository CycloneDDS-only while avoiding the previous same-process DDS participant conflict.
