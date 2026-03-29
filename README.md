# go2w_d1_arm

`go2w_d1_arm` is a standalone ROS 2 Humble package and Docker deployment target for the Unitree D1 arm mounted on a GO2-W. It bridges the arm's DDS JSON protocol on `eth0` into standard ROS 2 topics and services that can be consumed from a desktop PC over WiFi.

## Features

- Publishes `joint_states` from the D1 `current_servo_angle` DDS stream
- Publishes parsed `arm_status` and passthrough `raw_feedback`
- Accepts `joint_command` as a 7-joint ROS `sensor_msgs/msg/JointState`
- Exposes `enable`, `disable`, `power_on`, `power_off`, `zero`, and `single_joint_command`
- Uses a vendored `unitree_sdk2` git submodule plus the required DDS entity patch for coexistence with ROS 2 DDS
- Defaults to `rmw_cyclonedds_cpp` for ROS 2, with `rmw_fastrtps_cpp` available as a fallback

## Repository Setup

The `unitree_sdk2` submodule is tracked at `third_party/unitree_sdk2`.

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

Typical GO2-W settings:

```bash
export UNITREE_NETWORK_INTERFACE=eth0
export ROS_NETWORK_INTERFACE=wlan0
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
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp97s0"/></Interfaces></General></Domain></CycloneDDS>'
ros2 topic echo /joint_states
ros2 service call /enable std_srvs/srv/Trigger "{}"
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
- If two CycloneDDS participants in the same process conflict on the target system, set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` before `make up`.
- `UNITREE_NETWORK_INTERFACE` selects the arm-side Ethernet interface for the Unitree SDK. `ROS_NETWORK_INTERFACE` selects the ROS 2-facing interface used in the default `CYCLONEDDS_URI`.

