# go2w_d1_arm

`go2w_d1_arm` is a ROS 2 Humble bridge for controlling a Unitree D1 arm mounted on a GO2-W.

This repository runs the arm-side DDS transport on the robot's internal network and exposes a normal ROS 2 interface over WiFi so a desktop PC can monitor the arm and send commands wirelessly.

## Table of Contents

- [Overview](#overview)
- [Network Contract](#network-contract)
- [Repository Setup](#repository-setup)
- [Robot Quick Start](#robot-quick-start)
- [Desktop PC Quick Start](#desktop-pc-quick-start)
  - [Minimal Setup](#minimal-setup)
  - [If You Want `/single_joint_command`](#if-you-want-single_joint_command)
- [First End-to-End Checks](#first-end-to-end-checks)
- [ROS Interface Summary](#ros-interface-summary)
- [Default Motion Behavior](#default-motion-behavior)
- [Troubleshooting](#troubleshooting)
- [Notes](#notes)

## Overview

This repository provides:

- live `/joint_states` from the D1 arm
- `/arm_status` and `/raw_feedback` for monitoring
- `/enable`, `/disable`, `/power_on`, `/power_off`, and `/zero`
- `/single_joint_command` for a cautious first motion test
- `/joint_command` for full 7-joint absolute commands

The runtime is split into two processes inside one container:

- a raw CycloneDDS transport process for the D1 arm
- a ROS 2 bridge process for desktop communication

That split keeps the project CycloneDDS-only while avoiding the DDS participant conflict that occurs when both sides share one process.

## Network Contract

Default GO2-W network layout:

- Robot `eth0`: D1 arm DDS transport side
- Robot `wlan0`: ROS 2 DDS side for desktop communication
- Desktop example `enp97s0`: WiFi NIC used to talk to the robot

Default DDS settings:

- `ROS_DOMAIN_ID=0`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Robot-side DDS configuration is handled inside the container. You do not need to source host-side DDS setup on the robot for normal use of this repository.

If your interface names differ from `eth0` and `wlan0`, you can override them with:

```bash
UNITREE_NETWORK_INTERFACE=<robot-arm-nic> ROS_NETWORK_INTERFACE=<robot-ros-nic> make up
```

## Repository Setup

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/koki67/go2w_d1_arm.git
cd go2w_d1_arm
```

If you already cloned the repository without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

The repository uses two official Eclipse CycloneDDS submodules for the raw D1 transport process:

- `third_party/cyclonedds`
- `third_party/cyclonedds-cxx`

This repository does not redistribute Unitree's D1 arm SDK package.

For the robot-side transport, download `D1Arm_services` from Unitree and place it at:

```bash
third_party/d1_sdk
```

The local package should contain at least:

```bash
third_party/d1_sdk/src/msg/ArmString_.hpp
third_party/d1_sdk/src/msg/PubServoInfo_.hpp
```

Download source:

- https://support.unitree.com/home/en/developer/D1Arm_services

`third_party/d1_sdk` is ignored by Git so each user provides their own local copy.

If you build outside Docker, you can also point CMake at a local SDK checkout with either:

```bash
export D1_SDK_ROOT=/absolute/path/to/d1_sdk
```

or:

```bash
colcon build --packages-select go2w_d1_arm --cmake-args -DD1_SDK_ROOT=/absolute/path/to/d1_sdk
```

Full non-Docker transport builds also require CycloneDDS plus CycloneDDS-CXX to be installed locally. The default robot workflow avoids that extra setup by building those dependencies inside Docker.

## Robot Quick Start

Before `make build`, make sure you have:

- initialized Git submodules
- downloaded Unitree's `D1Arm_services` package into `third_party/d1_sdk`

Build the container image:

```bash
make build
```

Start the robot-side bridge:

```bash
make up
```

Useful robot-side commands:

```bash
make ps
make logs
make doctor
make shell
make down
```

What each command does:

- `make up`: starts the long-running bridge container
- `make ps`: shows whether the container is still running
- `make logs`: follows the bridge logs
- `make doctor`: runs preflight checks without launching the bridge
- `make shell`: opens a shell inside the running container
- `make down`: stops and removes the container

The bridge is healthy when:

- `make ps` shows the container as `Up`
- `make logs` shows both the raw D1 transport and the ROS 2 bridge starting
- the logs contain `transport client connected`

`make build` compiles the official CycloneDDS core and CycloneDDS-CXX dependencies from `third_party/cyclonedds` and `third_party/cyclonedds-cxx` inside the Docker image, so no Unitree SDK bundle is used for DDS support.

## Desktop PC Quick Start

### Minimal Setup

If you only need:

- `/joint_states`
- `/enable`
- `/disable`
- `/power_on`
- `/power_off`
- `/zero`
- `/joint_command`

then the desktop only needs standard ROS 2 packages plus CycloneDDS.

Example desktop shell setup:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp97s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

Basic communication checks:

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 service call /enable std_srvs/srv/Trigger "{}"
```

If those work, the desktop-to-robot DDS path is up.

### If You Want `/single_joint_command`

`/single_joint_command` uses the custom service type `go2w_d1_arm/srv/SingleJointCommand`, so the desktop must also have this package built locally.

Add this repository to a desktop ROS 2 workspace:

```bash
<your_workspace>/src/go2w_d1_arm
```

If you use the [`go2w-docker-dev`](https://github.com/koki67/go2w-docker-dev) desktop container, the typical path is:

```bash
/workspace/ros2_ws/src/go2w_d1_arm
```

Then build it from the workspace root, not from `src`:

```bash
cd <your_workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select go2w_d1_arm --cmake-args -DGO2W_D1_ARM_BUILD_TRANSPORT=OFF
source install/setup.bash
```

That desktop build path keeps the custom ROS 2 interfaces and bridge code, but skips the raw D1 transport executable, so the desktop does not need a local `D1Arm_services` download just to call `/single_joint_command`.

Quick sanity check:

```bash
ros2 interface show go2w_d1_arm/srv/SingleJointCommand
```

If that command works, the desktop can call `/single_joint_command`.

## First End-to-End Checks

Recommended first verification sequence from the desktop:

1. Confirm discovery:

```bash
ros2 topic list
```

2. Confirm live arm state:

```bash
ros2 topic echo /joint_states
```

3. Enable motion:

```bash
ros2 service call /enable std_srvs/srv/Trigger "{}"
```

4. Try a very small single-joint motion:

```bash
ros2 service call /single_joint_command go2w_d1_arm/srv/SingleJointCommand "{joint_id: 5, angle_rad: 0.05, delay_ms: 500}"
```

5. Then try a small full-arm command.

The safest way to test `/joint_command` is:

- first read the current pose from `/joint_states`
- then publish a full 7-joint command that keeps six joints near the current pose and changes only one joint slightly

Example full-arm command shape:

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: ['d1_joint_0', 'd1_joint_1', 'd1_joint_2', 'd1_joint_3', 'd1_joint_4', 'd1_joint_5', 'd1_joint_6'], position: [-0.0646, -1.5813, 1.5813, -0.0576, 0.1484, 0.05, -0.5376]}"
```

That example came from a real observed pose and adjusts joint 5 only slightly. Do not assume those exact angles are safe for every arm state. Replace them with the current pose you actually read from `/joint_states`.

While testing motion, keep the robot logs open:

```bash
make logs
```

You can also monitor:

```bash
ros2 topic echo /arm_status
ros2 topic echo /raw_feedback
```

## ROS Interface Summary

Published topics:

- `/joint_states` : `sensor_msgs/msg/JointState`
- `/arm_status` : `go2w_d1_arm/msg/D1ArmStatus`
- `/raw_feedback` : `std_msgs/msg/String`

Subscribed topics:

- `/joint_command` : `sensor_msgs/msg/JointState`

Services:

- `/enable` : `std_srvs/srv/Trigger`
- `/disable` : `std_srvs/srv/Trigger`
- `/power_on` : `std_srvs/srv/Trigger`
- `/power_off` : `std_srvs/srv/Trigger`
- `/zero` : `std_srvs/srv/Trigger`
- `/single_joint_command` : `go2w_d1_arm/srv/SingleJointCommand`

## Default Motion Behavior

Default parameters are defined in `config/d1_arm_params.yaml`:

- `require_enable_before_motion: true`
- `enable_on_start: false`
- `zero_on_start: false`
- `multi_joint_mode: 1`
- `lock_force: 80000`

Important default behavior:

- Motion commands are gated until `/enable` succeeds.
- The bridge does not auto-enable or auto-zero on startup.
- `/joint_command` sends absolute 7-joint commands.

## Troubleshooting

### `make shell` says the service is not running

The bridge container likely exited after startup.

Check:

```bash
make ps
make logs
```

### Desktop sees standard topics but `/single_joint_command` says the service type is invalid

The desktop does not have the custom `go2w_d1_arm` interface package installed locally.

Build and source this repository in the desktop workspace, then retry.

### `Unknown package 'go2w_d1_arm'` after a desktop build

You probably built from the wrong directory.

Wrong:

```bash
cd <workspace>/src
colcon build --packages-select go2w_d1_arm
```

Correct:

```bash
cd <workspace>
colcon build --packages-select go2w_d1_arm
source install/setup.bash
```

### `make doctor` fails because an interface is missing

The default robot contract expects:

- `eth0` for the D1 side
- `wlan0` for the ROS 2 side

If your robot uses different interface names, override them when launching:

```bash
UNITREE_NETWORK_INTERFACE=<robot-arm-nic> ROS_NETWORK_INTERFACE=<robot-ros-nic> make doctor
UNITREE_NETWORK_INTERFACE=<robot-arm-nic> ROS_NETWORK_INTERFACE=<robot-ros-nic> make up
```

### `make build` fails because `d1_sdk` is missing

The robot-side Docker build requires a user-provided local copy of Unitree's `D1Arm_services` package.

Download it from Unitree, then place it at:

```bash
third_party/d1_sdk
```

Make sure these files exist before rebuilding:

```bash
third_party/d1_sdk/src/msg/ArmString_.hpp
third_party/d1_sdk/src/msg/PubServoInfo_.hpp
```

### `make build` fails because `cyclonedds` or `cyclonedds-cxx` is missing

The repository expects the official CycloneDDS submodules to be initialized locally.

Run:

```bash
git submodule update --init --recursive
```

## Notes

- The robot-side container handles the CycloneDDS setup internally.
- The desktop side must use the same `ROS_DOMAIN_ID`.
- Latest builds suppress warning noise from third-party DDS headers so the build output stays readable.
