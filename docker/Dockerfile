FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      iproute2 \
      python3-colcon-common-extensions \
      ros-humble-ament-cmake \
      ros-humble-cyclonedds \
      ros-humble-launch \
      ros-humble-launch-ros \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-default-generators \
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

COPY . /ros2_ws/src/go2w_d1_arm

WORKDIR /ros2_ws

RUN /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select go2w_d1_arm \
    "

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/runtime_common.sh /runtime_common.sh
COPY docker/setup_condition.sh /setup_condition.sh
COPY docker/start_bridge.sh /start_bridge.sh
COPY docker/doctor.sh /doctor.sh
RUN chmod +x /entrypoint.sh /runtime_common.sh /setup_condition.sh /start_bridge.sh /doctor.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start_bridge.sh"]
