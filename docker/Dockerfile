FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      iproute2 \
      python3-colcon-common-extensions \
      ros-humble-ament-cmake \
      ros-humble-launch \
      ros-humble-launch-ros \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-rosidl-default-generators \
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

COPY third_party/unitree_sdk2/ /tmp/unitree_sdk2/
COPY docker/patch_unitree_sdk2_dds_entity.py /tmp/patch_unitree_sdk2_dds_entity.py

RUN rm -rf /tmp/unitree_sdk2/build /tmp/unitree_sdk2_build && \
    python3 /tmp/patch_unitree_sdk2_dds_entity.py /tmp/unitree_sdk2/include/unitree/common/dds/dds_entity.hpp && \
    mkdir -p /tmp/unitree_sdk2_build && \
    cd /tmp/unitree_sdk2_build && \
    cmake /tmp/unitree_sdk2 -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF && \
    cmake --build . --parallel "$(nproc)" && \
    cmake --install . && \
    rm -rf /tmp/unitree_sdk2 /tmp/unitree_sdk2_build /tmp/patch_unitree_sdk2_dds_entity.py

ENV CMAKE_PREFIX_PATH=/opt/unitree_robotics:${CMAKE_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}

COPY . /ros2_ws/src/go2w_d1_arm

WORKDIR /ros2_ws

RUN /bin/bash -lc "\
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select go2w_d1_arm \
    "

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/runtime_common.sh /runtime_common.sh
COPY docker/start_bridge.sh /start_bridge.sh
COPY docker/doctor.sh /doctor.sh
RUN chmod +x /entrypoint.sh /runtime_common.sh /start_bridge.sh /doctor.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start_bridge.sh"]

