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
    if [ ! -f /ros2_ws/src/go2w_d1_arm/third_party/cyclonedds/CMakeLists.txt ]; then \
      echo 'Missing third_party/cyclonedds. Run git submodule update --init --recursive before make build.' >&2; \
      exit 1; \
    fi && \
    if [ ! -f /ros2_ws/src/go2w_d1_arm/third_party/cyclonedds-cxx/CMakeLists.txt ]; then \
      echo 'Missing third_party/cyclonedds-cxx. Run git submodule update --init --recursive before make build.' >&2; \
      exit 1; \
    fi && \
    if [ ! -f /ros2_ws/src/go2w_d1_arm/third_party/d1_sdk/src/msg/ArmString_.hpp ] || \
       [ ! -f /ros2_ws/src/go2w_d1_arm/third_party/d1_sdk/src/msg/PubServoInfo_.hpp ]; then \
      echo 'Missing third_party/d1_sdk. Download Unitree D1Arm_services and place it under third_party/d1_sdk before running make build.' >&2; \
      exit 1; \
    fi && \
    cmake -S /ros2_ws/src/go2w_d1_arm/third_party/cyclonedds \
      -B /tmp/cyclonedds-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_TESTING=OFF \
      -DBUILD_IDLC=OFF \
      -DBUILD_DDSPERF=OFF && \
    cmake --build /tmp/cyclonedds-build --parallel && \
    cmake --install /tmp/cyclonedds-build && \
    cmake -S /ros2_ws/src/go2w_d1_arm/third_party/cyclonedds-cxx \
      -B /tmp/cyclonedds-cxx-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_PREFIX_PATH=/usr/local \
      -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_IDLLIB=OFF && \
    cmake --build /tmp/cyclonedds-cxx-build --parallel && \
    cmake --install /tmp/cyclonedds-cxx-build && \
    export CMAKE_PREFIX_PATH=/usr/local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}} && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select go2w_d1_arm --cmake-args -DGO2W_D1_ARM_BUILD_TRANSPORT=ON \
    "

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/runtime_common.sh /runtime_common.sh
COPY docker/setup_condition.sh /setup_condition.sh
COPY docker/start_bridge.sh /start_bridge.sh
COPY docker/doctor.sh /doctor.sh
RUN chmod +x /entrypoint.sh /runtime_common.sh /setup_condition.sh /start_bridge.sh /doctor.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start_bridge.sh"]
