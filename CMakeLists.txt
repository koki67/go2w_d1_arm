cmake_minimum_required(VERSION 3.8)
project(go2w_d1_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
  set(UNITREE_THIRDPARTY_ARCH aarch64)
else()
  set(UNITREE_THIRDPARTY_ARCH x86_64)
endif()

find_path(
  DDSCXX_INCLUDE_DIR
  dds/topic/TopicTraits.hpp
  PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2/thirdparty/include/ddscxx"
    /usr/include/ddscxx
)

find_path(
  DDSC_INCLUDE_DIR
  dds/dds.h
  PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2/thirdparty/include"
    /usr/include
)

find_library(
  DDSCXX_LIBRARY
  NAMES ddscxx
  PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2/thirdparty/lib/${UNITREE_THIRDPARTY_ARCH}"
    /usr/lib
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
)

find_library(
  DDSC_LIBRARY
  NAMES ddsc
  PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2/thirdparty/lib/${UNITREE_THIRDPARTY_ARCH}"
    /usr/lib
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
)

if(NOT DDSCXX_INCLUDE_DIR OR NOT DDSC_INCLUDE_DIR OR NOT DDSCXX_LIBRARY OR NOT DDSC_LIBRARY)
  message(FATAL_ERROR "CycloneDDS C/C++ headers or libraries were not found. Populate third_party/unitree_sdk2 or install CycloneDDS development packages.")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/D1ArmStatus.msg"
  "srv/SingleJointCommand.srv"
  DEPENDENCIES std_msgs
)

add_executable(d1_arm_bridge_node
  src/d1_arm_bridge_node.cpp
  src/msg/ArmString_.cpp
  src/msg/PubServoInfo_.cpp
)

add_executable(d1_arm_transport
  src/d1_arm_transport.cpp
  src/msg/ArmString_.cpp
  src/msg/PubServoInfo_.cpp
)

foreach(target_name d1_arm_bridge_node d1_arm_transport)
  target_compile_features(${target_name} PRIVATE cxx_std_17)
  target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${DDSCXX_INCLUDE_DIR}
    ${DDSC_INCLUDE_DIR}
  )
  target_link_libraries(${target_name}
    ${DDSCXX_LIBRARY}
    ${DDSC_LIBRARY}
    Threads::Threads
  )
  get_filename_component(DDS_LIBRARY_DIR "${DDSCXX_LIBRARY}" DIRECTORY)
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${DDS_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN"
  )
endforeach()

ament_target_dependencies(d1_arm_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(d1_arm_bridge_node
  ${cpp_typesupport_target}
)
target_link_libraries(d1_arm_transport
  ${cpp_typesupport_target}
)

install(TARGETS d1_arm_bridge_node d1_arm_transport
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${DDSCXX_LIBRARY}
  ${DDSC_LIBRARY}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
