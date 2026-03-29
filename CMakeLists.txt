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

find_package(unitree_sdk2 CONFIG QUIET)
if(NOT unitree_sdk2_FOUND)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2/CMakeLists.txt")
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/unitree_sdk2 EXCLUDE_FROM_ALL)
  else()
    message(FATAL_ERROR "unitree_sdk2 not found. Populate third_party/unitree_sdk2 or install /opt/unitree_robotics.")
  endif()
endif()

find_path(
  DDSCXX_INCLUDE_DIR
  dds/topic/TopicTraits.hpp
  PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2/thirdparty/include/ddscxx"
    /opt/unitree_robotics/include/ddscxx
    /usr/local/include/ddscxx
)

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

target_compile_features(d1_arm_bridge_node PRIVATE cxx_std_17)
target_include_directories(d1_arm_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(DDSCXX_INCLUDE_DIR)
  target_include_directories(d1_arm_bridge_node PRIVATE ${DDSCXX_INCLUDE_DIR})
endif()

ament_target_dependencies(d1_arm_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(d1_arm_bridge_node
  unitree_sdk2
  ${cpp_typesupport_target}
)

install(TARGETS d1_arm_bridge_node
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

