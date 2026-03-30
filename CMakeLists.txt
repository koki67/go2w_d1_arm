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

set(GO2W_D1_ARM_BUILD_TRANSPORT "AUTO" CACHE STRING
  "Build the raw D1 DDS transport target. Valid values: AUTO, ON, OFF.")
set_property(CACHE GO2W_D1_ARM_BUILD_TRANSPORT PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${GO2W_D1_ARM_BUILD_TRANSPORT}" GO2W_D1_ARM_BUILD_TRANSPORT)

if(NOT GO2W_D1_ARM_BUILD_TRANSPORT MATCHES "^(AUTO|ON|OFF)$")
  message(FATAL_ERROR
    "GO2W_D1_ARM_BUILD_TRANSPORT must be one of AUTO, ON, or OFF.")
endif()

function(resolve_d1_sdk_root out_var)
  foreach(candidate IN LISTS ARGN)
    if(candidate
      AND EXISTS "${candidate}/src/msg/ArmString_.hpp"
      AND EXISTS "${candidate}/src/msg/ArmString_.cpp"
      AND EXISTS "${candidate}/src/msg/PubServoInfo_.hpp"
      AND EXISTS "${candidate}/src/msg/PubServoInfo_.cpp")
      file(REAL_PATH "${candidate}" resolved_candidate)
      set(${out_var} "${resolved_candidate}" PARENT_SCOPE)
      return()
    endif()
  endforeach()

  set(${out_var} "" PARENT_SCOPE)
endfunction()

set(D1_SDK_SEARCH_ROOTS)
if(DEFINED D1_SDK_ROOT AND NOT "${D1_SDK_ROOT}" STREQUAL "")
  list(APPEND D1_SDK_SEARCH_ROOTS "${D1_SDK_ROOT}")
endif()
if(DEFINED ENV{D1_SDK_ROOT} AND NOT "$ENV{D1_SDK_ROOT}" STREQUAL "")
  list(APPEND D1_SDK_SEARCH_ROOTS "$ENV{D1_SDK_ROOT}")
endif()
list(APPEND D1_SDK_SEARCH_ROOTS
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/d1_sdk"
)
list(REMOVE_DUPLICATES D1_SDK_SEARCH_ROOTS)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/D1ArmStatus.msg"
  "srv/SingleJointCommand.srv"
  DEPENDENCIES std_msgs
)

add_executable(d1_arm_bridge_node
  src/d1_arm_bridge_node.cpp
)

set(GO2W_D1_ARM_TARGETS d1_arm_bridge_node)
set(GO2W_D1_ARM_TRANSPORT_ENABLED OFF)
set(DDS_RUNTIME_LIBS)

if(NOT GO2W_D1_ARM_BUILD_TRANSPORT STREQUAL "OFF")
  resolve_d1_sdk_root(D1_SDK_ROOT_RESOLVED ${D1_SDK_SEARCH_ROOTS})

  if(D1_SDK_ROOT_RESOLVED)
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
      message(FATAL_ERROR
        "CycloneDDS C/C++ headers or libraries were not found. Populate third_party/unitree_sdk2 or install CycloneDDS development packages.")
    endif()

    get_filename_component(DDS_LIBRARY_DIR "${DDSCXX_LIBRARY}" DIRECTORY)
    file(GLOB DDS_RUNTIME_LIBS
      "${DDS_LIBRARY_DIR}/libddsc.so*"
      "${DDS_LIBRARY_DIR}/libddscxx.so*"
    )

    add_executable(d1_arm_transport
      src/d1_arm_transport.cpp
      "${D1_SDK_ROOT_RESOLVED}/src/msg/ArmString_.cpp"
      "${D1_SDK_ROOT_RESOLVED}/src/msg/PubServoInfo_.cpp"
    )

    set(GO2W_D1_ARM_TRANSPORT_ENABLED ON)
    list(APPEND GO2W_D1_ARM_TARGETS d1_arm_transport)
    message(STATUS "Using local D1 SDK from ${D1_SDK_ROOT_RESOLVED}")
  elseif(GO2W_D1_ARM_BUILD_TRANSPORT STREQUAL "ON")
    string(REPLACE ";" "\n  " D1_SDK_SEARCH_ROOTS_PRETTY "${D1_SDK_SEARCH_ROOTS}")
    message(FATAL_ERROR
      "D1 SDK message files were not found.\n"
      "Download Unitree's D1Arm_services package and either:\n"
      "  1. extract it to ${CMAKE_CURRENT_SOURCE_DIR}/third_party/d1_sdk\n"
      "  2. configure with -DD1_SDK_ROOT=<path>\n"
      "  3. export D1_SDK_ROOT=<path>\n"
      "Checked paths:\n"
      "  ${D1_SDK_SEARCH_ROOTS_PRETTY}")
  else()
    message(STATUS
      "D1 SDK not found; skipping d1_arm_transport. "
      "Set GO2W_D1_ARM_BUILD_TRANSPORT=ON once a local D1 SDK is available.")
  endif()
endif()

foreach(target_name IN LISTS GO2W_D1_ARM_TARGETS)
  target_compile_features(${target_name} PRIVATE cxx_std_17)
  target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
endforeach()

if(GO2W_D1_ARM_TRANSPORT_ENABLED)
  target_include_directories(d1_arm_transport SYSTEM PRIVATE
    ${DDSCXX_INCLUDE_DIR}
    ${DDSC_INCLUDE_DIR}
    "${D1_SDK_ROOT_RESOLVED}/src"
  )
  target_link_libraries(d1_arm_transport
    ${DDSCXX_LIBRARY}
    ${DDSC_LIBRARY}
    Threads::Threads
  )
  set_target_properties(d1_arm_transport PROPERTIES
    BUILD_RPATH "${DDS_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN"
  )
endif()

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
if(GO2W_D1_ARM_TRANSPORT_ENABLED)
  target_link_libraries(d1_arm_transport
    ${cpp_typesupport_target}
  )
endif()

install(TARGETS d1_arm_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(GO2W_D1_ARM_TRANSPORT_ENABLED)
  install(TARGETS d1_arm_transport
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    ${DDS_RUNTIME_LIBS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
