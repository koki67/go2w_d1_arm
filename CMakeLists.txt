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

function(get_imported_library_location target out_var)
  foreach(prop
      IMPORTED_LOCATION
      IMPORTED_LOCATION_RELEASE
      IMPORTED_LOCATION_RELWITHDEBINFO
      IMPORTED_LOCATION_MINSIZEREL
      IMPORTED_LOCATION_DEBUG)
    get_target_property(candidate "${target}" "${prop}")
    if(candidate AND NOT candidate STREQUAL "candidate-NOTFOUND")
      set(${out_var} "${candidate}" PARENT_SCOPE)
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
    find_package(CycloneDDS REQUIRED)
    find_package(CycloneDDS-CXX REQUIRED)

    get_imported_library_location(CycloneDDS::ddsc DDSC_LIBRARY)
    get_imported_library_location(CycloneDDS-CXX::ddscxx DDSCXX_LIBRARY)

    if(NOT DDSC_LIBRARY OR NOT DDSCXX_LIBRARY)
      message(FATAL_ERROR
        "CycloneDDS or CycloneDDS-CXX library locations could not be resolved. "
        "Install CycloneDDS plus CycloneDDS-CXX and ensure CMAKE_PREFIX_PATH points to them.")
    endif()

    get_filename_component(DDSC_LIBRARY_DIR "${DDSC_LIBRARY}" DIRECTORY)
    get_filename_component(DDSCXX_LIBRARY_DIR "${DDSCXX_LIBRARY}" DIRECTORY)
    file(GLOB DDS_RUNTIME_LIBS
      "${DDSC_LIBRARY_DIR}/libddsc.so*"
      "${DDSCXX_LIBRARY_DIR}/libddscxx.so*"
    )
    list(REMOVE_DUPLICATES DDS_RUNTIME_LIBS)

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
  target_include_directories(d1_arm_transport PRIVATE
    "${D1_SDK_ROOT_RESOLVED}/src"
  )
  target_link_libraries(d1_arm_transport
    CycloneDDS-CXX::ddscxx
    Threads::Threads
  )
  set_target_properties(d1_arm_transport PROPERTIES
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
