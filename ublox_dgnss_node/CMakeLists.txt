cmake_minimum_required(VERSION 3.22)
project(ublox_dgnss_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(ament_cmake_uncrustify REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(ublox_ubx_interfaces REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtv_gnss_utils REQUIRED) # rtv_gnss package from dius git repo

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)

# Pass the ROS_DISTRO environment variable to the C++ preprocessor as a string
if(DEFINED ENV{ROS_DISTRO})
  add_definitions(-DROS_DISTRO_$ENV{ROS_DISTRO})
  message(STATUS "Compiling for ROS 2: $ENV{ROS_DISTRO}")
endif()

include_directories(include SYSTEM)

add_library(ublox_dgnss_components SHARED
  src/usb.cpp
  src/ublox_dgnss_node.cpp
  src/parameters.cpp
  src/device_family.cpp
  src/ubx_config_loader.cpp
  src/stream_parser.cpp
  src/pty_link.cpp
)

target_compile_definitions(ublox_dgnss_components
  PRIVATE "UBLOX_DGNSS_NODE_BUILDING_DLL"
)

# TODO(review): Original commented out: it hid definitions in headers duplicated in every .cpp
# (ubxKeyCfgItemMap was destroyed once per .cpp at exit); they are inline now, and the linker
# should report the next one
# target_link_options(ublox_dgnss_components PRIVATE
#   "LINKER:--allow-multiple-definition"
# )

target_link_libraries(ublox_dgnss_components PUBLIC
  ${nmea_msgs_TARGETS}
  ${rtcm_msgs_TARGETS}
  ${rtv_gnss_utils_TARGETS}
  ${std_msgs_TARGETS}
  ${ublox_ubx_interfaces_TARGETS}
  ${ublox_ubx_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  usb-1.0
)

rclcpp_components_register_node(ublox_dgnss_components
  PLUGIN "ublox_dgnss::UbloxDGNSSNode"
  EXECUTABLE ublox_dgnss_node)

install(TARGETS
  ublox_dgnss_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stream_parser test/test_stream_parser.cpp src/stream_parser.cpp)
  target_include_directories(test_stream_parser PRIVATE include)
  ament_add_gtest(test_pty_link test/test_pty_link.cpp src/pty_link.cpp)
  target_include_directories(test_pty_link PRIVATE include)
endif()

ament_package()
