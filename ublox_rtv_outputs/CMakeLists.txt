cmake_minimum_required(VERSION 3.22)
project(ublox_rtv_outputs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtv_gnss_utils REQUIRED) # rtv_gnss dius git repo
find_package(sensor_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)

# conversions + epoch assembler, no node: shared by the component and the tests
add_library(ublox_rtv_outputs_core STATIC
  src/conversions.cpp
  src/epoch_assembler.cpp
)
set_target_properties(ublox_rtv_outputs_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ublox_rtv_outputs_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ublox_rtv_outputs_core PUBLIC
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${gps_msgs_TARGETS}
  ${rtv_gnss_utils_TARGETS}
  ${sensor_msgs_TARGETS}
  ${ublox_ubx_msgs_TARGETS}
)

add_library(ublox_rtv_outputs_components SHARED src/rtv_outputs_node.cpp)
target_link_libraries(ublox_rtv_outputs_components PUBLIC
  ublox_rtv_outputs_core
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rclcpp_components::component
)
rclcpp_components_register_node(ublox_rtv_outputs_components
  PLUGIN "ublox_rtv_outputs::RtvOutputsNode"
  EXECUTABLE rtv_outputs)

install(TARGETS ublox_rtv_outputs_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ublox_rtv_outputs_core)
  ament_add_gtest(test_epoch_assembler test/test_epoch_assembler.cpp)
  target_link_libraries(test_epoch_assembler ublox_rtv_outputs_core)
endif()

ament_package()
