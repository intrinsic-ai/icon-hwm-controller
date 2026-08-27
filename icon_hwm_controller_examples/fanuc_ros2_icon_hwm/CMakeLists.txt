cmake_minimum_required(VERSION 3.8)
project(fanuc_ros2_icon_hwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(fanuc_msgs REQUIRED)
find_package(icon_hwm_controller_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_library(
  fanuc_operational_state
  src/fanuc_operational_state_node.cpp
)
target_include_directories(fanuc_operational_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fanuc_operational_state PUBLIC
  ${fanuc_msgs_TARGETS}
  ${icon_hwm_controller_msgs_TARGETS}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

add_executable(
  fanuc_operational_state_node
  src/fanuc_operational_state_node_main.cpp
)
target_link_libraries(fanuc_operational_state_node PUBLIC
  fanuc_operational_state
  ${fanuc_msgs_TARGETS}
  ${icon_hwm_controller_msgs_TARGETS}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

install(TARGETS
  fanuc_operational_state
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  fanuc_operational_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(fanuc_operational_state_test test/fanuc_operational_state_node_test.cpp)
  target_include_directories(fanuc_operational_state_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(fanuc_operational_state_test
    fanuc_operational_state
    ${fanuc_msgs_TARGETS}
    ${icon_hwm_controller_msgs_TARGETS}
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(fanuc_operational_state)
ament_export_dependencies(
  fanuc_msgs
  icon_hwm_controller_msgs
  rclcpp
  std_msgs
  std_srvs
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()