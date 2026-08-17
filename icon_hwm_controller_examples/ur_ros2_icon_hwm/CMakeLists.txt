cmake_minimum_required(VERSION 3.8)
project(ur_ros2_icon_hwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(icon_hwm_controller_msgs REQUIRED)

include_directories(
  include
)

add_library(
  ur_operational_state
  src/ur_operational_state_node.cpp
)
target_include_directories(ur_operational_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ur_operational_state PUBLIC
  ${icon_hwm_controller_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${ur_dashboard_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
add_executable(
  ur_operational_state_node
  src/ur_operational_state_node_main.cpp
)
target_link_libraries(ur_operational_state_node PUBLIC
  ur_operational_state
  ${icon_hwm_controller_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${ur_dashboard_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

install(TARGETS
  ur_operational_state
  ur_operational_state_node
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
  ament_add_gmock(ur_operational_state_test test/ur_operational_state_node_test.cpp)
  target_include_directories(ur_operational_state_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(ur_operational_state_test
    ur_operational_state
    ${icon_hwm_controller_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${ur_dashboard_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(ur_operational_state)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  ur_dashboard_msgs
  icon_hwm_controller_msgs
)

# Install launch and config files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
