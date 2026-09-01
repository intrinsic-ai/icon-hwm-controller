# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(ur_ros2_icon_hwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20 standard.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(icon_hwm_controller_msgs REQUIRED)
find_package(icon_shared_memory_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

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
  icon_shared_memory_icon_utils_mutex
  icon_shared_memory_icon_utils_status
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${ur_dashboard_msgs_TARGETS}
)
add_executable(
  ur_operational_state_node
  src/ur_operational_state_node_main.cpp
)
target_link_libraries(ur_operational_state_node PUBLIC
  ur_operational_state
  ${icon_hwm_controller_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${ur_dashboard_msgs_TARGETS}
)

install(TARGETS
  ur_operational_state
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ur_operational_state_node
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

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gmock(ur_operational_state_test
    test/ur_operational_state_node_test.cpp
  )
  target_include_directories(ur_operational_state_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(ur_operational_state_test
    ur_operational_state
    ${icon_hwm_controller_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${ur_dashboard_msgs_TARGETS}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(ur_operational_state)
ament_export_dependencies(
  icon_hwm_controller_msgs
  icon_shared_memory_vendor
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  ur_dashboard_msgs
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
