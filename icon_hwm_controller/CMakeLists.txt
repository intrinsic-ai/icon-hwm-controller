cmake_minimum_required(VERSION 3.19)
project(icon_hwm_controller CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20 standard.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(icon_hwm_controller_msgs REQUIRED)
find_package(icon_shared_memory_vendor REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(icon_hwm_controller_parameters
  src/icon_hwm_controller_parameters.yaml
)

add_library(icon_hwm_controller SHARED
  src/icon_hwm_controller.cpp
  src/ros2_hwm_impl.cpp
)

set_property(
  TARGET icon_hwm_controller
  PROPERTY POSITION_INDEPENDENT_CODE ON
)
target_include_directories(icon_hwm_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(icon_hwm_controller PUBLIC
  icon_hwm_controller_parameters
  controller_interface::controller_interface
  ${controller_manager_msgs_TARGETS}
  hardware_interface::hardware_interface
  ${icon_hwm_controller_msgs_TARGETS}
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_strerror
  icon_shared_memory_icon_utils_time
  icon_shared_memory_icon_hal_interfaces_joint_command_fbs_utils
  icon_shared_memory_icon_hal_interfaces_joint_state_fbs_utils
  icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils
  icon_shared_memory_icon_hal_interfaces_hardware_module_state_fbs_utils
  icon_shared_memory_icon_hal_hardware_interface_traits
  icon_shared_memory_icon_hal_hardware_interface_registry
  icon_shared_memory_icon_hal_hardware_module_interface
  icon_shared_memory_icon_hal_hardware_module_runtime
  icon_shared_memory_icon_hal_icon_state_register
  icon_shared_memory_icon_hal_realtime_clock
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server
  icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server
  icon_shared_memory_icon_interprocess_shared_memory_lockstep
  ${lifecycle_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tl::expected
)

pluginlib_export_plugin_description_file(controller_interface
  icon_hwm_controller.xml
)

install(TARGETS
  icon_hwm_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(icon_hwm_controller)
ament_export_dependencies(
  flatbuffers_vendor
  tl_expected
  icon_shared_memory_vendor
  eigen_vendor
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_manager_msgs
  lifecycle_msgs
  std_msgs
  std_srvs
  generate_parameter_library
  icon_hwm_controller_msgs
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_arguments test/test_launch_arguments.py)
endif()

ament_package()
