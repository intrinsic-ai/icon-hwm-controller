#include <cstdlib>
#include <memory>

#include "fanuc_ros2_icon_hwm/fanuc_operational_state_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fanuc_ros2_icon_hwm::FanucOperationalStateNode>();

  // Use MultiThreadedExecutor to support concurrent service handling.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
