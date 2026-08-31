#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ur_ros2_icon_hwm/ur_operational_state_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ur_ros2_icon_hwm::UrOperationalStateNode>();

  // Use MultiThreadedExecutor to support concurrent service and action handling.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
