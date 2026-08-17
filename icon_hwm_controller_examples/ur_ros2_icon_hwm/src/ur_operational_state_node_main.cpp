#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ur_operational_state_node/ur_operational_state_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ur_operational_state_node::UrOperationalStateNode>();

  // Use MultiThreadedExecutor to support concurrent service and action handling.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
