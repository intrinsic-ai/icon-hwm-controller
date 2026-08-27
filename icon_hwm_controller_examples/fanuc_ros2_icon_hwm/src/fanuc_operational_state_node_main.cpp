#include <memory>

#include "fanuc_operational_state_node/fanuc_operational_state_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fanuc_operational_state_node::FanucOperationalStateNode>();

  // Use MultiThreadedExecutor to support concurrent service handling.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
