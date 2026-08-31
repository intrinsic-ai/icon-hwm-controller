#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "fanuc_msgs/msg/robot_status.hpp"
#include "fanuc_ros2_icon_hwm/fanuc_operational_state_node.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fanuc_ros2_icon_hwm
{
namespace
{

using icon_hwm_controller_msgs::msg::OperationalStatus;

class FanucOperationalStateNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(FanucOperationalStateNodeTest, UnknownWhenMissingInputs)
{
  const std::optional<fanuc_msgs::msg::RobotStatus> status_msg = std::nullopt;

  const auto status = ToOperationalStatus(status_msg);
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(FanucOperationalStateNodeTest, EnabledInNormalOperation)
{
  const auto status_msg = fanuc_msgs::msg::RobotStatus()
    .set__in_error(false)
    .set__e_stopped(false)
    .set__tp_enabled(false)
    .set__motion_possible(true)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);

  const auto status = ToOperationalStatus(status_msg);
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(FanucOperationalStateNodeTest, FaultedWhenEStopped)
{
  const auto status_msg = fanuc_msgs::msg::RobotStatus()
    .set__in_error(true)
    .set__e_stopped(true)
    .set__tp_enabled(false)
    .set__motion_possible(false)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);

  const auto status = ToOperationalStatus(status_msg);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("emergency stop"), std::string::npos);
}

TEST_F(FanucOperationalStateNodeTest, FaultedWhenEStopReleasedBeforeReset)
{
  const auto status_msg = fanuc_msgs::msg::RobotStatus()
    .set__in_error(true)
    .set__e_stopped(false)
    .set__tp_enabled(false)
    .set__motion_possible(false)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);

  const auto status = ToOperationalStatus(status_msg);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("error/alarm"), std::string::npos);
}

TEST_F(FanucOperationalStateNodeTest, DisabledWhenTeachPendantEnabled)
{
  const auto status_msg = fanuc_msgs::msg::RobotStatus()
    .set__in_error(false)
    .set__e_stopped(false)
    .set__tp_enabled(true)
    .set__motion_possible(false)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);

  const auto status = ToOperationalStatus(status_msg);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(FanucOperationalStateNodeTest, FaultedWhenContactStop)
{
  const auto status_msg = fanuc_msgs::msg::RobotStatus()
    .set__in_error(false)
    .set__e_stopped(false)
    .set__tp_enabled(false)
    .set__motion_possible(true)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_SAFE);

  const auto status = ToOperationalStatus(status_msg);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("contact stop"), std::string::npos);
}

TEST_F(FanucOperationalStateNodeTest, NodeConstructionAndTopicHandling)
{
  auto node = std::make_shared<FanucOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  // Initial state should be unknown.
  const auto status = node->GetOperationalStatus();
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);

  auto test_pub_node = std::make_shared<rclcpp::Node>("test_pub_node");
  auto robot_status_pub = test_pub_node->create_publisher<fanuc_msgs::msg::RobotStatus>(
    "/fanuc_gpio_controller/robot_status", 10);

  OperationalStatus received_status;
  std::atomic_bool status_received = false;
  auto status_sub = test_pub_node->create_subscription<OperationalStatus>(
    "operational_status", 10,
    [&](const OperationalStatus::SharedPtr msg) {
      received_status = *msg;
      status_received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_pub_node);

  // Publish enabled robot status.
  const auto rs = fanuc_msgs::msg::RobotStatus()
    .set__in_error(false)
    .set__e_stopped(false)
    .set__tp_enabled(false)
    .set__motion_possible(true)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);
  robot_status_pub->publish(rs);

  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetOperationalStatus().state, OperationalStatus::ENABLED);
}

TEST_F(FanucOperationalStateNodeTest, FaultTransitionTopicHandling)
{
  auto node = std::make_shared<FanucOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  auto test_pub_node = std::make_shared<rclcpp::Node>("fault_test_pub_node");
  auto robot_status_pub = test_pub_node->create_publisher<fanuc_msgs::msg::RobotStatus>(
    "/fanuc_gpio_controller/robot_status", 10);

  OperationalStatus received_status;
  std::atomic_bool status_received = false;
  auto status_sub = test_pub_node->create_subscription<OperationalStatus>(
    "operational_status", 10,
    [&](const OperationalStatus::SharedPtr msg) {
      received_status = *msg;
      status_received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_pub_node);

  // Publish faulted robot status.
  const auto rs = fanuc_msgs::msg::RobotStatus()
    .set__in_error(true)
    .set__e_stopped(false)
    .set__tp_enabled(false)
    .set__motion_possible(false)
    .set__contact_stop_mode(fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);
  robot_status_pub->publish(rs);

  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetOperationalStatus().state, OperationalStatus::FAULTED);
}

}  // namespace
}  // namespace fanuc_ros2_icon_hwm
