#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "fanuc_msgs/msg/robot_status.hpp"
#include "fanuc_operational_state_node/fanuc_operational_state_node.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"

using fanuc_operational_state_node::EvaluateOperationalStatus;
using fanuc_operational_state_node::FanucOperationalStateNode;
using fanuc_operational_state_node::RobotStateInputs;
using icon_hwm_controller_msgs::msg::OperationalStatus;

// Unit tests for FANUC Operational Status evaluation and node.
// Reference: https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
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
  RobotStateInputs inputs;
  inputs.in_error = std::nullopt;
  inputs.e_stopped = std::nullopt;
  inputs.motion_possible = std::nullopt;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(FanucOperationalStateNodeTest, EnabledInNormalOperation)
{
  // Documented normal operation:
  // in_error: false, tp_enabled: false, e_stopped: false, motion_possible: true, contact_stop_mode: 0
  RobotStateInputs inputs;
  inputs.in_error = false;
  inputs.tp_enabled = false;
  inputs.e_stopped = false;
  inputs.motion_possible = true;
  inputs.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(FanucOperationalStateNodeTest, FaultedWhenEStopped)
{
  // Documented e-stop state:
  // in_error: true, tp_enabled: false, e_stopped: true, motion_possible: false, contact_stop_mode: 0
  RobotStateInputs inputs;
  inputs.in_error = true;
  inputs.tp_enabled = false;
  inputs.e_stopped = true;
  inputs.motion_possible = false;
  inputs.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("emergency stop"), std::string::npos);
}

TEST_F(FanucOperationalStateNodeTest, FaultedWhenEStopReleasedBeforeReset)
{
  // Documented released e-stop (faults unacknowledged) state:
  // in_error: true, tp_enabled: false, e_stopped: false, motion_possible: false, contact_stop_mode: 0
  RobotStateInputs inputs;
  inputs.in_error = true;
  inputs.tp_enabled = false;
  inputs.e_stopped = false;
  inputs.motion_possible = false;
  inputs.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("error/alarm"), std::string::npos);
}

TEST_F(FanucOperationalStateNodeTest, DisabledWhenTeachPendantEnabled)
{
  RobotStateInputs inputs;
  inputs.in_error = false;
  inputs.tp_enabled = true;
  inputs.e_stopped = false;
  inputs.motion_possible = false;
  inputs.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(FanucOperationalStateNodeTest, FaultedWhenContactStop)
{
  RobotStateInputs inputs;
  inputs.in_error = false;
  inputs.tp_enabled = false;
  inputs.e_stopped = false;
  inputs.motion_possible = true;
  inputs.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_SAFE;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("contact stop"), std::string::npos);
}

TEST_F(FanucOperationalStateNodeTest, NodeConstructionAndTopicHandling)
{
  auto node = std::make_shared<FanucOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  // Initial state is UNKNOWN
  auto status = node->GetLatestStatus();
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);

  // Helper publisher
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

  // Publish normal RobotStatus
  fanuc_msgs::msg::RobotStatus rs;
  rs.in_error = false;
  rs.tp_enabled = false;
  rs.e_stopped = false;
  rs.motion_possible = true;
  rs.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE;
  robot_status_pub->publish(rs);

  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetLatestStatus().state, OperationalStatus::ENABLED);
}
