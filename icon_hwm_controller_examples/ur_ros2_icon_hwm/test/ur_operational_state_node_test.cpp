#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_operational_state_node/ur_operational_state_node.hpp"

using icon_hwm_controller_msgs::msg::OperationalStatus;
using ur_operational_state_node::EvaluateOperationalStatus;
using ur_operational_state_node::RobotStateInputs;
using ur_operational_state_node::UrOperationalStateNode;

class OperationalStateNodeTest : public ::testing::Test {
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

TEST_F(OperationalStateNodeTest, UnknownWhenMissingInputs) {
  RobotStateInputs inputs;
  inputs.robot_mode = std::nullopt;
  inputs.safety_mode = std::nullopt;
  inputs.program_running = std::nullopt;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);
  EXPECT_FALSE(status.message.empty());

  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);
}

TEST_F(OperationalStateNodeTest, EnabledWhenRunningAndProgramPlaying) {
  RobotStateInputs inputs;
  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  inputs.program_running = true;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());

  // Also enabled when in REDUCED safety mode and program running.
  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::REDUCED;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(OperationalStateNodeTest, DisabledWhenRunningButProgramNotPlaying) {
  RobotStateInputs inputs;
  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  inputs.program_running = false;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(OperationalStateNodeTest, DisabledWhenPowerOffOrIdle) {
  RobotStateInputs inputs;
  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  inputs.program_running = false;

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::POWER_OFF;
  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::IDLE;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::POWER_ON;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::BOOTING;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::BACKDRIVE;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(OperationalStateNodeTest, FaultedWhenProtectiveStop) {
  RobotStateInputs inputs;
  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP;
  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  inputs.program_running = true;

  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("protective stop"), std::string::npos);
}

TEST_F(OperationalStateNodeTest, FaultedWhenEmergencyStop) {
  RobotStateInputs inputs;
  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  inputs.program_running = true;

  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP;
  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(OperationalStateNodeTest, FaultedWhenSafetyFaultOrViolation) {
  RobotStateInputs inputs;
  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  inputs.program_running = true;

  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::VIOLATION;
  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::FAULT;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  // Unknown safety mode code.
  inputs.safety_mode = 99;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_NE(status.message.find("Unknown safety mode: 99"), std::string::npos);
}

TEST_F(OperationalStateNodeTest, FaultedWhenRobotDisconnectedOrNoController) {
  RobotStateInputs inputs;
  inputs.safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  inputs.program_running = false;

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::DISCONNECTED;
  auto status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  inputs.robot_mode = ur_dashboard_msgs::msg::RobotMode::CONFIRM_SAFETY;
  status = EvaluateOperationalStatus(inputs);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(OperationalStateNodeTest, NodeConstructionAndTopicHandling) {
  auto node = std::make_shared<UrOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  // Initial state is UNKNOWN.
  auto status = node->GetLatestStatus();
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);

  // Helper node to publish test messages.
  auto test_publisher_node = std::make_shared<rclcpp::Node>("test_publisher_node");
  auto robot_mode_pub = test_publisher_node->create_publisher<ur_dashboard_msgs::msg::RobotMode>(
    "io_and_status_controller/robot_mode", 10);
  auto safety_mode_pub = test_publisher_node->create_publisher<ur_dashboard_msgs::msg::SafetyMode>(
    "io_and_status_controller/safety_mode", 10);
  auto program_running_pub = test_publisher_node->create_publisher<std_msgs::msg::Bool>(
    "io_and_status_controller/robot_program_running", 10);

  // Subscribes to OperationalStatus.
  OperationalStatus received_status;
  std::atomic_bool status_received = false;
  auto status_sub = test_publisher_node->create_subscription<OperationalStatus>(
    "operational_status", 10,
    [&](const OperationalStatus::SharedPtr msg) {
      received_status = *msg;
      status_received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_publisher_node);

  // Publishes ENABLED state.
  ur_dashboard_msgs::msg::RobotMode rm;
  rm.mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  robot_mode_pub->publish(rm);

  ur_dashboard_msgs::msg::SafetyMode sm;
  sm.mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  safety_mode_pub->publish(sm);

  std_msgs::msg::Bool pr;
  pr.data = true;
  program_running_pub->publish(pr);

  // Spins briefly to process callbacks.
  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetLatestStatus().state, OperationalStatus::ENABLED);
}
