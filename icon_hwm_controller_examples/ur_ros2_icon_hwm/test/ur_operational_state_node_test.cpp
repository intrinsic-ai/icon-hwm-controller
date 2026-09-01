// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ur_ros2_icon_hwm/ur_operational_state_node.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"

namespace ur_ros2_icon_hwm
{
namespace
{

using icon_hwm_controller_msgs::msg::OperationalStatus;

class UrOperationalStateNodeTest : public ::testing::Test
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

TEST_F(UrOperationalStateNodeTest, UnknownWhenMissingInputs)
{
  auto status = ToOperationalStatus({});
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);
  EXPECT_FALSE(status.message.empty());

  status = ToOperationalStatus({
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
      });
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
      });
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);
}

TEST_F(UrOperationalStateNodeTest, EnabledWhenRunningAndProgramPlaying)
{
  auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());

  // Also enabled when in `REDUCED` safety mode and program running.
  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::REDUCED,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(UrOperationalStateNodeTest, DisabledWhenRunningButProgramNotPlaying)
{
  const auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(UrOperationalStateNodeTest, DisabledWhenPowerOffOrIdle)
{
  auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::POWER_OFF,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::IDLE,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::POWER_ON,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::BOOTING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::BACKDRIVE,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::DISABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(UrOperationalStateNodeTest, FaultedWhenProtectiveStop)
{
  const auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("protective stop"), std::string::npos);
}

TEST_F(UrOperationalStateNodeTest, FaultedWhenEmergencyStop)
{
  auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(UrOperationalStateNodeTest, FaultedWhenSafetyFaultOrViolation)
{
  auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::VIOLATION,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::FAULT,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  // Unknown safety mode code.
  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING,
        .safety_mode = 99,
        .program_running = true,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_NE(status.message.find("Unknown safety mode: 99"), std::string::npos);
}

TEST_F(UrOperationalStateNodeTest, FaultedWhenRobotDisconnectedOrNoController)
{
  auto status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::DISCONNECTED,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());

  status = ToOperationalStatus({
        .robot_mode = ur_dashboard_msgs::msg::RobotMode::CONFIRM_SAFETY,
        .safety_mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL,
        .program_running = false,
      });
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
}

TEST_F(UrOperationalStateNodeTest, NodeConstructionAndTopicHandling)
{
  auto node = std::make_shared<UrOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  // Initial state should be unknown.
  const auto status = node->GetOperationalStatus();
  EXPECT_EQ(status.state, OperationalStatus::UNKNOWN);

  // Helper node to publish test messages.
  auto test_publisher_node = std::make_shared<rclcpp::Node>("test_publisher_node");
  auto robot_mode_pub = test_publisher_node->create_publisher<ur_dashboard_msgs::msg::RobotMode>(
    "io_and_status_controller/robot_mode", 10);
  auto safety_mode_pub = test_publisher_node->create_publisher<ur_dashboard_msgs::msg::SafetyMode>(
    "io_and_status_controller/safety_mode", 10);
  auto program_running_pub = test_publisher_node->create_publisher<std_msgs::msg::Bool>(
    "io_and_status_controller/robot_program_running", 10);

  // Subscribes to `OperationalStatus`.
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

  // Publishes `ENABLED` state.
  const auto rm = ur_dashboard_msgs::msg::RobotMode()
    .set__mode(ur_dashboard_msgs::msg::RobotMode::RUNNING);
  robot_mode_pub->publish(rm);

  const auto sm = ur_dashboard_msgs::msg::SafetyMode()
    .set__mode(ur_dashboard_msgs::msg::SafetyMode::NORMAL);
  safety_mode_pub->publish(sm);

  const auto pr = std_msgs::msg::Bool()
    .set__data(true);
  program_running_pub->publish(pr);

  // Spins briefly to process callbacks.
  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetOperationalStatus().state, OperationalStatus::ENABLED);
}

TEST_F(UrOperationalStateNodeTest, FaultTransitionTopicHandling)
{
  auto node = std::make_shared<UrOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  auto test_publisher_node = std::make_shared<rclcpp::Node>("fault_test_publisher_node");
  auto robot_mode_pub = test_publisher_node->create_publisher<ur_dashboard_msgs::msg::RobotMode>(
    "io_and_status_controller/robot_mode", 10);
  auto safety_mode_pub = test_publisher_node->create_publisher<ur_dashboard_msgs::msg::SafetyMode>(
    "io_and_status_controller/safety_mode", 10);
  auto program_running_pub = test_publisher_node->create_publisher<std_msgs::msg::Bool>(
    "io_and_status_controller/robot_program_running", 10);

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

  // Publish faulted state.
  const auto rm = ur_dashboard_msgs::msg::RobotMode()
    .set__mode(ur_dashboard_msgs::msg::RobotMode::RUNNING);
  robot_mode_pub->publish(rm);

  const auto sm = ur_dashboard_msgs::msg::SafetyMode()
    .set__mode(ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP);
  safety_mode_pub->publish(sm);

  const auto pr = std_msgs::msg::Bool()
    .set__data(true);
  program_running_pub->publish(pr);

  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetOperationalStatus().state, OperationalStatus::FAULTED);
}

}  // namespace
}  // namespace ur_ros2_icon_hwm
