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

#include "khi_ros2_icon_hwm/khi_operational_state_node.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_ros2_icon_hwm
{
namespace
{

using icon_hwm_controller_msgs::msg::OperationalStatus;

class KhiOperationalStateNodeTest : public ::testing::Test
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

TEST_F(KhiOperationalStateNodeTest, EnabledWhenMissingInputs)
{
  const std::optional<khi_msgs::msg::ErrorInfo> error_info = std::nullopt;

  const auto status = ToOperationalStatus(error_info);
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(KhiOperationalStateNodeTest, EnabledWhenNoErrors)
{
  khi_msgs::msg::ErrorInfo error_info;

  const auto status = ToOperationalStatus(error_info);
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);
  EXPECT_TRUE(status.message.empty());
}

TEST_F(KhiOperationalStateNodeTest, FaultedWhenErrorActive)
{
  khi_msgs::msg::ErrorInfo error_info;
  error_info.error_codes.push_back(-31135);
  error_info.error_msgs.push_back("Motor power OFF");

  const auto status = ToOperationalStatus(error_info);
  EXPECT_EQ(status.state, OperationalStatus::FAULTED);
  EXPECT_FALSE(status.message.empty());
  EXPECT_NE(status.message.find("Motor power OFF"), std::string::npos);
}

TEST_F(KhiOperationalStateNodeTest, NodeConstructionAndTopicHandling)
{
  auto node = std::make_shared<KhiOperationalStateNode>();
  EXPECT_NE(node, nullptr);

  // Initial state should be enabled.
  const auto status = node->GetOperationalStatus();
  EXPECT_EQ(status.state, OperationalStatus::ENABLED);

  auto test_pub_node = std::make_shared<rclcpp::Node>("test_pub_node");
  auto error_info_pub = test_pub_node->create_publisher<khi_msgs::msg::ErrorInfo>(
    "/khi_controller0/khi_publisher/error_info", 10);

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

  // Publish faulted error info.
  khi_msgs::msg::ErrorInfo error_info;
  error_info.error_codes.push_back(-31135);
  error_info.error_msgs.push_back("Motor power OFF");
  error_info_pub->publish(error_info);

  for (int i = 0; i < 20 && !status_received; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(node->GetOperationalStatus().state, OperationalStatus::FAULTED);
}

}  // namespace
}  // namespace khi_ros2_icon_hwm
