// Copyright 2019 Magazino GmbH
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

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf_service_msgs/srv/can_transform.hpp"
#include "tf_service_msgs/srv/lookup_transform.hpp"

namespace tf_service
{

class BufferClient
{
public:
  explicit BufferClient(
    const std::string & server_node_name = "/tf_service",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~BufferClient();

  bool waitForServer(std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max());

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & time,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(0.0));

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame,
    const rclcpp::Time & target_time,
    const std::string & source_frame,
    const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(0.0));

  bool canTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & time,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(0.0),
    std::string * errstr = nullptr);

  bool canTransform(
    const std::string & target_frame,
    const rclcpp::Time & target_time,
    const std::string & source_frame,
    const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(0.0),
    std::string * errstr = nullptr);

private:
  using CanTransform = tf_service_msgs::srv::CanTransform;
  using LookupTransform = tf_service_msgs::srv::LookupTransform;

  std::string serviceName(const std::string & server_node_name, const std::string & leaf) const;
  builtin_interfaces::msg::Time toMsg(const rclcpp::Time & time) const;
  builtin_interfaces::msg::Duration toMsg(const rclcpp::Duration & duration) const;
  void throwOnError(const tf2_msgs::msg::TF2Error & status) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Client<CanTransform>::SharedPtr can_transform_client_;
  rclcpp::Client<LookupTransform>::SharedPtr lookup_transform_client_;
  std::mutex mutex_;
};

}  // namespace tf_service
