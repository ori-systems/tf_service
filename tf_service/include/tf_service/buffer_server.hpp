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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf_service_msgs/srv/can_transform.hpp"
#include "tf_service_msgs/srv/lookup_transform.hpp"

namespace tf_service
{

struct ServerOptions
{
  double cache_time_sec{10.0};
  double max_timeout_sec{10.0};
  bool debug{false};
};

class BufferServerNode : public rclcpp::Node
{
public:
  explicit BufferServerNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  void init();

private:
  using CanTransform = tf_service_msgs::srv::CanTransform;
  using LookupTransform = tf_service_msgs::srv::LookupTransform;

  void handleLookupTransform(
    const std::shared_ptr<LookupTransform::Request> request,
    std::shared_ptr<LookupTransform::Response> response);

  void handleCanTransform(
    const std::shared_ptr<CanTransform::Request> request,
    std::shared_ptr<CanTransform::Response> response);

  bool timeoutAllowed(const builtin_interfaces::msg::Duration & timeout, std::string * errstr) const;
  rclcpp::Time toTime(const builtin_interfaces::msg::Time & stamp) const;
  tf2::Duration toTf2Duration(const builtin_interfaces::msg::Duration & duration) const;
  void fillError(tf2_msgs::msg::TF2Error & status, uint8_t code, const std::string & message) const;

  ServerOptions options_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<LookupTransform>::SharedPtr lookup_transform_service_;
  rclcpp::Service<CanTransform>::SharedPtr can_transform_service_;
};

}  // namespace tf_service
