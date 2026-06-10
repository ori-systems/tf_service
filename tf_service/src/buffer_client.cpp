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

#include "tf_service/buffer_client.hpp"

#include <stdexcept>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf2_error.hpp"
#include "tf_service/constants.hpp"

namespace tf_service
{

BufferClient::BufferClient(const std::string & server_node_name, const rclcpp::NodeOptions & node_options)
{
  node_ = std::make_shared<rclcpp::Node>("tf_service_buffer_client", node_options);
  can_transform_client_ = node_->create_client<CanTransform>(serviceName(server_node_name, kCanTransformServiceName));
  lookup_transform_client_ = node_->create_client<LookupTransform>(serviceName(server_node_name, kLookupTransformServiceName));
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });
}

BufferClient::~BufferClient()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

std::string BufferClient::serviceName(const std::string & server_node_name, const std::string & leaf) const
{
  if (server_node_name.empty() || server_node_name == "/") {
    return "/" + leaf;
  }
  if (server_node_name.back() == '/') {
    return server_node_name + leaf;
  }
  return server_node_name + "/" + leaf;
}

builtin_interfaces::msg::Time BufferClient::toMsg(const rclcpp::Time & time) const
{
  builtin_interfaces::msg::Time msg;
  const auto ns = time.nanoseconds();
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}

builtin_interfaces::msg::Duration BufferClient::toMsg(const rclcpp::Duration & duration) const
{
  builtin_interfaces::msg::Duration msg;
  const auto ns = duration.nanoseconds();
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}

bool BufferClient::waitForServer(std::chrono::nanoseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  auto remaining = timeout;
  if (!can_transform_client_->wait_for_service(remaining)) {
    return false;
  }
  if (timeout != std::chrono::nanoseconds::max()) {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= timeout) {
      return false;
    }
    remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout - elapsed);
  }
  return lookup_transform_client_->wait_for_service(remaining);
}

void BufferClient::throwOnError(const tf2_msgs::msg::TF2Error & status) const
{
  switch (status.error) {
    case tf2_msgs::msg::TF2Error::NO_ERROR:
      return;
    case tf2_msgs::msg::TF2Error::CONNECTIVITY_ERROR:
      throw tf2::ConnectivityException(status.error_string);
    case tf2_msgs::msg::TF2Error::EXTRAPOLATION_ERROR:
      throw tf2::ExtrapolationException(status.error_string);
    case tf2_msgs::msg::TF2Error::INVALID_ARGUMENT_ERROR:
      throw tf2::InvalidArgumentException(status.error_string);
    case tf2_msgs::msg::TF2Error::LOOKUP_ERROR:
      throw tf2::LookupException(status.error_string);
    case tf2_msgs::msg::TF2Error::TIMEOUT_ERROR:
      throw tf2::TimeoutException(status.error_string);
    case tf2_msgs::msg::TF2Error::TRANSFORM_ERROR:
      throw tf2::TransformException(status.error_string);
    default:
      throw tf2::TransformException(status.error_string);
  }
}

geometry_msgs::msg::TransformStamped BufferClient::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & time,
  const rclcpp::Duration & timeout)
{
  auto request = std::make_shared<LookupTransform::Request>();
  request->target_frame = target_frame;
  request->source_frame = source_frame;
  request->time = toMsg(time);
  request->timeout = toMsg(timeout);
  request->advanced = false;

  std::lock_guard<std::mutex> guard(mutex_);
  auto future = lookup_transform_client_->async_send_request(request);
  if (future.wait_for(timeout.to_chrono<std::chrono::nanoseconds>() + std::chrono::seconds(1)) != std::future_status::ready) {
    throw tf2::TimeoutException("service call to buffer server timed out");
  }
  auto response = future.get();
  throwOnError(response->status);
  return response->transform;
}

geometry_msgs::msg::TransformStamped BufferClient::lookupTransform(
  const std::string & target_frame,
  const rclcpp::Time & target_time,
  const std::string & source_frame,
  const rclcpp::Time & source_time,
  const std::string & fixed_frame,
  const rclcpp::Duration & timeout)
{
  auto request = std::make_shared<LookupTransform::Request>();
  request->target_frame = target_frame;
  request->target_time = toMsg(target_time);
  request->source_frame = source_frame;
  request->source_time = toMsg(source_time);
  request->fixed_frame = fixed_frame;
  request->timeout = toMsg(timeout);
  request->advanced = true;

  std::lock_guard<std::mutex> guard(mutex_);
  auto future = lookup_transform_client_->async_send_request(request);
  if (future.wait_for(timeout.to_chrono<std::chrono::nanoseconds>() + std::chrono::seconds(1)) != std::future_status::ready) {
    throw tf2::TimeoutException("service call to buffer server timed out");
  }
  auto response = future.get();
  throwOnError(response->status);
  return response->transform;
}

bool BufferClient::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & time,
  const rclcpp::Duration & timeout,
  std::string * errstr)
{
  auto request = std::make_shared<CanTransform::Request>();
  request->target_frame = target_frame;
  request->source_frame = source_frame;
  request->time = toMsg(time);
  request->timeout = toMsg(timeout);
  request->advanced = false;

  std::lock_guard<std::mutex> guard(mutex_);
  auto future = can_transform_client_->async_send_request(request);
  if (future.wait_for(timeout.to_chrono<std::chrono::nanoseconds>() + std::chrono::seconds(1)) != std::future_status::ready) {
    if (errstr != nullptr) {
      *errstr = "service call to buffer server timed out";
    }
    return false;
  }
  auto response = future.get();
  if (errstr != nullptr) {
    *errstr = response->errstr;
  }
  return response->can_transform;
}

bool BufferClient::canTransform(
  const std::string & target_frame,
  const rclcpp::Time & target_time,
  const std::string & source_frame,
  const rclcpp::Time & source_time,
  const std::string & fixed_frame,
  const rclcpp::Duration & timeout,
  std::string * errstr)
{
  auto request = std::make_shared<CanTransform::Request>();
  request->target_frame = target_frame;
  request->target_time = toMsg(target_time);
  request->source_frame = source_frame;
  request->source_time = toMsg(source_time);
  request->fixed_frame = fixed_frame;
  request->timeout = toMsg(timeout);
  request->advanced = true;

  std::lock_guard<std::mutex> guard(mutex_);
  auto future = can_transform_client_->async_send_request(request);
  if (future.wait_for(timeout.to_chrono<std::chrono::nanoseconds>() + std::chrono::seconds(1)) != std::future_status::ready) {
    if (errstr != nullptr) {
      *errstr = "service call to buffer server timed out";
    }
    return false;
  }
  auto response = future.get();
  if (errstr != nullptr) {
    *errstr = response->errstr;
  }
  return response->can_transform;
}

}  // namespace tf_service
