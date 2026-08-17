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

#define DEBUG_OUTPUT 0
#if DEBUG_OUTPUT
#include <chrono>
#include <iostream>
#endif

#include "tf_service/buffer_server.hpp"

#include <algorithm>
#include <string>

#include "tf2/exceptions.h"
#include "tf_service/constants.hpp"

namespace tf_service
{

BufferServerNode::BufferServerNode(const rclcpp::NodeOptions & node_options)
: Node("tf_service", node_options)
{
  this->declare_parameter<double>("cache_time_sec", 10.0);
  this->declare_parameter<double>("max_timeout_sec", 10.0);
  this->declare_parameter<bool>("debug", false);

  options_.cache_time_sec = this->get_parameter("cache_time_sec").as_double();
  options_.max_timeout_sec = this->get_parameter("max_timeout_sec").as_double();
  options_.debug = this->get_parameter("debug").as_bool();
}

void BufferServerNode::init()
{
  const auto cache_time = tf2::durationFromSec(options_.cache_time_sec);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), cache_time, shared_from_this());
  // Keep TF ingestion independent from potentially blocking service callbacks.
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), true);

  // Permit service callbacks, including multiple calls to the same service, to overlap.
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    std::cerr << "Hi my name is " << this->get_name() << std::endl << std::flush;
  lookup_transform_service_ = create_service<LookupTransform>(
    std::string(this->get_name()) + "/" + std::string(kLookupTransformServiceName),
    std::bind(&BufferServerNode::handleLookupTransform, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_callback_group_);

  can_transform_service_ = create_service<CanTransform>(
    std::string(this->get_name()) + "/" + std::string(kCanTransformServiceName),
    std::bind(&BufferServerNode::handleCanTransform, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_callback_group_);
}

rclcpp::Time BufferServerNode::toTime(const builtin_interfaces::msg::Time & stamp) const
{
  return rclcpp::Time(stamp, this->get_clock()->get_clock_type());
}

tf2::Duration BufferServerNode::toTf2Duration(const builtin_interfaces::msg::Duration & duration) const
{
  return tf2::durationFromSec(static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec));
}

bool BufferServerNode::timeoutAllowed(
  const builtin_interfaces::msg::Duration & timeout,
  std::string * errstr) const
{
  const double timeout_sec = static_cast<double>(timeout.sec) + 1e-9 * static_cast<double>(timeout.nanosec);
  if (timeout_sec > options_.max_timeout_sec) {
    if (errstr != nullptr) {
      *errstr = "Server is configured to block requests with a timeout above " +
        std::to_string(options_.max_timeout_sec) + " seconds.";
    }
    return false;
  }
  return true;
}

void BufferServerNode::fillError(
  tf2_msgs::msg::TF2Error & status, uint8_t code, const std::string & message) const
{
  status.error = code;
  status.error_string = message;
}

void BufferServerNode::handleLookupTransform(
  const std::shared_ptr<LookupTransform::Request> request,
  std::shared_ptr<LookupTransform::Response> response)
{
#if DEBUG_OUTPUT
  const auto start = std::chrono::steady_clock::now();

  std::cout
    << "[handleLookupTransform] Request:"
    << "\n  advanced=" << std::boolalpha << request->advanced
    << "\n  target_frame=" << request->target_frame
    << "\n  source_frame=" << request->source_frame
    << "\n  fixed_frame=" << request->fixed_frame
    << "\n  time.sec=" << request->time.sec
    << " time.nanosec=" << request->time.nanosec
    << "\n  target_time.sec=" << request->target_time.sec
    << " target_time.nanosec=" << request->target_time.nanosec
    << "\n  source_time.sec=" << request->source_time.sec
    << " source_time.nanosec=" << request->source_time.nanosec
    << "\n  timeout.sec=" << request->timeout.sec
    << " timeout.nanosec=" << request->timeout.nanosec
    << std::endl;
#endif

  std::string errstr;
  if (!timeoutAllowed(request->timeout, &errstr)) {
    fillError(response->status, tf2_msgs::msg::TF2Error::INVALID_ARGUMENT_ERROR, errstr);
#if DEBUG_OUTPUT
    const auto elapsed_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start).count();

    std::cout
      << "[handleLookupTransform] Response:"
      << "\n  status.error=" << int(response->status.error)
      << "\n  status.error_string=" << response->status.error_string
      << "\n  elapsed_us=" << elapsed_us
      << std::endl << std::flush;
#endif
    return;
  }

  try {
    if (request->advanced) {
      response->transform = tf_buffer_->lookupTransform(
        request->target_frame,
        toTime(request->target_time),
        request->source_frame,
        toTime(request->source_time),
        request->fixed_frame,
        toTf2Duration(request->timeout));
    } else {
      response->transform = tf_buffer_->lookupTransform(
        request->target_frame,
        request->source_frame,
        toTime(request->time),
        toTf2Duration(request->timeout));
    }
    fillError(response->status, tf2_msgs::msg::TF2Error::NO_ERROR, "Success.");
  } catch (const tf2::ConnectivityException & exception) {
    fillError(response->status, tf2_msgs::msg::TF2Error::CONNECTIVITY_ERROR, exception.what());
  } catch (const tf2::ExtrapolationException & exception) {
    fillError(response->status, tf2_msgs::msg::TF2Error::EXTRAPOLATION_ERROR, exception.what());
  } catch (const tf2::InvalidArgumentException & exception) {
    fillError(response->status, tf2_msgs::msg::TF2Error::INVALID_ARGUMENT_ERROR, exception.what());
  } catch (const tf2::LookupException & exception) {
    fillError(response->status, tf2_msgs::msg::TF2Error::LOOKUP_ERROR, exception.what());
  } catch (const tf2::TimeoutException & exception) {
    fillError(response->status, tf2_msgs::msg::TF2Error::TIMEOUT_ERROR, exception.what());
  } catch (const tf2::TransformException & exception) {
    fillError(response->status, tf2_msgs::msg::TF2Error::TRANSFORM_ERROR, exception.what());
  }
#if DEBUG_OUTPUT
  const auto elapsed_us =
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - start).count();

  std::cout
    << "[handleLookupTransform] Response:"
    << "\n  status.error=" << int(response->status.error)
    << "\n  status.error_string=" << response->status.error_string
    << "\n  child_frame_id=" << response->transform.child_frame_id
    << "\n  header.frame_id=" << response->transform.header.frame_id
    << "\n  translation=("
    << response->transform.transform.translation.x << ", "
    << response->transform.transform.translation.y << ", "
    << response->transform.transform.translation.z << ")"
    << "\n  rotation=("
    << response->transform.transform.rotation.x << ", "
    << response->transform.transform.rotation.y << ", "
    << response->transform.transform.rotation.z << ", "
    << response->transform.transform.rotation.w << ")"
    << "\n  elapsed_us=" << elapsed_us
    << std::endl << std::flush;
#endif
}

void BufferServerNode::handleCanTransform(
  const std::shared_ptr<CanTransform::Request> request,
  std::shared_ptr<CanTransform::Response> response)
{
  if (!timeoutAllowed(request->timeout, &response->errstr)) {
    response->can_transform = false;
    return;
  }

  if (request->advanced) {
    response->can_transform = tf_buffer_->canTransform(
      request->target_frame,
      toTime(request->target_time),
      request->source_frame,
      toTime(request->source_time),
      request->fixed_frame,
      toTf2Duration(request->timeout),
      &response->errstr);
  } else {
    response->can_transform = tf_buffer_->canTransform(
      request->target_frame,
      request->source_frame,
      toTime(request->time),
      toTf2Duration(request->timeout),
      &response->errstr);
  }
}

}  // namespace tf_service
