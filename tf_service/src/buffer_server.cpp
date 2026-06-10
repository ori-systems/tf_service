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
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), false);

  lookup_transform_service_ = create_service<LookupTransform>(
    kLookupTransformServiceName,
    std::bind(&BufferServerNode::handleLookupTransform, this, std::placeholders::_1, std::placeholders::_2));

  can_transform_service_ = create_service<CanTransform>(
    kCanTransformServiceName,
    std::bind(&BufferServerNode::handleCanTransform, this, std::placeholders::_1, std::placeholders::_2));
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
  std::string errstr;
  if (!timeoutAllowed(request->timeout, &errstr)) {
    fillError(response->status, tf2_msgs::msg::TF2Error::INVALID_ARGUMENT_ERROR, errstr);
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
