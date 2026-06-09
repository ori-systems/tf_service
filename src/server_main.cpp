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

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf_service/buffer_server.hpp"

namespace
{

double get_double_param(const rclcpp::Node::SharedPtr & node, const std::string & name, double default_value)
{
  node->declare_parameter<double>(name, default_value);
  return node->get_parameter(name).as_double();
}

int get_int_param(const rclcpp::Node::SharedPtr & node, const std::string & name, int default_value)
{
  node->declare_parameter<int>(name, default_value);
  return static_cast<int>(node->get_parameter(name).as_int());
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options_node = std::make_shared<rclcpp::Node>("tf_service_options");
  tf_service::ServerOptions options;
  options.cache_time_sec = get_double_param(options_node, "cache_time", 10.0);
  options.max_timeout_sec = get_double_param(options_node, "max_timeout", 10.0);
  options_node->declare_parameter<bool>("debug", false);
  options.debug = options_node->get_parameter("debug").as_bool();
  int num_threads = get_int_param(options_node, "num_threads", 0);

  if (num_threads < 0) {
    RCLCPP_ERROR(options_node->get_logger(), "num_threads cannot be negative");
    return EXIT_FAILURE;
  }
  if (num_threads == 0) {
    num_threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  }

  auto server = std::make_shared<tf_service::BufferServerNode>(options);
  server->init();

  RCLCPP_INFO(server->get_logger(), "Starting tf_service with %d executor threads", num_threads);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), static_cast<size_t>(num_threads));
  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
