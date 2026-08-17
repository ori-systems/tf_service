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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto server = std::make_shared<tf_service::BufferServerNode>();

  server->declare_parameter<int>("num_threads", 0);
  int num_threads = server->get_parameter("num_threads").as_int();

  if (num_threads < 0 || num_threads == 1) {
    RCLCPP_ERROR(server->get_logger(), "num_threads must be 0 (automatic) or at least 2");
    return EXIT_FAILURE;
  }
  if (num_threads == 0) {
    // TF has its own listener thread; reserve at least two executor threads for services.
    num_threads = static_cast<int>(std::max(2u, std::thread::hardware_concurrency()));
  }

  server->init();

  RCLCPP_INFO(server->get_logger(), "Starting tf_service with %d executor threads", num_threads);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), static_cast<size_t>(num_threads));
  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
