#!/usr/bin/env python3
# Copyright 2019 Magazino GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import threading

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.utilities import remove_ros_args

import tf_service


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tf_service_benchmark_client")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    parser = argparse.ArgumentParser()
    parser.add_argument("--lookup_frequency", type=float, default=10)
    parser.add_argument("--use_old_version", action="store_true")
    parsed_args = parser.parse_args(remove_ros_args(args=args)[1:])

    try:
        if parsed_args.use_old_version:
            buffer = tf2_ros.BufferClient(node, "/tf2_buffer_server")
        else:
            buffer = tf_service.BufferClient(node)
        buffer.wait_for_server()

        period = 1.0 / parsed_args.lookup_frequency
        while rclpy.ok():
            try:
                buffer.lookup_transform(
                    "map", "odom", Time(), Duration(seconds=1.0))
            except tf2_ros.TransformException as exc:
                node.get_logger().error(f"{type(exc)}: {exc}")
                break
            threading.Event().wait(period)
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
