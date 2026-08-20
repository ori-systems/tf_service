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

import threading
import unittest

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

import tf_service


EXPECTED_SERVER_NAME = "/tf_service"
EXPECTED_TARGET_FRAME = "map"
EXPECTED_SOURCE_FRAME = "odom"


class ClientRostest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("tf_service_client_rostest")
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.executor_thread.join(timeout=1.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def make_buffer(self, server_name=EXPECTED_SERVER_NAME):
        return tf_service.BufferClient(self.node, server_name)

    def test_wait_for_server_succeeds(self):
        self.assertTrue(self.make_buffer().wait_for_server(0.1))

    def test_wait_for_server_fails(self):
        self.assertFalse(self.make_buffer("/wrong_server_name").wait_for_server(0.1))

    def test_can_transform(self):
        buffer = self.make_buffer()
        self.assertTrue(buffer.wait_for_server(0.1))
        self.assertTrue(buffer.can_transform(
            EXPECTED_TARGET_FRAME, EXPECTED_SOURCE_FRAME, Time(),
            Duration(seconds=0.1))[0])
        self.assertFalse(buffer.can_transform(
            "bla", "blub", Time(), Duration(seconds=0.1))[0])

    def test_can_transform_advanced(self):
        buffer = self.make_buffer()
        self.assertTrue(buffer.wait_for_server(0.1))
        self.assertTrue(buffer.can_transform_full(
            EXPECTED_TARGET_FRAME, Time(), EXPECTED_SOURCE_FRAME, Time(),
            EXPECTED_TARGET_FRAME, Duration(seconds=0.1))[0])
        self.assertFalse(buffer.can_transform_full(
            "bla", Time(), "blub", Time(), "bla", Duration(seconds=0.1))[0])

    def test_lookup_transform(self):
        buffer = self.make_buffer()
        self.assertTrue(buffer.wait_for_server(0.1))
        buffer.lookup_transform(
            EXPECTED_TARGET_FRAME, EXPECTED_SOURCE_FRAME, Time(),
            Duration(seconds=0.1))
        with self.assertRaises(tf2_ros.LookupException):
            buffer.lookup_transform(
                "bla", "blub", Time(), Duration(seconds=0.1))

    def test_lookup_transform_advanced(self):
        buffer = self.make_buffer()
        self.assertTrue(buffer.wait_for_server(0.1))
        buffer.lookup_transform_full(
            EXPECTED_TARGET_FRAME, Time(), EXPECTED_SOURCE_FRAME, Time(),
            EXPECTED_TARGET_FRAME, Duration(seconds=0.1))
        with self.assertRaises(tf2_ros.LookupException):
            buffer.lookup_transform_full(
                "bla", Time(), "blub", Time(), "bla", Duration(seconds=0.1))

    def test_transform(self):
        buffer = self.make_buffer()
        self.assertTrue(buffer.wait_for_server(0.1))
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = EXPECTED_SOURCE_FRAME
        buffer.transform(pose, EXPECTED_TARGET_FRAME)

    def test_transform_full(self):
        buffer = self.make_buffer()
        self.assertTrue(buffer.wait_for_server(0.1))
        pose = PoseStamped()
        now = self.node.get_clock().now()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = EXPECTED_SOURCE_FRAME
        buffer.transform_full(
            pose, EXPECTED_TARGET_FRAME, now, EXPECTED_SOURCE_FRAME,
            Duration(seconds=1.0))


if __name__ == "__main__":
    unittest.main()
