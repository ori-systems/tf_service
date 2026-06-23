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

"""ROS 2 Python BufferClient for tf_service."""
# For consistency, the docstrings of tf2_ros.BufferInterface methods were copied
# from geometry2/tf2_ros/src/tf2_ros/buffer_client.py, which is
# subject to a BSD License. See 3rdparty/geometry2_LICENSE for details.

from __future__ import annotations

import threading
import typing
from typing import Optional
import time

import rclpy
import tf2_geometry_msgs # pylint: disable=unused-import
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TF2Error
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    InvalidArgumentException,
    LookupException,
    TimeoutException,
    TransformException,
    BufferInterface,
)

from tf_service_msgs.srv import CanTransform, LookupTransform

def _service_name(server_node_name: str, leaf: str) -> str:
    server_node_name = server_node_name.rstrip("/")
    return f"/{leaf}" if not server_node_name else f"{server_node_name}/{leaf}"

def to_time_msg(time_obj):
    if hasattr(time_obj, 'to_msg'):
        return time_obj.to_msg()
    return time_obj # Assume it's already a message

class BufferClient(BufferInterface):
    """
    A client for the tf_service.

    This client is a ROS 2 wrapper for the tf_service, which provides a service-based
    alternative to the standard tf2_ros.Buffer. It can be used to query for
    transforms without being part of a ROS 2 node's executor spin.
    """

    def __init__(self, server_node_name: str = "/tf_service", node: Optional[Node] = None, base_call_timeout_secs:float=0.1):
        """
        Constructor.
        :param server_node_name: The name of the tf_service server node.
        :param node: An existing rclpy.Node to use for the service clients. If None, a new node is created and spun in a background thread.
        :base_call_timeout_secs: The number of seconds to allow for network communication
        """
        super().__init__()
        self.base_call_timeout = int(base_call_timeout_secs*1e9)
        self._own_node = node is None
        self._cb_group = ReentrantCallbackGroup()
        self._node = node or rclpy.create_node("tf_service_buffer_client")
        self._lookup = self._node.create_client(
            LookupTransform, _service_name(server_node_name, "lookup_transform"), callback_group=self._cb_group
        )
        self._can = self._node.create_client(
            CanTransform, _service_name(server_node_name, "can_transform"), callback_group=self._cb_group
        )
        self._executor = None
        self._thread = None
        if self._own_node:
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()

    def destroy(self):
        """Clean up resources, shutting down the internal node if it was created by this client."""
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._own_node:
            self._node.destroy_node()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy()

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Block until the server is ready to respond to requests.

        :param timeout_sec: Time in seconds to wait for the server.
        :return: True if the server is ready, false otherwise.
        """
        return self._lookup.wait_for_service(timeout_sec) and self._can.wait_for_service(timeout_sec)

    def lookup_transform(self, target_frame: str, source_frame: str, time: Time, timeout: Duration = Duration()) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame.
        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (Time(seconds=0) will get the latest)
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        """
        req = LookupTransform.Request()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.time = time.to_msg()
        req.timeout = timeout.to_msg()
        req.advanced = False
        try:
            result = self._lookup.call(req, timeout.nanoseconds/1e9 + self.base_call_timeout)
        except Exception as ex:
            print(f"Call exception: {ex=}")
        print(result)
        # future = self._lookup.call_async(req)
        # result = self._wait(future, timeout)
        self._throw_on_error(result.status)
        return result.transform

    def lookup_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Duration = Duration(),
    ) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame using the advanced API.
        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (Time(seconds=0) will get the latest)
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (Time(seconds=0) will get the latest)
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        """
        req = LookupTransform.Request()
        req.target_frame = target_frame
        req.target_time = to_time_msg(target_time)
        req.source_frame = source_frame
        #print(f"{type(source_time)=}\n{source_time=}\n{repr(source_time)=}\n{dir(source_time)=}", flush=True)
        req.source_time = to_time_msg(source_time)
        req.fixed_frame = fixed_frame
        req.timeout = to_time_msg(timeout)
        req.advanced = True
        try:
            result = self._lookup.call(req, timeout.nanoseconds/1e9 + self.base_call_timeout)
        except Exception as ex:
            print(f"Call exception: {ex=}")
        #print(f"{result=}")
        # future = self._lookup.call_async(req)
        # result = self._wait(future, timeout)
        self._throw_on_error(result.status)
        return result.transform

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Duration = Duration(),
    ) -> tuple[bool, str]:
        """
        Check if a transform from the source frame to the target frame is possible.
        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (Time(seconds=0) will get the latest)
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: A tuple containing a boolean indicating if the transform is possible and a string with an error message if not.
        """
        req = CanTransform.Request()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.time = time.to_msg()
        req.timeout = to_time_msg(timeout)
        req.advanced = False
        try:
            result = self._can.call(req, timeout.nanoseconds/1e9 + self.base_call_timeout)
        except Exception as ex:
            print(f"Call exception: {ex=}")
        print(result)
        # future = self._can.call_async(req)
        # result = self._wait(future, timeout)
        return result.can_transform, result.errstr

    def can_transform_full(
        self,
        target_frame: str,
        target_time: Time,
        source_frame: str,
        source_time: Time,
        fixed_frame: str,
        timeout: Duration = Duration()
    ) -> bool:
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to (0 will get the latest).
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated (0 will get the latest).
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: Time to wait for the target frame to become available.
        :return: True if the transform is possible, false otherwise.
        """
        req = CanTransform.Request()
        req.target_frame = target_frame
        req.target_time = to_time_msg(target_time)
        req.source_frame = source_frame
        req.source_time = to_time_msg(source_time)
        req.fixed_frame = fixed_frame
        req.timeout = to_time_msg(timeout)
        req.advanced = True
        try:
            result = self._can.call(req, timeout.nanoseconds/1e9 + self.base_call_timeout)
        except Exception as ex:
            print(f"Call exception: {ex=}")
        print(result)
        #future = self._can.call_async(req)
        #result = self._wait(future, timeout)
        return result.can_transform, result.errstr

    def _wait(self, future, timeout: Duration):
        # The service server has its own timeout, but we add a grace period for the call itself to ensure the future completes.
        total_timeout = max(timeout.nanoseconds / 1e9, 0.0) + 1.0
        if self._own_node:
            if not future.done():
                future.result(timeout=total_timeout) # is invalid, should purge the onwn node stuff
        else:
            for i in range(20):
                if not future.done():
                    time.sleep(total_timeout/20.0)
                else:
                    break
            #rclpy.spin_until_future_complete(self._node, future, timeout_sec=total_timeout)
        if not future.done():
            print(f"{type(future)=}\n{dir(future)=}\n{future.done()=}")
            raise TimeoutException("service call to buffer server timed out")
        return future.result()

    @staticmethod
    def _throw_on_error(status: TF2Error) -> None:
        if status.error == TF2Error.NO_ERROR:
            return
        if status.error == TF2Error.CONNECTIVITY_ERROR:
            raise ConnectivityException(status.error_string)
        if status.error == TF2Error.EXTRAPOLATION_ERROR:
            raise ExtrapolationException(status.error_string)
        if status.error == TF2Error.INVALID_ARGUMENT_ERROR:
            raise InvalidArgumentException(status.error_string)
        if status.error == TF2Error.LOOKUP_ERROR:
            raise LookupException(status.error_string)
        if status.error == TF2Error.TIMEOUT_ERROR:
            raise TimeoutException(status.error_string)
        raise TransformException(status.error_string)
