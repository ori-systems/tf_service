#!/usr/bin/env python3
"""Minimal ROS 2 Python BufferClient for tf_service."""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TF2Error
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    InvalidArgumentException,
    LookupException,
    TimeoutException,
    TransformException,
)

from tf_service.srv import CanTransform, LookupTransform


def _service_name(server_node_name: str, leaf: str) -> str:
    server_node_name = server_node_name.rstrip("/")
    return f"/{leaf}" if not server_node_name else f"{server_node_name}/{leaf}"


class BufferClient:
    def __init__(self, server_node_name: str = "/tf_service", node: Optional[Node] = None):
        self._own_node = node is None
        self._node = node or rclpy.create_node("tf_service_buffer_client")
        self._lookup = self._node.create_client(
            LookupTransform, _service_name(server_node_name, "lookup_transform")
        )
        self._can = self._node.create_client(
            CanTransform, _service_name(server_node_name, "can_transform")
        )
        self._executor = None
        self._thread = None
        if self._own_node:
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()

    def destroy(self):
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._own_node:
            self._node.destroy_node()

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        return self._lookup.wait_for_service(timeout_sec) and self._can.wait_for_service(timeout_sec)

    def lookup_transform(self, target_frame: str, source_frame: str, time: Time, timeout: Duration = Duration()):
        req = LookupTransform.Request()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.time = time.to_msg()
        req.timeout = timeout.to_msg()
        req.advanced = False
        future = self._lookup.call_async(req)
        result = self._wait(future, timeout)
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
    ):
        req = LookupTransform.Request()
        req.target_frame = target_frame
        req.target_time = target_time.to_msg()
        req.source_frame = source_frame
        req.source_time = source_time.to_msg()
        req.fixed_frame = fixed_frame
        req.timeout = timeout.to_msg()
        req.advanced = True
        future = self._lookup.call_async(req)
        result = self._wait(future, timeout)
        self._throw_on_error(result.status)
        return result.transform

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: Time,
        timeout: Duration = Duration(),
    ) -> tuple[bool, str]:
        req = CanTransform.Request()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.time = time.to_msg()
        req.timeout = timeout.to_msg()
        req.advanced = False
        future = self._can.call_async(req)
        result = self._wait(future, timeout)
        return result.can_transform, result.errstr

    def _wait(self, future, timeout: Duration):
        total_timeout = max(timeout.nanoseconds / 1e9, 0.0) + 1.0
        if self._own_node:
            if not future.done():
                future.result(timeout=total_timeout)
        else:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=total_timeout)
        if not future.done():
            raise TimeoutException("service call to buffer server timed out")
        return future.result()

    @staticmethod
    def _throw_on_error(status: TF2Error):
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
