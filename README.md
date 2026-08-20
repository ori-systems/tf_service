# tf_service ROS 2 port

This is a ROS 2 / ament conversion of `magazino/tf_service`.

It keeps the original service-based design:

- `/tf_service/can_transform`
- `/tf_service/lookup_transform`

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -r tf_service_ros2 ~/ros2_ws/src/tf_service
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tf_service
source install/setup.bash
```

## Run

```bash
ros2 run tf_service server --ros-args \
  -p num_threads:=4 \
  -p cache_time:=10.0 \
  -p max_timeout:=10.0
```

or:

```bash
ros2 launch tf_service server.launch.py
```

## Python client example

```python
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from tf_service import BufferClient

rclpy.init()
node = rclpy.create_node('tf_service_client')
executor = MultiThreadedExecutor()
executor.add_node(node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()
try:
    buf = BufferClient(node)
    buf.wait_for_server(5.0)
    if buf.can_transform('map', 'base_link', Time(), Duration(seconds=1.0))[0]:
        transform = buf.lookup_transform(
            'map', 'base_link', Time(), Duration(seconds=1.0)
        )
        print(transform)
finally:
    executor.shutdown()
    executor_thread.join()
    node.destroy_node()
    rclpy.shutdown()
```

## Notes

ROS 1 persistent services do not have a direct ROS 2 equivalent. This port uses regular ROS 2 service clients and a MultiThreadedExecutor on the server. The Python client requires a node supplied by its caller; that node must be spinning for synchronous calls to receive service responses.
