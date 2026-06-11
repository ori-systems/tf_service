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
from rclpy.time import Time
from rclpy.duration import Duration
from tf_service import BufferClient

rclpy.init()
buf = BufferClient('/tf_service')
buf.wait_for_server(5.0)
if buf.can_transform('map', 'base_link', Time(), Duration(seconds=1.0))[0]:
    transform = buf.lookup_transform('map', 'base_link', Time(), Duration(seconds=1.0))
print(transform)
buf.destroy()
rclpy.shutdown()
```

## Notes

ROS 1 persistent services do not have a direct ROS 2 equivalent. This port uses regular ROS 2 service clients and a MultiThreadedExecutor on the server. The C++ and Python clients own a small executor thread by default so synchronous calls can receive service responses even when not embedded in another executor.
