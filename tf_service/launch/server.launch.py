from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_service',
            executable='server',
            name='tf_service',
            output='screen',
            parameters=[{
                'num_threads': 0,
                'cache_time': 10.0,
                'max_timeout': 10.0,
                'debug': False,
            }],
        )
    ])
