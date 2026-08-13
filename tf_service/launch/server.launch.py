from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_threads = LaunchConfiguration('num_threads')
    cache_time_sec = LaunchConfiguration('cache_time_sec')
    max_timeout_sec = LaunchConfiguration('max_timeout_sec')
    debug = LaunchConfiguration('debug')

    return LaunchDescription([
        DeclareLaunchArgument('num_threads', default_value='0'),
        DeclareLaunchArgument('cache_time_sec', default_value='10.0'),
        DeclareLaunchArgument('max_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('debug', default_value='false'),
        Node(
            package='tf_service',
            executable='server',
            name='tf_service',
            output='screen',
            parameters=[{
                'num_threads': num_threads,
                'cache_time_sec': cache_time_sec,
                'max_timeout_sec': max_timeout_sec,
                'debug': debug,
            }],
        )
    ])
