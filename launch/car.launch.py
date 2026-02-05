from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    poll_rate_hz = LaunchConfiguration('poll_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.33'),
        DeclareLaunchArgument('robot_port', default_value='80'),
        DeclareLaunchArgument('poll_rate_hz', default_value='10.0'),

        Node(
            package='car_bridge',
            executable='http_status_node',
            name='car_status_bridge',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'robot_port': robot_port,
                'poll_rate_hz': poll_rate_hz,
            }],
        ),

        Node(
            package='car_bridge',
            executable='vision_node',
            name='car_vision_bridge',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'robot_port': robot_port,
            }],
        ),
    ])
