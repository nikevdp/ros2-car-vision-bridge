from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    poll_rate_hz = LaunchConfiguration('poll_rate_hz')
    h1_low = LaunchConfiguration('h1_low')
    h1_high = LaunchConfiguration('h1_high')
    h2_low = LaunchConfiguration('h2_low')
    h2_high = LaunchConfiguration('h2_high')
    s_min = LaunchConfiguration('s_min')
    s_max = LaunchConfiguration('s_max')
    v_min = LaunchConfiguration('v_min')
    v_max = LaunchConfiguration('v_max')
    deadband_x = LaunchConfiguration('deadband_x')
    min_area = LaunchConfiguration('min_area')
    max_area = LaunchConfiguration('max_area')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.33'),
        DeclareLaunchArgument('robot_port', default_value='80'),
        DeclareLaunchArgument('poll_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('h1_low', default_value='0'),
        DeclareLaunchArgument('h1_high', default_value='10'),
        DeclareLaunchArgument('h2_low', default_value='170'),
        DeclareLaunchArgument('h2_high', default_value='179'),
        DeclareLaunchArgument('s_min', default_value='120'),
        DeclareLaunchArgument('s_max', default_value='255'),
        DeclareLaunchArgument('v_min', default_value='80'),
        DeclareLaunchArgument('v_max', default_value='255'),
        DeclareLaunchArgument('deadband_x', default_value='50'),
        DeclareLaunchArgument('min_area', default_value='8000'),
        DeclareLaunchArgument('max_area', default_value='15000'),

        Node(
            package='car_bridge',
            executable='http_status_node',
            name='car_status_bridge',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'robot_port': ParameterValue(robot_port, value_type=int),
                'poll_rate_hz': ParameterValue(poll_rate_hz, value_type=float),
            }],
        ),

        Node(
            package='car_bridge',
            executable='vision_node',
            name='car_vision_bridge',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'robot_port': ParameterValue(robot_port, value_type=int),
                'h1_low': ParameterValue(h1_low, value_type=int),
                'h1_high': ParameterValue(h1_high, value_type=int),
                'h2_low': ParameterValue(h2_low, value_type=int),
                'h2_high': ParameterValue(h2_high, value_type=int),
                's_min': ParameterValue(s_min, value_type=int),
                's_max': ParameterValue(s_max, value_type=int),
                'v_min': ParameterValue(v_min, value_type=int),
                'v_max': ParameterValue(v_max, value_type=int),
                'deadband_x': ParameterValue(deadband_x, value_type=int),
                'min_area': ParameterValue(min_area, value_type=int),
                'max_area': ParameterValue(max_area, value_type=int),
            }],
        ),
    ])