"""Launch the desktop OLED with configurable synthetic supply readings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulated_voltage', default_value='12.0'),
        DeclareLaunchArgument('simulated_current', default_value='0.0'),
        Node(
            package='rumblex_hmi',
            executable='node_hmi_simulated',
            name='node_hmi',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=float)
                for name in ('simulated_voltage', 'simulated_current')
            }],
        ),
    ])
