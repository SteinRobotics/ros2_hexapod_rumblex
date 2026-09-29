import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')

    house_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rumblex_navigation'), '/launch/map_launch.py']),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
        }.items(),
    )

    communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rumblex_communication'), '/launch/communication_launch.py']),
        launch_arguments={'robot': robot}.items(),
    )
    movement = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('rumblex_movement'), '/launch/movement_offline_launch.py']),
        launch_arguments={'robot': robot}.items(),
    )
    display_mesh = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rumblex_description'), '/launch/display_mesh.launch.py']),
        launch_arguments={'robot': robot, 'joint_state_publisher_gui': 'false'}.items(),
    )
    brain = launch.actions.TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('rumblex_brain'), '/launch/brain_launch.py']),
            launch_arguments={'robot': robot}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot configuration profile (for example: nox or nira)'),
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution([
                FindPackageShare('rumblex_navigation'), 'maps', 'house_contour.yaml']),
            description='Occupancy map YAML file'),
        DeclareLaunchArgument(
            'wall_height', default_value='2.0',
            description='Height of occupied map walls in metres'),
        DeclareLaunchArgument(
            'simulate_lidar', default_value='true',
            description='Publish offline scan_1d readings from map walls; disable for a real lidar'),
        DeclareLaunchArgument(
            'publish_test_map_tf', default_value='true',
            description='Place the test robot at the map origin; disable when using localization'),
        house_map,
        Node(
            package='rumblex_navigation', executable='test_map_walls.py',
            output='screen',
            parameters=[{
                'wall_height': ParameterValue(LaunchConfiguration('wall_height'), value_type=float),
                'simulate_lidar': ParameterValue(LaunchConfiguration('simulate_lidar'), value_type=bool),
            }],
        ),
        # This offline launch has no odometry/localization to connect these frames.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='test_map_to_base_link', output='screen',
            arguments=['--frame-id', 'map', '--child-frame-id', 'base_link'],
            condition=IfCondition(LaunchConfiguration('publish_test_map_tf')),
        ),
        communication,
        movement,
        display_mesh,
        brain,
    ])
