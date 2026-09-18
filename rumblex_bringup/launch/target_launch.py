import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')

    communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rumblex_communication'), '/launch/communication_launch.py']),
        launch_arguments={'robot': robot}.items(),
    )
    hmi = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare('rumblex_hmi'), '/launch/hmi_launch.py']))
    movement = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('rumblex_movement'), '/launch/movement_launch.py']),
        launch_arguments={'robot': robot}.items(),
    )
    teleop = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare('rumblex_teleop'), '/launch/teleop_launch.py']))
    lidar = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare('rumblex_lidar'), '/launch/lidar_launch.py']))

    bno055_config = PathJoinSubstitution([
        FindPackageShare('rumblex_bringup'), 'config', robot, 'bno055_params.yaml'])
    bno055 = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_config],
    )

    brain = launch.actions.TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('rumblex_brain'), '/launch/brain_launch.py']),
            launch_arguments={'robot': robot}.items(),
        )],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rumblex_navigation'), '/launch/navigation_launch.py']),
        launch_arguments={'robot': robot}.items(),
        condition=IfCondition(LaunchConfiguration('enable_navigation')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot configuration profile (for example: nox or nira)'),
        DeclareLaunchArgument(
            'enable_navigation', default_value='false',
            description='Launch rumblex_navigation (head-sweep scan + reactive nav)'),
        communication,
        hmi,
        movement,
        teleop,
        lidar,
        bno055,
        brain,
        navigation,
    ])

