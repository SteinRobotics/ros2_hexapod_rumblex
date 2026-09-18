import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')

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
        communication,
        movement,
        brain,
    ])

