"""Display a RumbleX robot model in RViz with joint controls."""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


_SNAP_GUI_OVERRIDES = {
    key: '' for key in (
        'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
        'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR',
    ) if key in os.environ
}


def _launch_robot(context):
    robot = LaunchConfiguration('robot').perform(context)
    if not re.fullmatch(r'[a-z][a-z0-9_]*', robot):
        raise RuntimeError(f'Invalid robot profile: {robot!r}')

    share = get_package_share_directory('rumblex_description')
    xacro_file = os.path.join(share, 'urdf', f'{robot}.urdf.xacro')
    if not os.path.isfile(xacro_file):
        raise RuntimeError(f'Robot profile {robot!r} has no model: {xacro_file}')

    description = xacro.process_file(xacro_file, mappings={'use_sim': 'false'}).toxml()
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            additional_env=_SNAP_GUI_OVERRIDES,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'model.rviz')],
            additional_env=_SNAP_GUI_OVERRIDES,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot model profile (for example: nox or nira)'),
        OpaqueFunction(function=_launch_robot),
    ])

