from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node=Node(
        package = 'rumblex_hmi',
        name = 'node_hmi',
        executable = 'node_hmi',
        output="screen"
    )
    ld.add_action(node)
    return ld
