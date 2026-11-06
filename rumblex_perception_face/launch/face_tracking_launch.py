"""Face observations from an existing image source; does not start capture or actuators."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rumblex_perception_face')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nira', choices=['nira']),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            share, 'config', LaunchConfiguration('robot'), 'face_tracking.yaml'])),
        DeclareLaunchArgument('model_path', default_value=PathJoinSubstitution([
            share, 'models', 'face_detection_yunet_2023mar.onnx'])),
        DeclareLaunchArgument('desktop_preview', default_value='false'),
        DeclareLaunchArgument('publish_debug_image', default_value='false'),
        Node(package='rumblex_perception_face', executable='node_face_detector',
             name='node_face_detector', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     'model_path': LaunchConfiguration('model_path'),
                     'compressed_input': ParameterValue(
                         LaunchConfiguration('desktop_preview'), value_type=bool),
                     'allow_approximate_intrinsics': ParameterValue(
                         LaunchConfiguration('desktop_preview'), value_type=bool),
                     'publish_debug_image': ParameterValue(
                         LaunchConfiguration('publish_debug_image'), value_type=bool),
                 }]),
    ])
