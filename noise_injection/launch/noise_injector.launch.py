"""
Launch the depth noise injector with default Nguyen 2012 parameters.
Override any param via launch CLI, e.g.:
    ros2 launch noise_injection noise_injector.launch.py axial_scale:=2.0 dropout_probability:=0.05
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('axial_scale',         default_value='1.0'),
        DeclareLaunchArgument('lateral_sigma_px',    default_value='0.8'),
        DeclareLaunchArgument('edge_probability',    default_value='0.30'),
        DeclareLaunchArgument('dropout_probability', default_value='0.02'),
        DeclareLaunchArgument('seed',                default_value='42'),
    ]

    node = Node(
        package='noise_injection',
        executable='depth_noise_injector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'axial_scale':         LaunchConfiguration('axial_scale'),
            'lateral_sigma_px':    LaunchConfiguration('lateral_sigma_px'),
            'edge_probability':    LaunchConfiguration('edge_probability'),
            'dropout_probability': LaunchConfiguration('dropout_probability'),
            'seed':                LaunchConfiguration('seed'),
        }],
    )

    return LaunchDescription(args + [node])
