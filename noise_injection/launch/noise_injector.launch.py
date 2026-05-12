"""
Launch the depth noise injector.

By default loads the OAK-D-measured noise parameters (Phase 2). Switch to
any other YAML in noise_injection/config/ via the params_file arg.
Per-parameter ablation overrides remain available via `--ros-args -p`.

Examples:
    # Default: measured OAK-D config
    ros2 launch noise_injection noise_injector.launch.py

    # Compare against the Nguyen Kinect literature defaults
    ros2 launch noise_injection noise_injector.launch.py \\
        params_file:=$(ros2 pkg prefix noise_injection)/share/noise_injection/config/nguyen_kinect.yaml

    # Ablation sweep on top of whichever YAML is loaded
    ros2 launch noise_injection noise_injector.launch.py \\
        --ros-args -p axial_scale:=2.0
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('noise_injection')
    default_params = os.path.join(pkg_share, 'config', 'oakd_measured.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to a parameter YAML for depth_noise_injector. '
                        'Defaults to the OAK-D measured config from Phase 2.',
        ),
    ]

    node = Node(
        package='noise_injection',
        executable='depth_noise_injector',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(args + [node])
