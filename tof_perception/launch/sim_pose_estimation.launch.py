from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # depth_topic chooses which depth stream perception consumes. Default is
    # the clean Gazebo plugin output (Sim-clean condition); pass
    # depth_topic:=/overhead_depth/depth/image_noisy for the Sim-noisy
    # condition (paired with the noise_injection package on Terminal 2).
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/overhead_depth/depth/image_raw',
        description='Depth topic that pose_estimator subscribes to. Switch '
                    'to /overhead_depth/depth/image_noisy for Sim-noisy.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='false = headless gzserver (no window); faster for sweeps')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xarm1s_description'),
                'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )

    pose_estimator = TimerAction(period=15.0, actions=[
        Node(
            package='tof_perception',
            executable='pose_estimator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'depth_topic': LaunchConfiguration('depth_topic'),
            }],
        )
    ])

    return LaunchDescription([depth_topic_arg, gui_arg, sim, pose_estimator])
