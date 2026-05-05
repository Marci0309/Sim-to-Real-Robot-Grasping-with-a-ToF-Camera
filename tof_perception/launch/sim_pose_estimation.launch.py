from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xarm1s_description'),
                'launch', 'spawn_robot.launch.py')
        )
    )

    pose_estimator = TimerAction(period=15.0, actions=[
        Node(
            package='tof_perception',
            executable='pose_estimator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    return LaunchDescription([sim, pose_estimator])
