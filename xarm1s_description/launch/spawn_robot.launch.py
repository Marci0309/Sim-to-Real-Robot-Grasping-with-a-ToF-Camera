import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc = get_package_share_directory('xarm1s_description')
    pkg_gazebo = get_package_share_directory('xarm1s_gazebo')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'xarm.urdf')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'tabletop.world')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
             f'world:={world_path}'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    spawn = TimerAction(period=10.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description',
                      '-entity', 'xarm1s',
                      '-x', '0.0', '-y', '0.0', '-z', '0.76'],
            output='screen'
        )
    ])

    return LaunchDescription([gazebo, rsp, spawn])
