import os
import re
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('xarm1s_description')
    pkg_gazebo = get_package_share_directory('xarm1s_gazebo')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'xarm.urdf')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'tabletop.world')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # gazebo_ros2_control does NOT resolve $(find ...) in plain URDF (only xacro
    # does). Substitute the absolute path here before loading robot_description,
    # otherwise the plugin segfaults on a non-existent file path.
    controllers_yaml = os.path.join(pkg_desc, 'config', 'controllers.yaml')
    robot_desc = robot_desc.replace(
        '$(find xarm1s_description)/config/controllers.yaml',
        controllers_yaml,
    )

    # Humble's gazebo_ros2_control passes the URDF through rcl --param parsing,
    # which feeds the value to YAML. YAML refuses scalars starting with '<',
    # so we strip the <?xml ... ?> declaration and collapse whitespace.
    robot_desc = re.sub(r'<\?xml[^?]*\?>', '', robot_desc)
    robot_desc = re.sub(r'\s+', ' ', robot_desc).strip()

    # Prepend our patched gazebo_ros2_control build to GAZEBO_PLUGIN_PATH so
    # Gazebo loads it instead of the buggy /opt/ros/humble/lib version.
    patched_plugin_dir = os.path.expanduser(
        '~/thesis_ws/install/gazebo_ros2_control/lib')
    plugin_path = patched_plugin_dir + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')

    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
             f'world:={world_path}'],
        output='screen',
        additional_env={'GAZEBO_PLUGIN_PATH': plugin_path},
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

    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.76', '0', '0', '0', 'world', 'xarm_base_link'],
        output='screen'
    )

    # Spawn controllers after the robot is spawned. Order matters:
    # joint_state_broadcaster first, then arm + gripper controllers.
    load_jsb = TimerAction(period=14.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_state_broadcaster'],
            output='screen'
        )
    ])
    load_arm = TimerAction(period=16.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'arm_controller'],
            output='screen'
        )
    ])
    load_gripper = TimerAction(period=18.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'gripper_controller'],
            output='screen'
        )
    ])

    return LaunchDescription([
        gazebo, rsp, spawn, world_tf,
        load_jsb, load_arm, load_gripper,
    ])
