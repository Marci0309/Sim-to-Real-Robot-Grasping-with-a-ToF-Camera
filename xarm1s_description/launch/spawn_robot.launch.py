import os
import re
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
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

    # ─── Gazebo environment ────────────────────────────────────────────
    # 1) PLUGIN PATH: previously we prepended a locally-patched
    #    gazebo_ros2_control build at ~/thesis_ws/install/gazebo_ros2_control/lib
    #    to work around a Humble-era race in controller activation.
    #    As of 2026-05-11 the patched build silently fails to register the
    #    controller_manager service, so we fall back to the upstream Humble
    #    build (/opt/ros/humble/lib/libgazebo_ros2_control.so).
    #
    #    CORRECTION (2026-05-30): the upstream build is the BROKEN one on this
    #    Humble — it forwards robot_description to the controller_manager as a
    #    `--param` CLI arg, and RCL's parser rejects our URDF value ("Couldn't
    #    parse parameter override rule: --param robot_description:=<robot ..."),
    #    so the controller_manager never starts, controllers never activate, and
    #    the arm hangs limp. The vendored build in src/_thirdparty does NOT have
    #    this bug. We therefore PREPEND the vendored plugin dir so it always wins
    #    — otherwise a shell that exports GAZEBO_PLUGIN_PATH=...:/opt/ros/humble/lib
    #    (e.g. via ~/.bashrc) silently selects the broken upstream plugin.
    install_root = os.path.dirname(os.path.dirname(os.path.dirname(pkg_desc)))
    vendored_plugin_dir = os.path.join(install_root, 'gazebo_ros2_control', 'lib')
    # our MIT fixed-joint grasp plugin (libxarm1s_link_attacher.so) lives here
    grasp_plugin_dir = os.path.join(install_root, 'xarm1s_grasp_plugin', 'lib')
    plugin_path = (vendored_plugin_dir + ':' + grasp_plugin_dir + ':'
                   + os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    # 2) MODEL PATH: tabletop.world has <include><uri>model://banana</uri></include>
    #    etc. Gazebo resolves these by searching GAZEBO_MODEL_PATH for
    #    <name>/model.sdf. Without this prepend the shell's GAZEBO_MODEL_PATH
    #    is typically empty, the includes fail with "Unable to find uri[…]",
    #    and the corresponding objects silently disappear from the scene.
    models_dir = os.path.join(pkg_gazebo, 'models', 'tabletop_objects')
    model_path = models_dir + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')

    # 3) MODEL DATABASE URI: disable the online gazebosim.org fetch. On
    #    constrained networks (WSL bridged via Hyper-V; lab firewalls) the
    #    fetch can hang for tens of seconds and starve plugin initialisation
    #    so badly that gazebo_ros2_control never registers
    #    /controller_manager. All models we use are local (bundled with
    #    Gazebo 11 for ground_plane/sun, in xarm1s_gazebo for the four
    #    objects), so we don't need the online DB.
    gazebo_env = {
        'GAZEBO_PLUGIN_PATH':       plugin_path,
        'GAZEBO_MODEL_PATH':        model_path,
        'GAZEBO_MODEL_DATABASE_URI': '',
    }

    # gui:=false runs gzserver headless (no gzclient window) -- used for
    # automated/headless verification and faster data sweeps.
    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
             f'world:={world_path}', ['gui:=', LaunchConfiguration('gui')]],
        output='screen',
        additional_env=gazebo_env,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    # The URDF has a `world` virtual link + fixed joint at z = 0.76 m, so
    # robot_state_publisher already publishes world -> xarm_base_link.
    # No static_transform_publisher needed; spawning at world origin lets
    # the fixed joint place the base on the table.
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'xarm1s',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-timeout', '90'],
        output='screen',
    )

    # The controller_manager service appears once gazebo_ros2_control loads
    # inside the spawned robot. The `spawner` node waits for it indefinitely
    # (default), which is more reliable than the `ros2 control load_controller`
    # CLI in Humble. Order: joint_state_broadcaster first; arm + gripper after.
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Real dependency chain (no guessed timer offsets): once spawn_entity
    # exits cleanly, kick off the joint_state_broadcaster spawner; once that
    # exits, kick off arm + gripper concurrently. The Humble race that
    # bricked OnProcessExit before was rooted in spawn never completing
    # (missing GAZEBO_MODEL_PATH cascaded through to a dead /spawn_entity
    # service); with that fixed, OnProcessExit is the right primitive.
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])
    )
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm, spawn_gripper])
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = headless gzserver (no window)'),
        gazebo, rsp, spawn,
        after_spawn, after_jsb,
    ])
