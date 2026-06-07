"""Launch the Phase 4A grasp_executor against an already-running sim.

Run the sim first (Terminal 1):
    ros2 launch tof_perception sim_pose_estimation.launch.py
Then, for ground-truth bring-up:
    ros2 launch xarm1s_manipulation manipulation.launch.py target_source:=ground_truth
For a perception-driven experiment run:
    ros2 launch xarm1s_manipulation manipulation.launch.py \\
        target_source:=perception run_tag:=sim_clean n_trials:=30
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("xarm1s_manipulation"),
        "config", "manip_params.yaml")

    args = [
        DeclareLaunchArgument("run_tag", default_value="sim_clean"),
        DeclareLaunchArgument("target_source", default_value="ground_truth",
                              description="ground_truth | perception"),
        DeclareLaunchArgument("n_trials", default_value="1"),
        DeclareLaunchArgument("carry_object", default_value="true"),
        DeclareLaunchArgument("bias_correction", default_value="true"),
        DeclareLaunchArgument("execute_motion", default_value="true",
                              description="false = sample perception + analytic "
                                          "capture only (fast sweep, no arm motion)"),
        DeclareLaunchArgument("vary_placement", default_value="false",
                              description="true = success-rate sweep: teleport each "
                                          "object to n_trials varied reachable poses"),
        DeclareLaunchArgument("placement_seed", default_value="0"),
    ]

    node = Node(
        package="xarm1s_manipulation",
        executable="grasp_executor",
        name="grasp_executor",
        output="screen",
        parameters=[config, {
            "use_sim_time": True,
            "run_tag": LaunchConfiguration("run_tag"),
            "target_source": LaunchConfiguration("target_source"),
            "n_trials": ParameterValue(LaunchConfiguration("n_trials"),
                                       value_type=int),
            "carry_object": ParameterValue(LaunchConfiguration("carry_object"),
                                           value_type=bool),
            "bias_correction": ParameterValue(
                LaunchConfiguration("bias_correction"), value_type=bool),
            "execute_motion": ParameterValue(
                LaunchConfiguration("execute_motion"), value_type=bool),
            "vary_placement": ParameterValue(
                LaunchConfiguration("vary_placement"), value_type=bool),
            "placement_seed": ParameterValue(
                LaunchConfiguration("placement_seed"), value_type=int),
        }],
    )

    return LaunchDescription(args + [node])
