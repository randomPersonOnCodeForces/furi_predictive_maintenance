from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_file = os.path.join(
        get_package_share_directory("furi_bringup"),
        "config",
        "software_pipeline.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_trajectory = LaunchConfiguration("enable_trajectory")
    trajectory_type = LaunchConfiguration("trajectory_type")
    update_rate_hz = LaunchConfiguration("update_rate_hz")
    faults_enabled = LaunchConfiguration("faults_enabled")
    friction_enabled = LaunchConfiguration("friction_enabled")
    noise_enabled = LaunchConfiguration("noise_enabled")

    common_params = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulated ROS time.",
            ),
            DeclareLaunchArgument(
                "enable_trajectory",
                default_value="false",
                description="Enable generated joint commands.",
            ),
            DeclareLaunchArgument(
                "trajectory_type",
                default_value="sine",
                description="Trajectory type: sine, constant, or square.",
            ),
            DeclareLaunchArgument(
                "update_rate_hz",
                default_value="100.0",
                description="Pipeline update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "faults_enabled",
                default_value="false",
                description="Enable the fault injection stage.",
            ),
            DeclareLaunchArgument(
                "friction_enabled",
                default_value="false",
                description="Enable friction fault injection.",
            ),
            DeclareLaunchArgument(
                "noise_enabled",
                default_value="false",
                description="Enable sensor noise injection.",
            ),
            Node(
                package="furi_control",
                executable="trajectory_generator",
                name="trajectory_generator",
                output="screen",
                parameters=[
                    config_file,
                    common_params,
                    {
                        "enabled": ParameterValue(
                            enable_trajectory,
                            value_type=bool,
                        ),
                        "trajectory_type": ParameterValue(
                            trajectory_type,
                            value_type=str,
                        ),
                        "update_rate_hz": ParameterValue(
                            update_rate_hz,
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="furi_control",
                executable="fault_injector",
                name="fault_injector",
                output="screen",
                parameters=[
                    config_file,
                    common_params,
                    {
                        "faults.enabled": ParameterValue(
                            faults_enabled,
                            value_type=bool,
                        ),
                        "friction.enabled": ParameterValue(
                            friction_enabled,
                            value_type=bool,
                        ),
                        "noise.enabled": ParameterValue(
                            noise_enabled,
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="furi_control",
                executable="mock_robot",
                name="mock_robot",
                output="screen",
                parameters=[
                    config_file,
                    common_params,
                    {
                        "update_rate_hz": ParameterValue(
                            update_rate_hz,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )