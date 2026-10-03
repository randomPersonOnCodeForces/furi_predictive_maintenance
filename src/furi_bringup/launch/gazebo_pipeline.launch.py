from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_trajectory = LaunchConfiguration("enable_trajectory")
    trajectory_type = LaunchConfiguration("trajectory_type")
    update_rate_hz = LaunchConfiguration("update_rate_hz")
    faults_enabled = LaunchConfiguration("faults_enabled")
    friction_enabled = LaunchConfiguration("friction_enabled")
    noise_enabled = LaunchConfiguration("noise_enabled")

    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare("furi_description"),
            "urdf",
            "dobot_magician_lite.urdf.xacro",
        ]
    )

    software_config = PathJoinSubstitution(
        [
            FindPackageShare("furi_bringup"),
            "config",
            "software_pipeline.yaml",
        ]
    )

    bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("furi_bringup"),
            "config",
            "gazebo_pipeline.yaml",
        ]
    )

    controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("furi_bringup"),
            "config",
            "gazebo_controllers.yaml",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                robot_xacro,
                " controllers_file:=",
                controllers_config,
            ]),
            value_type=str,
        )
    }

    common_params = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_trajectory", default_value="true"),
            DeclareLaunchArgument("trajectory_type", default_value="sine"),
            DeclareLaunchArgument("update_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("faults_enabled", default_value="false"),
            DeclareLaunchArgument("friction_enabled", default_value="false"),
            DeclareLaunchArgument("noise_enabled", default_value="false"),

            gazebo,

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description, common_params],
            ),

            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_dobot_magician_lite",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "dobot_magician_lite",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                ],
            ),

            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager",
                            "/controller_manager",
                            "--param-file",
                            controllers_config,
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "position_controller",
                            "--controller-manager",
                            "/controller_manager",
                            "--param-file",
                            controllers_config,
                        ],
                        output="screen",
                    ),
                ],
            ),

            Node(
                package="furi_control",
                executable="trajectory_generator",
                name="trajectory_generator",
                output="screen",
                parameters=[
                    software_config,
                    common_params,
                    {
                        "enabled": ParameterValue(enable_trajectory, value_type=bool),
                        "trajectory_type": ParameterValue(trajectory_type, value_type=str),
                        "update_rate_hz": ParameterValue(update_rate_hz, value_type=float),
                    },
                ],
            ),

            Node(
                package="furi_control",
                executable="fault_injector",
                name="fault_injector",
                output="screen",
                parameters=[
                    software_config,
                    common_params,
                    {
                        "faults.enabled": ParameterValue(faults_enabled, value_type=bool),
                        "friction.enabled": ParameterValue(friction_enabled, value_type=bool),
                        "noise.enabled": ParameterValue(noise_enabled, value_type=bool),
                    },
                ],
            ),

            Node(
                package="furi_control",
                executable="sim_robot_bridge",
                name="sim_robot_bridge",
                output="screen",
                parameters=[bridge_config, common_params],
            ),
        ]
    )