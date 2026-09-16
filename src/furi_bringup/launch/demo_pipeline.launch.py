from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    launch_file = os.path.join(
        get_package_share_directory("furi_bringup"),
        "launch",
        "software_pipeline.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "enable_trajectory": "true",
                    "trajectory_type": "sine",
                    "update_rate_hz": "100.0",
                    "faults_enabled": "true",
                    "friction_enabled": "false",
                    "noise_enabled": "true",
                }.items(),
            )
        ]
    )