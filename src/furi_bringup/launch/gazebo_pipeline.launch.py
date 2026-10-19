"""Run the velocity-to-position pipeline with one Gazebo-owned controller manager."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, Shutdown, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    """Chain successful spawn and controller activation, with a wall-time deadline."""
    bringup = Path(get_package_share_directory('furi_bringup'))
    description = Path(get_package_share_directory('furi_description'))
    controllers = str(bringup / 'config' / 'gazebo_controllers.yaml')
    document = xacro.process_file(
        str(description / 'urdf' / 'dobot_magician_lite.urdf.xacro'),
        mappings={'controllers_file': controllers, 'enable_gazebo': 'true'},
    )
    # Humble's plugin reparses URDF as a YAML CLI parameter. XML comments
    # containing ': ' fail that parser. ElementTree removes comments and headers.
    robot_xml = ET.tostring(ET.fromstring(document.toxml()), encoding='unicode')
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    rate = {
        'update_rate_hz': ParameterValue(LaunchConfiguration('update_rate_hz'), value_type=float)
    }
    software_config = LaunchConfiguration('pipeline_config')
    timeout = LaunchConfiguration('startup_timeout')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('gazebo_ros')) / 'launch' / 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'), 'pause': 'false',
            'server_required': 'true', 'verbose': LaunchConfiguration('verbose'),
            'params_file': str(bringup / 'config' / 'gazebo_pipeline.yaml'),
        }.items(),
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_xml}, common],
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', '/robot_description', '-entity', 'dobot_magician_lite',
                   '-timeout', timeout],
    )

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[controller, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', timeout,
                       '--service-call-timeout', '5', '--switch-timeout', '10'],
        )

    broadcaster = spawner('joint_state_broadcaster')
    position = spawner('position_controller')
    bridge = Node(
        package='furi_control', executable='sim_robot_bridge', name='sim_robot_bridge',
        output='screen',
        parameters=[str(bringup / 'config' / 'gazebo_pipeline.yaml'), common, rate],
    )
    faults = Node(
        package='furi_control', executable='fault_injector', name='fault_injector',
        output='screen', parameters=[software_config, common, {
            f'{name}.enabled': ParameterValue(LaunchConfiguration(arg), value_type=bool)
            for name, arg in [('faults', 'faults_enabled'), ('friction', 'friction_enabled'),
                              ('noise', 'noise_enabled')]
        }],
    )
    trajectory = Node(
        package='furi_control', executable='trajectory_generator', name='trajectory_generator',
        output='screen', parameters=[software_config, common, rate, {
            'enabled': ParameterValue(LaunchConfiguration('enable_trajectory'), value_type=bool),
            'trajectory_type': LaunchConfiguration('trajectory_type'),
        }],
    )
    ready = {'value': False}

    def advance(label, actions):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                reason = (f'{label} failed (exit {event.returncode}); '
                          'inspect Gazebo/plugin logs above.')
                raise RuntimeError(reason)
            return actions
        return callback

    def activated(event, context):
        if event.returncode != 0 or context.is_shutdown:
            return advance('Position controller activation', [])(event, context)
        ready['value'] = True
        return [LogInfo(msg='FURI controllers active; starting velocity-to-position pipeline.'),
                bridge, faults, trajectory]

    def deadline(context):
        if not ready['value']:
            reason = ('FURI startup deadline exceeded: require spawn and active controllers. '
                      'Check robot_description parsing, controller YAML and advancing /clock.')
            raise RuntimeError(reason)
        return []

    # This is a deadline, not a startup delay; successful process exits advance startup.
    watchdog = TimerAction(period=timeout, actions=[OpaqueFunction(function=deadline)])
    args = [
        ('use_sim_time', 'true'), ('enable_trajectory', 'true'), ('trajectory_type', 'sine'),
        ('update_rate_hz', '100.0'), ('faults_enabled', 'false'), ('friction_enabled', 'false'),
        ('noise_enabled', 'false'), ('gui', 'true'), ('startup_timeout', '60.0'),
        ('verbose', 'false'),
        ('pipeline_config', str(bringup / 'config' / 'software_pipeline.yaml')),
    ]
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=advance('Robot spawn', [broadcaster]))),
        RegisterEventHandler(OnProcessExit(
            target_action=broadcaster, on_exit=advance('Joint-state broadcaster', [position]))),
        RegisterEventHandler(OnProcessExit(target_action=position, on_exit=activated)),
    ]
    for process in [rsp, bridge, faults, trajectory]:
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                Shutdown(reason='Required FURI node exited; stopping simulation.')],
        )))
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in args]
        + handlers + [watchdog, gazebo, rsp, spawn]
    )
