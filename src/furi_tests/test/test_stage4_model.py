"""Validate installed model/config contracts and the Humble parameter-parser regression."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import rclpy
import xacro
import yaml


@pytest.mark.parametrize('gazebo', [False, True])
def test_model(gazebo, tmp_path):
    """Expand both Stage 3 and Stage 4 variants and inspect the real URDF."""
    description = Path(get_package_share_directory('furi_description'))
    bringup = Path(get_package_share_directory('furi_bringup'))
    controllers_file = bringup / 'config/gazebo_controllers.yaml'
    document = xacro.process_file(
        str(description / 'urdf/dobot_magician_lite.urdf.xacro'),
        mappings={'enable_gazebo': str(gazebo).lower(), 'controllers_file': str(controllers_file)})
    root = ET.fromstring(document.toxml())
    robot_xml = ET.tostring(root, encoding='unicode')
    model_file = tmp_path / 'dobot.urdf'
    model_file.write_text(robot_xml)
    result = subprocess.run(
        ['check_urdf', str(model_file)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    joints = root.findall("joint[@type='revolute']")
    names = [joint.attrib['name'] for joint in joints]
    assert names == ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    config = yaml.safe_load((description / 'config/joint_limits.yaml').read_text())
    config = config['dobot_magician_lite']
    bridge = yaml.safe_load((bringup / 'config/gazebo_pipeline.yaml').read_text())
    bridge = bridge['sim_robot_bridge']['ros__parameters']
    software = yaml.safe_load((bringup / 'config/software_pipeline.yaml').read_text())
    controllers = yaml.safe_load(controllers_file.read_text())
    assert controllers['position_controller']['ros__parameters']['joints'] == names
    assert bridge['joint_names'] == names
    assert controllers['position_controller']['ros__parameters']['interface_name'] == 'position'
    assert config['joint_names'] == names
    for node in ['trajectory_generator', 'fault_injector', 'mock_robot']:
        assert software[node]['ros__parameters']['joint_names'] == names
    for i, joint in enumerate(joints):
        limits = {key: float(value) for key, value in joint.find('limit').attrib.items()}
        assert np.isfinite(list(limits.values())).all()
        assert limits['lower'] < 0 < limits['upper']
        assert limits['velocity'] > 0 and limits['effort'] > 0
        reference = config['joint_limits'][names[i]]
        assert limits['lower'] == bridge['lower_limits'][i] == reference['lower']
        assert limits['upper'] == bridge['upper_limits'][i] == reference['upper']
        assert 0 < bridge['velocity_limits'][i] <= limits['velocity']
        assert float(config['joint_limits'][names[i]]['velocity']) == limits['velocity']
        axis = np.fromstring(joint.find('axis').attrib['xyz'], sep=' ')
        assert np.linalg.norm(axis) == pytest.approx(1)
        assert axis.tolist() == ([0, -1, 0] if i in (1, 2) else [0, 0, 1])
    for link in root.findall('link'):
        if link.attrib['name'] in ('world', 'base_footprint'):
            continue
        assert link.find('collision/geometry') is not None
        assert float(link.find('inertial/mass').attrib['value']) > 0
        inertia = {key: float(value)
                   for key, value in link.find('inertial/inertia').attrib.items()}
        matrix = np.array([[inertia['ixx'], inertia['ixy'], inertia['ixz']],
                           [inertia['ixy'], inertia['iyy'], inertia['iyz']],
                           [inertia['ixz'], inertia['iyz'], inertia['izz']]])
        eig = np.linalg.eigvalsh(matrix)
        assert np.all(eig > 0) and eig[-1] <= eig[0] + eig[1] + 1e-12
    control = root.find('ros2_control')
    if not gazebo:
        assert control is None and root.find('gazebo/plugin') is None
        assert root.find("joint[@name='world_mount']") is None
        return
    assert root.find("joint[@name='world_mount']/parent").attrib['link'] == 'world'
    assert [joint.attrib['name'] for joint in control.findall('joint')] == names
    assert control.find('hardware/plugin').text == 'gazebo_ros2_control/GazeboSystem'
    assert len(root.findall('gazebo/plugin')) == 1
    assert root.find('gazebo/plugin/parameters').text == str(controllers_file)
    for joint in control.findall('joint'):
        assert [item.attrib['name'] for item in joint.findall('command_interface')] == ['position']
        states = {item.attrib['name'] for item in joint.findall('state_interface')}
        assert states == {'position', 'velocity', 'effort'}
        limits = config['joint_limits'][joint.attrib['name']]
        params = {item.attrib['name']: float(item.text)
                  for item in joint.findall('command_interface/param')}
        assert params == {'min': limits['lower'], 'max': limits['upper']}
    # Exercise the exact CLI parser used inside gazebo_ros2_control, not just XML parsing.
    context = rclpy.context.Context()
    try:
        context.init(args=['--ros-args', '-p', 'robot_description:=' + robot_xml])
        assert context.ok()
    finally:
        context.try_shutdown()
