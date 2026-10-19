"""
Opt-in real Gazebo tests: four fresh launches, services, movement and pipeline.

Run with FURI_RUN_GAZEBO_TESTS=1 in a sourced workspace. Each launch is isolated
from normal ROS sessions and reuses its test domain/port to check cleanup.
"""

from pathlib import Path
import os
import re
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from gazebo_msgs.srv import GetModelList
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml


pytestmark = pytest.mark.skipif(
    os.environ.get('FURI_RUN_GAZEBO_TESTS') != '1',
    reason='Set FURI_RUN_GAZEBO_TESTS=1 to run headless Gazebo integration tests')


def spin_until(node, predicate, timeout, description, process=None):
    """Wait on actual ROS callbacks with a monotonic deadline."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None:
            assert process.poll() is None, f'Launch exited early: {description}'
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    pytest.fail(f'Timed out after {timeout}s: {description}')


def call(node, service_type, name, timeout=15):
    """Require an actual service response, not just DDS service discovery."""
    client = node.create_client(service_type, name)
    assert client.wait_for_service(timeout_sec=timeout), f'Missing service {name}'
    future = client.call_async(service_type.Request())
    spin_until(node, future.done, timeout, name)
    assert future.exception() is None
    response = future.result()
    node.destroy_client(client)
    return response


@pytest.mark.parametrize('run', range(4), ids=['constant', 'sine', 'faults', 'limits'])
def test_gazebo_pipeline(run, tmp_path, monkeypatch):
    """Prove startup, movement, limits, baseline/fault behavior and clean shutdown."""
    domain = int(os.environ.get('FURI_TEST_DOMAIN_BASE', '81'))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('GAZEBO_MASTER_URI', f'http://127.0.0.1:{11400 + domain}')
    monkeypatch.setenv('GAZEBO_MODEL_DATABASE_URI', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    bringup = Path(get_package_share_directory('furi_bringup'))
    config = yaml.safe_load((bringup / 'config/software_pipeline.yaml').read_text())
    generator = config['trajectory_generator']['ros__parameters']
    generator['offset'] = [0.08] * 4 if run != 1 else [0.0] * 4
    if run == 3:
        generator['offset'] = [1.0, 1.0, 1.0, 1.5]
    generator['amplitude'] = [0.08] * 4
    generator['frequency'] = [0.1] * 4
    fault_config = config['fault_injector']['ros__parameters']
    fault_config['friction']['viscous'] = [0.25] * 4
    fault_config['friction']['coulomb'] = [0.01] * 4
    config_file = tmp_path / 'pipeline.yaml'
    config_file.write_text(yaml.safe_dump(config))
    launch_log = tmp_path / 'launch.log'
    output = launch_log.open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'furi_bringup', 'gazebo_pipeline.launch.py', 'gui:=false',
        'startup_timeout:=45.0', 'enable_trajectory:=true',
        f'trajectory_type:={"sine" if run == 1 else "constant"}',
        f'faults_enabled:={str(run == 2).lower()}',
        f'noise_enabled:={str(run == 2).lower()}',
        f'friction_enabled:={str(run == 2).lower()}', f'pipeline_config:={config_file}',
    ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node(f'stage4_validation_{run}')
    samples, raw, applied, controller_commands, noisy = [], [], [], [], {}
    names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

    def state_callback(msg):
        assert msg.name == names
        assert len(msg.position) == 4 and np.isfinite(msg.position).all()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        samples.append((stamp, np.array(msg.position), key, msg))

    def velocity_callback(target):
        def callback(msg):
            assert len(msg.data) == 4 and np.isfinite(msg.data).all()
            target.append((samples[-1][0] if samples else None, np.array(msg.data)))
        return callback

    node.create_subscription(JointState, '/furi/joint_states/clean', state_callback, 100)
    node.create_subscription(Float64MultiArray, '/furi/command/raw', velocity_callback(raw), 100)
    node.create_subscription(
        Float64MultiArray, '/furi/command/applied', velocity_callback(applied), 100)
    node.create_subscription(Float64MultiArray, '/position_controller/commands',
                             velocity_callback(controller_commands), 100)

    def noisy_callback(msg):
        noisy[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg

    node.create_subscription(JointState, '/furi/joint_states/noisy', noisy_callback, 100)
    try:
        spin_until(node, lambda: 'FURI controllers active;' in launch_log.read_text(),
                   50, 'successful spawn and both spawners', process)
        controllers = call(
            node, ListControllers, '/controller_manager/list_controllers', timeout=50)
        states = {controller.name: controller.state for controller in controllers.controller}
        # Service response may precede sequential spawner completion. Require active states.
        deadline = time.monotonic() + 20
        while states != {'joint_state_broadcaster': 'active', 'position_controller': 'active'}:
            assert time.monotonic() < deadline, states
            controllers = call(
                node, ListControllers, '/controller_manager/list_controllers', timeout=5)
            states = {controller.name: controller.state for controller in controllers.controller}
            rclpy.spin_once(node, timeout_sec=0.05)
        interfaces = call(
            node, ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
        assert {interface.name for interface in interfaces.command_interfaces
                if interface.is_claimed and interface.is_available} == {
                    f'{name}/position' for name in names}
        spin_until(node, lambda: len(samples) > 100 and samples[-1][0] - samples[0][0] >= 3.0
                   and len(applied) > 100 and len(raw) > 100 and len(controller_commands) > 100,
                   20, 'three simulation seconds of complete pipeline feedback', process)
        positions = np.array([sample[1] for sample in samples])
        limits = yaml.safe_load((bringup / 'config/gazebo_pipeline.yaml').read_text())
        limits = limits['sim_robot_bridge']['ros__parameters']
        assert np.all(positions >= np.array(limits['lower_limits']) - 0.02)
        assert np.all(positions <= np.array(limits['upper_limits']) + 0.02)
        np.savez(tmp_path / 'samples.npz', positions=positions,
                 stamps=np.array([sample[0] for sample in samples]),
                 applied=np.array([value for _, value in applied]),
                 targets=np.array([value for _, value in controller_commands]))
        displacement = positions[-1] - positions[0]
        assert np.all(displacement > 0.015), f'Expected positive joint motion: {displacement}'
        commands = np.array([value for stamp, value in applied if stamp is not None])
        times = np.array([stamp for stamp, value in applied if stamp is not None])
        assert np.max(np.abs(commands)) <= (1.5 if run == 3 else 0.081)
        predicted = np.trapz(commands, times, axis=0)
        if run == 3:
            predicted = np.minimum(predicted, np.array(limits['upper_limits']) - positions[0])
            np.testing.assert_allclose(positions[-1], limits['upper_limits'], atol=0.02)
        error = np.abs(displacement - predicted)
        assert np.all(error < 0.045), f'Tracking integral mismatch: {displacement}, {predicted}'
        raw_values = np.array([value for _, value in raw])
        if run == 3:
            np.testing.assert_allclose(raw_values, np.tile([1.0, 1.0, 1.0, 1.5], (len(raw), 1)))
        elif run != 1:
            assert np.allclose(raw_values, 0.08)
            expected = 0.08 if run == 0 else 0.08 * 0.75 - 0.01 * np.tanh(0.08 / 0.02)
            assert np.allclose(commands, expected)
        else:
            assert np.ptp(raw_values[:, 0]) > 0.03  # Require changing generator output.
            # Float64MultiArray has no stamp. Match by value rather than assuming
            # independent DDS streams have identical queue offsets.
            assert all(np.min(np.abs(raw_values[:, 0] - value)) < 0.001
                       for value in commands[:, 0])
        paired = [(sample[3], noisy[sample[2]]) for sample in samples if sample[2] in noisy]
        assert len(paired) > 50
        differences = np.array([np.array(dirty.position) - np.array(clean.position)
                                for clean, dirty in paired])
        if run == 2:
            assert np.max(np.abs(differences)) > 0.001
            assert np.std(differences) == pytest.approx(0.002, abs=0.0006)
        else:
            assert np.all(differences == 0)
        # Validate a live bad parameter request is rejected without killing the node.
        parameters = node.create_client(SetParameters, '/fault_injector/set_parameters')
        assert parameters.wait_for_service(timeout_sec=5)
        request = SetParameters.Request()
        request.parameters = [
            Parameter('friction.smoothing_velocity', value=0.0).to_parameter_msg()]
        future = parameters.call_async(request)
        spin_until(node, future.done, 5, 'invalid parameter rejection')
        assert not future.result().results[0].successful
        graph = node.get_node_names_and_namespaces()
        assert sum(name == 'controller_manager' for name, _ in graph) == 1
        assert not any(name.startswith('spawner_') for name, _ in graph)
        assert node.count_publishers('/joint_states') == 1
        assert node.count_publishers('/furi/joint_states/clean') == 1
        spawn_message = 'Successfully spawned entity [dobot_magician_lite]'
        assert launch_log.read_text().count(spawn_message) == 1
        models = call(node, GetModelList, '/get_model_list')
        assert models.model_names.count('dobot_magician_lite') == 1
        rates = np.diff([sample[0] for sample in samples])
        print(f'run={run} displacement={displacement.tolist()} tracking_error={error.tolist()} '
              f'feedback_hz_sim={1 / np.median(rates):.2f} '
              f'samples={len(samples)} log={launch_log}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        forced = False
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        output.close()
        log = launch_log.read_text()
        print(f'Gazebo launch log: {launch_log}')
        assert not forced, 'Launch cleanup exceeded 15 seconds'
        assert process.returncode == 0, log[-6000:]
        assert 'process has died' not in log and 'Traceback' not in log, log[-6000:]
        for pid in re.findall(r'process started with pid \[(\d+)\]', log):
            assert not Path(f'/proc/{pid}').exists(), f'Leaked child process {pid}'


def test_startup_deadline(tmp_path, monkeypatch):
    """An intentionally impossible startup deadline fails without hanging spawners."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '89')
    monkeypatch.setenv('GAZEBO_MASTER_URI', 'http://127.0.0.1:11489')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    result = subprocess.run(
        ['ros2', 'launch', 'furi_bringup', 'gazebo_pipeline.launch.py',
         'gui:=false', 'startup_timeout:=0.1'],
        capture_output=True, text=True, timeout=20)
    (tmp_path / 'deadline.log').write_text(result.stdout + result.stderr)
    assert result.returncode != 0
    assert 'FURI startup deadline exceeded' in result.stdout + result.stderr


def test_mock_launch(tmp_path, monkeypatch):
    """The separate lightweight ROS pipeline still transports commands and moves joints."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '88')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    output = (tmp_path / 'mock.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'furi_bringup', 'software_pipeline.launch.py',
         'enable_trajectory:=true'],
        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('stage4_mock_validation')
    states, commands = [], []
    node.create_subscription(JointState, '/furi/joint_states/clean', states.append, 100)
    node.create_subscription(Float64MultiArray, '/furi/command/applied', commands.append, 100)
    try:
        spin_until(node, lambda: len(states) > 150 and len(commands) > 100,
                   15, 'mock command and clean feedback streams', process)
        assert states[-1].name == ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        positions = np.array([msg.position for msg in states])
        assert np.isfinite(positions).all()
        assert np.all(positions[-1] - positions[0] > 0.01)
        assert node.count_publishers('/furi/joint_states/clean') == 1
        assert not any(name == 'controller_manager'
                       for name, _ in node.get_node_names_and_namespaces())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            pytest.fail('Mock launch did not clean up')
        output.close()
        log = (tmp_path / 'mock.log').read_text()
        assert process.returncode == 0 and 'Traceback' not in log, log
