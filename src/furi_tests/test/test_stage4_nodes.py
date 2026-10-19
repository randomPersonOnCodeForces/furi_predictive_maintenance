"""Exercise node callbacks, input validation, clocks, watchdogs and fault isolation."""

import time

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from furi_control.fault_injector import FaultInjector
from furi_control.fault_models import FrictionConfig, apply_friction_proxy
from furi_control.mock_robot import MockRobot
from furi_control.sim_robot_bridge import SimRobotBridge
from furi_control.trajectory_generator import TrajectoryGenerator


class Capture:
    """Capture published ROS messages without depending on DDS delivery timing."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        """Retain the original message so tests can check mutation isolation."""
        self.messages.append(message)


@pytest.fixture
def nodes():
    """Create real rclpy nodes and destroy every timer/subscription after each test."""
    rclpy.init(args=[])
    created = []

    def create(node_type):
        node = node_type()
        created.append(node)
        for name in ('publisher', 'command_pub', 'clean_state_pub', 'noisy_state_pub'):
            if hasattr(node, name):
                setattr(node, name, Capture())
        return node

    yield create
    for node in created:
        node.destroy_node()
    rclpy.try_shutdown()


def command(values):
    """Construct a velocity command."""
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return msg


def feedback(position=None):
    """Construct complete stamped feedback in deliberately reversed joint order."""
    msg = JointState()
    msg.name = ['joint_4', 'joint_3', 'joint_2', 'joint_1']
    msg.position = list(position if position is not None else [0.4, 0.3, 0.2, 0.1])
    msg.velocity = [0.0] * 4
    msg.header.stamp.sec = 1
    return msg


def test_baseline_and_fault_streams(nodes):
    """Baseline is exact; attenuation and noise affect only their intended streams."""
    node = nodes(FaultInjector)
    raw = command([0.2, -0.2, 0.0, 0.4])
    node.command_callback(raw)
    assert list(node.command_pub.messages[-1].data) == list(raw.data)
    clean = feedback()
    node.joint_state_callback(clean)
    assert list(node.noisy_state_pub.messages[-1].position) == list(clean.position)[::-1]
    parameters = {
        'faults.enabled': True, 'friction.enabled': True, 'noise.enabled': True,
        'friction.viscous': [0.25] * 4, 'friction.coulomb': [0.01] * 4,
        'noise.position_std': [0.02] * 4, 'noise.velocity_std': [0.03] * 4,
    }
    assert all(result.successful for result in node.set_parameters(
        [Parameter(name, value=value) for name, value in parameters.items()]))
    node.command_callback(raw)
    expected = apply_friction_proxy(np.array(raw.data), FrictionConfig(
        np.full(4, 0.25), np.full(4, 0.01), 0.02))
    np.testing.assert_allclose(list(node.command_pub.messages[-1].data), expected)
    assert np.all(np.abs(expected) <= np.abs(raw.data))
    node.joint_state_callback(clean)
    noisy = node.noisy_state_pub.messages[-1]
    assert list(noisy.position) != list(clean.position)[::-1]
    assert list(noisy.velocity) != list(clean.velocity)
    assert list(clean.position) == [0.4, 0.3, 0.2, 0.1]  # No input mutation.
    node.set_parameters([Parameter('faults.enabled', value=False)])
    node.command_callback(raw)
    assert list(node.command_pub.messages[-1].data) == list(raw.data)
    node.joint_state_callback(clean)
    assert list(node.noisy_state_pub.messages[-1].position) == list(clean.position)[::-1]


@pytest.mark.parametrize('name,value', [
    ('friction.viscous', [-1.0] * 4), ('friction.coulomb', [float('nan')] * 4),
    ('friction.smoothing_velocity', 0.0), ('friction.smoothing_velocity', float('inf')),
    ('noise.position_std', [0.1]), ('noise.velocity_std', [-0.1] * 4),
])
def test_invalid_fault_parameters(nodes, name, value):
    """Unsafe live fault configuration is rejected before a callback can crash."""
    node = nodes(FaultInjector)
    assert not node.set_parameters([Parameter(name, value=value)])[0].successful


@pytest.mark.parametrize('values', [[0.1], [float('nan')] * 4, [float('inf')] * 4])
def test_invalid_commands_stop(nodes, values):
    """Malformed commands explicitly stop the fault stage, bridge and mock."""
    fault = nodes(FaultInjector)
    bridge = nodes(SimRobotBridge)
    mock = nodes(MockRobot)
    for node in [fault, bridge, mock]:
        node.command_callback(command([0.2] * 4))
        node.command_callback(command(values))
    assert list(fault.command_pub.messages[-1].data) == [0.0] * 4
    assert not bridge.command_pub.messages  # No position command without measured initialization.
    assert np.all(mock.command == 0)


def test_bridge_watchdogs_and_limits(nodes, monkeypatch):
    """Initialize from feedback, bound dt/positions and hold on stale inputs/reset."""
    from types import SimpleNamespace
    node = nodes(SimRobotBridge)
    tick = {'ns': 1_000_000_000}
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=tick['ns'])))
    node.command_callback(command([9.0] * 4))
    node.publish_command()
    assert not node.command_pub.messages
    node.joint_state_callback(feedback())
    initial = np.array([0.1, 0.2, 0.3, 0.4])
    np.testing.assert_allclose(node.target, initial)
    node.publish_command()
    np.testing.assert_allclose(node.command_pub.messages[-1].data, initial)
    tick['ns'] += 1_000_000_000  # A large clock jump must integrate at most max_step_sec.
    node.publish_command()
    expected = initial + node.velocity_limits * node.max_step_sec
    np.testing.assert_allclose(node.command_pub.messages[-1].data, expected)
    node.last_feedback = time.monotonic() - 1
    node.joint_state_callback(feedback())  # Repeated stamp must not refresh the watchdog.
    tick['ns'] += 10_000_000
    node.publish_command()
    np.testing.assert_allclose(node.target, expected)
    msg = feedback()
    msg.header.stamp.sec = 2
    node.joint_state_callback(msg)
    np.testing.assert_allclose(node.target, initial)  # Recovery reinitializes from feedback.
    node.last_command = time.monotonic() - 1
    tick['ns'] += 10_000_000
    node.publish_command()
    np.testing.assert_allclose(node.target, initial)
    node.target = node.upper_limits.copy()
    node.last_feedback = time.monotonic()
    node.command_callback(command([0.5] * 4))
    tick['ns'] += 10_000_000
    node.publish_command()
    np.testing.assert_allclose(node.target, node.upper_limits)
    node.command_callback(command([-0.5] * 4))
    tick['ns'] += 10_000_000
    node.publish_command()
    assert np.all(node.target < node.upper_limits)
    node.target = node.lower_limits.copy()
    tick['ns'] += 10_000_000
    node.publish_command()
    np.testing.assert_allclose(node.target, node.lower_limits)
    msg.header.stamp.sec = 0
    node.joint_state_callback(msg)
    assert node.last_command is None
    node.stop()
    np.testing.assert_allclose(node.command_pub.messages[-1].data, node.target)


@pytest.mark.parametrize('problem', ['missing', 'short', 'nan', 'duplicate'])
def test_malformed_feedback(nodes, problem):
    """Reject malformed JointState arrays before indexing or publishing clean feedback."""
    bridge = nodes(SimRobotBridge)
    fault = nodes(FaultInjector)
    msg = feedback()
    if problem == 'missing':
        msg.name[0] = 'other_joint'
    elif problem == 'short':
        msg.velocity = [0.0]
    elif problem == 'nan':
        msg.position[0] = float('nan')
    else:
        msg.name[0] = msg.name[1]
    bridge.joint_state_callback(msg)
    fault.joint_state_callback(msg)
    assert not bridge.clean_state_pub.messages and not fault.noisy_state_pub.messages
    assert not bridge.command_pub.messages


def test_friction_proxy_never_reverses():
    """Large attenuation coefficients stop motion rather than create reverse commands."""
    result = apply_friction_proxy(np.array([0.1, -0.1]), FrictionConfig(
        np.array([10.0, 10.0]), np.array([2.0, 2.0]), 0.02))
    np.testing.assert_array_equal(result, [0.0, 0.0])


def test_trajectory_and_mock(nodes):
    """Preserve the lightweight pipeline's velocity semantics and finite-input checks."""
    generator = nodes(TrajectoryGenerator)
    mock = nodes(MockRobot)
    generator.publish_command()
    assert list(generator.publisher.messages[-1].data) == [0.0] * 4
    generator.offset = np.array([0.2] * 4)
    generator.set_parameters([Parameter('enabled', value=True),
                              Parameter('trajectory_type', value='constant')])
    generator.publish_command()
    mock.command_callback(generator.publisher.messages[-1])
    mock.last_update_ns -= 20_000_000
    mock.update()
    assert all(position > 0 for position in list(mock.publisher.messages[-1].position))
    assert not generator.set_parameters([Parameter('trajectory_type', value='bad')])[0].successful
    result = generator.set_parameters([Parameter('duration_sec', value=float('nan'))])
    assert not result[0].successful
    mock.last_command_ns -= 1_000_000_000
    mock.update()
    assert list(mock.publisher.messages[-1].velocity) == [0.0] * 4
