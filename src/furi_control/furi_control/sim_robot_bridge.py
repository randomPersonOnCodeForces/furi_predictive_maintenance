"""Integrate FURI velocities (rad/s) into bounded Gazebo position targets (rad)."""

from __future__ import annotations

import time

import numpy as np
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


from furi_control.runtime import run_node


class SimRobotBridge(Node):
    """Validate commands/feedback and stop on stale inputs or joint limits."""

    def __init__(self) -> None:
        super().__init__('sim_robot_bridge')
        defaults = {
            'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            'command_input_topic': '/furi/command/applied',
            'controller_command_topic': '/position_controller/commands',
            'joint_state_input_topic': '/joint_states',
            'clean_joint_state_topic': '/furi/joint_states/clean',
            'update_rate_hz': 100.0,
            'command_timeout_sec': 0.2,
            'feedback_timeout_sec': 0.2,
            'max_step_sec': 0.02,
            'lower_limits': [-2.35619449, -0.08726646, -0.17453293, -2.53072742],
            'upper_limits': [2.35619449, 1.39626340, 1.48352986, 2.53072742],
            'velocity_limits': [1.0, 1.0, 1.0, 1.5],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.joint_count = len(self.joint_names)
        if not self.joint_count or len(set(self.joint_names)) != self.joint_count:
            raise ValueError('joint_names must be nonempty and unique')
        for name in ['lower_limits', 'upper_limits', 'velocity_limits']:
            values = np.asarray(self.get_parameter(name).value, dtype=float)
            if values.shape != (self.joint_count,) or not np.all(np.isfinite(values)):
                raise ValueError(f'{name} must contain one finite value per joint')
            setattr(self, name, values)
        if np.any(self.lower_limits >= self.upper_limits) or np.any(self.velocity_limits <= 0):
            raise ValueError('Require ordered position limits and positive velocity limits')
        for name in ['update_rate_hz', 'command_timeout_sec', 'feedback_timeout_sec',
                     'max_step_sec']:
            value = float(self.get_parameter(name).value)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        self.command = np.zeros(self.joint_count)
        self.position = None
        self.target = None
        self.last_update_sim = None
        self.feedback_valid = False
        self.last_command = None
        self.last_feedback = None
        self.last_stamp = None
        self.command_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter('controller_command_topic').value), 1)
        self.clean_state_pub = self.create_publisher(
            JointState, str(self.get_parameter('clean_joint_state_topic').value), 10)
        self.create_subscription(
            Float64MultiArray, str(self.get_parameter('command_input_topic').value),
            self.command_callback, 1)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_input_topic').value),
            self.joint_state_callback, qos_profile_sensor_data)
        # A wall clock watchdog must still stop latched velocity commands if /clock stalls.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.update_rate_hz, self.publish_command,
                          clock=self.watchdog_clock)

    def stop(self):
        """Clear velocity and hold the last finite position target, including on shutdown."""
        self.command.fill(0.0)
        self.last_command = None
        if self.context.ok() and self.target is not None:
            msg = Float64MultiArray()
            msg.data = self.target.tolist()
            self.command_pub.publish(msg)

    def command_callback(self, msg):
        """Accept only complete, finite velocity vectors."""
        values = np.asarray(msg.data, dtype=float)
        if values.shape != (self.joint_count,) or not np.all(np.isfinite(values)):
            self.get_logger().warning('Rejected malformed velocity command; stopping')
            self.stop()
            return
        self.command = np.clip(values, -self.velocity_limits, self.velocity_limits)
        self.last_command = time.monotonic()

    def joint_state_callback(self, msg):
        """Publish ordered clean feedback only after validating every array."""
        count = len(msg.name)
        valid = len(set(msg.name)) == count and len(msg.position) == count
        valid = valid and all(name in msg.name for name in self.joint_names)
        for values in [msg.position, msg.velocity, msg.effort]:
            valid = valid and (len(values) in (0, count)) and np.all(np.isfinite(values))
        if not valid:
            self.position = None
            self.last_feedback = None
            self.feedback_valid = False
            self.stop()
            self.get_logger().warning('Rejected incomplete or nonfinite simulated feedback')
            return
        indices = [msg.name.index(name) for name in self.joint_names]
        out = JointState()
        out.header = msg.header
        out.name = self.joint_names
        out.position = [msg.position[i] for i in indices]
        out.velocity = [msg.velocity[i] for i in indices] if msg.velocity else []
        out.effort = [msg.effort[i] for i in indices] if msg.effort else []
        self.position = np.asarray(out.position)
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.stop()  # A simulation reset must not replay a pre-reset command.
            self.feedback_valid = False
        if stamp != self.last_stamp:
            self.last_feedback = time.monotonic()
        new_stamp = stamp != self.last_stamp
        self.last_stamp = stamp
        if not self.feedback_valid and new_stamp:
            self.target = np.clip(self.position, self.lower_limits, self.upper_limits)
            self.last_update_sim = self.get_clock().now().nanoseconds
            self.feedback_valid = True
        self.clean_state_pub.publish(out)

    def publish_command(self):
        """Integrate fresh rad/s over bounded simulation time; otherwise hold position."""
        now = time.monotonic()
        sim_ns = self.get_clock().now().nanoseconds
        dt = 0.0 if self.last_update_sim is None else (sim_ns - self.last_update_sim) * 1e-9
        self.last_update_sim = sim_ns
        feedback_fresh = (self.last_feedback is not None
                          and now - self.last_feedback <= self.feedback_timeout_sec)
        if not feedback_fresh:
            # Reinitialize from observed position when feedback recovers, never integrate blind.
            self.feedback_valid = False
        if dt < 0:
            self.stop()
            self.feedback_valid = False
        command_fresh = (self.last_command is not None
                         and now - self.last_command <= self.command_timeout_sec)
        if self.target is None:
            return  # No position command before the first complete measured state.
        if feedback_fresh and self.feedback_valid and command_fresh and dt > 0:
            self.target = np.clip(
                self.target + self.command * min(dt, self.max_step_sec),
                self.lower_limits, self.upper_limits)
        msg = Float64MultiArray()
        msg.data = self.target.tolist()
        self.command_pub.publish(msg)


def main(args=None):
    """Run until shutdown, leaving a bounded stationary position target."""
    run_node(SimRobotBridge, args=args, stop=lambda node: node.stop())


if __name__ == '__main__':
    main()
