from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MockRobot(Node):
    def __init__(self) -> None:
        super().__init__("mock_robot")

        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4"],
        )
        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter("command_timeout_sec", 0.2)
        self.declare_parameter("velocity_limits", [1.0, 1.0, 1.0, 1.5])

        self.joint_names = list(
            self.get_parameter("joint_names").value
        )
        self.n = len(self.joint_names)

        self.position = np.zeros(self.n, dtype=np.float64)
        self.velocity = np.zeros(self.n, dtype=np.float64)
        self.command = np.zeros(self.n, dtype=np.float64)

        self.velocity_limits = np.asarray(
            self.get_parameter("velocity_limits").value,
            dtype=np.float64,
        )

        self.timeout_ns = int(
            float(self.get_parameter("command_timeout_sec").value) * 1e9
        )
        self.last_command_ns: int | None = None
        self.last_update_ns = time.monotonic_ns()

        self.publisher = self.create_publisher(
            JointState,
            "/furi/joint_states/clean",
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/furi/command/applied",
            self.command_callback,
            10,
        )

        rate = float(self.get_parameter("update_rate_hz").value)
        self.create_timer(1.0 / rate, self.update)

    def command_callback(self, msg: Float64MultiArray) -> None:
        command = np.asarray(msg.data, dtype=np.float64)

        if command.shape != (self.n,) or not np.all(np.isfinite(command)):
            self.get_logger().warning("Rejected command")
            return

        self.command[:] = np.clip(
            command,
            -self.velocity_limits,
            self.velocity_limits,
        )
        self.last_command_ns = time.monotonic_ns()

    def update(self) -> None:
        now_ns = time.monotonic_ns()
        dt = min((now_ns - self.last_update_ns) * 1e-9, 0.05)
        self.last_update_ns = now_ns

        stale = (
            self.last_command_ns is None
            or now_ns - self.last_command_ns > self.timeout_ns
        )

        if stale:
            self.velocity.fill(0.0)
        else:
            self.velocity[:] = self.command

        self.position += self.velocity * dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.position.tolist()
        msg.velocity = self.velocity.tolist()
        self.publisher.publish(msg)

def main() -> None:
    rclpy.init()
    node = MockRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()