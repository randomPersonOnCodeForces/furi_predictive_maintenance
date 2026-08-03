from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TrajectoryGenerator(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_generator")

        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4"],
        )
        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter("enabled", False)
        self.declare_parameter("trajectory_type", "sine")
        self.declare_parameter("amplitude", [0.2, 0.15, 0.1, 0.1])
        self.declare_parameter("frequency", [0.1, 0.15, 0.2, 0.25])
        self.declare_parameter("phase", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("offset", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("velocity_limits", [1.0, 1.0, 1.0, 1.5])
        self.declare_parameter("duration_sec", 0.0)
        self.declare_parameter("topic", "/furi/command/raw")

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_count = len(self.joint_names)

        if self.joint_count == 0:
            raise ValueError("joint_names cannot be empty")

        self.update_rate_hz = float(
            self.get_parameter("update_rate_hz").value
        )

        if self.update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")

        self.amplitude = self._vector_parameter("amplitude")
        self.frequency = self._vector_parameter("frequency")
        self.phase = self._vector_parameter("phase")
        self.offset = self._vector_parameter("offset")
        self.velocity_limits = self._vector_parameter("velocity_limits")

        if np.any(self.frequency < 0.0):
            raise ValueError("frequency values cannot be negative")

        if np.any(self.velocity_limits <= 0.0):
            raise ValueError("velocity_limits must be positive")

        self.start_ns = time.monotonic_ns()
        self.publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("topic").value),
            1,
        )

        self.timer = self.create_timer(
            1.0 / self.update_rate_hz,
            self.publish_command,
        )

    def _vector_parameter(self, name: str) -> np.ndarray:
        values = np.asarray(
            self.get_parameter(name).value,
            dtype=np.float64,
        ).reshape(-1)

        if values.size == 1:
            values = np.full(
                self.joint_count,
                values.item(),
                dtype=np.float64,
            )

        if values.size != self.joint_count:
            raise ValueError(
                f"{name} must contain one value or "
                f"{self.joint_count} values"
            )

        if not np.all(np.isfinite(values)):
            raise ValueError(f"{name} contains invalid values")

        return values

    def publish_command(self) -> None:
        enabled = bool(self.get_parameter("enabled").value)
        trajectory_type = str(
            self.get_parameter("trajectory_type").value
        )
        duration_sec = float(
            self.get_parameter("duration_sec").value
        )

        elapsed = (time.monotonic_ns() - self.start_ns) * 1e-9

        if not enabled or (
            duration_sec > 0.0 and elapsed >= duration_sec
        ):
            command = np.zeros(
                self.joint_count,
                dtype=np.float64,
            )
        elif trajectory_type == "sine":
            command = (
                self.offset
                + self.amplitude
                * np.sin(
                    2.0 * np.pi * self.frequency * elapsed
                    + self.phase
                )
            )
        elif trajectory_type == "constant":
            command = self.offset.copy()
        elif trajectory_type == "square":
            command = (
                self.offset
                + self.amplitude
                * np.sign(
                    np.sin(
                        2.0
                        * np.pi
                        * self.frequency
                        * elapsed
                        + self.phase
                    )
                )
            )
        else:
            command = np.zeros(
                self.joint_count,
                dtype=np.float64,
            )

        command = np.clip(
            command,
            -self.velocity_limits,
            self.velocity_limits,
        )

        message = Float64MultiArray()
        message.data = command.tolist()
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero_message = Float64MultiArray()
        zero_message.data = [0.0] * node.joint_count
        node.publisher.publish(zero_message)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()