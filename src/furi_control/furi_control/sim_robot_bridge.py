from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SimRobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("sim_robot_bridge")

        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3", "joint_4"])
        self.declare_parameter("command_input_topic", "/furi/command/applied")
        self.declare_parameter("controller_command_topic", "/position_controller/commands")
        self.declare_parameter("joint_state_input_topic", "/joint_states")
        self.declare_parameter("clean_joint_state_topic", "/furi/joint_states/clean")

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_count = len(self.joint_names)

        self.last_missing_log_time = 0.0

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("controller_command_topic").value),
            10,
        )
        self.clean_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("clean_joint_state_topic").value),
            10,
        )

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("command_input_topic").value),
            self.command_callback,
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_input_topic").value),
            self.joint_state_callback,
            10,
        )

    def command_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != self.joint_count:
            self.get_logger().warning(
                f"Expected {self.joint_count} joint commands, got {len(msg.data)}"
            )
            return

        out = Float64MultiArray()
        out.data = list(msg.data)
        self.command_pub.publish(out)

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        missing = [name for name in self.joint_names if name not in name_to_index]
        if missing:
            now = time.monotonic()
            if now - self.last_missing_log_time > 5.0:
                self.get_logger().warning(f"Missing simulated joints: {missing}")
                self.last_missing_log_time = now
            return

        indices = [name_to_index[name] for name in self.joint_names]

        out = JointState()
        out.header = msg.header
        out.name = list(self.joint_names)

        if msg.position:
            out.position = [msg.position[i] for i in indices]
        if msg.velocity:
            out.velocity = [msg.velocity[i] for i in indices]
        if msg.effort:
            out.effort = [msg.effort[i] for i in indices]

        self.clean_state_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimRobotBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()