import rclpy
from rclpy.node import Node 

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import numpy as np 

from furi_control.fault_models import (
    FrictionConfig,
    apply_friction_proxy,
    add_gaussian_noise,
)

class FaultInjector(Node):
    def get_vector_parameter(self, name):
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)

        if values.shape != (self.n,):
            raise ValueError(f"{name} must have {self.n} values")

        return values

    def __init__(self):
        super().__init__("fault_injector")

        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3", "joint_4"])
        self.declare_parameter("faults.enabled", False)

        self.declare_parameter("friction.enabled", False)
        self.declare_parameter("friction.viscous", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("friction.coulomb", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("friction.smoothing_velocity", 0.02)

        self.declare_parameter("noise.enabled", False)
        self.declare_parameter("noise.position_std", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("noise.velocity_std", [0.0, 0.0, 0.0, 0.0])

        self.declare_parameter("random_seed", 42)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.n = len(self.joint_names)

        seed = int(self.get_parameter("random_seed").value)
        self.rng = np.random.default_rng(seed)

        self.command_pub = self.create_publisher(Float64MultiArray, "/furi/command/applied", 10)
        self.noisy_state_pub = self.create_publisher(JointState, "/furi/joint_states/noisy", 10)
        
        self.command_sub = self.create_subscription(Float64MultiArray, "/furi/command/raw", self.command_callback, 10)
        self.clean_state_sub = self.create_subscription(JointState, "/furi/joint_states/clean", self.joint_state_callback, 10)

    def command_callback(self, msg: Float64MultiArray):
        command = np.asarray(msg.data, dtype=np.float64)

        if command.shape != (self.n,):
            self.get_logger().warning("Wrong sized command")
            return

        if not np.all(np.isfinite(command)):
            self.get_logger().warning("Command contains NaN or infinity")
            return

        faults_enabled = bool(self.get_parameter("faults.enabled").value)
        friction_enabled = bool(self.get_parameter("friction.enabled").value)

        applied = command

        if faults_enabled and friction_enabled:
            viscous = self.get_vector_parameter("friction.viscous")
            coulomb = self.get_vector_parameter("friction.coulomb")
            smoothing_velocity = float(
                self.get_parameter("friction.smoothing_velocity").value
            )

            config = FrictionConfig(
                viscous=viscous,
                coulomb=coulomb,
                smoothing_velocity=smoothing_velocity,
            )

            applied = apply_friction_proxy(command, config)

        out = Float64MultiArray()
        out.data = applied.tolist()
        self.command_pub.publish(out)

    def joint_state_callback(self, msg: JointState):
        faults_enabled = bool(self.get_parameter("faults.enabled").value)
        noise_enabled = bool(self.get_parameter("noise.enabled").value)

        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)

        if not (faults_enabled and noise_enabled):
            self.noisy_state_pub.publish(out)
            return

        if len(msg.position) == self.n:
            position = np.asarray(msg.position, dtype=np.float64)
            position_std = self.get_vector_parameter("noise.position_std")
            out.position = add_gaussian_noise(
                position,
                position_std,
                self.rng,
            ).tolist()

        if len(msg.velocity) == self.n:
            velocity = np.asarray(msg.velocity, dtype=np.float64)
            velocity_std = self.get_vector_parameter("noise.velocity_std")
            out.velocity = add_gaussian_noise(
                velocity,
                velocity_std,
                self.rng,
            ).tolist()

        self.noisy_state_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)

    node = FaultInjector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()