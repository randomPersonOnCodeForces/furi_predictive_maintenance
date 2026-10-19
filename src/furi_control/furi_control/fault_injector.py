"""Apply command attenuation and sensor noise without changing clean feedback."""

import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from furi_control.fault_models import FrictionConfig, add_gaussian_noise, apply_friction_proxy


from furi_control.runtime import run_node


class FaultInjector(Node):
    """Keep the Stage 1 command proxy separate from Gazebo contact dynamics."""

    def __init__(self):
        super().__init__('fault_injector')
        defaults = {
            'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            'faults.enabled': False, 'friction.enabled': False,
            'friction.viscous': [0.0] * 4, 'friction.coulomb': [0.0] * 4,
            'friction.smoothing_velocity': 0.02, 'noise.enabled': False,
            'noise.position_std': [0.0] * 4, 'noise.velocity_std': [0.0] * 4,
            'random_seed': 42,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.n = len(self.joint_names)
        self.validate({name: self.get_parameter(name).value for name in defaults})
        self.add_on_set_parameters_callback(self.validate_update)
        self.rng = np.random.default_rng(int(self.get_parameter('random_seed').value))
        self.command_pub = self.create_publisher(Float64MultiArray, '/furi/command/applied', 1)
        self.noisy_state_pub = self.create_publisher(JointState, '/furi/joint_states/noisy', 10)
        self.create_subscription(Float64MultiArray, '/furi/command/raw', self.command_callback, 1)
        self.create_subscription(
            JointState, '/furi/joint_states/clean', self.joint_state_callback, 10)

    def validate(self, values):
        """Validate the whole proposed parameter set before accepting any update."""
        if not self.n or len(set(values['joint_names'])) != self.n:
            raise ValueError('joint_names must be nonempty and unique')
        for name in ['friction.viscous', 'friction.coulomb', 'noise.position_std',
                     'noise.velocity_std']:
            array = np.asarray(values[name], dtype=float)
            if array.shape != (self.n,) or not np.all(np.isfinite(array)) or np.any(array < 0):
                raise ValueError(f'{name} requires {self.n} finite nonnegative values')
        smoothing = float(values['friction.smoothing_velocity'])
        if not np.isfinite(smoothing) or smoothing <= 0:
            raise ValueError('friction.smoothing_velocity must be finite and positive')
        if int(values['random_seed']) < 0:
            raise ValueError('random_seed must be nonnegative')

    def validate_update(self, parameters):
        """Reject unsafe dynamic updates and changes requiring node reconstruction."""
        values = {name: parameter.value for name, parameter in self._parameters.items()}
        for parameter in parameters:
            if parameter.name in ('joint_names', 'random_seed'):
                return SetParametersResult(
                    successful=False, reason='Restart to change joint names/seed')
            values[parameter.name] = parameter.value
        try:
            self.validate(values)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def get_vector_parameter(self, name):
        """Return a validated vector in configured joint order."""
        return np.asarray(self.get_parameter(name).value, dtype=float)

    def command_callback(self, msg):
        """Forward rad/s unchanged at baseline, or apply the friction proxy."""
        command = np.asarray(msg.data, dtype=float)
        if command.shape != (self.n,) or not np.all(np.isfinite(command)):
            self.get_logger().warning('Malformed command; publishing zero velocities')
            applied = np.zeros(self.n)
        elif (self.get_parameter('faults.enabled').value
              and self.get_parameter('friction.enabled').value):
            applied = apply_friction_proxy(command, FrictionConfig(
                viscous=self.get_vector_parameter('friction.viscous'),
                coulomb=self.get_vector_parameter('friction.coulomb'),
                smoothing_velocity=float(self.get_parameter('friction.smoothing_velocity').value),
            ))
        else:
            applied = command
        out = Float64MultiArray()
        out.data = applied.tolist()
        self.command_pub.publish(out)

    def joint_state_callback(self, msg):
        """Order and validate feedback; add noise only to the noisy stream."""
        if (len(msg.name) != self.n or set(msg.name) != set(self.joint_names)
                or len(msg.position) != self.n):
            self.get_logger().warning('Rejected malformed clean feedback')
            return
        for values in [msg.position, msg.velocity, msg.effort]:
            if len(values) not in (0, self.n) or not np.all(np.isfinite(values)):
                self.get_logger().warning('Rejected malformed clean feedback')
                return
        indices = [msg.name.index(name) for name in self.joint_names]
        out = JointState()
        out.header = msg.header
        out.name = self.joint_names
        out.position = [msg.position[i] for i in indices]
        out.velocity = [msg.velocity[i] for i in indices] if msg.velocity else []
        out.effort = [msg.effort[i] for i in indices] if msg.effort else []
        if (self.get_parameter('faults.enabled').value
                and self.get_parameter('noise.enabled').value):
            out.position = add_gaussian_noise(
                np.asarray(out.position), self.get_vector_parameter('noise.position_std'),
                self.rng).tolist()
            if out.velocity:
                out.velocity = add_gaussian_noise(
                    np.asarray(out.velocity), self.get_vector_parameter('noise.velocity_std'),
                    self.rng).tolist()
        self.noisy_state_pub.publish(out)


def main(args=None):
    """Run the node with cooperative shutdown between callbacks."""
    run_node(FaultInjector, args=args)


if __name__ == '__main__':
    main()
