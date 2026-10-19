# FURI predictive maintenance — Dobot Magician Lite simulation

ROS 2 Humble / Gazebo Classic 11 software validation for a four-joint research arm. Stage 4 implements spawning, controller activation, joint motion and the existing command/fault/feedback pipeline. No hardware driver or Stage 5 deployment is included.

The URDF is an approximate serial-chain simulation model, **not a calibrated Dobot digital twin**. Geometry, masses, inertias, effort limits and servo behavior are approximations. The physical Dobot linkage and hardware angle conventions are not reproduced.

## Dependencies and build

Use Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic 11:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-urdf \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-numpy python3-yaml
source /opt/ros/humble/setup.bash
# Run from this repository/workspace root:
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch furi_bringup gazebo_pipeline.launch.py
# Headless equivalent:
ros2 launch furi_bringup gazebo_pipeline.launch.py gui:=false
```

One Gazebo-owned `/controller_manager` loads `joint_state_broadcaster` and `position_controller` (`forward_command_controller/ForwardCommandController`, position interface). Successful spawn starts the broadcaster spawner; successful broadcaster activation starts the position spawner; successful position activation starts the FURI nodes. Spawners wait on real services with bounded calls/switches. The overall startup deadline also catches a discovered but unresponsive manager or a stuck spawn request. Failure raises a launch error. No standalone `ros2_control_node` is started.

Arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_sim_time` | `true` | ROS simulation clock for trajectory and position integration |
| `enable_trajectory` | `true` | Publish generated velocities; false publishes zeros and holds position |
| `trajectory_type` | `sine` | `sine`, `constant`, or `square` |
| `update_rate_hz` | `100.0` | Generator and bridge timer rate; controller and `/clock` are configured at 100 Hz |
| `faults_enabled` | `false` | Master fault switch |
| `friction_enabled` | `false` | Enable the command attenuation proxy when master switch is on |
| `noise_enabled` | `false` | Enable sensor noise when master switch is on |
| `gui` | `true` | Set false for headless execution |
| `startup_timeout` | `60.0` | Overall startup wall-time deadline in seconds |
| `verbose` | `false` | Gazebo verbose logs |
| `pipeline_config` | installed `software_pipeline.yaml` | Override trajectory/fault parameters with a ROS parameter YAML file |

The default `constant` trajectory has zero offset, so it holds still. Supply a config with a nonzero `offset` for constant motion. Keep `use_sim_time:=true` for simulation-time trajectory tracking. Increasing only `update_rate_hz` does not increase the controller or Gazebo clock rate; edit their YAML settings together when changing rates.

## Topic contracts and units

All vectors use `[joint_1, joint_2, joint_3, joint_4]` order.

```text
trajectory_generator
  -> /furi/command/raw                 Float64MultiArray, joint velocities [rad/s]
fault_injector
  -> /furi/command/applied             Float64MultiArray, joint velocities [rad/s]
sim_robot_bridge (velocity-to-position adapter)
  -> /position_controller/commands     Float64MultiArray, joint positions [rad]
Gazebo + gazebo_ros2_control + joint_state_broadcaster
  -> /joint_states                    sensor_msgs/JointState
sim_robot_bridge
  -> /furi/joint_states/clean          sensor_msgs/JointState
fault_injector
  -> /furi/joint_states/noisy          sensor_msgs/JointState
```

`JointState.position` is radians, `velocity` rad/s, and `effort` Nm when present; this model's effort output is not calibrated actuator torque. There is one joint-state broadcaster and one clean-feedback publisher. The bridge accepts sensor-data QoS on `/joint_states` and validates/reorders joint arrays. FURI command and feedback publishers use reliable QoS, compatible with the current consumers.

The adapter waits for complete finite observed joint positions, initializes its target from those positions, then integrates velocity using ROS time and a maximum step of 0.02 s. It clips velocities to `[1, 1, 1, 1.5]` rad/s and targets to the URDF position limits. It holds the last target after 0.2 s without valid commands or advancing feedback; malformed commands immediately stop integration. Feedback recovery reinitializes from observation, and clock resets invalidate the old command. A steady-clock watchdog continues running while Gazebo is paused. Shutdown leaves a bounded stationary target. Long clock jumps are deliberately not replayed.

Native velocity control was investigated first and failed initial gravity/motion validation. The explicit adapter gives bounded position targets and deterministic stationary hold without inventing PID gains. Position control in this Gazebo plugin is idealized kinematic actuation, so this setup validates software integration rather than real actuator dynamics.

## Fault injection

```bash
ros2 launch furi_bringup gazebo_pipeline.launch.py gui:=false \
  faults_enabled:=true friction_enabled:=true noise_enabled:=true
ros2 param set /fault_injector friction.viscous '[0.25, 0.25, 0.25, 0.25]'
ros2 param set /fault_injector friction.coulomb '[0.01, 0.01, 0.01, 0.01]'
```

Default friction coefficients are zero. Set nonzero coefficients to produce command attenuation. The implemented proxy is `v - b*v - c*tanh(v/s)`, clipped so it cannot reverse the requested direction or increase its magnitude. `b` is dimensionless, `c` and `s` use rad/s. This is a software degradation proxy; it does **not** change Gazebo friction/contact coefficients, motor torque or temperature. Physics-based friction degradation is unsupported.

Noise uses seeded independent Gaussian perturbations on the noisy position/velocity stream only. Default standard deviations are 0.002 rad and 0.005 rad/s. Clean feedback is unaffected. Disabling the master fault switch produces exact command passthrough and unchanged sensor values. Invalid coefficients, smoothing values and noise standard deviations are rejected, including live updates. Joint names, bridge settings and cached generator vectors/rate require a node restart; enable, trajectory type and duration can be changed live.

## Verify and test

In another sourced terminal while Gazebo runs:

```bash
ros2 node list
ros2 service list | grep controller_manager
timeout 10s ros2 control list_controllers
timeout 10s ros2 control list_hardware_interfaces
ros2 topic list
timeout 5s ros2 topic echo /joint_states --once
timeout 5s ros2 topic echo /furi/joint_states/clean --once
```

Expect both controllers `active`, four claimed position command interfaces, and finite positions that change under the enabled sine trajectory. Service discovery alone does not establish manager readiness.

```bash
# Static/model/callback tests and the existing package checks:
colcon test
colcon test-result --verbose
# Include actual Gazebo startup, motion, faults, limits, deadline and mock launch tests:
FURI_RUN_GAZEBO_TESTS=1 colcon test
colcon test-result --verbose
# Run the focused suite with detailed per-launch measurements:
FURI_RUN_GAZEBO_TESTS=1 python3 -m pytest -v -s src/furi_tests/test/test_stage4_*.py
```

Integration tests start their own headless simulations. They reuse ROS domain 81 and Gazebo port 11481 for four consecutive successful launches and check child-process cleanup. Set `FURI_TEST_DOMAIN_BASE` to change that successful-launch test domain. Deadline and mock tests use domains 89 and 88. Keep those domains/ports free and do not run this suite concurrently with itself. Gazebo tests are explicitly skipped unless opted in. Existing copyright-template checks remain skipped because the repository has no declared copyright headers.

See [Stage 4 validation](docs/stage4_validation.md) for measured results, evidence and the XML-schema catalog workaround for network-restricted test hosts.

## Other stages and troubleshooting

```bash
# Stages 1–2: lightweight velocity-integrating mock; no Gazebo or controllers:
ros2 launch furi_bringup software_pipeline.launch.py enable_trajectory:=true
# Stage 3: visualization model; Gazebo plugin/control tags omitted by default:
ros2 launch furi_description view_model.launch.py
```

The Gazebo variant fixes the base to `world`; positive shoulder/elbow pitch raises this simplified arm above the tabletop. The visualization variant retains the same four joints and geometry, with the corrected pitch axes. Future hardware integration needs explicit linkage/sign mapping, measured dynamics and a hardware-specific controller/watchdog; none is provided here.

If the robot spawns but controllers do not respond, inspect the Gazebo log before increasing timeouts. The reproduced failure was `Couldn't parse parameter override rule: robot_description:=...` caused by XML comments being reparsed as YAML by `gazebo_ros2_control` 0.4.10. The launch now strips comments from the generated parameter string. Retain this normalization when editing robot-description generation. The [Humble plugin source](https://github.com/ros-controls/gazebo_ros2_control/blob/humble/gazebo_ros2_control/src/gazebo_ros2_control_plugin.cpp) shows this parameter parsing path.

Check that `/clock` advances and is configured to publish at 100 Hz, the simulation is unpaused, only one simulation is using the domain/port, and all terminals source the same install. A 100 Hz controller against 1 kHz Gazebo physics emits a slower-control-period warning; that comparison is expected. Socket errors or read-only `~/.gazebo` errors are host permission failures and require normal local ROS networking/runtime-directory access. Graphical-client operation additionally requires a working display and was not validated in the headless test environment.

## Stage 4 checklist

- [x] Headless Gazebo launch, exactly one robot and one controller manager.
- [x] Responsive manager services and both controllers active.
- [x] Four-joint motion, finite feedback and position limits.
- [x] Trajectory → fault stage → adapter → Gazebo → clean feedback.
- [x] Baseline passthrough, noise isolation and command degradation proxy.
- [x] Repeated startup, startup deadline and cleanup checks.
- [x] Lightweight mock launch and Stage 3 model validation.
- [x] Automated package checks and reproducible documentation.

Acceptance covers the tested software-only/headless environment. GUI operation, calibrated dynamics, physical friction degradation and hardware validation remain outside the verified scope.
