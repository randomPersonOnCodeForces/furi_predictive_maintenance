# Stage 4 validation report

Validated October 18, 2026 on Ubuntu 22.04 / WSL2, Python 3.10, ROS 2 Humble, Gazebo Classic 11.10.2, `gazebo_ros2_control` 0.4.10, controller manager 2.54.0 and forward command controller 2.53.1. All runtime validation was headless. No hardware was connected.

## Diagnosis and corrections

1. **Reproduced controller-manager failure:** the original robot spawned, the plugin connected to `robot_state_publisher` and loaded the controller YAML, then logged `Couldn't parse parameter override rule: '--param robot_description:=...'`. The excerpt ends in XML comments. Humble's plugin puts the complete URDF into a CLI parameter argument, calls `rcl_parse_arguments` and returns on parsing failure before constructing/spinning its controller manager. Removing XML comments from the generated parameter string fixes this exact failure. The existing explicit robot parameter/node configuration was retained. Evidence: `/tmp/furi-stage4/baseline-open.log`; regression: `test_stage4_model.py` initializes an actual ROS context with the normalized XML CLI parameter. [Upstream parsing path](https://github.com/ros-controls/gazebo_ros2_control/blob/humble/gazebo_ros2_control/src/gazebo_ros2_control_plugin.cpp).
2. **Startup ordering:** replaced startup sleeps with successful spawn → broadcaster activation → position-controller activation → FURI nodes. Bounded native spawner service/switch timeouts plus an overall wall-time deadline prevent unbounded waits, including Gazebo's spawn client, which otherwise has an unbounded response loop. Required process exits stop the launch. An intentionally impossible deadline now produces a nonzero launch exit.
3. **Command semantics:** the original bridge forwarded rad/s directly to a rad position interface. Native velocity control was tried first; initial motion checks failed. The finalized position adapter preserves FURI velocity contracts, initializes from feedback, bounds simulation-time integration, enforces limits and holds on invalid/stale inputs. It does not claim validated torque/actuator dynamics.
4. **Clock:** the default 10 Hz Gazebo ROS clock caused a 0.02-second-bounded adapter to discard most of every 0.1-second jump. Configuring `/clock` at 100 Hz resolved the observed factor-of-five tracking loss.
5. **Physics/geometry:** the base is now fixed to world only in the simulation variant. The original positive shoulder/elbow axes lowered the arm through the ground at approximately 0.22 rad on both pitch joints; recorded feedback became unstable while commanded positions stayed bounded. Positive simulation pitch now lifts the arm. Masses/inertias remain approximate; no physical calibration was invented. Stage 3 Xacro still expands without Gazebo tags.
6. **Shutdown/validation:** added a shared cooperative node lifecycle: signal handlers set a flag, the executor exits between callbacks, and cleanup publishes a hold/zero command while the context is alive. This avoids intermittent Humble pybind conversion errors when SIGINT raises during message handling. Added finite/dimension checks, safe malformed-command handling and live fault-parameter validation. Corrected the pre-existing `furi_interfaces` manifest group/dependency ordering exposed by schema validation and fixed EOF/formatting lint findings.

The initial sandbox attempt separately failed on blocked sockets and a read-only Gazebo runtime directory. Runtime results below came from executions with normal local networking and runtime-directory access. These permission failures are distinct from the reproduced URDF parser failure. Service discovery by itself was never counted as controller readiness.

## Executed checks

| Test | Result | Evidence |
|---|---|---|
| Clean build | PASS | All five packages; `/tmp/furi-stage4/build-final.log` |
| Xacro/URDF validity | PASS | Both visualization and simulation variants; `check_urdf`, four joints, axes, limits, configuration consistency, positive-definite inertias, collision geometry, control interfaces and actual ROS CLI parameter parsing |
| Gazebo robot spawn | PASS | Four successful consecutive launches reuse the same test domain and port; `/get_model_list` verifies exactly one Dobot entity |
| Controller activation | PASS | Actual `ListControllers` response: broadcaster and position controller active; hardware service verifies all four position interfaces available and claimed |
| Joint movement | PASS | All four joints move positively under bounded known commands; 301 feedback samples per scenario; finite positions and limit tolerance checks |
| End-to-end pipeline | PASS | Constant, sine and attenuation scenarios; measured position displacement agrees with the applied-velocity integral within 0.045 rad; observed errors below 0.008 rad in the recorded four-scenario run |
| Fault injection | PASS | Exact disabled baseline, enabled command attenuation, seeded noisy feedback with measured 0.002-rad standard deviation tolerance; malformed commands and invalid live parameters rejected safely |
| Position limits | PASS | High bounded velocity inputs drive all joints to their configured upper limits; numerical deviations below 0.0001 rad in recorded run; lower-limit clamping and inward recovery also exercised in callback tests |
| Repeated startup/cleanup | PASS | Four clean launches with one model, one manager, no hanging spawners, duplicate state publishers, crashes, forced termination or surviving child PIDs |
| Startup timeout | PASS | Intentionally impossible 0.1-second deadline produces nonzero exit within the bounded test |
| Lightweight mock launch | PASS | Real ROS launch transports commands and produces changing finite four-joint feedback without a controller manager |
| Default Gazebo trajectory | PASS | 12 simulated seconds / 1201 samples; position ranges 0.6365, 0.3184, 0.1603, 0.1293 rad; clean shutdown |
| Requested ROS CLI inspections | PASS | Node/service/topic lists, controller and hardware-interface lists, `/joint_states` and clean feedback samples; `/tmp/furi-stage4/default/cli.log` |
| Package checks | PASS | `colcon test-result`: **46 tests, 0 errors, 0 failures, 3 skipped**; skips are existing copyright-template checks |

The four-scenario observed feedback interval was 0.01 simulation seconds (100 Hz). This is a measured simulation-time rate, not a guaranteed wall-time rate or hardware sampling rate. The configured command/controller/clock rates are 100 Hz. The test suite allows a small motion-tracking error because clocks, callbacks and DDS queues are asynchronous; it uses multiple feedback samples and real service responses.

Recorded four-scenario measurements before the final model-count/cleanup assertions were added:

| Input | Four-joint displacement (rad) | Largest integration error (rad) |
|---|---|---|
| Constant baseline | 0.228800, 0.228722, 0.228894, 0.228808 | 0.007278 |
| Sine baseline | 0.167738, 0.167661, 0.167828, 0.167743 | 0.001243 |
| Attenuation and noise | 0.149520, 0.149443, 0.149610, 0.149524 | 0.000090 |
| Upper-limit saturation | 2.356190, 1.396321, 1.483540, 2.530729 | 0.000097 |

Full suite results and logs are in `/tmp/furi-stage4/colcon-results-final.log`, `/tmp/furi-stage4/colcon-test-final.log`. Pytest prints per-launch logs on failure and retains feedback/command arrays as `samples.npz` in its temporary directory. Temporary artifact paths describe this execution only; reproduction does not depend on those paths.

## Reproduce

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch furi_bringup gazebo_pipeline.launch.py gui:=false
```

Stop the manual launch before running the independent integration suite:

```bash
FURI_RUN_GAZEBO_TESTS=1 colcon test
colcon test-result --verbose
```

This execution used a separate clean build/install/log tree in `/tmp/furi-stage4` because the task's writable workspace was `src/`. No generated build/install/log artifacts were added to Git.

### XML schema access on restricted hosts

The existing package XML processing instruction asks `xmllint` for an HTTP schema. On this host that endpoint was inaccessible, and its HTTPS certificate also failed validation. Tests were run with a temporary XML catalog pointing to the official ROS schema from its source repository. Schema validation was retained; no lint checks were suppressed.

```bash
validation_dir=$(mktemp -d)
curl -fL https://raw.githubusercontent.com/ros-infrastructure/rep/master/xsd/package_format3.xsd \
  -o "$validation_dir/package_format3.xsd"
curl -fL https://raw.githubusercontent.com/ros-infrastructure/rep/master/xsd/package_common.xsd \
  -o "$validation_dir/package_common.xsd"
cat > "$validation_dir/catalog.xml" <<'XML'
<?xml version="1.0"?>
<catalog xmlns="urn:oasis:names:tc:entity:xmlns:xml:catalog">
  <uri name="http://download.ros.org/schema/package_format3.xsd" uri="package_format3.xsd"/>
</catalog>
XML
XML_CATALOG_FILES="$validation_dir/catalog.xml" FURI_RUN_GAZEBO_TESTS=1 colcon test
colcon test-result --verbose
```

## Files changed

Paths are relative to the repository root.

| File | Purpose |
|---|---|
| `README.md` | Dependencies, build/launch commands, architecture, parameters, faults, troubleshooting and completion checklist |
| `docs/stage4_validation.md` | Diagnosis, measured validation, limitations and reproduction |
| `src/furi_bringup/launch/gazebo_pipeline.launch.py` | Normalize generated XML, readiness-based startup, native spawner deadlines, startup failure handling, headless/config options |
| `src/furi_bringup/config/gazebo_pipeline.yaml` | Adapter topics, position/velocity limits, watchdogs, bounded integration and 100 Hz Gazebo clock |
| `src/furi_bringup/config/gazebo_controllers.yaml` | Retains the intended position-controller configuration; EOF normalized |
| `src/furi_control/furi_control/sim_robot_bridge.py` | Initialized velocity-to-position adapter, validation, feedback ordering, timeouts, limits, reset and shutdown handling |
| `src/furi_control/furi_control/trajectory_generator.py` | ROS-clock trajectory phase, finite/parameter validation and reliable shutdown |
| `src/furi_control/furi_control/fault_injector.py` | Safe malformed-command behavior, complete parameter/feedback validation, baseline and noise isolation |
| `src/furi_control/furi_control/fault_models.py` | Prevent command attenuation from reversing requested velocity |
| `src/furi_control/furi_control/runtime.py` | Cooperative SIGINT/SIGTERM lifecycle shared by all four control nodes |
| `src/furi_control/furi_control/mock_robot.py` | Preserve mock semantics while validating settings and fixing malformed input/shutdown behavior |
| `src/furi_description/urdf/dobot_magician_lite.urdf.xacro` | Simulation-only world mount/plugin/control, explicit manager configuration, corrected pitch axes and approximation notes |
| `src/furi_tests/test/test_stage4_model.py` | Model, configuration and parser-regression checks |
| `src/furi_tests/test/test_stage4_nodes.py` | Callback, watchdog, initialization, limits, reset, fault and malformed-input checks |
| `src/furi_tests/test/test_stage4_gazebo.py` | Four repeated headless integration scenarios, actual entity/service checks, deadline, cleanup and mock launch regression |
| `src/furi_bringup/package.xml` | Explicit controller/plugin runtime dependencies |
| `src/furi_control/package.xml` | NumPy and parameter-interface dependencies |
| `src/furi_tests/package.xml` | Model/control/integration test dependencies |
| `src/furi_interfaces/package.xml` | Correct dependency/group order required by the ROS package schema |
| `src/furi_bringup/launch/software_pipeline.launch.py` | EOF newline only, for existing lint checks |
| `src/furi_bringup/launch/demo_pipeline.launch.py` | EOF newline only, for existing lint checks |
| `src/furi_bringup/setup.py` | EOF newline only, for existing lint checks |
| `src/furi_description/launch/view_model.launch.py` | EOF newline only, for existing lint checks |

## Remaining limitations

Stage 4 software-only acceptance is met in the tested headless environment. Graphical Gazebo/RViz display operation was not exercised; the Stage 3 model was expanded and parsed. The model is a simplified serial chain, and changing the pitch axes establishes a simulation convention rather than a verified physical Dobot mapping. Position commands use the plugin's idealized kinematic actuation, not calibrated servo dynamics. Noise is injected into a separate feedback topic; friction is command attenuation, not simulated physical actuator wear or friction torque. Hardware validation, Stage 5, Raspberry Pi deployment and machine-learning models remain unimplemented.
