# PRD: xela_taxel_joint_state_publisher

## 1. Overview
xela_taxel_joint_state_publisher is a ros2_control controller that publishes a filtered JointState stream for XELA taxel sensors. The controller filters incoming JointState messages or initializes joint positions from configured lists, then publishes a consolidated JointState topic for downstream applications. It is designed to run inside ros2_control and be managed by controller_manager.

## 2. Goals
- Provide a ros2_control controller that publishes taxel JointState for a selected device profile.
- Support multiple device profiles (e.g., allegrohand, 2f_gripper) from a single YAML config.
- Allow configuration of output topic, publish rate, input sources, and joint ordering.
- Operate without impacting the primary MoveIt control JointState stream.
- Be robust to missing inputs by using initial positions where available.

## 3. Non-Goals
- Parsing URDFs or XACROs directly (handled by upstream nodes or config files).
- Dynamically reloading device profiles at runtime (restart/reload expected).
- Publishing non-joint sensor data (force, tactile values, etc.).

## 4. Users and Use Cases
### 4.1 Users
- Robotics developers integrating XELA taxel sensors into MoveIt Pro configurations.
- Operators running robots that require a separate full-model joint state stream.

### 4.2 Use Cases
- Publish taxel joint states for a full robot model on `/joint_states_full` while keeping `/joint_states` lightweight for MoveIt.
- Publish taxel joint states for a 2-finger gripper package on `/joint_states`.
- Combine multiple taxel device profiles into one output stream.

## 5. Functional Requirements
### 5.1 Configuration
- Controller must read a YAML config file specified by `config_yaml`.
- Config supports `device_profiles` with `keep_joints_files` lists per profile.
- Config supports optional global parameters: `output_topic`, `publish_rate`, `preserve_input_order`, `source_list`.
- Controller accepts parameter `device_profiles` to select one or more profiles.

### 5.2 Input JointState Handling
- Controller subscribes to topics listed in `source_list`.
- If `source_list` is empty, the controller publishes using initial positions only.
- If a source topic equals `output_topic`, it is ignored to prevent loops.

### 5.3 Output JointState
- Output topic is `output_topic` (default `/joint_states`).
- Output includes `name` and `position` arrays. `velocity` and `effort` are only included if present in inputs.
- Output ordering:
  - If `ordered_keep_joints` exists in config, it defines output order.
  - Else if `preserve_input_order` is true and input received, use input order filtered to keep joints.
  - Else output order follows merged keep_joints list.

### 5.4 Initial Positions
- Controller supports `initial_positions` per profile in config (optional).
- If no position has been received for a joint, initial position is used.

### 5.5 Package Path Resolution
- `config_yaml` supports `package://<pkg>/path` resolution using ament index.

## 6. Non-Functional Requirements
- ROS 2 Humble compatible.
- Real-time safe behavior: no blocking operations in `update()`.
- Robust to missing inputs and config errors (log and fail gracefully on configure).
- Maintain deterministic output order per configuration.

## 7. Configuration Schema
### 7.1 Example
```yaml
output_topic: /joint_states
publish_rate: 30.0
preserve_input_order: true
source_list:
  - /joint_states_full

device_profiles:
  allegrohand:
    keep_joints_files:
      - list_allegrohand_joint.yaml
  2f_gripper:
    keep_joints_files:
      - list_2f_gripper_joint.yaml
```

### 7.2 keep_joints_files
- Each file is YAML containing `keep_joints:` followed by a list of joint names.

## 8. Interface (Parameters)
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| config_yaml | string | "" | Path to config YAML (supports package://) |
| device_profiles | string[] | [] | Profiles to load from config |
| output_topic | string | /joint_states | Output JointState topic |
| source_list | string[] | [] | Input JointState topics to merge |
| publish_rate | double | 30.0 | Output publish frequency in Hz |
| preserve_input_order | bool | true | Preserve input order when possible |

## 9. Integration Requirements
- Controller must be listed under `controller_manager` in the target ros2_control YAML.
- Controller parameters must be defined under a top-level node matching controller name.
- Add controller to `controllers_active_at_startup` in MoveIt Pro config if required.

## 10. Failure Modes and Logging
- If `config_yaml` is missing or invalid, controller configuration fails.
- If `device_profiles` is empty or missing in config, configuration fails.
- If `keep_joints` resolves to empty, configuration fails.
- All failures must log clear error messages.

## 11. Testing Plan
- Unit-style validation via:
  - Launch controller with valid config and verify output topic publishes expected joints.
  - Launch controller without `source_list` and validate initial positions publish.
  - Launch controller with invalid config and ensure configure fails gracefully.
- Integration validation with MoveIt Pro configuration:
  - Confirm controller is listed by `ros2 control list_controllers`.
  - Confirm output topic is populated with expected taxel joints.

## 12. Open Questions
- Should `source_list` allow wildcard or namespace expansion?
- Should publish be suppressed until at least one input message is received?
- Do we need runtime profile switching without controller restart?
