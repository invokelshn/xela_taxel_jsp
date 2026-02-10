# PRD: xela_taxel_joint_state_publisher

## 1. Product Overview

`xela_taxel_joint_state_publisher` is a ROS 2 `ros2_control` controller plugin that publishes
a filtered taxel-focused `sensor_msgs/msg/JointState` stream from configured device profiles.

The controller is intended to provide a stable and profile-driven joint state stream for
visualization, integration, and downstream processing without changing upstream sensor drivers.

## 2. Goals

- Provide profile-based filtering of `JointState` for XELA taxel systems.
- Support multiple hardware profiles with reusable YAML lists.
- Publish deterministic output order for selected joints.
- Allow left/right hand prefix remapping without duplicating profile files.
- Work as a standard `controller_manager` plugin in ROS 2 Humble.

## 3. Non-Goals

- Hardware device access or CAN communication.
- URDF/XACRO parsing and automatic profile generation.
- Runtime hot-reload of profile files without controller reconfiguration.
- Publishing non-`JointState` tactile values.

## 4. Primary Users

- ROS 2 integrators deploying XELA tactile setups.
- MoveIt/visualization pipeline maintainers requiring a separate joint stream.

## 5. Functional Requirements

### 5.1 Configuration Loading

- The controller must accept `config_yaml` and resolve `package://` paths.
- Config must contain `device_profiles` map.
- Selected profiles must be provided by `device_profiles` (or `device_profile` override).
- Joint definitions must be loaded from:
  - `keep_joints_files`
  - optional inline `keep_joints`
  - optional `ordered_keep_joints`
  - optional `initial_positions`

### 5.2 Profile Selection

- `device_profile` (string) overrides `device_profiles` (string array) when non-empty.
- Multiple profiles may be merged using `device_profiles`.
- Duplicate joint names must be removed while preserving insertion order.

### 5.3 Input Handling

- Subscribe to each topic in `source_list`.
- Ignore topics equal to `output_topic` to prevent self-loop.
- Maintain latest value cache per joint (`position`, `velocity`, `effort`).
- Track latest input joint order for optional order preservation.

### 5.4 Output Publishing

- Publish `JointState` on `output_topic`.
- Publish at `publish_rate` (Hz).
- Always publish `name` and `position`.
- Publish `velocity` only when at least one source provides velocity.
- Publish `effort` only when at least one source provides effort.

### 5.5 Output Order Rules

Order priority must be:

1. `ordered_keep_joints` intersection with keep set
2. latest input order filtered by keep set (if `preserve_input_order=true`)
3. merged keep list order

### 5.6 Initial Position Fallback

- If no position has been received for a selected joint:
  - use `initial_positions[joint]` when provided
  - otherwise use `0.0`

### 5.7 Hand Side Mapping

- `hand_side=right|r`: convert `x_taxel_0_` prefix to `x_taxel_1_`
- `hand_side=left|l`: convert `x_taxel_1_` prefix to `x_taxel_0_`
- Mapping must apply to `keep_joints` and `ordered_keep_joints`.

## 6. Non-Functional Requirements

- Compatible with ROS 2 Humble and `controller_interface`.
- No blocking operations in `update()`.
- Thread-safe shared state access between callback and update loops.
- Clear error logs and configuration failure on invalid setup.

## 7. Interfaces

### 7.1 Plugin Interface

- Plugin name:
  `xela_taxel_joint_state_publisher/XelaTaxelJointStatePublisher`
- Base class:
  `controller_interface::ControllerInterface`

### 7.2 Parameters

| Parameter | Type | Default | Required | Notes |
| --- | --- | --- | --- | --- |
| `config_yaml` | string | `""` | Yes | Main YAML path, supports `package://` |
| `device_profiles` | string[] | `[]` | Yes* | Profiles to load |
| `device_profile` | string | `""` | No | Single profile override |
| `hand_side` | string | `left` | No | Prefix mapping helper |
| `output_topic` | string | `/joint_states` | No | Can be overridden by config when unchanged |
| `source_list` | string[] | `[]` | No | Input topics |
| `publish_rate` | double | `30.0` | No | Hz |
| `preserve_input_order` | bool | `true` | No | Output ordering policy |

\* At least one effective profile is required after applying `device_profile` override.

## 8. Failure Conditions

- `config_yaml` missing or unreadable.
- `device_profiles` missing in config file.
- No effective selected profile.
- Selected profiles resolve to empty `keep_joints`.

All failures should abort controller configure step and log explicit reason.

## 9. Acceptance Criteria

- Controller configures and activates with valid config/profile.
- Output topic publishes only selected joints.
- Ordering behavior matches rules in section 5.5.
- Hand-side mapping changes prefix as expected.
- Invalid config fails during configure with clear log.

## 10. Risks and Constraints

- Multi-source inputs are merged by latest callback write per joint; there is no timestamp arbitration.
- If profile files are changed on disk, controller restart/reconfigure is required.
