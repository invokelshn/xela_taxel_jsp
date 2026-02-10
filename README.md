# xela_taxel_joint_state_publisher

ROS 2 `ros2_control` controller plugin that publishes filtered taxel `sensor_msgs/msg/JointState`
from one or more input `JointState` topics.

## What it does

- Loads one or more device profiles from a YAML config.
- Builds a `keep_joints` set from profile joint files/lists.
- Subscribes to configured input topics (`source_list`) and keeps latest values per joint.
- Publishes filtered output at `publish_rate` to `output_topic`.
- Supports optional right/left hand prefix remapping with `hand_side`.

## Plugin Info

- Package: `xela_taxel_joint_state_publisher`
- Plugin class: `xela_taxel_joint_state_publisher/XelaTaxelJointStatePublisher`
- Base class: `controller_interface::ControllerInterface`

## Parameters

| Name | Type | Default | Required | Description |
| --- | --- | --- | --- | --- |
| `config_yaml` | string | `""` | Yes | Main config file path. Supports `package://<pkg>/<path>` |
| `device_profiles` | string[] | `[]` | Yes* | Profile names to load from `device_profiles` map |
| `device_profile` | string | `""` | No | If set, overrides `device_profiles` with a single profile |
| `hand_side` | string | `left` | No | `left`/`l` or `right`/`r`; remaps `x_taxel_0_` and `x_taxel_1_` prefixes |
| `output_topic` | string | `/joint_states` | No | Output topic |
| `source_list` | string[] | `[]` | No | Input `JointState` topics |
| `publish_rate` | double | `30.0` | No | Output publish rate (Hz) |
| `preserve_input_order` | bool | `true` | No | If true, prefer latest input order for output names |

\* One of `device_profiles` or `device_profile` must result in at least one valid profile.

## Config File Schema

Main config (`config_yaml`) example:

```yaml
output_topic: /joint_states
publish_rate: 30.0
preserve_input_order: true
source_list: []

device_profiles:
  uSCuAH_left:
    keep_joints_files:
      - uscuahl_joint.yaml
  uSCuAH_right:
    keep_joints_files:
      - uscuahr_joint.yaml
  uSCuAH:
    keep_joints_files:
      - uscuahl_joint.yaml
  uSPr2F:
    keep_joints_files:
      - uspr2f_joint.yaml
  uSPrHE35:
    keep_joints_files:
      - usprhe35_joint.yaml
  uSPrDS:
    keep_joints_files:
      - usprds_joint.yaml
```

Profile entries support:

- `keep_joints_files`: list of YAML files containing `keep_joints: [...]`
- `keep_joints`: inline joint name list
- `ordered_keep_joints`: explicit output order
- `initial_positions`: map of joint name to default position

## Runtime Behavior

- Output order priority:
  1. `ordered_keep_joints` (if provided)
  2. latest input order filtered by keep set (`preserve_input_order=true`)
  3. merged `keep_joints` order
- If a joint has no incoming position, `initial_positions` is used when available; else `0.0`.
- `velocity`/`effort` fields are published only if any source message provided them.
- A source topic equal to `output_topic` is ignored to prevent loopback.

## Example Controller Manager Config

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    xela_taxel_jsp:
      type: xela_taxel_joint_state_publisher/XelaTaxelJointStatePublisher

xela_taxel_jsp:
  ros__parameters:
    config_yaml: package://xela_taxel_joint_state_publisher/config/xela_taxel_jsp_config.yaml
    device_profile: uSPr2F
    hand_side: left
    output_topic: /joint_states_full
    source_list:
      - /joint_states
    publish_rate: 30.0
    preserve_input_order: true
```

## Build

From workspace root (`~/xela_robotics/99_support/00_ext_xmodels_ws`):

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/xela_base_extention --packages-select xela_taxel_joint_state_publisher --symlink-install
source install/setup.bash
```

## Validation Checklist

- `ros2 control list_controllers` shows controller as `active`.
- `ros2 topic echo <output_topic>` includes expected taxel joints.
- No config errors in controller logs (`config_yaml`, profile names, keep joint files).

## PRD

Product requirements document: `PRD.md`
