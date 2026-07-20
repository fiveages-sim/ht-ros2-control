# Panthera HT ROS2 Control

ROS2 Control hardware interface for HighTorque Panthera HT.
Uses vendored `motor_cpp` (`hightorque_motor`) only — no `robot_cpp` / `panthera::Panthera`
planning or dynamics layer (OCS2 owns control).

## Plugin

| Plugin | Use |
|--------|-----|
| `panthera_ros2_control/PantheraHardwareInterface` | Single and dual arm |

One `SystemInterface` + one `hightorque_robot::robot`. Motor count comes from YAML:
- single / left / right: **7** motors (`Panthera.yaml`)
- dual: **14** motors (`PantheraDual.yaml`)

Joint layout drives arm count (`N*8` joints → `N` arms × 7 motors).
Talks to hardware through per-motor commands (`pos_vel_MAXtqe` / `pos_vel_tqe_kp_kd`).

## Build

```bash
cd ~/open-deploy-ws-ht
colcon build --packages-select panthera_ros2_control --symlink-install
source install/setup.bash
```

SDK is built via `add_subdirectory(external/motor_cpp)`.
YAML configs live under `external/motor_cpp/robot_param`:
- single: `Panthera.yaml`
- dual: `PantheraDual.yaml`

## Dependencies

ROS:
- `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`

System (for vendored `motor_cpp`):
- `libserialport`, `yaml-cpp`

Optional (not used by this HI): vendor LCM via `HIGHTORQUE_MOTOR_ENABLE_LCM=ON`
(needs glib; only for `lcm_enable()` / `motor_msg` UDP debug publish).

## OCS2 real robot (single arm)

```bash
source ~/open-deploy-ws-ht/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=panthera_ht hardware:=real
```

Optional args (passed through xacro):

- `control_mode:=full_control` (default, OCS2 MIX: pos/vel/effort/kp/kd) / `pd_control` / `position_velocity`
- `config_file:=...` (default: `Panthera.yaml` under this package share)
- Ctrl+C shutdown: hardware moves to `shutdown_home` (default all zeros), then `set_stop` (damping). Disable with `shutdown_return_home:=false` in xacro.

## Dual arm real

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=panthera_ht type:=dual hardware:=real
```

Uses `PantheraDual.yaml` by default (`dual_config_file` xacro arg).

## Joint layout

Expects arm joints `joint1`…`joint6`, primary gripper `gripper_joint` (or legacy `L_finger_joint`),
and optional mimic `gripper_joint2` (or legacy `R_finger_joint`, state only).
Dual stacks left then right (16 joints → 14 motors).
