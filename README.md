# Panthera HT ROS2 Control 硬件接口包

本包提供 HighTorque Panthera HT 机械臂的 ROS2 Control 硬件接口实现。
使用 vendored `motor_cpp`（`hightorque_motor`），不包含 `robot_cpp` 规划/动力学层（控制由 OCS2 负责）。

仓库布局与 [arx-ros2-control](https://github.com/fiveages-sim/arx-ros2-control) 一致：ROS 包在仓库根目录，`external/` 下 vendored SDK。

## 插件

| Plugin | 用途 |
|--------|------|
| `panthera_ros2_control/PantheraHardwareInterface` | 单臂 / 双臂 |

一个 `SystemInterface` + 一个 `hightorque_robot::robot`。电机数量由 YAML 决定：
- 单臂 / left / right：**7** 电机（`Panthera.yaml`）
- 双臂：**14** 电机（`PantheraDual.yaml`）

关节布局推导臂数（`N×8` 关节 → `N` 臂 × 7 电机）。
通过 `pos_vel_MAXtqe` / `pos_vel_tqe_kp_kd` 下发电机指令。

## 依赖项

### ROS2 依赖
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`

### 系统依赖（vendored `motor_cpp`）
- `libserialport`
- `yaml-cpp`

> LCM 为可选调试功能（`lcm_enable()`），本 HI 默认 **不启用**，无需安装系统 `lcm`。

## 编译步骤

`external/motor_cpp` 由 CMake `add_subdirectory` 自动编译，无需像 ARX 那样单独预编译 external SDK。

```bash
cd ~/opne-deploy-ws-ht
colcon build --packages-select panthera_ros2_control --symlink-install
source install/setup.bash
```

电机总线 YAML 位于 `external/motor_cpp/robot_param/`：
- 单臂：`Panthera.yaml`
- 双臂：`PantheraDual.yaml`

## OCS2 真机（单臂）

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=panthera_ht hardware:=real
```

可选 xacro 参数：
- `control_mode:=full_control`（默认）/ `pd_control` / `position_velocity`
- `config_file:=...`（默认本包 share 下 `Panthera.yaml`）
- Ctrl+C 关机：插值回 `shutdown_home` 后 `set_stop`（阻尼）；`shutdown_return_home:=false` 可关闭

## OCS2 真机（双臂）

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=panthera_ht type:=dual hardware:=real
```

默认使用 `PantheraDual.yaml`（xacro `dual_config_file`）。

## 关节布局

`joint1`…`joint6`、主夹爪 `gripper_joint`（或旧名 `L_finger_joint`）、
可选 mimic `gripper_joint2`（或 `R_finger_joint`，仅状态）。
双臂顺序：左臂 + 右臂（16 关节 → 14 电机）。
