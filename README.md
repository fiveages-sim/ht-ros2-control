# Panthera HT ROS2 Control 硬件接口包

本包提供 HighTorque Panthera HT 机械臂的 ROS2 Control 硬件接口实现。
使用 vendored `motor_cpp`（`hightorque_motor`），不包含 `robot_cpp` 规划/动力学层（控制由 OCS2 负责）。

仓库布局与 [arx-ros2-control](https://github.com/fiveages-sim/arx-ros2-control) 一致：ROS 包在仓库根目录，`external/` 下 vendored SDK。

机器人模型、夹爪与双臂安装间距等由描述包
[`panthera_ht_description`](https://github.com/fiveages-sim/robot-descriptions-ht)
（工作空间子模块 `robot-descriptions-ht`）提供；本包只负责真机硬件接口。

## 插件

| Plugin | 用途 |
|--------|------|
| `panthera_ros2_control/PantheraHardwareInterface` | 单臂 / 双臂 |

一个 `SystemInterface` + 一个 `hightorque_robot::robot`。电机数量由 YAML 决定：

- 单臂 / left / right：**7** 电机（`Panthera.yaml`）
- 双臂：**14** 电机（`PantheraDual.yaml`）

关节布局推导臂数（`N×8` 关节 → `N` 臂 × 7 电机）。
通过 `pos_vel_MAXtqe` / `pos_vel_tqe_kp_kd` 下发电机指令。

启动时会等待各臂电机反馈有效（过滤 SDK 占位值 `999` 等）；反馈不全时拒绝 `on_activate`，避免使能后 runaway。

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

在部署工作空间（规范目录 `~/ht-deploy-ws`）中：

```bash
cd ~/ht-deploy-ws
# 推荐：./quick_start.sh → 1) 编译 → 2) 编译真机所需包
colcon build --packages-select panthera_ros2_control panthera_ht_description --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

电机总线 YAML 位于 `external/motor_cpp/robot_param/`：

- 单臂：`Panthera.yaml`（串口前缀默认 `/dev/ttyACM`）
- 双臂：`PantheraDual.yaml`

由描述包 `xacro/ros2_control/robot.xacro` 按 `type` 选择 `config_file` / `dual_config_file`。

## 真机串口（启动前必查）

电机通信默认走 `/dev/ttyACM*`（见 `Panthera.yaml` / `PantheraDual.yaml` 的 `Serial_Type`）：

```bash
# 确认设备存在
ls /dev/ttyACM*

# 赋予读写权限（插拔后可能需重做）
sudo chmod a+rw /dev/ttyACM*
```

无输出时先查 USB 连接与供电；也可将用户加入 `dialout` 组后重新登录：`sudo usermod -aG dialout $USER`。

## OCS2 真机（单臂）

单臂请显式传 `type:=single`（与描述包 xacro 默认一致）。若不传 `type`，
`robot_common_launch` / OCS2 规划侧可能按双臂处理，与 7 电机配置不匹配。

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=panthera_ht type:=single hardware:=real
```

可选参数（经描述包 ros2_control xacro 传入本 HI）：

| Launch / xacro | 说明 |
|----------------|------|
| `xacro_control_mode:=full_control` | 默认；OCS2 MIX（位置+速度+力矩+kp/kd） |
| `xacro_control_mode:=pd_control` | 位置+力矩，kp/kd |
| `xacro_control_mode:=position_velocity` | 位置+速度+最大力矩 |
| `xacro_config_file:=...` | 覆盖单臂电机 YAML（默认本包 share 下 `Panthera.yaml`） |

Ctrl+C 关机：默认插值回 `shutdown_home` 后 `set_stop`（阻尼）。
回零开关在描述包 `xacro/ros2_control/robot.xacro` 的硬件参数
`shutdown_return_home`（当前默认 `true`）。

也可用工作空间 `./quick_start.sh` → **启动** → **单臂** → **真机**（会传 `type:=single` 并可选控制模式）。

## OCS2 真机（双臂）

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=panthera_ht type:=dual hardware:=real
```

默认使用 `PantheraDual.yaml`（xacro `dual_config_file`）。

双臂左右安装间距等几何参数在描述包 `xacro/robot.xacro` 的
`left_mount_xyz` / `right_mount_xyz`（可用 `xacro_left_mount_xyz` 等在 launch 覆盖）。
OCS2 规划 URDF 启动时由同一份 xacro 生成，改间距后重启即可。

## 关节布局

单臂：`joint1`…`joint6`、主夹爪 `gripper_joint`（或旧名 `L_finger_joint`）、
可选 mimic `gripper_joint2`（或 `R_finger_joint`，仅状态）。

双臂顺序：左臂 + 右臂（16 关节 → 14 电机），关节带 `left_` / `right_` 前缀。
