# HighTorque ROS2 Control 硬件接口包

本包提供 HighTorque 机械臂的 ROS2 Control 硬件接口。当前实现为 **Panthera HT**。
使用 vendored `motor_cpp`（`hightorque_motor`），只负责真机电机通信与 `SystemInterface`。

仓库布局与 [arx-ros2-control](https://github.com/fiveages-sim/arx-ros2-control) 一致：ROS 包在仓库根目录，`external/` 下 vendored SDK。

机器人模型、夹爪与双臂安装几何等由描述包
[`panthera_ht_description`](https://github.com/fiveages-sim/robot-descriptions-ht)
提供；本包只负责硬件接口。

## 插件

| Plugin | 用途 |
|--------|------|
| `ht_ros2_control/PantheraHardwareInterface` | Panthera HT 单臂 / 双臂 |

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
- `libserialport-dev`
- `libyaml-cpp-dev`

## 编译步骤

`external/motor_cpp` 由 CMake `add_subdirectory` 自动编译，无需像 ARX 那样单独预编译 external SDK。

```bash
cd ~/ros2_ws
colcon build --packages-select ht_ros2_control --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

电机总线 YAML 位于 `external/motor_cpp/robot_param/`（安装后在
`share/ht_ros2_control/external/motor_cpp/robot_param/`）：

- 单臂：`Panthera.yaml`（串口前缀默认 `/dev/ttyACM`）
- 双臂：`PantheraDual.yaml`

描述包 `xacro/ros2_control/robot.xacro` 按 `type` 选择 `config_file` / `dual_config_file`，
并在 `hardware:=real` 时加载本插件。

## 真机串口（启动前必查）

电机通信默认走 `/dev/ttyACM*`（见 `Panthera.yaml` / `PantheraDual.yaml` 的 `Serial_Type`）：

```bash
# 确认设备存在
ls /dev/ttyACM*

# 赋予读写权限（插拔后可能需重做）
sudo chmod a+rw /dev/ttyACM*
```

无输出时先查 USB 连接与供电；也可将用户加入 `dialout` 组后重新登录：`sudo usermod -aG dialout $USER`。

#### 多套机械臂同机（usb_select 控制盒选择）

一个 Livelybot 控制盒是一个多路 USB 复合设备（7 对 CDC-ACM 接口 → 7 个 ttyACM，
VID/PID 相同），双臂只用其中 2 路。多套机械臂同插时无法靠 VID/PID/序列号区分，
由硬件参数 `usb_select` 在驱动层选择：

- 默认 `auto`：检测到 **0 个**控制盒报错退出；**1 个**正常连接；**多个**报错退出并列出各盒路径。
- 指定路径（如 `usb_select:=usb-0:1.2`）：只保留该控制盒的端口。
  路径用 `udevadm info -n /dev/ttyACMx` 查询，支持 `1-1.2` / `usb-0:1.2` / 完整 `ID_PATH`，子串匹配。
- 端口顺序已由 `list_serial_ports` 改为**名称排序**（原 `reverse(readdir)` 顺序随重启变化），
  `serial_id 1,2` 恒对应控制盒通道 1/2（接线口）。
- 串口数量不足时（`serial_id` 超出可用端口）报错退出，避免越界访问。

启动时经描述包 xacro 透传（两个入口都支持 `xacro_usb_select:=` 前缀参数）：

```bash
# GC 连盒 A、OCS2 连盒 B
ros2 launch ht_gravity_compensation gravity_compensation.launch.py hardware:=real \
  xacro_usb_select:=usb-0:1.2 ...
# quick_start.sh 真机启动：选"控制盒"菜单，或手动追加 xacro_usb_select:=usb-0:4.2
```

同一套控制盒的 7 路里只有接线的 2 路会被使用，其余空闲通道不受影响。

## 硬件参数

由描述包 ros2_control xacro 传入本 HI（常用）：

| 参数 | 说明 |
|------|------|
| `control_mode:=mit` | 默认；位置 + 速度 + 力矩 + kp/kd（kp/kd 允许为 0，等效纯力矩前馈；旧名 `full_control` 仍兼容） |
| `control_mode:=effort` | 纯力矩：直接下发控制器 effort 命令（SDK torque 接口） |
| `control_mode:=position` | 纯位置：只下发位置命令（SDK position 接口） |
| `config_file:=...` | 覆盖单臂电机 YAML（默认本包 share 下 `Panthera.yaml`） |
| `dual_config_file:=...` | 双臂电机 YAML（默认 `PantheraDual.yaml`） |
| `usb_select:=auto` | 控制盒选择：`auto`=仅允许 1 个控制盒；或 USB 路径（多套同机时指定，见下文） |
| `max_torques:=...` | 每臂 7 值 CSV（6 臂关节 + 夹爪），力矩限幅；dual 自动拼接 |
| `max_velocities:=...` | 每臂 7 值 CSV，速度上限（mit 模式夹爪速度上限） |
| `joint_kp` / `joint_kd` | 每臂 6 值 CSV，臂关节增益；**同时暴露为 ROS 参数，rqt 可调** |
| `gripper_kp` / `gripper_kd` | 夹爪增益标量；**ROS 参数，rqt 可调** |
| `gripper_rad_to_m:=0.025` | 夹爪电机 rad ↔ 关节 m 换算 |

Ctrl+C 关机：默认插值回 `shutdown_home` 后 `set_stop`（阻尼）。
回零开关在描述包 xacro 硬件参数 `shutdown_return_home`（当前默认 `true`）。

### 速度前馈与 kp/kd 管理

- **速度前馈 = 控制器 velocity 命令透传**：控制器写了 velocity（如 OCS2 MIX / MoveIt）
  即作为期望速度下发；未写 / 未 claim 即为 0。不再做位置差分推导（无死区/滤波/限幅参数）。
- **kp/kd 不再作为命令接口导出**：由 URDF hardware 参数提供初始值，并在 `on_init`
  覆盖为节点参数（`joint_kp`/`joint_kd`/`gripper_kp`/`gripper_kd`），IO 线程每 ~200ms
  同步一次 —— 运行中用 `ros2 param set` / rqt 调节即时生效（如拖动模式低刚度）。
- `perform_command_mode_switch` 会对停止使用的 velocity/effort 接口清零，
  防止控制器切换后旧速度残留。
- 注意：kp/kd 不再导出后，OCS2 自动降级 POSITION 模式（不写 kp/kd 与 HOLD 重力补偿
  effort）；需要重力补偿时用 `ht_gravity_compensation` 包，或用 rqt 调大 `joint_kp`。

## 关节布局

单臂：`joint1`…`joint6` + 夹爪 `gripper_joint`（7 关节 ↔ 7 电机，一一对应）。

双臂顺序：左臂 + 右臂（14 关节 → 14 电机），关节带 `left_` / `right_` 前缀。

> URDF 中仍有 `<mimic>` 的 `gripper_joint2`（纯运动学，用于 TF），
> 但它不在 ros2_control 硬件接口里（不再导出状态/指令）。
