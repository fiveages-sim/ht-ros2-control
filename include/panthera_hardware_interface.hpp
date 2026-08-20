#ifndef HT_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_
#define HT_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hightorque_robot
{
class robot;
}

namespace ht_ros2_control
{

namespace detail
{

/// 每臂关节布局：6 臂关节 + 1 夹爪 = 7 电机，关节与电机一一对应。
struct MotorLayout
{
  static constexpr size_t kMotorsPerArm = 7;
  static constexpr size_t kArmJointCount = 6;

  size_t arm_count = 1;

  size_t motor_count() const { return arm_count * kMotorsPerArm; }
  /// 每臂最后一个关节（offset 6）是夹爪
  bool isGripper(size_t index) const { return index % kMotorsPerArm == (kMotorsPerArm - 1); }
};

/// 硬件配置：全部从 URDF hardware 参数集中解析；
/// kp/kd 同时暴露为节点 ROS 参数（rqt 可调），由 IO 线程周期同步。
struct HardwareConfig
{
  std::string config_file;
  /// 控制盒 USB 选择："auto"=自动检测（只允许一个控制盒）；或指定 USB 路径
  /// （如 "1-1.2" / "usb-0:1.2"，子串匹配，从 udevadm info 复制的 ID_PATH/KERNELS 均可）
  std::string usb_select = "auto";
  std::string control_mode = "position_velocity";
  bool full_control = false;

  double gripper_rad_to_m = 0.025;

  bool shutdown_return_home = true;
  std::vector<double> shutdown_home_positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double shutdown_home_timeout_sec = 3.5;
  double shutdown_home_tolerance = 0.05;
  double shutdown_home_velocity = 0.3;

  // 每臂 7 值 CSV（含夹爪）；双臂 = 14 值
  std::vector<double> max_torques;
  std::vector<double> max_velocities;
  // 每臂 6 值 CSV（仅臂关节）；双臂 = 12 值
  std::vector<double> joint_kp;
  std::vector<double> joint_kd;
  double gripper_kp = 5.0;
  double gripper_kd = 0.1;

  /// 解析 URDF hardware 参数（on_init 调用），返回 false 表示必需参数缺失/非法
  bool load(const hardware_interface::HardwareInfo & info, size_t arm_count);
  /// 把 kp/kd 暴露为节点参数（rqt 可调）；初始值来自 URDF 解析结果
  void declareRosParameters(rclcpp::Node & node) const;
  /// IO 线程周期调用：读回 rqt 修改的 kp/kd 并重新展开
  void refreshRosParameters(rclcpp::Node & node);
  /// 把限位 / 增益展开成每电机数组（motor_count 长度）
  void expand(const MotorLayout & layout);

  // 展开后的每电机数组
  std::vector<double> motor_max_torques;
  std::vector<double> motor_max_velocities;
  std::vector<double> motor_kp;
  std::vector<double> motor_kd;
};

/// 反馈有效性过滤与等待
namespace feedback
{
/// SDK 占位 999 / NaN / 超范围（±10 rad 或 ±10 m）视为无效反馈
bool isValid(double position);
/// 只检查每臂 6 个臂关节；夹爪初始反馈允许为 0
bool allArmMotorsValid(const std::vector<double> & latest, const MotorLayout & layout);
/// 轮询直到全部臂关节反馈有效或超时；poll 由调用方提供（生命周期线程执行）
bool waitForValid(
  const std::string & logger_name, const std::function<void()> & poll,
  const std::vector<double> & latest, const MotorLayout & layout,
  const char * context, int timeout_ms);
}  // namespace feedback

/// SDK 电机数与布局一致性校验（on_configure / on_activate 各一次）
bool validateMotorCount(
  const std::string & logger_name, const hightorque_robot::robot & robot,
  const MotorLayout & layout);

/// 关机回 home：平滑插值到 shutdown_home 后进入阻尼
void moveToShutdownHome(
  hightorque_robot::robot & robot, const MotorLayout & layout, const HardwareConfig & config,
  const rclcpp::Logger & logger);

/// 后台 IO 线程：独占 robot_，承载全部阻塞式 SDK 串口调用。
/// read()/write() 仅通过共享缓冲与其交换数据，控制循环不被串口 I/O 阻塞。
class IoLoop
{
public:
  IoLoop(
    hightorque_robot::robot & robot, const MotorLayout & layout, HardwareConfig & config,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::Node::SharedPtr & node);
  ~IoLoop();
  IoLoop(const IoLoop &) = delete;
  IoLoop & operator=(const IoLoop &) = delete;

  void start();
  void stop();
  bool running() const;

  // ---- 生命周期线程专用（IO 线程未启动时调用，无并发） ----
  /// 阻塞查询电机状态并刷新 latest_*
  void pollOnce();
  /// 以当前实测位姿作为保持指令阻塞下发（激活瞬间防失控）
  void sendActivateHold();
  /// 以关节指令播种发送状态（last_motor_command_positions_ / 夹爪跟踪）
  void seedCommands(const std::vector<double> & joint_positions);
  /// 读最新反馈（无锁；仅 IO 线程未启动时安全）
  const std::vector<double> & latestPositions() const { return latest_positions_; }

  // ---- RT 线程（write()） ----
  /// 把控制器命令快照交给 IO 线程（非阻塞）
  void publishCommands(
    const std::vector<double> & positions, const std::vector<double> & velocities,
    const std::vector<double> & efforts);
  // ---- RT 线程（read()） ----
  /// 拷贝最新反馈（非阻塞）
  void getLatest(
    std::vector<double> & positions, std::vector<double> & velocities,
    std::vector<double> & efforts);

private:
  void threadLoop();
  void sendMotorCommands();
  void refreshGains();
  double toMotorPosition(size_t index, double joint_value) const;
  double toJointPosition(size_t index, double motor_value) const;
  double toMotorVelocity(size_t index, double joint_value) const;

  hightorque_robot::robot & robot_;
  const MotorLayout & layout_;
  HardwareConfig & config_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr node_;

  // ---- 共享缓冲（mutex_ 保护）：RT 线程与 IO 线程交换数据 ----
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  bool exit_ = false;
  bool pending_ = false;
  std::vector<double> cmd_positions_;
  std::vector<double> cmd_velocities_;
  std::vector<double> cmd_efforts_;
  std::vector<double> latest_positions_;
  std::vector<double> latest_velocities_;
  std::vector<double> latest_efforts_;

  // ---- IO 线程私有（仅 IO 线程访问，无需加锁） ----
  std::vector<double> last_motor_command_positions_;
  bool has_last_motor_command_positions_ = false;
  int gain_refresh_counter_ = 0;
};

}  // namespace detail

/// Panthera HT 机械臂硬件接口（单臂 / 双臂）。
///
/// 每臂 7 个关节（6 臂关节 + 夹爪）对应 7 个电机，关节与电机一一对应。
/// 所有阻塞式 SDK 串口调用都在后台 IO 线程执行；read()/write() 仅做共享内存拷贝。
///
/// 速度前馈 = 控制器 velocity 命令透传（未写即 0）；kp/kd 不再作为命令接口导出，
/// 由 hardware 参数 + ROS 参数（rqt 可调）管理。
class PantheraHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantheraHardwareInterface)

  ~PantheraHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool resolveArmLayout();

  detail::MotorLayout layout_;
  detail::HardwareConfig config_;
  std::unique_ptr<hightorque_robot::robot> robot_;
  std::unique_ptr<detail::IoLoop> io_loop_;

  // ---- RT 缓冲（export_* 直接指向；索引与关节一一对应） ----
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
};

}  // namespace ht_ros2_control

#endif  // HT_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_
