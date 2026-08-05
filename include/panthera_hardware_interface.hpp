#ifndef HT_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_
#define HT_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hightorque_robot
{
class robot;
}

namespace ht_ros2_control
{

/// Panthera HT 机械臂硬件接口（单臂 / 双臂）。
///
/// 每臂 7 个关节（6 臂关节 + 夹爪）对应 7 个电机，关节与电机一一对应。
/// 所有阻塞式 SDK 串口调用都在后台 IO 线程执行；read()/write() 仅做共享内存拷贝，
/// 避免 controller manager 的控制循环被串口 I/O 阻塞。
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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 每臂常量：6 个臂关节 + 1 个夹爪 = 7 关节 = 7 电机
  static constexpr size_t kArmJointCount = 6;
  static constexpr size_t kMotorsPerArm = 7;
  // 无新命令时，后台 IO 线程兜底轮询反馈的间隔
  static constexpr std::chrono::milliseconds kPollPeriod{10};

  bool resolveArmLayout();
  bool validateStorageLayout(const char * context) const;
  bool isPrimaryGripperJoint(size_t joint_index) const;
  // 单位换算：夹爪电机(rad) 与 关节(m) 互转；其余关节 1:1
  double jointPositionToMotorPosition(size_t joint_index, double joint_position) const;
  double motorPositionToJointPosition(size_t joint_index, double motor_position) const;
  double jointVelocityToMotorVelocity(size_t joint_index, double joint_velocity) const;
  double motorVelocityToJointVelocity(size_t joint_index, double motor_velocity) const;
  bool validateMotorCount(const char * context) const;
  bool isValidJointFeedback(double position) const;
  bool allArmMotorsHaveValidFeedback() const;
  bool waitForValidMotorFeedback(const char * context, int timeout_ms);
  void sendActivateHoldCommand();
  void moveToShutdownHomeThenStop();

  // ---- 后台 IO 线程（独占 robot_，仅活跃期间运行） ----
  void startIoThread();
  void stopIoThread();
  void ioThreadLoop();
  // 阻塞式查询电机状态并刷新 latest_*（生命周期线程 / IO 线程调用，二者不同时）
  void pollRobotState();
  // 消费命令快照：死区 / 速度前馈 / 滤波 → 填电机指令 → 阻塞发送
  void sendMotorCommands();

  std::unique_ptr<hightorque_robot::robot> robot_;

  size_t arm_count_{1};
  size_t expected_motors_{kMotorsPerArm};

  std::string config_file_;
  std::string control_mode_;
  bool full_control_{false};

  // ---- 状态 / 指令缓冲（controller manager 通过导出的接口指针直接读写） ----
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_commands_kp_;
  std::vector<double> hw_commands_kd_;

  std::vector<double> max_torques_;
  std::vector<double> max_velocities_;
  std::vector<double> kp_gains_;
  std::vector<double> kd_gains_;

  // ---- 共享缓冲（io_mutex_ 保护）：RT 线程与 IO 线程交换数据 ----
  std::mutex io_mutex_;
  std::condition_variable io_cv_;
  std::thread io_thread_;
  bool io_exit_{false};          // 请求 IO 线程退出
  bool command_pending_{false};  // 有待发送的新命令
  std::vector<double> cmd_positions_;
  std::vector<double> cmd_velocities_;
  std::vector<double> cmd_efforts_;
  std::vector<double> cmd_kp_;
  std::vector<double> cmd_kd_;
  std::vector<double> latest_positions_;
  std::vector<double> latest_velocities_;
  std::vector<double> latest_efforts_;

  // ---- 参数 ----
  double arm_velocity_filter_alpha_{0.25};
  double arm_command_velocity_limit_scale_{0.25};
  double arm_command_position_deadband_{1e-5};
  double gripper_rad_to_m_{0.025};
  double gripper_command_epsilon_{1e-4};

  // ---- IO 线程内部状态（仅 IO 线程访问，无需加锁） ----
  std::vector<double> last_motor_command_positions_;
  std::vector<double> filtered_motor_command_velocities_;
  std::vector<double> last_gripper_command_m_;
  bool has_last_motor_command_positions_{false};
  // 上次实际发送时刻，用于按真实指令节拍推导速度前馈
  std::chrono::steady_clock::time_point last_io_send_time_{};

  bool shutdown_return_home_{true};
  std::vector<double> shutdown_home_positions_;
  double shutdown_home_timeout_sec_{3.5};
  double shutdown_home_tolerance_{0.05};
  double shutdown_home_velocity_{0.3};
};

}  // namespace ht_ros2_control

#endif  // HT_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_
