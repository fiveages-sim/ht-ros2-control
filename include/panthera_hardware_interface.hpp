#ifndef PANTHERA_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_
#define PANTHERA_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
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

namespace panthera_ros2_control
{

/// Single SystemInterface for one or two Panthera HT arms.
/// One hightorque_robot::robot; motor count is 7 per arm (YAML-driven).
class PantheraHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantheraHardwareInterface)

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
  static constexpr size_t kArmJointCount = 6;
  static constexpr size_t kMotorsPerArm = 7;
  static constexpr size_t kJointsPerArm = 8;

  bool resolveArmLayout();
  bool validateJointLayout() const;
  bool validateStorageLayout(const char * context) const;
  bool isMimicGripperJoint(size_t joint_index) const;
  bool isPrimaryGripperJoint(size_t joint_index) const;
  size_t motorIndexFromJointIndex(size_t joint_index) const;
  size_t jointIndexFromMotorIndex(size_t motor_index) const;
  double jointPositionToMotorPosition(size_t joint_index, double joint_position) const;
  double motorPositionToJointPosition(size_t joint_index, double motor_position) const;
  double jointVelocityToMotorVelocity(size_t joint_index, double joint_velocity) const;
  double motorVelocityToJointVelocity(size_t joint_index, double motor_velocity) const;
  bool validateMotorCount(const char * context) const;
  void sendActivateHoldCommand();
  void moveToShutdownHomeThenStop();

  std::unique_ptr<hightorque_robot::robot> robot_;

  size_t arm_count_{1};
  size_t expected_motors_{kMotorsPerArm};
  /// When true, each arm has a full 8-joint layout (6 + gripper + mimic).
  bool full_arm_layout_{true};

  std::string config_file_;
  std::string control_mode_;

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

  bool command_debug_enabled_{false};
  double command_debug_period_{1.0};
  std::chrono::steady_clock::time_point last_command_debug_time_;

  double arm_velocity_filter_alpha_{0.25};
  double arm_command_velocity_limit_scale_{0.25};
  double arm_command_position_deadband_{1e-5};
  double gripper_rad_to_m_{0.025};
  double gripper_command_epsilon_{1e-4};

  std::vector<double> last_motor_command_positions_;
  std::vector<double> filtered_motor_command_velocities_;
  std::vector<double> last_gripper_command_m_;
  bool has_last_motor_command_positions_{false};

  bool use_velocity_commands_{false};
  bool use_effort_commands_{false};
  bool use_gain_commands_{false};

  bool shutdown_return_home_{true};
  std::vector<double> shutdown_home_positions_;
  double shutdown_home_timeout_sec_{3.5};
  double shutdown_home_tolerance_{0.05};
  double shutdown_home_velocity_{0.3};
};

}  // namespace panthera_ros2_control

#endif  // PANTHERA_ROS2_CONTROL__PANTHERA_HARDWARE_INTERFACE_HPP_
