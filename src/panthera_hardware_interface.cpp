#include "panthera_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "hardware/robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace panthera_ros2_control
{

namespace
{
constexpr const char * kLoggerName = "PantheraHardwareInterface";

std::string formatVector(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << "[";
  for (size_t i = 0; i < values.size(); ++i)
  {
    if (i > 0)
    {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

std::vector<double> parseCsvDoubles(const std::string & text, size_t expected_count)
{
  std::vector<double> values;
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ','))
  {
    const auto first = token.find_first_not_of(" \t\n\r");
    if (first == std::string::npos)
    {
      continue;
    }
    const auto last = token.find_last_not_of(" \t\n\r");
    values.push_back(std::stod(token.substr(first, last - first + 1)));
  }
  if (values.size() != expected_count)
  {
    throw std::invalid_argument(
      "expected " + std::to_string(expected_count) + " values, got " +
      std::to_string(values.size()));
  }
  return values;
}

bool endsWith(const std::string & joint_name, std::string_view suffix)
{
  return joint_name.size() >= suffix.size() &&
         joint_name.compare(joint_name.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isMimicGripperJointName(const std::string & joint_name)
{
  return endsWith(joint_name, "R_finger_joint") || endsWith(joint_name, "gripper_joint2");
}

bool mimicUsesNegatedSign(const std::string & joint_name)
{
  return endsWith(joint_name, "R_finger_joint");
}
}  // namespace

bool PantheraHardwareInterface::resolveArmLayout()
{
  const size_t joint_count = info_.joints.size();
  auto logger = rclcpp::get_logger(kLoggerName);

  if (joint_count >= kJointsPerArm && joint_count % kJointsPerArm == 0)
  {
    arm_count_ = joint_count / kJointsPerArm;
    full_arm_layout_ = true;
    expected_motors_ = arm_count_ * kMotorsPerArm;
    return true;
  }

  // Legacy single-arm layouts: 6 arm joints, optional primary gripper, optional mimic.
  if (joint_count >= kArmJointCount && joint_count <= kJointsPerArm)
  {
    arm_count_ = 1;
    full_arm_layout_ = (joint_count == kJointsPerArm);
    expected_motors_ = kMotorsPerArm;
    return true;
  }

  RCLCPP_ERROR(
    logger,
    "Unsupported joint layout: expected 6-8 joints (single) or N*8 joints (N arms), got %zu",
    joint_count);
  return false;
}

bool PantheraHardwareInterface::validateJointLayout() const
{
  auto logger = rclcpp::get_logger(kLoggerName);

  if (arm_count_ == 0 || expected_motors_ == 0)
  {
    RCLCPP_ERROR(logger, "Arm layout was not resolved");
    return false;
  }

  if (full_arm_layout_)
  {
    if (info_.joints.size() != arm_count_ * kJointsPerArm)
    {
      RCLCPP_ERROR(
        logger,
        "Unsupported joint layout: expected %zu joints for %zu arm(s), got %zu",
        arm_count_ * kJointsPerArm, arm_count_, info_.joints.size());
      return false;
    }
    return true;
  }

  // Legacy single: optional gripper / mimic by name.
  if (info_.joints.size() > 6 && isMimicGripperJointName(info_.joints[6].name))
  {
    RCLCPP_ERROR(
      logger,
      "Invalid joint layout: joint index 6 (%s) cannot be a mimic gripper joint",
      info_.joints[6].name.c_str());
    return false;
  }
  if (info_.joints.size() > 7 && !isMimicGripperJointName(info_.joints[7].name))
  {
    RCLCPP_ERROR(
      logger,
      "Invalid joint layout: joint index 7 (%s) must be a mimic gripper joint",
      info_.joints[7].name.c_str());
    return false;
  }
  return true;
}

bool PantheraHardwareInterface::validateStorageLayout(const char * context) const
{
  const auto joint_count = info_.joints.size();
  auto logger = rclcpp::get_logger(kLoggerName);
  auto validate_size = [&](const std::vector<double> & values, const char * label)
  {
    if (values.size() < joint_count)
    {
      RCLCPP_ERROR(
        logger,
        "%s failed: %s size %zu is smaller than expected joint count %zu",
        context, label, values.size(), joint_count);
      return false;
    }
    return true;
  };

  return validate_size(hw_positions_, "hw_positions") &&
         validate_size(hw_velocities_, "hw_velocities") &&
         validate_size(hw_efforts_, "hw_efforts") &&
         validate_size(hw_commands_positions_, "hw_commands_positions") &&
         validate_size(hw_commands_velocities_, "hw_commands_velocities") &&
         validate_size(hw_commands_efforts_, "hw_commands_efforts") &&
         validate_size(hw_commands_kp_, "hw_commands_kp") &&
         validate_size(hw_commands_kd_, "hw_commands_kd") &&
         validate_size(max_torques_, "max_torques") &&
         validate_size(max_velocities_, "max_velocities") &&
         validate_size(kp_gains_, "kp_gains") &&
         validate_size(kd_gains_, "kd_gains");
}

bool PantheraHardwareInterface::isMimicGripperJoint(size_t joint_index) const
{
  if (full_arm_layout_)
  {
    return (joint_index % kJointsPerArm) == 7;
  }
  return joint_index == 7 && info_.joints.size() > 7;
}

bool PantheraHardwareInterface::isPrimaryGripperJoint(size_t joint_index) const
{
  if (full_arm_layout_)
  {
    return (joint_index % kJointsPerArm) == 6;
  }
  return joint_index == 6 && info_.joints.size() > 6;
}

size_t PantheraHardwareInterface::motorIndexFromJointIndex(size_t joint_index) const
{
  if (full_arm_layout_)
  {
    const size_t arm_index = joint_index / kJointsPerArm;
    const size_t offset = joint_index % kJointsPerArm;
    if (offset >= 7)
    {
      return expected_motors_;  // mimic: invalid
    }
    return arm_index * kMotorsPerArm + offset;
  }

  if (joint_index < 7)
  {
    return joint_index;
  }
  return expected_motors_;
}

size_t PantheraHardwareInterface::jointIndexFromMotorIndex(size_t motor_index) const
{
  if (full_arm_layout_)
  {
    const size_t arm_index = motor_index / kMotorsPerArm;
    const size_t offset = motor_index % kMotorsPerArm;
    return arm_index * kJointsPerArm + offset;
  }
  return motor_index;
}

double PantheraHardwareInterface::jointPositionToMotorPosition(
  size_t joint_index, double joint_position) const
{
  if (isPrimaryGripperJoint(joint_index))
  {
    return joint_position / gripper_rad_to_m_;
  }
  return joint_position;
}

double PantheraHardwareInterface::motorPositionToJointPosition(
  size_t joint_index, double motor_position) const
{
  if (isPrimaryGripperJoint(joint_index))
  {
    return motor_position * gripper_rad_to_m_;
  }
  return motor_position;
}

double PantheraHardwareInterface::jointVelocityToMotorVelocity(
  size_t joint_index, double joint_velocity) const
{
  if (isPrimaryGripperJoint(joint_index))
  {
    return joint_velocity / gripper_rad_to_m_;
  }
  return joint_velocity;
}

double PantheraHardwareInterface::motorVelocityToJointVelocity(
  size_t joint_index, double motor_velocity) const
{
  if (isPrimaryGripperJoint(joint_index))
  {
    return motor_velocity * gripper_rad_to_m_;
  }
  return motor_velocity;
}

bool PantheraHardwareInterface::validateMotorCount(const char * context) const
{
  if (!robot_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "%s failed: robot instance is null",
      context);
    return false;
  }
  if (robot_->Motors.size() != expected_motors_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "%s failed: expected %zu motors (%zu arm(s) x %zu), got %zu",
      context, expected_motors_, arm_count_, kMotorsPerArm, robot_->Motors.size());
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto config_it = info_.hardware_parameters.find("config_file");
  if (config_it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Parameter 'config_file' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  config_file_ = config_it->second;

  const auto mode_it = info_.hardware_parameters.find("control_mode");
  control_mode_ = mode_it == info_.hardware_parameters.end() ? "position_velocity" : mode_it->second;

  if (!resolveArmLayout() || !validateJointLayout())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto parse_bool_parameter = [this](const std::string & name, bool default_value)
  {
    const auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end())
    {
      return default_value;
    }

    std::string value = it->second;
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value == "true" || value == "1" || value == "yes" || value == "on";
  };

  command_debug_enabled_ = parse_bool_parameter("command_debug_enabled", false);
  if (const auto it = info_.hardware_parameters.find("command_debug_period");
    it != info_.hardware_parameters.end())
  {
    command_debug_period_ = std::stod(it->second);
  }
  if (!std::isfinite(command_debug_period_) || command_debug_period_ <= 0.0)
  {
    command_debug_period_ = 1.0;
  }
  last_command_debug_time_ = std::chrono::steady_clock::now();

  if (const auto it = info_.hardware_parameters.find("arm_velocity_filter_alpha");
    it != info_.hardware_parameters.end())
  {
    arm_velocity_filter_alpha_ = std::stod(it->second);
  }
  arm_velocity_filter_alpha_ = std::clamp(arm_velocity_filter_alpha_, 0.0, 1.0);

  if (const auto it = info_.hardware_parameters.find("arm_command_velocity_limit_scale");
    it != info_.hardware_parameters.end())
  {
    arm_command_velocity_limit_scale_ = std::stod(it->second);
  }
  if (!std::isfinite(arm_command_velocity_limit_scale_) ||
    arm_command_velocity_limit_scale_ <= 0.0)
  {
    arm_command_velocity_limit_scale_ = 0.25;
  }

  if (const auto it = info_.hardware_parameters.find("arm_command_position_deadband");
    it != info_.hardware_parameters.end())
  {
    arm_command_position_deadband_ = std::stod(it->second);
  }
  if (!std::isfinite(arm_command_position_deadband_) || arm_command_position_deadband_ < 0.0)
  {
    arm_command_position_deadband_ = 1e-5;
  }

  if (const auto it = info_.hardware_parameters.find("gripper_rad_to_m");
    it != info_.hardware_parameters.end())
  {
    gripper_rad_to_m_ = std::stod(it->second);
  }
  if (!std::isfinite(gripper_rad_to_m_) || std::abs(gripper_rad_to_m_) < 1e-9)
  {
    gripper_rad_to_m_ = 0.025;
  }

  if (const auto it = info_.hardware_parameters.find("gripper_command_epsilon");
    it != info_.hardware_parameters.end())
  {
    gripper_command_epsilon_ = std::stod(it->second);
  }
  if (!std::isfinite(gripper_command_epsilon_) || gripper_command_epsilon_ < 0.0)
  {
    gripper_command_epsilon_ = 1e-4;
  }

  shutdown_return_home_ = parse_bool_parameter("shutdown_return_home", true);
  shutdown_home_positions_.assign(kArmJointCount, 0.0);
  if (const auto it = info_.hardware_parameters.find("shutdown_home");
    it != info_.hardware_parameters.end())
  {
    try
    {
      shutdown_home_positions_ = parseCsvDoubles(it->second, kArmJointCount);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Invalid shutdown_home '%s': %s", it->second.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (const auto it = info_.hardware_parameters.find("shutdown_home_timeout");
    it != info_.hardware_parameters.end())
  {
    shutdown_home_timeout_sec_ = std::stod(it->second);
  }
  if (!std::isfinite(shutdown_home_timeout_sec_) || shutdown_home_timeout_sec_ <= 0.0)
  {
    shutdown_home_timeout_sec_ = 3.5;
  }
  if (const auto it = info_.hardware_parameters.find("shutdown_home_tolerance");
    it != info_.hardware_parameters.end())
  {
    shutdown_home_tolerance_ = std::stod(it->second);
  }
  if (!std::isfinite(shutdown_home_tolerance_) || shutdown_home_tolerance_ <= 0.0)
  {
    shutdown_home_tolerance_ = 0.05;
  }
  if (const auto it = info_.hardware_parameters.find("shutdown_home_velocity");
    it != info_.hardware_parameters.end())
  {
    shutdown_home_velocity_ = std::stod(it->second);
  }
  if (!std::isfinite(shutdown_home_velocity_) || shutdown_home_velocity_ <= 0.0)
  {
    shutdown_home_velocity_ = 0.3;
  }
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Shutdown return-home: %s, home=%s, timeout=%.1fs, tol=%.3f, vel=%.2f",
    shutdown_return_home_ ? "enabled" : "disabled",
    formatVector(shutdown_home_positions_).c_str(),
    shutdown_home_timeout_sec_, shutdown_home_tolerance_, shutdown_home_velocity_);

  use_velocity_commands_ = (control_mode_ == "full_control");
  use_effort_commands_ = (control_mode_ == "full_control");
  use_gain_commands_ = (control_mode_ == "full_control");

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_kp_.resize(info_.joints.size(), 0.0);
  hw_commands_kd_.resize(info_.joints.size(), 0.0);
  max_torques_.resize(info_.joints.size(), 0.0);
  max_velocities_.resize(info_.joints.size(), 0.5);
  kp_gains_.resize(info_.joints.size(), 0.0);
  kd_gains_.resize(info_.joints.size(), 0.0);
  last_motor_command_positions_.assign(expected_motors_, 0.0);
  filtered_motor_command_velocities_.assign(expected_motors_, 0.0);
  last_gripper_command_m_.assign(arm_count_, std::numeric_limits<double>::quiet_NaN());
  has_last_motor_command_positions_ = false;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (const auto it = info_.joints[i].parameters.find("max_torque");
      it != info_.joints[i].parameters.end())
    {
      max_torques_[i] = std::stod(it->second);
    }
    else
    {
      max_torques_[i] = 10.0;
    }

    if (const auto it = info_.joints[i].parameters.find("max_velocity");
      it != info_.joints[i].parameters.end())
    {
      max_velocities_[i] = std::stod(it->second);
    }
    else
    {
      max_velocities_[i] = 0.5;
    }

    if (const auto it = info_.joints[i].parameters.find("kp");
      it != info_.joints[i].parameters.end())
    {
      kp_gains_[i] = std::stod(it->second);
    }
    else
    {
      kp_gains_[i] = 4.0;
    }

    if (const auto it = info_.joints[i].parameters.find("kd");
      it != info_.joints[i].parameters.end())
    {
      kd_gains_[i] = std::stod(it->second);
    }
    else
    {
      kd_gains_[i] = 0.5;
    }

    hw_commands_kp_[i] = kp_gains_[i];
    hw_commands_kd_[i] = kd_gains_[i];
  }

  if (!validateStorageLayout("on_init"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Panthera hardware initialized | arms=%zu motors=%zu joints=%zu | config=%s | mode=%s",
    arm_count_, expected_motors_, info_.joints.size(),
    config_file_.c_str(), control_mode_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    robot_ = std::make_unique<hightorque_robot::robot>(config_file_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to initialize Panthera robot: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validateMotorCount("on_configure") || !validateStorageLayout("on_configure"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_->send_get_motor_state_cmd();
  robot_->motor_send_cmd();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)) ==
           hardware_interface::return_type::OK ?
           hardware_interface::CallbackReturn::SUCCESS :
           hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface>
PantheraHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PantheraHardwareInterface::export_command_interfaces()
{
  // Always export the same set as interfaces.xacro (position/velocity/effort/kp/kd).
  // control_mode_ only changes how write() uses those buffers — not which are advertised.
  // Otherwise jazzy resource_manager rejects pd_control / position_velocity as a discrepancy.
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (isMimicGripperJoint(i))
    {
      continue;
    }

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);

    // Grippers stay position-only (AdaptiveGripperController).
    if (!isPrimaryGripperJoint(i))
    {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]);
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]);
      command_interfaces.emplace_back(
        info_.joints[i].name, "kp", &hw_commands_kp_[i]);
      command_interfaces.emplace_back(
        info_.joints[i].name, "kd", &hw_commands_kd_[i]);
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!validateMotorCount("on_activate"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)) !=
    hardware_interface::return_type::OK)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    // Hold current pose until a controller starts writing valid commands.
    // Non-finite position targets are a common cause of motor runaway on enable.
    if (std::isfinite(hw_positions_[joint_index]))
    {
      hw_commands_positions_[joint_index] = hw_positions_[joint_index];
    }
    hw_commands_velocities_[joint_index] = 0.0;
    hw_commands_efforts_[joint_index] = 0.0;
    hw_commands_kp_[joint_index] = kp_gains_[joint_index];
    hw_commands_kd_[joint_index] = kd_gains_[joint_index];
  }

  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    const size_t joint_index = jointIndexFromMotorIndex(motor_index);
    if (joint_index >= info_.joints.size())
    {
      continue;
    }
    last_motor_command_positions_[motor_index] =
      jointPositionToMotorPosition(joint_index, hw_commands_positions_[joint_index]);
    filtered_motor_command_velocities_[motor_index] = 0.0;
  }
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
  {
    const size_t joint_index = full_arm_layout_ ?
      arm_index * kJointsPerArm + 6 : 6;
    if (joint_index < info_.joints.size() && isPrimaryGripperJoint(joint_index))
    {
      last_gripper_command_m_[arm_index] = hw_commands_positions_[joint_index];
    }
  }
  has_last_motor_command_positions_ = true;

  try
  {
    sendActivateHoldCommand();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to send activate hold command: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void PantheraHardwareInterface::sendActivateHoldCommand()
{
  if (!robot_ || !validateMotorCount("activate_hold"))
  {
    return;
  }

  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    const bool is_gripper_motor = ((motor_index + 1) % kMotorsPerArm) == 0;
    if (is_gripper_motor)
    {
      continue;
    }

    const size_t joint_index = jointIndexFromMotorIndex(motor_index);
    if (joint_index >= info_.joints.size())
    {
      continue;
    }

    const double hold_pos = std::isfinite(hw_positions_[joint_index]) ?
      hw_positions_[joint_index] : 0.0;
    const double motor_pos = jointPositionToMotorPosition(joint_index, hold_pos);
    const double kp = joint_index < kp_gains_.size() ? kp_gains_[joint_index] : 4.0;
    const double kd = joint_index < kd_gains_.size() ? kd_gains_[joint_index] : 0.5;

    auto * motor = robot_->Motors[motor_index];
    if (control_mode_ == "full_control" || control_mode_ == "pd_control")
    {
      motor->pos_vel_tqe_kp_kd(motor_pos, 0.0f, 0.0f, kp, kd);
    }
    else
    {
      const double max_tqe = joint_index < max_torques_.size() ? max_torques_[joint_index] : 10.0;
      motor->pos_vel_MAXtqe(motor_pos, 0.0f, static_cast<float>(max_tqe));
    }
  }
  robot_->motor_send_cmd();
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (robot_)
  {
    try
    {
      if (shutdown_return_home_)
      {
        moveToShutdownHomeThenStop();
      }
      else
      {
        robot_->set_stop();
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Failed during deactivation: %s", e.what());
      try
      {
        robot_->set_stop();
      }
      catch (...)
      {
      }
    }
  }
  has_last_motor_command_positions_ = false;
  last_gripper_command_m_.assign(arm_count_, std::numeric_limits<double>::quiet_NaN());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void PantheraHardwareInterface::moveToShutdownHomeThenStop()
{
  if (!robot_ || !validateMotorCount("shutdown_home"))
  {
    if (robot_)
    {
      robot_->set_stop();
    }
    return;
  }

  const auto logger = rclcpp::get_logger(kLoggerName);

  // Read current joint positions — commands must start from here and interpolate to home.
  // Never jump the position command to 0 in one step.
  robot_->send_get_motor_state_cmd();
  robot_->motor_send_cmd();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  robot_->send_get_motor_state_cmd();
  robot_->motor_send_cmd();

  std::vector<double> start_positions(arm_count_ * kArmJointCount, 0.0);
  double max_travel = 0.0;
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
  {
    const size_t motor_base = arm_index * kMotorsPerArm;
    for (size_t joint_offset = 0; joint_offset < kArmJointCount; ++joint_offset)
    {
      const auto * state =
        robot_->Motors[motor_base + joint_offset]->get_current_motor_state();
      const double start = state->position;
      const double goal = shutdown_home_positions_[joint_offset];
      start_positions[arm_index * kArmJointCount + joint_offset] = start;
      max_travel = std::max(max_travel, std::abs(goal - start));
    }
  }

  // Time from farthest joint / commanded speed (capped so launch SIGTERM ~5s still OK).
  // Never shorter than 2s — sub-second return-to-home is unsafe on a real arm.
  constexpr double kMinDurationSec = 2.0;
  constexpr double kMaxDurationSec = 4.0;
  const double speed = std::max(0.05, std::abs(shutdown_home_velocity_));
  const double duration_sec = std::clamp(
    std::max(max_travel / speed, kMinDurationSec),
    kMinDurationSec,
    std::min(kMaxDurationSec, std::max(kMinDurationSec, shutdown_home_timeout_sec_)));

  RCLCPP_WARN(
    logger,
    "Shutdown interpolate: max_travel=%.3f rad, duration=%.2fs, speed<=%.2f rad/s, home=%s",
    max_travel, duration_sec, speed, formatVector(shutdown_home_positions_).c_str());

  if (max_travel <= shutdown_home_tolerance_)
  {
    RCLCPP_INFO(logger, "Already near shutdown home; entering damping");
    robot_->set_stop();
    robot_->set_stop();
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  constexpr auto kDt = std::chrono::milliseconds(20);
  double prev_alpha = 0.0;

  // Smoothstep: 0→1 with zero start/end slope (gentler than raw linear).
  auto smoothstep = [](double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  };

  while (true)
  {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double alpha_lin = std::clamp(elapsed / duration_sec, 0.0, 1.0);
    const double alpha = smoothstep(alpha_lin);
    const double d_alpha = std::max(0.0, alpha - prev_alpha);
    prev_alpha = alpha;

    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
    {
      const size_t motor_base = arm_index * kMotorsPerArm;
      const size_t joint_base = full_arm_layout_ ? arm_index * kJointsPerArm : 0;

      for (size_t joint_offset = 0; joint_offset < kArmJointCount; ++joint_offset)
      {
        const size_t motor_index = motor_base + joint_offset;
        const size_t joint_index = joint_base + joint_offset;
        const double start = start_positions[arm_index * kArmJointCount + joint_offset];
        const double goal = shutdown_home_positions_[joint_offset];
        // Interpolated command: start → goal (never a one-shot assignment to 0).
        const double cmd = start + alpha * (goal - start);
        // Feedforward vel from this step's command delta.
        const double step_dt = std::max(0.001, std::chrono::duration<double>(kDt).count());
        const double cmd_vel = std::clamp(
          (d_alpha * (goal - start)) / step_dt, -speed, speed);

        const double urdf_tqe =
          (joint_index < max_torques_.size() && max_torques_[joint_index] > 0.0)
            ? max_torques_[joint_index]
            : 10.0;
        const double max_tqe = std::max(urdf_tqe, 80.0);

        robot_->Motors[motor_index]->pos_vel_MAXtqe(
          static_cast<float>(cmd),
          static_cast<float>(cmd_vel),
          static_cast<float>(max_tqe));
      }

      const size_t gripper_motor_index = motor_base + kArmJointCount;
      const size_t gripper_joint_index = joint_base + kArmJointCount;
      const double grip_tqe =
        (gripper_joint_index < max_torques_.size() && max_torques_[gripper_joint_index] > 0.0)
          ? max_torques_[gripper_joint_index]
          : 3.0;
      robot_->Motors[gripper_motor_index]->pos_vel_MAXtqe(
        0.0f, 0.0f, static_cast<float>(grip_tqe));
    }
    robot_->motor_send_cmd();

    if (alpha_lin >= 1.0)
    {
      break;
    }
    std::this_thread::sleep_for(kDt);
  }

  // Hold final home (vel=0) briefly, still as continuous commands at the goal.
  for (int i = 0; i < 15; ++i)
  {
    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
    {
      const size_t motor_base = arm_index * kMotorsPerArm;
      const size_t joint_base = full_arm_layout_ ? arm_index * kJointsPerArm : 0;
      for (size_t joint_offset = 0; joint_offset < kArmJointCount; ++joint_offset)
      {
        const size_t joint_index = joint_base + joint_offset;
        const double urdf_tqe =
          (joint_index < max_torques_.size() && max_torques_[joint_index] > 0.0)
            ? max_torques_[joint_index]
            : 10.0;
        const double max_tqe = std::max(urdf_tqe, 80.0);
        robot_->Motors[motor_base + joint_offset]->pos_vel_MAXtqe(
          static_cast<float>(shutdown_home_positions_[joint_offset]),
          0.0f,
          static_cast<float>(max_tqe));
      }
    }
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(kDt);
  }

  RCLCPP_INFO(logger, "Shutdown home interpolation finished; entering damping (set_stop)");
  robot_->set_stop();
  robot_->set_stop();
}

hardware_interface::return_type PantheraHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!validateMotorCount("read") || !validateStorageLayout("read"))
  {
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();

    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
    {
      const size_t motor_base = arm_index * kMotorsPerArm;
      const size_t joint_base = full_arm_layout_ ? arm_index * kJointsPerArm : 0;

      for (size_t joint_offset = 0; joint_offset < kArmJointCount; ++joint_offset)
      {
        auto * state = robot_->Motors[motor_base + joint_offset]->get_current_motor_state();
        hw_positions_[joint_base + joint_offset] = state->position;
        hw_velocities_[joint_base + joint_offset] = state->velocity;
        hw_efforts_[joint_base + joint_offset] = state->torque;
      }

      const size_t gripper_joint = joint_base + 6;
      if (gripper_joint < info_.joints.size() && isPrimaryGripperJoint(gripper_joint))
      {
        auto * gripper_state =
          robot_->Motors[motor_base + kArmJointCount]->get_current_motor_state();
        hw_positions_[gripper_joint] =
          motorPositionToJointPosition(gripper_joint, gripper_state->position);
        hw_velocities_[gripper_joint] =
          motorVelocityToJointVelocity(gripper_joint, gripper_state->velocity);
        hw_efforts_[gripper_joint] = gripper_state->torque;

        const size_t mimic_joint = joint_base + 7;
        if (mimic_joint < info_.joints.size() && isMimicGripperJoint(mimic_joint))
        {
          const double mimic_sign =
            mimicUsesNegatedSign(info_.joints[mimic_joint].name) ? -1.0 : 1.0;
          hw_positions_[mimic_joint] = mimic_sign * hw_positions_[gripper_joint];
          hw_velocities_[mimic_joint] = mimic_sign * hw_velocities_[gripper_joint];
          hw_efforts_[mimic_joint] = 0.0;
        }
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      *rclcpp::Clock::make_shared(), 1000,
      "Failed to read hardware state: %s",
      e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PantheraHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!validateMotorCount("write") || !validateStorageLayout("write"))
  {
    return hardware_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 1e-4 || dt > 1.0)
  {
    dt = 0.01;
  }

  // Seed with current state so any skipped/invalid joint never defaults to position 0.
  std::vector<double> motor_positions(expected_motors_, 0.0);
  std::vector<double> motor_velocities(expected_motors_, 0.0);
  std::vector<double> motor_efforts(expected_motors_, 0.0);
  std::vector<double> motor_torque_limits(expected_motors_, 0.0);
  std::vector<double> motor_kp(expected_motors_, 0.0);
  std::vector<double> motor_kd(expected_motors_, 0.0);
  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    const size_t joint_index = jointIndexFromMotorIndex(motor_index);
    if (joint_index >= info_.joints.size())
    {
      continue;
    }
    const double hold_pos = std::isfinite(hw_positions_[joint_index]) ?
      hw_positions_[joint_index] : 0.0;
    motor_positions[motor_index] = jointPositionToMotorPosition(joint_index, hold_pos);
    motor_torque_limits[motor_index] =
      joint_index < max_torques_.size() ? max_torques_[joint_index] : 10.0;
    motor_kp[motor_index] = joint_index < kp_gains_.size() ? kp_gains_[joint_index] : 4.0;
    motor_kd[motor_index] = joint_index < kd_gains_.size() ? kd_gains_[joint_index] : 0.5;
  }

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    if (isMimicGripperJoint(joint_index))
    {
      continue;
    }

    const size_t motor_index = motorIndexFromJointIndex(joint_index);
    if (motor_index >= expected_motors_)
    {
      continue;
    }

    double cmd_pos = hw_commands_positions_[joint_index];
    if (!std::isfinite(cmd_pos))
    {
      // Invalid command (common right after claim): hold measured position.
      cmd_pos = std::isfinite(hw_positions_[joint_index]) ? hw_positions_[joint_index] : 0.0;
      hw_commands_positions_[joint_index] = cmd_pos;
    }

    double cmd_vel = hw_commands_velocities_[joint_index];
    if (!std::isfinite(cmd_vel))
    {
      cmd_vel = 0.0;
      hw_commands_velocities_[joint_index] = 0.0;
    }

    double cmd_eff = 0.0;
    if (control_mode_ == "full_control")
    {
      cmd_eff = hw_commands_efforts_[joint_index];
      if (!std::isfinite(cmd_eff))
      {
        cmd_eff = 0.0;
        hw_commands_efforts_[joint_index] = 0.0;
      }
    }

    motor_positions[motor_index] = jointPositionToMotorPosition(joint_index, cmd_pos);
    motor_velocities[motor_index] = jointVelocityToMotorVelocity(joint_index, cmd_vel);
    motor_efforts[motor_index] = cmd_eff;
    motor_torque_limits[motor_index] = max_torques_[joint_index];

    double kp = use_gain_commands_ ? hw_commands_kp_[joint_index] : kp_gains_[joint_index];
    double kd = use_gain_commands_ ? hw_commands_kd_[joint_index] : kd_gains_[joint_index];
    if (!std::isfinite(kp) || kp <= 0.0)
    {
      kp = kp_gains_[joint_index];
    }
    if (!std::isfinite(kd) || kd < 0.0)
    {
      kd = kd_gains_[joint_index];
    }
    motor_kp[motor_index] = kp;
    motor_kd[motor_index] = kd;
  }

  std::vector<double> derived_motor_velocities(expected_motors_, 0.0);
  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    const bool is_gripper = ((motor_index + 1) % kMotorsPerArm) == 0;
    const size_t joint_index = jointIndexFromMotorIndex(motor_index);
    if (joint_index >= info_.joints.size())
    {
      continue;
    }
    const double epsilon = is_gripper ? gripper_command_epsilon_ / gripper_rad_to_m_ :
      arm_command_position_deadband_;

    const double command_delta = has_last_motor_command_positions_ ?
      motor_positions[motor_index] - last_motor_command_positions_[motor_index] : 0.0;

    double target_error = 0.0;
    if (std::isfinite(hw_positions_[joint_index]))
    {
      const double current_motor_position =
        jointPositionToMotorPosition(joint_index, hw_positions_[joint_index]);
      target_error = motor_positions[motor_index] - current_motor_position;
    }

    if (std::abs(command_delta) <= epsilon && std::abs(target_error) <= epsilon)
    {
      derived_motor_velocities[motor_index] = 0.0;
      continue;
    }

    const double joint_velocity_limit = std::abs(max_velocities_[joint_index]);
    const double motor_velocity_limit_from_joint = std::abs(
      jointVelocityToMotorVelocity(joint_index, joint_velocity_limit));
    double velocity_limit = std::max(
      0.02, std::abs(motor_velocities[motor_index]) > 1e-9 ?
      std::abs(motor_velocities[motor_index]) :
      motor_velocity_limit_from_joint);
    velocity_limit *= arm_command_velocity_limit_scale_;

    double raw_velocity = 0.0;
    if (std::abs(command_delta) > epsilon)
    {
      raw_velocity = command_delta / dt;
    }
    else if (control_mode_ != "pd_control" && std::abs(target_error) > epsilon)
    {
      // pd_control: rely on MIT kp/kd when the position setpoint is steady.
      // Chasing steady-state error with velocity feedforward can destabilize
      // gravity-loaded joints (e.g. joint3) at startup.
      raw_velocity = std::copysign(velocity_limit, target_error);
    }

    raw_velocity = std::clamp(raw_velocity, -velocity_limit, velocity_limit);
    derived_motor_velocities[motor_index] =
      arm_velocity_filter_alpha_ * raw_velocity +
      (1.0 - arm_velocity_filter_alpha_) * filtered_motor_command_velocities_[motor_index];
  }

  last_motor_command_positions_ = motor_positions;
  filtered_motor_command_velocities_ = derived_motor_velocities;
  has_last_motor_command_positions_ = true;

  try
  {
    for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
    {
      const bool is_gripper_motor = ((motor_index + 1) % kMotorsPerArm) == 0;
      if (is_gripper_motor)
      {
        continue;
      }

      auto * motor = robot_->Motors[motor_index];
      if (control_mode_ == "full_control" || control_mode_ == "pd_control")
      {
        const double velocity = control_mode_ == "full_control" ?
          motor_velocities[motor_index] : derived_motor_velocities[motor_index];
        motor->pos_vel_tqe_kp_kd(
          motor_positions[motor_index], velocity, motor_efforts[motor_index],
          motor_kp[motor_index], motor_kd[motor_index]);
      }
      else
      {
        motor->pos_vel_MAXtqe(
          motor_positions[motor_index], derived_motor_velocities[motor_index],
          motor_torque_limits[motor_index]);
      }
    }
    robot_->motor_send_cmd();

    // Gripper commands separately when targets change (avoids dual-arm bus ignore).
    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
    {
      const size_t joint_index = full_arm_layout_ ?
        arm_index * kJointsPerArm + 6 : 6;
      const size_t motor_index = arm_index * kMotorsPerArm + 6;
      if (joint_index >= info_.joints.size() || !isPrimaryGripperJoint(joint_index))
      {
        continue;
      }
      const double gripper_pos_m = hw_commands_positions_[joint_index];
      if (!std::isfinite(gripper_pos_m))
      {
        continue;
      }
      if (last_gripper_command_m_.size() <= arm_index)
      {
        continue;
      }
      if (std::isfinite(last_gripper_command_m_[arm_index]) &&
        std::abs(gripper_pos_m - last_gripper_command_m_[arm_index]) <= gripper_command_epsilon_)
      {
        continue;
      }

      auto * motor = robot_->Motors[motor_index];
      const double gripper_pos_rad = jointPositionToMotorPosition(joint_index, gripper_pos_m);
      const double gripper_vel_joint = use_velocity_commands_ ?
        hw_commands_velocities_[joint_index] : max_velocities_[joint_index];
      const double gripper_vel_rad =
        jointVelocityToMotorVelocity(joint_index, gripper_vel_joint);

      if (control_mode_ == "full_control")
      {
        motor->pos_vel_tqe_kp_kd(
          gripper_pos_rad, gripper_vel_rad, hw_commands_efforts_[joint_index],
          kp_gains_[joint_index], kd_gains_[joint_index]);
      }
      else
      {
        motor->pos_vel_MAXtqe(
          gripper_pos_rad, gripper_vel_rad, max_torques_[joint_index]);
      }
      robot_->motor_send_cmd();
      last_gripper_command_m_[arm_index] = gripper_pos_m;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      *rclcpp::Clock::make_shared(), 1000,
      "Failed to write hardware command: %s",
      e.what());
    return hardware_interface::return_type::ERROR;
  }

  if (command_debug_enabled_)
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_command_debug_time_).count();
    if (elapsed >= command_debug_period_)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger(kLoggerName),
        "Command positions=%s | state=%s | arms=%zu motors=%zu",
        formatVector(hw_commands_positions_).c_str(),
        formatVector(hw_positions_).c_str(),
        arm_count_, expected_motors_);
      last_command_debug_time_ = now;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace panthera_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panthera_ros2_control::PantheraHardwareInterface, hardware_interface::SystemInterface)
