#include "panthera_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware/robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ht_ros2_control
{

namespace
{
constexpr const char * kLoggerName = "PantheraHardwareInterface";
constexpr double kInvalidMotorPosition = 999.0;
constexpr double kMaxReasonableJointPosition = 10.0;

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
}  // namespace

bool PantheraHardwareInterface::resolveArmLayout()
{
  const size_t joint_count = info_.joints.size();
  auto logger = rclcpp::get_logger(kLoggerName);

  // 每臂 7 关节（6 臂关节 + 夹爪），关节与电机一一对应
  if (joint_count == 0 || joint_count % kMotorsPerArm != 0)
  {
    RCLCPP_ERROR(
      logger,
      "Unsupported joint layout: expected N*%zu joints (N arms), got %zu",
      kMotorsPerArm, joint_count);
    return false;
  }
  arm_count_ = joint_count / kMotorsPerArm;
  expected_motors_ = joint_count;
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

bool PantheraHardwareInterface::isPrimaryGripperJoint(size_t joint_index) const
{
  // 每臂最后一个关节（offset 6）是夹爪
  return joint_index % kMotorsPerArm == (kMotorsPerArm - 1);
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

bool PantheraHardwareInterface::isValidJointFeedback(double position) const
{
  return std::isfinite(position) &&
         std::abs(position - kInvalidMotorPosition) > 1.0 &&
         std::abs(position) < kMaxReasonableJointPosition;
}

bool PantheraHardwareInterface::allArmMotorsHaveValidFeedback() const
{
  // 只检查 6 个臂关节；夹爪初始反馈允许为 0（初始值 0.01 m）
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
  {
    for (size_t offset = 0; offset < kArmJointCount; ++offset)
    {
      const size_t joint_index = arm_index * kMotorsPerArm + offset;
      if (joint_index >= latest_positions_.size() ||
        !isValidJointFeedback(latest_positions_[joint_index]))
      {
        return false;
      }
    }
  }
  return true;
}

bool PantheraHardwareInterface::waitForValidMotorFeedback(
  const char * context, int timeout_ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline)
  {
    pollRobotState();
    if (allArmMotorsHaveValidFeedback())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "%s: all arm motors reported valid joint feedback", context);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  RCLCPP_WARN(
    rclcpp::get_logger(kLoggerName),
    "%s: timed out after %d ms waiting for valid joint feedback on all arm motors",
    context, timeout_ms);
  return false;
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

  if (!resolveArmLayout())
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

  full_control_ = (control_mode_ == "full_control");

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

  // 共享缓冲（RT 线程与后台 IO 线程通过 io_mutex_ 交换）
  latest_positions_.resize(info_.joints.size(), 0.0);
  latest_velocities_.resize(info_.joints.size(), 0.0);
  latest_efforts_.resize(info_.joints.size(), 0.0);
  cmd_positions_.resize(info_.joints.size(), 0.0);
  cmd_velocities_.resize(info_.joints.size(), 0.0);
  cmd_efforts_.resize(info_.joints.size(), 0.0);
  cmd_kp_.resize(info_.joints.size(), 0.0);
  cmd_kd_.resize(info_.joints.size(), 0.0);

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

  if (!validateMotorCount("on_configure"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!waitForValidMotorFeedback("on_configure", 3000))
  {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Proceeding with partial motor feedback; startup may be unstable until all motors respond");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
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

  if (!waitForValidMotorFeedback("on_activate", 3000))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Refusing to activate: not all arm motors have valid position feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 以当前实测位姿作为初始保持指令（拒绝 SDK 占位 999 / NaN，防止使能后失控）
  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    if (isValidJointFeedback(latest_positions_[joint_index]))
    {
      hw_commands_positions_[joint_index] = latest_positions_[joint_index];
    }
    hw_commands_velocities_[joint_index] = 0.0;
    hw_commands_efforts_[joint_index] = 0.0;
    hw_commands_kp_[joint_index] = kp_gains_[joint_index];
    hw_commands_kd_[joint_index] = kd_gains_[joint_index];
  }

  // 初始化速度前馈 / 夹爪跟踪状态（关节与电机一一对应）
  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    last_motor_command_positions_[motor_index] =
      jointPositionToMotorPosition(motor_index, hw_commands_positions_[motor_index]);
    filtered_motor_command_velocities_[motor_index] = 0.0;
  }
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
  {
    const size_t joint_index = arm_index * kMotorsPerArm + (kMotorsPerArm - 1);
    last_gripper_command_m_[arm_index] = hw_commands_positions_[joint_index];
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

  // 激活完成后启动后台 IO 线程接管电机指令与状态轮询
  startIoThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void PantheraHardwareInterface::sendActivateHoldCommand()
{
  if (!robot_)
  {
    return;
  }

  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    // 夹爪由夹爪控制器接管，激活保持只下臂关节
    if (isPrimaryGripperJoint(motor_index))
    {
      continue;
    }

    if (!isValidJointFeedback(latest_positions_[motor_index]))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Activate hold skipped motor %zu: invalid feedback %.3f",
        motor_index, latest_positions_[motor_index]);
      continue;
    }
    const double motor_pos =
      jointPositionToMotorPosition(motor_index, latest_positions_[motor_index]);

    auto * motor = robot_->Motors[motor_index];
    if (full_control_ || control_mode_ == "pd_control")
    {
      motor->pos_vel_tqe_kp_kd(
        motor_pos, 0.0f, 0.0f, kp_gains_[motor_index], kd_gains_[motor_index]);
    }
    else
    {
      motor->pos_vel_MAXtqe(
        motor_pos, 0.0f, static_cast<float>(max_torques_[motor_index]));
    }
  }
  robot_->motor_send_cmd();
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 先停止后台 IO 线程，避免与下面的阻塞式 SDK 调用并发访问 robot_
  stopIoThread();
  has_last_motor_command_positions_ = false;
  last_gripper_command_m_.assign(arm_count_, std::numeric_limits<double>::quiet_NaN());

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
  return hardware_interface::CallbackReturn::SUCCESS;
}

void PantheraHardwareInterface::moveToShutdownHomeThenStop()
{
  if (!robot_)
  {
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

      for (size_t joint_offset = 0; joint_offset < kArmJointCount; ++joint_offset)
      {
        const size_t motor_index = motor_base + joint_offset;
        const size_t joint_index = motor_index;  // 关节与电机一一对应
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
      const double grip_tqe =
        (gripper_motor_index < max_torques_.size() && max_torques_[gripper_motor_index] > 0.0)
          ? max_torques_[gripper_motor_index]
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
      for (size_t joint_offset = 0; joint_offset < kArmJointCount; ++joint_offset)
      {
        const size_t joint_index = motor_base + joint_offset;
        const double urdf_tqe =
          (joint_index < max_torques_.size() && max_torques_[joint_index] > 0.0)
            ? max_torques_[joint_index]
            : 10.0;
        const double max_tqe = std::max(urdf_tqe, 80.0);
        robot_->Motors[joint_index]->pos_vel_MAXtqe(
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

PantheraHardwareInterface::~PantheraHardwareInterface()
{
  stopIoThread();
}

void PantheraHardwareInterface::startIoThread()
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (io_thread_.joinable())
  {
    return;  // 已在运行
  }
  io_exit_ = false;
  command_pending_ = false;
  io_thread_ = std::thread(&PantheraHardwareInterface::ioThreadLoop, this);
}

void PantheraHardwareInterface::stopIoThread()
{
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    io_exit_ = true;
  }
  io_cv_.notify_all();
  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void PantheraHardwareInterface::ioThreadLoop()
{
  std::unique_lock<std::mutex> lock(io_mutex_);
  while (!io_exit_)
  {
    // 命令到达立即唤醒发送；无命令时按 kPollPeriod 兜底轮询反馈
    io_cv_.wait_for(lock, kPollPeriod, [this] { return io_exit_ || command_pending_; });
    if (io_exit_)
    {
      break;
    }
    command_pending_ = false;
    lock.unlock();

    sendMotorCommands();  // 有新命令则下发；无命令则重发上次指令保持电机活跃
    pollRobotState();     // 阻塞查询反馈，刷新 latest_*

    lock.lock();
  }
}

void PantheraHardwareInterface::pollRobotState()
{
  if (!robot_)
  {
    return;
  }
  try
  {
    // 阻塞式查询电机状态（内部异步收包线程维护缓存）
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();

    // 读取电机状态缓存，换算成关节单位（夹爪 rad -> m，其余 1:1）
    std::vector<double> positions(info_.joints.size(), 0.0);
    std::vector<double> velocities(info_.joints.size(), 0.0);
    std::vector<double> efforts(info_.joints.size(), 0.0);
    for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
    {
      auto * state = robot_->Motors[motor_index]->get_current_motor_state();
      if (isPrimaryGripperJoint(motor_index))
      {
        positions[motor_index] = motorPositionToJointPosition(motor_index, state->position);
        velocities[motor_index] = motorVelocityToJointVelocity(motor_index, state->velocity);
      }
      else
      {
        positions[motor_index] = state->position;
        velocities[motor_index] = state->velocity;
      }
      efforts[motor_index] = state->torque;
    }

    // 只更新有效反馈；SDK 占位 999 / NaN 保留上次值
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
      {
        if (isValidJointFeedback(positions[joint_index]))
        {
          latest_positions_[joint_index] = positions[joint_index];
          latest_velocities_[joint_index] = velocities[joint_index];
          latest_efforts_[joint_index] = efforts[joint_index];
        }
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      *get_clock(), 1000,
      "Failed to poll robot state: %s", e.what());
  }
}

void PantheraHardwareInterface::sendMotorCommands()
{
  if (!robot_)
  {
    return;
  }

  // 按真实发送节拍推导速度前馈（500Hz 指令 ≈ 2ms）
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::clamp(
    std::chrono::duration<double>(now - last_io_send_time_).count(), 0.001, 0.1);
  last_io_send_time_ = now;

  // 在锁内拷贝命令快照与最新反馈（write() 在 RT 线程写入 cmd_*）
  std::vector<double> cmd_pos, cmd_vel, cmd_eff, cmd_kp, cmd_kd, cur_pos;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    cmd_pos = cmd_positions_;
    cmd_vel = cmd_velocities_;
    cmd_eff = cmd_efforts_;
    cmd_kp = cmd_kp_;
    cmd_kd = cmd_kd_;
    cur_pos = latest_positions_;
  }

  // 以最新反馈 / 上次指令为种子，避免把 SDK 占位 999 当目标
  std::vector<double> motor_positions(expected_motors_, 0.0);
  std::vector<double> motor_velocities(expected_motors_, 0.0);
  std::vector<double> motor_efforts(expected_motors_, 0.0);
  std::vector<double> motor_torque_limits(expected_motors_, 0.0);
  std::vector<double> motor_kp(expected_motors_, 0.0);
  std::vector<double> motor_kd(expected_motors_, 0.0);
  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    if (isValidJointFeedback(cur_pos[motor_index]))
    {
      motor_positions[motor_index] =
        jointPositionToMotorPosition(motor_index, cur_pos[motor_index]);
    }
    else if (has_last_motor_command_positions_)
    {
      motor_positions[motor_index] = last_motor_command_positions_[motor_index];
    }
    motor_torque_limits[motor_index] = max_torques_[motor_index];
    motor_kp[motor_index] = kp_gains_[motor_index];
    motor_kd[motor_index] = kd_gains_[motor_index];
  }

  // 合并控制器指令到电机命令（关节与电机一一对应）
  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    double cmd_position = cmd_pos[joint_index];
    if (!std::isfinite(cmd_position) || !isValidJointFeedback(cmd_position))
    {
      // 指令无效（如刚 claim 时）：保持实测位姿
      if (isValidJointFeedback(cur_pos[joint_index]))
      {
        cmd_position = cur_pos[joint_index];
      }
      else if (has_last_motor_command_positions_)
      {
        cmd_position = motorPositionToJointPosition(
          joint_index, last_motor_command_positions_[joint_index]);
      }
      else
      {
        continue;
      }
    }

    double cmd_velocity = cmd_vel[joint_index];
    if (!std::isfinite(cmd_velocity))
    {
      cmd_velocity = 0.0;
    }

    double cmd_effort = full_control_ ? cmd_eff[joint_index] : 0.0;
    if (!std::isfinite(cmd_effort))
    {
      cmd_effort = 0.0;
    }

    motor_positions[joint_index] = jointPositionToMotorPosition(joint_index, cmd_position);
    motor_velocities[joint_index] = jointVelocityToMotorVelocity(joint_index, cmd_velocity);
    motor_efforts[joint_index] = cmd_effort;
    motor_torque_limits[joint_index] = max_torques_[joint_index];

    double kp = full_control_ ? cmd_kp[joint_index] : kp_gains_[joint_index];
    double kd = full_control_ ? cmd_kd[joint_index] : kd_gains_[joint_index];
    if (!std::isfinite(kp) || kp <= 0.0)
    {
      kp = kp_gains_[joint_index];
    }
    if (!std::isfinite(kd) || kd < 0.0)
    {
      kd = kd_gains_[joint_index];
    }
    motor_kp[joint_index] = kp;
    motor_kd[joint_index] = kd;
  }

  // 推导速度前馈：死区 + 限幅 + 一阶低通滤波
  std::vector<double> derived_motor_velocities(expected_motors_, 0.0);
  for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
  {
    const bool is_gripper = isPrimaryGripperJoint(motor_index);
    const double epsilon = is_gripper ? gripper_command_epsilon_ / gripper_rad_to_m_
                                      : arm_command_position_deadband_;

    const double command_delta = has_last_motor_command_positions_
      ? motor_positions[motor_index] - last_motor_command_positions_[motor_index]
      : 0.0;

    double target_error = 0.0;
    if (std::isfinite(cur_pos[motor_index]))
    {
      const double current_motor_position =
        jointPositionToMotorPosition(motor_index, cur_pos[motor_index]);
      target_error = motor_positions[motor_index] - current_motor_position;
    }

    if (std::abs(command_delta) <= epsilon && std::abs(target_error) <= epsilon)
    {
      derived_motor_velocities[motor_index] = 0.0;
      continue;
    }

    const double motor_velocity_limit_from_joint = std::abs(
      jointVelocityToMotorVelocity(motor_index, std::abs(max_velocities_[motor_index])));
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
      // pd_control：位置设定点稳定时依赖 MIT kp/kd，不用速度前馈追稳态误差，
      // 避免对重力负载关节（如 joint3）在启动时造成不稳定
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

  // 阻塞下发：全部臂关节合成一条指令帧
  try
  {
    for (size_t motor_index = 0; motor_index < expected_motors_; ++motor_index)
    {
      if (isPrimaryGripperJoint(motor_index))
      {
        continue;  // 夹爪在下方按需单独下发
      }

      auto * motor = robot_->Motors[motor_index];
      if (full_control_ || control_mode_ == "pd_control")
      {
        const double velocity = full_control_
          ? motor_velocities[motor_index]
          : derived_motor_velocities[motor_index];
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

    // 夹爪：目标变化时才单独补发（避免双臂总线忽略）
    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index)
    {
      const size_t joint_index = arm_index * kMotorsPerArm + (kMotorsPerArm - 1);
      const double gripper_pos_m = cmd_pos[joint_index];
      if (!std::isfinite(gripper_pos_m))
      {
        continue;
      }
      if (std::isfinite(last_gripper_command_m_[arm_index]) &&
        std::abs(gripper_pos_m - last_gripper_command_m_[arm_index]) <= gripper_command_epsilon_)
      {
        continue;
      }

      auto * motor = robot_->Motors[joint_index];
      const double gripper_pos_rad = jointPositionToMotorPosition(joint_index, gripper_pos_m);
      const double gripper_vel_joint = full_control_
        ? cmd_vel[joint_index]
        : max_velocities_[joint_index];
      const double gripper_vel_rad =
        jointVelocityToMotorVelocity(joint_index, gripper_vel_joint);

      if (full_control_)
      {
        motor->pos_vel_tqe_kp_kd(
          gripper_pos_rad, gripper_vel_rad, cmd_eff[joint_index],
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
      *get_clock(), 1000,
      "Failed to send motor commands: %s", e.what());
  }
}

hardware_interface::return_type PantheraHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 非阻塞：从后台 IO 线程维护的共享缓冲拷贝最新反馈
  std::lock_guard<std::mutex> lock(io_mutex_);
  std::copy(latest_positions_.begin(), latest_positions_.end(), hw_positions_.begin());
  std::copy(latest_velocities_.begin(), latest_velocities_.end(), hw_velocities_.begin());
  std::copy(latest_efforts_.begin(), latest_efforts_.end(), hw_efforts_.begin());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PantheraHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 非阻塞：把控制器指令快照交给后台 IO 线程发送
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    std::copy(
      hw_commands_positions_.begin(), hw_commands_positions_.end(),
      cmd_positions_.begin());
    std::copy(
      hw_commands_velocities_.begin(), hw_commands_velocities_.end(),
      cmd_velocities_.begin());
    std::copy(
      hw_commands_efforts_.begin(), hw_commands_efforts_.end(),
      cmd_efforts_.begin());
    std::copy(hw_commands_kp_.begin(), hw_commands_kp_.end(), cmd_kp_.begin());
    std::copy(hw_commands_kd_.begin(), hw_commands_kd_.end(), cmd_kd_.begin());
    command_pending_ = true;
  }
  io_cv_.notify_one();
  return hardware_interface::return_type::OK;
}

}  // namespace ht_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ht_ros2_control::PantheraHardwareInterface, hardware_interface::SystemInterface)
