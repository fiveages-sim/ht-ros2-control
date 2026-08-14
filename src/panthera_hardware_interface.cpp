#include "panthera_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
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
constexpr double kInvalidMotorPosition = 999.0;  // SDK 未连接/占位反馈
constexpr double kMaxReasonableJointPosition = 10.0;  // rad（夹爪 m）

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

double parseDoubleOrDefault(const std::string & text, double default_value)
{
  try
  {
    return std::stod(text);
  }
  catch (const std::exception &)
  {
    return default_value;
  }
}

bool parseBoolOrDefault(std::string text, bool default_value)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return (text == "true" || text == "1" || text == "yes" || text == "on") ?
    true : default_value;
}
}  // namespace

namespace detail
{

// ==================== 反馈有效性过滤 ====================

namespace feedback
{

bool isValid(double position)
{
  // SDK 未连接时返回占位 999；NaN / 超范围同样不可信
  return std::isfinite(position) &&
         std::abs(position - kInvalidMotorPosition) > 1.0 &&
         std::abs(position) < kMaxReasonableJointPosition;
}

bool allArmMotorsValid(const std::vector<double> & latest, const MotorLayout & layout)
{
  // 只检查每臂 6 个臂关节；夹爪初始反馈允许为 0（初始值 0.01 m）
  for (size_t arm_index = 0; arm_index < layout.arm_count; ++arm_index)
  {
    for (size_t offset = 0; offset < MotorLayout::kArmJointCount; ++offset)
    {
      const size_t joint_index = arm_index * MotorLayout::kMotorsPerArm + offset;
      if (joint_index >= latest.size() || !isValid(latest[joint_index]))
      {
        return false;
      }
    }
  }
  return true;
}

bool waitForValid(
  const std::string & logger_name, const std::function<void()> & poll,
  const std::vector<double> & latest, const MotorLayout & layout,
  const char * context, int timeout_ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline)
  {
    poll();
    if (allArmMotorsValid(latest, layout))
    {
      RCLCPP_INFO(
        rclcpp::get_logger(logger_name),
        "%s: all arm motors reported valid joint feedback", context);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  RCLCPP_WARN(
    rclcpp::get_logger(logger_name),
    "%s: timed out after %d ms waiting for valid joint feedback on all arm motors",
    context, timeout_ms);
  return false;
}

}  // namespace feedback

// ==================== 电机数校验 ====================

bool validateMotorCount(
  const std::string & logger_name, const hightorque_robot::robot & robot,
  const MotorLayout & layout)
{
  if (robot.Motors.size() != layout.motor_count())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_name),
      "Motor count mismatch: expected %zu motors (%zu arm(s) x %zu), got %zu",
      layout.motor_count(), layout.arm_count, MotorLayout::kMotorsPerArm,
      robot.Motors.size());
    return false;
  }
  return true;
}

// ==================== 硬件配置解析 ====================

bool HardwareConfig::load(const hardware_interface::HardwareInfo & info, size_t arm_count)
{
  const auto & params = info.hardware_parameters;
  auto get = [&params](const std::string & name, const std::string & fallback)
  {
    const auto it = params.find(name);
    return it == params.end() ? fallback : it->second;
  };

  config_file = get("config_file", "");
  if (config_file.empty())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Parameter 'config_file' not found in hardware parameters");
    return false;
  }

  usb_select = get("usb_select", "auto");

  control_mode = get("control_mode", "position_velocity");
  full_control = (control_mode == "full_control");

  gripper_rad_to_m = parseDoubleOrDefault(get("gripper_rad_to_m", "0.025"), 0.025);
  if (!std::isfinite(gripper_rad_to_m) || std::abs(gripper_rad_to_m) < 1e-9)
  {
    gripper_rad_to_m = 0.025;
  }

  shutdown_return_home = parseBoolOrDefault(get("shutdown_return_home", "true"), true);
  try
  {
    shutdown_home_positions = parseCsvDoubles(
      get("shutdown_home", "0.0, 0.0, 0.0, 0.0, 0.0, 0.0"), MotorLayout::kArmJointCount);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Invalid shutdown_home '%s': %s", get("shutdown_home", "").c_str(), e.what());
    return false;
  }
  shutdown_home_timeout_sec =
    parseDoubleOrDefault(get("shutdown_home_timeout", "3.5"), 3.5);
  if (!std::isfinite(shutdown_home_timeout_sec) || shutdown_home_timeout_sec <= 0.0)
  {
    shutdown_home_timeout_sec = 3.5;
  }
  shutdown_home_tolerance =
    parseDoubleOrDefault(get("shutdown_home_tolerance", "0.05"), 0.05);
  if (!std::isfinite(shutdown_home_tolerance) || shutdown_home_tolerance <= 0.0)
  {
    shutdown_home_tolerance = 0.05;
  }
  shutdown_home_velocity =
    parseDoubleOrDefault(get("shutdown_home_velocity", "0.3"), 0.3);
  if (!std::isfinite(shutdown_home_velocity) || shutdown_home_velocity <= 0.0)
  {
    shutdown_home_velocity = 0.3;
  }

  // 限位 / 增益：完整 CSV（每臂 7 值含夹爪 / 每臂 6 值），缺失或非法时回退默认
  const std::vector<double> default_max_torques{21.0, 36.0, 36.0, 21.0, 10.0, 10.0, 0.5};
  const std::vector<double> default_max_velocities{4.2, 5.0, 5.0, 4.2, 3.7, 3.7, 0.3};
  const std::vector<double> default_joint_kp{20.0, 30.0, 40.0, 20.0, 20.0, 20.0};
  const std::vector<double> default_joint_kd{0.2, 0.3, 0.4, 0.2, 0.2, 0.2};

  auto parse_or_fallback = [&](const std::string & name, const std::vector<double> & fallback)
  {
    std::vector<double> out = fallback;
    const std::string text = get(name, "");
    if (!text.empty())
    {
      try
      {
        out = parseCsvDoubles(text, fallback.size() * arm_count);
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "Invalid %s '%s': %s; using default", name.c_str(), text.c_str(), e.what());
      }
    }
    return out;
  };

  max_torques = parse_or_fallback("max_torques", default_max_torques);
  max_velocities = parse_or_fallback("max_velocities", default_max_velocities);
  joint_kp = parse_or_fallback("joint_kp", default_joint_kp);
  joint_kd = parse_or_fallback("joint_kd", default_joint_kd);
  gripper_kp = parseDoubleOrDefault(get("gripper_kp", "5.0"), 5.0);
  gripper_kd = parseDoubleOrDefault(get("gripper_kd", "0.1"), 0.1);

  return true;
}

void HardwareConfig::expand(const MotorLayout & layout)
{
  const size_t n = layout.motor_count();
  motor_max_torques.assign(n, 10.0);
  motor_max_velocities.assign(n, 0.5);
  motor_kp.assign(n, 4.0);
  motor_kd.assign(n, 0.5);
  for (size_t i = 0; i < n; ++i)
  {
    if (i < max_torques.size()) motor_max_torques[i] = max_torques[i];
    if (i < max_velocities.size()) motor_max_velocities[i] = max_velocities[i];
    if (layout.isGripper(i))
    {
      motor_kp[i] = gripper_kp;
      motor_kd[i] = gripper_kd;
    }
    else
    {
      const size_t arm = i / MotorLayout::kMotorsPerArm;
      const size_t offset = i % MotorLayout::kMotorsPerArm;
      const size_t j = arm * MotorLayout::kArmJointCount + offset;
      if (j < joint_kp.size()) motor_kp[i] = joint_kp[j];
      if (j < joint_kd.size()) motor_kd[i] = joint_kd[j];
    }
  }
}

void HardwareConfig::declareRosParameters(rclcpp::Node & node) const
{
  // RM 已把 URDF <param> 作为 string 注入节点参数；覆盖为 double_array / double
  // 使 rqt 等外部工具可直接调节（IO 线程周期同步回来）。
  auto expose_array = [&node](const std::string & name, const std::vector<double> & value)
  {
    try
    {
      if (node.has_parameter(name))
      {
        node.set_parameter(rclcpp::Parameter(name, value));
      }
      else
      {
        node.declare_parameter(name, value);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(node.get_logger(), "Failed to expose parameter '%s': %s", name.c_str(), e.what());
    }
  };
  auto expose_scalar = [&node](const std::string & name, double value)
  {
    try
    {
      if (node.has_parameter(name))
      {
        node.set_parameter(rclcpp::Parameter(name, value));
      }
      else
      {
        node.declare_parameter(name, value);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(node.get_logger(), "Failed to expose parameter '%s': %s", name.c_str(), e.what());
    }
  };

  expose_array("joint_kp", joint_kp);
  expose_array("joint_kd", joint_kd);
  expose_scalar("gripper_kp", gripper_kp);
  expose_scalar("gripper_kd", gripper_kd);
}

void HardwareConfig::refreshRosParameters(rclcpp::Node & node)
{
  // 读回 rqt 调节的 kp/kd；类型或长度不符时忽略（保持上次有效值）
  auto read_array = [&node](const std::string & name, std::vector<double> & out)
  {
    if (!node.has_parameter(name)) return;
    const auto p = node.get_parameter(name);
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) return;
    const auto & values = p.as_double_array();
    if (values.size() != out.size()) return;
    out = values;
  };
  auto read_scalar = [&node](const std::string & name, double & out)
  {
    if (!node.has_parameter(name)) return;
    const auto p = node.get_parameter(name);
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) return;
    out = p.as_double();
  };

  read_array("joint_kp", joint_kp);
  read_array("joint_kd", joint_kd);
  read_scalar("gripper_kp", gripper_kp);
  read_scalar("gripper_kd", gripper_kd);
}

}  // namespace detail

// ==================== 硬件接口：布局 / 生命周期 / 接口导出 ====================

bool PantheraHardwareInterface::resolveArmLayout()
{
  const size_t joint_count = info_.joints.size();
  auto logger = rclcpp::get_logger(kLoggerName);

  // 每臂 7 关节（6 臂关节 + 夹爪），关节与电机一一对应
  if (joint_count == 0 || joint_count % detail::MotorLayout::kMotorsPerArm != 0)
  {
    RCLCPP_ERROR(
      logger,
      "Unsupported joint layout: expected N*%zu joints (N arms), got %zu",
      detail::MotorLayout::kMotorsPerArm, joint_count);
    return false;
  }
  layout_.arm_count = joint_count / detail::MotorLayout::kMotorsPerArm;
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

  if (!resolveArmLayout())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!config_.load(info_, layout_.arm_count))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  config_.expand(layout_);

  // 状态 / 指令缓冲（controller manager 通过导出的接口指针直接读写）
  const size_t joint_count = info_.joints.size();
  hw_positions_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.assign(joint_count, 0.0);
  hw_commands_efforts_.assign(joint_count, 0.0);

  // kp/kd 暴露为 ROS 参数（rqt 可调），初始值 = URDF hardware 参数
  config_.declareRosParameters(*get_node());

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Panthera hardware initialized | arms=%zu motors=%zu joints=%zu | config=%s | mode=%s",
    layout_.arm_count, layout_.motor_count(), joint_count,
    config_.config_file.c_str(), config_.control_mode.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    robot_ = std::make_unique<hightorque_robot::robot>(
      config_.config_file, config_.usb_select);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to initialize Panthera robot: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!detail::validateMotorCount(kLoggerName, *robot_, layout_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  io_loop_ = std::make_unique<detail::IoLoop>(
    *robot_, layout_, config_, rclcpp::get_logger(kLoggerName), get_clock(), get_node());

  if (!detail::feedback::waitForValid(
      kLoggerName, [this]() { io_loop_->pollOnce(); }, io_loop_->latestPositions(),
      layout_, "on_configure", 3000))
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
  // 与 interfaces.xacro 保持一致：position/velocity/effort（无 kp/kd ——
  // 增益由硬件参数 + ROS 参数管理，不再作为命令接口导出）。
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);

    // 夹爪保持 position-only（AdaptiveGripperController）
    if (!layout_.isGripper(i))
    {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]);
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]);
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!robot_ || !io_loop_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Refusing to activate: hardware not configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!detail::validateMotorCount(kLoggerName, *robot_, layout_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!detail::feedback::waitForValid(
      kLoggerName, [this]() { io_loop_->pollOnce(); }, io_loop_->latestPositions(),
      layout_, "on_activate", 3000))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Refusing to activate: not all arm motors have valid position feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 以当前实测位姿作为初始保持指令（拒绝 SDK 占位 999 / NaN，防止使能后失控）
  const auto & latest = io_loop_->latestPositions();
  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    if (detail::feedback::isValid(latest[joint_index]))
    {
      hw_commands_positions_[joint_index] = latest[joint_index];
    }
    hw_commands_velocities_[joint_index] = 0.0;
    hw_commands_efforts_[joint_index] = 0.0;
  }

  // 播种发送状态（位置种子 / 夹爪跟踪），并作为初始命令快照
  io_loop_->seedCommands(hw_commands_positions_);

  try
  {
    io_loop_->sendActivateHold();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to send activate hold command: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 激活完成后启动后台 IO 线程接管电机指令与状态轮询
  io_loop_->start();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 先停止后台 IO 线程，避免与下面的阻塞式 SDK 调用并发访问 robot_
  if (io_loop_)
  {
    io_loop_->stop();
  }

  if (robot_)
  {
    try
    {
      if (config_.shutdown_return_home)
      {
        detail::moveToShutdownHome(
          *robot_, layout_, config_, rclcpp::get_logger(kLoggerName));
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

hardware_interface::return_type PantheraHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & stop_interfaces)
{
  // 对停止使用的 velocity/effort 命令接口清零，防止旧控制器残留值
  // 被下一个不写该接口的控制器"继承"（如 OCS2 MIX → 纯 position 控制器）。
  // RT 上下文：只写 RT 缓冲，不取锁。
  for (const auto & full_name : stop_interfaces)
  {
    const size_t sep = full_name.find('/');
    if (sep == std::string::npos)
    {
      continue;
    }
    const std::string joint_name = full_name.substr(0, sep);
    const std::string interface_name = full_name.substr(sep + 1);
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (info_.joints[i].name != joint_name)
      {
        continue;
      }
      if (interface_name == hardware_interface::HW_IF_VELOCITY)
      {
        hw_commands_velocities_[i] = 0.0;
      }
      else if (interface_name == hardware_interface::HW_IF_EFFORT)
      {
        hw_commands_efforts_[i] = 0.0;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

void detail::moveToShutdownHome(
  hightorque_robot::robot & robot, const MotorLayout & layout, const HardwareConfig & config,
  const rclcpp::Logger & logger)
{
  // Read current joint positions — commands must start from here and interpolate to home.
  // Never jump the position command to 0 in one step.
  robot.send_get_motor_state_cmd();
  robot.motor_send_cmd();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  robot.send_get_motor_state_cmd();
  robot.motor_send_cmd();

  std::vector<double> start_positions(layout.arm_count * MotorLayout::kArmJointCount, 0.0);
  double max_travel = 0.0;
  for (size_t arm_index = 0; arm_index < layout.arm_count; ++arm_index)
  {
    const size_t motor_base = arm_index * MotorLayout::kMotorsPerArm;
    for (size_t joint_offset = 0; joint_offset < MotorLayout::kArmJointCount; ++joint_offset)
    {
      const auto * state =
        robot.Motors[motor_base + joint_offset]->get_current_motor_state();
      const double start = state->position;
      const double goal = config.shutdown_home_positions[joint_offset];
      start_positions[arm_index * MotorLayout::kArmJointCount + joint_offset] = start;
      max_travel = std::max(max_travel, std::abs(goal - start));
    }
  }

  // Time from farthest joint / commanded speed (capped so launch SIGTERM ~5s still OK).
  // Never shorter than 2s — sub-second return-to-home is unsafe on a real arm.
  constexpr double kMinDurationSec = 2.0;
  constexpr double kMaxDurationSec = 4.0;
  const double speed = std::max(0.05, std::abs(config.shutdown_home_velocity));
  const double duration_sec = std::clamp(
    std::max(max_travel / speed, kMinDurationSec),
    kMinDurationSec,
    std::min(kMaxDurationSec, std::max(kMinDurationSec, config.shutdown_home_timeout_sec)));

  RCLCPP_WARN(
    logger,
    "Shutdown interpolate: max_travel=%.3f rad, duration=%.2fs, speed<=%.2f rad/s, home=%s",
    max_travel, duration_sec, speed, formatVector(config.shutdown_home_positions).c_str());

  if (max_travel <= config.shutdown_home_tolerance)
  {
    RCLCPP_INFO(logger, "Already near shutdown home; entering damping");
    robot.set_stop();
    robot.set_stop();
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

    for (size_t arm_index = 0; arm_index < layout.arm_count; ++arm_index)
    {
      const size_t motor_base = arm_index * MotorLayout::kMotorsPerArm;

      for (size_t joint_offset = 0; joint_offset < MotorLayout::kArmJointCount; ++joint_offset)
      {
        const size_t motor_index = motor_base + joint_offset;
        const double start = start_positions[arm_index * MotorLayout::kArmJointCount + joint_offset];
        const double goal = config.shutdown_home_positions[joint_offset];
        // Interpolated command: start → goal (never a one-shot assignment to 0).
        const double cmd = start + alpha * (goal - start);
        // Feedforward vel from this step's command delta.
        const double step_dt = std::max(0.001, std::chrono::duration<double>(kDt).count());
        const double cmd_vel = std::clamp(
          (d_alpha * (goal - start)) / step_dt, -speed, speed);

        const double max_tqe = std::max(
          motor_index < config.motor_max_torques.size() ? config.motor_max_torques[motor_index] : 10.0,
          80.0);

        robot.Motors[motor_index]->pos_vel_MAXtqe(
          static_cast<float>(cmd),
          static_cast<float>(cmd_vel),
          static_cast<float>(max_tqe));
      }

      const size_t gripper_motor_index = motor_base + MotorLayout::kArmJointCount;
      const double grip_tqe =
        (gripper_motor_index < config.motor_max_torques.size() &&
         config.motor_max_torques[gripper_motor_index] > 0.0)
          ? config.motor_max_torques[gripper_motor_index]
          : 3.0;
      robot.Motors[gripper_motor_index]->pos_vel_MAXtqe(
        0.0f, 0.0f, static_cast<float>(grip_tqe));
    }
    robot.motor_send_cmd();

    if (alpha_lin >= 1.0)
    {
      break;
    }
    std::this_thread::sleep_for(kDt);
  }

  // Hold final home (vel=0) briefly, still as continuous commands at the goal.
  for (int i = 0; i < 15; ++i)
  {
    for (size_t arm_index = 0; arm_index < layout.arm_count; ++arm_index)
    {
      const size_t motor_base = arm_index * MotorLayout::kMotorsPerArm;
      for (size_t joint_offset = 0; joint_offset < MotorLayout::kArmJointCount; ++joint_offset)
      {
        const size_t motor_index = motor_base + joint_offset;
        const double max_tqe = std::max(
          motor_index < config.motor_max_torques.size() ? config.motor_max_torques[motor_index] : 10.0,
          80.0);
        robot.Motors[motor_index]->pos_vel_MAXtqe(
          static_cast<float>(config.shutdown_home_positions[joint_offset]),
          0.0f,
          static_cast<float>(max_tqe));
      }
    }
    robot.motor_send_cmd();
    std::this_thread::sleep_for(kDt);
  }

  RCLCPP_INFO(logger, "Shutdown home interpolation finished; entering damping (set_stop)");
  robot.set_stop();
  robot.set_stop();
}

PantheraHardwareInterface::~PantheraHardwareInterface()
{
  // IoLoop 析构会 join 后台 IO 线程（若还在运行）
  io_loop_.reset();
  robot_.reset();
}

// ==================== 后台 IO 线程 ====================

namespace detail
{

IoLoop::IoLoop(
  hightorque_robot::robot & robot, const MotorLayout & layout, HardwareConfig & config,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock,
  const rclcpp::Node::SharedPtr & node)
: robot_(robot), layout_(layout), config_(config), logger_(logger), clock_(clock), node_(node)
{
  const size_t n = layout.motor_count();
  cmd_positions_.assign(n, 0.0);
  cmd_velocities_.assign(n, 0.0);
  cmd_efforts_.assign(n, 0.0);
  latest_positions_.assign(n, 0.0);
  latest_velocities_.assign(n, 0.0);
  latest_efforts_.assign(n, 0.0);
  last_gripper_command_m_.assign(layout.arm_count, std::numeric_limits<double>::quiet_NaN());
}

IoLoop::~IoLoop()
{
  stop();
}

void IoLoop::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_.joinable())
  {
    return;  // 已在运行
  }
  exit_ = false;
  pending_ = false;
  thread_ = std::thread(&IoLoop::threadLoop, this);
}

void IoLoop::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    exit_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable())
  {
    thread_.join();
  }
}

bool IoLoop::running() const
{
  return thread_.joinable();
}

void IoLoop::threadLoop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (!exit_)
  {
    // 命令到达立即唤醒发送；无命令时按 10ms 兜底轮询反馈
    cv_.wait_for(lock, std::chrono::milliseconds(10), [this] { return exit_ || pending_; });
    if (exit_)
    {
      break;
    }
    pending_ = false;
    lock.unlock();

    refreshGains();       // 周期同步 rqt 调节的 kp/kd
    sendMotorCommands();  // 透传命令；无命令时重发上次指令保持电机活跃
    pollOnce();           // 阻塞查询反馈，刷新 latest_*

    lock.lock();
  }
}

void IoLoop::refreshGains()
{
  if (++gain_refresh_counter_ < 20)
  {
    return;  // 每 ~200ms 一次（循环 10ms）
  }
  gain_refresh_counter_ = 0;
  config_.refreshRosParameters(*node_);
  config_.expand(layout_);
}

void IoLoop::pollOnce()
{
  try
  {
    // 阻塞式查询电机状态（内部异步收包线程维护缓存）
    robot_.send_get_motor_state_cmd();
    robot_.motor_send_cmd();

    // 读取电机状态缓存，换算成关节单位（夹爪 rad -> m，其余 1:1）
    std::vector<double> positions(layout_.motor_count(), 0.0);
    std::vector<double> velocities(layout_.motor_count(), 0.0);
    std::vector<double> efforts(layout_.motor_count(), 0.0);
    for (size_t motor_index = 0; motor_index < layout_.motor_count(); ++motor_index)
    {
      auto * state = robot_.Motors[motor_index]->get_current_motor_state();
      if (layout_.isGripper(motor_index))
      {
        positions[motor_index] = toJointPosition(motor_index, state->position);
        velocities[motor_index] = state->velocity * config_.gripper_rad_to_m;
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
      std::lock_guard<std::mutex> lock(mutex_);
      for (size_t joint_index = 0; joint_index < layout_.motor_count(); ++joint_index)
      {
        if (feedback::isValid(positions[joint_index]))
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
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "Failed to poll robot state: %s", e.what());
  }
}

void IoLoop::seedCommands(const std::vector<double> & joint_positions)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_positions_ = joint_positions;
  cmd_velocities_.assign(cmd_positions_.size(), 0.0);
  cmd_efforts_.assign(cmd_positions_.size(), 0.0);
  last_motor_command_positions_.resize(layout_.motor_count(), 0.0);
  for (size_t i = 0; i < layout_.motor_count(); ++i)
  {
    last_motor_command_positions_[i] = toMotorPosition(i, joint_positions[i]);
  }
  has_last_motor_command_positions_ = true;
  last_gripper_command_m_.assign(layout_.arm_count, std::numeric_limits<double>::quiet_NaN());
  pending_ = true;
}

void IoLoop::sendActivateHold()
{
  // 夹爪由夹爪控制器接管，激活保持只下臂关节
  for (size_t motor_index = 0; motor_index < layout_.motor_count(); ++motor_index)
  {
    if (layout_.isGripper(motor_index))
    {
      continue;
    }

    if (!feedback::isValid(latest_positions_[motor_index]))
    {
      RCLCPP_ERROR(
        logger_,
        "Activate hold skipped motor %zu: invalid feedback %.3f",
        motor_index, latest_positions_[motor_index]);
      continue;
    }
    const double motor_pos = toMotorPosition(motor_index, latest_positions_[motor_index]);

    auto * motor = robot_.Motors[motor_index];
    if (config_.full_control || config_.control_mode == "pd_control")
    {
      motor->pos_vel_tqe_kp_kd(
        motor_pos, 0.0f, 0.0f, config_.motor_kp[motor_index], config_.motor_kd[motor_index]);
    }
    else
    {
      motor->pos_vel_MAXtqe(
        motor_pos, 0.0f, static_cast<float>(config_.motor_max_torques[motor_index]));
    }
  }
  robot_.motor_send_cmd();
}

void IoLoop::publishCommands(
  const std::vector<double> & positions, const std::vector<double> & velocities,
  const std::vector<double> & efforts)
{
  // 非阻塞：把控制器指令快照交给后台 IO 线程发送
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::copy(positions.begin(), positions.end(), cmd_positions_.begin());
    std::copy(velocities.begin(), velocities.end(), cmd_velocities_.begin());
    std::copy(efforts.begin(), efforts.end(), cmd_efforts_.begin());
    pending_ = true;
  }
  cv_.notify_one();
}

void IoLoop::getLatest(
  std::vector<double> & positions, std::vector<double> & velocities,
  std::vector<double> & efforts)
{
  // 非阻塞：从后台 IO 线程维护的共享缓冲拷贝最新反馈
  std::lock_guard<std::mutex> lock(mutex_);
  std::copy(latest_positions_.begin(), latest_positions_.end(), positions.begin());
  std::copy(latest_velocities_.begin(), latest_velocities_.end(), velocities.begin());
  std::copy(latest_efforts_.begin(), latest_efforts_.end(), efforts.begin());
}

double IoLoop::toMotorPosition(size_t index, double joint_value) const
{
  return layout_.isGripper(index) ? joint_value / config_.gripper_rad_to_m : joint_value;
}

double IoLoop::toJointPosition(size_t index, double motor_value) const
{
  return layout_.isGripper(index) ? motor_value * config_.gripper_rad_to_m : motor_value;
}

double IoLoop::toMotorVelocity(size_t index, double joint_value) const
{
  return layout_.isGripper(index) ? joint_value / config_.gripper_rad_to_m : joint_value;
}

void IoLoop::sendMotorCommands()
{
  // 在锁内拷贝命令快照与最新反馈（write() 在 RT 线程写入 cmd_*）
  std::vector<double> cmd_pos, cmd_vel, cmd_eff, cur_pos;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_pos = cmd_positions_;
    cmd_vel = cmd_velocities_;
    cmd_eff = cmd_efforts_;
    cur_pos = latest_positions_;
  }

  // 以最新反馈 / 上次指令为种子，避免把 SDK 占位 999 当目标
  std::vector<double> motor_positions(layout_.motor_count(), 0.0);
  std::vector<double> motor_velocities(layout_.motor_count(), 0.0);
  std::vector<double> motor_efforts(layout_.motor_count(), 0.0);
  for (size_t motor_index = 0; motor_index < layout_.motor_count(); ++motor_index)
  {
    if (feedback::isValid(cur_pos[motor_index]))
    {
      motor_positions[motor_index] = toMotorPosition(motor_index, cur_pos[motor_index]);
    }
    else if (has_last_motor_command_positions_)
    {
      motor_positions[motor_index] = last_motor_command_positions_[motor_index];
    }
  }

  // 合并控制器指令到电机命令（关节与电机一一对应）
  // 速度前馈 = 控制器 velocity 命令透传；未写（0 / 未 claim）即为 0。
  for (size_t joint_index = 0; joint_index < layout_.motor_count(); ++joint_index)
  {
    double cmd_position = cmd_pos[joint_index];
    if (!std::isfinite(cmd_position) || !feedback::isValid(cmd_position))
    {
      // 指令无效（如刚 claim 时）：保持实测位姿
      if (feedback::isValid(cur_pos[joint_index]))
      {
        cmd_position = cur_pos[joint_index];
      }
      else if (has_last_motor_command_positions_)
      {
        cmd_position = toJointPosition(joint_index, last_motor_command_positions_[joint_index]);
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

    double cmd_effort = config_.full_control ? cmd_eff[joint_index] : 0.0;
    if (!std::isfinite(cmd_effort))
    {
      cmd_effort = 0.0;
    }

    motor_positions[joint_index] = toMotorPosition(joint_index, cmd_position);
    motor_velocities[joint_index] = toMotorVelocity(joint_index, cmd_velocity);
    motor_efforts[joint_index] = cmd_effort;
  }

  last_motor_command_positions_ = motor_positions;
  has_last_motor_command_positions_ = true;

  // 阻塞下发：全部臂关节合成一条指令帧
  try
  {
    for (size_t motor_index = 0; motor_index < layout_.motor_count(); ++motor_index)
    {
      if (layout_.isGripper(motor_index))
      {
        continue;  // 夹爪在下方按需单独下发
      }

      auto * motor = robot_.Motors[motor_index];
      if (config_.full_control || config_.control_mode == "pd_control")
      {
        motor->pos_vel_tqe_kp_kd(
          motor_positions[motor_index], motor_velocities[motor_index],
          motor_efforts[motor_index],
          config_.motor_kp[motor_index], config_.motor_kd[motor_index]);
      }
      else
      {
        motor->pos_vel_MAXtqe(
          motor_positions[motor_index], motor_velocities[motor_index],
          config_.motor_max_torques[motor_index]);
      }
    }
    robot_.motor_send_cmd();

    // 夹爪：目标变化时才单独补发一帧（避免双臂总线忽略重复下发）。
    // 判据为精确相等：命令来自同一 double 拷贝，目标不变时 diff 恒为 0。
    for (size_t arm_index = 0; arm_index < layout_.arm_count; ++arm_index)
    {
      const size_t joint_index = arm_index * MotorLayout::kMotorsPerArm + MotorLayout::kArmJointCount;
      const double gripper_pos_m = cmd_pos[joint_index];
      if (!std::isfinite(gripper_pos_m))
      {
        continue;
      }
      if (std::isfinite(last_gripper_command_m_[arm_index]) &&
        last_gripper_command_m_[arm_index] == gripper_pos_m)
      {
        continue;
      }

      auto * motor = robot_.Motors[joint_index];
      const double gripper_pos_rad = toMotorPosition(joint_index, gripper_pos_m);
      const double gripper_vel_joint = config_.full_control
        ? cmd_vel[joint_index]
        : config_.motor_max_velocities[joint_index];
      const double gripper_vel_rad = toMotorVelocity(joint_index, gripper_vel_joint);

      if (config_.full_control)
      {
        motor->pos_vel_tqe_kp_kd(
          gripper_pos_rad, gripper_vel_rad, cmd_eff[joint_index],
          config_.motor_kp[joint_index], config_.motor_kd[joint_index]);
      }
      else
      {
        motor->pos_vel_MAXtqe(
          gripper_pos_rad, gripper_vel_rad, config_.motor_max_torques[joint_index]);
      }
      robot_.motor_send_cmd();
      last_gripper_command_m_[arm_index] = gripper_pos_m;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "Failed to send motor commands: %s", e.what());
  }
}

}  // namespace detail

hardware_interface::return_type PantheraHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 非阻塞：从后台 IO 线程维护的共享缓冲拷贝最新反馈
  if (io_loop_)
  {
    io_loop_->getLatest(hw_positions_, hw_velocities_, hw_efforts_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PantheraHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 非阻塞：把控制器指令快照交给后台 IO 线程发送
  if (io_loop_)
  {
    io_loop_->publishCommands(
      hw_commands_positions_, hw_commands_velocities_, hw_commands_efforts_);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ht_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ht_ros2_control::PantheraHardwareInterface, hardware_interface::SystemInterface)
