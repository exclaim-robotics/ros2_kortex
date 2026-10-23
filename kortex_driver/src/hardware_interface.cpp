// Copyright 2021, PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author Marq Rasmussen marq.rasmussen@picknik.ai
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-06-15
 *
 */
//----------------------------------------------------------------------

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");

std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}
}  // namespace

namespace kortex_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware()
: router_tcp_{
    &transport_tcp_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_{&router_tcp_},
  router_udp_realtime_{
    &transport_udp_realtime_,
    [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); }},
  session_manager_real_time_{&router_udp_realtime_},
  k_api_twist_(nullptr),
  base_{&router_tcp_},
  base_cyclic_{&router_udp_realtime_},
  actuator_config_{&router_tcp_},
  gripper_motor_command_(nullptr),
  gripper_command_max_velocity_(100.0),
  gripper_command_max_force_(100.0),
  servoing_mode_hw_(k_api::Base::ServoingModeInformation()),
  joint_based_controller_running_(false),
  twist_controller_running_(false),
  gripper_controller_running_(false),
  fault_controller_running_(false),
  stop_joint_based_controller_(false),
  stop_twist_controller_(false),
  stop_gripper_controller_(false),
  stop_fault_controller_(false),
  start_joint_based_controller_(false),
  start_twist_controller_(false),
  start_gripper_controller_(false),
  start_fault_controller_(false),
  start_joint_velocity_control_(false),
  start_joint_position_control_(false),
  start_joint_effort_control_(false),
  use_velocity_command_(false),
  use_effort_command_(false),
  first_pass_(true),
  gripper_joint_name_(""),
  use_internal_bus_gripper_comm_(false)
{
  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

CallbackReturn KortexMultiInterfaceHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = params.hardware_info;
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  if (robot_ip.empty())
  {
    RCLCPP_ERROR(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip.c_str());
  }
  // Username to log into the robot controller
  std::string username = info_.hardware_parameters["username"];
  if (username.empty())
  {
    RCLCPP_ERROR(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username.c_str());
  }
  // Password to log into the robot controller
  std::string password = info_.hardware_parameters["password"];
  if (password.empty())
  {
    RCLCPP_ERROR(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  int port = std::stoi(info_.hardware_parameters["port"]);
  if (port <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Port used '%d'", port);
  }
  int port_realtime = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Realtime port used '%d'", port_realtime);
  }

  int session_inactivity_timeout =
    std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout);
  }
  int connection_inactivity_timeout =
    std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout);
  }
  // gripper joint name
  gripper_joint_name_ = info_.hardware_parameters["gripper_joint_name"];
  if (gripper_joint_name_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  }

  gripper_command_max_velocity_ = std::stod(info_.hardware_parameters["gripper_max_velocity"]);
  gripper_command_max_force_ = std::stod(info_.hardware_parameters["gripper_max_force"]);

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  // connections
  transport_tcp_.connect(robot_ip, port);
  transport_udp_realtime_.connect(robot_ip, port_realtime);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(session_inactivity_timeout);  // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(
    connection_inactivity_timeout);  // (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(LOGGER, "Creating session for communication");
  session_manager_.CreateSession(create_session_info);
  session_manager_real_time_.CreateSession(create_session_info);
  api_connected_ = true;
  RCLCPP_INFO(LOGGER, "Session created");

  // reset faults on activation, go back to low level servoing after
  {
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = Kinova::Api::Base::SINGLE_LEVEL_SERVOING;

    try
    {
      base_.ClearFaults();
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }

    // low level servoing on startup
    servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
    arm_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;
    base_.SetServoingMode(servoing_mode_hw_);
  }

  // initialize kortex api twist commandd
  {
    k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    // command.set_duration = execute time (milliseconds) according to the api ->
    // (not implemented yet)
    // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
    // k_api_twist_command_.set_duration(0);
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  actuator_count_ = base_.GetActuatorCount().count();
  RCLCPP_INFO(LOGGER, "Actuator count reported by robot is '%lu'", actuator_count_);

  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_joints_control_level_.resize(
    actuator_count_, integration_lvl_t::UNDEFINED);  // start in undefined
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  // set size of the twist interface
  twist_commands_.resize(6, 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  // Torque mode limits. Arm joints are info_.joints minus the gripper, in actuator order (the
  // same pairing export_*_interfaces uses). A joint without a URDF effort limit cannot be put
  // into torque mode: prepare_command_mode_switch refuses it.
  effort_limits_.assign(actuator_count_, std::numeric_limits<double>::infinity());
  last_effort_commands_.assign(actuator_count_, 0.0);
  torque_integrals_.assign(actuator_count_, 0.0);
  joint_calibration_.assign(actuator_count_, JointCalibration());
  {
    std::size_t arm_index = 0;
    for (const auto & joint : info_.joints)
    {
      if (joint.name == gripper_joint_name_ || arm_index >= actuator_count_)
      {
        continue;
      }
      arm_joint_names_.push_back(joint.name);
      const auto limits = info_.limits.find(joint.name);
      if (
        limits != info_.limits.end() && limits->second.has_effort_limits &&
        std::isfinite(limits->second.max_effort) && limits->second.max_effort > 0.0)
      {
        effort_limits_[arm_index] = limits->second.max_effort;
      }
      RCLCPP_INFO(
        LOGGER, "Torque mode limit for '%s': %.2f N*m", joint.name.c_str(),
        effort_limits_[arm_index]);
      // Kinematic calibration (joint_calibration.hpp): readings and position commands only,
      // never the actuators' own zero. Inert unless the joint carries the params.
      try
      {
        joint_calibration_[arm_index] = parseJointCalibration(joint.parameters);
      }
      catch (const std::exception & ex)
      {
        RCLCPP_ERROR(LOGGER, "Joint '%s': %s", joint.name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }
      if (joint_calibration_[arm_index].active())
      {
        RCLCPP_INFO(
          LOGGER,
          "Joint calibration for '%s': offset %+.6f rad (%+.3f deg), compliance %.6g rad/(N*m)",
          joint.name.c_str(), joint_calibration_[arm_index].offset,
          KortexMathUtil::toDeg(joint_calibration_[arm_index].offset),
          joint_calibration_[arm_index].compliance);
      }
      ++arm_index;
    }
  }
  if (std::none_of(
        joint_calibration_.begin(), joint_calibration_.end(),
        [](const JointCalibration & c) { return c.active(); }))
  {
    RCLCPP_INFO(LOGGER, "Joint calibration: none (raw joint angles)");
  }
  else
  {
    RCLCPP_WARN(
      LOGGER,
      "Joint calibration applied to positions and position commands. Twist mode (the arm's own "
      "Cartesian control) is NOT corrected; the arm's joint limits stay raw.");
  }
  // Optional torque mode parameters; the defaults are in the header.
  const auto param = [this](const std::string & name) -> std::string
  {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? std::string() : it->second;
  };
  try
  {
    if (!param("effort_command_max_step").empty())
    {
      effort_command_max_step_ = std::stod(param("effort_command_max_step"));
    }
    if (!param("effort_check_tolerance").empty())
    {
      effort_check_tolerance_ = std::stod(param("effort_check_tolerance"));
    }
    if (!param("effort_check_cycles").empty())
    {
      effort_check_cycles_ = std::stoi(param("effort_check_cycles"));
    }
    if (!param("effort_startup_check_ms").empty())
    {
      effort_startup_check_ms_ = std::stoi(param("effort_startup_check_ms"));
    }
    if (!param("cyclic_async").empty())
    {
      cyclic_async_ = param("cyclic_async") == "true" || param("cyclic_async") == "True";
    }
    if (!param("feedback_timeout_ms").empty())
    {
      feedback_timeout_ = std::chrono::milliseconds(std::stoi(param("feedback_timeout_ms")));
    }
    torque_integral_ = parseTorqueIntegral(param("torque_integral"));
    if (!param("torque_control_mode").empty())
    {
      const std::string mode = param("torque_control_mode");
      if (mode == "torque")
      {
        torque_control_mode_ = k_api::ActuatorConfig::ControlMode::TORQUE;
      }
      else if (mode == "torque_high_velocity")
      {
        torque_control_mode_ = k_api::ActuatorConfig::ControlMode::TORQUE_HIGH_VELOCITY;
      }
      else
      {
        throw std::invalid_argument(
          "torque_control_mode '" + mode + "': expected torque or torque_high_velocity");
      }
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(LOGGER, "Bad torque mode parameter: %s", ex.what());
    return CallbackReturn::ERROR;
  }
  if (
    !(effort_command_max_step_ > 0.0) || !(effort_check_tolerance_ > 0.0) ||
    effort_check_cycles_ < 1 || effort_startup_check_ms_ < 0)
  {
    RCLCPP_ERROR(
      LOGGER,
      "Torque mode parameters out of range: effort_command_max_step and effort_check_tolerance "
      "must be > 0, effort_check_cycles >= 1, effort_startup_check_ms >= 0!");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    LOGGER,
    "Torque mode (%s): commands change by at most %.3f N*m per cycle; sign check %.2f N*m over "
    "%d cycles, %d ms while holding at the switch",
    k_api::ActuatorConfig::ControlMode_Name(torque_control_mode_).c_str(),
    effort_command_max_step_, effort_check_tolerance_, effort_check_cycles_,
    effort_startup_check_ms_);
  if (torque_integral_.on())
  {
    RCLCPP_INFO(
      LOGGER,
      "Torque integral on: %.3g/s of (command - measured torque), leaking at %.3g/s, at most "
      "%.2f N*m",
      torque_integral_.gain, torque_integral_.leak, torque_integral_.limit);
  }
  if (feedback_timeout_.count() <= 0)
  {
    RCLCPP_ERROR(LOGGER, "feedback_timeout_ms must be > 0!");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    LOGGER, "Cyclic exchange: %s, feedback timeout %ld ms",
    cyclic_async_ ? "non-blocking (Refresh_callback)" : "blocking (Refresh)",
    static_cast<long>(feedback_timeout_.count()));

  // "1 1 1 1 1 1" (or comma separated): +1 where kortex's torque_joint command already is in
  // the ROS convention, -1 where it is reversed. Kinova's example 108 commands the negated
  // reported torque to hold, which makes it +1 on every joint; that is the default, but it is
  // unmeasured until the per-joint sign test has been run on this arm.
  effort_command_signs_.assign(actuator_count_, 1.0);
  {
    std::string signs = param("effort_command_signs");
    std::replace(signs.begin(), signs.end(), ',', ' ');
    std::istringstream stream(signs);
    std::vector<double> parsed;
    for (double sign; stream >> sign;)
    {
      parsed.push_back(sign);
    }
    if (!signs.empty() && signs.find_first_not_of(' ') != std::string::npos)
    {
      if (
        !stream.eof() || parsed.size() != actuator_count_ ||
        std::any_of(parsed.begin(), parsed.end(), [](double v) { return v != 1.0 && v != -1.0; }))
      {
        RCLCPP_ERROR(
          LOGGER, "effort_command_signs must be %zu values, each 1 or -1; got '%s'",
          actuator_count_, param("effort_command_signs").c_str());
        return CallbackReturn::ERROR;
      }
      effort_command_signs_ = parsed;
      effort_command_signs_set_ = true;
    }
  }
  effort_disagreement_cycles_.assign(actuator_count_, 0);
  torque_joints_.assign(actuator_count_, false);
  start_torque_joints_.assign(actuator_count_, false);

  if (
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
  {
    use_internal_bus_gripper_comm_ = true;
    RCLCPP_INFO(LOGGER, "Using internal bus communication for gripper!");
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  // state interface which reports if robot is faulted
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_velocity", &gripper_speed_command_));
      gripper_speed_command_ = gripper_command_max_velocity_;
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "set_gripper_max_effort", &gripper_force_command_));
      gripper_force_command_ = gripper_command_max_force_;
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }
  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
  }

  // register twist command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_commands_[3]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_commands_[4]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_commands_[5]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "reset_fault", "async_success", &reset_fault_async_success_));

  return command_interfaces;
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = false;
  start_joint_velocity_control_ = start_joint_position_control_ = start_joint_effort_control_ =
    false;
  std::fill(start_torque_joints_.begin(), start_torque_joints_.end(), false);

  // sleep to ensure all outgoing write commands have finished
  block_write = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  start_modes_.clear();
  stop_modes_.clear();

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto & key : stop_interfaces)
  {
    for (auto & joint : info_.joints)
    {
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
        joint.name == gripper_joint_name_)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_GRIPPER);
        continue;
      }
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
        joint.name == gripper_joint_name_)
      {
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL);
      }
      // effort is a joint based controller too: the actuators go to TORQUE mode
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS_VEL);
      }
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_TWIST);
    }
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_FAULT_CTRL);
    }
  }

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto & key : start_interfaces)
  {
    for (auto & joint : info_.joints)
    {
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
        joint.name == gripper_joint_name_)
      {
        start_modes_.emplace_back(StopStartInterface::START_GRIPPER);
        continue;
      }
      if (
        key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
        joint.name == gripper_joint_name_)
      {
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL);
        start_joint_position_control_ = true;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL);
        start_joint_velocity_control_ = true;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS_VEL);
        start_joint_effort_control_ = true;
        // only the claimed joints go to TORQUE mode, the others hold their position
        const auto arm_joint =
          std::find(arm_joint_names_.begin(), arm_joint_names_.end(), joint.name);
        if (arm_joint != arm_joint_names_.end())
        {
          start_torque_joints_[static_cast<std::size_t>(arm_joint - arm_joint_names_.begin())] =
            true;
        }
      }
    }
    if (
      (key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") ||
      (key == "tcp/twist.linear.z") || (key == "tcp/twist.angular.x") ||
      (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      start_modes_.emplace_back(StopStartInterface::START_TWIST);
    }
    if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    {
      start_modes_.emplace_back(StopStartInterface::START_FAULT_CTRL);
    }
  }

  // prepare flags for performing the switch
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_POS_VEL) !=
      stop_modes_.end())
  {
    stop_joint_based_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_TWIST) !=
      stop_modes_.end())
  {
    stop_twist_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_GRIPPER) !=
      stop_modes_.end())
  {
    stop_gripper_controller_ = true;
  }
  if (
    !stop_modes_.empty() &&
    std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_FAULT_CTRL) !=
      stop_modes_.end())
  {
    stop_fault_controller_ = true;
  }

  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_POS_VEL) !=
     start_modes_.end()))
  {
    start_joint_based_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_TWIST) !=
      start_modes_.end())
  {
    start_twist_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_GRIPPER) !=
     start_modes_.end()))
  {
    start_gripper_controller_ = true;
  }
  if (
    !start_modes_.empty() &&
    (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_FAULT_CTRL) !=
     start_modes_.end()))
  {
    start_fault_controller_ = true;
  }

  // handle exclusiveness between twist and joint based controller
  if (twist_controller_running_ && start_joint_based_controller_ && !stop_twist_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start joint based controller while twist controller is running!");
    return hardware_interface::return_type::ERROR;
  }
  if (joint_based_controller_running_ && start_twist_controller_ && !stop_joint_based_controller_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start twist controller while joint based controller is running!");
    return hardware_interface::return_type::ERROR;
  }

  // position, velocity and effort joint command interfaces drive the same actuators with
  // conflicting control modes, so only one joint based controller (position OR velocity OR
  // effort) may run at a time
  if (
    static_cast<int>(start_joint_position_control_) +
      static_cast<int>(start_joint_velocity_control_) +
      static_cast<int>(start_joint_effort_control_) >
    1)
  {
    RCLCPP_ERROR(
      LOGGER,
      "Can't start a joint based controller claiming more than one of the position, velocity "
      "and effort interfaces!");
    return hardware_interface::return_type::ERROR;
  }
  if (
    joint_based_controller_running_ && !stop_joint_based_controller_ &&
    ((start_joint_velocity_control_ && !use_velocity_command_) ||
     (start_joint_effort_control_ && !use_effort_command_) ||
     (start_joint_position_control_ && (use_velocity_command_ || use_effort_command_))))
  {
    RCLCPP_ERROR(
      LOGGER,
      "Can't start a joint controller of a different kind (position, velocity, effort) while "
      "another joint controller is running! Deactivate the running controller first.");
    return hardware_interface::return_type::ERROR;
  }
  if (start_joint_effort_control_)
  {
    // Unclamped torque on a real arm is not an option, and neither is switching a faulted one.
    bool unlimited = false;
    for (std::size_t i = 0; i < actuator_count_; i++)
    {
      unlimited |= start_torque_joints_[i] && std::isinf(effort_limits_[i]);
    }
    if (unlimited)
    {
      RCLCPP_ERROR(
        LOGGER, "Can't start an effort controller: a joint has no URDF effort limit to clamp to!");
      return hardware_interface::return_type::ERROR;
    }
    if (in_fault_ != 0.0)
    {
      RCLCPP_ERROR(LOGGER, "Can't start an effort controller while the robot is faulted!");
      return hardware_interface::return_type::ERROR;
    }
  }

  return ret_val;
}

return_type KortexMultiInterfaceHardware::perform_command_mode_switch(
  const vector<std::string> & /*start_interfaces*/, const vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_joint_based_controller_)
  {
    joint_based_controller_running_ = false;
    // if the stopped controller was commanding velocities, put the actuators back into the
    // default position control mode so a following position command does not surprise the user
    if (use_velocity_command_)
    {
      setActuatorsControlMode(k_api::ActuatorConfig::ControlMode::POSITION);
      use_velocity_command_ = false;
    }
    if (use_effort_command_)
    {
      leaveTorqueMode();
    }
    holdMeasuredPosition();
    std::fill(arm_commands_velocities_.begin(), arm_commands_velocities_.end(), 0.0);
  }
  if (stop_twist_controller_)
  {
    twist_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (stop_gripper_controller_)
  {
    gripper_controller_running_ = false;
    gripper_command_position_ = gripper_position_;
  }
  if (stop_fault_controller_)
  {
    fault_controller_running_ = false;
  }

  if (start_joint_based_controller_)
  {
    servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
    twist_controller_running_ = false;
    holdMeasuredPosition();
    std::fill(arm_commands_velocities_.begin(), arm_commands_velocities_.end(), 0.0);
    joint_based_controller_running_ = true;
    // select the actuator control mode matching the claimed command interface
    if (start_joint_velocity_control_)
    {
      setActuatorsControlMode(k_api::ActuatorConfig::ControlMode::VELOCITY);
      use_velocity_command_ = true;
    }
    else if (start_joint_effort_control_)
    {
      use_velocity_command_ = false;
      torque_joints_ = start_torque_joints_;
      if (!enterTorqueMode())
      {
        // enterTorqueMode has put the arm back into POSITION mode, holding
        joint_based_controller_running_ = false;
        ret_val = hardware_interface::return_type::ERROR;
      }
    }
    else
    {
      setActuatorsControlMode(k_api::ActuatorConfig::ControlMode::POSITION);
      use_velocity_command_ = false;
    }
    // refresh feedback
    feedback_ = base_cyclic_.RefreshFeedback();
    acceptBlockingFeedback();
  }
  if (start_twist_controller_)
  {
    servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_.SetServoingMode(servoing_mode_hw_);
    arm_mode_ = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
    joint_based_controller_running_ = false;
    twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    twist_controller_running_ = true;
  }
  if (start_gripper_controller_)
  {
    gripper_command_position_ = gripper_position_;
    gripper_controller_running_ = true;
  }
  if (start_fault_controller_)
  {
    fault_controller_running_ = true;
  }

  // reset auxiliary switching booleans
  stop_joint_based_controller_ = stop_twist_controller_ = stop_fault_controller_ =
    stop_gripper_controller_ = false;
  start_joint_based_controller_ = start_twist_controller_ = start_fault_controller_ =
    start_gripper_controller_ = false;
  start_joint_velocity_control_ = start_joint_position_control_ = start_joint_effort_control_ =
    false;

  start_modes_.clear();
  stop_modes_.clear();

  block_write = false;

  return ret_val;
}

CallbackReturn KortexMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");

  // Coming back from a deactivation needs no reconnect: on_deactivate leaves
  // the API connection up and only drops the arm to SINGLE_LEVEL_SERVOING, so
  // activation is simply the reverse of that.
  //
  // Reconnecting instead was tried and does not work. The transports can be
  // reconnected and the routers reactivated, but nothing re-registers the
  // router's onMessage() callback on the transport, so the receive thread
  // invokes an empty std::function and throws std::bad_function_call from
  // TransportClientTcp::receiveThread -- on a detached thread, which aborts
  // ros2_control_node rather than failing the transition. The callback is
  // registered in RouterClient's constructor and there is no public way to
  // re-register it, so the connection must simply not be torn down.
  //
  // Clear faults first, as on_init does: re-acquiring the arm is how an operator
  // recovers it, so it should recover it.
  try
  {
    base_.ClearFaults();
  }
  catch (k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "ClearFaults on activation: " << ex.what());
  }

  // Setting the mode unconditionally matches on_init, which also ends in
  // LOW_LEVEL_SERVOING every time.
  servoing_mode_hw_.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode_hw_);
  arm_mode_ = Kinova::Api::Base::LOW_LEVEL_SERVOING;

  // add_actuators() appends, and on_activate can now run more than once, so
  // without this the repeated field grows by actuator_count_ per activation.
  base_command_.clear_actuators();

  auto base_feedback = base_cyclic_.RefreshFeedback();

  // Add each actuator to the base_command_ and set the command to its current position
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    base_command_.add_actuators()->set_position(base_feedback.actuators(i).position());
  }

  // Initialize gripper
  float gripper_initial_position =
    base_feedback.interconnect().gripper_feedback().motor()[0].position();
  RCLCPP_INFO(LOGGER, "Gripper initial position is '%f'.", gripper_initial_position);

  // to radians
  gripper_command_position_ = gripper_initial_position / 100.0 * 0.81;

  // Initialize interconnect command to current gripper position.
  base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
  // Clear before adding: add_motor_cmd() appends, and on_activate can now run
  // more than once, so without this the repeated field grows by one entry per
  // activation and the gripper command we hold is not the only one sent.
  base_command_.mutable_interconnect()->mutable_gripper_command()->clear_motor_cmd();
  gripper_motor_command_ =
    base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
  gripper_motor_command_->set_position(gripper_initial_position);  // % position
  gripper_motor_command_->set_velocity(gripper_speed_command_);    // % speed
  gripper_motor_command_->set_force(gripper_force_command_);       // % force

  // Send a first frame
  base_feedback = base_cyclic_.Refresh(base_command_);
  acceptBlockingFeedback();
  // Set some default values
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (std::isnan(arm_positions_[i]))
    {
      arm_positions_[i] = calibratedPosition(base_feedback, i);  // rad
    }
    if (std::isnan(arm_velocities_[i]))
    {
      arm_velocities_[i] = 0;
    }
    if (std::isnan(arm_efforts_[i]))
    {
      arm_efforts_[i] = 0;
    }
    if (std::isnan(arm_commands_positions_[i]))
    {
      arm_commands_positions_[i] = calibratedPosition(base_feedback, i);  // rad
    }
    if (std::isnan(arm_commands_velocities_[i]))
    {
      arm_commands_velocities_[i] = 0;
    }
    if (std::isnan(arm_commands_efforts_[i]))
    {
      arm_commands_efforts_[i] = 0;
    }
    arm_joints_control_level_[i] = integration_lvl_t::UNDEFINED;
  }

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating KortexMultiInterfaceHardware...");

  // make sure the actuators are left in the default position control mode
  if (use_velocity_command_)
  {
    setActuatorsControlMode(k_api::ActuatorConfig::ControlMode::POSITION);
    use_velocity_command_ = false;
  }
  if (use_effort_command_)
  {
    leaveTorqueMode();
  }

  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set back the servoing mode to Single Level Servoing
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode);

  // The API connection is deliberately left UP, and the sessions open.
  //
  // Deactivating is a handover, not a shutdown: SINGLE_LEVEL_SERVOING above is
  // what frees the arm for the teach pendant and admittance mode, and that is
  // all the caller wants. Closing the sessions and dropping the transports as
  // well made deactivation a one-way door, because the connection cannot be
  // rebuilt from on_activate (see the note there).
  //
  // The teardown now lives in on_cleanup/on_shutdown, so a real shutdown still
  // closes its sessions politely instead of leaving the arm holding them until
  // they time out.

  // k_api_twist_ and gripper_motor_command_ used to be deleted here. Both
  // point INTO protobuf messages that own them -- mutable_twist() and
  // add_motor_cmd() respectively -- so deleting them was undefined behaviour,
  // and it orphaned k_api_twist_, which is assigned only in on_init, leaving
  // sendTwistCommand() writing through a dangling pointer after any
  // deactivate/activate cycle. The owning messages are members and free their
  // own storage.

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

void KortexMultiInterfaceHardware::disconnectApi()
{
  if (!api_connected_)
  {
    return;
  }
  api_connected_ = false;

  RCLCPP_INFO(LOGGER, "Closing Kortex sessions and disconnecting.");
  try
  {
    session_manager_.CloseSession();
    session_manager_real_time_.CloseSession();
  }
  catch (const std::exception & ex)
  {
    // Worth logging but not worth failing the transition: we are on the way
    // out either way, and an unreachable arm cannot be told politely.
    RCLCPP_WARN_STREAM(LOGGER, "While closing the Kortex sessions: " << ex.what());
  }

  router_tcp_.SetActivationStatus(false);
  transport_tcp_.disconnect();
  router_udp_realtime_.SetActivationStatus(false);
  transport_udp_realtime_.disconnect();
}

CallbackReturn KortexMultiInterfaceHardware::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  disconnectApi();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  disconnectApi();
  return CallbackReturn::SUCCESS;
}

return_type KortexMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (first_pass_)
  {
    first_pass_ = false;
    feedback_ = base_cyclic_.RefreshFeedback();
    acceptBlockingFeedback();
  }

  if (cyclic_async_)
  {
    // Take the newest reply if there is one. try_to_lock: the receive thread holds the lock only
    // to copy a reply in, and waiting for it is exactly what this replaces -- on a miss, the
    // reply is taken next cycle.
    std::unique_lock<std::mutex> lock(replies_->mutex, std::try_to_lock);
    if (lock.owns_lock() && replies_->fresh)
    {
      feedback_.Swap(&replies_->feedback);
      replies_->fresh = false;
    }
  }
  // A silent arm must not leave the controllers running on frozen feedback -- in_fault_ below
  // is read from it too. (In blocking mode Refresh itself waits, and throws after 3 s.)
  const std::int64_t unanswered_since = replies_->unanswered_since_ns.load();
  const auto silent = std::chrono::nanoseconds(steadyNowNs() - unanswered_since);
  if (unanswered_since != 0 && silent > feedback_timeout_)
  {
    RCLCPP_ERROR(
      LOGGER, "No feedback from the arm for %ld ms (feedback_timeout_ms %ld)",
      static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(silent).count()),
      static_cast<long>(feedback_timeout_.count()));
    return return_type::ERROR;
  }

  // Whether the arm is faulted is the arm's own call (ARMSTATE_IN_FAULT), plus any live
  // actuator fault. The base's fault banks are not part of it: they keep a fault latched after
  // the arm has recovered and ClearFaults does not reset it (a JOINT_FAULT bit, 2026-10-14:
  // arm state normal, every actuator clean, the Kinova's own admittance working), and counting
  // it held every command back until a reconnect, with nothing in the log. They are logged.
  in_fault_ = (feedback_.base().active_state() == Kinova::Api::Common::ArmState::ARMSTATE_IN_FAULT);
  logBankChanges();

  // read gripper state
  readGripperPosition();

  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    // read torque
    arm_efforts_[i] = REPORTED_TORQUE_SIGN * feedback_.actuators(i).torque();  // N*m, ROS sign
    // read velocity
    arm_velocities_[i] = KortexMathUtil::toRad(feedback_.actuators(i).velocity());  // rad/sec
    // read position, calibrated (joint_calibration_; the raw angle when there is none)
    arm_positions_[i] = calibratedPosition(feedback_, i);  // rad

    in_fault_ += (feedback_.actuators(i).fault_bank_a() + feedback_.actuators(i).fault_bank_b());

    // TODO(livanov93): separate warnings into another variable to expose it via fault controller
    //       feedback_.actuators(i).warning_bank_a() + feedback_.actuators(i).warning_bank_b());
  }


  // add mode that can't be easily reached
  in_fault_ += (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_READY);

  return return_type::OK;
}

namespace
{
template <typename NameFn>
std::string bitNames(uint32_t bits, NameFn name)
{
  std::string out;
  for (uint32_t b = 1; b != 0; b <<= 1)
  {
    if (bits & b)
    {
      std::string n = name(static_cast<int>(b));
      out += (out.empty() ? "" : " ") + (n.empty() ? std::to_string(b) : n);
    }
  }
  return out.empty() ? "none" : out;
}
}  // namespace

void KortexMultiInterfaceHardware::logBankChanges()
{
  // Rare (a fault or warning appearing or clearing), so logging here does not load the loop.
  std::vector<uint32_t> banks{feedback_.base().fault_bank_a(), feedback_.base().fault_bank_b(),
                              feedback_.base().warning_bank_a(), feedback_.base().warning_bank_b()};
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    banks.push_back(feedback_.actuators(i).fault_bank_a());
    banks.push_back(feedback_.actuators(i).warning_bank_a());
  }
  if (banks == logged_banks_)
  {
    return;
  }
  if (logged_banks_.size() != banks.size())
  {
    logged_banks_.assign(banks.size(), 0);
  }
  auto base_name = [](int v)
  { return k_api::Base::SafetyIdentifier_Name(static_cast<k_api::Base::SafetyIdentifier>(v)); };
  auto act_name = [](int v)
  {
    return k_api::ActuatorConfig::SafetyIdentifierBankA_Name(
      static_cast<k_api::ActuatorConfig::SafetyIdentifierBankA>(v));
  };
  const auto state = k_api::Common::ArmState_Name(feedback_.base().active_state());
  const char * kind[] = {"faults A", "faults B", "warnings A", "warnings B"};
  for (std::size_t k = 0; k < 4; k++)
  {
    if (banks[k] != logged_banks_[k])
    {
      RCLCPP_WARN(
        LOGGER, "Kinova base %s: %s (arm state %s)", kind[k], bitNames(banks[k], base_name).c_str(),
        state.c_str());
    }
  }
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    for (std::size_t k = 0; k < 2; k++)
    {
      const std::size_t j = 4 + 2 * i + k;
      if (banks[j] != logged_banks_[j])
      {
        RCLCPP_WARN(
          LOGGER, "Kinova joint_%zu %s: %s (arm state %s)", i + 1, k ? "warnings" : "faults",
          bitNames(banks[j], act_name).c_str(), state.c_str());
      }
    }
  }
  logged_banks_ = banks;
}

double KortexMultiInterfaceHardware::calibratedPosition(
  const k_api::BaseCyclic::Feedback & feedback, std::size_t i) const
{
  const auto & actuator = feedback.actuators(static_cast<int>(i));
  const double raw = KortexMathUtil::toRad(actuator.position());  // rad
  const double effort = REPORTED_TORQUE_SIGN * actuator.torque();  // N*m, ROS sign
  return KortexMathUtil::wrapRadiansFromMinusPiToPi(joint_calibration_[i].calibrated(raw, effort));
}

void KortexMultiInterfaceHardware::readGripperPosition()
{
  // max joint angle = 0.81 for robotiq_2f_85
  // TODO(anyone) read in as parameter from kortex_controllers.yaml
  if (use_internal_bus_gripper_comm_)
  {
    gripper_position_ =
      feedback_.interconnect().gripper_feedback().motor()[0].position() / 100.0 * 0.81;  // rad
  }
}

return_type KortexMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // The torque integral's step: a late cycle integrates no more than 10 ms.
  write_period_s_ = std::clamp(period.seconds(), 0.0, 0.01);
  if (block_write)
  {
    cyclicRefresh(false);
    return return_type::OK;
  }

  if (!std::isnan(reset_fault_cmd_) && fault_controller_running_)
  {
    try
    {
      // change servoing mode first
      servoing_mode_hw_.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base_.SetServoingMode(servoing_mode_hw_);
      // apply emergency stop - twice to make it sure as calling it once appeared to be unreliable
      // (detected by testing)
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      // clear faults
      base_.ClearFaults();
      // back to original servoing mode
      if (
        arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING ||
        arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        servoing_mode_hw_.set_servoing_mode(arm_mode_);
        base_.SetServoingMode(servoing_mode_hw_);
      }
      // A fault clears in POSITION mode, holding. The effort controller stays active but its
      // commands are ignored until it is re-activated: torque mode is never resumed silently.
      if (use_effort_command_)
      {
        leaveTorqueMode();
        RCLCPP_WARN(
          LOGGER,
          "Fault reset left torque mode: the arm holds position. Re-activate the effort "
          "controller to resume.");
      }
      reset_fault_async_success_ = 1.0;
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
      reset_fault_async_success_ = 0.0;
    }
    catch (...)
    {
      reset_fault_async_success_ = 0.0;
    }
    reset_fault_cmd_ = NO_CMD;
  }

  if (in_fault_ == 0.0)
  {
    if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
    {
      // Twist controller active
      if (twist_controller_running_)
      {
        // twist control
        sendTwistCommand();
      }
      else
      {
        // Keep alive mode - no controller active
        RCLCPP_DEBUG(LOGGER, "No controller active in SINGLE_LEVEL_SERVOING mode!");
      }

      // gripper control
      sendGripperCommand(
        arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
      // read after write in twist mode
      cyclicRefresh(false);
    }
    else if (
      (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
      (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL))
    {
      // Per joint controller active

      // gripper control
      sendGripperCommand(
        arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);

      if (joint_based_controller_running_)
      {
        // send commands to the joints
        sendJointCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        cyclicRefresh(false);
        // RCLCPP_DEBUG(LOGGER, "No controller active in LOW_LEVEL_SERVOING mode !");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      cyclicRefresh(false);
      RCLCPP_DEBUG(
        LOGGER,
        "Fault was not recognized on the robot but combination of Control Mode and Active State "
        "are not supported!");
    }
  }
  else
  {
    // this is needed when the robot was faulted
    // so we can internally conclude it is not faulted anymore
    cyclicRefresh(false);
  }

  return return_type::OK;
}

void KortexMultiInterfaceHardware::prepareCommands()
{  // update the command for each joint
  for (size_t i = 0; i < actuator_count_; i++)
  {
    // set command per joint
    // The command is in calibrated angles; the arm takes raw ones. The compliance term uses the
    // torque the joint carries now (the last read), so a joint holding a load is commanded where
    // its encoder will read once deflected.
    cmd_degrees_tmp_ = static_cast<float>(
      KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(
        joint_calibration_[i].raw(arm_commands_positions_[i], arm_efforts_[i]))));
    cmd_vel_tmp_ = static_cast<float>(KortexMathUtil::toDeg(arm_commands_velocities_[i]));

    if (use_effort_command_ && torque_joints_[i])
    {
      // In TORQUE control mode (Kinova example 108) the position field also follows the
      // measurement: the following error cannot trip while the torque moves the arm, but does if
      // these commands stop arriving and the joint runs away under its last torque, which faults
      // it into holding. A non-finite command repeats the last one; every command is rate
      // limited and clamped (see effort_command_max_step_), then converted to kortex's sign.
      base_command_.mutable_actuators(static_cast<int>(i))
        ->set_position(feedback_.actuators(static_cast<int>(i)).position());
      double effort = arm_commands_efforts_[i];
      if (!std::isfinite(effort))
      {
        effort = last_effort_commands_[i];
      }
      effort = std::clamp(
        effort, last_effort_commands_[i] - effort_command_max_step_,
        last_effort_commands_[i] + effort_command_max_step_);
      effort = std::clamp(effort, -effort_limits_[i], effort_limits_[i]);
      last_effort_commands_[i] = effort;
      // What the actuator leaves of the command, by the torque sensor, accumulated and added
      // (torque_integral.hpp); the command itself when off.
      const double sent = torque_integral_.step(
        torque_integrals_[i], effort, arm_efforts_[i], effort_limits_[i], write_period_s_);
      base_command_.mutable_actuators(static_cast<int>(i))
        ->set_torque_joint(static_cast<float>(effort_command_signs_[i] * sent));
    }
    else if (use_velocity_command_)
    {
      // In VELOCITY control mode the actuator tracks the velocity command (deg/s). The position
      // field is kept synced to the latest measured position so that the following error does not
      // trigger and the joint holds when the control mode switches back to position.
      base_command_.mutable_actuators(static_cast<int>(i))
        ->set_position(feedback_.actuators(static_cast<int>(i)).position());
      base_command_.mutable_actuators(static_cast<int>(i))->set_velocity(cmd_vel_tmp_);
    }
    else
    {
      // POSITION mode; in torque mode, the joints the effort controller does not claim, holding
      base_command_.mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp_);
    }
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  }
}

bool KortexMultiInterfaceHardware::setActuatorsControlMode(
  k_api::ActuatorConfig::ControlMode control_mode, const std::vector<bool> & joints)
{
  auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(control_mode);
  std::string names;
  try
  {
    // Device IDs are 1-based, so actuator i maps to device id i + 1.
    for (std::size_t i = 0; i < actuator_count_; i++)
    {
      if (!joints.empty() && !joints[i])
      {
        continue;
      }
      actuator_config_.SetControlMode(control_mode_message, static_cast<uint32_t>(i + 1));
      // Read it back: SetControlMode returning is not proof the actuator changed mode.
      const auto actual =
        actuator_config_.GetControlMode(static_cast<uint32_t>(i + 1)).control_mode();
      if (actual != control_mode)
      {
        RCLCPP_ERROR(
          LOGGER, "Actuator '%s' asked for %s control mode reports %s", arm_joint_names_[i].c_str(),
          k_api::ActuatorConfig::ControlMode_Name(control_mode).c_str(),
          k_api::ActuatorConfig::ControlMode_Name(actual).c_str());
        return false;
      }
      names += (names.empty() ? "" : ", ") + arm_joint_names_[i];
    }
    RCLCPP_INFO(
      LOGGER, "Set actuators [%s] to %s control mode", names.c_str(),
      k_api::ActuatorConfig::ControlMode_Name(control_mode).c_str());
    return true;
  }
  catch (k_api::KDetailedException & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception while setting control mode: " << ex.what());
    RCLCPP_ERROR_STREAM(
      LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::exception & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Exception while setting control mode: " << ex.what());
  }
  return false;
}

void KortexMultiInterfaceHardware::holdMeasuredPosition()
{
  feedback_ = base_cyclic_.RefreshFeedback();
  acceptBlockingFeedback();
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    const float measured = feedback_.actuators(static_cast<int>(i)).position();  // deg, raw
    base_command_.mutable_actuators(static_cast<int>(i))->set_position(measured);
    arm_commands_positions_[i] = calibratedPosition(feedback_, i);
  }
}

bool KortexMultiInterfaceHardware::sendHoldFrame()
{
  incrementId();
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  }
  try
  {
    feedback_ = base_cyclic_.Refresh(base_command_);
    acceptBlockingFeedback();
    return true;
  }
  catch (std::exception & ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Could not send the hold frame: " << ex.what());
    return false;
  }
}

bool KortexMultiInterfaceHardware::effortSignsAgree()
{
  bool agree = true;
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (!torque_joints_[i])
    {
      continue;
    }
    const double commanded = last_effort_commands_[i];
    const double measured =
      REPORTED_TORQUE_SIGN * feedback_.actuators(static_cast<int>(i)).torque();
    if (commanded * measured < 0.0 && std::abs(commanded - measured) > effort_check_tolerance_)
    {
      if (++effort_disagreement_cycles_[i] >= effort_check_cycles_)
      {
        RCLCPP_ERROR(
          LOGGER,
          "Torque sign check failed on '%s': commanded %+.2f N*m, measured %+.2f N*m for %d "
          "cycles while holding. Wrong effort_command_signs entry for this joint?",
          arm_joint_names_[i].c_str(), commanded, measured, effort_disagreement_cycles_[i]);
        agree = false;
      }
    }
    else
    {
      effort_disagreement_cycles_[i] = 0;
    }
  }
  return agree;
}

bool KortexMultiInterfaceHardware::enterTorqueMode()
{
  // Kinova example 108, for every torque actuator: first a frame whose position is the measured
  // one and whose torque is the one the joint carries right now, then the switch. Each actuator
  // picks up that torque the moment it changes mode, so the arm holds until the controller's
  // first command, which the rate limit then ramps in.
  holdMeasuredPosition();
  std::string names;
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (!torque_joints_[i])
    {
      continue;
    }
    const double hold = std::clamp(
      REPORTED_TORQUE_SIGN * feedback_.actuators(static_cast<int>(i)).torque(), -effort_limits_[i],
      effort_limits_[i]);
    base_command_.mutable_actuators(static_cast<int>(i))
      ->set_torque_joint(static_cast<float>(effort_command_signs_[i] * hold));
    arm_commands_efforts_[i] = hold;
    last_effort_commands_[i] = hold;
    torque_integrals_[i] = 0.0;
    effort_disagreement_cycles_[i] = 0;
    names += (names.empty() ? "" : ", ") + arm_joint_names_[i] +
             (effort_command_signs_[i] > 0 ? " (+1)" : " (-1)");
  }
  RCLCPP_INFO(LOGGER, "Entering torque mode on [%s]", names.c_str());
  if (!effort_command_signs_set_)
  {
    RCLCPP_WARN(
      LOGGER,
      "effort_command_signs is not set: using +1 on every joint, from Kinova's example, NOT "
      "measured on this arm. Run the per-joint sign test before trusting it.");
  }
  if (!sendHoldFrame())
  {
    torque_joints_.assign(actuator_count_, false);
    return false;
  }
  use_effort_command_ = true;
  if (!setActuatorsControlMode(torque_control_mode_, torque_joints_))
  {
    // Possibly half switched: put back whatever did switch.
    leaveTorqueMode();
    return false;
  }

  // Startup sign check, still holding and still inside the mode switch, so that a wrong command
  // sign fails the activation instead of dropping the arm under a controller (see the header).
  // Cannot see a joint that carries ~no torque (joint_1 on a level base, joint_6): that is what
  // the per-joint sign test is for.
  const auto check_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(effort_startup_check_ms_);
  while (std::chrono::steady_clock::now() < check_end)
  {
    for (std::size_t i = 0; i < actuator_count_; i++)
    {
      if (torque_joints_[i])
      {
        base_command_.mutable_actuators(static_cast<int>(i))
          ->set_position(feedback_.actuators(static_cast<int>(i)).position());
      }
    }
    if (!sendHoldFrame() || !effortSignsAgree())
    {
      RCLCPP_ERROR(LOGGER, "Torque mode startup check failed: back to POSITION mode.");
      leaveTorqueMode();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

void KortexMultiInterfaceHardware::leaveTorqueMode()
{
  // The reverse: a frame holding the measured position (and still the last torque, for the
  // actuators until they switch), then POSITION mode. Each actuator stops where it is.
  holdMeasuredPosition();
  sendHoldFrame();
  setActuatorsControlMode(k_api::ActuatorConfig::ControlMode::POSITION, torque_joints_);
  use_effort_command_ = false;
  torque_joints_.assign(actuator_count_, false);
  torque_integrals_.assign(actuator_count_, 0.0);
}

void KortexMultiInterfaceHardware::sendJointCommands()
{
  // identifier++
  incrementId();

  prepareCommands();

  // send the command to the robot
  try
  {
    cyclicRefresh(true);
  }
  catch (k_api::KDetailedException & ex)
  {
    cyclicRefresh(false);
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(
      LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error & ex_runtime)
  {
    cyclicRefresh(false);
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error & ex_future)
  {
    cyclicRefresh(false);
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception & ex_std)
  {
    cyclicRefresh(false);
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void KortexMultiInterfaceHardware::cyclicRefresh(bool with_command)
{
  if (!cyclic_async_)
  {
    feedback_ = with_command ? base_cyclic_.Refresh(base_command_) : base_cyclic_.RefreshFeedback();
    acceptBlockingFeedback();
    return;
  }
  // Runs on the API's receive thread. Captures replies_, not this: it may outlive the driver.
  auto on_reply = [replies = replies_, seq = ++next_seq_](
                    const k_api::Error & err, const k_api::BaseCyclic::Feedback & feedback)
  {
    if (err.error_code() != k_api::ErrorCodes::ERROR_NONE)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(replies->mutex);
    if (seq <= replies->seq)
    {
      return;  // overtaken by a newer reply, or sent before a blocking refresh
    }
    replies->feedback = feedback;
    replies->seq = seq;
    replies->fresh = true;
    replies->unanswered_since_ns = 0;
  };
  std::int64_t none = 0;
  replies_->unanswered_since_ns.compare_exchange_strong(none, steadyNowNs());
  if (with_command)
  {
    base_cyclic_.Refresh_callback(base_command_, on_reply);
  }
  else
  {
    base_cyclic_.RefreshFeedback_callback(on_reply);
  }
}

void KortexMultiInterfaceHardware::acceptBlockingFeedback()
{
  std::lock_guard<std::mutex> lock(replies_->mutex);
  replies_->seq = next_seq_;
  replies_->fresh = false;
  replies_->unanswered_since_ns = 0;
}

void KortexMultiInterfaceHardware::incrementId()
{
  // Incrementing identifier ensures actuators can reject out of time frames
  base_command_.set_frame_id(base_command_.frame_id() + 1);
  if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);
}

void KortexMultiInterfaceHardware::sendGripperCommand(
  k_api::Base::ServoingMode arm_mode, double position, double velocity, double force)
{
  if (gripper_controller_running_ && !std::isnan(position) && use_internal_bus_gripper_comm_)
  {
    try
    {
      if (arm_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
      {
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(
          static_cast<float>(position / 0.81));  // This values needs to be between 0 and 1
        base_.SendGripperCommand(gripper_command);
      }
      else if (arm_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        // % open/closed, this values needs to be between 0 and 100
        gripper_motor_command_->set_position(static_cast<float>(position / 0.81 * 100.0));
        // % gripper speed between 0 and 100 percent
        gripper_motor_command_->set_velocity(static_cast<float>(velocity));
        // % max force threshold, between 0 and 100
        gripper_motor_command_->set_force(static_cast<float>(force));
      }
    }
    catch (k_api::KDetailedException & ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception caught while sending internal gripper command!");
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(
        LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                  k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }
  }
}

void KortexMultiInterfaceHardware::sendTwistCommand()
{
  k_api_twist_->set_linear_x(static_cast<float>(twist_commands_[0]));
  k_api_twist_->set_linear_y(static_cast<float>(twist_commands_[1]));
  k_api_twist_->set_linear_z(static_cast<float>(twist_commands_[2]));
  // The Kortex API takes angular velocity in DEGREES per second, while the
  // ros2_control twist interface is rad/s like every other ROS angular rate.
  // Passing radians straight through makes every commanded rotation 57x too
  // small, which presents as Cartesian rotation not working at all while
  // translation is fine. The joint path already converts with toDeg for both
  // position and velocity; this one was missed.
  k_api_twist_->set_angular_x(static_cast<float>(KortexMathUtil::toDeg(twist_commands_[3])));
  k_api_twist_->set_angular_y(static_cast<float>(KortexMathUtil::toDeg(twist_commands_[4])));
  k_api_twist_->set_angular_z(static_cast<float>(KortexMathUtil::toDeg(twist_commands_[5])));
  base_.SendTwistCommand(k_api_twist_command_);
}

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
