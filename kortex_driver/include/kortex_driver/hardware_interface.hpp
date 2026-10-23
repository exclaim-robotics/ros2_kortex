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
#ifndef KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/joint_calibration.hpp"
#include "kortex_driver/torque_integral.hpp"
#include "kortex_driver/visibility_control.h"

#include "ActuatorConfigClientRpc.h"
#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"

namespace hardware_interface
{
constexpr char HW_IF_TWIST[] = "twist";
constexpr char HW_IF_FAULT[] = "fault";

}  // namespace hardware_interface

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace k_api = Kinova::Api;

namespace kortex_driver
{
enum class StopStartInterface
{
  NONE,
  STOP_POS_VEL,
  STOP_TWIST,
  STOP_GRIPPER,
  STOP_FAULT_CTRL,
  START_POS_VEL,
  START_TWIST,
  START_GRIPPER,
  START_FAULT_CTRL,
};
class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  KortexMultiInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) final;
  KORTEX_DRIVER_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) final;

  // The API teardown lives here, not in on_deactivate. Deactivating is a
  // handover -- the arm drops to SINGLE_LEVEL_SERVOING so the pendant works,
  // and the connection stays up so it can be taken back. Tearing the
  // connection down belongs to the transitions that really are the end.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) final;

private:
  // Close the sessions and drop both transports. Idempotent: on_cleanup and
  // on_shutdown are not mutually exclusive and either may be skipped.
  void disconnectApi();
  bool api_connected_{false};

  k_api::TransportClientTcp transport_tcp_;
  k_api::RouterClient router_tcp_;
  k_api::SessionManager session_manager_;
  k_api::TransportClientUdp transport_udp_realtime_;
  k_api::RouterClient router_udp_realtime_;
  k_api::SessionManager session_manager_real_time_;

  // twist temporary command
  Kinova::Api::Base::Twist * k_api_twist_;
  k_api::Base::TwistCommand k_api_twist_command_;

  // Control of the robot arm itself
  k_api::Base::BaseClient base_;
  k_api::BaseCyclic::BaseCyclicClient base_cyclic_;
  // Used to switch individual actuators between POSITION, VELOCITY and TORQUE control mode
  k_api::ActuatorConfig::ActuatorConfigClient actuator_config_;
  k_api::BaseCyclic::Command base_command_;
  std::size_t actuator_count_;
  // To minimize bandwidth we synchronize feedback with the robot only when write() is called
  k_api::BaseCyclic::Feedback feedback_;

  // Cyclic exchange that does not hold up the controller manager. The per-cycle Refresh in
  // write() was a synchronous round trip to the arm: ~0.35 ms, but 1-7 ms whenever the arm
  // answered late, and the whole controller manager loop (gripper, F/T sensor, every controller)
  // waited for it -- "Overrun detected!". write() now sends with Refresh_callback, as Kinova's
  // example 200-Actuator_low_level_control/02 does, and returns; the reply lands on the API's
  // receive thread and read() takes the newest one. Normally that is the reply to the previous
  // cycle's frame, exactly what the blocking call gave; when the arm is late, read() keeps the
  // last feedback instead of stalling, and fails once none has come for feedback_timeout_.
  //
  // Held by shared_ptr because the callbacks capture it: a reply may arrive after this object is
  // gone. Replies carry a send sequence number, and only a newer one replaces the stored reply,
  // so late or reordered replies -- and replies to frames sent before a blocking refresh in a
  // mode switch -- never overwrite fresher feedback.
  struct CyclicReplies
  {
    std::mutex mutex;
    k_api::BaseCyclic::Feedback feedback;  // the newest reply, until read() takes it
    bool fresh = false;                    // feedback not taken by read() yet
    std::uint64_t seq = 0;                 // replies with seq <= this are dropped
    // steady_clock ns of the oldest send not answered yet, 0 if none. Not "time since the last
    // reply": a mode switch or fault reset can spend well over feedback_timeout_ on the TCP
    // port without sending a cyclic frame, and that is not the arm going silent.
    std::atomic<std::int64_t> unanswered_since_ns{0};
  };
  std::shared_ptr<CyclicReplies> replies_ = std::make_shared<CyclicReplies>();
  std::uint64_t next_seq_ = 0;  // controller manager thread only
  bool cyclic_async_ = true;    // false: the old blocking Refresh in write()
  std::chrono::milliseconds feedback_timeout_{100};
  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_velocities_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // twist command interfaces
  std::vector<double> twist_commands_;

  // Gripper
  k_api::GripperCyclic::MotorCommand * gripper_motor_command_;
  double gripper_command_position_ = 0.0;
  double gripper_command_max_velocity_ = 0.0;
  double gripper_command_max_force_ = 0.0;
  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_force_command_ = 0.0;
  double gripper_speed_command_ = 0.0;

  rclcpp::Time controller_switch_time_;
  std::atomic<bool> block_write = false;
  k_api::Base::ServoingMode arm_mode_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_lvl_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    EFFORT = 3
  };

  std::vector<integration_lvl_t> arm_joints_control_level_;

  // changing active controller on the hardware
  k_api::Base::ServoingModeInformation servoing_mode_hw_;
  // what controller is running
  bool joint_based_controller_running_;
  bool twist_controller_running_;
  bool gripper_controller_running_;
  bool fault_controller_running_;
  // switching auxiliary vars
  // keeping track of which controller is active so appropriate control mode can be adjusted
  // controller manager sends array of interfaces that should be stopped/started and this is the
  // way to internally collect information on which controller should be stopped and started
  // (different controllers claim different interfaces)
  std::vector<StopStartInterface> stop_modes_;
  std::vector<StopStartInterface> start_modes_;
  // switching auxiliary booleans
  bool stop_joint_based_controller_;
  bool stop_twist_controller_;
  bool stop_gripper_controller_;
  bool stop_fault_controller_;
  bool start_joint_based_controller_;
  bool start_twist_controller_;
  bool start_gripper_controller_;
  bool start_fault_controller_;
  // Distinguishes whether the starting joint based controller claims the position or the
  // velocity command interface. Both flow to start_joint_based_controller_ but require a
  // different actuator control mode on the robot.
  bool start_joint_velocity_control_;
  bool start_joint_position_control_;
  bool start_joint_effort_control_;
  // True while the active joint based controller commands joint velocities instead of positions.
  bool use_velocity_command_;
  // True while the actuators in torque_joints_ are in TORQUE mode and follow
  // arm_commands_efforts_. The other arm joints stay in POSITION mode, holding.
  bool use_effort_command_;
  // Arm joint names in actuator order (info_.joints without the gripper).
  std::vector<std::string> arm_joint_names_;
  // Per arm joint (actuator order): the kinematic calibration between the arm's raw angles and
  // the ones ROS sees, from the joints' joint_offset / joint_compliance params. All zero (inert)
  // when they are absent. See joint_calibration.hpp for what it does and does not cover.
  std::vector<JointCalibration> joint_calibration_;
  // The calibrated angle of actuator i in feedback, wrapped to [-pi, pi] as before (rad).
  double calibratedPosition(const k_api::BaseCyclic::Feedback & feedback, std::size_t i) const;
  // Arm joints (actuator order) whose effort interface the starting (running) controller claims.
  std::vector<bool> start_torque_joints_;
  std::vector<bool> torque_joints_;

  // Torque sign. Everything ROS sees is in the ROS convention: positive torque drives the joint
  // positive. kortex reports joint torque the other way round on every joint (measured
  // 2026-10-08), so the effort state is negated. The command sign is a separate fact
  // (effort_command_signs, per joint) and must be measured, not inferred from the reported one.
  static constexpr double REPORTED_TORQUE_SIGN = -1.0;
  std::vector<double> effort_command_signs_;
  bool effort_command_signs_set_ = false;

  // Torque mode safety. Kinova does not compensate gravity in TORQUE mode: whatever is sent is
  // what the joint gets, so every command is clamped to the joint's URDF effort limit and may
  // change by at most effort_command_max_step_ per cycle, so a late or stale cycle cannot jump
  // the arm. last_effort_commands_ is what was actually sent (ROS convention, N*m).
  std::vector<double> effort_limits_;
  double effort_command_max_step_ = 0.5;

  // Which of Kinova's torque modes the effort joints go to (hardware param torque_control_mode):
  // TORQUE ("torque", the default) or TORQUE_HIGH_VELOCITY ("torque_high_velocity"). Through
  // TORQUE, joint_6 looked like 4-10 kg m^2 and 14-36 Nm s/rad, both changing with frequency
  // (torque-mode-id.py, 2026-10-23) - not the 0.22 kg m^2 rotor; the AIST paper on the Gen3
  // (arXiv 2502.02967) says TORQUE feeds back the joint velocity and the high-velocity mode
  // does not, but tracks torque worse. Commands, limits and the sign checks are the same in both.
  k_api::ActuatorConfig::ControlMode torque_control_mode_ =
    k_api::ActuatorConfig::ControlMode::TORQUE;

  // Leaky integral on each effort joint's torque error, added after the rate limit and clamp
  // (hardware param torque_integral, "K_I theta limit"; empty: off). See torque_integral.hpp.
  // Reset on entering and leaving torque mode; the startup sign check runs before it acts.
  TorqueIntegral torque_integral_;
  // Per joint, what each actuator delivers of what it is sent (hardware param torque_gains, one
  // number per joint; empty: 1): every torque command is divided by it. See torque_integral.hpp.
  std::vector<double> torque_gains_;
  std::vector<double> torque_integrals_;  // N*m, per joint; exported as torque_integral
  std::vector<double> arm_efforts_sent_;  // N*m, command + integral; exported as effort_sent
  double write_period_s_ = 0.001;         // s, the last write's period, at most 10 ms
  std::vector<double> last_effort_commands_;

  // Startup sign check, for effort_startup_check_ms_ inside the mode switch, while the driver
  // holds each torque joint at the torque it measured just before. The effort sensor reads the
  // torque passed on to the link, so it does NOT follow the command in general (a free joint
  // under a torque pulse reads ~0, measured on joint_6 2026-10-08), and this is no check to run
  // under a controller. While holding a gravity load it does work: with a wrong command sign
  // the motor pushes with the load instead of against it and the reading flips. A joint whose
  // measured effort has the opposite sign to its command, by more than
  // effort_check_tolerance_, for effort_check_cycles_ cycles in a row fails the activation.
  double effort_check_tolerance_ = 1.0;
  int effort_check_cycles_ = 5;
  int effort_startup_check_ms_ = 150;
  std::vector<int> effort_disagreement_cycles_;

  // first pass flag
  bool first_pass_;

  // gripper stuff
  std::string gripper_joint_name_;
  bool use_internal_bus_gripper_comm_;

  // temp variables to use in update loop
  float cmd_degrees_tmp_;
  float cmd_vel_tmp_;
  int num_turns_tmp_ = 0;

  // fault control
  double reset_fault_cmd_;
  double reset_fault_async_success_;
  double in_fault_;
  // Fault and warning banks as last logged (base A, base B, then A/B per actuator), so a
  // change is logged once, by name, instead of every cycle or never.
  std::vector<uint32_t> logged_banks_;
  void logBankChanges();
  static constexpr double NO_CMD = std::numeric_limits<double>::quiet_NaN();

  void sendTwistCommand();
  // The per-cycle exchange from write(): base_command_ and feedback (with_command) or feedback
  // only. Does not wait unless cyclic_async_ is off.
  void cyclicRefresh(bool with_command);
  // After a blocking Refresh/RefreshFeedback has set feedback_ directly: drop every reply to a
  // frame sent before it, and restart the feedback timeout.
  void acceptBlockingFeedback();
  void incrementId();
  void sendJointCommands();
  void prepareCommands();
  // Set the arm actuators (all, or those in joints) into the given control mode (POSITION,
  // VELOCITY or TORQUE). Returns false (and logs) if the robot refused it for any actuator.
  bool setActuatorsControlMode(
    k_api::ActuatorConfig::ControlMode control_mode, const std::vector<bool> & joints = {});
  // Put the torque_joints_ actuators into TORQUE mode without moving the arm, then run the
  // startup sign check (see the definition). On failure the arm is left in POSITION mode,
  // holding.
  bool enterTorqueMode();
  // Put the torque_joints_ actuators back into POSITION mode holding where they are now.
  void leaveTorqueMode();
  // False once a torque joint's measured effort has disagreed with its command (see above).
  bool effortSignsAgree();
  // Seed the position command, here and on the robot, with the measured position.
  void holdMeasuredPosition();
  // Send base_command_ once, outside the control loop, during a mode switch.
  bool sendHoldFrame();
  void sendGripperCommand(
    k_api::Base::ServoingMode arm_mode, double position, double velocity, double force);

  void readGripperPosition();
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
