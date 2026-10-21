// Copyright 2026, Exclaim Robotics.
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

#ifndef KORTEX_DRIVER__JOINT_CALIBRATION_HPP_
#define KORTEX_DRIVER__JOINT_CALIBRATION_HPP_

#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace kortex_driver
{
// Per-joint kinematic calibration, applied to the readings and the position commands only:
//
//   q_true = q_raw + offset + compliance * tau
//
// offset (rad) is the joint's zero error, compliance (rad/(N*m)) its deflection under the torque
// it carries, tau the joint torque as the effort state interface reports it (N*m, ROS sign).
// Fitted by rackthor's omnicalibrator. Nothing is written to the actuators: their own zero and
// their own joint limits stay exactly as they were, the driver only translates between the arm's
// numbers (raw) and the calibrated ones every ROS consumer sees.
//
// Consequences, by design:
// - Position commands are translated back: raw_cmd = cmd - offset - compliance * tau_now.
//   Velocity and effort commands, and the velocity and effort states, are unchanged.
// - Joint limits stay raw. The arm faults on its own raw thresholds while the URDF limits are
//   compared with calibrated angles, so a limit is effectively moved by -offset. Offsets are
//   under 2 deg and the URDF limits keep 2+ deg of margin inside the arm's thresholds.
// - Twist mode (Kinova's internal Cartesian control) runs the arm's own kinematics on raw
//   joints and cannot be corrected; its Cartesian motion is in the uncalibrated model.
struct JointCalibration
{
  double offset = 0.0;      // rad
  double compliance = 0.0;  // rad / (N*m)

  bool active() const { return offset != 0.0 || compliance != 0.0; }

  // offset + compliance * effort. A non-finite effort (no reading yet) contributes nothing, and
  // with compliance 0 the effort is never looked at.
  double correction(double effort) const
  {
    if (compliance == 0.0 || !std::isfinite(effort))
    {
      return offset;
    }
    return offset + compliance * effort;
  }

  // Raw reading -> calibrated (rad, unwrapped: the caller wraps as before).
  double calibrated(double raw, double effort) const { return raw + correction(effort); }
  // Calibrated position command -> the raw one to send (rad, unwrapped).
  double raw(double calibrated_cmd, double effort) const
  {
    return calibrated_cmd - correction(effort);
  }
};

// From a <joint>'s ros2_control <param>s: "joint_offset" and "joint_compliance", each optional
// with default 0. Throws std::invalid_argument on a value that is not a finite number.
inline JointCalibration parseJointCalibration(
  const std::unordered_map<std::string, std::string> & parameters)
{
  JointCalibration cal;
  const auto read = [&parameters](const char * name, double & out)
  {
    const auto it = parameters.find(name);
    if (it == parameters.end() || it->second.empty())
    {
      return;
    }
    std::size_t used = 0;
    double value = 0.0;
    try
    {
      value = std::stod(it->second, &used);
    }
    catch (const std::exception &)
    {
      used = 0;
    }
    while (used < it->second.size() && std::isspace(static_cast<unsigned char>(it->second[used])))
    {
      ++used;
    }
    if (used == 0 || used != it->second.size() || !std::isfinite(value))
    {
      throw std::invalid_argument(std::string(name) + " '" + it->second + "' is not a number");
    }
    out = value;
  };
  read("joint_offset", cal.offset);
  read("joint_compliance", cal.compliance);
  return cal;
}

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__JOINT_CALIBRATION_HPP_
