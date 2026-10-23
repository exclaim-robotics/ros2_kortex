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

#ifndef KORTEX_DRIVER__TORQUE_INTEGRAL_HPP_
#define KORTEX_DRIVER__TORQUE_INTEGRAL_HPP_

#include <algorithm>
#include <cmath>
#include <istream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace kortex_driver
{
// A leaky integral on the torque error of a joint in torque mode, added to what is sent:
//
//   I += dt * (K_I * (command - measured) - theta * I),  |I| <= limit
//   sent = clamp(command + I, effort limit)
//
// the leaky integrator K_I / (s + theta) of the AIST paper's Gen3 torque loop (arXiv
// 2502.02967; mc_kortex runs theta 0.1). It takes out the static torque error: through
// TORQUE_HIGH_VELOCITY, joint_2 holding 18 N*m of gravity sagged with the command 6 N*m and more
// short (2026-10-23). A static error is left at 1 / (1 + K_I / theta) of what it was.
//
// measured is the joint torque sensor, which reads the torque passed on to the link. On a free
// joint the integral therefore makes the actuator a torque source at the link, its friction and
// rotor taken out up to ~K_I rad/s - which is the point, and which also means a free joint under
// a constant command accelerates harder than without it.
//
// It stops growing toward the effort limit while the sum is there (no wind-up against a clamp).
// Off with a zero gain or limit.
// C++14, as the driver's own default (no std::clamp).
struct TorqueIntegral
{
  double gain = 0.0;   // K_I, 1/s
  double leak = 0.0;   // theta, 1/s
  double limit = 0.0;  // N*m, |I| at most

  bool on() const { return gain > 0.0 && limit > 0.0; }

  // One cycle of dt seconds: updates integral, returns the torque to send.
  double step(
    double & integral, double command, double measured, double effort_limit, double dt) const
  {
    if (!on())
    {
      return command;
    }
    const double error = command - measured;
    const double sum = command + integral;
    const bool saturated = std::abs(sum) >= effort_limit && sum * error > 0.0;
    if (!saturated)
    {
      integral += dt * (gain * error - leak * integral);
    }
    integral = std::max(-limit, std::min(limit, integral));
    return std::max(-effort_limit, std::min(effort_limit, command + integral));
  }
};

// "K_I theta limit" (1/s, 1/s, N*m), three numbers >= 0; empty: off. Throws
// std::invalid_argument otherwise.
inline TorqueIntegral parseTorqueIntegral(const std::string & text)
{
  TorqueIntegral t;
  if (text.find_first_not_of(" \t") == std::string::npos)
  {
    return t;
  }
  std::istringstream in(text);
  if (!(in >> t.gain >> t.leak >> t.limit) || !(in >> std::ws).eof())
  {
    throw std::invalid_argument(
      "torque_integral '" + text + "': expected three numbers, \"K_I theta limit\" (1/s, 1/s, N*m)");
  }
  if (!(t.gain >= 0.0) || !(t.leak >= 0.0) || !(t.limit >= 0.0))
  {
    throw std::invalid_argument("torque_integral '" + text + "': every number must be >= 0");
  }
  return t;
}

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__TORQUE_INTEGRAL_HPP_
