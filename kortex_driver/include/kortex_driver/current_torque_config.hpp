// Copyright (c) 2026 Exclaim Robotics
// Licensed under the Apache License, Version 2.0 (see LICENSE).
//
// torque_control: current's settings, from the YAML file current_torque_config
// (config/current_torque.yaml has them all, with why). Per-joint values are a list, one per arm
// joint in actuator order, or a single value for all of them:
//
//   lambda, torque_constant (N*m/A), current_limit (A)     required
//   gear_ratio                                             default 100
//   friction: velocity_threshold (rad/s), error_threshold (N*m),
//             coulomb, viscous, stiction                   default 0: off
//   integral: gain (1/s), leak (s), bound (N*m)            gain 0: off; bound 5% of the URDF
//                                                          effort limit
//   safety:   speed_limit (rad/s, 0 off), kt_check_ratio, kt_check_min_torque (N*m)

#ifndef KORTEX_DRIVER__CURRENT_TORQUE_CONFIG_HPP_
#define KORTEX_DRIVER__CURRENT_TORQUE_CONFIG_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "kortex_driver/current_torque_loop.hpp"

namespace kortex_driver
{

struct CurrentTorqueConfig
{
  std::vector<CurrentTorqueJointConfig> joints;
  // A joint under the loop faster than this (rad/s) ends CURRENT mode: the arm holds. 0: off.
  double speed_limit = 1.0;
  // At the switch, while the arm still holds in POSITION mode: on every joint carrying more than
  // kt_check_min_torque, the torque its measured current makes through torque_constant and
  // gear_ratio must agree in sign with the measured torque and be within kt_check_ratio of it.
  double kt_check_ratio = 1.6;
  double kt_check_min_torque = 3.0;
};

namespace detail
{
// key's value per joint: a list of n, or one value for all; `fallback` when absent (NaN:
// required).
inline std::vector<double> perJoint(
  const YAML::Node & node, const std::string & key, std::size_t n, double fallback)
{
  const YAML::Node v = node[key];
  if (!v)
  {
    if (std::isnan(fallback))
    {
      throw std::runtime_error("'" + key + "' is required");
    }
    return std::vector<double>(n, fallback);
  }
  if (v.IsScalar())
  {
    return std::vector<double>(n, v.as<double>());
  }
  auto out = v.as<std::vector<double>>();
  if (out.size() != n)
  {
    throw std::runtime_error(
      "'" + key + "' has " + std::to_string(out.size()) + " values for " + std::to_string(n) +
      " joints");
  }
  return out;
}

inline double scalar(const YAML::Node & node, const std::string & key, double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}
}  // namespace detail

// The settings in `yaml` (the file's text) for the arm joints, whose URDF effort limits are
// effort_limits (N*m). Throws std::runtime_error with what is wrong.
inline CurrentTorqueConfig parseCurrentTorqueConfig(
  const std::string & yaml, const std::vector<double> & effort_limits)
{
  using detail::perJoint;
  using detail::scalar;
  const std::size_t n = effort_limits.size();
  const double required = std::nan("");
  YAML::Node root;
  try
  {
    root = YAML::Load(yaml);
  }
  catch (const YAML::Exception & ex)
  {
    throw std::runtime_error(std::string("not YAML: ") + ex.what());
  }
  const YAML::Node none;
  const YAML::Node friction = root["friction"] ? root["friction"] : none;
  const YAML::Node integral = root["integral"] ? root["integral"] : none;
  const YAML::Node safety = root["safety"] ? root["safety"] : none;

  const auto lambda = perJoint(root, "lambda", n, required);
  const auto kt = perJoint(root, "torque_constant", n, required);
  const auto limit = perJoint(root, "current_limit", n, required);
  const auto ratio = perJoint(root, "gear_ratio", n, 100.0);
  const auto coulomb = perJoint(friction, "coulomb", n, 0.0);
  const auto viscous = perJoint(friction, "viscous", n, 0.0);
  const auto stiction = perJoint(friction, "stiction", n, 0.0);
  std::vector<double> bound(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    bound[i] = std::isfinite(effort_limits[i]) ? 0.05 * effort_limits[i] : 0.0;
  }
  if (integral["bound"])
  {
    bound = perJoint(integral, "bound", n, 0.0);
  }

  CurrentTorqueConfig config;
  config.speed_limit = scalar(safety, "speed_limit", config.speed_limit);
  config.kt_check_ratio = scalar(safety, "kt_check_ratio", config.kt_check_ratio);
  config.kt_check_min_torque = scalar(safety, "kt_check_min_torque", config.kt_check_min_torque);
  const double velocity_threshold = scalar(friction, "velocity_threshold", 0.035);
  const double error_threshold = scalar(friction, "error_threshold", 0.5);
  const double gain = scalar(integral, "gain", 0.0);
  const double leak = scalar(integral, "leak", 0.1);

  const auto bad = [](const std::string & what) { throw std::runtime_error(what); };
  if (!(config.speed_limit >= 0.0)) bad("safety.speed_limit must be >= 0");
  if (!(config.kt_check_ratio > 1.0)) bad("safety.kt_check_ratio must be > 1");
  if (!(config.kt_check_min_torque > 0.0)) bad("safety.kt_check_min_torque must be > 0");
  if (!(velocity_threshold > 0.0) || !(error_threshold > 0.0))
    bad("friction.velocity_threshold and error_threshold must be > 0");
  if (!(gain >= 0.0) || !(leak > 0.0)) bad("integral.gain must be >= 0 and integral.leak > 0");

  for (std::size_t i = 0; i < n; ++i)
  {
    const std::string j = " (joint " + std::to_string(i + 1) + ")";
    if (!(lambda[i] >= 0.0)) bad("lambda must be >= 0" + j);
    if (!(kt[i] > 0.0) || !(ratio[i] > 0.0)) bad("torque_constant and gear_ratio must be > 0" + j);
    if (!(limit[i] > 0.0)) bad("current_limit must be > 0" + j);
    if (!(coulomb[i] >= 0.0) || !(viscous[i] >= 0.0) || !(stiction[i] >= 0.0))
      bad("friction terms must be >= 0" + j);
    if (!(bound[i] >= 0.0)) bad("integral.bound must be >= 0" + j);
    CurrentTorqueJointConfig c;
    c.lambda = lambda[i];
    c.torque_constant = kt[i];
    c.gear_ratio = ratio[i];
    c.current_limit = limit[i];
    c.coulomb = coulomb[i];
    c.viscous = viscous[i];
    c.stiction = stiction[i];
    c.velocity_threshold = velocity_threshold;
    c.error_threshold = error_threshold;
    c.integral_gain = gain;
    c.integral_leak = leak;
    c.integral_bound = bound[i];
    config.joints.push_back(c);
  }
  return config;
}

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__CURRENT_TORQUE_CONFIG_HPP_
