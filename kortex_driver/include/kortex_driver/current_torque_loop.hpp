// Copyright (c) 2026 Exclaim Robotics
// Licensed under the Apache License, Version 2.0 (see LICENSE).
//
// The driver's own joint torque loop for an actuator in CURRENT mode (torque_control: current).
//
// Kinova's TORQUE mode closes its own torque loop in the actuator, and through it a Gen3 joint
// moves like a heavy, strongly damped one: joint_6 looked 4-10 kg m^2 and 14-36 N*m s/rad against
// its rotor's 0.22 kg m^2 (rackthor torque-mode-id.py, 2026-10-23). CURRENT mode has no such
// loop: the motor current is the command. This loop puts one back, on the joint torque sensor,
// after the AIST paper on the Gen3 (arXiv 2502.02967) and its mc_kortex
// (KinovaRobot::currentTorqueControlLaw). Per joint, each 1 kHz cycle:
//
//   e       = tau_desired - tau_measured                     the joint torque sensor, ROS sign
//   current = (tau_desired + lambda * H(e) + I + F) / (Kt * G)
//
// tau_desired is the feedforward: the torque, as a motor current through the motor's torque
// constant Kt and the gear ratio G. H is the paper's fixed discrete filter on the torque error
// (Kinova's high-velocity transfer function, unity gain at DC, at 1 kHz), times lambda: the
// feedback that makes the joint torque follow the command where friction and Kt's errors keep
// the feedforward from it. I is a slow leaky integral of e, for what is left at rest. F is
// friction compensation:
//
//   |v| >= velocity_threshold    coulomb * sign(v) + viscous * v
//   |e| >= error_threshold       stiction * sign(e)      (slower: quasi-static)
//   otherwise                    0
//
// or, with velocity_scale > 0, the same terms without a step anywhere:
//
//   coulomb * tanh(v / velocity_scale) + viscous * v
//     + stiction * tanh(e / error_threshold) * (1 - |tanh(v / velocity_scale)|)
//
// The paper switches stiction on the desired acceleration; the driver is given torques only, so
// at rest stiction pushes the way the torque error points. Its rotor inertia term needs the
// desired acceleration too and is left out.
//
// Overcompensated friction pushes a joint along by itself: every friction term starts at 0 and
// is raised on the arm, joint by joint. The current is clamped to current_limit.
//
// Or, with observer_bandwidth > 0, a friction observer in place of lambda * H, I and F (after
// Le Tien and Albu-Schaeffer's for joints with torque sensors): on the gearbox's side
//
//   f_hat  <- low-pass at observer_bandwidth of (tau_motor - tau_measured - rotor_inertia * a)
//   current = (tau_desired + f_hat) / (Kt * G)
//
// with tau_motor the torque last commanded and a the joint's acceleration from its velocity.
// It needs no friction model and follows load, pose and temperature. While a joint is stuck the
// 'friction' it measures is just the torque applied, and integrating that hunts (a slow
// slip back and forth at rest); so f_hat is held while the joint is still (|v| under
// observer_velocity_gate) and the error small (|e| under observer_deadband). Simulated on a big
// joint with rackthor-001's friction (rackthor_common identification/friction-sim.py): ~5x the
// tracking of lambda * H, quiet at rest, unstable from ~15 Hz (the drive's elasticity, the
// delay); f_hat is bounded by observer_bound, which has to clear the breakaway.

#ifndef KORTEX_DRIVER__CURRENT_TORQUE_LOOP_HPP_
#define KORTEX_DRIVER__CURRENT_TORQUE_LOOP_HPP_

#include <algorithm>
#include <cmath>

namespace kortex_driver
{

struct CurrentTorqueJointConfig
{
  double lambda = 0.0;           // gain on the filtered torque error
  double torque_constant = 0.0;  // N*m/A, the motor's (before the gear)
  double gear_ratio = 100.0;
  double current_limit = 0.0;    // A, both ways
  double coulomb = 0.0;          // N*m
  double viscous = 0.0;          // N*m s/rad
  double stiction = 0.0;         // N*m
  double velocity_threshold = 0.035;  // rad/s: slower is quasi-static
  double error_threshold = 0.5;       // N*m: the torque error that brings stiction in at rest
  double velocity_scale = 0.0;        // rad/s: > 0, the smooth friction model (see update)
  double integral_gain = 0.0;         // 1/s: d I/dt = integral_gain * e - I / integral_leak
  double integral_leak = 0.1;         // s
  double integral_bound = 0.0;        // N*m, both ways
  double observer_bandwidth = 0.0;    // Hz: > 0, the friction observer in place of lambda * H
  double observer_deadband = 0.5;     // N*m: |e| under which a still joint's f_hat is held
  double observer_velocity_gate = 0.005;  // rad/s: slower is still, for the deadband
  double observer_bound = 0.0;        // N*m, |f_hat| at most
  double rotor_inertia = 0.0;         // kg m^2, at the joint (Gen3: 0.40 big, 0.22 small)
};

class CurrentTorqueLoop
{
public:
  // The filter H(z) = (B0 + B1 z^-1 + B2 z^-2) / (1 + A1 z^-1 + A2 z^-2), at PERIOD.
  static constexpr double B0 = 0.02017482;
  static constexpr double B1 = -0.03697504;
  static constexpr double B2 = 0.01691718;
  static constexpr double A1 = -1.975063;
  static constexpr double A2 = 0.9751799;
  static constexpr double PERIOD = 0.001;  // s

  explicit CurrentTorqueLoop(const CurrentTorqueJointConfig & config = {}) : config_(config) {}

  const CurrentTorqueJointConfig & config() const { return config_; }

  // Forget the filter's and the integral's history: at the switch into CURRENT mode.
  void reset()
  {
    e1_ = e2_ = h1_ = h2_ = 0.0;
    integral_ = 0.0;
    error_ = filtered_ = friction_ = current_ = 0.0;
    f_hat_ = last_torque_ = v_f_ = acceleration_ = 0.0;
    started_ = false;
  }

  // The motor current (A, ROS sign) for one cycle of dt seconds. The filter assumes PERIOD.
  double update(double tau_desired, double tau_measured, double velocity, double dt)
  {
    const auto & c = config_;
    if (c.observer_bandwidth > 0.0)
    {
      return observe(tau_desired, tau_measured, velocity, dt);
    }
    error_ = tau_desired - tau_measured;
    filtered_ = B0 * error_ + B1 * e1_ + B2 * e2_ - A1 * h1_ - A2 * h2_;
    e2_ = e1_;
    e1_ = error_;
    h2_ = h1_;
    h1_ = filtered_;

    if (dt > 0.0 && c.integral_leak > 0.0)
    {
      // Exact for a constant e over the step: I relaxes to integral_gain * leak * e.
      const double a = std::exp(-dt / c.integral_leak);
      integral_ = a * integral_ + (1.0 - a) * c.integral_gain * c.integral_leak * error_;
      integral_ = clamp(integral_, c.integral_bound);
    }

    if (c.velocity_scale > 0.0)
    {
      // Smooth: no step anywhere. The hard switches below step the torque by 2 x coulomb each
      // time the velocity crosses the threshold, and noise at rest crosses it all the time
      // (joint_3: 774 flips of +-3 N*m while 95% stuck; joint_4 in a 12 Hz stick-slip cycle with
      // crisp, 2026-10-27). Coulomb fades in with tanh(v / velocity_scale); stiction, the way
      // the error points, with tanh(e / error_threshold), and fades out as the joint moves.
      const double moving = std::tanh(velocity / c.velocity_scale);
      friction_ = c.coulomb * moving + c.viscous * velocity +
                  c.stiction * std::tanh(error_ / c.error_threshold) * (1.0 - std::abs(moving));
    }
    else if (std::abs(velocity) >= c.velocity_threshold)
    {
      friction_ = c.coulomb * sign(velocity) + c.viscous * velocity;
    }
    else if (std::abs(error_) >= c.error_threshold)
    {
      friction_ = c.stiction * sign(error_);
    }
    else
    {
      friction_ = 0.0;
    }

    const double torque = tau_desired + c.lambda * filtered_ + integral_ + friction_;
    current_ = clamp(currentFor(torque), c.current_limit);
    return current_;
  }

  // The current that gives this joint torque with nothing else in the way, and the reverse.
  double currentFor(double torque) const
  {
    return torque / (config_.torque_constant * config_.gear_ratio);
  }
  double torqueFor(double current) const
  {
    return current * config_.torque_constant * config_.gear_ratio;
  }

  // The last update's terms, for logging and tuning. feedback(): lambda * H(e), 0 under the
  // observer, whose estimate is friction().
  double error() const { return error_; }
  double filtered() const { return filtered_; }
  double feedback() const
  {
    return config_.observer_bandwidth > 0.0 ? 0.0 : config_.lambda * filtered_;
  }
  double acceleration() const { return acceleration_; }
  double integral() const { return integral_; }
  double friction() const { return friction_; }
  double current() const { return current_; }

private:
  double observe(double tau_desired, double tau_measured, double velocity, double dt)
  {
    const auto & c = config_;
    error_ = tau_desired - tau_measured;
    // The acceleration from the velocity, filtered first (the actuator's velocity is noisy):
    // ~60 Hz at 1 kHz, below the drive's elastic mode, above the observer.
    if (!started_)
    {
      v_f_ = velocity;
      started_ = true;
    }
    const double v_prev = v_f_;
    v_f_ += VELOCITY_FILTER * (velocity - v_f_);
    acceleration_ = dt > 0.0 ? (v_f_ - v_prev) / dt : 0.0;

    const double residual = last_torque_ - tau_measured - c.rotor_inertia * acceleration_;
    const double moving = std::tanh(std::pow(v_f_ / c.observer_velocity_gate, 2));
    const double pushed = std::tanh(std::pow(error_ / c.observer_deadband, 4));
    const double alpha =
      (1.0 - std::exp(-2.0 * PI * c.observer_bandwidth * dt)) * std::max(moving, pushed);
    f_hat_ = clamp(f_hat_ + alpha * (residual - f_hat_), c.observer_bound);
    friction_ = f_hat_;
    filtered_ = integral_ = 0.0;

    current_ = clamp(currentFor(tau_desired + f_hat_), c.current_limit);
    last_torque_ = torqueFor(current_);  // what the clamp let through
    return current_;
  }

  static constexpr double VELOCITY_FILTER = 0.3;
  static constexpr double PI = 3.14159265358979323846;
  static double sign(double x) { return (x > 0.0) - (x < 0.0); }
  static double clamp(double x, double limit) { return std::max(-limit, std::min(x, limit)); }

  CurrentTorqueJointConfig config_;
  double e1_ = 0.0, e2_ = 0.0, h1_ = 0.0, h2_ = 0.0;
  double integral_ = 0.0;
  double error_ = 0.0, filtered_ = 0.0, friction_ = 0.0, current_ = 0.0;
  double f_hat_ = 0.0, last_torque_ = 0.0, v_f_ = 0.0, acceleration_ = 0.0;
  bool started_ = false;
};

// The switch's check of one joint, held still in POSITION mode: the torque its sensor reads
// (measured) against the torque its motor current makes through torque_constant and
// gear_ratio (from_current), both N*m in the ROS sign.
enum class TorqueConstantCheck
{
  AGREES,
  DISAGREES,        // the ratio is out of bounds or the sign is opposite
  UNLOADED,         // under min_torque held: nothing to compare
  HELD_BY_FRICTION  // a load, but the current makes under min_torque of it
};

inline TorqueConstantCheck checkTorqueConstant(
  double measured, double from_current, double min_torque, double ratio_bound)
{
  if (std::abs(measured) < min_torque)
  {
    return TorqueConstantCheck::UNLOADED;
  }
  // A joint that settled while lowering carries its load on the gearbox's friction and draws
  // ~no current: joint_3 held 11.6 N*m on 0.04 A (2026-10-27). Its current is noise, so
  // neither the constant nor the sign shows; a wrong sign under a real current still does.
  if (std::abs(from_current) < min_torque)
  {
    return TorqueConstantCheck::HELD_BY_FRICTION;
  }
  const double ratio = from_current / measured;
  return (ratio >= 1.0 / ratio_bound && ratio <= ratio_bound) ? TorqueConstantCheck::AGREES
                                                              : TorqueConstantCheck::DISAGREES;
}

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__CURRENT_TORQUE_LOOP_HPP_
