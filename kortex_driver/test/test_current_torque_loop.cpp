// Copyright (c) 2026 Exclaim Robotics
// Licensed under the Apache License, Version 2.0 (see LICENSE).
//
// torque_control: current's torque loop and settings file, without the Kortex API or a robot.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "kortex_driver/current_torque_config.hpp"
#include "kortex_driver/current_torque_loop.hpp"

using kortex_driver::CurrentTorqueJointConfig;
using kortex_driver::CurrentTorqueLoop;
using kortex_driver::parseCurrentTorqueConfig;

namespace
{
constexpr double DT = 0.001;

CurrentTorqueJointConfig bigJoint(double lambda = 0.0)
{
  CurrentTorqueJointConfig c;
  c.lambda = lambda;
  c.torque_constant = 0.11;
  c.gear_ratio = 100.0;
  c.current_limit = 4.4;
  c.integral_bound = 2.0;
  return c;
}

const std::vector<double> LIMITS = {39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
}  // namespace

TEST(CurrentTorqueLoop, FilterHasUnityGainAtDc)
{
  CurrentTorqueLoop loop(bigJoint());
  for (int k = 0; k < 20000; ++k)
  {
    loop.update(3.0, 2.0, 0.0, DT);  // e = 1 N*m, held
  }
  EXPECT_NEAR(loop.filtered(), 1.0, 1e-3);
}

TEST(CurrentTorqueLoop, FeedforwardIsTheTorqueThroughTheConstants)
{
  CurrentTorqueLoop loop(bigJoint());  // lambda 0, no friction, no integral
  EXPECT_NEAR(loop.update(11.0, 0.0, 0.0, DT), 1.0, 1e-12);  // 11 N*m / (0.11 * 100)
  EXPECT_NEAR(loop.currentFor(-5.5), -0.5, 1e-12);
  EXPECT_NEAR(loop.torqueFor(2.0), 22.0, 1e-12);
}

TEST(CurrentTorqueLoop, ATorqueErrorAddsLambdaTimesItAtSteadyState)
{
  CurrentTorqueLoop loop(bigJoint(7.0));
  double current = 0.0;
  for (int k = 0; k < 20000; ++k)
  {
    current = loop.update(10.0, 9.0, 0.0, DT);
  }
  EXPECT_NEAR(current, (10.0 + 7.0 * 1.0) / 11.0, 1e-3);
}

TEST(CurrentTorqueLoop, TheCurrentIsClampedBothWays)
{
  CurrentTorqueLoop loop(bigJoint());
  EXPECT_DOUBLE_EQ(loop.update(1000.0, 0.0, 0.0, DT), 4.4);
  EXPECT_DOUBLE_EQ(loop.update(-1000.0, 0.0, 0.0, DT), -4.4);
}

TEST(CurrentTorqueLoop, FrictionMovingCoulombAndViscousAtRestStictionOnTheError)
{
  auto c = bigJoint();
  c.coulomb = 2.0;
  c.viscous = 1.5;
  c.stiction = 2.5;
  CurrentTorqueLoop loop(c);
  loop.update(0.0, 0.0, 0.2, DT);  // moving +
  EXPECT_NEAR(loop.friction(), 2.0 + 1.5 * 0.2, 1e-12);
  loop.update(0.0, 0.0, -0.2, DT);  // moving -
  EXPECT_NEAR(loop.friction(), -2.0 - 1.5 * 0.2, 1e-12);
  loop.update(0.0, 1.0, 0.0, DT);  // at rest, e = -1 past the threshold: the way e points
  EXPECT_NEAR(loop.friction(), -2.5, 1e-12);
  loop.update(0.0, 0.2, 0.0, DT);  // at rest, small error: nothing
  EXPECT_EQ(loop.friction(), 0.0);
}

TEST(CurrentTorqueLoop, TheIntegralSettlesAtGainTimesLeakTimesTheErrorAndIsBounded)
{
  auto c = bigJoint();
  c.integral_gain = 5.0;
  c.integral_leak = 0.1;
  CurrentTorqueLoop loop(c);
  for (int k = 0; k < 5000; ++k)
  {
    loop.update(1.0, 0.0, 0.0, DT);  // e = 1
  }
  EXPECT_NEAR(loop.integral(), 0.5, 1e-6);  // 5 * 0.1 * 1, under the bound 2
  for (int k = 0; k < 5000; ++k)
  {
    loop.update(10.0, 0.0, 0.0, DT);  // e = 10: would be 5
  }
  EXPECT_DOUBLE_EQ(loop.integral(), 2.0);
  loop.reset();
  EXPECT_EQ(loop.integral(), 0.0);
}

TEST(CurrentTorqueLoop, TheLoopTakesOutWhatFrictionLosesOnAStuckJoint)
{
  // The gearbox loses 2 N*m before the sensor on a joint that does not move: feedforward alone
  // leaves the 2 N*m short; lambda 7 leaves 2 / (1 + 7).
  const auto settle = [](double lambda)
  {
    CurrentTorqueLoop loop(bigJoint(lambda));
    double measured = 0.0;
    for (int k = 0; k < 20000; ++k)
    {
      const double current = loop.update(10.0, measured, 0.0, DT);
      measured = loop.torqueFor(current) - 2.0;
    }
    return 10.0 - measured;
  };
  EXPECT_NEAR(settle(0.0), 2.0, 1e-6);
  EXPECT_NEAR(settle(7.0), 2.0 / 8.0, 1e-3);
}

TEST(CurrentTorqueConfig, TheShippedFileParses)
{
  std::ifstream file(std::string(KORTEX_DRIVER_CONFIG_DIR) + "/current_torque.yaml");
  ASSERT_TRUE(file.good());
  std::stringstream text;
  text << file.rdbuf();
  const auto cfg = parseCurrentTorqueConfig(text.str(), LIMITS);
  ASSERT_EQ(cfg.joints.size(), 6u);
  EXPECT_DOUBLE_EQ(cfg.joints[0].torque_constant, 0.11);
  EXPECT_DOUBLE_EQ(cfg.joints[5].torque_constant, 0.076);
  EXPECT_DOUBLE_EQ(cfg.joints[3].lambda, 7.0);
  EXPECT_DOUBLE_EQ(cfg.joints[2].coulomb, 0.0);  // friction off until identified
  EXPECT_DOUBLE_EQ(cfg.joints[2].integral_gain, 0.0);
  EXPECT_DOUBLE_EQ(cfg.joints[0].integral_bound, 0.05 * 39.0);  // the default
  EXPECT_DOUBLE_EQ(cfg.speed_limit, 1.0);
}

TEST(CurrentTorqueConfig, OneValueForAllAndDefaults)
{
  const auto cfg = parseCurrentTorqueConfig(
    "lambda: 7\ntorque_constant: 0.1\ncurrent_limit: 2\nfriction: {viscous: 0.5}\n", LIMITS);
  for (const auto & j : cfg.joints)
  {
    EXPECT_DOUBLE_EQ(j.lambda, 7.0);
    EXPECT_DOUBLE_EQ(j.gear_ratio, 100.0);
    EXPECT_DOUBLE_EQ(j.viscous, 0.5);
    EXPECT_DOUBLE_EQ(j.coulomb, 0.0);
  }
}

TEST(CurrentTorqueConfig, WhatIsMissingOrWrongIsRefused)
{
  const auto refused = [](const std::string & yaml)
  {
    try
    {
      parseCurrentTorqueConfig(yaml, LIMITS);
    }
    catch (const std::runtime_error &)
    {
      return true;
    }
    return false;
  };
  EXPECT_TRUE(refused("lambda: 7\ncurrent_limit: 2\n"));                       // no torque_constant
  EXPECT_TRUE(refused("lambda: 7\ntorque_constant: 0.1\n"));                   // no current_limit
  EXPECT_TRUE(refused("lambda: [7, 7]\ntorque_constant: 0.1\ncurrent_limit: 2\n"));  // 2 for 6
  EXPECT_TRUE(refused("lambda: 7\ntorque_constant: 0\ncurrent_limit: 2\n"));
  EXPECT_TRUE(refused("lambda: 7\ntorque_constant: 0.1\ncurrent_limit: 2\nfriction: {coulomb: -1}\n"));
  EXPECT_TRUE(refused("lambda: 7\ntorque_constant: 0.1\ncurrent_limit: 2\nsafety: {kt_check_ratio: 1}\n"));
  EXPECT_TRUE(refused("lambda: [7, 7\n"));                                     // not YAML
  EXPECT_FALSE(refused("lambda: 7\ntorque_constant: 0.1\ncurrent_limit: 2\n"));
}

TEST(TorqueConstantCheck, AJointHeldByFrictionIsNotCheckedAWrongSignUnderCurrentFails)
{
  using kortex_driver::checkTorqueConstant;
  using kortex_driver::TorqueConstantCheck;
  // Settled lifting: the current makes 9.3 of the 12.0 N*m held (joint_3, 2026-10-27).
  EXPECT_EQ(checkTorqueConstant(12.0, 9.26, 1.0, 1.6), TorqueConstantCheck::AGREES);
  // Settled lowering: friction holds it, 0.04 A of noise either way (joint_3, joint_1).
  EXPECT_EQ(checkTorqueConstant(10.7, 0.13, 1.0, 1.6), TorqueConstantCheck::HELD_BY_FRICTION);
  EXPECT_EQ(checkTorqueConstant(1.26, -0.13, 1.0, 1.6), TorqueConstantCheck::HELD_BY_FRICTION);
  // A wrong sign, or a constant far off, under a current that carries the load.
  EXPECT_EQ(checkTorqueConstant(12.0, -9.0, 1.0, 1.6), TorqueConstantCheck::DISAGREES);
  EXPECT_EQ(checkTorqueConstant(12.0, 4.0, 1.0, 1.6), TorqueConstantCheck::DISAGREES);
  EXPECT_EQ(checkTorqueConstant(5.0, 9.0, 1.0, 1.6), TorqueConstantCheck::DISAGREES);
  // Too little held to tell anything.
  EXPECT_EQ(checkTorqueConstant(0.3, 0.0, 1.0, 1.6), TorqueConstantCheck::UNLOADED);
}

TEST(CurrentTorqueLoop, SmoothFrictionHasNoStepAndMatchesTheSwitchedFarFromZero)
{
  auto c = bigJoint();
  c.coulomb = 3.0;
  c.viscous = 1.0;
  c.stiction = 2.0;
  c.velocity_scale = 0.02;
  CurrentTorqueLoop loop(c);
  // Across v = 0 and across the old threshold, at a fixed error: no step bigger than the slope.
  double last = 0.0;
  double biggest = 0.0;
  for (int k = -1000; k <= 1000; ++k)
  {
    const double v = 1e-4 * k;  // -0.1 .. 0.1 rad/s
    loop.update(0.0, -1.0, v, DT);
    if (k > -1000)
    {
      biggest = std::max(biggest, std::abs(loop.friction() - last));
    }
    last = loop.friction();
  }
  // The steepest slope is (coulomb + stiction * tanh(e / error_threshold)) / velocity_scale per
  // rad/s, stiction fading out as Coulomb fades in: ~246 * 1e-4 = 0.025 N*m a step. The switched
  // model steps 2 x coulomb = 6 N*m at the threshold.
  EXPECT_LT(biggest, 0.03);
  // Moving well past the scale: the switched model's Coulomb and viscous.
  loop.update(0.0, 0.0, 0.2, DT);
  EXPECT_NEAR(loop.friction(), 3.0 + 0.2, 1e-6);
  // At rest: stiction the way the error points, smoothly in the error.
  loop.update(0.0, 2.0, 0.0, DT);  // e = -2
  EXPECT_NEAR(loop.friction(), -2.0 * std::tanh(2.0 / 0.5), 1e-12);
  loop.update(0.0, 0.05, 0.0, DT);  // a small error: a small push, not nothing and not 2 N*m
  EXPECT_NEAR(loop.friction(), -2.0 * std::tanh(0.1), 1e-12);
}

TEST(CurrentTorqueConfig, VelocityScaleIsOffByDefaultAndNotNegative)
{
  const std::string base = "lambda: 7\ntorque_constant: 0.11\ncurrent_limit: 2\n";
  EXPECT_EQ(parseCurrentTorqueConfig(base, LIMITS).joints[0].velocity_scale, 0.0);
  const auto smooth =
    parseCurrentTorqueConfig(base + "friction:\n  velocity_scale: 0.02\n", LIMITS);
  EXPECT_EQ(smooth.joints[5].velocity_scale, 0.02);
  EXPECT_THROW(
    parseCurrentTorqueConfig(base + "friction:\n  velocity_scale: -0.1\n", LIMITS),
    std::runtime_error);
}

namespace
{
// A rigid joint: rotor (J_m) and link (J_l) as one, Coulomb friction on the rotor's side, the
// sensor between gearbox and link reading J_l * a. The observer's residual is then the friction.
struct RigidJoint
{
  double J_m = 0.40, J_l = 1.5, coulomb = 5.0;
  double v = 0.0, q = 0.0, a = 0.0;
  double sensor() const { return J_l * a; }
  void step(double tau_motor, double dt)
  {
    const double f = v > 0.0 ? coulomb : v < 0.0 ? -coulomb : 0.0;
    a = (tau_motor - f) / (J_m + J_l);
    v += a * dt;
    q += v * dt;
  }
};

CurrentTorqueJointConfig observerJoint(double bandwidth = 10.0)
{
  auto c = bigJoint();
  c.observer_bandwidth = bandwidth;
  c.observer_bound = 20.0;
  c.rotor_inertia = 0.40;
  return c;
}
}  // namespace

TEST(CurrentTorqueLoop, TheObserverFindsTheFrictionOfAMovingJoint)
{
  CurrentTorqueLoop loop(observerJoint());
  RigidJoint joint;
  joint.v = 0.05;  // moving: the observer integrates
  for (int k = 0; k < 2000; ++k)
  {
    // a soft speed loop around 0.05 rad/s stands in for crisp
    const double tau_d = 20.0 * (0.05 - joint.v);
    const double current = loop.update(tau_d, joint.sensor(), joint.v, DT);
    joint.step(loop.torqueFor(current), DT);
  }
  EXPECT_NEAR(loop.friction(), 5.0, 0.5);       // the estimate is the friction
  EXPECT_NEAR(joint.v, 0.05, 0.005);            // so the speed loop no longer carries it
  EXPECT_EQ(loop.feedback(), 0.0);              // no lambda * H under the observer
}

TEST(CurrentTorqueLoop, TheObserverHoldsAStillJointInsideTheDeadband)
{
  // Stuck, a small error: what it would measure is just the torque applied, and integrating
  // that hunts. The gate is smooth (tanh((e / deadband)^4)): the estimate barely moves.
  CurrentTorqueLoop loop(observerJoint());
  for (int k = 0; k < 2000; ++k)
  {
    loop.update(3.0, 2.9, 0.0, DT);  // e = 0.1 N*m, the joint still
  }
  EXPECT_LT(std::abs(loop.friction()), 0.05);  // ungated, it would be ~3 N*m
  // Pushed past the deadband, it integrates even while still: breakaway.
  for (int k = 0; k < 2000; ++k)
  {
    loop.update(3.0, 1.0, 0.0, DT);  // e = 2 N*m
  }
  EXPECT_GT(loop.friction(), 1.0);
}

TEST(CurrentTorqueLoop, TheObserverIsBoundedAndTheCurrentClamped)
{
  auto c = observerJoint();
  c.observer_bound = 4.0;
  CurrentTorqueLoop loop(c);
  for (int k = 0; k < 5000; ++k)
  {
    loop.update(30.0, -30.0, 0.1, DT);  // a huge residual, moving
  }
  EXPECT_NEAR(std::abs(loop.friction()), 4.0, 1e-9);
  EXPECT_LE(std::abs(loop.current()), c.current_limit + 1e-12);
}

TEST(CurrentTorqueConfig, TheObserverIsOffByDefaultAndChecked)
{
  const std::string base = "lambda: 7\ntorque_constant: 0.11\ncurrent_limit: 2\n";
  EXPECT_EQ(parseCurrentTorqueConfig(base, LIMITS).joints[0].observer_bandwidth, 0.0);
  const std::string obs =
    "observer:\n  bandwidth: [5, 5, 5, 0, 0, 0]\n  bound: 20\n  rotor_inertia: 0.4\n";
  const auto on = parseCurrentTorqueConfig(base + obs, LIMITS);
  EXPECT_EQ(on.joints[0].observer_bandwidth, 5.0);
  EXPECT_EQ(on.joints[3].observer_bandwidth, 0.0);
  EXPECT_EQ(on.joints[2].observer_deadband, 0.5);
  EXPECT_THROW(
    parseCurrentTorqueConfig(base + "observer:\n  bandwidth: 60\n  bound: 20\n", LIMITS),
    std::runtime_error);
  EXPECT_THROW(  // on, but no bound: it could never reach the breakaway
    parseCurrentTorqueConfig(base + "observer:\n  bandwidth: 5\n", LIMITS), std::runtime_error);
}

TEST(CurrentTorqueConfig, TheShippedFileKeepsTheObserverOff)
{
  std::ifstream file(std::string(KORTEX_DRIVER_CONFIG_DIR) + "/current_torque.yaml");
  std::stringstream text;
  text << file.rdbuf();
  const auto config = parseCurrentTorqueConfig(text.str(), LIMITS);
  for (const auto & j : config.joints)
  {
    EXPECT_EQ(j.observer_bandwidth, 0.0);
  }
}

namespace
{
// A joint on a spring (crisp's stiffness, its own damping left out), set swinging, no friction:
// the largest |q| over the last second of `seconds`.
double swingAfter(CurrentTorqueJointConfig c, double seconds)
{
  CurrentTorqueLoop loop(c);
  RigidJoint joint;
  joint.coulomb = 0.0;
  joint.q = 0.02;
  double peak = 0.0;
  const int steps = static_cast<int>(seconds / DT);
  for (int k = 0; k < steps; ++k)
  {
    const double current = loop.update(-200.0 * joint.q, joint.sensor(), joint.v, DT);
    joint.step(loop.torqueFor(current), DT);
    if (k >= steps - 1000)
    {
      peak = std::max(peak, std::abs(joint.q));
    }
  }
  return peak;
}
}  // namespace

TEST(CurrentTorqueLoop, TheDamperTakesOutASwing)
{
  // 1.9 kg m^2 on 200 N*m/rad: 1.6 Hz. 3 N*m s/rad is 8% of critical, a decay of 0.8/s.
  auto c = bigJoint();
  const double undamped = swingAfter(c, 3.0);
  EXPECT_GT(undamped, 0.019);  // undamped, it still swings
  c.damping = 3.0;
  EXPECT_LT(swingAfter(c, 3.0), 0.25 * undamped);
  c.observer_bandwidth = 5.0;  // and under the observer
  c.observer_bound = 20.0;
  c.rotor_inertia = 0.40;
  EXPECT_LT(swingAfter(c, 3.0), 0.25 * undamped);
}

TEST(CurrentTorqueLoop, TheDamperIsMinusDampingTimesTheVelocityAndNothingAtRest)
{
  auto c = bigJoint();
  c.damping = 2.0;
  CurrentTorqueLoop loop(c);
  EXPECT_NEAR(loop.update(11.0, 0.0, 0.0, DT), 1.0, 1e-12);  // still: the feedforward alone
  EXPECT_EQ(loop.damper(), 0.0);
  for (int k = 0; k < 100; ++k)
  {
    loop.update(11.0, 0.0, 0.5, DT);  // the filter settled
  }
  EXPECT_NEAR(loop.damper(), -1.0, 1e-9);
  EXPECT_NEAR(loop.current(), 10.0 / 11.0, 1e-9);
}

TEST(CurrentTorqueConfig, TheDamperIsOffByDefaultAndChecked)
{
  const std::string base = "lambda: 7\ntorque_constant: 0.11\ncurrent_limit: 2\n";
  EXPECT_EQ(parseCurrentTorqueConfig(base, LIMITS).joints[0].damping, 0.0);
  const auto on = parseCurrentTorqueConfig(base + "damping: [1, 1, 1, 0.3, 0.3, 0.3]\n", LIMITS);
  EXPECT_EQ(on.joints[0].damping, 1.0);
  EXPECT_EQ(on.joints[5].damping, 0.3);
  EXPECT_THROW(parseCurrentTorqueConfig(base + "damping: -1\n", LIMITS), std::runtime_error);
}
