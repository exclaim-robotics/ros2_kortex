// Copyright (c) 2026 Exclaim Robotics
// Licensed under the Apache License, Version 2.0 (see LICENSE).
//
// torque_control: current's torque loop and settings file, without the Kortex API or a robot.

#include <gtest/gtest.h>

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
