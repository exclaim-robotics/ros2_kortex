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

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "kortex_driver/torque_integral.hpp"

using kortex_driver::parseTorqueGains;
using kortex_driver::parseTorqueIntegral;
using kortex_driver::TorqueIntegral;

namespace
{
constexpr double DT = 0.001;

// An actuator that delivers `delivered` of what it is sent, as the torque sensor reads it: the
// static error of TORQUE_HIGH_VELOCITY under load. Runs `seconds` at a constant command and
// returns the last measured torque.
double settle(
  const TorqueIntegral & t, double & integral, double command, double delivered,
  double effort_limit, double seconds)
{
  double measured = 0.0;
  for (int k = 0; k < static_cast<int>(seconds / DT); k++)
  {
    measured = delivered * t.step(integral, command, measured, effort_limit, DT);
  }
  return measured;
}
}  // namespace

TEST(TorqueIntegral, OffSendsTheCommand)
{
  const TorqueIntegral off;
  double integral = 0.0;
  EXPECT_FALSE(off.on());
  EXPECT_DOUBLE_EQ(off.step(integral, 5.0, 1.0, 39.0, DT), 5.0);
  EXPECT_DOUBLE_EQ(integral, 0.0);
  EXPECT_FALSE((TorqueIntegral{2.0, 0.1, 0.0}).on());  // a zero limit is off too
}

TEST(TorqueIntegral, Parse)
{
  EXPECT_FALSE(parseTorqueIntegral("").on());
  EXPECT_FALSE(parseTorqueIntegral("  ").on());
  const auto t = parseTorqueIntegral(" 2.0 0.1 8 ");
  EXPECT_DOUBLE_EQ(t.gain, 2.0);
  EXPECT_DOUBLE_EQ(t.leak, 0.1);
  EXPECT_DOUBLE_EQ(t.limit, 8.0);
  EXPECT_THROW(parseTorqueIntegral("2.0 0.1"), std::invalid_argument);
  EXPECT_THROW(parseTorqueIntegral("2.0 0.1 8 1"), std::invalid_argument);
  EXPECT_THROW(parseTorqueIntegral("a b c"), std::invalid_argument);
  EXPECT_THROW(parseTorqueIntegral("-1 0.1 8"), std::invalid_argument);
  EXPECT_THROW(parseTorqueIntegral("2 0.1 nan"), std::invalid_argument);
}

TEST(TorqueIntegral, TakesOutAStaticError)
{
  // joint_2's case: 18 N*m commanded, the actuator a third short.
  double integral = 0.0;
  const TorqueIntegral pure{2.0, 0.0, 10.0};
  EXPECT_NEAR(settle(pure, integral, -18.0, 2.0 / 3.0, 39.0, 20.0), -18.0, 0.01);
  EXPECT_NEAR(integral, -9.0, 0.02);  // the third it was short of, at 18 / (2/3)

  // With the leak, 1 / (1 + K_I / theta * delivered) of the error is left: e = c (1 - d) - d I,
  // I = (K_I / theta) e.
  integral = 0.0;
  const TorqueIntegral leaky{2.0, 0.1, 10.0};
  const double c = -18.0, d = 2.0 / 3.0;
  const double left = c * (1.0 - d) / (1.0 + (2.0 / 0.1) * d);
  EXPECT_NEAR(c - settle(leaky, integral, c, d, 39.0, 120.0), left, 0.01);
}

TEST(TorqueIntegral, BoundedByItsLimit)
{
  // Nothing reaches the sensor (a joint pushing on nothing, or a dead sensor): the integral
  // stops at its limit, and what is sent at the command plus the limit.
  const TorqueIntegral t{5.0, 0.0, 4.0};
  double integral = 0.0;
  double sent = 0.0;
  for (int k = 0; k < 10000; k++)
  {
    sent = t.step(integral, 3.0, 0.0, 39.0, DT);
  }
  EXPECT_DOUBLE_EQ(integral, 4.0);
  EXPECT_DOUBLE_EQ(sent, 7.0);
}

TEST(TorqueIntegral, NoWindUpAtTheEffortLimit)
{
  // The command near the effort limit, the actuator short: what is sent stops at the limit and
  // the integral stops growing there, instead of running to its own limit behind the clamp.
  const TorqueIntegral t{5.0, 0.0, 8.0};
  double integral = 0.0;
  double sent = 0.0;
  for (int k = 0; k < 10000; k++)
  {
    sent = t.step(integral, 8.0, 4.0, 9.0, DT);
  }
  EXPECT_DOUBLE_EQ(sent, 9.0);
  EXPECT_NEAR(integral, 1.0, 0.03);
  // Back off: it unwinds at once, not from 8 N*m.
  sent = t.step(integral, 2.0, 4.0, 9.0, DT);
  EXPECT_LT(sent, 3.1);
}

TEST(TorqueIntegral, NoTimeNoChange)
{
  const TorqueIntegral t{2.0, 0.1, 8.0};
  double integral = 1.5;
  EXPECT_DOUBLE_EQ(t.step(integral, 3.0, 0.0, 39.0, 0.0), 4.5);
  EXPECT_DOUBLE_EQ(integral, 1.5);
}

TEST(TorqueIntegral, GainDividesTheCommand)
{
  // Off: only the gain.
  const TorqueIntegral off;
  double integral = 0.0;
  EXPECT_DOUBLE_EQ(off.step(integral, -17.2, 0.0, 39.0, DT, 0.86), -20.0);
  EXPECT_DOUBLE_EQ(off.step(integral, 38.0, 0.0, 39.0, DT, 0.5), 39.0);  // still clamped
}

TEST(TorqueIntegral, GainLeavesTheIntegralOnlyTheRest)
{
  // joint_2's actuator: 0.86 of what it is sent. With that gain the first cycle already
  // delivers the command, and the integral has nothing to do (but the one cycle before the
  // sensor reads it, which leaks away).
  const TorqueIntegral t{2.0, 0.1, 10.0};
  double integral = 0.0, measured = 0.0;
  for (int k = 0; k < 3000; k++)
  {
    measured = 0.86 * t.step(integral, -20.0, measured, 39.0, DT, 0.86);
  }
  EXPECT_NEAR(measured, -20.0, 1e-3);
  EXPECT_NEAR(integral, 0.0, 1e-3);
  // A gain a little off leaves the integral the difference.
  integral = 0.0;
  measured = 0.0;
  for (int k = 0; k < 30000; k++)
  {
    measured = 0.80 * t.step(integral, -20.0, measured, 39.0, DT, 0.86);
  }
  const double left = -20.0 * (1.0 - 0.80 / 0.86) / (1.0 + (2.0 / 0.1) * 0.80);
  EXPECT_NEAR(-20.0 - measured, left, 0.01);
}

TEST(TorqueIntegral, ParseGains)
{
  EXPECT_EQ(parseTorqueGains("", 6), std::vector<double>(6, 1.0));
  EXPECT_EQ(parseTorqueGains(" ", 6), std::vector<double>(6, 1.0));
  const std::vector<double> want{0.86, 0.86, 0.86, 1.0, 1.0, 1.0};
  EXPECT_EQ(parseTorqueGains("0.86 0.86 0.86 1 1 1", 6), want);
  EXPECT_EQ(parseTorqueGains("0.86,0.86,0.86,1,1,1", 6), want);
  EXPECT_THROW(parseTorqueGains("0.86 0.86", 6), std::invalid_argument);
  EXPECT_THROW(parseTorqueGains("1 1 1 1 1 1 1", 6), std::invalid_argument);
  EXPECT_THROW(parseTorqueGains("1 1 1 1 1 0", 6), std::invalid_argument);
  EXPECT_THROW(parseTorqueGains("1 1 1 1 1 2.5", 6), std::invalid_argument);
  EXPECT_THROW(parseTorqueGains("1 1 1 1 1 x", 6), std::invalid_argument);
}
