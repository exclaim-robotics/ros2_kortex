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

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/component_parser.hpp"
#include "kortex_driver/joint_calibration.hpp"
#include "kortex_driver/kortex_math_util.hpp"

using kortex_driver::JointCalibration;
using kortex_driver::parseJointCalibration;

namespace
{
// What the driver does with a reading: arm degrees in, ROS radians out.
double readPosition(const JointCalibration & cal, double raw_deg, double effort)
{
  return KortexMathUtil::wrapRadiansFromMinusPiToPi(
    cal.calibrated(KortexMathUtil::toRad(raw_deg), effort));
}
// ...and with a position command: ROS radians in, arm degrees out.
double commandDegrees(const JointCalibration & cal, double cmd_rad, double effort)
{
  return KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(cal.raw(cmd_rad, effort)));
}
// Two angles in degrees, equal modulo 360.
double degDiff(double a, double b) { return std::remainder(a - b, 360.0); }
}  // namespace

TEST(JointCalibration, DefaultIsInert)
{
  const JointCalibration cal;
  EXPECT_FALSE(cal.active());
  for (double deg : {0.0, 10.0, 179.9, 180.1, 359.0, -45.0})
  {
    const double effort = 12.5;
    EXPECT_DOUBLE_EQ(
      readPosition(cal, deg, effort),
      KortexMathUtil::wrapRadiansFromMinusPiToPi(KortexMathUtil::toRad(deg)));
    EXPECT_DOUBLE_EQ(
      commandDegrees(cal, KortexMathUtil::toRad(deg), effort),
      KortexMathUtil::wrapDegreesFromZeroTo360(deg));
  }
}

TEST(JointCalibration, ReadingIsRawPlusOffsetPlusComplianceTimesEffort)
{
  JointCalibration cal;
  cal.offset = 0.01;
  cal.compliance = 0.002;
  EXPECT_TRUE(cal.active());
  EXPECT_NEAR(readPosition(cal, 30.0, 5.0), KortexMathUtil::toRad(30.0) + 0.01 + 0.002 * 5.0, 1e-12);
  EXPECT_NEAR(
    readPosition(cal, -30.0, -7.0), KortexMathUtil::toRad(-30.0) + 0.01 - 0.002 * 7.0, 1e-12);
}

TEST(JointCalibration, CommandIsTheInverseAtTheSameEffort)
{
  JointCalibration cal;
  cal.offset = -0.02;
  cal.compliance = 0.0015;
  // Holding: commanding the position just read sends the raw position just read.
  for (double raw_deg : {0.5, 45.0, 130.0, 200.0, 315.0, 359.5})
  {
    for (double effort : {-20.0, 0.0, 3.3, 25.0})
    {
      const double q = readPosition(cal, raw_deg, effort);
      EXPECT_NEAR(degDiff(commandDegrees(cal, q, effort), raw_deg), 0.0, 1e-9)
        << raw_deg << " deg, " << effort << " N*m";
    }
  }
}

TEST(JointCalibration, CommandUsesTheEffortNow)
{
  JointCalibration cal;
  cal.compliance = 0.001;
  // A command of 0 rad with 10 N*m on the joint goes out as -0.01 rad raw.
  EXPECT_NEAR(degDiff(commandDegrees(cal, 0.0, 10.0), KortexMathUtil::toDeg(-0.01)), 0.0, 1e-9);
}

TEST(JointCalibration, EndlessJointWrapsAcrossPi)
{
  JointCalibration cal;
  cal.offset = 0.03;
  // Raw 179 deg + 0.03 rad (1.72 deg) is past pi: it comes out wrapped, as the raw reading would.
  const double q = readPosition(cal, 179.0, 0.0);
  EXPECT_LE(q, M_PI);
  EXPECT_NEAR(q, KortexMathUtil::toRad(179.0) + 0.03 - 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(degDiff(commandDegrees(cal, q, 0.0), 179.0), 0.0, 1e-9);
}

TEST(JointCalibration, NoEffortReadingYet)
{
  JointCalibration cal;
  cal.offset = 0.01;
  cal.compliance = 0.002;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(cal.correction(nan), 0.01);
  JointCalibration offset_only;
  offset_only.offset = 0.01;
  EXPECT_DOUBLE_EQ(offset_only.correction(std::numeric_limits<double>::infinity()), 0.01);
}

TEST(JointCalibration, ParsesTheJointParams)
{
  EXPECT_FALSE(parseJointCalibration({}).active());
  EXPECT_FALSE(parseJointCalibration({{"initial_value", "0.3"}}).active());
  const auto cal =
    parseJointCalibration({{"joint_offset", "-0.0123"}, {"joint_compliance", "1.5e-4 "}});
  EXPECT_DOUBLE_EQ(cal.offset, -0.0123);
  EXPECT_DOUBLE_EQ(cal.compliance, 1.5e-4);
  EXPECT_DOUBLE_EQ(parseJointCalibration({{"joint_offset", ""}}).offset, 0.0);
  EXPECT_THROW(parseJointCalibration({{"joint_offset", "abc"}}), std::invalid_argument);
  EXPECT_THROW(parseJointCalibration({{"joint_offset", "0.1rad"}}), std::invalid_argument);
  EXPECT_THROW(parseJointCalibration({{"joint_compliance", "nan"}}), std::invalid_argument);
}

TEST(JointCalibration, FromTheRos2ControlJointTag)
{
  // What kortex_description's kortex_joint_calibration macro emits: <param>s of the <joint>.
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="r">
  <link name="base"/><link name="l1"/><link name="l2"/>
  <joint name="joint_1" type="continuous">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="l1"/><child link="l2"/><axis xyz="0 0 1"/>
    <limit lower="-2.41" upper="2.41" effort="39" velocity="1.4"/>
  </joint>
  <ros2_control name="arm" type="system">
    <hardware><plugin>kortex_driver/KortexMultiInterfaceHardware</plugin></hardware>
    <joint name="joint_1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint_2">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.1</param></state_interface>
      <param name="joint_offset">-0.00964</param>
      <param name="joint_compliance">0.0002095</param>
    </joint>
  </ros2_control>
</robot>)";
  const auto hw = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(hw.size(), 1u);
  ASSERT_EQ(hw[0].joints.size(), 2u);
  EXPECT_FALSE(parseJointCalibration(hw[0].joints[0].parameters).active());
  const auto cal = parseJointCalibration(hw[0].joints[1].parameters);
  EXPECT_DOUBLE_EQ(cal.offset, -0.00964);
  EXPECT_DOUBLE_EQ(cal.compliance, 0.0002095);
}
