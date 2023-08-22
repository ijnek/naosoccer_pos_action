// Copyright 2023 Kenji Brameld
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

#include <cmath>

#include "gtest/gtest.h"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "../src/parser.hpp"

using JointIndexes = nao_command_msgs::msg::JointIndexes;

TEST(TestParser, TestIncorrectPositionSize)
{
  std::vector<std::string> testString = {
    "! 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  EXPECT_FALSE(parseResult.successful);
}

TEST(TestParser, TestPositionsAndStiffnessesSize)
{
  std::vector<std::string> testString = {
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  EXPECT_EQ(parseResult.keyFrames.at(0).stiffnesses.stiffnesses.size(), JointIndexes::NUMJOINTS);
  EXPECT_EQ(parseResult.keyFrames.at(0).positions.positions.size(), JointIndexes::NUMJOINTS);
}

TEST(TestParser, TestIndexesFilled)
{
  std::vector<std::string> testString = {
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  const auto & key_frame = parseResult.keyFrames.at(0);

  // Position indexes
  ASSERT_EQ(key_frame.positions.indexes.size(), JointIndexes::NUMJOINTS);
  EXPECT_EQ(key_frame.positions.indexes.at(0), JointIndexes::HEADYAW);
  EXPECT_EQ(key_frame.positions.indexes.at(1), JointIndexes::HEADPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(2), JointIndexes::LSHOULDERPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(3), JointIndexes::LSHOULDERROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(4), JointIndexes::LELBOWYAW);
  EXPECT_EQ(key_frame.positions.indexes.at(5), JointIndexes::LELBOWROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(6), JointIndexes::LWRISTYAW);
  EXPECT_EQ(key_frame.positions.indexes.at(7), JointIndexes::LHIPYAWPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(8), JointIndexes::LHIPROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(9), JointIndexes::LHIPPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(10), JointIndexes::LKNEEPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(11), JointIndexes::LANKLEPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(12), JointIndexes::LANKLEROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(13), JointIndexes::RHIPROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(14), JointIndexes::RHIPPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(15), JointIndexes::RKNEEPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(16), JointIndexes::RANKLEPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(17), JointIndexes::RANKLEROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(18), JointIndexes::RSHOULDERPITCH);
  EXPECT_EQ(key_frame.positions.indexes.at(19), JointIndexes::RSHOULDERROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(20), JointIndexes::RELBOWYAW);
  EXPECT_EQ(key_frame.positions.indexes.at(21), JointIndexes::RELBOWROLL);
  EXPECT_EQ(key_frame.positions.indexes.at(22), JointIndexes::RWRISTYAW);
  EXPECT_EQ(key_frame.positions.indexes.at(23), JointIndexes::LHAND);
  EXPECT_EQ(key_frame.positions.indexes.at(24), JointIndexes::RHAND);

  // Stiffness indexes
  ASSERT_EQ(key_frame.stiffnesses.indexes.size(), JointIndexes::NUMJOINTS);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(0), JointIndexes::HEADYAW);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(1), JointIndexes::HEADPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(2), JointIndexes::LSHOULDERPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(3), JointIndexes::LSHOULDERROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(4), JointIndexes::LELBOWYAW);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(5), JointIndexes::LELBOWROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(6), JointIndexes::LWRISTYAW);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(7), JointIndexes::LHIPYAWPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(8), JointIndexes::LHIPROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(9), JointIndexes::LHIPPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(10), JointIndexes::LKNEEPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(11), JointIndexes::LANKLEPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(12), JointIndexes::LANKLEROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(13), JointIndexes::RHIPROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(14), JointIndexes::RHIPPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(15), JointIndexes::RKNEEPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(16), JointIndexes::RANKLEPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(17), JointIndexes::RANKLEROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(18), JointIndexes::RSHOULDERPITCH);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(19), JointIndexes::RSHOULDERROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(20), JointIndexes::RELBOWYAW);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(21), JointIndexes::RELBOWROLL);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(22), JointIndexes::RWRISTYAW);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(23), JointIndexes::LHAND);
  EXPECT_EQ(key_frame.stiffnesses.indexes.at(24), JointIndexes::RHAND);
}

TEST(TestParser, TestParsePosition)
{
  std::vector<std::string> testString = {
    "! 0 90 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  ASSERT_TRUE(parseResult.successful);
  ASSERT_EQ(parseResult.keyFrames.size(), 1u);
  EXPECT_EQ(parseResult.keyFrames.at(0).t_ms, 300u);
  EXPECT_NEAR(parseResult.keyFrames.at(0).positions.positions.at(1), 90 * M_PI / 180.0, 0.0001);
}

TEST(TestParser, TestStiffnessIsOneByDefault)
{
  std::vector<std::string> testString = {
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  const auto & keyFrameOneStiffnesses = parseResult.keyFrames.at(0).stiffnesses.stiffnesses;
  ASSERT_EQ(keyFrameOneStiffnesses.size(), JointIndexes::NUMJOINTS);
  for (unsigned i = 0; i < JointIndexes::NUMJOINTS; ++i) {
    EXPECT_EQ(keyFrameOneStiffnesses.at(i), 1.0);
  }
}

TEST(TestParser, TestParseStiffness)
{
  std::vector<std::string> testString = {
    "$ 0 0.2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  ASSERT_TRUE(parseResult.successful);
  ASSERT_EQ(parseResult.keyFrames.size(), 1u);
  EXPECT_EQ(parseResult.keyFrames.at(0).t_ms, 300u);
  EXPECT_NEAR(parseResult.keyFrames.at(0).stiffnesses.stiffnesses.at(1), 0.2, 0.0001);
}

TEST(TestParser, TestDuration)
{
  std::vector<std::string> testString = {
    "Keyframe 1",
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
    "Keyframe 2",
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  ASSERT_TRUE(parseResult.successful);
  ASSERT_EQ(parseResult.keyFrames.size(), 2u);
  EXPECT_EQ(parseResult.keyFrames.at(0).t_ms, 300u);
  EXPECT_EQ(parseResult.keyFrames.at(1).t_ms, 600u);
}
