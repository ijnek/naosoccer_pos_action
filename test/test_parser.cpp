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
#include "../src/parser.hpp"

TEST(TestParser, TestIncorrectPositionSize)
{
  std::vector<std::string> testString = {
    "! 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  EXPECT_FALSE(parseResult.successful);
}

TEST(TestParser, TestParsePosition)
{
  std::vector<std::string> testString = {
    "! 0 90 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  ASSERT_TRUE(parseResult.successful);
  ASSERT_EQ(parseResult.keyFrames.size(), 1);
  EXPECT_EQ(parseResult.keyFrames.at(0).t_ms, 300);
  EXPECT_NEAR(parseResult.keyFrames.at(0).positions.positions.at(1), 90 * M_PI / 180.0, 0.0001);
}

TEST(TestParser, TestParseStiffness)
{
  std::vector<std::string> testString = {
    "$ 0 0.2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    "! 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300",
  };

  auto parseResult = parser::parse(testString);
  ASSERT_TRUE(parseResult.successful);
  ASSERT_EQ(parseResult.keyFrames.size(), 1);
  EXPECT_EQ(parseResult.keyFrames.at(0).t_ms, 300);
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
  ASSERT_EQ(parseResult.keyFrames.size(), 2);
  EXPECT_EQ(parseResult.keyFrames.at(0).t_ms, 300);
  EXPECT_EQ(parseResult.keyFrames.at(1).t_ms, 600);
}
