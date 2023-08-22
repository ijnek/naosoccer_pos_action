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

#include "parser.hpp"

#include <cmath>
#include <iterator>
#include <iostream>

#include "rclcpp/logging.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "indexes.hpp"

// +2 because there is the "!" at the start, and the duration at the end
#define POSITIONS_SIZE (nao_command_msgs::msg::JointIndexes::NUMJOINTS + 2)
// +1 because there is the "$" at the start
#define STIFFNESSES_SIZE (nao_command_msgs::msg::JointIndexes::NUMJOINTS + 1)

const auto stiffnessMax = nao_command_msgs::msg::JointStiffnesses()
  .set__indexes(indexes::indexes)
  .set__stiffnesses({1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});

namespace parser
{

static rclcpp::Logger logger = rclcpp::get_logger("parser");

// Forward declaration
std::vector<std::string> split(const std::string & line);

ParseResult parse(const std::vector<std::string> & in)
{
  ParseResult parseResult;

  unsigned keyFrameTime = 0;
  auto jointStiffnesses = stiffnessMax;

  for (const auto & line : in) {
    if (line.front() == '!') {
      RCLCPP_DEBUG_STREAM(logger, "Position: " << line);
      auto splitted_line = split(line);

      // Check size
      if (splitted_line.size() != POSITIONS_SIZE) {
        RCLCPP_ERROR_STREAM(
          logger,
          "pos file line has " << splitted_line.size() << " elements, but expected " <<
            POSITIONS_SIZE);
        parseResult.successful = false;
        return parseResult;
      }

      // Convert to data type
      nao_command_msgs::msg::JointPositions jointPositions;
      jointPositions.indexes = indexes::indexes;
      for (unsigned int i = 1; i < nao_command_msgs::msg::JointIndexes::NUMJOINTS + 1; ++i) {
        std::string position_deg_string = splitted_line.at(i);

        try {
          float position_deg = std::stof(position_deg_string);
          float position_rad = position_deg * M_PI / 180;
          jointPositions.positions.push_back(position_rad);
        } catch (std::invalid_argument &) {
          RCLCPP_ERROR(
            logger,
            ("joint value '" + position_deg_string +
            "' is not a valid joint value (cannot be converted to float)").c_str());
          parseResult.successful = false;
          return parseResult;
        }
      }

      std::string duration_string = splitted_line.back();
      try {
        int duration = std::stoi(duration_string);
        keyFrameTime += duration;
      } catch (std::invalid_argument &) {
        RCLCPP_ERROR(
          logger,
          ("duration '" + duration_string +
          "' is not a valid duration value (cannot be converted to int)").c_str());
        parseResult.successful = false;
        return parseResult;
      }

      // Append to vector
      parseResult.keyFrames.push_back(KeyFrame{keyFrameTime, jointPositions, jointStiffnesses});

      // Reset jointStiffnesses to max
      jointStiffnesses = stiffnessMax;
    } else if (line.front() == '$') {
      RCLCPP_DEBUG_STREAM(logger, "Stiffness: " << line);
      auto splitted_line = split(line);

      // Check size
      if (splitted_line.size() != STIFFNESSES_SIZE) {
        RCLCPP_ERROR_STREAM(
          logger,
          "pos file line has " << splitted_line.size() << " elements, but expected " <<
            STIFFNESSES_SIZE);
        parseResult.successful = false;
        return parseResult;
      }

      // Convert to data type
      jointStiffnesses.stiffnesses.clear();
      for (unsigned int i = 1; i < nao_command_msgs::msg::JointIndexes::NUMJOINTS + 1; ++i) {
        std::string stiffness_string = splitted_line.at(i);

        try {
          float stiffness_float = std::stof(stiffness_string);
          jointStiffnesses.stiffnesses.push_back(stiffness_float);
        } catch (std::invalid_argument &) {
          RCLCPP_ERROR(
            logger,
            ("stiffness value '" + stiffness_string +
            "' is not a valid stiffness value (cannot be converted to float)").c_str());
          parseResult.successful = false;
          return parseResult;
        }
      }
    } else {
      RCLCPP_DEBUG_STREAM(logger, "Ignoring: " << line);
    }
  }

  parseResult.successful = true;
  return parseResult;
}

std::vector<std::string> split(const std::string & line)
{
  std::istringstream ss(line);
  return std::vector<std::string>{
    std::istream_iterator<std::string>{ss},
    std::istream_iterator<std::string>()};
}

}  // namespace parser
