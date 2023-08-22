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

#ifndef INDEXES_HPP_
#define INDEXES_HPP_

#include <vector>

#include "nao_command_msgs/msg/joint_indexes.hpp"

namespace indexes
{

static const std::vector<uint8_t> indexes = {
  nao_command_msgs::msg::JointIndexes::HEADYAW,
  nao_command_msgs::msg::JointIndexes::HEADPITCH,
  nao_command_msgs::msg::JointIndexes::LSHOULDERPITCH,
  nao_command_msgs::msg::JointIndexes::LSHOULDERROLL,
  nao_command_msgs::msg::JointIndexes::LELBOWYAW,
  nao_command_msgs::msg::JointIndexes::LELBOWROLL,
  nao_command_msgs::msg::JointIndexes::LWRISTYAW,
  nao_command_msgs::msg::JointIndexes::LHIPYAWPITCH,
  nao_command_msgs::msg::JointIndexes::LHIPROLL,
  nao_command_msgs::msg::JointIndexes::LHIPPITCH,
  nao_command_msgs::msg::JointIndexes::LKNEEPITCH,
  nao_command_msgs::msg::JointIndexes::LANKLEPITCH,
  nao_command_msgs::msg::JointIndexes::LANKLEROLL,
  nao_command_msgs::msg::JointIndexes::RHIPROLL,
  nao_command_msgs::msg::JointIndexes::RHIPPITCH,
  nao_command_msgs::msg::JointIndexes::RKNEEPITCH,
  nao_command_msgs::msg::JointIndexes::RANKLEPITCH,
  nao_command_msgs::msg::JointIndexes::RANKLEROLL,
  nao_command_msgs::msg::JointIndexes::RSHOULDERPITCH,
  nao_command_msgs::msg::JointIndexes::RSHOULDERROLL,
  nao_command_msgs::msg::JointIndexes::RELBOWYAW,
  nao_command_msgs::msg::JointIndexes::RELBOWROLL,
  nao_command_msgs::msg::JointIndexes::RWRISTYAW,
  nao_command_msgs::msg::JointIndexes::LHAND,
  nao_command_msgs::msg::JointIndexes::RHAND};

}  // namespace indexes

#endif  // INDEXES_HPP_
