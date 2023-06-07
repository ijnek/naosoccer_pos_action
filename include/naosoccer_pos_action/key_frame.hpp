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

#ifndef KEY_FRAME_HPP_
#define KEY_FRAME_HPP_

#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"

class KeyFrame
{
public:
  KeyFrame(
    unsigned t_ms,
    const nao_command_msgs::msg::JointPositions &positions,
    const nao_command_msgs::msg::JointStiffnesses &stiffnesses)
    : t_ms(t_ms), positions(positions), stiffnesses(stiffnesses)
    {}

  unsigned t_ms;
  nao_command_msgs::msg::JointPositions positions;
  nao_command_msgs::msg::JointStiffnesses stiffnesses;
};

#endif  // KEY_FRAME_HPP_
