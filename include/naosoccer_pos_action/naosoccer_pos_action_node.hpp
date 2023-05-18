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

#ifndef NAOSOCCER_POS_ACTION__NAOSOCCER_POS_ACTION_NODE_HPP_
#define NAOSOCCER_POS_ACTION__NAOSOCCER_POS_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "naosoccer_pos_action/key_frame.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/bool.hpp"


namespace naosoccer_pos_action
{

class NaosoccerPosActionNode : public rclcpp::Node
{
public:
  explicit NaosoccerPosActionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~NaosoccerPosActionNode();

private:
  std::string getDefaultFullFilePath();
  std::vector<std::string> readLines(std::ifstream & ifstream);
  void calculateEffectorJoints(nao_sensor_msgs::msg::JointPositions & sensor_joints);
  const KeyFrame & findPreviousKeyFrame(int time_ms);
  const KeyFrame & findNextKeyFrame(int time_ms);
  bool posFinished(int time_ms);

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start;
  rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joint_positions;
  rclcpp::Publisher<nao_command_msgs::msg::JointStiffnesses>::SharedPtr pub_joint_stiffnesses;

  bool fileSuccessfullyRead = false;
  std::vector<KeyFrame> keyFrames;
  bool posInAction = false;
  bool firstTickSinceActionStarted = true;
  std::unique_ptr<KeyFrame> keyFrameStart;
  rclcpp::Time begin;
};

}  // namespace naosoccer_pos_action

#endif  // NAOSOCCER_POS_ACTION__NAOSOCCER_POS_ACTION_NODE_HPP_
