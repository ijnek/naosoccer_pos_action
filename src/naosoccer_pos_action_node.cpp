// Copyright 2021 Kenji Brameld
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

#include "naosoccer_pos_action/naosoccer_pos_action_node.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"

#include "indexes.hpp"
#include "parser.hpp"

namespace fs = boost::filesystem;

namespace naosoccer_pos_action
{

NaosoccerPosActionNode::NaosoccerPosActionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"NaosoccerPosActionNode", options}
{
  this->declare_parameter<std::string>("file", getDefaultFullFilePath());

  pub_joint_positions = create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 1);
  pub_joint_stiffnesses = create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
    "effectors/joint_stiffnesses", 1);

  sub_joint_states =
    create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 1,
    [this](nao_lola_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      if (posInAction) {
        calculateEffectorJoints(*sensor_joints);
      }
    });

  sub_start =
    create_subscription<std_msgs::msg::Bool>(
    "start_pos_action", 1,
    [this](std_msgs::msg::Bool::UniquePtr start_pos_action) {
      if (start_pos_action->data && fileSuccessfullyRead && !posInAction) {
        RCLCPP_DEBUG(this->get_logger(), "Starting Pos Action");
        begin = rclcpp::Node::now();
        posInAction = true;
        firstTickSinceActionStarted = true;
      }
    });

  std::string filePath;
  this->get_parameter("file", filePath);

  std::ifstream ifstream(filePath);
  if (ifstream.is_open()) {
    RCLCPP_DEBUG(this->get_logger(), ("Pos file succesfully loaded from " + filePath).c_str());
    fileSuccessfullyRead = true;
    auto lines = readLines(ifstream);
    auto parseResult = parser::parse(lines);
    fileSuccessfullyRead = parseResult.successful;
    keyFrames = parseResult.keyFrames;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Couldn't open file");
    fileSuccessfullyRead = false;
  }
}

NaosoccerPosActionNode::~NaosoccerPosActionNode() {}

std::string NaosoccerPosActionNode::getDefaultFullFilePath()
{
  std::string file = "pos/action.pos";
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "naosoccer_pos_action");

  fs::path dir_path(package_share_directory);
  fs::path file_path(file);
  fs::path full_path = dir_path / file_path;
  return full_path.string();
}

std::vector<std::string> NaosoccerPosActionNode::readLines(std::ifstream & ifstream)
{
  std::vector<std::string> ret;

  while (!ifstream.eof()) {
    std::string line;
    std::getline(ifstream, line);
    ret.push_back(line);
  }

  return ret;
}

void NaosoccerPosActionNode::calculateEffectorJoints(
  nao_lola_sensor_msgs::msg::JointPositions & sensor_joints)
{
  int time_ms = (rclcpp::Node::now() - begin).nanoseconds() / 1e6;

  if (posFinished(time_ms)) {
    // We've finished the motion, set to DONE
    posInAction = false;
    RCLCPP_DEBUG(this->get_logger(), "Pos finished");
    return;
  }

  if (firstTickSinceActionStarted) {
    nao_lola_command_msgs::msg::JointPositions command;
    command.indexes = indexes::indexes;
    command.positions = std::vector<float>(
      sensor_joints.positions.begin(), sensor_joints.positions.end());
    keyFrameStart =
      std::make_unique<KeyFrame>(0, command, nao_lola_command_msgs::msg::JointStiffnesses{});
    firstTickSinceActionStarted = false;
  }

  RCLCPP_DEBUG(this->get_logger(), ("time_ms is: " + std::to_string(time_ms)).c_str());

  const auto & previousKeyFrame = findPreviousKeyFrame(time_ms);
  const auto & nextKeyFrame = findNextKeyFrame(time_ms);

  float timeFromPreviousKeyFrame = time_ms - previousKeyFrame.t_ms;
  float timeToNextKeyFrame = nextKeyFrame.t_ms - time_ms;
  float duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

  RCLCPP_DEBUG(
    this->get_logger(), ("timeFromPreviousKeyFrame, timeFromPreviousKeyFrame, duration: " +
    std::to_string(timeFromPreviousKeyFrame) + ", " + std::to_string(timeToNextKeyFrame) + ", " +
    std::to_string(duration)).c_str());

  float alpha = timeToNextKeyFrame / duration;
  float beta = timeFromPreviousKeyFrame / duration;

  RCLCPP_DEBUG(
    this->get_logger(), ("alpha, beta: " + std::to_string(alpha) + ", " + std::to_string(
      beta)).c_str());

  nao_lola_command_msgs::msg::JointPositions effector_joints;
  effector_joints.indexes = indexes::indexes;

  for (unsigned int i = 0; i < nao_lola_command_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    float previous = previousKeyFrame.positions.positions.at(i);
    float next = nextKeyFrame.positions.positions.at(i);
    effector_joints.positions.push_back(previous * alpha + next * beta);

    RCLCPP_DEBUG(
      this->get_logger(), ("previous, next, result: " + std::to_string(
        previous) + ", " + std::to_string(next) + ", " +
      std::to_string(effector_joints.positions.at(i))).c_str());
  }

  pub_joint_positions->publish(effector_joints);
  pub_joint_stiffnesses->publish(nextKeyFrame.stiffnesses);
}

const KeyFrame & NaosoccerPosActionNode::findPreviousKeyFrame(int time_ms)
{
  for (auto it = keyFrames.rbegin(); it != keyFrames.rend(); ++it) {
    const auto & keyFrame = *it;
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms >= keyFrameDeadline) {
      return keyFrame;
    }
  }

  return *keyFrameStart;
}

const KeyFrame & NaosoccerPosActionNode::findNextKeyFrame(int time_ms)
{
  for (const auto & keyFrame : keyFrames) {
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms < keyFrameDeadline) {
      return keyFrame;
    }
  }

  RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
  return keyFrames.back();
}

bool NaosoccerPosActionNode::posFinished(int time_ms)
{
  if (keyFrames.size() == 0) {
    return true;
  }

  const auto lastKeyFrame = keyFrames.back();
  int lastKeyFrameTime = lastKeyFrame.t_ms;
  if (time_ms >= lastKeyFrameTime) {
    return true;
  }

  return false;
}

}  // namespace naosoccer_pos_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(naosoccer_pos_action::NaosoccerPosActionNode)
