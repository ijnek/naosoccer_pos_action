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
#include <iostream>



#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"

#include "indexes.hpp"
#include "parser.hpp"

namespace fs = boost::filesystem;

namespace naosoccer_pos_action {

NaosoccerPosActionNode::NaosoccerPosActionNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node{"NaosoccerPosActionNode", options} {
    
  pub_joint_positions = create_publisher<nao_lola_command_msgs::msg::JointPositions>(
                          "effectors/joint_positions", 1);
  pub_joint_stiffnesses = create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
                            "effectors/joint_stiffnesses", 1);

  sub_joint_states =
    create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
  [this](nao_lola_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
    if (pos_in_action_) {
      calculateEffectorJoints(*sensor_joints);
    }
  });

  action_server_ = rclcpp_action::create_server<naosoccer_pos_action_interfaces::action::Action>(
                     this,
                     "naosoccer_pos_action",
                     std::bind(&NaosoccerPosActionNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                     std::bind(&NaosoccerPosActionNode::handleCancel, this, std::placeholders::_1),
                     std::bind(&NaosoccerPosActionNode::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "NaosoccerPosActionServer initialized");
}

NaosoccerPosActionNode::~NaosoccerPosActionNode() {}

void NaosoccerPosActionNode::readPosFile(std::string & filePath) {
  std::ifstream ifstream(filePath);
  if (ifstream.is_open()) {
    RCLCPP_DEBUG(this->get_logger(), ("Pos file succesfully loaded from " + filePath).c_str());
    file_successfully_read_ = true;
    auto lines = readLines(ifstream);
    auto parseResult = parser::parse(lines);
    file_successfully_read_ = parseResult.successful;
    key_frames_ = parseResult.keyFrames;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Couldn't open file");
    file_successfully_read_ = false;
  }
}

std::string NaosoccerPosActionNode::getFullFilePath(std::string & filename) {
  std::string file = "pos/" + filename;
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
                                          "naosoccer_pos_action");

  fs::path dir_path(package_share_directory);
  fs::path file_path(file);
  fs::path full_path = dir_path / file_path;
  return full_path.string();
}

std::vector<std::string> NaosoccerPosActionNode::readLines(std::ifstream & ifstream) {
  std::vector<std::string> ret;

  while (!ifstream.eof()) {
    std::string line;
    std::getline(ifstream, line);
    ret.push_back(line);
  }

  return ret;
}

float NaosoccerPosActionNode::findElem( const std::vector<uint8_t> & indexes , const std::vector<float> & data,  uint8_t joint) {

    for (uint8_t a = 0; a < indexes.size() ; a++) {
      if (indexes.at(a) == joint) {
        return data.at(a);
      }
    }
    return NAN;
}

void NaosoccerPosActionNode::calculateEffectorJoints(
  nao_lola_sensor_msgs::msg::JointPositions & sensor_joints) {
  std::lock_guard<std::mutex> lock(mutex);

  int time_ms = (rclcpp::Node::now() - initial_time_).nanoseconds() / 1e6;

  if (posFinished(time_ms)) {
    // We've finished the motion, set to DONE
    pos_in_action_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Pos finished before");
    auto result = std::make_shared<naosoccer_pos_action_interfaces::action::Action::Result>();
    goal_handle_->succeed(result);
    RCLCPP_DEBUG(this->get_logger(), "Pos finished after");
    return;
  }

  if (firstTickSinceActionStarted_) {
    nao_lola_command_msgs::msg::JointPositions command;
    command.indexes = indexes::indexes;
    command.positions = std::vector<float>(
                          sensor_joints.positions.begin(), sensor_joints.positions.end());
    key_frame_start_ =
      std::make_unique<KeyFrame>(0, command, nao_lola_command_msgs::msg::JointStiffnesses{});
  }

  const auto & previousKeyFrame = findPreviousKeyFrame(time_ms);
  const auto & nextKeyFrame = findNextKeyFrame(time_ms);

  if (firstTickSinceActionStarted_) {
    for (auto i : nextKeyFrame.positions.indexes) {
      selected_joints_.push_back(i);
    }
    firstTickSinceActionStarted_ = false;

    RCLCPP_DEBUG(this->get_logger(), "first tick false");
    /*std::string s1="";
    for (unsigned c = 0; c < selected_joints_.size(); c++) {
        s1.append( std::to_string(selected_joints_.at(c))+", " );
    }
    RCLCPP_INFO(this->get_logger(), ("selected_joints_: "+s1).c_str() );*/

  }

  float timeFromPreviousKeyFrame = time_ms - previousKeyFrame.t_ms;
  float timeToNextKeyFrame = nextKeyFrame.t_ms - time_ms;
  float duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

  RCLCPP_DEBUG(
    this->get_logger(), ("timeFromPreviousKeyFrame, timeToNextKeyFrame, duration: " +
                         std::to_string(timeFromPreviousKeyFrame) + ", " + std::to_string(timeToNextKeyFrame) + ", " +
                         std::to_string(duration)).c_str());

  float alpha = timeToNextKeyFrame / duration; //normalized coefficent k of convex combination
  float beta = timeFromPreviousKeyFrame / duration; //normalized 1-k

  RCLCPP_DEBUG(
    this->get_logger(), ("alpha, beta: " + std::to_string(alpha) + ", " + std::to_string(
                           beta)).c_str());

  nao_lola_command_msgs::msg::JointPositions effector_joints;
  nao_lola_command_msgs::msg::JointStiffnesses effector_joints_stiff;

  float nextPos=NAN, previousPos=NAN, nextStiff=NAN;

  for (uint8_t i : selected_joints_) {

    nextPos = findElem( nextKeyFrame.positions.indexes, nextKeyFrame.positions.positions, i );
    nextStiff = findElem( nextKeyFrame.stiffnesses.indexes, nextKeyFrame.stiffnesses.stiffnesses, i );
    previousPos = findElem( previousKeyFrame.positions.indexes, previousKeyFrame.positions.positions, i );


    effector_joints.indexes.push_back(i);
    effector_joints.positions.push_back(previousPos * alpha + nextPos * beta);
    effector_joints_stiff.indexes.push_back(i);
    effector_joints_stiff.stiffnesses.push_back( nextStiff );
    
    nextPos=NAN, previousPos=NAN, nextStiff=NAN;

  }

  pub_joint_positions->publish(effector_joints);
  pub_joint_stiffnesses->publish(effector_joints_stiff);
  RCLCPP_DEBUG(this->get_logger(), "published to nao_lola topic");
}
const KeyFrame & NaosoccerPosActionNode::findPreviousKeyFrame(int time_ms) {
  for (auto it = key_frames_.rbegin(); it != key_frames_.rend(); ++it) {
    const auto & keyFrame = *it;
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms >= keyFrameDeadline) {
      return keyFrame;
    }
  }

  return *key_frame_start_;
}

const KeyFrame & NaosoccerPosActionNode::findNextKeyFrame(int time_ms) {
  for (const auto & keyFrame : key_frames_) {
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms < keyFrameDeadline) {
      return keyFrame;
    }
  }

  RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
  return key_frames_.back();
}

bool NaosoccerPosActionNode::posFinished(int time_ms) {
  if (key_frames_.size() == 0) {
    return true;
  }

  const auto lastKeyFrame = key_frames_.back();
  int lastKeyFrameTime = lastKeyFrame.t_ms;
  if (time_ms >= lastKeyFrameTime) {
    return true;
  }

  return false;
}

rclcpp_action::GoalResponse NaosoccerPosActionNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const naosoccer_pos_action_interfaces::action::Action::Goal> goal) {
  std::lock_guard<std::mutex> lock(mutex);
  RCLCPP_INFO(get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;

  std::string filename = goal->action_name + ".pos";
  std::string path = getFullFilePath(filename);
  readPosFile(path);

  if (!file_successfully_read_ || pos_in_action_) {
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}


rclcpp_action::CancelResponse NaosoccerPosActionNode::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<naosoccer_pos_action_interfaces::action::Action>> goal_handle) {
  std::lock_guard<std::mutex> lock(mutex);
  // RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  pos_in_action_ = false;
  goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}
void NaosoccerPosActionNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<naosoccer_pos_action_interfaces::action::Action>> goal_handle) {
  std::lock_guard<std::mutex> lock(mutex);
  RCLCPP_DEBUG(this->get_logger(), "Starting Pos Action");
  initial_time_ = rclcpp::Node::now();
  pos_in_action_ = true;
  firstTickSinceActionStarted_ = true;
  selected_joints_.clear();
  goal_handle_ = goal_handle;
}

}  // namespace naosoccer_pos_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(naosoccer_pos_action::NaosoccerPosActionNode)
