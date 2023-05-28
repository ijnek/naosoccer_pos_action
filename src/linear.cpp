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

#include <string>
#include <vector>
#include <utility>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "std_msgs/msg/bool.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/time.hpp"

namespace fs = boost::filesystem;

std::vector<uint8_t> indexes = {
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

class Linear : public rclcpp::Node
{
public:
  Linear()
  : Node("PosGenerator")
  {
    this->declare_parameter<std::string>("file", getDefaultFullFilePath());

    pub = create_publisher<nao_command_msgs::msg::JointPositions>("effectors/joint_positions", 1);

    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
        if (posInAction) {
          calculate_effector_joints(*sensor_joints);
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
    fileSuccessfullyRead = initialiseKeyFrameVector(filePath);
    RCLCPP_DEBUG(this->get_logger(), ("Pos file succesfully loaded from " + filePath).c_str());
  }

private:
  std::string getDefaultFullFilePath()
  {
    std::string file = "pos/move_head.pos";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(
      "naosoccer_pos_action");

    fs::path dir_path(package_share_directory);
    fs::path file_path(file);
    fs::path full_path = dir_path / file_path;
    return full_path.string();
  }

  bool initialiseKeyFrameVector(std::string filePath)
  {
    std::ifstream in(filePath);
    if (!in.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't open file");
      return false;
    }

    int keyFrameTime = 0;

    while (!in.eof()) {
      std::string line;
      std::getline(in, line);

      if (line.front() == '!') {
        RCLCPP_DEBUG(this->get_logger(), ("Found joint line: " + line).c_str());
        line.erase(line.begin());

        std::istringstream ss(line);

        std::vector<std::string> splitted_line(std::istream_iterator<std::string>{ss},
          std::istream_iterator<std::string>());

        if (splitted_line.size() != nao_command_msgs::msg::JointIndexes::NUMJOINTS + 1) {
          // +1 because there is the duration component as well
          RCLCPP_ERROR(this->get_logger(), "pos file joint line missing joint values or duration!");
          return false;
        }

        nao_command_msgs::msg::JointPositions newJoint;
        newJoint.indexes = indexes;

        for (unsigned int i = 0; i < nao_command_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
          std::string position_deg_string = splitted_line.at(i);

          try {
            float position_deg = std::stof(position_deg_string);
            float position_rad = position_deg * M_PI / 180;
            newJoint.positions.push_back(position_rad);
          } catch (std::invalid_argument &) {
            RCLCPP_ERROR(
              this->get_logger(),
              ("joint value '" + position_deg_string +
              "' is not a valid joint value (cannot be converted to float)").c_str());
            return false;
          }
        }

        std::string duration_string = splitted_line.back();
        try {
          int duration = std::stoi(duration_string);
          keyFrameTime += duration;
        } catch (std::invalid_argument &) {
          RCLCPP_ERROR(
            this->get_logger(),
            ("duration '" + duration_string +
            "' is not a valid duration value (cannot be converted to int)").c_str());
          return false;
        }

        keyFrames.push_back(std::make_pair(newJoint, keyFrameTime));
      }
    }

    return true;
  }

  void calculate_effector_joints(nao_sensor_msgs::msg::JointPositions & sensor_joints)
  {
    int time_ms = (rclcpp::Node::now() - begin).nanoseconds() / 1e6;

    if (posFinished(time_ms)) {
      // We've finished the motion, set to DONE
      posInAction = false;
      RCLCPP_DEBUG(this->get_logger(), "Pos finished");
      return;
    }

    if (firstTickSinceActionStarted) {
      nao_command_msgs::msg::JointPositions command;
      command.indexes = indexes;
      command.positions = std::vector<float>(
        sensor_joints.positions.begin(), sensor_joints.positions.end());
      jointsWhenActionStarted = std::make_pair(command, 0);
      firstTickSinceActionStarted = false;
    }

    RCLCPP_DEBUG(this->get_logger(), ("time_ms is: " + std::to_string(time_ms)).c_str());

    std::pair<nao_command_msgs::msg::JointPositions, int> & previousKeyFrame = findPreviousKeyFrame(
      time_ms);
    std::pair<nao_command_msgs::msg::JointPositions,
      int> & nextKeyFrame = findNextKeyFrame(time_ms);

    float timeFromPreviousKeyFrame = time_ms - previousKeyFrame.second;
    float timeToNextKeyFrame = nextKeyFrame.second - time_ms;
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

    nao_command_msgs::msg::JointPositions effector_joints;
    effector_joints.indexes = indexes;

    for (unsigned int i = 0; i < nao_command_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
      float previous = previousKeyFrame.first.positions.at(i);
      float next = nextKeyFrame.first.positions.at(i);
      effector_joints.positions.push_back(previous * alpha + next * beta);

      RCLCPP_DEBUG(
        this->get_logger(), ("previous, next, result: " + std::to_string(
          previous) + ", " + std::to_string(next) + ", " +
        std::to_string(effector_joints.positions.at(i))).c_str());
    }

    pub->publish(effector_joints);
  }

  std::pair<nao_command_msgs::msg::JointPositions, int> & findPreviousKeyFrame(int time_ms)
  {
    for (auto it = keyFrames.rbegin(); it != keyFrames.rend(); ++it) {
      int keyFrameDeadline = it->second;
      if (time_ms >= keyFrameDeadline) {
        return *it;
      }
    }

    return jointsWhenActionStarted;
  }

  std::pair<nao_command_msgs::msg::JointPositions, int> & findNextKeyFrame(int time_ms)
  {
    for (std::pair<nao_command_msgs::msg::JointPositions, int> & keyFrame : keyFrames) {
      int keyFrameDeadline = keyFrame.second;
      if (time_ms < keyFrameDeadline) {
        return keyFrame;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
    return keyFrames.back();
  }

  bool posFinished(int time_ms)
  {
    if (keyFrames.size() == 0) {
      return true;
    }

    std::pair<nao_command_msgs::msg::JointPositions, int> lastKeyFrame = keyFrames.back();
    int lastKeyFrameTime = lastKeyFrame.second;
    if (time_ms >= lastKeyFrameTime) {
      return true;
    }

    return false;
  }

  std::vector<std::pair<nao_command_msgs::msg::JointPositions, int>> keyFrames;

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start;
  rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub;

  bool fileSuccessfullyRead = false;
  bool posInAction = false;
  bool firstTickSinceActionStarted = true;
  std::pair<nao_command_msgs::msg::JointPositions, int> jointsWhenActionStarted;
  rclcpp::Time begin;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Linear>());
  rclcpp::shutdown();
  return 0;
}
