#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nao_interfaces/msg/joint_commands.hpp>
#include <nao_interfaces/msg/joint_positions.hpp>
#include <std_msgs/msg/bool.hpp>
#include <boost/filesystem.hpp>
#include <rclcpp/time.hpp>

namespace fs = boost::filesystem;

class Linear : public rclcpp::Node
{
public:
    Linear() : Node("PosGenerator")
    {
        this->declare_parameter<std::string>("file", getDefaultFullFilePath());

        pub = create_publisher<nao_interfaces::msg::JointCommands>("/joint_commands", 1);

        sub_joint_states =
            create_subscription<nao_interfaces::msg::JointPositions>(
                "/joint_positions", 1,
                [this](nao_interfaces::msg::JointPositions::UniquePtr joint_states) {
                    if (posInAction)
                    {
                        calculate_joint_command(std::move(joint_states));
                    }
                });

        sub_start =
            create_subscription<std_msgs::msg::Bool>(
                "/start_pos_action", 1,
                [this](std_msgs::msg::Bool::UniquePtr start_pos_action) {
                    if (start_pos_action->data && fileSuccessfullyRead && !posInAction)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "Starting Pos Action");
                        begin = rclcpp::Node::now();
                        posInAction = true;
                        firstTickSinceActionStarted = true;
                    }
                });

        std::string filePath;
        this->get_parameter("file", filePath);
        fileSuccessfullyRead = initialiseKeyFrameVector(filePath);
        RCLCPP_DEBUG(this->get_logger(), "Pos file succesfully loaded from " + filePath);
    }

private:
    std::string getDefaultFullFilePath()
    {
        std::string file = "pos/action.pos";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("naosoccer_pos_action");

        fs::path dir_path(package_share_directory);
        fs::path file_path(file);
        fs::path full_path = dir_path / file_path;
        return full_path.string();
    }

    bool initialiseKeyFrameVector(std::string filePath)
    {
        std::ifstream in(filePath);
        if (!in.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't open file");
            return false;
        }

        int keyFrameTime = 0;

        while (!in.eof())
        {
            std::string line;
            std::getline(in, line);

            if (line.front() == '!')
            {
                RCLCPP_DEBUG(this->get_logger(), "Found joint line: " + line);
                line.erase(line.begin());

                std::istringstream ss(line);

                std::vector<std::string> splitted_line(std::istream_iterator<std::string>{ss},
                                                       std::istream_iterator<std::string>());

                if (splitted_line.size() != joint_names.size() + 1) // +1 because there is the duration component as well
                {
                    RCLCPP_ERROR(this->get_logger(), "pos file joint line missing joint values or duration!");
                    return false;
                }

                nao_interfaces::msg::JointCommands newJoint;

                for (unsigned int i = 0; i < joint_names.size(); ++i)
                {

                    std::string position_deg_string = splitted_line[i];

                    try
                    {
                        float position_deg = std::stof(position_deg_string);
                        float position_rad = position_deg * M_PI / 180;
                        newJoint.name.push_back(joint_names[i]);
                        newJoint.position.push_back(position_rad);
                    }
                    catch (std::invalid_argument &)
                    {
                        RCLCPP_ERROR(this->get_logger(), "joint value '" + position_deg_string + "' is not a valid joint value (cannot be converted to float)");
                        return false;
                    }
                }

                std::string duration_string = splitted_line.back();
                try
                {
                    int duration = std::stoi(duration_string);
                    keyFrameTime += duration;
                }
                catch (std::invalid_argument &)
                {
                    RCLCPP_ERROR(this->get_logger(), "duration '" + duration_string + "' is not a valid duration value (cannot be converted to int)");
                    return false;
                }

                keyFrames.push_back(std::make_pair(newJoint, keyFrameTime));
            }
        }

        return true;
    }

    void calculate_joint_command(nao_interfaces::msg::JointPositions::UniquePtr joint_states)
    {
        int time_ms = (rclcpp::Node::now() - begin).nanoseconds() / 1e6;

        if (posFinished(time_ms))
        {
            // We've finished the motion, set to DONE
            posInAction = false;
            RCLCPP_DEBUG(this->get_logger(), "Pos finished");
            return;
        }

        if (firstTickSinceActionStarted)
        {
            nao_interfaces::msg::JointCommands JointCommand;
            std::vector<std::string> &jointStateNames = joint_states->name;

            for (std::string &jointName : joint_names)
            {
                auto itJointStateName = std::find(jointStateNames.begin(), jointStateNames.end(), jointName);
                bool jointStateNameExists = (itJointStateName != jointStateNames.end());

                if (jointStateNameExists)
                {
                    int jointStateNameIndex = itJointStateName - jointStateNames.begin();
                    JointCommand.name.push_back(jointName);
                    JointCommand.position.push_back(joint_states->position[jointStateNameIndex]);
                }
            }

            JointCommandWhenActionStarted = std::make_pair(JointCommand, 0);

            firstTickSinceActionStarted = false;
        }

        std::pair<nao_interfaces::msg::JointCommands, int> &previousKeyFrame = findPreviousKeyFrame(time_ms);
        std::pair<nao_interfaces::msg::JointCommands, int> &nextKeyFrame = findNextKeyFrame(time_ms);

        int timeFromPreviousKeyFrame = time_ms - previousKeyFrame.second;
        int timeToNextKeyFrame = nextKeyFrame.second - time_ms;
        int duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

        std::vector<std::string> &previousKeyFrameNames = previousKeyFrame.first.name;
        std::vector<std::string> &nextKeyFrameNames = nextKeyFrame.first.name;

        std::vector<float> &previousKeyFramePositions = previousKeyFrame.first.position;
        std::vector<float> &nextKeyFramePositions = nextKeyFrame.first.position;

        nao_interfaces::msg::JointCommands JointCommand;

        for (std::string &jointName : joint_names)
        {
            auto itPreviousKeyFrameName = std::find(previousKeyFrameNames.begin(), previousKeyFrameNames.end(), jointName);
            auto itNextKeyFrameName = std::find(nextKeyFrameNames.begin(), nextKeyFrameNames.end(), jointName);

            bool previousKeyFrameNameExists = (itPreviousKeyFrameName != previousKeyFrameNames.end());
            bool nextKeyFrameNameExists = (itNextKeyFrameName != nextKeyFrameNames.end());

            if (previousKeyFrameNameExists && nextKeyFrameNameExists)
            {
                int previousKeyFrameNameIndex = itPreviousKeyFrameName - previousKeyFrameNames.begin();
                int nextKeyFrameNameIndex = itNextKeyFrameName - nextKeyFrameNames.begin();

                float previous = previousKeyFramePositions[previousKeyFrameNameIndex];
                float next = nextKeyFramePositions[nextKeyFrameNameIndex];

                float weightedAverage = (previous * timeToNextKeyFrame + next * timeFromPreviousKeyFrame) / duration;

                JointCommand.name.push_back(jointName);
                JointCommand.position.push_back(weightedAverage);
            }
        }

        pub->publish(std::move(JointCommand));         
    }

    std::pair<nao_interfaces::msg::JointCommands, int> &findPreviousKeyFrame(int time_ms)
    {
        for (auto it = keyFrames.rbegin(); it != keyFrames.rend(); ++it)
        {
            int keyFrameDeadline = it->second;
            if (time_ms >= keyFrameDeadline)
            {
                return *it;
            }
        }

        return JointCommandWhenActionStarted;
    }

    std::pair<nao_interfaces::msg::JointCommands, int> &findNextKeyFrame(int time_ms)
    {
        for (std::pair<nao_interfaces::msg::JointCommands, int> &keyFrame : keyFrames)
        {
            int keyFrameDeadline = keyFrame.second;
            if (time_ms < keyFrameDeadline)
            {
                return keyFrame;
            }
        }

        RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
        return keyFrames.back();
    }

    bool posFinished(int time_ms)
    {
        if (keyFrames.size() == 0)
            return true;

        std::pair<nao_interfaces::msg::JointCommands, int> lastKeyFrame = keyFrames.back();
        int lastKeyFrameTime = lastKeyFrame.second;
        if (time_ms > lastKeyFrameTime)
            return true;

        return false;
    }

    std::vector<std::string> joint_names = {
        "HeadYaw",
        "HeadPitch",
        "LShoulderPitch",
        "LShoulderRoll",
        "LElbowYaw",
        "LElbowRoll",
        "LWristYaw",
        "LHipYawPitch",
        "LHipRoll",
        "LHipPitch",
        "LKneePitch",
        "LAnklePitch",
        "LAnkleRoll",
        "RHipRoll",
        "RHipPitch",
        "RKneePitch",
        "RAnklePitch",
        "RAnkleRoll",
        "RShoulderPitch",
        "RShoulderRoll",
        "RElbowYaw",
        "RElbowRoll",
        "RWristYaw",
        "LHand",
        "RHand"};

    std::vector<std::pair<nao_interfaces::msg::JointCommands, int>> keyFrames;

    rclcpp::Subscription<nao_interfaces::msg::JointPositions>::SharedPtr sub_joint_states;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start;
    rclcpp::Publisher<nao_interfaces::msg::JointCommands>::SharedPtr pub;

    bool fileSuccessfullyRead = false;
    bool posInAction = false;
    bool firstTickSinceActionStarted = true;
    std::pair<nao_interfaces::msg::JointCommands, int> JointCommandWhenActionStarted;
    rclcpp::Time begin;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Linear>());
    rclcpp::shutdown();
    return 0;
}