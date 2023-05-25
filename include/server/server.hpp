#pragma once

#include <vector>
#include <list>

#include <rclcpp/rclcpp.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"
#include "multibot_ros2_interface/msg/robot_config.hpp"
#include "multibot_ros2_interface/srv/robot_configs.hpp"

namespace Server
{
    class MultibotServer : public rclcpp::Node
    {
    private:
        struct Robot
        {
            std::string name_;

            double size_;
            double wheel_radius_;
            double wheel_seperation_;

            geometry_msgs::msg::Pose2D start;
            geometry_msgs::msg::Pose2D goal;
            geometry_msgs::msg::Pose2D pose;

            bool is_initial_pose_;
        };

    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfig       = multibot_ros2_interface::msg::RobotConfig;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;

    private:
        void update_callback();
        void update_robotStates(RobotStateArray &_robot_states);
        void send_robotConfigs(const std::shared_ptr<RobotConfigs::Request>  _request,
                               std::shared_ptr<RobotConfigs::Response>       _response);
        void read_task();

    private:
        rclcpp::Publisher<RobotStateArray>::SharedPtr robot_states_pub_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<RobotConfigs>::SharedPtr registration_server_;

        std::vector<double> robot_state_;
        std::list<std::string> robotTypes_;

        std::unordered_map<std::string, Robot> robotList_;

        std::string task_fPath_;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server