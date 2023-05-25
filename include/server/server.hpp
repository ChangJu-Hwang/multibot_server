#pragma once

#include <vector>

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
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfig       = multibot_ros2_interface::msg::RobotConfig;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;

    private:
        void update_callback();
        void update_robotStates(RobotStateArray &_robot_states);
        void send_robotConfigs(const std::shared_ptr<RobotConfigs::Request>  _request,
                               std::shared_ptr<RobotConfigs::Response>       _response);

    private:
        rclcpp::Publisher<RobotStateArray>::SharedPtr robot_states_pub_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<RobotConfigs>::SharedPtr registration_server_;

        std::vector<double> robot_state_;
        double prior_x = 3.1;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server