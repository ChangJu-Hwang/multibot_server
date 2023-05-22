#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"

namespace Server
{
    class MultibotServer : public rclcpp::Node
    {
    private:
        void update_callback();
        void update_robotStates(multibot_ros2_interface::msg::RobotStateArray &_robot_states);

    private:
        rclcpp::Publisher<multibot_ros2_interface::msg::RobotStateArray>::SharedPtr robot_states_pub_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        std::vector<double> robot_state_;
        double prior_x = 3.1;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server