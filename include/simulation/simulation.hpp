#pragma once

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"

namespace Simulation
{
    enum Pose{x, y, theta};

    class MultibotSim : public rclcpp::Node
    {
    private:
        struct Robot
        {
            std::string name_;
            double size_;
            std::vector<double> pose_;
        };
    private:
        void update_callback();
        void update_rviz(visualization_msgs::msg::MarkerArray &_markerArray);

        void robot_states_callback(const multibot_ros2_interface::msg::RobotStateArray::SharedPtr _robot_states);

        void init_variables();

        void update_robotList(const multibot_ros2_interface::msg::RobotState &_robot_state);
    
    private:
        // ROS time
        rclcpp::Time prev_rviz_update_time_;

        // ROS timer
        rclcpp::TimerBase::SharedPtr update_timer_;

        // ROS topic publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<multibot_ros2_interface::msg::RobotStateArray>::SharedPtr robot_states_sub_;

        std::unordered_map<std::string, Robot> robotList_;
    public:
        MultibotSim();
        ~MultibotSim();
    };
} // namespace Simulation