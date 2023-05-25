#pragma once

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"
#include "multibot_ros2_interface/msg/robot_config.hpp"
#include "multibot_ros2_interface/srv/robot_configs.hpp"

namespace Simulation
{
    class MultibotSim : public rclcpp::Node
    {
    private:
        //Todo: Struct Robot: Just a temporary struct
        struct Robot
        {
            std::string name_;

            double size_;
            double wheel_radius_;
            double wheel_seperation_;

            geometry_msgs::msg::Pose2D pose_;
            geometry_msgs::msg::Pose2D prior_pose_;

            rclcpp::Time time_now_;
            rclcpp::Time prior_time_;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

            bool is_initial_pose_;
        };

    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;

    public:
        void registration_request();

    private:
        void update_callback();
        void update_rviz(visualization_msgs::msg::MarkerArray &_markerArray);
        void update_gazebo();
        geometry_msgs::msg::Twist update_cmd_vel(const Robot &_robot);

        void robot_states_callback(const RobotStateArray::SharedPtr _robot_states);

        void init_variables();

        void update_robotList(const RobotState &_robot_state);
    
    private:
        // ROS time
        rclcpp::Time prev_rviz_update_time_;
        rclcpp::Time prev_gazebo_update_time_;
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

        // ROS timer
        rclcpp::TimerBase::SharedPtr update_timer_;

        // ROS topic publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<RobotStateArray>::SharedPtr robot_states_sub_;

        // ROS Service
        rclcpp::Client<RobotConfigs>::SharedPtr registration_;

        // Variables
        std::unordered_map<std::string, Robot> robotList_;
    public:
        MultibotSim();
        ~MultibotSim();
    };
} // namespace Simulation