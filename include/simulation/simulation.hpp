#pragma once

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
            int32_t id_;

            double size_;
            double wheel_radius_;
            double wheel_seperation_;

            geometry_msgs::msg::Pose2D pose_;
            geometry_msgs::msg::Pose2D prior_pose_;
            geometry_msgs::msg::Pose2D gazebo_pose_;

            rclcpp::Time time_now_;
            rclcpp::Time prior_time_;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

            bool is_initial_pose_;
        };

    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;

    public:
        void register_robots();
        // void set_odomSubscribers();

    private:
        void request_registration();
        // void set_odomSubscriber(Robot &_robot);

        void update_callback();
        visualization_msgs::msg::Marker update_rviz(const Robot &_robot);
        void update_gazebo(const Robot &_robot);

        void robot_states_callback(const RobotStateArray::SharedPtr _robot_states);

        void update_robotList(const RobotState &_robot_state);
    
    private:
        // ROS time
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

        // ROS timer
        rclcpp::TimerBase::SharedPtr update_timer_;

        // ROS topic publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

        // ROS topic subscribers
        rclcpp::Subscription<RobotStateArray>::SharedPtr robot_states_sub_;
        // std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_sub_list_;

        // ROS Service
        rclcpp::Client<RobotConfigs>::SharedPtr registration_;

        // Variables
        std::unordered_map<std::string, Robot> robotList_;
        bool registrationFlag_ = false;
        
    public:
        MultibotSim();
        ~MultibotSim();
    };
} // namespace Simulation