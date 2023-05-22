#include "server/server.hpp"

using namespace Server;
using namespace std::chrono_literals;

void MultibotServer::update_callback()
{
    auto robot_states = multibot_ros2_interface::msg::RobotStateArray();
    update_robotStates(robot_states);
    robot_states_pub_->publish(robot_states);
}

void MultibotServer::update_robotStates(multibot_ros2_interface::msg::RobotStateArray& _robot_states)
{
    _robot_states.robot_states.clear();

    auto robotState = multibot_ros2_interface::msg::RobotState();
    robotState.robot_name  = "Agent1";
    robotState.robot_size  = 0.7;
    robotState.robot_x     = prior_x;
        prior_x = prior_x + 0.001;
    robotState.robot_y     = 7.1;
    robotState.robot_theta = 0.0;

    _robot_states.robot_states.push_back(robotState);
}

MultibotServer::MultibotServer()
: Node("server")
{   
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    robot_states_pub_ = this->create_publisher<multibot_ros2_interface::msg::RobotStateArray>("robot_states", qos);

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotServer::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}