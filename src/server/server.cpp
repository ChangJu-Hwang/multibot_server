#include "server/server.hpp"

using namespace Server;
using namespace std::chrono_literals;

void MultibotServer::update_callback()
{
    auto robot_states = RobotStateArray();
    update_robotStates(robot_states);
    robot_states_pub_->publish(robot_states);
}

void MultibotServer::update_robotStates(RobotStateArray& _robot_states)
{
    _robot_states.robot_states.clear();

    auto robotState = RobotState();
    robotState.name = "Agent1";
    robotState.pose.x = prior_x;
        prior_x = prior_x + 0.001;
    robotState.pose.y = 7.1;
    robotState.pose.theta = 0.0;

    _robot_states.robot_states.push_back(robotState);
}

void MultibotServer::send_robotConfigs(const std::shared_ptr<RobotConfigs::Request>  _request,
                                       std::shared_ptr<RobotConfigs::Response>       _response)
{
    auto robotConfig = RobotConfig();
    robotConfig.name                = "Agent1";
    robotConfig.size                = 0.50;
    robotConfig.wheel_seperation    = 0.50;
    robotConfig.wheel_radius        = 0.32;
    _response->configs.push_back(robotConfig);
}

MultibotServer::MultibotServer()
: Node("server")
{   
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    robot_states_pub_ = this->create_publisher<RobotStateArray>("robot_states", qos);

    registration_server_ = this->create_service<RobotConfigs>("registration",
                                                             std::bind(&MultibotServer::send_robotConfigs, this, std::placeholders::_1, std::placeholders::_2));

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotServer::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}