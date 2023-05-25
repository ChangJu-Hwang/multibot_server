#include "server/server.hpp"

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

using namespace Server;
using namespace std::chrono_literals;

void MultibotServer::update_callback()
{
    auto robot_states = RobotStateArray();
    update_robotStates(robot_states);
    robot_states_pub_->publish(robot_states);
}

void MultibotServer::update_robotStates(RobotStateArray &_robot_states)
{
    _robot_states.robot_states.clear();

    for (auto &robot : robotList_)
    {
        auto robotState = RobotState();
            robotState.name = robot.second.name_;

            if(robot.second.pose.x < robot.second.goal.x)
                robot.second.pose.x += 0.01;

            robotState.pose = robot.second.pose;
            
        _robot_states.robot_states.push_back(robotState);
    }
}

void MultibotServer::send_robotConfigs(const std::shared_ptr<RobotConfigs::Request>,
                                       std::shared_ptr<RobotConfigs::Response> _response)
{
    for (const auto &robot : robotList_)
    {
        auto robotConfig = RobotConfig();
        robotConfig.name = robot.second.name_;
        robotConfig.size = robot.second.size_;
        robotConfig.wheel_seperation = robot.second.wheel_seperation_;
        robotConfig.wheel_radius = robot.second.wheel_radius_;

        _response->configs.push_back(robotConfig);
    }
}

void MultibotServer::read_task()
{
    robotTypes_.clear();

    this->declare_parameter("task_fPath");
    this->get_parameter("task_fPath", task_fPath_);

    std::vector<YAML::Node> tasks = YAML::LoadAllFromFile(task_fPath_);

    for (const auto &task : tasks)
    {
        Robot robot;

        robot.name_ = task["name"].as<std::string>();
        std::string type = task["type"].as<std::string>();

        if (std::find(robotTypes_.begin(), robotTypes_.end(), type) == robotTypes_.end())
        {
            this->declare_parameter(type + ".size");
            this->declare_parameter(type + ".wheels.separation");
            this->declare_parameter(type + ".wheels.radius");

            robotTypes_.push_back(type);
        }

        this->get_parameter_or(type + ".size", robot.size_, 0.0);
        this->get_parameter_or(type + ".wheels.separation", robot.wheel_seperation_, 0.0);
        this->get_parameter_or(type + ".wheels.radius", robot.wheel_radius_, 0.0);

        geometry_msgs::msg::Pose2D startPose;
        startPose.x = task["start"]["x"].as<double>();
        startPose.y = task["start"]["y"].as<double>();
        startPose.theta = task["start"]["theta"].as<double>();
        robot.start = startPose;

        geometry_msgs::msg::Pose2D goalPose;
        goalPose.x = task["goal"]["x"].as<double>();
        goalPose.y = task["goal"]["y"].as<double>();
        goalPose.theta = task["goal"]["theta"].as<double>();
        robot.goal = goalPose;

        robot.pose = robot.start;

        robotList_.insert(std::make_pair(robot.name_, robot));
    }
}

MultibotServer::MultibotServer()
    : Node("server")
{
    read_task();

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