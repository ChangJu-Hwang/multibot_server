#include "server/server.hpp"

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

using namespace Server;

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
            std::string robotName = robot.second.name_;
            robotState.name = robotName;

            auto time = this->now();
            if(this->now() - nodeStartTime_ > rclcpp::Duration(10, 0))
                updateRobotPose(robotName);

            robotState.pose = robot.second.pose_;
            
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
    nodeStartTime_ = this->now();

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

            this->declare_parameter(type + ".linear.velocity");
            this->declare_parameter(type + ".linear.acceleration");
            this->declare_parameter(type + ".angular.velocity");
            this->declare_parameter(type + ".angular.acceleration");

            robotTypes_.push_back(type);
        }

        this->get_parameter_or(type + ".size", robot.size_, 0.0);
        this->get_parameter_or(type + ".wheels.separation", robot.wheel_seperation_, 0.0);
        this->get_parameter_or(type + ".wheels.radius", robot.wheel_radius_, 0.0);

        this->get_parameter_or(type + ".linear.velocity", robot.max_linear_velocity_, 0.0);
        this->get_parameter_or(type + ".linear.acceleration", robot.max_linear_acceleration_, 0.0);
        this->get_parameter_or(type + ".angular.velocity", robot.max_angular_velocity_, 0.0);
        this->get_parameter_or(type + ".angular.acceleration", robot.max_angular_accerlation_, 0.0);

        geometry_msgs::msg::Pose2D startPose;
        startPose.x = task["start"]["x"].as<double>();
        startPose.y = task["start"]["y"].as<double>();
        startPose.theta = task["start"]["theta"].as<double>();
        robot.start_ = startPose;

        geometry_msgs::msg::Pose2D goalPose;
        goalPose.x = task["goal"]["x"].as<double>();
        goalPose.y = task["goal"]["y"].as<double>();
        goalPose.theta = task["goal"]["theta"].as<double>();
        robot.goal_ = goalPose;

        robot.pose_ = robot.start_;

        robotList_.insert(std::make_pair(robot.name_, robot));
    }
}

void MultibotServer::updateRobotPose(const std::string& _robotName)
{
    Robot &robot = robotList_[_robotName];
    robot.localTimer_ = robot.localTimer_ + 10ms;

    if(std::fabs(robot.goal_.x - robot.pose_.x) < 1e-8 and
       std::fabs(robot.goal_.y - robot.pose_.y) < 1e-8)
       return;
    
    double theta = std::atan2(robot.goal_.y - robot.start_.y, robot.goal_.x - robot.start_.x);
    double max_v_x = robot.max_linear_velocity_ * std::cos(theta);
    double max_v_y = robot.max_linear_velocity_ * std::sin(theta);
    double max_a_x = robot.max_linear_acceleration_ * std::cos(theta);
    double max_a_y = robot.max_linear_acceleration_ * std::sin(theta);

    robot.pose_.x = displacementComputer(robot.start_.x, robot.goal_.x, robot.localTimer_.count() * 1e-3, max_v_x, max_a_x);
    robot.pose_.y = displacementComputer(robot.start_.y, robot.goal_.y, robot.localTimer_.count() * 1e-3, max_v_y, max_a_y);
}

double MultibotServer::displacementComputer(const double start_, const double goal_, const double t_,
                                            const double max_velocity_, const double max_acceleration_)
{
    double max_s = std::fabs(goal_ - start_);

    if(max_s < 1e-8)
        return start_;
    
    int sign = start_ > goal_ ? -1:1;

    double v = std::fabs(max_velocity_);    double a = std::fabs(max_acceleration_);
    
    double s = 0;
    // Trapezoidal velocity profile
    if(max_s > v * v / a)
    {
        if(t_ < v / a)
            s = std::fabs(0.5 * a * t_ * t_);
        else if(t_ < max_s / v)
            s = std::fabs(v * t_ - 0.5 * v * v / a);
        else
            s = std::fabs(max_s - 0.5 * a * std::pow((max_s / v + v / a - t_), 2));
    }
    // Triangular velocity profile
    else
    {
        if(t_ < std::fabs(0.5 * a * t_ * t_))
            s = std::fabs(0.5 * a * t_ * t_);
        else
            s = std::fabs(max_s - 0.5 * a * std::pow((2 * std::sqrt(max_s / a) - t_), 2));
    }

    return (start_ + sign * s);
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