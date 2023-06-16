#include "server/server.hpp"

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

using namespace Server;
using namespace Instance;

void MultibotServer::loadInstances()
{
    loadMap();
    loadTasks();
}

void MultibotServer::update_callback()
{
    auto robot_states = RobotStateArray();
    update_robotStates(robot_states);
    robot_states_pub_->publish(robot_states);
}

// Todo: Seperate Server and Robot
void MultibotServer::update_robotStates(RobotStateArray &_robot_states)
{
    _robot_states.robot_states.clear();

    for (auto &robot : robotList_)
    {
        auto robotState = RobotState();
        std::string robotName = robot.second.robotInfo_.name_;
        robotState.name = robotName;

        auto time = this->now();
        if (this->now() - nodeStartTime_ > rclcpp::Duration(10, 0))
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
        robotConfig.name = robot.second.robotInfo_.name_;
        robotConfig.size = robot.second.robotInfo_.size_;
        robotConfig.wheel_seperation = robot.second.robotInfo_.wheel_seperation_;
        robotConfig.wheel_radius = robot.second.robotInfo_.wheel_radius_;

        _response->configs.push_back(robotConfig);
    }
}

void MultibotServer::loadMap()
{
    while (!this->mapLoading_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_ERROR(this->get_logger(), "Map Loading from Map Server is not available, waiting again...");
    }
    // Todo: Let this member function to wait lifecycle node and map server node.(Wait dynamic time)
    rclcpp::sleep_for(500ms);

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    auto response_received_callback = [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture _future)
    {
        auto response = _future.get();

        MapInstance::BinaryOccupancyMap map;

        map.property_.resolution_ = response->map.info.resolution;
        map.property_.width_ = response->map.info.width;
        map.property_.height_ = response->map.info.height;
        map.property_.origin_.x_ = response->map.info.origin.position.x;
        map.property_.origin_.y_ = response->map.info.origin.position.y;

        map.mapData_.clear();
        for (uint32_t x = 0; x < response->map.info.width; x++)
        {
            std::vector<MapInstance::Cell> mapColumnData;
            mapColumnData.clear();
            for (uint32_t y = 0; y < response->map.info.height; y++)
            {
                MapInstance::Cell cell;
                cell.idx_.x_ = x;
                cell.idx_.y_ = y;
                cell.coord_.x_ = map.property_.origin_.x_ + x * map.property_.resolution_;
                cell.coord_.y_ = map.property_.origin_.y_ + y * map.property_.resolution_;
                cell.occupied_ = static_cast<bool>(response->map.data[x + y * map.property_.width_]);

                mapColumnData.push_back(cell);
            }
            map.mapData_.push_back(mapColumnData);
        }
        
        instance_manager_.saveMap(map);

        return;
    };

    auto future_result =
        mapLoading_->async_send_request(request, response_received_callback);
}

void MultibotServer::loadTasks()
{
    nodeStartTime_ = this->now();
    std::list<std::string> robotTypes;  robotTypes.clear();
    std::unordered_map<std::string, AgentInstance::Agent> agentList;
    std::string task_fPath;

    this->declare_parameter("task_fPath");
    this->get_parameter("task_fPath", task_fPath);

    std::vector<YAML::Node> tasks = YAML::LoadAllFromFile(task_fPath);

    for (const auto &task : tasks)
    {
        AgentInstance::Agent agent;
        
        agent.name_         = task["name"].as<std::string>();
        std::string type    = task["type"].as<std::string>();

        if (std::find(robotTypes.begin(), robotTypes.end(), type) == robotTypes.end())
        {
            this->declare_parameter(type + ".size");
            this->declare_parameter(type + ".wheels.separation");
            this->declare_parameter(type + ".wheels.radius");

            this->declare_parameter(type + ".linear.velocity");
            this->declare_parameter(type + ".linear.acceleration");
            this->declare_parameter(type + ".angular.velocity");
            this->declare_parameter(type + ".angular.acceleration");

            robotTypes.push_back(type);
        }

        this->get_parameter_or(type + ".size", agent.size_, 0.0);
        this->get_parameter_or(type + ".wheels.separation", agent.wheel_seperation_, 0.0);
        this->get_parameter_or(type + ".wheels.radius", agent.wheel_radius_, 0.0);

        this->get_parameter_or(type + ".linear.velocity", agent.max_linVel_, 0.0);
        this->get_parameter_or(type + ".linear.acceleration", agent.max_linAcc_, 0.0);
        this->get_parameter_or(type + ".angular.velocity", agent.max_angVel_, 0.0);
        this->get_parameter_or(type + ".angular.acceleration", agent.max_angAcc_, 0.0);

        geometry_msgs::msg::Pose2D startPose;
        startPose.x = task["start"]["x"].as<double>();
        startPose.y = task["start"]["y"].as<double>();
        startPose.theta = task["start"]["theta"].as<double>();
        agent.start_.component_ = startPose;

        geometry_msgs::msg::Pose2D goalPose;
        goalPose.x = task["goal"]["x"].as<double>();
        goalPose.y = task["goal"]["y"].as<double>();
        goalPose.theta = task["goal"]["theta"].as<double>();
        agent.goal_.component_  = goalPose;

        agentList.insert(std::make_pair(agent.name_, agent));

        Robot robot;
        robot.robotInfo_    = agent;
        robot.pose_         = agent.start_.component_; 

        robotList_.insert(std::make_pair(robot.robotInfo_.name_, robot));
    }
    instance_manager_.saveAgents(agentList);
}

void MultibotServer::updateRobotPose(const std::string &_robotName)
{
    Robot &robot = robotList_[_robotName];
    robot.localTimer_ = robot.localTimer_ + 10ms;

    if (std::fabs(robot.robotInfo_.goal_.component_.x - robot.pose_.x) < 1e-8 and
        std::fabs(robot.robotInfo_.goal_.component_.y - robot.pose_.y) < 1e-8)
        return;

    double theta = std::atan2(robot.robotInfo_.goal_.component_.y - robot.robotInfo_.start_.component_.y,
                              robot.robotInfo_.goal_.component_.x - robot.robotInfo_.start_.component_.x);
    double max_v_x = robot.robotInfo_.max_linVel_ * std::cos(theta);
    double max_v_y = robot.robotInfo_.max_linVel_ * std::sin(theta);
    double max_a_x = robot.robotInfo_.max_linAcc_ * std::cos(theta);
    double max_a_y = robot.robotInfo_.max_linAcc_ * std::sin(theta);

    robot.pose_.x = displacementComputer(robot.robotInfo_.start_.component_.x, robot.robotInfo_.goal_.component_.x, robot.localTimer_.count() * 1e-3, max_v_x, max_a_x);
    robot.pose_.y = displacementComputer(robot.robotInfo_.start_.component_.y, robot.robotInfo_.goal_.component_.y, robot.localTimer_.count() * 1e-3, max_v_y, max_a_y);
}

double MultibotServer::displacementComputer(const double start_, const double goal_, const double t_,
                                            const double max_velocity_, const double max_acceleration_)
{
    double max_s = std::fabs(goal_ - start_);

    if (max_s < 1e-8)
        return start_;

    int sign = start_ > goal_ ? -1 : 1;

    double v = std::fabs(max_velocity_);
    double a = std::fabs(max_acceleration_);

    double s = 0;
    // Trapezoidal velocity profile
    if (max_s > v * v / a)
    {
        if (t_ < v / a)
            s = std::fabs(0.5 * a * t_ * t_);
        else if (t_ < max_s / v)
            s = std::fabs(v * t_ - 0.5 * v * v / a);
        else
            s = std::fabs(max_s - 0.5 * a * std::pow((max_s / v + v / a - t_), 2));
    }
    // Triangular velocity profile
    else
    {
        if (t_ < std::fabs(0.5 * a * t_ * t_))
            s = std::fabs(0.5 * a * t_ * t_);
        else
            s = std::fabs(max_s - 0.5 * a * std::pow((2 * std::sqrt(max_s / a) - t_), 2));
    }

    return (start_ + sign * s);
}

MultibotServer::MultibotServer()
    : Node("server")
{
    // read_task();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    mapLoading_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

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