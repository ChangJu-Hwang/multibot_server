#include "multibot_server/server.hpp"

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

#include <tf2/LinearMath/Quaternion.h>

using namespace Server;
using namespace Instance;

void MultibotServer::loadInstances()
{
    loadMap();
    loadTasks();

    instance_manager_->notify();
}

void MultibotServer::request_registrations()
{
    for (auto single_request : registration_request_)
    {
        while (!single_request.second->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_ERROR(this->get_logger(), "Robot registration not available, waiting again...");
        }
        request_registration(single_request.first, single_request.second);
    }
}

void MultibotServer::plan_multibots()
{
    auto foo = solver_->solve();

    for (const auto singlePath : foo.first)
    {   
        std::cout << singlePath.second << std::endl;
    }
}

void MultibotServer::request_controls()
{
    for (auto robot : robotList_)
    {
        while (!robot.second.control_cmd_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_ERROR(this->get_logger(), "Send path not availiable, waiting again...");
        }
        request_control(robot.second.robotInfo_.name_, robot.second.control_cmd_);
    }
}

void MultibotServer::update_callback()
{
    if (robotList_.empty())
        return;
    
    visualization_msgs::msg::MarkerArray arrowArray;    arrowArray.markers.clear();
    for(const auto& robot : robotList_)
    {
        if(robot.second.prior_update_time_.seconds() < 1e-8)
            continue;

        auto robotMarker = update_Rviz_SinglePose(robot.second);
       
        if (robotMarker.ns == "")
            continue;
        arrowArray.markers.push_back(robotMarker);
    }

    rviz_poses_pub_->publish(arrowArray);
}

void MultibotServer::robotState_callback(const RobotState::SharedPtr _state_msg)
{
    robotList_[_state_msg->name].prior_update_time_ = robotList_[_state_msg->name].last_update_time_;
    robotList_[_state_msg->name].last_update_time_ = this->now();
    robotList_[_state_msg->name].robotInfo_.pose_.component_   = _state_msg->pose;
}

void MultibotServer::request_registration(
    const std::string &_robotName,
    std::shared_ptr<rclcpp::Client<MultibotServer::RobotInfo>> _service)
{
    auto request = std::make_shared<RobotInfo::Request>();

    request->config.name = _robotName;
    request->config.size = robotList_[_robotName].robotInfo_.size_;
    request->config.wheel_radius = robotList_[_robotName].robotInfo_.wheel_radius_;
    request->config.wheel_seperation = robotList_[_robotName].robotInfo_.wheel_seperation_;

    request->config.max_linvel = robotList_[_robotName].robotInfo_.max_linVel_;
    request->config.max_linacc = robotList_[_robotName].robotInfo_.max_linAcc_;
    request->config.max_angvel = robotList_[_robotName].robotInfo_.max_angVel_;
    request->config.max_angacc = robotList_[_robotName].robotInfo_.max_angAcc_;

    auto response_received_callback = [this](rclcpp::Client<RobotInfo>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        _service->async_send_request(request, response_received_callback);
}

void MultibotServer::request_control(
    const std::string &_robotName,
    std::shared_ptr<rclcpp::Client<MultibotServer::Path>> _service)
{
    auto request = std::make_shared<Path::Request>();

    // Todo: Change to MAPF Result
    LocalPath localPath;
        localPath.start = robotList_[_robotName].robotInfo_.start_.component_;
        localPath.goal  = robotList_[_robotName].robotInfo_.goal_.component_;
        localPath.departure_time = 10;
        localPath.arrival_time   = 20;
    
    request->path.clear();
    request->path.push_back(localPath);
    request->start_time = 0.0;

    auto response_received_callback = [this](rclcpp::Client<Path>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_rusult = 
        _service->async_send_request(request, response_received_callback);
}

visualization_msgs::msg::Marker MultibotServer::update_Rviz_SinglePose(const Robot &_robot)
{
    auto robotMarker = visualization_msgs::msg::Marker();

    robotMarker.header.frame_id = "map";
    robotMarker.header.stamp    = _robot.last_update_time_;
    robotMarker.ns = _robot.robotInfo_.name_;
    robotMarker.id = _robot.id_;
    robotMarker.type = visualization_msgs::msg::Marker::ARROW;
    robotMarker.action = visualization_msgs::msg::Marker::ADD;

    robotMarker.scale.x = 0.5;
    robotMarker.scale.y = 0.125;
    robotMarker.scale.z = 0.125;

    robotMarker.color.r = 0.5;
    robotMarker.color.g = 0.5;
    robotMarker.color.b = 1.0;
    robotMarker.color.a = 1.0;

    robotMarker.lifetime = _robot.last_update_time_ - _robot.prior_update_time_;

    robotMarker.pose.position.x = _robot.robotInfo_.pose_.component_.x;
    robotMarker.pose.position.y = _robot.robotInfo_.pose_.component_.y;
    robotMarker.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, _robot.robotInfo_.pose_.component_.theta);
    robotMarker.pose.orientation.x = q.x();
    robotMarker.pose.orientation.y = q.y();
    robotMarker.pose.orientation.z = q.z();
    robotMarker.pose.orientation.w = q.w();

    return robotMarker;
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

    auto result = mapLoading_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();

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

        instance_manager_->saveMap(map);
    }
}

void MultibotServer::loadTasks()
{
    nodeStartTime_ = this->now();
    std::list<std::string> robotTypes;
    robotTypes.clear();
    std::unordered_map<std::string, AgentInstance::Agent> agentList;
    std::string task_fPath;

    this->declare_parameter("task_fPath");
    this->get_parameter("task_fPath", task_fPath);

    std::vector<YAML::Node> tasks = YAML::LoadAllFromFile(task_fPath);

    for (const auto &task : tasks)
    {
        AgentInstance::Agent agent;

        agent.name_ = task["name"].as<std::string>();
        std::string type = task["type"].as<std::string>();

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
        agent.goal_.component_ = goalPose;

        agentList.insert(std::make_pair(agent.name_, agent));

        auto singleRobot_registration = this->create_client<RobotInfo>("/" + agent.name_ + "/info");
        registration_request_.push_back(std::make_pair(agent.name_, singleRobot_registration));

        Robot robot;
        robot.robotInfo_ = agent;
        robot.id_        = robotList_.size();

        robot.prior_update_time_ = this->now();
        robot.last_update_time_  = this->now();

        robot.state_sub_ = this->create_subscription<RobotState>(
            "/" + agent.name_ + "/state", qos_,
            std::bind(&MultibotServer::robotState_callback, this, std::placeholders::_1));
        robot.control_cmd_ = this->create_client<Path>("/" + agent.name_ + "/path");

        robotList_.insert(std::make_pair(robot.robotInfo_.name_, robot));
    }
    instance_manager_->saveAgents(agentList);
}

MultibotServer::MultibotServer()
    : Node("server")
{
    mapLoading_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    rviz_poses_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_list", qos_);

    registration_request_.clear();

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotServer::update_callback, this));

    instance_manager_   = std::make_shared<Instance_Manager>();
    solver_             = std::make_shared<CPBS::Solver>(instance_manager_);

    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}