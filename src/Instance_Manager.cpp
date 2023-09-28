#include "multibot_server/Instance_Manager.hpp"

#include <mutex>
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

using namespace Instance;
using namespace std::chrono_literals;

void Instance_Manager::robotState_callback(const AgentInstance::Robot::State::SharedPtr _state_msg)
{
    auto &robot = robots_[_state_msg->name];

    robot.prior_update_time_ = robot.last_update_time_;
    robot.last_update_time_ = nh_->now();

    robot.robotInfo_.pose_.component_ = _state_msg->pose;
    robot.robotInfo_.linVel_ = _state_msg->lin_vel;
    robot.robotInfo_.angVel_ = _state_msg->ang_vel;
}

void Instance_Manager::loadMap()
{
    while (!this->mapLoader_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_ERROR(nh_->get_logger(), "Map Loading from Map Server is not available, waiting again...");
    }
    // Todo: Let this member function to wait lifecycle node and map server node.(Wait dynamic time)
    rclcpp::sleep_for(500ms);

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    auto result = mapLoader_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(nh_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
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

        map_ = map;

        notify();
    }
}

void Instance_Manager::insertRobot(const AgentInstance::Robot &_robot)
{
    if (robots_.contains(_robot.robotInfo_.name_))
    {
        return;
    }

    std::string robotName = _robot.robotInfo_.name_;

    AgentInstance::Robot robot = _robot;
    {
        robot.state_sub_ = nh_->create_subscription<AgentInstance::Robot::State>(
            "/" + robotName + "/state", qos_,
            std::bind(&Instance_Manager::robotState_callback, this, std::placeholders::_1));
        robot.send_traj_ = nh_->create_client<AgentInstance::Robot::Traj>("/" + robotName + "/path");
        robot.cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robotName + "/cmd_vel", qos_);
        robot.kill_robot_cmd_ = nh_->create_publisher<std_msgs::msg::Bool>(
            "/" + robotName + "/kill", qos_);
    }
    robots_.insert(std::make_pair(robot.robotInfo_.name_, robot));

    RCLCPP_INFO(nh_->get_logger(), "Instance_Manager::insertRobot()");
    RCLCPP_INFO(nh_->get_logger(), "New Robot " + robot.robotInfo_.name_ + " is registered.");
    RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA");
    std::cout << robot.robotInfo_ << std::endl;
}

void Instance_Manager::deleteRobot(
    const std::string _robotName)
{
    if (robots_.contains(_robotName))
    {
        robots_.erase(_robotName);

        RCLCPP_INFO(nh_->get_logger(), "Robot " + _robotName + " is deleted.");
        RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA\n");
    }
}

const AgentInstance::Robot &Instance_Manager::getRobot(const std::string _robotName) const
{
    const auto &robot = robots_.find(_robotName)->second;

    return robot;
}

void Instance_Manager::fixStartPoses()
{
    for (auto &robotPair : robots_)
        robotPair.second.robotInfo_.start_ = robotPair.second.robotInfo_.pose_;

    notify();
}

void Instance_Manager::sendTrajectories(Traj::TrajSet &_trajSet)
{
    for (auto &robotPair : robots_)
    {
        auto &robot = robotPair.second;

        while (!robot.send_traj_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_ERROR(nh_->get_logger(), "Sending %s trajectory not availiable, waiting again...", robot.robotInfo_.name_);
        }

        auto request = std::make_shared<AgentInstance::Robot::Traj::Request>();

        request->traj.clear();

        for (const auto &nodePair : _trajSet[robot.robotInfo_.name_].nodes_)
        {
            AgentInstance::Robot::LocalTraj LocalTraj;
            LocalTraj.start = nodePair.first.pose_.component_;
            LocalTraj.goal = nodePair.second.pose_.component_;
            LocalTraj.departure_time = nodePair.first.departure_time_.count();
            LocalTraj.arrival_time = nodePair.second.arrival_time_.count();

            request->traj.push_back(LocalTraj);
        }
        request->start_time = 0.0;

        auto response_received_callback = [this](rclcpp::Client<AgentInstance::Robot::Traj>::SharedFuture _future)
        {
            auto response = _future.get();
            return;
        };

        auto future_rusult =
            robot.send_traj_->async_send_request(request, response_received_callback);

        robot.mode_ = PanelUtil::Mode::AUTO;
    }
}

void Instance_Manager::setGoal(
    const std::string _robotName, const geometry_msgs::msg::Pose2D _goal)
{
    if (robots_.contains(_robotName))
    {
        robots_[_robotName].robotInfo_.goal_.component_ = _goal;

        std::cout << "Changing the Goal of " << _robotName << ": "
                  << robots_[_robotName].robotInfo_.goal_ << std::endl;

        notify();
    }
}

void Instance_Manager::setMode(
    const std::string _robotName, const PanelUtil::Mode _mode)
{
    if (not(robots_.contains(_robotName)))
        return;

    robots_[_robotName].mode_ = _mode;

    if (_mode == PanelUtil::Mode::REMOTE)
    {
        geometry_msgs::msg::Twist stop_cmd_vel;
        {
            stop_cmd_vel.linear.x = 0.0;
            stop_cmd_vel.angular.z = 0.0;
        }
        robots_[_robotName].cmd_vel_pub_->publish(stop_cmd_vel);
    }
}

void Instance_Manager::request_modeChange(
    const std::string _robotName, const PanelUtil::Mode _mode)
{
    if (not(robots_.contains(_robotName)))
        return;

    auto &robot = robots_[_robotName];

    while (!robot.modeFromServer_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_ERROR(nh_->get_logger(), "ModeChange not available, waiting again...");
    }

    auto request = std::make_shared<PanelUtil::ModeSelection::Request>();

    request->name = _robotName;

    if (_mode == PanelUtil::Mode::REMOTE)
        request->is_remote = true;
    else if (_mode == PanelUtil::Mode::MANUAL)
        request->is_remote = false;
    else
    {
    }

    auto response_received_callback = [this](rclcpp::Client<PanelUtil::ModeSelection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        robot.modeFromServer_->async_send_request(request, response_received_callback);

    if (future_result.get()->is_complete)
        robot.mode_ = _mode;

    return;
}

void Instance_Manager::request_kill(const std::string _robotName)
{
    if (not(robots_.contains(_robotName)))
        return;

    std_msgs::msg::Bool killActivated;
    killActivated.data = true;
    robots_[_robotName].kill_robot_cmd_->publish(killActivated);
}

void Instance_Manager::remote_control(
    const std::string _robotName, const geometry_msgs::msg::Twist &_remote_cmd_vel)
{
    if (robots_.contains(_robotName))
        robots_[_robotName].cmd_vel_pub_->publish(_remote_cmd_vel);
}

void Instance_Manager::attach(Observer::ObserverInterface<InstanceMsg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    if (std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
        list_observer_.push_back(&_observer);
}

void Instance_Manager::detach(Observer::ObserverInterface<InstanceMsg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    auto observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    while (observer_iter != list_observer_.end())
    {
        list_observer_.remove(&_observer);
        observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    }
}

void Instance_Manager::notify()
{
    std::scoped_lock<std::mutex> lock(mtx_);

    std::unordered_map<std::string, AgentInstance::Agent> agents;
    {
        agents.clear();

        for (const auto &robotPair : robots_)
        {
            std::string robotName = robotPair.first;
            AgentInstance::Agent robotInfo = robotPair.second.robotInfo_;

            agents.insert(std::make_pair(robotName, robotInfo));
        }
    }

    Instance::InstanceMsg msg;
    msg.first = agents;
    msg.second = map_;

    for (auto &observer : list_observer_)
        observer->update(msg);
}

Instance_Manager::Instance_Manager(std::shared_ptr<rclcpp::Node> _nh)
    : nh_(_nh)
{
    robots_.clear();

    mapLoader_ = nh_->create_client<nav_msgs::srv::GetMap>("/map_server/map");
    loadMap();

    std::cout << "Instance Manager has been initialzied" << std::endl;
}