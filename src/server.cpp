#include "multibot_server/server.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <QApplication>

using namespace Server;
using namespace Instance;

void MultibotServer::execServerPanel(int argc, char *argv[])
{
    QApplication app(argc, argv);

    serverPanel_ = std::make_shared<Panel>();
    serverPanel_->attach(*this);
    serverPanel_->show();

    pannel_is_running_ = true;

    app.exec();
}

void MultibotServer::plan_multibots()
{
    paths_.clear();
    auto plans = solver_->solve();

    if (plans.second == true)
    {
        paths_ = plans.first;

        for (const auto singlePath : paths_)
            std::cout << singlePath.second << std::endl;

        instance_manager_->exportResult(paths_);
    }
}

void MultibotServer::request_controls()
{
    if (paths_.empty())
        return;

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

    visualization_msgs::msg::MarkerArray arrowArray;
    arrowArray.markers.clear();
    for (const auto &robot : robotList_)
    {
        if (robot.second.prior_update_time_.seconds() < 1e-8)
            continue;

        auto robotMarker = update_Rviz_SinglePose(robot.second);

        if (robotMarker.ns == "")
            continue;
        arrowArray.markers.push_back(robotMarker);
    }

    rviz_poses_pub_->publish(arrowArray);

    if (pannel_is_running_ and
        robotList_.contains(panelActivatedRobot_) and
        robotList_[panelActivatedRobot_].mode_ == PanelUtil::Mode::REMOTE)
    {
        geometry_msgs::msg::Twist cmd_vel;
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }

        if (serverPanel_->getCurrentTabIndex() == PanelUtil::Tab::ROBOT)
            cmd_vel = serverPanel_->get_cmd_vel();

        robotList_[panelActivatedRobot_].cmd_vel_pub_->publish(cmd_vel);
    }
}

void MultibotServer::robotState_callback(const RobotState::SharedPtr _state_msg)
{
    robotList_[_state_msg->name].prior_update_time_ = robotList_[_state_msg->name].last_update_time_;
    robotList_[_state_msg->name].last_update_time_ = this->now();
    robotList_[_state_msg->name].robotInfo_.pose_.component_ = _state_msg->pose;
    robotList_[_state_msg->name].robotInfo_.linVel_ = _state_msg->lin_vel;
    robotList_[_state_msg->name].robotInfo_.angVel_ = _state_msg->ang_vel;

    if (_state_msg->name == panelActivatedRobot_)
    {
        serverPanel_->setModeState(
            robotList_[_state_msg->name].mode_);

        if (not(serverPanel_->getModeState() == PanelUtil::Mode::REMOTE))
        {
            serverPanel_->setVelocity(
                _state_msg->lin_vel, _state_msg->ang_vel);
        }
    }
}

void MultibotServer::register_robot(
    const std::shared_ptr<Connection::Request> _request,
    std::shared_ptr<Connection::Response> _response)
{
    if (robotList_.contains(_request->config.name))
    {
        _response->is_connected = true;
        return;
    }

    AgentInstance::Agent robotInfo;

    robotInfo.name_ = _request->config.name;

    robotInfo.type_ = _request->config.type;
    robotInfo.size_ = _request->config.size;
    robotInfo.wheel_radius_ = _request->config.wheel_radius;
    robotInfo.wheel_seperation_ = _request->config.wheel_seperation;

    robotInfo.max_linVel_ = _request->config.max_linvel;
    robotInfo.max_linAcc_ = _request->config.max_linacc;
    robotInfo.max_angVel_ = _request->config.max_angvel;
    robotInfo.max_angAcc_ = _request->config.max_angacc;

    robotInfo.goal_.component_ = _request->goal;

    instance_manager_->insertAgent(std::make_pair(robotInfo.name_, robotInfo));

    Robot robot;
    {
        robot.robotInfo_ = robotInfo;
        robot.id_ = robot_id_;
        robot_id_++;
        robot.mode_ = PanelUtil::Mode::MANUAL;

        robot.prior_update_time_ = this->now();
        robot.last_update_time_ = this->now();

        robot.state_sub_ = this->create_subscription<RobotState>(
            "/" + robotInfo.name_ + "/state", qos_,
            std::bind(&MultibotServer::robotState_callback, this, std::placeholders::_1));
        robot.control_cmd_ = this->create_client<Path>("/" + robotInfo.name_ + "/path");
        robot.cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robotInfo.name_ + "/cmd_vel", qos_);
        robot.kill_robot_cmd_ = this->create_publisher<std_msgs::msg::Bool>(
            "/" + robotInfo.name_ + "/kill", qos_);
        robot.modeFromRobot_ = this->create_service<ModeSelection>(
            "/" + robotInfo.name_ + "/modeFromRobot",
            std::bind(&MultibotServer::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));
        robot.modeFromServer_ = this->create_client<ModeSelection>("/" + robotInfo.name_ + "/modeFromServer");
    }
    robotList_.insert(std::make_pair(robot.robotInfo_.name_, robot));
    serverPanel_->addRobot(robot.robotInfo_.name_);

    RCLCPP_INFO(this->get_logger(), "New Robot " + robot.robotInfo_.name_ + " is registered.");
    RCLCPP_INFO(this->get_logger(), "Total Robots: " + std::to_string(robotList_.size()) + "EA");
    std::cout << robot.robotInfo_ << std::endl;

    _response->is_connected = true;
}

void MultibotServer::delete_robot(
    const std::shared_ptr<Disconnection::Request> _request,
    std::shared_ptr<Disconnection::Response> _response)
{
    expireRobot(_request->name);

    _response->is_disconnected = true;
}

void MultibotServer::change_robot_mode(
    const std::shared_ptr<ModeSelection::Request> _request,
    std::shared_ptr<ModeSelection::Response> _reponse)
{
    if (_request->is_remote == true)
    {
        robotList_[_request->name].mode_ = PanelUtil::Mode::REMOTE;
        serverPanel_->setVelocity(0.0, 0.0);
    }
    else
        robotList_[_request->name].mode_ = PanelUtil::Mode::MANUAL;

    _reponse->is_complete = true;
}

bool MultibotServer::request_modeChange(
    const std::string _robotName,
    bool _is_remote)
{
    while (!robotList_[_robotName].modeFromServer_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(this->get_logger(), "ModeChange not available, waiting again...");
    }

    auto request = std::make_shared<ModeSelection::Request>();

    request->name = _robotName;
    request->is_remote = _is_remote;

    auto response_received_callback = [this](rclcpp::Client<ModeSelection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        robotList_[_robotName].modeFromServer_->async_send_request(request, response_received_callback);

    return future_result.get()->is_complete;
}

void MultibotServer::request_control(
    const std::string &_robotName,
    std::shared_ptr<rclcpp::Client<MultibotServer::Path>> _service)
{
    auto request = std::make_shared<Path::Request>();

    request->path.clear();
    for (const auto &nodePair : paths_[_robotName].nodes_)
    {
        LocalPath localPath;
        localPath.start = nodePair.first.pose_.component_;
        localPath.goal = nodePair.second.pose_.component_;
        localPath.departure_time = nodePair.first.departure_time_.count();
        localPath.arrival_time = nodePair.second.arrival_time_.count();

        request->path.push_back(localPath);
    }
    request->start_time = 10.0;

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
    robotMarker.header.stamp = _robot.last_update_time_;
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

void MultibotServer::expireRobot(const std::string _robotName)
{
    instance_manager_->deleteAgent(_robotName);

    if (robotList_.contains(_robotName))
    {
        robotList_.erase(_robotName);
        serverPanel_->deleteRobot(_robotName);

        RCLCPP_INFO(this->get_logger(), "Robot " + _robotName + " is deleted.");
        RCLCPP_INFO(this->get_logger(), "Total Robots: " + std::to_string(robotList_.size()) + "EA\n");
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

void MultibotServer::update(const PanelUtil::Msg &_msg)
{
    if (not(pannel_is_running_))
        return;

    switch (std::get<0>(_msg))
    {
    case PanelUtil::Request::SET_GOAL:
    {
        std::string robotName = std::get<1>(_msg);
        geometry_msgs::msg::Pose2D goal = std::get<2>(_msg);
        if (robotList_.contains(robotName))
        {
            robotList_[robotName].robotInfo_.goal_.component_ = goal;
            instance_manager_->setGoal(robotName, goal);

            std::cout << "Changing the Goal of " << robotName << ": "
                      << robotList_[robotName].robotInfo_.goal_ << std::endl;
        }
        break;
    }

    case PanelUtil::Request::SET_TARGET:
    {
        panelActivatedRobot_ = std::get<1>(_msg);
        serverPanel_->storeGoal(
            robotList_[panelActivatedRobot_].robotInfo_.goal_.component_);
        break;
    }

    case PanelUtil::Request::SCAN:
    {
        std_msgs::msg::Bool scanActivated;
        scanActivated.data = true;

        serverScan_->publish(scanActivated);
        break;
    }

    case PanelUtil::Request::KILL:
    {
        if (std::get<1>(_msg) != panelActivatedRobot_)
            std::abort();

        std_msgs::msg::Bool killActivated;
        killActivated.data = true;
        robotList_[panelActivatedRobot_].kill_robot_cmd_->publish(killActivated);

        expireRobot(panelActivatedRobot_);
        panelActivatedRobot_ = std::string();

        break;
    }

    case PanelUtil::Request::REMOTE_REQUEST:
    {
        if (std::get<1>(_msg) != panelActivatedRobot_)
            std::abort();

        bool is_complete = request_modeChange(panelActivatedRobot_, true);

        if (is_complete)
        {
            robotList_[panelActivatedRobot_].mode_ = PanelUtil::Mode::REMOTE;
            serverPanel_->setVelocity(0.0, 0.0);
        }

        break;
    }

    case PanelUtil::Request::MANUAL_REQUEST:
    {
        if (std::get<1>(_msg) != panelActivatedRobot_)
            std::abort();

        bool is_complete = request_modeChange(panelActivatedRobot_, false);

        if (is_complete)
            robotList_[panelActivatedRobot_].mode_ = PanelUtil::Mode::MANUAL;

        break;
    }

    default:
        break;
    }
}

MultibotServer::MultibotServer()
    : Node("server")
{
    mapLoading_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    rviz_poses_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_list", qos_);

    connection_ = this->create_service<Connection>(
        "/connection",
        std::bind(&MultibotServer::register_robot, this, std::placeholders::_1, std::placeholders::_2));

    disconnection_ = this->create_service<Disconnection>(
        "/disconnection",
        std::bind(&MultibotServer::delete_robot, this, std::placeholders::_1, std::placeholders::_2));

    serverScan_ = this->create_publisher<std_msgs::msg::Bool>("/server_scan", qos_);

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotServer::update_callback, this));

    instance_manager_ = std::make_shared<Instance_Manager>();
    solver_ = std::make_shared<CPBS::Solver>(instance_manager_);

    loadMap();
    instance_manager_->notify();

    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}