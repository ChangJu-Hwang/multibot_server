#include "server/server.hpp"

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

using namespace Server;
using namespace Instance;

void MultibotServer::loadInstances()
{
    loadMap();
    loadTasks();
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

// void MultibotServer::request_controls()
// {
//     for (auto single_request : controller_action_clinets_)
//     {
//         request_control(single_request.first, single_request.second);
//     }
// }

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

void MultibotServer::request_registration(const std::string &_robotName,
                                          std::shared_ptr<rclcpp::Client<Server::MultibotServer::RobotInfo>> _service)
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

// // void MultibotServer::request_control(const std::string &_robotName,
// //                                      rclcpp_action::Client<Path>::SharedPtr &_control_client)
// // {
// //     if(not(_control_client))
// //         RCLCPP_WARN(this->get_logger(), "Action client not initialized");
    
// //     if(not(_control_client->wait_for_action_server(std::chrono::seconds(10))))
// //     {
// //         RCLCPP_WARN(this->get_logger(), "%s controller is not available.", _robotName);
// //         return;
// //     }

// //     //Todo: Amend to send CPBS results
// //     LocalPath localPath;
// //     localPath.start = robotList_[_robotName].robotInfo_.start_.component_;
// //     localPath.goal  = robotList_[_robotName].robotInfo_.goal_.component_;
// //     localPath.departure_time  = rclcpp::Time(2, 0, RCL_SYSTEM_TIME);

// //     auto goal_msg = Path::Goal();   goal_msg.path_segments.clear(); 
// //     goal_msg.path_segments.push_back(localPath);

// //     auto send_goal_options = rclcpp_action::Client<Path>::SendGoalOptions();
// //     send_goal_options.goal_response_callback = 
// //         std::bind(&MultibotServer::get_control_action_goal, this, std::placeholders::_1);
// //     send_goal_options.feedback_callback = 
// //         std::bind(&MultibotServer::get_control_action_feedback, this, std::placeholders::_1, std::placeholders::_2);
// //     send_goal_options.result_callback = 
// //         std::bind(&MultibotServer::get_control_action_result, this, std::placeholders::_1);

// //     _control_client->async_send_goal(goal_msg, send_goal_options);
// // }

// // void MultibotServer::get_control_action_goal(std::shared_future<GoalHandlePath::SharedPtr> _future)
// // {
// //     auto goal_handle = _future.get();
// //     if (!goal_handle)
// //         RCLCPP_WARN(this->get_logger(), "Control goal rejected.");
// //     else
// //         RCLCPP_INFO(this->get_logger(), "Control goal accepted.");
// // }

// // void MultibotServer::get_control_action_feedback(
// //     GoalHandlePath::SharedPtr,
// //     const std::shared_ptr<const Path::Feedback> _feedback)
// // {
// //     std::cout << _feedback->odom.pose.pose.position.x << ", "
// //               << _feedback->odom.pose.pose.position.y << std::endl;
// // }

// void MultibotServer::get_control_action_result(const GoalHandlePath::WrappedResult &_result)
// {
//     switch (_result.code)
//     {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//         {
//             RCLCPP_INFO(this->get_logger(), "Control action succeeded.");
//             Position::Pose resultPose(_result.result->pose);
//             std::cout << resultPose << std::endl;
//             return;
//         }
//         case rclcpp_action::ResultCode::ABORTED:
//         {
//             RCLCPP_WARN(this->get_logger(), "The control action was aborted.");
//             return;
//         }
//         case rclcpp_action::ResultCode::CANCELED:
//         {
//             RCLCPP_WARN(this->get_logger(), "The control action was canceled.");
//             return;
//         }
//         default:
//         {
//             RCLCPP_WARN(this->get_logger(), "Unkown result code");
//             return;
//         }
//     }
// }

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

        auto singleRobot_registration = this->create_client<RobotInfo>("/" + agent.name_ + "/robotInfo");
        registration_request_.push_back(std::make_pair(agent.name_, singleRobot_registration));

        // rclcpp_action::Client<Path>::SharedPtr singleRobot_controller_client = rclcpp_action::create_client<Path>(
        //     this->get_node_base_interface(),
        //     this->get_node_graph_interface(),
        //     this->get_node_logging_interface(),
        //     this->get_node_waitables_interface(),
        //     "/" + agent.name_ + "/controller");
        // controller_action_clinets_.push_back(std::make_pair(agent.name_, singleRobot_controller_client));

        Robot robot;
        robot.robotInfo_ = agent;
        robot.pose_ = agent.start_.component_;

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
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    mapLoading_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    robot_states_pub_ = this->create_publisher<RobotStateArray>("robot_states", qos);

    registration_server_ = this->create_service<RobotConfigs>("registration",
                                                              std::bind(&MultibotServer::send_robotConfigs, this, std::placeholders::_1, std::placeholders::_2));
    registration_request_.clear();
    // controller_action_clinets_.clear();

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotServer::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}