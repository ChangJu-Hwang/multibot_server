#include "simulation/simulation.hpp"

#include <tf2/LinearMath/Quaternion.h>

using namespace Simulation;
using namespace std::chrono_literals;

void MultibotSim::update_callback()
{
    visualization_msgs::msg::MarkerArray arrowArray;
    update_rviz(arrowArray);
    rviz_poses_pub_->publish(arrowArray);

    update_gazebo();
}

void MultibotSim::update_rviz(visualization_msgs::msg::MarkerArray &_markerArray)
{
    _markerArray.markers.clear();

    int32_t id = 0;
    for (const auto &robot_state : robotList_)
    {
        auto robotMarker = visualization_msgs::msg::Marker();

        robotMarker.header.frame_id = "/map";
        robotMarker.header.stamp = this->now();
        robotMarker.id = id++;
        robotMarker.type = visualization_msgs::msg::Marker::ARROW;
        robotMarker.action = visualization_msgs::msg::Marker::ADD;

        robotMarker.scale.x = 0.5;
        robotMarker.scale.y = 0.125;
        robotMarker.scale.z = 0.125;

        robotMarker.color.r = 0.5;
        robotMarker.color.g = 0.5;
        robotMarker.color.b = 1.0;
        robotMarker.color.a = 1.0;

        robotMarker.lifetime = robot_state.second.time_now_ - robot_state.second.prior_time_;

        robotMarker.pose.position.x = robot_state.second.pose_.x;
        robotMarker.pose.position.y = robot_state.second.pose_.y;
        robotMarker.pose.position.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_state.second.pose_.theta);
        robotMarker.pose.orientation.x = q.x();
        robotMarker.pose.orientation.y = q.y();
        robotMarker.pose.orientation.z = q.z();
        robotMarker.pose.orientation.w = q.w();

        _markerArray.markers.push_back(robotMarker);
    }
}

void MultibotSim::update_gazebo()
{
    for (const auto &robot_state : robotList_)
    {
        geometry_msgs::msg::Twist cmd_vel = update_cmd_vel(robot_state.second);
        robot_state.second.cmd_vel_pub_->publish(cmd_vel);
    }
}

geometry_msgs::msg::Twist MultibotSim::update_cmd_vel(const Robot &_robot)
{

    double delta_x      = _robot.pose_.x - _robot.prior_pose_.x;
    double delta_y      = _robot.pose_.y - _robot.prior_pose_.y;

    double delta_s      = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    double delta_theta  = _robot.pose_.theta - _robot.prior_pose_.theta;

    double delta_t      = (_robot.time_now_ - _robot.prior_time_).nanoseconds() / 1e9;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x    = delta_s / delta_t;
    cmd_vel.angular.z   = delta_theta / delta_t;

    return cmd_vel;
}

void MultibotSim::robot_states_callback(const RobotStateArray::SharedPtr _robot_states)
{
    for (const auto &robot_state : _robot_states->robot_states)
        update_robotList(robot_state);
}

void MultibotSim::registration_request()
{
    auto request = std::make_shared<RobotConfigs::Request>();

    auto response_received_callback = [this](rclcpp::Client<RobotConfigs>::SharedFuture _future)
    {
        auto response = _future.get();
        for (const auto &config : response->configs)
        {
            Robot robot;
                robot.name_ = config.name;
                
                robot.prior_time_   = this->now();
                robot.time_now_     = this->now();

                robot.size_ = config.size;
                robot.wheel_radius_ = config.wheel_radius;
                robot.wheel_seperation_ = config.wheel_seperation;

                robot.is_initial_pose_ = true;

                robot.cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + config.name + "/cmd_vel", qos_);

            robotList_.insert(std::make_pair(config.name, robot));
        }
        return;
    };

    auto future_result =
        registration_->async_send_request(request, response_received_callback);
}

void MultibotSim::init_variables()
{
    robotList_.clear();
    prev_rviz_update_time_ = this->now();
    prev_gazebo_update_time_ = this->now();
}

void MultibotSim::update_robotList(const RobotState &_robot_state)
{
    try
    {
        if (robotList_.find(_robot_state.name) == robotList_.end())
            throw _robot_state.name;

        if (robotList_[_robot_state.name].is_initial_pose_)
        {
            robotList_[_robot_state.name].prior_pose_ = _robot_state.pose;
            robotList_[_robot_state.name].is_initial_pose_ = false;
        }
        else
        {
            robotList_[_robot_state.name].prior_pose_ = robotList_[_robot_state.name].pose_;
        }
        robotList_[_robot_state.name].pose_ = _robot_state.pose;

        robotList_[_robot_state.name].prior_time_ = robotList_[_robot_state.name].time_now_;
        robotList_[_robot_state.name].time_now_ = this->now();
    }
    catch (std::string _err_robot_name)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot find %s in the robot list.", _err_robot_name);
    }
}

MultibotSim::MultibotSim()
    : Node("simulation")
{
    init_variables();

    registration_ = this->create_client<RobotConfigs>("registration");
    while (!registration_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_ERROR(this->get_logger(), "Robot registration not available, waiting again...");
    }

    rviz_poses_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_list", qos_);

    robot_states_sub_ = this->create_subscription<RobotStateArray>(
        "robot_states", qos_, std::bind(&MultibotSim::robot_states_callback, this, std::placeholders::_1));

    update_timer_ = this->create_wall_timer(10ms, std::bind(&MultibotSim::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "MultibotSim has been initialized");
}

MultibotSim::~MultibotSim()
{
    RCLCPP_INFO(this->get_logger(), "MultibotSim has been terminated");
}