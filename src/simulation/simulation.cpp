#include "simulation/simulation.hpp"

#include <tf2/LinearMath/Quaternion.h>

using namespace Simulation;
using namespace std::chrono_literals;

void MultibotSim::update_callback()
{
    visualization_msgs::msg::MarkerArray arrowArray;
    update_rviz(arrowArray);
    rviz_poses_pub_->publish(arrowArray);
}

void MultibotSim::update_rviz(visualization_msgs::msg::MarkerArray &_markerArray)
{
    _markerArray.markers.clear();

    int32_t id = 0;
    for(const auto &robot_state : robotList_)
    {
        rclcpp::Time time_now = this->now();
        rclcpp::Duration duration(time_now - prev_rviz_update_time_);
        prev_rviz_update_time_ = time_now;

        auto robotMarker = visualization_msgs::msg::Marker();

        robotMarker.header.frame_id     = "/map";
        robotMarker.header.stamp        = this->now();
        robotMarker.id                  = id++;
        robotMarker.type                = visualization_msgs::msg::Marker::ARROW;
        robotMarker.action              = visualization_msgs::msg::Marker::ADD;

        robotMarker.scale.x             = 0.5;
        robotMarker.scale.y             = 0.125;
        robotMarker.scale.z             = 0.125;

        robotMarker.color.r             = 0.5;
        robotMarker.color.g             = 0.5;
        robotMarker.color.b             = 1.0;
        robotMarker.color.a             = 1.0;

        robotMarker.lifetime            = duration;

        robotMarker.pose.position.x     = robot_state.second.pose_[Pose::x];
        robotMarker.pose.position.y     = robot_state.second.pose_[Pose::y];
        robotMarker.pose.position.z     = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_state.second.pose_[Pose::theta]);
        robotMarker.pose.orientation.x  = q.x();
        robotMarker.pose.orientation.y  = q.y();
        robotMarker.pose.orientation.z  = q.z();
        robotMarker.pose.orientation.w  = q.w();

        _markerArray.markers.push_back(robotMarker);
    }
}

void MultibotSim::robot_states_callback(const multibot_ros2_interface::msg::RobotStateArray::SharedPtr _robot_states)
{
    for(const auto& robot_state : _robot_states->robot_states)
        update_robotList(robot_state);
}

void MultibotSim::init_variables()
{
    robotList_.clear();
    prev_rviz_update_time_ = this->now();
}

void MultibotSim::update_robotList(const multibot_ros2_interface::msg::RobotState &_robot_state)
{
    if(robotList_.find(_robot_state.robot_name) != robotList_.end())
    {
        robotList_[_robot_state.robot_name].size_               = _robot_state.robot_size;
        robotList_[_robot_state.robot_name].pose_[Pose::x]      = _robot_state.robot_x;
        robotList_[_robot_state.robot_name].pose_[Pose::y]      = _robot_state.robot_y;
        robotList_[_robot_state.robot_name].pose_[Pose::theta]  = _robot_state.robot_theta;
    }
    else
    {
        Robot robotInfo;
            robotInfo.name_ = _robot_state.robot_name;
            robotInfo.size_ = _robot_state.robot_size;

            robotInfo.pose_.clear();
            robotInfo.pose_.push_back(_robot_state.robot_x);
            robotInfo.pose_.push_back(_robot_state.robot_y);
            robotInfo.pose_.push_back(_robot_state.robot_theta);
        
        robotList_.insert(std::make_pair(_robot_state.robot_name, robotInfo));
    }
}

MultibotSim::MultibotSim()
: Node("simulation")
{
    init_variables();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    rviz_poses_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_list", qos);

    robot_states_sub_ = this->create_subscription<multibot_ros2_interface::msg::RobotStateArray>(
        "robot_states", qos, std::bind(&MultibotSim::robot_states_callback, this, std::placeholders::_1)
    );

    update_timer_ = this->create_wall_timer(10ms, std::bind(&MultibotSim::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "MultibotSim has been initialized");
}

MultibotSim::~MultibotSim()
{
    RCLCPP_INFO(this->get_logger(), "MultibotSim has been terminated");
}