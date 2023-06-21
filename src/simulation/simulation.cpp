#include "simulation/simulation.hpp"

#include <future>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace Simulation;
using namespace std::chrono_literals;

void MultibotSim::register_robots()
{
    while (!this->registration_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_ERROR(this->get_logger(), "Robot registration not available, waiting again...");
    }
    request_registration();
}

void MultibotSim::request_registration()
{
    auto request = std::make_shared<RobotConfigs::Request>();

    auto response_received_callback = [this](rclcpp::Client<RobotConfigs>::SharedFuture _future)
    {
        auto response = _future.get();
        for (const auto &config : response->configs)
        {
            Robot robot;
            robot.name_             = config.name;
            robot.id_               = robotList_.size();

            robot.prior_time_       = this->now();
            robot.time_now_         = this->now();

            robot.size_             = config.size;
            robot.wheel_radius_     = config.wheel_radius;
            robot.wheel_seperation_ = config.wheel_seperation;

            robot.is_initial_pose_  = true;

            robot.cmd_vel_pub_      = this->create_publisher<geometry_msgs::msg::Twist>("/" + config.name + "/cmd_vel" ,qos_);

            robotList_.insert(std::make_pair(config.name, robot));
        }
        registrationFlag_ = true;
        return;
    };

    auto future_result = 
        registration_->async_send_request(request, response_received_callback);
}

// void MultibotSim::set_odomSubscribers()
// {
//     for(auto& robot : robotList_)
//     {
//         auto thread = std::async(std::launch::async, &MultibotSim::set_odomSubscriber, this, std::ref(robot.second));
//     }
// }

// void MultibotSim::set_odomSubscriber(Robot &_robot)
// {
//     _robot.odom_sub_    = this->create_subscription<nav_msgs::msg::Odometry>
//     (
//         "/" + _robot.name_ + "/odom", qos_,
//         [&_robot](const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
//         {
//             _robot.gazebo_pose_.x      = _odom_msg->pose.pose.position.x;
//             _robot.gazebo_pose_.y      = _odom_msg->pose.pose.position.y;

//             tf2::Quaternion q
//             (
//                 _odom_msg->pose.pose.orientation.x,
//                 _odom_msg->pose.pose.orientation.y,
//                 _odom_msg->pose.pose.orientation.z,
//                 _odom_msg->pose.pose.orientation.w
//             );
//             tf2::Matrix3x3 m(q);
//             double roll, pitch, yaw;
//             m.getRPY(roll, pitch, yaw);

//             _robot.gazebo_pose_.theta  = yaw;
//         }
//     );
// }

void MultibotSim::update_callback()
{
    std::vector<std::future<visualization_msgs::msg::Marker>> rviz_threads;
    rviz_threads.clear();

    for (const auto &robot_state : robotList_)
    {
        rviz_threads.push_back(std::async(std::launch::async, &MultibotSim::update_rviz, this, robot_state.second));
        auto gazebo_thread = std::async(std::launch::async, &MultibotSim::update_gazebo, this, robot_state.second);
    }

    visualization_msgs::msg::MarkerArray arrowArray;
    arrowArray.markers.clear();
    for (auto &rviz_thread : rviz_threads)
    {
        arrowArray.markers.push_back(rviz_thread.get());
    }

    rviz_poses_pub_->publish(arrowArray);
}

visualization_msgs::msg::Marker MultibotSim::update_rviz(const Robot &_robot)
{
    auto robotMarker = visualization_msgs::msg::Marker();

    robotMarker.header.frame_id = "/map";
    robotMarker.header.stamp = _robot.time_now_;
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

    robotMarker.lifetime = _robot.time_now_ - _robot.prior_time_;

    robotMarker.pose.position.x = _robot.pose_.x;
    robotMarker.pose.position.y = _robot.pose_.y;
    robotMarker.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, _robot.pose_.theta);
    robotMarker.pose.orientation.x = q.x();
    robotMarker.pose.orientation.y = q.y();
    robotMarker.pose.orientation.z = q.z();
    robotMarker.pose.orientation.w = q.w();

    return robotMarker;
}

void MultibotSim::update_gazebo(const Robot &_robot)
{
    double delta_x = _robot.pose_.x - _robot.prior_pose_.x;
    double delta_y = _robot.pose_.y - _robot.prior_pose_.y;

    double delta_s = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    double delta_theta = _robot.pose_.theta - _robot.prior_pose_.theta;

    double delta_t = (_robot.time_now_ - _robot.prior_time_).nanoseconds() / 1e9;

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = delta_s / delta_t;
    cmd_vel_msg.angular.z = delta_theta / delta_t;

    _robot.cmd_vel_pub_->publish(cmd_vel_msg);
}

void MultibotSim::robot_states_callback(const RobotStateArray::SharedPtr _robot_states)
{
    for (const auto &robot_state : _robot_states->robot_states)
    {
        if(registrationFlag_)
            update_robotList(robot_state);
    }
}

void MultibotSim::update_robotList(const RobotState &_robot_state)
{
    try
    {
        if (robotList_.find(_robot_state.name) == robotList_.end())
            std::cout << _robot_state.name << std::endl;

        if (robotList_[_robot_state.name].is_initial_pose_)
        {
            robotList_[_robot_state.name].prior_pose_       = _robot_state.pose;
            robotList_[_robot_state.name].gazebo_pose_      = _robot_state.pose;
            robotList_[_robot_state.name].is_initial_pose_  = false;
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
    robotList_.clear();

    registration_ = this->create_client<RobotConfigs>("registration");

    // robot_odom_sub_list_.clear();

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