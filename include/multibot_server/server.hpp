#pragma once

#include <vector>
#include <list>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/srv/connection.hpp"
#include "multibot_ros2_interface/srv/disconnection.hpp"
#include "multibot_ros2_interface/msg/local_path.hpp"
#include "multibot_ros2_interface/srv/path.hpp"

#include "multibot_server/CPBS.hpp"
#include "multibot_server/Instance_Manager.hpp"

using namespace High_Level_Engine;
using namespace Instance;

namespace Server
{
    using namespace std::chrono_literals;

    class MultibotServer : public rclcpp::Node
    {
    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using Connection        = multibot_ros2_interface::srv::Connection;
        using Disconnection     = multibot_ros2_interface::srv::Disconnection;
        using Path              = multibot_ros2_interface::srv::Path;
        using LocalPath         = multibot_ros2_interface::msg::LocalPath;

    private:
        struct Robot
        {
            AgentInstance::Agent robotInfo_;
            int32_t id_;

            rclcpp::Subscription<RobotState>::SharedPtr state_sub_;
            rclcpp::Client<Path>::SharedPtr control_cmd_;

            rclcpp::Time last_update_time_;
            rclcpp::Time prior_update_time_;
        };

    public:
        void plan_multibots();
        void request_controls();

    private:
        void update_callback();
        void robotState_callback(const RobotState::SharedPtr _state_msg);
        
        void register_robot(
            const std::shared_ptr<Connection::Request> _request,
            std::shared_ptr<Connection::Response> _response);

        void delete_robot(
            const std::shared_ptr<Disconnection::Request> _request,
            std::shared_ptr<Disconnection::Response> _response);
        
        void request_control(
            const std::string &_robotName,
            std::shared_ptr<rclcpp::Client<Path>> _service);
        
    private:
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

        rclcpp::Service<Connection>::SharedPtr connection_;
        rclcpp::Service<Disconnection>::SharedPtr disconnection_;

        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr mapLoading_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

    private:
        void loadMap();
        visualization_msgs::msg::Marker update_Rviz_SinglePose(const Robot &_robot);

    private:
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::Time nodeStartTime_;

        std::map<std::string, Robot> robotList_;
        MAPF_Util::Path::PathSet paths_;

        std::shared_ptr<CPBS::Solver> solver_;
        std::shared_ptr<Instance_Manager> instance_manager_;

        int32_t robot_id_ = 0;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server