#pragma once

#include <vector>
#include <list>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"
#include "multibot_ros2_interface/msg/robot_config.hpp"
#include "multibot_ros2_interface/srv/robot_configs.hpp"
#include "multibot_ros2_interface/srv/robot_info.hpp"

#include "server/Instance_Manager.hpp"

using namespace Instance;

namespace Server
{
    using namespace std::chrono_literals;

    class MultibotServer : public rclcpp::Node
    {
    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfig       = multibot_ros2_interface::msg::RobotConfig;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;
        using RobotInfo         = multibot_ros2_interface::srv::RobotInfo;

    private:
        struct Robot
        {
            AgentInstance::Agent robotInfo_;
            int32_t id_;

            rclcpp::Subscription<RobotState>::SharedPtr state_sub_;

            rclcpp::Time last_update_time_;
            rclcpp::Time prior_update_time_;
        };

    public:
        void loadInstances();
        void request_registrations();

    private:
        void update_callback();
        // void update_robotStates(RobotStateArray &_robot_states);
        // void send_robotConfigs(const std::shared_ptr<RobotConfigs::Request> _request,
        //                        std::shared_ptr<RobotConfigs::Response> _response);
        void robotState_callback(const RobotState::SharedPtr _state_msg);
        
        void request_registration(const std::string &_robotName,
                                  std::shared_ptr<rclcpp::Client<Server::MultibotServer::RobotInfo>> _service);
        
        
    private:
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

        // rclcpp::Publisher<RobotStateArray>::SharedPtr robot_states_pub_;
        // rclcpp::Service<RobotConfigs>::SharedPtr registration_server_;
        std::vector<std::pair<std::string, rclcpp::Client<RobotInfo>::SharedPtr>> registration_request_;

        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr mapLoading_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

    private:
        void loadMap();
        void loadTasks();

        visualization_msgs::msg::Marker update_Rviz_SinglePose(const Robot &_robot);

        // Todo: Refactorying(Split, Generalize)
        // void updateRobotPose(const std::string &_robotName);
        // double displacementComputer(const double start_, const double goal_, const double t_,
        //                             const double max_velocity_, const double max_acceleration_);

    private:
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::Time nodeStartTime_;

        std::unordered_map<std::string, Robot> robotList_;
        Instance_Manager instance_manager_;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server