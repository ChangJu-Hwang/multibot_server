#pragma once

#include <vector>
#include <list>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"
#include "multibot_ros2_interface/msg/robot_config.hpp"
#include "multibot_ros2_interface/srv/robot_configs.hpp"

#include "server/Instance_Manager.hpp"

using namespace Instance;

namespace Server
{
    using namespace std::chrono_literals;
    
    class MultibotServer : public rclcpp::Node
    {
    private:
        struct Robot
        {
            AgentInstance::Agent robotInfo_;
            geometry_msgs::msg::Pose2D pose_;

            //Todo: Generalize: below localTimer is just for one linear movement.
            std::chrono::milliseconds localTimer_ = 0ms;
        };

    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfig       = multibot_ros2_interface::msg::RobotConfig;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;

    public:
        void loadInstances();

    private:
        void update_callback();
        void update_robotStates(RobotStateArray &_robot_states);
        void send_robotConfigs(const std::shared_ptr<RobotConfigs::Request>  _request,
                               std::shared_ptr<RobotConfigs::Response>       _response);

        void loadMap();
        void loadTasks();

        // Todo: Refactorying(Split, Generalize)
        void updateRobotPose(const std::string& _robotName);
        double displacementComputer(const double start_, const double goal_, const double t_,
                                  const double max_velocity_, const double max_acceleration_);

    private:
        rclcpp::Publisher<RobotStateArray>::SharedPtr robot_states_pub_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<RobotConfigs>::SharedPtr registration_server_;
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr mapLoading_;

        rclcpp::Time nodeStartTime_;

        std::unordered_map<std::string, Robot> robotList_;
        Instance_Manager instance_manager_;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server