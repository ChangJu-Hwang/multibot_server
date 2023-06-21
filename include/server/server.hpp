#pragma once

#include <vector>
#include <list>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/robot_state_array.hpp"
#include "multibot_ros2_interface/msg/robot_config.hpp"
#include "multibot_ros2_interface/srv/robot_configs.hpp"
#include "multibot_ros2_interface/srv/robot_info.hpp"
#include "multibot_ros2_interface/action/path.hpp"

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

            // Todo: Generalize: below localTimer is just for one linear movement.
            std::chrono::milliseconds localTimer_ = 0ms;
        };

    private:
        using RobotState        = multibot_ros2_interface::msg::RobotState;
        using RobotStateArray   = multibot_ros2_interface::msg::RobotStateArray;
        using RobotConfig       = multibot_ros2_interface::msg::RobotConfig;
        using RobotConfigs      = multibot_ros2_interface::srv::RobotConfigs;
        using RobotInfo         = multibot_ros2_interface::srv::RobotInfo;
        using LocalPath         = multibot_ros2_interface::msg::LocalPath;
        using Path              = multibot_ros2_interface::action::Path;
        using GoalHandlePath    = rclcpp_action::ClientGoalHandle<Path>;

    public:
        void loadInstances();
        void request_registrations();
        // void request_controls();

    private:
        void update_callback();
        void update_robotStates(RobotStateArray &_robot_states);
        void send_robotConfigs(const std::shared_ptr<RobotConfigs::Request> _request,
                               std::shared_ptr<RobotConfigs::Response> _response);
        
        void request_registration(const std::string &_robotName,
                                  std::shared_ptr<rclcpp::Client<Server::MultibotServer::RobotInfo>> _service);
        
        // void request_control(const std::string &_robotName,
        //                      rclcpp_action::Client<Path>::SharedPtr &_control_client);
        // void get_control_action_goal(std::shared_future<GoalHandlePath::SharedPtr> _future);
        // void get_control_action_feedback(
        //     GoalHandlePath::SharedPtr,
        //     const std::shared_ptr<const Path::Feedback> _feedback);
        // void get_control_action_result(const GoalHandlePath::WrappedResult &_result);

    private:
        rclcpp::Publisher<RobotStateArray>::SharedPtr robot_states_pub_;
        rclcpp::Service<RobotConfigs>::SharedPtr registration_server_;
        std::vector<std::pair<std::string, rclcpp::Client<RobotInfo>::SharedPtr>> registration_request_;
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr mapLoading_;
        std::vector<std::pair<std::string, rclcpp_action::Client<Path>::SharedPtr>> controller_action_clinets_;

    private:
        void loadMap();
        void loadTasks();

        // Todo: Refactorying(Split, Generalize)
        void updateRobotPose(const std::string &_robotName);
        double displacementComputer(const double start_, const double goal_, const double t_,
                                    const double max_velocity_, const double max_acceleration_);

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