#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include "multibot_util/Instance.hpp"
#include "multibot_util/Panel_Util.hpp"
#include "multibot_util/Interface/Observer_Interface.hpp"

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/msg/local_path.hpp"
#include "multibot_ros2_interface/srv/path.hpp"

using namespace MAPF_Util;

namespace Instance
{
    typedef std::pair<std::unordered_map<std::string, AgentInstance::Agent>,
                      MapInstance::BinaryOccupancyMap>
        InstanceMsg;

    namespace AgentInstance
    {
        struct Robot
        {
            using State = multibot_ros2_interface::msg::RobotState;
            using Path = multibot_ros2_interface::srv::Path;
            using LocalPath = multibot_ros2_interface::msg::LocalPath;

            AgentInstance::Agent robotInfo_;
            int32_t id_;
            PanelUtil::Mode mode_;

            rclcpp::Time last_update_time_;
            rclcpp::Time prior_update_time_;

            rclcpp::Subscription<State>::SharedPtr state_sub_;
            rclcpp::Client<Path>::SharedPtr send_traj_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd_;
            rclcpp::Client<PanelUtil::ModeSelection>::SharedPtr modeFromServer_;
            rclcpp::Service<PanelUtil::ModeSelection>::SharedPtr modeFromRobot_;
        }; // struct Robot
    }      // namespace AgentInstance

    class Instance_Manager : public Observer::SubjectInterface<InstanceMsg>
    {
    private:
        using Path = multibot_ros2_interface::srv::Path;
        using LocalTraj = multibot_ros2_interface::msg::LocalPath;

    private:
        void robotState_callback(const AgentInstance::Robot::State::SharedPtr _state_msg);
        void loadMap();

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr mapLoader_;

    public:
        void insertRobot(const AgentInstance::Robot &_robot);
        void deleteRobot(const std::string _robotName);

        const AgentInstance::Robot &getRobot(const std::string _robotName) const;

        void fixStartPoses();
        void sendTrajectories(MAPF_Util::Traj::TrajSet &_trajSet);

        void setGoal(const std::string _robotName, const geometry_msgs::msg::Pose2D _goal);
        void setMode(const std::string _robotName, const PanelUtil::Mode _mode);
        void request_modeChange(const std::string _robotName, const PanelUtil::Mode _mode);
        void request_kill(const std::string _robotName);
        void remote_control(const std::string _robotName, const geometry_msgs::msg::Twist &_remote_cmd_vel);

    public:
        void attach(Observer::ObserverInterface<InstanceMsg> &_observer) override;
        void detach(Observer::ObserverInterface<InstanceMsg> &_observer) override;
        void notify() override;

    private:
        std::unordered_map<std::string, AgentInstance::Agent> agents_;
        std::unordered_map<std::string, AgentInstance::Robot> robots_;
        MapInstance::BinaryOccupancyMap map_;

        std::list<Observer::ObserverInterface<InstanceMsg> *> list_observer_;

    public:
        Instance_Manager(std::shared_ptr<rclcpp::Node> _nh);
        ~Instance_Manager() {}
    }; // class Instance_Manager

} // namespace Instance