#pragma once

#include <vector>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "multibot_server/CPBS.hpp"
#include "multibot_server/Instance_Manager.hpp"
#include "multibot_server/server_panel.hpp"

using namespace High_Level_Engine;
using namespace Instance;

namespace Server
{
    using namespace std::chrono_literals;

    class MultibotServer : public rclcpp::Node, public Observer::ObserverInterface<PanelUtil::Msg>
    {
    public:
        void execServerPanel(int argc, char *argv[]);
                
    private:
        std::shared_ptr<rclcpp::Node> nh_;

    public:
        void update(const PanelUtil::Msg &_msg) override;

    private:
        Traj::TrajSet trajSet_;

        std::shared_ptr<CPBS::Solver> solver_;
        std::shared_ptr<Instance_Manager> instance_manager_;

        std::shared_ptr<Panel> serverPanel_;
        bool pannel_is_running_;

    public:
        MultibotServer();
        ~MultibotServer();
    };
} // namespace Server