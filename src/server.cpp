#include "multibot_server/server.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <QApplication>

using namespace Server;
using namespace Instance;

void MultibotServer::execServerPanel(int argc, char *argv[])
{
    QApplication app(argc, argv);

    serverPanel_ = std::make_shared<Panel>(nh_, instance_manager_);
    serverPanel_->attach(*this);
    serverPanel_->show();

    pannel_is_running_ = true;

    app.exec();
}

void MultibotServer::update(const PanelUtil::Msg &_msg)
{
    if (not(pannel_is_running_))
        return;

    switch (_msg)
    {
    case PanelUtil::Request::PLAN_REQUEST:
    {
        trajSet_.clear();
        instance_manager_->fixStartPoses();

        auto plans = solver_->solve();

        if (plans.second == true)
        {
            serverPanel_->setPlanState(PanelUtil::PlanState::SUCCESS);

            trajSet_ = plans.first;

            for (const auto singleTraj : trajSet_)
                std::cout << singleTraj.second << std::endl;
        }
        else
            serverPanel_->setPlanState(PanelUtil::PlanState::FAIL);

        break;
    }

    case PanelUtil::Request::START_REQUEST:
    {
        if (not(trajSet_.empty()))
            instance_manager_->sendTrajectories(trajSet_);

        break;
    }

    default:
        break;
    }
}

MultibotServer::MultibotServer()
    : Node("server")
{
    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    instance_manager_ = std::make_shared<Instance_Manager>(nh_);
    solver_ = std::make_shared<CPBS::Solver>(instance_manager_);

    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}