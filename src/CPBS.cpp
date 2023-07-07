#include "multibot_server/CPBS.hpp"

using namespace High_Level_Engine;

void CPBS::Solver::solve()
{
    std::cout << "CPBS::Solver::solve()" << std::endl;

    // auto searchSpace = restrict_searchSpace();
    planner_->search("Agent1");
}

std::pair<Position::Coordinates, Position::Coordinates> CPBS::Solver::restrict_searchSpace()
{
    Position::Coordinates lower_left(
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    
    Position::Coordinates upper_right(
        -1 * std::numeric_limits<double>::infinity(), -1 * std::numeric_limits<double>::infinity());

    for (const auto &agent : agents_)
    {
        lower_left.x_ = std::min(agent.second.start_.component_.x, lower_left.x_);
        lower_left.y_ = std::min(agent.second.start_.component_.y, lower_left.y_);

        lower_left.x_ = std::min(agent.second.goal_.component_.x, lower_left.x_);
        lower_left.y_ = std::min(agent.second.goal_.component_.y, lower_left.y_);

        upper_right.x_ = std::max(agent.second.start_.component_.x, upper_right.x_);
        upper_right.y_ = std::max(agent.second.start_.component_.y, upper_right.y_);

        upper_right.x_ = std::max(agent.second.goal_.component_.x, upper_right.x_);
        upper_right.y_ = std::max(agent.second.goal_.component_.y, upper_right.y_);
    }

    return {lower_left, upper_right};
}

CPBS::Solver::Solver(std::shared_ptr<Instance::Instance_Manager> _instance_manager)
{
    _instance_manager->attach(*(this));

    planner_ = std::make_shared<AA_SIPP::Planner>(_instance_manager);

    std::cout << "CPBS::Solver Constructor" << std::endl;
}