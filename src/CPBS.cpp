#include "multibot_server/CPBS.hpp"

using namespace High_Level_Engine;

void CPBS::Solver::solve()
{
    std::cout << "CPBS::Solver::solve()" << std::endl;
    planner_->search("Agent1");
}

CPBS::Solver::Solver(std::shared_ptr<Instance::Instance_Manager> _instance_manager)
{
    planner_ = std::make_shared<AA_SIPP::Planner>(_instance_manager);

    std::cout << "CPBS::Solver Constructor" << std::endl;
}