#include "multibot_server/CPBS.hpp"

using namespace High_Level_Engine;

void CPBS::Solver::solve()
{
    planner_->search();
}

CPBS::Solver::Solver()
{
    planner_ = std::make_shared<AA_SIPP::Planner>();
}