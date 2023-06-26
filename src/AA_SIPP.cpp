#include "multibot_server/AA_SIPP.hpp"

using namespace Low_Level_Engine;

std::pair<Path::SinglePath, bool> AA_SIPP::Planner::search(
    const std::string &_agentName,
    const double _timeLimit,
    const std::vector<std::string> &_higher_agents,
    const Path::PathSet &_pathSet)
{
    std::cout << "AA_SIPP::Planner::search" << std::endl;

    std::cout << agents_[_agentName] << std::endl;

    return find_partial_path(
        _agentName,
        std::list<Time::TimeInterval>(),
        std::list<Time::TimeInterval>(),
        0);
}

std::pair<Path::SinglePath, bool> AA_SIPP::Planner::find_partial_path(
    const std::string &_agentName,
    const std::list<Time::TimeInterval> &_starts,
    const std::list<Time::TimeInterval> &_goals,
    const double _timeLimit)
{
    return std::make_pair(Path::SinglePath(), false);
}

AA_SIPP::Planner::Planner(std::shared_ptr<Instance_Manager> _instance_manager)
{
    _instance_manager->attach(*this);

    _instance_manager->attach(*(motion_manager_));
}