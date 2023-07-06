#include "multibot_server/AA_SIPP.hpp"

using namespace Low_Level_Engine;

std::pair<Path::SinglePath, bool> AA_SIPP::Planner::search(
    const std::string &_agentName,
    const std::pair<Position::Coordinates, Position::Coordinates> &_searchSpace,
    const double _timeLimit,
    const std::vector<std::string> &_higher_agents,
    const Path::PathSet &_pathSet)
{
    std::cout << "AA_SIPP::Planner::search" << std::endl;

    map_utility_->restrictArea(_searchSpace);
    reservation_table_->init(agents_[_agentName].size_);
    higher_paths_.clear();

    for (const auto &singlePath : _pathSet)
    {
        if (std::find(_higher_agents.begin(), _higher_agents.end(), singlePath.second.agentName_) == _higher_agents.end())
            continue;

        reservation_table_->applySinglePath(_agentName, singlePath.second);
        higher_paths_.push_back(singlePath.second);
    }

    std::list<Time::TimeInterval> starts = {reservation_table_->getSafeIntervals(
        agents_[_agentName].start_).front()};
    std::list<Time::TimeInterval> goals = reservation_table_->getSafeIntervals(
        agents_[_agentName].goal_);
    
    auto partialPath = find_partial_path(
        _agentName, starts, goals, _timeLimit);

    return std::make_pair(Path::SinglePath(), false);
}

std::pair<Path::SinglePath, bool> AA_SIPP::Planner::find_partial_path(
    const std::string &_agentName,
    const std::list<Time::TimeInterval> &_starts,
    const std::list<Time::TimeInterval> &_goals,
    const double _timeLimit)
{
    generateRoot(agents_[_agentName].start_, _starts);

    return std::make_pair(Path::SinglePath(), false);
}

void AA_SIPP::Planner::generateRoot(
    const Position::Pose &_startPose,
    const std::list<Time::TimeInterval> &_starts)
{
    open_.clear();
    close_.clear();

    for (const auto &safe_interval : _starts)
    {
        AA_SIPP::Node startNode;

        startNode.pose_ = _startPose;
        startNode.idx_  = map_utility_->convertPoseToIndex(_startPose);

        startNode.gVal_ = 0;
        startNode.hVal_ = 0;

        startNode.safe_interval_ = safe_interval;
        startNode.parent_ = nullptr;
        
        open_.insert(startNode);
    }
}

AA_SIPP::Planner::Planner(std::shared_ptr<Instance_Manager> &_instance_manager)
{
    _instance_manager->attach(*this);

    _instance_manager->attach(*(map_utility_));
    _instance_manager->attach(*(motion_manager_));
    _instance_manager->attach(*(conflict_checker_));
    _instance_manager->attach(*(reservation_table_));
}