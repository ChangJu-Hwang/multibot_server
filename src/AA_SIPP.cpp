#include "multibot_server/AA_SIPP.hpp"

using namespace Low_Level_Engine;

std::pair<Path::SinglePath, bool> AA_SIPP::Planner::search(
    const std::string &_agentName,
    const double _timeLimit,
    const std::vector<std::string> &_higher_agents,
    const Path::PathSet &_pathSet)
{
    std::cout << "AA_SIPP::Planner::search" << std::endl;

    Path::SinglePath LowerPath;
        LowerPath.agentName_ = "Agent1";
            Path::SinglePath::Node lowerStart;
                lowerStart.pose_.component_.x = 0;
                lowerStart.pose_.component_.y = 5;
                lowerStart.pose_.component_.theta = 0;
                lowerStart.departure_time_ = Time::TimePoint(3);
            Path::SinglePath::Node lowerGoal;
                lowerGoal.pose_.component_.x = 10;
                lowerGoal.pose_.component_.y = 5;
                lowerGoal.pose_.component_.theta = 0;
                lowerGoal.arrival_time_ = lowerStart.departure_time_ + motion_manager_->getMoveTime(
                    LowerPath.agentName_, lowerStart.pose_, lowerGoal.pose_);

        LowerPath.nodes_.push_back(std::make_pair(lowerStart, lowerGoal));
        LowerPath.cost_ = (lowerGoal.arrival_time_ - lowerGoal.departure_time_).count();

    Path::SinglePath HigherPath;
        HigherPath.agentName_ = "Agent2";
            Path::SinglePath::Node higherStart;
                higherStart.pose_.component_.x = 0;
                higherStart.pose_.component_.y = 10;
                higherStart.departure_time_ = Time::TimePoint(0);
            Path::SinglePath::Node higherGoal;
                higherGoal.pose_.component_.x = 10;
                higherGoal.pose_.component_.y = 0;
                
            higherStart.pose_.component_.theta = atan2(
                higherGoal.pose_.component_.y - higherStart.pose_.component_.y,
                higherGoal.pose_.component_.x - higherStart.pose_.component_.x);
            higherGoal.pose_.component_.theta = higherStart.pose_.component_.theta;
            higherGoal.arrival_time_ = higherStart.departure_time_ + motion_manager_->getMoveTime(
                HigherPath.agentName_, higherStart.pose_, higherGoal.pose_);

        HigherPath.nodes_.push_back(std::make_pair(higherStart, higherGoal));
        HigherPath.cost_ = (higherGoal.arrival_time_ - higherStart.departure_time_).count();

    auto delay = conflict_checker_->getDelayTime(HigherPath, LowerPath);
    std::cout << "Delay: "<<  delay << std::endl;

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

    _instance_manager->attach(*(map_utility_));
    _instance_manager->attach(*(motion_manager_));
    _instance_manager->attach(*(conflict_checker_));
}